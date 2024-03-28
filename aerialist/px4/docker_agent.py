from copy import deepcopy
import os.path as path
import logging
import subprocess
from decouple import config
import asyncio
from . import file_helper
from .command import Command
from .drone_test import AgentConfig, DroneTest, DroneTestResult
from .test_agent import TestAgent

logger = logging.getLogger(__name__)


class DockerAgent(TestAgent):
    CMD = "python3 aerialist {params}"
    DOCKER_CMD = "docker exec -it {id} "
    DOCKER_IMG = config("DOCKER_IMG", default="skhatiri/aerialist")
    COPY_DIR = config("LOGS_COPY_DIR", "results/logs/")
    DOCKER_TIMEOUT = config("DOCKER_TIMEOUT", default=1000, cast=int)
    SIMULATION_TIMEOUT = config("SIMULATION_TIMEOUT", cast=int, default=-1)

    def __init__(self, config: DroneTest) -> None:
        super().__init__(config)
        envs = ""
        if self.SIMULATION_TIMEOUT > 0:
            envs = f"-e SIMULATION_TIMEOUT={self.SIMULATION_TIMEOUT} "
        cmd = f"docker run --rm {envs}-td {self.DOCKER_IMG}"
        create_cmd = subprocess.run(cmd, shell=True, capture_output=True)

        if create_cmd.returncode == 0:
            self.container_id = create_cmd.stdout.decode("ascii").strip()
            logger.info(f"new container:{self.container_id[:12]}")
        else:
            if create_cmd.stdout:
                logger.info(create_cmd.stdout.decode("ascii"))
            if create_cmd.stderr:
                logger.error(create_cmd.stderr.decode("ascii"))

        self.docker_config = self.import_config()
        self.docker_cmd = self.DOCKER_CMD.format(
            id=self.container_id
        ) + self.format_command(self.docker_config)

    def format_command(
        self,
        config: DroneTest,
    ):
        params = DroneTest(
            config.drone, config.simulation, config.test, None, config.agent
        ).cmd_params()
        cmd = self.CMD.format(params=params)
        if self.DOCKER_TIMEOUT is not None:
            cmd = f"timeout {self.DOCKER_TIMEOUT} " + cmd
        return cmd

    def run(self):
        logger.debug(self.docker_cmd)
        replay_cmd = subprocess.run(self.docker_cmd, shell=True, capture_output=True)
        self.process_output(
            replay_cmd.returncode,
            replay_cmd.stdout.decode("ascii"),
            replay_cmd.stderr.decode("ascii"),
            True,
        )
        return self.results

    async def run_async(self):
        logger.debug(self.docker_cmd)
        replay_cmd = await asyncio.create_subprocess_shell(
            self.docker_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await replay_cmd.communicate()
        self.process_output(
            replay_cmd.returncode, stdout.decode("ascii"), stderr.decode("ascii")
        )
        logger.info("replay finished")

    def process_output(self, returncode, stdout, stderr, print_logs=False):
        try:
            docker_log = stdout[stdout.find("LOG:") + 4 :].split()[0]
            if self.config.test.commands_file is not None:
                log_add = f"{self.COPY_DIR}{path.basename(self.config.test.commands_file)[:-4]}_{self.container_id[:12]}.ulg"
            elif self.config.drone.mission_file is not None:
                log_add = f"{self.COPY_DIR}{path.basename(self.config.drone.mission_file)[:-5]}_{self.container_id[:12]}.ulg"
            else:
                log_add = f"{self.COPY_DIR}{self.container_id[:12]}.ulg"

            self.export_file(docker_log, log_add)
            self.results.append(DroneTestResult(log_add))
            if print_logs:
                logger.info("************************************")
                logger.info("Logs from the Docker container:")
                if stdout:
                    logger.info(stdout)
                if stderr:
                    logger.error(stderr)
                logger.info("************************************")
        except:
            if stdout:
                logger.info(stdout)
            if stderr:
                logger.error(stderr)

        subprocess.run(f"docker kill {self.container_id}", shell=True)

    def import_config(self):
        docker_config = deepcopy(self.config)

        # Drone Config
        if self.config.drone is not None:
            if self.config.drone.mission_file is not None:
                self.import_file(self.config.drone.mission_file, "/io/")
                docker_config.drone.mission_file = (
                    f"/io/{path.basename(self.config.drone.mission_file)}"
                )

            if self.config.drone.params_file is not None:
                self.import_file(self.config.drone.params_file, "/io/")
                docker_config.drone.params_file = (
                    f"/io/{path.basename(self.config.drone.params_file)}"
                )

        # Test Config
        if self.config.test is not None:
            if (
                self.config.test.commands is not None
                and self.config.test.commands_file is None
            ):
                self.config.test.commands_file = (
                    f"/tmp/{file_helper.time_filename()}.csv"
                )
                Command.save_csv(
                    self.config.test.commands, self.config.test.commands_file
                )
            if self.config.test.commands_file is not None:
                self.import_file(self.config.test.commands_file, "/io/")
                docker_config.test.commands_file = (
                    f"/io/{path.basename(self.config.test.commands_file)}"
                )

        # Assertion Config
        docker_config.assertion = None
        # if self.config.assertion is not None:
        #     if self.config.assertion.log_file is not None:
        #         self.import_file(self.config.assertion.log_file, "/io/")
        #         docker_config.assertion.log_file = (
        #             f"/io/{path.basename(self.config.assertion.log_file)}"
        #         )

        if docker_config.agent is not None:
            docker_config.agent.engine = AgentConfig.LOCAL
            docker_config.agent.count = 1

        return docker_config

    def import_file(self, src, dest):
        subprocess.run(
            f"docker cp '{src}' {self.container_id}:'{dest}'",
            shell=True,
        )

    def export_file(self, src, dest):
        subprocess.run(
            f"docker cp {self.container_id}:'{src}' '{dest}'",
            shell=True,
        )
