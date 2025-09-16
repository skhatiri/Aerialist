from copy import deepcopy
import os
import os.path as path
import logging
import subprocess
from typing import List
from decouple import config
import asyncio
from . import file_helper
from .aerialist_test import AgentConfig, AerialistTest, AerialistTestResult
from .test_agent import TestAgent

logger = logging.getLogger(__name__)


# extension_hint: implement a subclass for the specific usecase, handling setting up the docker container, executing the tests, copying files in and out, and parsing the results
class DockerAgent(TestAgent):
    CMD = "python3 aerialist exec --test {test_file}"
    DOCKER_CMD = "docker exec -it {id} {cmd}"
    DOCKER_IMG = config("DOCKER_IMG", default="skhatiri/aerialist")
    COPY_DIR = config("LOGS_COPY_DIR", "results/logs/")
    DOCKER_TIMEOUT = config("DOCKER_TIMEOUT", default=1000, cast=int)
    SIMULATION_TIMEOUT = config("SIMULATION_TIMEOUT", cast=int, default=-1)
    DOCKER_DISPLAY = config("DOCKER_DISPLAY", cast=bool, default=False)
    USE_VOLUME = config("DOCKER_USE_VOLUME", cast=bool, default=False)
    VOLUME_PATH = config("DOCKER_VOLUME_PATH", default="/src/aerialist/results/")

    def __init__(self, config: AerialistTest) -> None:
        self.config = config
        self.results: List[AerialistTestResult] = []

        # for the moment, containers needs to be created before import_config if using docker_cp, and after it if using volumes
        if self.USE_VOLUME:
            self.container_config = self.import_config()
        self.create_container()
        if not self.USE_VOLUME:
            self.container_config = self.import_config()

        cmd = self.CMD.format(test_file=self.container_test_yaml)
        if self.DOCKER_TIMEOUT is not None:
            cmd = f"timeout {self.DOCKER_TIMEOUT} " + cmd
        self.docker_cmd = self.DOCKER_CMD.format(id=self.container_id, cmd=cmd)
        logger.debug(f"docker command:{self.docker_cmd}")

    def create_container(self):
        params = ""
        if self.config.agent.id is not None:
            params += f"--name aerialist_{self.config.agent.id} "
        if self.SIMULATION_TIMEOUT > 0:
            params += f"-e SIMULATION_TIMEOUT={self.SIMULATION_TIMEOUT} "
        if self.DOCKER_DISPLAY and not config.simulation.headless:
            params += '-e DISPLAY -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" '
        if self.USE_VOLUME:
            params += (
                f'-v "{os.path.abspath(self.config.agent.path)}:{self.VOLUME_PATH}:rw" '
            )
        cmd = f"docker run --rm {params}-td {self.DOCKER_IMG}"
        create_process = subprocess.run(cmd, shell=True, capture_output=True)

        if create_process.returncode == 0:
            self.container_id = create_process.stdout.decode("ascii").strip()
            logger.info(f"new container:{self.container_id[:12]}")
            return True
        else:
            if create_process.stdout:
                logger.info(create_process.stdout.decode("ascii"))
            if create_process.stderr:
                logger.error(create_process.stderr.decode("ascii"))
            return False

    def run(self):
        exec_process = subprocess.run(self.docker_cmd, shell=True, capture_output=True)
        self.process_output(
            exec_process.returncode,
            exec_process.stdout.decode("utf-8", errors="strict"),
            exec_process.stdout.decode("utf-8", errors="strict"),
            True,
        )
        logger.info("test execution finished")
        return self.results

    async def run_async(self):
        exec_process = await asyncio.create_subprocess_shell(
            self.docker_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await exec_process.communicate()
        self.process_output(
            exec_process.returncode,
            stdout.decode("utf-8", errors="strict"),
            stderr.decode("utf-8", errors="strict"),
        )
        logger.info("test execution finished")

    def process_output(self, returncode, stdout, stderr, print_logs=False):
        try:
            container_log = None

            # First try the standard "LOG:" pattern
            if "LOG:" in stdout:
                container_log = stdout.split("LOG:")[1].split()[0]
            # Fallback: look for "logging started:"
            elif "logging started:" in stdout:
                container_log = stdout.split("logging started:")[1].split()[0]
            else:
                logger.error(
                    "No log path found in stdout. Patterns checked: 'LOG:', 'logging started:'"
                )
                if print_logs:
                    logger.info(stdout)
                return  # Exit early since no logs were found

            if self.USE_VOLUME:
                volume_folder = self.config.agent.path
                log_add = container_log.replace(self.VOLUME_PATH, volume_folder)
            else:
                if self.config.agent.id is not None:
                    log_add = f"{self.config.agent.path}{self.config.agent.id}_{self.container_id[:12]}.ulg"
                else:
                    log_add = f"{self.config.agent.path}{self.container_id[:12]}.ulg"
                self.docker_cp_export(container_log, log_add)

            # extension_hint: infer the test status from the log
            # extension_hint: use usecase specific trajectory class as the variable type
            result = AerialistTestResult(log_add, status=AerialistTestResult.Status.UNKNOWN)
            self.results.append(result)

            if print_logs:
                logger.info("************************************")
                logger.info("Logs from the Docker container:")
                if stdout:
                    logger.info(stdout)
                if stderr:
                    logger.error(stderr)
                logger.info("************************************")

        except Exception as e:
            logger.error(f"Error processing output: {e}")
            if stdout:
                logger.info(stdout)
            if stderr:
                logger.error(stderr)

        if hasattr(self, "container_id"):
            subprocess.run(f"docker kill {self.container_id}", shell=True)
        else:
            logger.info("No container_id found; skipping kill.")

    def import_config(self):
        if self.config.agent.id is None:
            self.config.agent.id = file_helper.time_filename()
        else:
            self.config.agent.id += "-" + file_helper.time_filename()
        if self.config.agent.path is None:
            self.config.agent.path = self.COPY_DIR
        if not self.config.agent.path.endswith("/"):
            self.config.agent.path += "/"
        self.config.agent.path += self.config.agent.id + "/"

        if self.USE_VOLUME:
            container_config = self.copy_files_volume()
        else:
            container_config = self.copy_files_cp()
        return container_config

    def copy_files_cp(self):
        volume_folder = self.config.agent.path
        container_config = deepcopy(self.config)
        os.makedirs(volume_folder, exist_ok=True)
        # Drone Config
        if self.config.robot is not None:
            if self.config.robot.mission_file is not None:
                self.docker_cp_import(self.config.robot.mission_file, self.VOLUME_PATH)
                container_config.robot.mission_file = (
                    f"{self.VOLUME_PATH}{path.basename(self.config.robot.mission_file)}"
                )

            if (
                self.config.robot.params is None
                and self.config.robot.params_file is not None
            ):
                self.docker_cp_import(self.config.robot.params_file, self.VOLUME_PATH)
                container_config.robot.params_file = (
                    f"{self.VOLUME_PATH}{path.basename(self.config.robot.params_file)}"
                )

        # Test Config
        if self.config.mission is not None:
            self.config.mission.save_commands_list_if_needed(self.config.agent.path)
            if (
                self.config.mission.commands_file is not None
                and self.config.mission.commands is not None
                and len(self.config.mission.commands)
                > self.config.mission.MAX_INLINE_COMMANDS
            ):
                self.docker_cp_import(
                    self.config.mission.commands_file, self.VOLUME_PATH
                )
                container_config.mission.commands_file = f"{self.VOLUME_PATH}{path.basename(self.config.mission.commands_file)}"

        # Assertion Config
        container_config.assertion = None
        # if self.config.assertion is not None:
        #     if self.config.assertion.log_file is not None:
        #         self.import_file(self.config.assertion.log_file, self.VOLUME_PATH)
        #         container_config.assertion.log_file = (
        #             f"{self.VOLUME_PATH}{path.basename(self.config.assertion.log_file)}"
        #         )

        if container_config.agent is not None:
            container_config.agent.engine = AgentConfig.LOCAL
            container_config.agent.count = 1
            container_config.agent.path = None

        container_test_yaml = container_config.to_yaml(
            f"{self.config.agent.path}{self.config.agent.id}.yaml"
        )
        self.docker_cp_import(container_test_yaml, self.VOLUME_PATH)
        self.container_test_yaml = container_test_yaml.replace(
            self.config.agent.path, self.VOLUME_PATH
        )
        logger.info(f"files copied")
        return container_config

    def copy_files_volume(self):
        volume_folder = self.config.agent.path

        container_config = deepcopy(self.config)
        os.makedirs(volume_folder, exist_ok=True)

        # Drone Config
        if self.config.robot is not None:
            if self.config.robot.mission_file is not None:
                container_config.robot.mission_file = file_helper.copy(
                    self.config.robot.mission_file, volume_folder
                ).replace(volume_folder, self.VOLUME_PATH)
            if (
                self.config.robot.params is None
                and self.config.robot.params_file is not None
            ):
                container_config.robot.params_file = file_helper.copy(
                    self.config.robot.params_file, volume_folder
                ).replace(volume_folder, self.VOLUME_PATH)

        # Test Config
        if self.config.mission is not None:
            self.config.mission.save_commands_list_if_needed(self.config.agent.path)
            if (
                self.config.mission.commands_file is not None
                and self.config.mission.commands is not None
                and len(self.config.mission.commands)
                > self.config.mission.MAX_INLINE_COMMANDS
            ):
                container_config.mission.commands_file = file_helper.copy(
                    self.config.mission.commands_file, volume_folder
                ).replace(volume_folder, self.VOLUME_PATH)

        # Assertion Config
        container_config.assertion = None
        # if self.config.assertion is not None:
        #     if self.config.assertion.log_file is not None:
        #         k8s_config.assertion.log_file = file_helper.upload(
        #             self.config.assertion.log_file, cloud_folder
        #         )

        if container_config.agent is not None:
            container_config.agent.engine = AgentConfig.LOCAL
            container_config.agent.count = 1
            container_config.agent.path = None

        self.container_test_yaml = container_config.to_yaml(
            f"{volume_folder}{self.config.agent.id}.yaml"
        ).replace(volume_folder, self.VOLUME_PATH)
        logger.info(f"files copied")
        return container_config

    def docker_cp_import(self, src, dest):
        logger.debug(f"docker_cp_import: src={src}, dest={dest}")
        subprocess.run(
            f"docker cp '{src}' {self.container_id}:'{dest}'",
            shell=True,
        )

    def docker_cp_export(self, src, dest):
        logger.debug(f"docker_cp_import: src={src}, dest={dest}")
        subprocess.run(
            f"docker cp {self.container_id}:'{src}' '{dest}'",
            shell=True,
        )
