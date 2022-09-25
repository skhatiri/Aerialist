from copy import deepcopy
import os.path as path
import logging
import subprocess
from decouple import config
import asyncio
from .drone_test import DroneTest, DroneTestResult
from .test_agent import TestAgent

logger = logging.getLogger(__name__)


class DockerAgent(TestAgent):

    # CMD should be updated if the interface in entry.py changes
    CMD = "timeout {timeout} python3 aerialist {optionals} --drone {drone} --simulator {sim} --speed {speed} --headless"
    DOCKER_CMD = "docker exec -it {id} "
    DOCKER_IMG = config("DOCKER_IMG", default="aerialist")
    COPY_DIR = config("LOGS_COPY_DIR", "results/logs/")

    def __init__(self, config: DroneTest) -> None:
        super().__init__(config)
        create_cmd = subprocess.run(
            f"docker run -td {self.DOCKER_IMG}", shell=True, capture_output=True
        )
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
        ) + self.format_command(
            self.docker_config.drone.port,
            self.docker_config.simulation.simulator,
            self.docker_config.simulation.speed,
            self.docker_config.assertion.log_file,
            self.docker_config.test.commands_file,
            self.docker_config.drone.params_file,
            self.docker_config.simulation.obstacles,
            self.docker_config.drone.mission_file,
        )

    def format_command(
        self,
        drone,
        sim,
        speed,
        log,
        commands,
        params_csv,
        obstacles,
        mission,
        output_path=None,
    ):
        optionals = ""
        if params_csv is not None:
            optionals += f"--params '{params_csv}' "
        if obstacles != None and len(obstacles) >= 1:
            obs = ""
            for o in obstacles[0:6]:
                obs += str(o) + " "
            optionals += f"--obst {obs}"
        if obstacles != None and len(obstacles) >= 7:
            obs2 = ""
            for o in obstacles[6:12]:
                obs2 += str(o) + " "
            optionals += f"--obst2 {obs2}"
        if mission is not None:
            optionals += f"--mission '{mission}' "
        if log is not None:
            optionals += f"--log '{log}' "
        if commands is not None:
            optionals += f"--commands '{commands}' "

        if output_path:
            optionals += f"--path '{output_path}' "

        cmd = self.CMD.format(
            optionals=optionals,
            drone=drone,
            sim=sim,
            speed=speed,
            timeout=config("DOCKER_TIMEOUT", default=180, cast=int),
        )
        return cmd

    def run(self, config: DroneTest):
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
                if stdout:
                    logger.debug(stdout)
                if stderr:
                    logger.warning(stderr)
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
            if self.config.test.commands_file is not None:
                self.import_file(self.config.test.commands_file, "/io/")
                docker_config.test.commands_file = (
                    f"/io/{path.basename(self.config.test.commands_file)}"
                )

        # Assertion Config
        if self.config.assertion is not None:
            if self.config.assertion.log_file is not None:
                self.import_file(self.config.assertion.log_file, "/io/")
                docker_config.assertion.log_file = (
                    f"/io/{path.basename(self.config.assertion.log_file)}"
                )

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

    # def run(self, commands: List[Command]):
    #     self.original_log = (
    #         f'{self.COPY_DIR}{datetime.now().strftime("%m-%d %H:%M:%S")}.csv'
    #     )
    #     Command.save_csv(commands, self.original_log)
    #     self.load_commands()
    #     self.replay()

    # @classmethod
    # def replay_parallel(
    #     cls,
    #     runs: int = 1,
    #     drone: str = config("DRONE", default="sim"),
    #     env: str = config("SIMULATOR", default="gazebo"),
    #     headless: bool = True,
    #     speed: float = config("SPEED", default=1, cast=int),
    #     log: str = None,
    #     is_jmavsim=False,
    #     params_csv=None,
    #     obstacles: List[float] = None,
    #     mission_file: str = None,
    # ):
    #     experiments: List[DockerExperiment] = []
    #     for i in range(runs):
    #         experiments.append(
    #             DockerExperiment(
    #                 drone,
    #                 env,
    #                 headless,
    #                 speed,
    #                 log,
    #                 is_jmavsim,
    #                 params_csv,
    #                 obstacles,
    #                 mission_file,
    #             )
    #         )

    #     loop = asyncio.get_event_loop()
    #     loop.run_until_complete(
    #         asyncio.gather(*[e.replay_async() for e in experiments])
    #     )
    #     return [e for e in experiments if hasattr(e, "trajectory")]

    # @classmethod
    # def replay_multiple(
    #     cls,
    #     runs: int = 1,
    #     drone: str = config("DRONE", default="sim"),
    #     env: str = config("SIMULATOR", default="gazebo"),
    #     headless: bool = True,
    #     speed: float = config("SPEED", default=1, cast=int),
    #     log: str = None,
    #     is_jmavsim=False,
    #     params_csv=None,
    #     obstacles: List[float] = None,
    #     mission_file: str = None,
    # ):

    #     experiments = cls.replay_parallel(
    #         runs,
    #         drone,
    #         env,
    #         headless,
    #         speed,
    #         log,
    #         is_jmavsim,
    #         params_csv,
    #         obstacles,
    #         mission_file,
    #     )
    #     logger.info(f"{len(experiments)} evalations completed")
    #     obst = None
    #     if obstacles != None:
    #         obst = Obstacle.from_coordinates_multiple(obstacles)

    #     Trajectory.plot_multiple(
    #         [e.trajectory for e in experiments],
    #         experiments[0].original_trajectory,
    #         distance=None
    #         if experiments[0].original_log is None
    #         else median(
    #             [
    #                 e.trajectory.distance(experiments[0].original_trajectory)
    #                 for e in experiments
    #             ]
    #         ),
    #         obstacles=None
    #         if obstacles is None
    #         else Obstacle.from_coordinates_multiple(obstacles),
    #     )
