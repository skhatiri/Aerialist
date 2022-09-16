import os.path as path
from datetime import datetime
from statistics import median
import logging
import subprocess
from typing import List
from decouple import config
import asyncio
from .drone_test import DroneTest
from .test_agent import TestAgent
from .command import Command
from .obstacle import Obstacle
from .trajectory import Trajectory

logger = logging.getLogger(__name__)


class DockerAgent(TestAgent):

    # CMD should be updated if the interface in run.py changes
    CMD = "timeout {timeout} ./run.py {optionals}--drone {drone} --env {sim} --speed {speed} --headless experiment replay"
    DOCKER_CMD = "docker exec -it {id} "
    DOCKER_IMG = config("DOCKER_IMG", default="drone-experiments")
    COPY_DIR = config("LOGS_COPY_DIR", "results/logs/")

    def __init__(
        self,
        config: DroneTest
        # drone: str = config("DRONE", default="sim"),
        # env: str = config("SIMULATOR", default="gazebo"),
        # headless: bool = True,
        # speed: float = config("SPEED", default=1, cast=int),
        # log: str = None,
        # is_jmavsim=False,
        # params_csv=None,
        # obstacles: List[float] = None,
        # mission_file: str = None,
    ) -> None:
        super().__init__(config)
        # self.simulator = env
        # self.drone = drone
        # self.speed = speed
        # self.original_log = log
        # self.params_csv = params_csv
        # self.mission = mission_file
        # self.obstacles = obstacles
        # self.original_trajectory = None
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

        self.load_commands()
        if self.original_log is not None and self.original_log.endswith(".ulg"):
            self.original_trajectory = Trajectory.extract_from_log(
                self.original_log, is_jmavsim=is_jmavsim
            )
        self.docker_cmd = self.DOCKER_CMD.format(
            id=self.container_id
        ) + self.format_command(
            self.drone,
            self.simulator,
            self.speed,
            self.docker_log,
            self.docker_params,
            self.obstacles,
            self.docker_mission,
        )

    @classmethod
    def format_command(
        cls,
        drone,
        sim,
        speed,
        log,
        params_csv,
        obstacles,
        mission,
        cloud=False,
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
        if cloud:
            optionals += "--cloud "
            if output_path:
                optionals += f"--output '{output_path}' "

        cmd = cls.CMD.format(
            optionals=optionals,
            drone=drone,
            sim=sim,
            speed=speed,
            timeout=config("DOCKER_TIMEOUT", default=180, cast=int),
        )
        return cmd

    def replay(self):
        logger.debug(self.docker_cmd)
        replay_cmd = subprocess.run(self.docker_cmd, shell=True, capture_output=True)
        self.process_output(
            replay_cmd.returncode,
            replay_cmd.stdout.decode("ascii"),
            replay_cmd.stderr.decode("ascii"),
            True,
        )

    async def replay_async(self):

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
            self.docker_log = stdout[stdout.find("LOG:") + 4 :].split()[0]
            self.export_file(self.docker_log, self.log)
            self.trajectory = Trajectory.extract_from_log(self.log)
            if hasattr(self, "original_trajectory"):
                self.trajectory.plot(
                    self.original_trajectory,
                    obstacles=self.obstacles,
                )

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

    def load_commands(self):
        self.docker_log = None
        self.docker_mission = None
        self.docker_params = None
        if self.config.assertion.log is not None:
            self.import_file(self.config.assertion.log, "/io/")
            self.docker_log = f"/io/{path.basename(self.config.assertion.log)}"
            self.log = f"{self.COPY_DIR}{path.basename(self.original_log)[:-4]}_{self.container_id[:12]}.ulg"

        if self.mission is not None:
            self.import_file(self.mission, "/io/")
            self.docker_mission = f"/io/{path.basename(self.mission)}"
            self.log = f"{self.COPY_DIR}{path.basename(self.mission)[:-5]}_{self.container_id[:12]}.ulg"

        if self.params_csv is not None:
            self.import_file(self.params_csv, "/io/")
            self.docker_params = f"/io/{path.basename(self.params_csv)}"

    def run(self, commands: List[Command]):
        self.original_log = (
            f'{self.COPY_DIR}{datetime.now().strftime("%m-%d %H:%M:%S")}.csv'
        )
        Command.save_csv(commands, self.original_log)
        self.load_commands()
        self.replay()

    def manual(self):
        pass

    @classmethod
    def replay_parallel(
        cls,
        runs: int = 1,
        drone: str = config("DRONE", default="sim"),
        env: str = config("SIMULATOR", default="gazebo"),
        headless: bool = True,
        speed: float = config("SPEED", default=1, cast=int),
        log: str = None,
        is_jmavsim=False,
        params_csv=None,
        obstacles: List[float] = None,
        mission_file: str = None,
    ):
        experiments: List[DockerExperiment] = []
        for i in range(runs):
            experiments.append(
                DockerExperiment(
                    drone,
                    env,
                    headless,
                    speed,
                    log,
                    is_jmavsim,
                    params_csv,
                    obstacles,
                    mission_file,
                )
            )

        loop = asyncio.get_event_loop()
        loop.run_until_complete(
            asyncio.gather(*[e.replay_async() for e in experiments])
        )
        return [e for e in experiments if hasattr(e, "trajectory")]

    @classmethod
    def replay_multiple(
        cls,
        runs: int = 1,
        drone: str = config("DRONE", default="sim"),
        env: str = config("SIMULATOR", default="gazebo"),
        headless: bool = True,
        speed: float = config("SPEED", default=1, cast=int),
        log: str = None,
        is_jmavsim=False,
        params_csv=None,
        obstacles: List[float] = None,
        mission_file: str = None,
    ):

        experiments = cls.replay_parallel(
            runs,
            drone,
            env,
            headless,
            speed,
            log,
            is_jmavsim,
            params_csv,
            obstacles,
            mission_file,
        )
        logger.info(f"{len(experiments)} evalations completed")
        obst = None
        if obstacles != None:
            obst = Obstacle.from_coordinates_multiple(obstacles)

        Trajectory.plot_multiple(
            [e.trajectory for e in experiments],
            experiments[0].original_trajectory,
            distance=None
            if experiments[0].original_log is None
            else median(
                [
                    e.trajectory.distance(experiments[0].original_trajectory)
                    for e in experiments
                ]
            ),
            obstacles=None
            if obstacles is None
            else Obstacle.from_coordinates_multiple(obstacles),
        )

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
