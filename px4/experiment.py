from typing import List
from decouple import config
from px4.drone import Drone, MavAddress
from px4.command import Command
from px4.obstacle import Obstacle
from px4.position import Position
from px4.schedular import Schedular
from px4.simulator import Simulator, Environment
from px4.trajectory import Trajectory


class Experiment(object):
    def __init__(
        self,
        drone: str = config("DRONE", default="sim"),
        env: str = config("SIMULATOR", default="gazebo"),
        headless: bool = config("HEADLESS", default=False, cast=bool),
        speed: float = config("SPEED", default=1, cast=int),
        log: str = None,
        is_jmavsim=False,
        params_csv=None,
        obstacles: List[float] = None,
        mission_file: str = None,
    ) -> None:

        super().__init__()

        self.original_log = log
        self.speed = speed
        self.obstacles = None
        self.mission = mission_file
        if obstacles != None:
            self.obstacles = Obstacle.from_coordinates_multiple(obstacles)

        self.simulator = Simulator(
            Environment[env.upper()], headless, speed, self.obstacles
        )
        if drone != "none":
            params = None
            if params_csv != None:
                params = Command.extract_params_from_csv(params_csv)

            self.drone = Drone(MavAddress[drone.upper()], params, self.mission)
            if self.original_log is not None:
                if self.mission is None:
                    self.load_commands()

                if self.original_log.endswith(".ulg"):
                    self.original_trajectory = Trajectory.extract_from_log(
                        self.original_log, is_jmavsim=is_jmavsim
                    )

    def load_commands(self):
        if self.original_log.endswith(".ulg"):
            self.commands = Command.extract_from_log(self.original_log)
            Command.save_csv(
                self.commands, self.original_log.replace(".ulg", "_commands.csv")
            )
        elif self.original_log.endswith(".csv"):
            self.commands = Command.extract_from_csv(self.original_log)
        self.scheduler = Schedular(self.drone, self.commands, self.speed)

    def replay(self):
        try:
            if self.mission is None:
                self.scheduler.run()
            else:
                self.drone.start_mission()
            self.log = self.simulator.get_log()
            self.trajectory = Trajectory.extract_from_log(self.log)
            if hasattr(self, "original_trajectory"):
                self.trajectory.plot(
                    self.original_trajectory,
                    obstacles=self.obstacles,
                )
        except Exception as e:
            self.simulator.kill()
            raise (e)

    def set_commands(self, commands: List[Command]):
        if commands is not None:
            self.commands = commands
            self.scheduler = Schedular(self.drone, self.commands, self.speed)

    def manual(self):
        self.drone.manual()
        self.log = Simulator.get_log()
