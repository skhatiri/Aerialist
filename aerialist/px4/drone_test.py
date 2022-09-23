from __future__ import annotations
from statistics import median
from typing import List
from .command import Command
from .obstacle import Obstacle
from .trajectory import Trajectory
from . import file_helper


class DroneTest:
    def __init__(
        self,
        drone: DroneConfig = None,
        simulation: SimulationConfig = None,
        test: TestConfig = None,
        assertion: AssertionConfig = None,
        runner: RunnerConfig = None,
    ) -> None:
        self.drone = drone
        self.simulation = simulation
        self.test = test
        self.assertion = assertion
        self.runner = runner


class DroneConfig:
    CF_PORT = 14550
    SITL_PORT = 14540
    ROS_PORT = 14541

    def __init__(
        self,
        port: int | str = SITL_PORT,
        params: dict | str = None,
        mission=None,
    ) -> None:

        if isinstance(port, int):
            self.port = port
        else:
            if port.isdigit():
                self.port = int(port)
            if port == "sitl" or port == "sim":
                self.port = self.SITL_PORT
            if port == "ros" or port == "avoidance":
                self.port = self.ROS_PORT
            if port == "cf" or port == "hw":
                self.port = self.CF_PORT

        if isinstance(params, str):
            # params file path
            self.params_file = params

            self.params = Command.extract_params_from_csv(
                file_helper.get_local_file(params)
            )

        else:
            self.params_file = None
            self.params = params

        if isinstance(mission, str):
            self.mission_file = file_helper.get_local_file(mission)
        else:
            self.mission_file = None
            self.mission = mission


class SimulationConfig:
    JMAVSIM = "jmavsim"
    GAZEBO = "gazebo"
    ROS = "ros"

    def __init__(
        self,
        simulator=GAZEBO,
        world: str = "default",
        speed=1,
        headless=True,
        obstacles=[],
    ) -> None:
        self.simulator = simulator
        self.world = world
        self.speed = speed
        self.headless = headless

        if (
            obstacles is not None
            and len(obstacles) > 0
            and not isinstance(obstacles[0], Obstacle)
        ):
            self.obstacles = Obstacle.from_coordinates_multiple(obstacles)
        else:
            self.obstacles = obstacles


class TestConfig:
    def __init__(
        self,
        commands: List | str = None,
        speed: float = 1,
    ) -> None:

        if isinstance(commands, str):
            self.commands_file = commands
            # commands file path
            self.commands = Command.extract(file_helper.get_local_file(commands))

        else:
            self.commands_file = None
            self.commands = commands

        self.speed = speed


class AssertionConfig:
    TRAJECTORY = "trajectory"

    def __init__(
        self,
        log_file: str = None,
        variable: str = TRAJECTORY,
        expectation=None,
    ) -> None:
        self.log_file = log_file
        self.expectation = expectation
        if expectation is None and isinstance(log_file, str):
            if variable == self.TRAJECTORY:
                self.expectation = Trajectory.extract(
                    file_helper.get_local_file(log_file)
                )


class RunnerConfig:
    def __init__(self, agent, count=1, path=None, id=None) -> None:
        self.agent = agent
        self.count = count
        self.path = path
        self.id = id


class DroneTestResult:
    def __init__(
        self,
        log_file: str = None,
        variable: str = AssertionConfig.TRAJECTORY,
        record=None,
    ) -> None:
        self.log_file = log_file
        if record is None:
            if variable == AssertionConfig.TRAJECTORY:
                self.record = Trajectory.extract(file_helper.get_local_file(log_file))
            else:
                self.record = None


def Plot(test: DroneTest, results: List[DroneTestResult]) -> None:
    if results is not None and len(results) >= 1:
        Trajectory.plot_multiple(
            [r.record for r in results],
            goal=test.assertion.expectation if test.assertion is not None else None,
            distance=None
            if test.assertion.expectation is None
            else median(
                [r.record.distance(test.assertion.expectation) for r in results]
            ),
            obstacles=test.simulation.obstacles,
        )
