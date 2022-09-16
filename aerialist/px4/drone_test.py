from __future__ import annotations
from typing import List
from .command import Command
from .obstacle import Obstacle
from .trajectory import Trajectory


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
            if port == "sitl" or port == "sim":
                self.port = self.SITL_PORT
            if port == "ros" or port == "avoidance":
                self.port = self.ROS_PORT
            if port == "cf" or port == "hw":
                self.port = self.CF_PORT

        if isinstance(params, str):
            # params file path
            self.params = Command.extract_params_from_csv(params)
        else:
            self.params = params
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
            # commands file path
            self.commands = Command.extract(commands)
        else:
            self.commands = commands

        self.speed = speed


class AssertionConfig:
    TRAJECTORY = "trajectory"

    def __init__(
        self,
        log: str = None,
        variable: str = TRAJECTORY,
        expectation=None,
    ) -> None:
        self.log = log
        if expectation is None:
            if variable == self.TRAJECTORY:
                self.expectation = Trajectory.extract(log)
            else:
                self.expectation = None


class RunnerConfig:
    def __init__(self, agent, count=1, path=None) -> None:
        self.agent = agent
        self.count = count
        self.path = path


class DroneTestResult:
    def __init__(
        self,
        log: str = None,
        variable: str = AssertionConfig.TRAJECTORY,
        record=None,
    ) -> None:
        self.log = log
        if record is None:
            if variable == AssertionConfig.TRAJECTORY:
                self.record = Trajectory.extract(log)
            else:
                self.record = None
