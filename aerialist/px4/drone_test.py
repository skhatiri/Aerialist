from __future__ import annotations
from statistics import median
from typing import List
import munch
import yaml
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
        agent: AgentConfig = None,
    ) -> None:
        self.drone = drone
        self.simulation = simulation
        self.test = test
        self.assertion = assertion
        self.agent = agent
        if simulation is not None and simulation.home_position is None:
            if assertion is not None and assertion.log_file is not None:
                simulation.home_position = Trajectory.get_home(assertion.log_file)
        if simulation is not None and test is not None:
            self.test.speed = self.simulation.speed

    @classmethod
    def from_yaml(cls, address):
        file_address = file_helper.get_local_file(address)
        with open(file_address) as file:
            data_dict = munch.DefaultMunch.fromYAML(file, None)
        config = cls()
        if data_dict.drone is not None:
            config.drone = DroneConfig(**data_dict.drone)
        if data_dict.simulation is not None:
            config.simulation = SimulationConfig(**data_dict.simulation)
        if data_dict.test is not None:
            config.test = TestConfig(**data_dict.test)
        if data_dict.assertion is not None:
            config.assertion = AssertionConfig(**data_dict.assertion)
        if data_dict.agent is not None:
            config.agent = AgentConfig(**data_dict.agent)
        return cls(
            config.drone, config.simulation, config.test, config.assertion, config.agent
        )

    def to_yaml(self, address):
        with open(address, "w") as file:
            yaml.dump(self.to_dict(), file)
        return address

    def to_dict(self):
        dic = {}
        if self.drone is not None:
            dic["drone"] = self.drone.to_dict()
        if self.simulation is not None:
            dic["simulation"] = self.simulation.to_dict()
        if self.test is not None:
            dic["test"] = self.test.to_dict()
        if self.agent is not None:
            dic["agent"] = self.agent.to_dict()
        if self.assertion is not None:
            dic["assertion"] = self.assertion.to_dict()
        return dic

    def cmd_params(self):
        # CMD must be updated if the interface in entry.py changes
        params = "exec "
        if self.drone is not None:
            if self.drone.port is not None:
                params += f"--drone {self.drone.port} "
            if self.drone.params_file is not None:
                params += f"--params '{self.drone.params_file}' "
            if self.drone.mission_file is not None:
                params += f"--mission '{self.drone.mission_file}' "
        if self.simulation is not None:
            if self.simulation.simulator is not None:
                params += f"--simulator {self.simulation.simulator} "
            if self.simulation.speed is not None:
                params += f"--speed {self.simulation.speed} "
            if self.simulation.headless:
                params += f"--headless "
            if self.simulation.home_position is not None:
                params += f"--home {self.simulation.home_position[0]} {self.simulation.home_position[1]} {self.simulation.home_position[2]} "

            if (
                self.simulation.obstacles is not None
                and len(self.simulation.obstacles) >= 1
            ):
                params += "--obstacle "
                for p in self.simulation.obstacles[0].to_params():
                    params += f"{p} "
            if (
                self.simulation.obstacles is not None
                and len(self.simulation.obstacles) >= 2
            ):
                params += "--obstacle2 "
                for p in self.simulation.obstacles[1].to_params():
                    params += f"{p} "

        if self.test is not None:
            if self.test.commands_file is not None:
                params += f"--commands '{self.test.commands_file}' "

        if self.assertion is not None:
            if self.assertion.log_file is not None:
                params += f"--log '{self.assertion.log_file}' "
        if self.agent is not None:
            if self.agent.engine is not None:
                params += f"--agent {self.agent.engine} "
            if self.agent.path is not None:
                params += f"--path '{self.agent.path}' "
            if self.agent.count is not None:
                params += f"-n {self.agent.count} "
            if self.agent.id is not None:
                params += f"--id {self.agent.id} "

        return params

    @classmethod
    def plot(
        cls, test: DroneTest, results: List[DroneTestResult], obstacle_distance=True
    ) -> None:
        distance = None
        if obstacle_distance:
            distance = True
        elif test.assertion is not None and test.assertion.expectation is not None:
            distance = median(
                [r.record.distance(test.assertion.expectation) for r in results]
            )
        if results is not None and len(results) >= 1:
            Trajectory.plot_multiple(
                [r.record for r in results],
                goal=None if test.assertion is None else test.assertion.expectation,
                distance=distance,
                obstacles=None
                if test.simulation is None
                else test.simulation.obstacles,
            )


class DroneConfig:
    CF_PORT = 14550
    SITL_PORT = 14540
    ROS_PORT = 14541

    def __init__(
        self,
        port: int | str = SITL_PORT,
        params: dict = None,
        params_file: str = None,
        mission_file=None,
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
        self.params = params
        self.params_file = params_file
        if params is None and params_file is not None:
            self.params = Command.extract_params_from_csv(
                file_helper.get_local_file(params_file)
            )

        self.mission_file = mission_file
        if mission_file is not None:
            self.mission_file = file_helper.get_local_file(mission_file)

    def to_dict(self):
        dic = {}
        if self.port is not None:
            dic["port"] = self.port
        if self.params is not None:
            dic["params"] = self.params
        elif self.params_file is not None:
            dic["params_file"] = self.params_file
        if self.mission_file is not None:
            dic["mission_file"] = self.mission_file
        return dic


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
        obstacles: List[Obstacle] | List[float] = None,
        home_position: List[float] = None,
    ) -> None:
        self.simulator = simulator
        self.world = world
        self.speed = speed
        self.headless = headless
        self.obstacles: List[Obstacle] = obstacles
        self.home_position = home_position
        if (
            obstacles is not None
            and len(obstacles) > 0
            and not isinstance(obstacles[0], Obstacle)
        ):
            if isinstance(obstacles[0], munch.DefaultMunch):
                self.obstacles = Obstacle.from_dict_multiple(obstacles)
            else:
                self.obstacles = Obstacle.from_coordinates_multiple(obstacles)

    def to_dict(self):
        dic = {}
        if self.simulator is not None:
            dic["simulator"] = self.simulator
        if self.world != "default":
            dic["world"] = self.world
        if self.speed != 1:
            dic["speed"] = self.speed
        dic["headless"] = self.headless
        if self.obstacles is not None:
            dic["obstacles"] = [obs.to_dict() for obs in self.obstacles]
        if self.home_position is not None:
            dic["home_position"] = self.home_position
        return dic


class TestConfig:
    def __init__(
        self,
        commands: List[Command] = None,
        commands_file: str = None,
        speed: float = 1,
    ) -> None:
        self.speed = speed
        self.commands = commands
        self.commands_file = commands_file
        if commands is not None and type(commands[0]) != Command:
            self.commands = [Command(**c) for c in commands]
        if commands is None and commands_file is not None:
            self.commands = Command.extract(file_helper.get_local_file(commands_file))

    def to_dict(self):
        dic = {}
        if self.commands is not None and len(self.commands) < 10:
            dic["commands"] = [c.to_dict() for c in self.commands]
        elif self.commands_file is not None:
            dic["commands_file"] = self.commands_file
        if self.speed != 1:
            dic["speed"] = self.speed
        return dic


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
        self.variable = variable
        if expectation is None and log_file is not None:
            if variable == self.TRAJECTORY:
                # todo: jMavsim complications (take into account local positioning differences with gazebo)
                self.expectation = Trajectory.extract(
                    file_helper.get_local_file(log_file)
                )

    def to_dict(self):
        dic = {}
        if self.log_file is not None:
            dic["log_file"] = self.log_file
        if self.variable is not None:
            dic["variable"] = self.variable
        return dic


class AgentConfig:
    K8S = "k8s"
    DOCKER = "docker"
    LOCAL = "local"

    def __init__(
        self,
        engine: str,
        count: int = 1,
        path: str = None,
        id: str = None,
    ) -> None:
        self.engine = engine
        self.count = count
        self.path = path
        self.id = id

    def to_dict(self):
        dic = {}
        dic["engine"] = self.engine
        if self.count != 1:
            dic["count"] = self.count
        if self.path is not None:
            dic["path"] = self.path
        if self.id is not None:
            dic["id"] = self.id
        return dic


class DroneTestResult:
    def __init__(
        self,
        log_file: str = None,
        variable: str = AssertionConfig.TRAJECTORY,
        record=None,
    ) -> None:
        self.log_file = log_file
        self.record = record
        self.variable = variable
        if record is None and log_file is not None:
            # todo: jMavsim complications (take into account local positioning differences with gazebo)
            if variable == AssertionConfig.TRAJECTORY:
                self.record = Trajectory.extract(file_helper.get_local_file(log_file))

    @classmethod
    def load_folder(
        cls, logs_folder: str, variable: str = AssertionConfig.TRAJECTORY
    ) -> List[DroneTestResult]:
        logs_folder = file_helper.get_local_folder(logs_folder)
        logs = file_helper.get_logs_address(logs_folder)
        results = [DroneTestResult(log, variable) for log in logs]
        return results
