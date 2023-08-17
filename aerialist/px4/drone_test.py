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

    @classmethod
    def from_yaml(cls, address):
        with open(address) as file:
            data_dict = munch.DefaultMunch.fromYAML(file, None)
            # data_dict = yaml.safe_load(file)
        print(f'##data_dict{data_dict}')
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
            yaml.dump(self, file)
        return address

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
            # print(f'********{self.simulation.obstacles[0].size.x}')
            if (
                    self.simulation.obstacles is not None
                    and len(self.simulation.obstacles) >= 1
            ):
                params += "--obstacle "
                # temp = [self.simulation.obstacles[0].size.x, self.simulation.obstacles[0].size.y,
                #         self.simulation.obstacles[0].size.z, self.simulation.obstacles[0].position.x,
                #         self.simulation.obstacles[0].position.y, self.simulation.obstacles[0].position.z,
                #         self.simulation.obstacles[0].angle]
                # for p in temp:
                for p in self.simulation.obstacles[0].to_params():
                    params += f"{p} "
            if (
                    self.simulation.obstacles is not None
                    and len(self.simulation.obstacles) >= 2
            ):
                params += "--obstacle2 "
                for p in self.simulation.obstacles[1].to_params():
                    params += f"{p} "
            if(
                self.simulation.world_file_name is not None
            ):
                params += "--world_file_name "
                params += self.simulation.world_file_name[0]

            if (
                    self.simulation.pattern_design is not None
                    and len(self.simulation.pattern_design) >= 1
            ):
                params += " --pattern_design "
                params += f"{self.simulation.pattern_design[0]} "
            if (
                    self.simulation.pattern_design is not None
                    and len(self.simulation.pattern_design) >= 2
            ):
                params += "--pattern_design2 "
                params += f"{self.simulation.pattern_design[1]} "

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
            pattern: List[str] = None,
            world_file_name: List[str] = None,
            obstacles: List[Obstacle] | List[float] = None,
            pattern_design: List[str] = None,
            home_position: List[float] = None,
    ) -> None:
        self.simulator = simulator
        self.world = world
        self.speed = speed
        self.headless = headless
        self.obstacles: List[Obstacle] = obstacles
        self.home_position = home_position
        self.pattern_design = pattern_design
        self.pattern = pattern
        self.world_file_name = None
        if (
                obstacles is not None
                and len(obstacles) > 0
                and not isinstance(obstacles[0], Obstacle)
        ):
            print(f'*** obstacle is {type(obstacles[0])}')

            if isinstance(obstacles[0], munch.DefaultMunch):
                print("Default munch type")
                self.obstacles = Obstacle.from_dict_list(obstacles)
                print(obstacles[0].size.x)
            else:
                self.obstacles = Obstacle.from_coordinates_multiple(obstacles)
            # self.obstacles.append(Obstacle(**obstacles[0]))

        if (
                pattern is not None
                and len(pattern) > 0
        ):
            self.pattern = pattern

        if (
                pattern_design is not None
                and len(pattern_design) > 0
        ):
            self.pattern_design = pattern_design
        if(
            world_file_name is not None 
            and len(world_file_name) > 0
        ):
            self.world_file_name = world_file_name


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
        if commands is None and commands_file is not None:
            self.commands = Command.extract(file_helper.get_local_file(commands_file))


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
