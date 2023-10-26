from __future__ import annotations
from statistics import median
from typing import List
import munch
import pyulog
import yaml
import csv
from .command import Command
from .obstacle import Obstacle
from .trajectory import Trajectory
from . import file_helper
from pprint import pprint
from itertools import zip_longest
from decouple import config
from datetime import datetime


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
        # print(f'##data_dict{data_dict}')
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
            if (
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
            wind=0,
            light=0.4,
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
        self.wind = wind
        self.light = light
        self.world_file_name = None
        if (
                obstacles is not None
                and len(obstacles) > 0
                and not isinstance(obstacles[0], Obstacle)
        ):
            if isinstance(obstacles[0], munch.DefaultMunch):
                self.obstacles = Obstacle.from_obstacle_list(obstacles)
                print(self.obstacles)
            else:
                self.obstacles = Obstacle.from_coordinates_multiple(obstacles)

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
        if (
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
            wind=test.simulation.wind,
            light=test.simulation.light
        )

    find_threshold_limit(results)
    log_csv(test, results)


def find_threshold_limit(results: List[DroneTestResult]):
    result_dir = config("RESULTS_DIR", default="results/")
    threshold_file = config("THRESHOLD_FILE", default="threshold.csv")
    threshold_file_edit_mode = config("THRESHOLD_FILE_EDIT_MODE", default="a")
    average_traj = Trajectory.average([r.record for r in results])
    distance_list = [r.record.distance(average_traj) for r in results]
    print(f'The distance printed in the threshold is {distance_list}')
    f = open(result_dir + threshold_file, threshold_file_edit_mode)
    writer = csv.writer(f)
    writer.writerow(distance_list)
    f.close()


def log_csv(test: DroneTest, results: List[DroneTestResult]) -> None:
    result_dir = config("RESULTS_DIR", default="results/")
    dataset_file_combined = config("DATASET_FILE", default="dataset_file_combined")
    dataset_file_edit_mode = config("DATA_FILE_EDIT_MODE", default="a")
    cpu_file = config("CPU_FILE", default="cpu_file")
    cpu_file_edit_mode = config("CPU_FILE_EDIT_MODE", default="a")
    file_extension = config("FILE_EXTENSION", default=".csv")
    default_separation = config("DEFAULT_SEPARATION", default=",")
    file_ts = str(datetime.now().strftime("%Y%m%d%H%M%S"))
    wind = test.simulation.wind
    light = test.simulation.light
    print(f"***LOG:{results[0].log_file}")
    log = pyulog.ULog(results[0].log_file)
    print("**Printing cpu data list")
    cpu_data = log.get_dataset('cpuload')
    print(cpu_data)
    cpu_load = cpu_data.data['load']
    ram_usage = cpu_data.data['ram_usage']
    cpu_timestamp = cpu_data.data['timestamp']
    cpu_timestamp_list = []
    cpu_header = False
    print(f'**keys are {cpu_data.data.keys()}')
    print(f'cpu load and ram usage length are {len(cpu_load)},{len(ram_usage)}')
    for temp_cpu_load, temp_ram_usage, temp_cpu_timestamp in zip_longest(cpu_load, ram_usage, cpu_timestamp):
        cpu_row = [temp_cpu_timestamp, temp_cpu_load, temp_ram_usage]
        cpu_timestamp_list.append(temp_cpu_timestamp)
        f = open(result_dir + cpu_file + "_" + file_ts + file_extension,
                 cpu_file_edit_mode)
        writer = csv.writer(f)
        if not cpu_header:
            writer.writerow(["timestamp", "cpu_usage", "ram_usage"])
            cpu_header = True
        writer.writerow(cpu_row)
        f.close()

    trajectories: List[Trajectory] = [r.record for r in results]
    for trajectory in trajectories:
        positions = trajectory.positions
        obstacle_flag = False
        obstacles_present = 0
        obstacle_list = []
        # number_of_obstacles = -1
        number_of_trees = 0
        number_of_apartments = 0
        number_of_boxes = 0
        tree_cumm_list = []
        apt_cumm_list = []
        box_cumm_list = []
        box_min_distance = []
        tree_min_distance = []
        apt_min_distance = []
        obstacle_header_distance = []
        tree_count = apt_count = box_count = 0
        for obs in test.simulation.obstacles:
            min_distance, returned_list = trajectory.distance_to_obstacles([obs])
            if obs.shape == "TREE":
                tree_cumm_list.append(returned_list)
                tree_count += 1
                tree_index = "tree_" + str(tree_count) + "_min_distance"
                obstacle_header_distance.append(tree_index)
                tree_min_distance.append(min_distance)
            elif obs.shape == "APARTMENT":
                apt_cumm_list.append(returned_list)
                apt_count += 1
                apartment_index = "apt_" + str(apt_count) + "_min_distance"
                obstacle_header_distance.append(apartment_index)
                apt_min_distance.append(min_distance)
            elif obs.shape == "BOX":
                box_cumm_list.append(returned_list)
                box_count += 1
                box_index = "box_" + str(box_count) + "_min_distance"
                obstacle_header_distance.append(box_index)
                box_min_distance.append(min_distance)

        sum_tree = [sum(elts) for elts in zip(*tree_cumm_list)]
        sum_apt = [sum(elts) for elts in zip(*apt_cumm_list)]
        sum_box = [sum(elts) for elts in zip(*box_cumm_list)]
        avg_tree = [divmod(x, len(tree_cumm_list))[0] for x in sum_tree]
        avg_apt = [divmod(x, len(apt_cumm_list))[0] for x in sum_apt]
        avg_box = [divmod(x, len(box_cumm_list))[0] for x in sum_box]
        i = 0
        # print("*****Average tree****")
        # print(avg_tree)
        # print("Whole obstacle distance")
        # print([trajectory.distance_to_obstacles([obs]) for obs in test.simulation.obstacles])
        # print(f'distances and list are {distances} and {distance_list}')
        # x = y = z = r = wind = light = 0.0

        csv_header = ["x", "y", "z", "r", "timestamp", "wind", "light",
                      "obstacle_present"]
        csv_header_obstacles = ["no_of_obst", "no_of_boxes", "no_of_trees", "no_of_apt", "avg_dist_boxes",
                                "avg_dist_trees", "avg_dist_apt"]
        csv_header_obst_end = ["obst_details"]
        header_flag = False
        if len(test.simulation.obstacles) > 0:
            obstacle_flag = True
            obstacles_present = 1
            for obstacle in test.simulation.obstacles:
                if obstacle.shape == "TREE":
                    number_of_trees += 1
                elif obstacle.shape == "APARTMENT":
                    number_of_apartments += 1
                elif obstacle.shape == "BOX":
                    number_of_boxes += 1
                obstacle_type = obstacle.shape
                obst_x = obstacle.position.x
                obst_y = obstacle.position.y
                obst_z = obstacle.position.z
                obst_r = obstacle.position.r
                temp_obst_row = (obstacle_type, obst_x, obst_y, obst_z, obst_r)
                obstacle_list.append(temp_obst_row)
        for position in positions:
            x = position.x
            y = position.y
            z = position.z
            r = position.r
            timestamp = position.timestamp
            average_tree_distance = avg_tree[i]
            average_box_distance = avg_box[i]
            average_apt_distance = avg_apt[i]
            i += 1
            header_final = []
            if obstacle_flag:
                number_of_obstacles = len(obstacle_list)
                if not header_flag:
                    header_final = csv_header + csv_header_obstacles + obstacle_header_distance + csv_header_obst_end
                row = [x, y, z, r, timestamp, wind, light, obstacles_present, number_of_obstacles, number_of_trees,
                       number_of_boxes,
                       number_of_apartments, average_box_distance, average_tree_distance,
                       average_apt_distance] + tree_min_distance + box_min_distance + apt_min_distance + [obstacle_list]
            else:
                if not header_flag:
                    header_final = csv_header
                row = [x, y, z, r, timestamp, wind, light, obstacles_present]
            f = open(result_dir + dataset_file_combined + "_" + file_ts + file_extension, dataset_file_edit_mode)
            write = csv.writer(f)
            if not header_flag:
                write.writerow(header_final)
                header_flag = True
            write.writerow(row)
            f.close()

        '''position_data = log.get_dataset('vehicle_local_position')
        print(f'**keys of are {position_data.data.keys()}')
        x_position = position_data.data['x']
        y_position = position_data.data['y']
        z_position = position_data.data['z']
        heading = position_data.data['heading']
        timestamp = position_data.data['timestamp']
        print(f'****Printing position data')
        print(len(x_position))
        time_nav = 0
        for x_pos, y_pos, z_pos, heading, timestamp in zip_longest(x_position, y_position, z_position, heading,
                                                                   timestamp):
            if timestamp in cpu_timestamp_list:
                if time_nav < len(cpu_timestamp_list):
                    time_nav += 1
                if obstacle_flag:
                    number_of_obstacles = len(obstacle_list)
                    temp_row = [x_pos, y_pos, z_pos, timestamp, heading, wind, light, obstacles_present,
                                number_of_obstacles, number_of_trees, number_of_boxes,
                                number_of_apartments] + tree_min_distance + box_min_distance + apt_min_distance + [
                                   obstacle_list]
                else:
                    temp_row = [x_pos, y_pos, z_pos, timestamp, heading, wind, light, obstacles_present]

                f = open('/home/prasun/Aerialist/results/csvLog2.csv', 'a')
                write = csv.writer(f)
                write.writerow(temp_row)
                f.close()
            elif timestamp > cpu_timestamp[time_nav]:
                if timestamp - 4000 in cpu_timestamp_list:
                    number_of_obstacles = len(obstacle_list)
                    if time_nav < len(cpu_timestamp_list):
                        time_nav += 1
                    if obstacle_flag:
                        temp_row = [x_pos, y_pos, z_pos, timestamp, heading, wind, light, obstacles_present,
                                    number_of_obstacles, number_of_trees, number_of_boxes,
                                    number_of_apartments, obstacle_list]
                    else:
                        temp_row = [x_pos, y_pos, z_pos, timestamp, heading, wind, light, obstacles_present]

                    f = open('/home/prasun/Aerialist/results/csvLog2.csv', 'a')
                    write = csv.writer(f)
                    write.writerow(temp_row)
                    f.close()'''
