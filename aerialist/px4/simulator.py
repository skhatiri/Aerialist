from __future__ import annotations
import select
import time
import subprocess
import signal
import os
import threading
from decouple import config
import logging

import sys

sys.path.append('/aerialist')
from external_subprocess import run_tree_subprocess
from external_subprocess import run_box_subprocess
from external_subprocess import spawn_apartment_subprocess
from wind_handler import add_wind
from wind_handler import remove_wind
from light_handler import modify_light
from . import file_helper
from .drone_test import SimulationConfig
import math
from pprint import pprint

logger = logging.getLogger(__name__)


class Simulator(object):
    PX4_DIR = config("PX4_HOME")
    CATKIN_DIR = config("CATKIN_HOME")
    PX4_LOG_DIR = PX4_DIR + "build/px4_sitl_default/tmp/rootfs/"
    ROS_LOG_DIR = config("ROS_HOME")
    GAZEBO_GUI_AVOIDANCE = True
    BOX_PATH = config("AERIALIST_HOME") + config("BOX_PATH")
    WORLD_PATH = config("AERIALIST_HOME") + config("WORLD_PATH")
    AVOIDANCE_WORLD = config("AVOIDANCE_WORLD", default="collision_prevention")
    AVOIDANCE_LAUNCH = config(
        "AVOIDANCE_LAUNCH",
        default="aerialist/resources/simulation/collision_prevention.launch",
    )
    SIMULATION_TIMEOUT = config("SIMULATION_TIMEOUT", cast=int, default=-1)
    AVOIDANCE_BOX = config(
        "AVOIDANCE_BOX", default="aerialist/resources/simulation/box.xacro"
    )
    COPY_DIR = config("LOGS_COPY_DIR", None)
    LAND_TIMEOUT = 20

    def __init__(self, config: SimulationConfig) -> None:
        super().__init__()
        self.config = config
        sim_command = ""
        if config.home_position is not None:
            sim_command += f"export PX4_HOME_LAT={self.config.home_position[0]} ; export PX4_HOME_LON={self.config.home_position[1]} ; export PX4_HOME_ALT={self.config.home_position[2]} ; "
        self.log_file = None
        if (
                self.config.simulator == SimulationConfig.GAZEBO
                or self.config.simulator == SimulationConfig.JMAVSIM
        ):
            self.log_dir = self.PX4_LOG_DIR
            if self.config.headless:
                sim_command += f"HEADLESS=1 "
            if self.config.speed != 1:
                sim_command += f"PX4_SIM_SPEED_FACTOR={self.config.speed} "
            sim_command += f"make -C {self.PX4_DIR} px4_sitl {self.config.simulator}"
        elif self.config.simulator == SimulationConfig.ROS:
            self.log_dir = self.ROS_LOG_DIR
            sim_command += f"source {self.CATKIN_DIR}devel/setup.bash; "
            sim_command += (
                f"DONT_RUN=1 make -C {self.PX4_DIR} px4_sitl_default gazebo; "
            )
            sim_command += f". {self.PX4_DIR}Tools/setup_gazebo.bash {self.PX4_DIR} {self.PX4_DIR}build/px4_sitl_default; "
            # sim_command += f". {self.PX4_DIR}Tools/simulation/gazebo/setup_gazebo.bash {self.PX4_DIR} {self.PX4_DIR}build/px4_sitl_default; "
            sim_command += (
                    "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:"
                    + f"{self.PX4_DIR[:-1]}; "
            )
            sim_command += (
                    'echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:'
                    + f'{self.CATKIN_DIR}src/avoidance/avoidance/sim/models:{self.CATKIN_DIR}src/avoidance/avoidance/sim/worlds" >> ~/.bashrc; '
            )
            # print("***+++")
            # print(self.config.obstacles[0].size.x)
            # pprint(vars(self.config.obstacles[0]))
            sim_command += f"exec roslaunch {self.AVOIDANCE_LAUNCH} gui:={str((not self.config.headless) and self.GAZEBO_GUI_AVOIDANCE).lower()} rviz:={str(True and not self.config.headless).lower()} world_file_name:={self.config.world_file_name[0]} world_path:={self.WORLD_PATH} box_path:={self.BOX_PATH} "
            pattern_box_count = 0
            tree_count = 0
            apartment_count = 0
            box_count = 0
            if self.config.obstacles is not None and len(self.config.obstacles) > 0:
                for obstacle in self.config.obstacles:
                    if obstacle.shape == "BOX":
                        if obstacle.pattern_design is not None:
                            if pattern_box_count == 0:
                                sim_command += f"obst:=true obst_x:={obstacle.position.y} obst_y:={obstacle.position.x} obst_z:={obstacle.position.z} obst_l:={obstacle.size.y} obst_w:={obstacle.size.x} obst_h:={obstacle.size.z} obst_yaw:={math.radians(-obstacle.angle)} pattern_design:={obstacle.pattern_design} "
                                pattern_box_count += 1
                            if pattern_box_count == 1:
                                sim_command += f"obst2:=true obst2_x:={obstacle.position.y} obst2_y:={obstacle.position.x} obst2_z:={obstacle.position.z} obst2_l:={obstacle.size.y} obst2_w:={obstacle.size.x} obst2_h:={self.config.obstacles[1].size.z} obst2_yaw:={math.radians(-obstacle.angle)} pattern_design2:={obstacle.pattern_design} "
                        else:
                            box_count += 1
                            box_name = "box_" + str(box_count)
                            run_box_subprocess(box_name, str(obstacle.size.x),
                                               str(obstacle.size.y),
                                               str(obstacle.size.z),
                                               str(obstacle.position.x),
                                               str(obstacle.position.y),
                                               str(obstacle.position.z))
                    if obstacle.shape == "TREE":
                        tree_count += 1
                        tree_name = "tree_" + str(tree_count)
                        run_tree_subprocess(tree_name,
                                            str(obstacle.position.x),
                                            str(obstacle.position.y),
                                            str(obstacle.position.z))
                    if obstacle.shape == "APARTMENT":
                        apartment_count += 1
                        apartment_name = "apartment_" + str(apartment_count)
                        # print(f'apartment address is x={obstacle.position.x}, y={obstacle.position.y}')
                        spawn_apartment_subprocess(apartment_name,
                                            str(obstacle.position.x),
                                            str(obstacle.position.y),
                                            str(obstacle.position.z))

        world_file_path = self.WORLD_PATH + self.config.world_file_name[0]+".world"
        if self.config.wind != 0:
            remove_wind(world_file_path)
            add_wind(world_file_path, self.config.wind)
        else:
            remove_wind(world_file_path)
        if self.config.light > 0:
            modify_light(world_file_path, self.config.light)

        logger.info("executing:" + sim_command)
        self.sim_process = subprocess.Popen(
            sim_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid,
            executable="/bin/bash",
        )

        if self.start():
            logger.debug("simulator started")
            self.bkgnd = threading.Thread(target=self.sim_thread)
            self.bkgnd.start()
            # self.bkgnd
        else:
            raise (Exception("Simulator could not start"))

    def start(self) -> bool:
        try:
            while True:
                output = self.sim_process.stdout.readline().strip()
                if output.startswith("ERROR"):
                    logger.error(output)
                elif output:
                    logger.debug(output)

                self.handle_errors(output)

                if output.startswith("INFO  [logger] Opened full log file"):
                    self.log_file = output.replace(
                        "INFO  [logger] Opened full log file: ./", self.log_dir
                    )
                    logger.info(f"logging started: {self.log_file}")
                    if (
                            self.config.simulator == SimulationConfig.GAZEBO
                            or self.config.simulator == SimulationConfig.JMAVSIM
                    ):
                        return True

                if self.config.simulator == SimulationConfig.ROS and output.endswith(
                        "INFO  [tone_alarm] home set"
                ):
                    logger.info("Avoidance is ready (waiting 10 seconds to wrap up)")
                    time.sleep(10)
                    return True

                return_code = self.sim_process.poll()
                if return_code is not None:
                    # if return_code != 0:
                    logger.critical(f"in Simulation process: RETURN CODE:{return_code}")
                    for error in self.sim_process.stdout.readlines():
                        logger.critical(error.strip())
                    logger.critical(output)
                    return False
        except Exception as e:
            logger.critical(e)
            raise (e)

    def sim_thread(self):
        try:
            poll_obj = select.poll()
            poll_obj.register(self.sim_process.stdout, select.POLLIN)
            start_time = time.perf_counter()
            land_time = None
            while True:
                poll_result = poll_obj.poll(5000)
                if poll_result:
                    output = self.sim_process.stdout.readline().strip()
                else:
                    output = ""

                if output.startswith("ERROR"):
                    logger.error(output)
                elif output:
                    logger.debug(output)

                self.handle_errors(output)

                if (
                        land_time is not None
                        and time.perf_counter() - land_time > self.LAND_TIMEOUT
                ):
                    self.kill()
                    raise Exception(
                        "timeout expired after land - Killing the process..."
                    )

                if (
                    self.SIMULATION_TIMEOUT > 0
                    and time.perf_counter() - start_time > self.SIMULATION_TIMEOUT
                ):
                    logger.error(
                        "Simulation Timeout. Terminating the rest of the execution..."
                    )
                    self.kill()
                    logger.debug(f"logging finished: {self.log_file}")
                    if self.COPY_DIR is not None:
                        copy_path = file_helper.copy(
                            self.log_file,
                            self.COPY_DIR + file_helper.time_filename(True) + ".ulg",
                        )
                        if copy_path is not None:
                            self.log_file = os.path.abspath(copy_path)
                            logger.debug(f"log copied: {self.log_file}")
                    return

                if output.find("Landing detected") >= 0:
                    land_time = time.perf_counter()
                    logger.info("landing detected: starting the timeout")

                if output.startswith("INFO  [logger] Opened full log file"):
                    self.log_file = output.replace(
                        "INFO  [logger] Opened full log file: ./", self.log_dir
                    )
                    logger.debug(f"logging started: {self.log_file}")
                    land_time = None

                if output.startswith("INFO  [logger] closed logfile"):
                    self.kill()
                    logger.debug(f"logging finished: {self.log_file}")
                    if self.COPY_DIR is not None:
                        copy_path = file_helper.copy(
                            self.log_file,
                            self.COPY_DIR + file_helper.time_filename(True) + ".ulg",
                        )
                        if copy_path is not None:
                            self.log_file = os.path.abspath(copy_path)
                            logger.debug(f"log copied: {self.log_file}")
                    return

                return_code = self.sim_process.poll()
                if return_code is not None:
                    if return_code != 0:
                        logger.critical(
                            f"in Simulation process: RETURN CODE:{return_code}"
                        )
                        for error in self.sim_process.stdout.readlines():
                            logger.critical(error.strip())
                        logger.critical(output)
                    return
        except Exception as e:
            logger.critical(e)
            raise (e)

    def kill(self):
        try:
            if self.sim_process.poll() is None:
                os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
                self.sim_process.wait()
        except Exception as e:
            logger.error("Exception while killing the simulation process:" + str(e))

    def handle_errors(self, output: str):
        ex = None
        if output.startswith("ERROR [px4_daemon] error binding socket"):
            ex = Exception(("Unable to use the PX4 Lock - Killing the process..."))
        if output.startswith("ERROR [px4] Startup script returned"):
            ex = Exception("Unable to startup PX4 - Killing the process...")
        if output.startswith("ERROR [simulator] poll timeout"):
            ex = Exception("Unable to poll simulator - Killing the process...")

        if ex is not None:
            self.kill()
            raise ex

    def get_log(self):
        try:
            self.bkgnd.join()
        except Exception as e:
            logger.error(e)
        return self.log_file
