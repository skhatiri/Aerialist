from __future__ import annotations
import time
from typing import List
import subprocess
import signal
import os
import threading
from enum import Enum
from decouple import config
import logging
from .obstacle import Obstacle
from . import ulog_helper

logger = logging.getLogger(__name__)


class Environment(Enum):
    GAZEBO = 1
    JMAVSIM = 2
    AVOIDANCE = 3


class Simulator(object):
    # pre_commands = "export PX4_HOME_LAT=0 & export PX4_HOME_LON=-0 & export PX4_HOME_ALT=0"
    PX4_DIR = config("PX4_HOME")
    CATKIN_DIR = config("CATKIN_HOME")
    PX4_LOG_DIR = PX4_DIR + "build/px4_sitl_default/tmp/rootfs/"
    ROS_LOG_DIR = config("ROS_HOME")
    GAZEBO_GUI_AVOIDANCE = True
    AVOIDANCE_WORLD = config("AVOIDANCE_WORLD", default="boxes1")
    AVOIDANCE_LAUNCH = config(
        "AVOIDANCE_LAUNCH", default="resources/simulation/collision_prevention.launch"
    )
    COPY_DIR = config("LOGS_COPY_DIR", None)
    LAND_TIMEOUT = 20

    def __init__(
        self,
        env: Environment = Environment.GAZEBO,
        headless: bool = False,
        speed: float = 1,
        obstacles: List[Obstacle] = None,
    ) -> None:
        super().__init__()
        sim_command = ""
        self.env = env
        self.obstacles = obstacles
        self.log_file = None
        if env == Environment.GAZEBO or env == Environment.JMAVSIM:
            self.log_dir = self.PX4_LOG_DIR
            if headless:
                sim_command += f"HEADLESS=1 "
            if speed != 1:
                sim_command += f"PX4_SIM_SPEED_FACTOR={speed} "
            sim_command += f"make -C {self.PX4_DIR} px4_sitl {env.name.lower()}"
        elif env == Environment.AVOIDANCE:
            self.log_dir = self.ROS_LOG_DIR
            sim_command = f"source {self.CATKIN_DIR}devel/setup.bash; "
            sim_command += (
                f"DONT_RUN=1 make -C {self.PX4_DIR} px4_sitl_default gazebo; "
            )
            sim_command += f". {self.PX4_DIR}Tools/setup_gazebo.bash {self.PX4_DIR} {self.PX4_DIR}build/px4_sitl_default; "
            sim_command += (
                "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:"
                + f"{self.PX4_DIR[:-1]}; "
            )
            sim_command += (
                'echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:'
                + f'{self.CATKIN_DIR}src/avoidance/avoidance/sim/models:{self.CATKIN_DIR}src/avoidance/avoidance/sim/worlds" >> ~/.bashrc; '
            )
            sim_command += f"exec roslaunch {self.AVOIDANCE_LAUNCH} gui:={str((not headless) and self.GAZEBO_GUI_AVOIDANCE).lower()} rviz:={str(not headless).lower()} world_file_name:={self.AVOIDANCE_WORLD} "
            if obstacles != None and len(obstacles) > 0:
                sim_command += f"obst:=true obst_x:={obstacles[0].center().y} obst_y:={obstacles[0].center().x} obst_z:={obstacles[0].center().z} obst_l:={obstacles[0].size().y} obst_w:={obstacles[0].size().x} obst_h:={obstacles[0].size().z} "
                if len(obstacles) > 1:
                    sim_command += f"obst2:=true obst2_x:={obstacles[1].center().y} obst2_y:={obstacles[1].center().x} obst2_z:={obstacles[1].center().z} obst2_l:={obstacles[1].size().y} obst2_w:={obstacles[1].size().x} obst2_h:={obstacles[1].size().z} "

        logger.debug("executing:" + sim_command)
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
            self.bkgnd = threading.Thread(target=self.sim_thread)
            self.bkgnd.start()
            self.bkgnd
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
                    logger.debug(f"logging started: {self.log_file}")
                    if (
                        self.env == Environment.JMAVSIM
                        or self.env == Environment.GAZEBO
                    ):
                        return True

                if self.env == Environment.AVOIDANCE and output.endswith(
                    "INFO  [tone_alarm] home set"
                ):
                    logger.info("Avoidance is ready (waiting 5 seconds to wrap up)")
                    time.sleep(5)
                    return True

                return_code = self.sim_process.poll()
                if return_code is not None:
                    # if return_code != 0:
                    logger.critical("in Simulation process: RETURN CODE:", return_code)
                    for error in self.sim_process.stdout.readlines():
                        logger.critical(error.strip())
                    logger.critical(output)
                    return False
        except Exception as e:
            logger.critical(e)
            raise (e)

    def sim_thread(self):
        try:
            land_time = None
            while True:
                output = self.sim_process.stdout.readline().strip()
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
                        copy_path = ulog_helper.copy(
                            self.log_file,
                            self.COPY_DIR + ulog_helper.time_filename(True) + ".ulg",
                        )
                        if copy_path is not None:
                            self.log_file = os.path.abspath(copy_path)
                            logger.debug(f"log copied: {self.log_file}")
                    return

                return_code = self.sim_process.poll()
                if return_code is not None:
                    if return_code != 0:
                        logger.critical(
                            "in Simulation process: RETURN CODE:", return_code
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
