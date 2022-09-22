from __future__ import annotations
import time
import subprocess
import signal
import os
import threading
from decouple import config
import logging
from . import file_helper
from .drone_test import SimulationConfig

logger = logging.getLogger(__name__)


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

    def __init__(self, config: SimulationConfig) -> None:
        super().__init__()
        self.config = config

        sim_command = ""
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
            sim_command += f"exec roslaunch {self.AVOIDANCE_LAUNCH} gui:={str((not self.config.headless) and self.GAZEBO_GUI_AVOIDANCE).lower()} rviz:={str(not self.config.headless).lower()} world_file_name:={self.AVOIDANCE_WORLD} "
            if self.config.obstacles != None and len(self.config.obstacles) > 0:
                sim_command += f"obst:=true obst_x:={self.config.obstacles[0].center().y} obst_y:={self.config.obstacles[0].center().x} obst_z:={self.config.obstacles[0].center().z} obst_l:={self.config.obstacles[0].size().y} obst_w:={self.config.obstacles[0].size().x} obst_h:={self.config.obstacles[0].size().z} "
                if len(self.config.obstacles) > 1:
                    sim_command += f"obst2:=true obst2_x:={self.config.obstacles[1].center().y} obst2_y:={self.config.obstacles[1].center().x} obst2_z:={self.config.obstacles[1].center().z} obst2_l:={self.config.obstacles[1].size().y} obst2_w:={self.config.obstacles[1].size().x} obst2_h:={self.config.obstacles[1].size().z} "

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
                    logger.debug(f"logging started: {self.log_file}")
                    if (
                        self.config.simulator == SimulationConfig.GAZEBO
                        or self.config.simulator == SimulationConfig.JMAVSIM
                    ):
                        return True

                if self.config.simulator == SimulationConfig.ROS and output.endswith(
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
