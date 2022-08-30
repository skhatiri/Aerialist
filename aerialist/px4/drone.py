import logging
import time
from mavsdk import System, action, asyncio, mission, mission_raw
from px4.command import Command, FlightMode, DefaultCommands
import keyboard
from enum import Enum
from decouple import config
from datetime import datetime

logger = logging.getLogger(__name__)


class MavAddress(Enum):
    CF = 14550
    SIM = 14540
    ROS = 14541


class Drone(object):
    """sets up and maintains the conection to the drone"""

    SETPOINT_PERIOD = 0.2
    DEFAULT_PARAMS = config("PARAMS", default=None)

    def __init__(
        self, mav_address: MavAddress, params: dict = None, mission_file: str = None
    ) -> None:
        super().__init__()
        self.address = f"udp://:{mav_address.value}"
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.connect_async(self.address))

        if self.DEFAULT_PARAMS != None:
            default_params = Command.extract_params_from_csv(self.DEFAULT_PARAMS)
            self.set_params(default_params)
        if params != None:
            self.set_params(params)

        if mission_file != None:
            self.upload_mission(mission_file)

    def start_mission(self):
        loop = asyncio.get_event_loop()
        logger.info("starting mission")
        try:
            loop.run_until_complete(self.drone.action.arm())
            loop.run_until_complete(self.drone.mission_raw.start_mission())
            logger.info("mission started")
            return
        except Exception as e:
            raise Exception("Mission start denied:" + str(e))

    def upload_mission(self, mission_file):
        logger.info(f"uploading mission {mission_file}")
        loop = asyncio.get_event_loop()
        mission_data = loop.run_until_complete(
            self.drone.mission_raw.import_qgroundcontrol_mission(mission_file)
        )
        loop.run_until_complete(
            self.drone.mission_raw.upload_mission(mission_data.mission_items)
        )

    def run(self, command: Command):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.run_async(command))

    def run_long(self, command: Command, duration):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.run_long_async(command, duration))

    def set_params(self, params: dict):
        logger.info("updating drone parameters")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.set_params_async(params))

    async def run_long_async(self, command: Command, duration):
        start_time = time.time()
        while time.time() < start_time + duration:
            await self.run_async(command)
            await asyncio.sleep(self.SETPOINT_PERIOD)

    async def run_async(self, command: Command) -> None:
        try:
            if command.mode == FlightMode.Setpoint:
                await self.drone.manual_control.set_manual_control_input(
                    command.x, command.y, command.z, command.r
                )
            elif command.mode == FlightMode.Arm:
                await self.drone.action.arm()
            elif command.mode == FlightMode.Disarm:
                await self.drone.action.disarm()
            elif command.mode == FlightMode.Takeoff:
                await self.drone.action.takeoff()
            elif command.mode == FlightMode.Land:
                await self.drone.action.land()
            elif command.mode == FlightMode.Hold:
                await self.drone.action.hold()
            elif (
                command.mode == FlightMode.Manual
                or command.mode == FlightMode.Stabilized
            ):
                await self.run_async(DefaultCommands.Hover)
            elif command.mode == FlightMode.Altitude:
                await self.drone.manual_control.start_altitude_control()
            elif command.mode == FlightMode.Position:
                await self.drone.manual_control.start_position_control()
        except action.ActionError as e:
            logger.error(str(e))
        except ConnectionError as e:
            logger.error(str(e))

    async def set_params_async(self, params: dict):
        for name, value in params.items():
            logger.debug(f"param: {name}={value}")
            if isinstance(value, int):
                await self.drone.param.set_param_int(name, value)
            else:
                await self.drone.param.set_param_float(name, value)

    async def connect_async(self, mav_address):
        self.drone = System()
        await self.drone.connect(system_address=mav_address)

        # This waits till a mavlink based drone is connected
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                # logger.info(f"-- Connected to drone!")
                break

        # Checking if Global Position Estimate is ok
        async for global_lock in self.drone.telemetry.health():
            if global_lock.is_global_position_ok:
                # logger.info("-- Global position state is ok")
                break
        logger.info("Conected to the drone")

    def manual(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.manual_async())

    async def manual_async(self):
        """must run the script with sudo"""
        logger.info("in keyboard control")
        logger.info(
            "esc:\tend\nspace:\tarm\nenter:\tdisarm\n\nt:\ttakeoff\nl:\tland\np:\tposition\nm:\tmanual\n\nwasd & up down left right: direction"
        )
        in_setpoint_mode = False
        while True:
            try:
                command = None
                if keyboard.is_pressed("esc"):
                    return
                if keyboard.is_pressed("space"):
                    await self.run_long_async(
                        DefaultCommands.Hover, 3.1 * self.SETPOINT_PERIOD
                    )
                    await self.run_async(DefaultCommands.Arm)
                    logger.info("Armed")
                    await self.run_long_async(
                        DefaultCommands.Hover, 3.1 * self.SETPOINT_PERIOD
                    )
                    await self.run_async(DefaultCommands.Position)
                    logger.info("in position mode")
                    in_setpoint_mode = True
                    continue
                if keyboard.is_pressed("enter"):
                    await self.run_async(DefaultCommands.Disarm)
                    in_setpoint_mode = False
                    break

                if keyboard.is_pressed("t"):
                    await self.run_async(DefaultCommands.Takeoff)
                    in_setpoint_mode = False
                if keyboard.is_pressed("l"):
                    await self.run_async(DefaultCommands.Land)
                    in_setpoint_mode = False

                if keyboard.is_pressed("p"):
                    await self.run_async(DefaultCommands.Position)
                    in_setpoint_mode = True

                if keyboard.is_pressed("m"):
                    await self.run_async(DefaultCommands.Manual)
                    in_setpoint_mode = True

                if keyboard.is_pressed("w"):
                    command = DefaultCommands.Up
                if keyboard.is_pressed("s"):
                    command = DefaultCommands.Down
                if keyboard.is_pressed("a"):
                    command = DefaultCommands.Spin_left
                if keyboard.is_pressed("d"):
                    command = DefaultCommands.Spin_right
                if keyboard.is_pressed("up"):
                    command = DefaultCommands.Fornt
                if keyboard.is_pressed("down"):
                    command = DefaultCommands.Back
                if keyboard.is_pressed("left"):
                    command = DefaultCommands.Left
                if keyboard.is_pressed("right"):
                    command = DefaultCommands.Right

                if command != None:
                    in_setpoint_mode = True

                if in_setpoint_mode and command == None:
                    command = DefaultCommands.Hover

                if command != None:
                    await self.run_async(command)

                await asyncio.sleep(self.SETPOINT_PERIOD)

            except Exception as e:
                logger.error("Unexpected error:" + str(e))
                continue