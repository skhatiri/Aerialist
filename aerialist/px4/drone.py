import logging
import time
import sched
from mavsdk import System, action, asyncio
from decouple import config
from .command import Command, FlightMode, DefaultCommands
from .drone_test import DroneConfig, TestConfig

logger = logging.getLogger(__name__)


class Drone(object):
    """sets up and maintains the conection to the drone"""

    SETPOINT_PERIOD = 0.2
    DEFAULT_PARAMS = config("PARAMS", default=None)

    def __init__(self, config: DroneConfig) -> None:
        super().__init__()
        self.config = config
        self.address = f"udp://:{self.config.port}"
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.connect_async(self.address))

        if self.DEFAULT_PARAMS != None:
            default_params = Command.extract_params_from_csv(self.DEFAULT_PARAMS)
            self.set_params(default_params)
        if self.config.params != None:
            self.set_params(self.config.params)

        if self.config.mission_file != None:
            self.upload_mission(self.config.mission_file)

    def schedule_test(self, test: TestConfig, offset_sync=False) -> None:
        """sets an schedular object with input control commands to be replayed accurately"""
        self.scheduler = sched.scheduler(time.time, time.sleep)
        if test.commands is None:
            return
        armed = False
        timeoffset = 0
        if offset_sync:
            timeoffset = min([c.timestamp for c in test.commands])
        for c in test.commands:
            # add delay to the first start position/altitude control commands
            # they are actually logged before their actual commands are sent
            # the current added delay corresponds to the script for manual flight in drone.py
            if c.mode == FlightMode.Arm:
                armed = True
            if not armed and (
                c.mode == FlightMode.Altitude or c.mode == FlightMode.Position
            ):
                self.scheduler.enter(
                    float(c.timestamp - timeoffset) / (1000000 * test.speed)
                    + 6 * self.SETPOINT_PERIOD,
                    1,
                    self.run,
                    (c,),
                )

            else:
                self.scheduler.enter(
                    float(c.timestamp - timeoffset) / (1000000 * test.speed),
                    1,
                    self.run,
                    (c,),
                )

    def run_scheduled(self) -> None:
        logger.info(
            f"**SCHEDULER STARTED** running {len(self.scheduler.queue)} commands "
        )
        self.scheduler.run()

    def start_mission(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.start_mission_async())

    async def start_mission_async(self):
        logger.info("starting mission")
        try:
            await self.drone.action.arm()
            await self.drone.mission_raw.start_mission()
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
            elif command.mode == FlightMode.Mission:
                await self.start_mission_async()
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
