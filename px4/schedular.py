import sched, time
from typing import List
from px4.command import Command, FlightMode
from px4.drone import Drone
import logging

logger = logging.getLogger(__name__)


class Schedular(object):
    def __init__(
        self, drone: Drone, commands: List[Command], speed: float = 1, offset_sync=False
    ) -> None:
        """sets an schedular object with input control commands to be replayed accurately"""
        super().__init__()
        self.drone = drone
        self.scheduler = sched.scheduler(time.time, time.sleep)
        armed = False
        timeoffset = 0
        if offset_sync:
            timeoffset = min([c.timestamp for c in commands])

        for c in commands:
            # add delay to the first start position/altitude control commands
            # they are actually logged before their actual commands are sent
            # the current added delay corresponds to the script for manual flight in drone.py
            if c.mode == FlightMode.Arm:
                armed = True
            if not armed and (
                c.mode == FlightMode.Altitude or c.mode == FlightMode.Position
            ):
                self.scheduler.enter(
                    float(c.timestamp - timeoffset) / (1000000 * speed)
                    + 6 * drone.SETPOINT_PERIOD,
                    1,
                    self.drone.run,
                    (c,),
                )

            else:
                self.scheduler.enter(
                    float(c.timestamp - timeoffset) / (1000000 * speed),
                    1,
                    self.drone.run,
                    (c,),
                )

    def run(self) -> None:
        logger.info(
            f"**SCHEDULER STARTED**\nrunning {len(self.scheduler.queue)} commands "
        )
        self.scheduler.run()
