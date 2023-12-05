from __future__ import annotations
from statistics import mean
import pandas as pd
from enum import Enum
from typing import List
from . import file_helper


class FlightMode(Enum):
    # from PX4/src/modules/commander/commander_params.c
    Unassigned = -1
    Manual = 0
    Altitude = 1
    Position = 2
    Mission = 3
    Hold = 4
    Takeoff = 10
    Land = 11
    Return = 5
    Acro = 6  # not available in MavSDK
    Offboard = 7
    Stabilized = 8
    FollowMe = 12
    # introduced by us
    Arm = 20
    Disarm = 21
    Setpoint = 100


class Command(object):
    """log attributes for the contol commands to the drone"""

    def __init__(
        self, timestamp=0, x=0, y=0, z=0.5, r=0, mode: FlightMode = FlightMode.Setpoint
    ) -> None:
        super().__init__()
        self.timestamp = timestamp  # in microseconds since the start of logging
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        if type(mode) == FlightMode:
            self.mode = mode
        else:
            self.mode = FlightMode(mode)

    def __str__(self) -> str:
        return f"{int(self.timestamp)}\t{self.mode.name}\t({self.x},{self.y},{self.z},{self.r})\n"

    def __repr__(self) -> str:
        return str(self)

    def to_dict(self):
        return {
            "timestamp": self.timestamp,
            "mode": self.mode.value,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "r": self.r,
        }

    def project(self, x, y, z, r) -> Command:
        if self.mode == FlightMode.Setpoint:
            proj = Command(
                self.timestamp,
                min(1, max(-1, self.x * x)),
                min(1, max(-1, self.y * y)),
                0.5 + min(0.5, max(-0.5, (self.z - 0.5) * z)),
                min(1, max(-1, self.r * r)),
            )
            return proj
        else:
            return self

    @classmethod
    def extract_params_from_csv(cls, address: str) -> dict:
        """extracts and returns parameters"""

        params_csv = pd.read_csv(
            address, names=["name", "value"], dtype={"name": str, "value": str}
        )
        params = {}
        for row in params_csv.itertuples():
            try:
                value = int(row.value)
            except:
                value = float(row.value)
            params[row.name] = value
        return params

    @classmethod
    def save_csv(cls, commands: List[Command], address: str) -> None:
        """saves RC commands to file"""
        data_frame = pd.DataFrame.from_records([c.to_dict() for c in commands])
        data_frame.to_csv(address, index=False)

    @classmethod
    def average(cls, commands: List[Command]):
        return Command(
            mean([c.timestamp for c in commands]),
            mean([c.x for c in commands]),
            mean([c.y for c in commands]),
            mean([c.z for c in commands]),
            mean([c.r for c in commands]),
        )

    @classmethod
    def extract(cls, address: str) -> List[Command]:
        if address.endswith(".csv"):
            return cls.extract_from_csv(address)
        if address.endswith(".ulg"):
            return cls.extract_from_log(address)
        return None

    @classmethod
    def extract_from_csv(cls, address: str) -> List[Command]:
        """extracts and returns RC commands from the saved log"""
        commands = []

        commands_csv = pd.read_csv(address)
        for row in commands_csv.itertuples():
            commands.append(
                cls(
                    row.timestamp,
                    row.x,
                    row.y,
                    row.z,
                    row.r,
                    row.mode,
                )
            )
        return commands

    @classmethod
    def extract_from_log(cls, address: str) -> List[Command]:
        """extracts and returns RC commands from the input log"""

        manual_contorl = file_helper.extract(address, "manual_control_setpoint")

        commands = []

        # manual (remote control) set points
        for row in manual_contorl.itertuples():
            commands.append(cls(row.timestamp, row.x, row.y, row.z, row.r))

        # arm/disarm
        actuator_armed = file_helper.extract(address, "actuator_armed")
        arm_state = 0
        for row in actuator_armed.itertuples():
            if row.armed == arm_state:
                continue
            else:
                arm_state = row.armed
                if arm_state == 1:
                    commands.append(cls(row.timestamp, mode=FlightMode.Arm))
                else:
                    commands.append(cls(row.timestamp, mode=FlightMode.Disarm))

        # flight modes
        commander_state = file_helper.extract(address, "commander_state")
        mode = -1
        for row in commander_state.itertuples():
            if row.main_state_changes == 0:
                # skip invalid states before the first state change
                continue
            current_mode = row.main_state
            if current_mode == mode:
                continue
            else:
                commands.append(cls(row.timestamp, mode=FlightMode(current_mode)))
                mode = current_mode

        commands.sort(key=lambda x: x.timestamp)
        return commands

    def extract_params_from_log(cls, log_address: str) -> List[Command]:
        """extracts and returns RC commands from the input log"""
        pass


# Predifined Commands
class DefaultCommands(object):
    Hover = Command(0, 0, 0, 0.5, 0)
    Up = Command(0, 0, 0, 1, 0)
    Down = Command(0, 0, 0, 0, 0)
    Fornt = Command(0, 1, 0, 0.5, 0)
    Back = Command(0, -1, 0, 0.5, 0)
    Right = Command(0, 0, 1, 0.5, 0)
    Left = Command(0, 0, -1, 0.5, 0)
    Spin_right = Command(0, 0, 0, 0.5, 1)
    Spin_left = Command(0, 0, 0, 0.5, -1)

    Arm = Command(mode=FlightMode.Arm)
    Disarm = Command(mode=FlightMode.Disarm)
    Takeoff = Command(mode=FlightMode.Takeoff)
    Land = Command(mode=FlightMode.Land)
    Position = Command(mode=FlightMode.Position)
    Altitude = Command(mode=FlightMode.Altitude)
    Manual = Command(mode=FlightMode.Manual)
    Hold = Command(mode=FlightMode.Hold)
