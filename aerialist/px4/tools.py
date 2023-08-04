from typing import List, Tuple
import numpy as np
import ruptures as rpt
import logging
from mavsdk import asyncio
import keyboard

from . import timeserie_helper, file_helper
from .command import Command, FlightMode, DefaultCommands
from .drone import Drone
from .position import Position
from .trajectory import Trajectory

logger = logging.getLogger(__name__)


### Command:
def extract_command_segments(commands: List[Command]) -> List[List[Command]]:
    """extract segments of manual commadns"""
    data = np.array([[c.x, c.y, c.z, c.r] for c in commands])
    alg = rpt.Pelt(model="rbf").fit(data)
    change_idx = alg.predict(pen=2)
    auto_idx = [i for i, c in enumerate(commands) if c.mode != FlightMode.Setpoint]
    auto_idx += [i + 1 for i in auto_idx if i < len(commands) - 1]
    split_idx = list(set(change_idx + auto_idx + [0]))
    split_idx.sort()
    segments: List[Command] = []
    for i in range(len(split_idx) - 1):
        seg = commands[split_idx[i] : split_idx[i + 1]]
        segments.append(seg)

    return segments


#### Drone:
def manual_flight(drone: Drone):
    """manually send RC commands using the keyoard keys to the drone"""
    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_flight_async(drone))


#### Drone:
async def manual_flight_async(drone: Drone):
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
                await drone.run_long_async(
                    DefaultCommands.Hover, 3.1 * drone.SETPOINT_PERIOD
                )
                await drone.run_async(DefaultCommands.Arm)
                logger.info("Armed")
                await drone.run_long_async(
                    DefaultCommands.Hover, 3.1 * drone.SETPOINT_PERIOD
                )
                await drone.run_async(DefaultCommands.Position)
                logger.info("in position mode")
                in_setpoint_mode = True
                continue
            if keyboard.is_pressed("enter"):
                await drone.run_async(DefaultCommands.Disarm)
                in_setpoint_mode = False
                break

            if keyboard.is_pressed("t"):
                await drone.run_async(DefaultCommands.Takeoff)
                in_setpoint_mode = False
            if keyboard.is_pressed("l"):
                await drone.run_async(DefaultCommands.Land)
                in_setpoint_mode = False

            if keyboard.is_pressed("p"):
                await drone.run_async(DefaultCommands.Position)
                in_setpoint_mode = True

            if keyboard.is_pressed("m"):
                await drone.run_async(DefaultCommands.Manual)
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
                await drone.run_async(command)

            await asyncio.sleep(drone.SETPOINT_PERIOD)

        except Exception as e:
            logger.error("Unexpected error:" + str(e))
            continue


### Obstacle:
def estimate_box_pose(
    positions: List[Position], size: Position = None
) -> Tuple[Position, Position]:
    min_pos = Position(
        min(p.x for p in positions),
        min(p.y for p in positions),
        min(abs(p.z) for p in positions),
    )
    max_pos = Position(
        max(p.x for p in positions),
        max(p.y for p in positions),
        max(abs(p.z) for p in positions),
    )
    if size == None:
        size = (max_pos.x - min_pos.x, max_pos.y - min_pos.y, max_pos.z)

    location = Position(
        (max_pos.x + min_pos.x) / 2,
        (max_pos.y + min_pos.y) / 2,
        size[2] / 2,
    )

    logger.info(location, size)
    return location, size


### Obstacle:
def extract_obstacle_from_log(log_address: str, size: Position = None):
    """extracts and returns detected obstacle borders from the input log"""

    avoidance_active_periods = extract_collission_prevension_active_periods(log_address)

    points: List[Position] = []
    idx = 0

    obstacle_distance = file_helper.extract(log_address, "obstacle_distance_fused")
    # dropping duplicate rows (some rows are repated in the logs with the same timestamps)
    obstacle_distance.drop_duplicates("timestamp", inplace=True)
    for i, row in obstacle_distance.iterrows():
        tmstmp = int(row["timestamp"])
        if tmstmp < avoidance_active_periods[idx][0]:
            continue
        elif tmstmp <= avoidance_active_periods[idx][1]:
            p = Position(
                None,
                None,
                None,
                timestamp=row.timestamp,
            )
            p.distance = (
                float(row["distances[0]"]) / 100
            )  # convert to meters (originally cm)
            points.append(p)

        else:
            idx += 1
            if idx >= len(avoidance_active_periods):
                break
        # logger.info (points)

    trajectory = Trajectory.extract_from_log(log_address)
    trj_times = [p.timestamp for p in trajectory.positions]
    for p in points:
        # find nearest data in local positions
        idx = timeserie_helper.find_nearest_index(trj_times, p.timestamp)

        # updating position based on yaw angle
        obstacle_point = trajectory.positions[idx].get_position_in_relative_distance(
            p.distance
        )

        p.x, p.y, p.z, p.r = (
            obstacle_point.x,
            obstacle_point.y,
            obstacle_point.z,
            obstacle_point.r,
        )

    logger.info(points)
    return estimate_box_pose(points, size=size)


### Trajectory
def extract_collission_prevension_active_periods(log_address):
    collision_constraints = file_helper.extract(log_address, "collision_constraints")
    if collision_constraints is None:
        return []
    avoidance_active_periods = []
    period_start = -1
    for ind, row in collision_constraints.iterrows():
        # skip the rows where CP did not change setpoints (where in-active)
        if (
            row["original_setpoint[0]"] != row["adapted_setpoint[0]"]
            or row["original_setpoint[1]"] != row["adapted_setpoint[1]"]
        ):
            if period_start < 0:
                period_start = int(row["timestamp"])
                avoidance_active_periods.append((period_start, int(row["timestamp"])))
            else:
                avoidance_active_periods[-1] = (
                    period_start,
                    int(row["timestamp"]),
                )
        else:
            period_start = -1

    return avoidance_active_periods
