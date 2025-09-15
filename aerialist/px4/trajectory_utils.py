import math
from typing import List
import numpy as np
import ruptures as rpt

from .position import Position
from .trajectory import Trajectory


def quat2eulers(cls, q0: float, q1: float, q2: float, q3: float) -> tuple:
    """
    Compute yaw-pitch-roll Euler angles from a quaternion.

    Args
    ----
        q0 (w): Scalar component of quaternion. ()
        q1 (x), q2 (y), q3(z): Vector components of quaternion.

    Returns
    -------
        (roll, pitch, yaw) (tuple): 321 Euler angles in radians
    """
    roll = math.atan2(
        2 * ((q2 * q3) + (q0 * q1)), q0**2 - q1**2 - q2**2 + q3**2
    )  # radians
    pitch = math.asin(2 * ((q1 * q3) - (q0 * q2)))
    yaw = math.atan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2)
    return (roll, pitch, yaw)


def is_stuck(trj: Trajectory) -> bool:
    """check if the robot is stuck (not moving) in the last seconds"""
    last_pose_data = trj.to_data_frame()
    if len(last_pose_data) > 50:
        last_pose_data = last_pose_data[-50:, :]  # last 10 seconds
    std_x = last_pose_data[:, 1].std()
    std_y = last_pose_data[:, 2].std()
    std_sum = std_x + std_y
    if std_sum < 0.01:
        return True
    return False


def trim(positions: List[Position]):
    """trim the trajectory to remove the initial section before moving the robot to its initial position"""
    data = np.array([[c.x, c.y, c.z, c.r] for c in positions])
    alg = rpt.Pelt(model="rbf").fit(data)
    points = alg.predict(pen=0.01)
    print(points)
    cut_idx = points[1]
    positions = positions[cut_idx:]
    return positions
