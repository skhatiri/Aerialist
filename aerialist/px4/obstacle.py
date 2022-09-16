from __future__ import annotations
from typing import List
from shapely.geometry import box, LineString, Point
from decouple import config
import logging
from .position import Position

logger = logging.getLogger(__name__)


class Obstacle(object):
    DIR = config("RESULTS_DIR")
    BOX = "box"

    def __init__(
        self,
        p1: Position,
        p2: Position,
        shape: str = BOX,
        # positions: List[Position],
    ) -> None:
        super().__init__()
        # if positions != None:
        #     self.positions = positions
        #     self.location, self.size = self.estimate_box_pose(positions, size)
        # else:
        self.p1 = p1
        self.p2 = p2

    def center(self):
        return Position(
            (self.p1.x + self.p2.x) / 2.0,
            (self.p1.y + self.p2.y) / 2.0,
            (self.p1.z + self.p2.z) / 2.0,
        )

    def min(self):
        return Position(
            min(self.p1.x, self.p2.x),
            min(self.p1.y, self.p2.y),
            min(self.p1.z, self.p2.z),
        )

    def max(self):
        return Position(
            max(self.p1.x, self.p2.x),
            max(self.p1.y, self.p2.y),
            max(self.p1.z, self.p2.z),
        )

    def size(self):
        return Position(
            abs(self.p2.x - self.p1.x),
            abs(self.p2.y - self.p1.y),
            abs(self.p2.z - self.p1.z),
        )

    def to_params(self):
        return [
            self.p1.x,
            self.p1.y,
            self.p1.z,
            self.p2.x,
            self.p2.y,
            self.p2.z,
        ]

    def to_box(self):
        min = self.min()
        max = self.max()
        rect = box(min.x, min.y, max.x, max.y)
        return rect

    def intersects(self, other: Obstacle):
        return self.to_box().intersects(other.to_box())

    def distance(self, geometry):
        return self.to_box().distance(geometry)

    @classmethod
    def distance_to_many(cls, obstacles: List[Obstacle], line: LineString):
        boxes = [o.to_box() for o in obstacles]
        # for i in range(len(boxes)):
        #     for j in range(i + 1, len(boxes)):
        #         if boxes[i].intersects(boxes[j]):
        #             return 1000
        dist = min([sum([b.distance(Point(*p)) for b in boxes]) for p in line.coords])
        return dist

    @classmethod
    def from_coordinates(cls, coordinates: List[float]):
        p1 = Position(coordinates[0], coordinates[1], coordinates[2])
        p2 = Position(coordinates[3], coordinates[4], coordinates[5])
        return Obstacle(p1, p2)

    @classmethod
    def from_coordinates_multiple(cls, coordinates: List[float]):
        obst = []
        for i in range(0, len(coordinates), 6):
            obst.append(Obstacle.from_coordinates(coordinates[i : i + 6]))
        return obst

    # def estimate_box_pose(
    #     self, positions: List[Position], size: Position = None
    # ) -> Tuple[Position, Position]:
    #     min_pos = Position(
    #         min(p.x for p in positions),
    #         min(p.y for p in positions),
    #         min(abs(p.z) for p in positions),
    #     )
    #     max_pos = Position(
    #         max(p.x for p in positions),
    #         max(p.y for p in positions),
    #         max(abs(p.z) for p in positions),
    #     )
    #     if size == None:
    #         size = (max_pos.x - min_pos.x, max_pos.y - min_pos.y, max_pos.z)

    #     location = Position(
    #         (max_pos.x + min_pos.x) / 2,
    #         (max_pos.y + min_pos.y) / 2,
    #         size[2] / 2,
    #     )

    #     logger.info(location, size)
    #     return location, size

    # @classmethod
    # def extract_from_log(cls, log_address: str, size: Position = None):
    #     """extracts and returns detected obstacle borders from the input log"""

    #     avoidance_active_periods = Trajectory.extract_CP_active_periods(log_address)

    #     points: List[Position] = []
    #     idx = 0

    #     obstacle_distance = ulog_helper.extract(log_address, "obstacle_distance_fused")
    #     # dropping duplicate rows (some rows are repated in the logs with the same timestamps)
    #     obstacle_distance.drop_duplicates("timestamp", inplace=True)
    #     for i, row in obstacle_distance.iterrows():
    #         tmstmp = int(row["timestamp"])
    #         if tmstmp < avoidance_active_periods[idx][0]:
    #             continue
    #         elif tmstmp <= avoidance_active_periods[idx][1]:
    #             p = Position(
    #                 None,
    #                 None,
    #                 None,
    #                 timestamp=row.timestamp,
    #             )
    #             p.distance = (
    #                 float(row["distances[0]"]) / 100
    #             )  # convert to meters (originally cm)
    #             points.append(p)

    #         else:
    #             idx += 1
    #             if idx >= len(avoidance_active_periods):
    #                 break
    #         # logger.info (points)

    #     trajectory = Trajectory.extract_from_log(log_address)
    #     trj_times = [p.timestamp for p in trajectory.positions]
    #     for p in points:
    #         # find nearest data in local positions
    #         idx = timeserie_helper.find_nearest_index(trj_times, p.timestamp)

    #         # updating position based on yaw angle
    #         obstacle_point = trajectory.positions[
    #             idx
    #         ].get_position_in_relative_distance(p.distance)

    #         p.x, p.y, p.z, p.r = (
    #             obstacle_point.x,
    #             obstacle_point.y,
    #             obstacle_point.z,
    #             obstacle_point.r,
    #         )

    #     logger.info(points)
    #     return cls(points, size=size)
