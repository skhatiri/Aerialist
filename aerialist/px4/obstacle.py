from __future__ import annotations
from typing import List
from shapely.geometry import box, LineString, Point
from shapely import affinity
import matplotlib.patches as mpatches
from decouple import config
import logging
from .position import Position

logger = logging.getLogger(__name__)


class Obstacle(object):
    DIR = config("RESULTS_DIR", default="results/")
    BOX = "box"
    CENTER_POSITION = True

    def __init__(
        self,
        size: Position,
        position: Position,
        angle: float = 0,
        shape: str = BOX,
    ) -> None:
        super().__init__()
        if shape == self.BOX:
            if self.CENTER_POSITION:
                rect = box(-size.x / 2, -size.y / 2, size.x / 2, size.y / 2)
            else:
                rect = box(0, 0, size.x, size.y)
            self.geometry = affinity.rotate(rect, angle, "centroid")
            self.geometry = affinity.translate(self.geometry, position.x, position.y)
            self.unrotated_geometry = affinity.translate(rect, position.x, position.y)
            self.size = size
            self.position = position
            self.angle = angle

    def center(self):
        return Position(
            self.geometry.centroid.coords[0][0],
            self.geometry.centroid.coords[0][1],
            self.size.z / 2,
        )

    def anchor_point(self):
        return Position(
            self.unrotated_geometry.bounds[0],
            self.unrotated_geometry.bounds[1],
            0,
        )

    def corner(self):
        return Position(
            self.geometry.bounds[0],
            self.geometry.bounds[1],
            0,
        )

    def to_params(self):
        return [
            self.size.x,
            self.size.y,
            self.size.z,
            self.position.x,
            self.position.y,
            self.position.z,
            self.angle,
        ]

    def plt_patch(self):
        obst_patch = mpatches.Rectangle(
            (
                self.anchor_point().x,
                self.anchor_point().y,
            ),
            self.size.x,
            self.size.y,
            self.angle,
            rotation_point="center",
            edgecolor="gray",
            facecolor="gray",
            fill=True,
            alpha=0.5,
        )
        return obst_patch

    def intersects(self, other: Obstacle):
        return self.geometry.intersects(other.geometry)

    def distance(self, geometry):
        return self.geometry.distance(geometry)

    @classmethod
    def distance_to_many(cls, obstacles: List[Obstacle], line: LineString):
        boxes = [o.geometry for o in obstacles]
        # for i in range(len(boxes)):
        #     for j in range(i + 1, len(boxes)):
        #         if boxes[i].intersects(boxes[j]):
        #             return 1000
        # This checks the distance of drone from the object all the time...
        dist = min([sum([b.distance(Point(*p)) for b in boxes]) for p in line.coords])
        return dist

    @classmethod
    def from_coordinates(cls, coordinates: List[float]):
        size = Position(coordinates[0], coordinates[1], coordinates[2])
        position = Position(coordinates[3], coordinates[4], coordinates[5])
        angle = coordinates[6]
        return Obstacle(size, position, angle)

    @classmethod
    def from_coordinates_multiple(cls, coordinates: List[float]):
        obst = []
        for i in range(0, len(coordinates), 7):
            obst.append(Obstacle.from_coordinates(coordinates[i : i + 7]))
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
