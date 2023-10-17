from __future__ import annotations
from typing import List
from shapely.geometry import box, LineString, Point
from shapely import affinity
import matplotlib.patches as mpatches
from decouple import config
import logging
from .position import Position
import munch
from pprint import pprint

logger = logging.getLogger(__name__)


class Obstacle(object):
    DIR = config("RESULTS_DIR", default="results/")
    BOX = "box"
    TREE = "tree"
    CENTER_POSITION = True

    def __init__(
            self,
            size: Position,
            position: Position,
            angle: float = 0,
            shape: str = BOX,
            pattern_design: str = None,
    ) -> None:
        super().__init__()
        # print(f'***Checking the input received here {size} {position} {angle} {shape}')
        if shape == "BOX":
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
            self.shape = shape
            self.pattern_design = pattern_design
        elif shape == "TREE" or shape == "APARTMENT":
            self.size = size
            self.position = position
            self.angle = angle
            self.shape = shape
            self.geometry = Point(position.x, position.y)
            self.unrotated_geometry = Point(position.x, position.y)

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

    def plt_patch_circle(self, radius, color):
        obst_patch = mpatches.Circle((self.position.x, self.position.y), radius, color=color, alpha=0.5)
        return obst_patch

    def plt_patch_triangle(self):
        obst_patch = mpatches
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
            obst.append(Obstacle.from_coordinates(coordinates[i: i + 7]))
        return obst

    @classmethod
    def from_obstacle_def(cls, obstacle: munch.DefaultMunch):
        # print(f'Printing the obstacles shape here {obstacle.shape}')
        size_object = Position(obstacle.size.l, obstacle.size.w, obstacle.size.h)
        # pprint(f'This is the size object--> {vars(size_object)}')
        position_object = Position(obstacle.position.x, obstacle.position.y, obstacle.position.z)
        # pprint(f'This is the position object--> {vars(position_object)}')
        if obstacle.pattern_design is not None:
            obstacle_obj = Obstacle(size_object, position_object, obstacle.position.angle, obstacle.shape,
                                    obstacle.pattern_design)
        else:
            obstacle_obj = Obstacle(size_object, position_object, obstacle.position.angle, obstacle.shape)
        # pprint(f'The created obstacle object is --> {vars(obstacle_obj)}')
        return obstacle_obj

    @classmethod
    def from_obstacle_list(cls, obstacle_list: List[munch.DefaultMunch]):
        obst = []
        for obstacle in obstacle_list:
            # print(f"Printing the obstacle here {obstacle}")
            obst.append(Obstacle.from_obstacle_def(obstacle))
        return obst
