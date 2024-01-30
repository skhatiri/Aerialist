from __future__ import annotations
from typing import List, NamedTuple
from shapely.geometry import box, LineString, Point
from shapely import affinity
import matplotlib.patches as mpatches
import logging
# from .position import Position
import munch
from pprint import pprint

logger = logging.getLogger(__name__)


class Obstacle(object):
    class Size(NamedTuple):
        l: float
        w: float
        h: float

    class Position(NamedTuple):
        x: float
        y: float
        z: float = 0
        r: float = 0

    BOX = "box"
    TREE = "tree"
    CENTER_POSITION = True

    def __init__(
        self,
        size: Size,
        position: Position,
        # angle: float = 0,
        shape: str = BOX,
        pattern_design: str = None,
    ) -> None:
        super().__init__()
        print(f'***Checking the input received here {size} {position} {shape}')
        if shape == "BOX":
            if self.CENTER_POSITION:
                rect = box(-size.l / 2, -size.w / 2, size.l / 2, size.w / 2)
            else:
                rect = box(0, 0, size.l, size.w)
            self.geometry = affinity.rotate(rect, position.r, "centroid")
            self.geometry = affinity.translate(self.geometry, position.x, position.y)
            self.unrotated_geometry = affinity.translate(rect, position.x, position.y)
            self.size = size
            self.position = position
            # self.angle = angle
            self.shape = shape
            self.pattern_design = pattern_design
        elif shape == "TREE" or shape == "APARTMENT":
            self.size = size
            self.position = position
            # self.angle = angle
            self.shape = shape
            self.geometry = Point(position.x, position.y)
            self.unrotated_geometry = Point(position.x, position.y)
            self.pattern_design = pattern_design

    def center(self):
        return Obstacle.Position(
            self.geometry.centroid.coords[0][0],
            self.geometry.centroid.coords[0][1],
            self.size.h / 2,
        )

    def anchor_point(self):
        return Obstacle.Position(
            self.unrotated_geometry.bounds[0],
            self.unrotated_geometry.bounds[1],
            0,
        )

    def corner(self):
        return Obstacle.Position(
            self.geometry.bounds[0],
            self.geometry.bounds[1],
            0,
        )

    def to_params(self):
        return [
            self.size.l,
            self.size.w,
            self.size.h,
            self.position.x,
            self.position.y,
            self.position.z,
            self.position.r,
        ]

    def plt_patch(self):
        obst_patch = mpatches.Rectangle(
            (
                self.anchor_point().x,
                self.anchor_point().y,
            ),
            self.size.l,
            self.size.w,
            self.position.r,
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
        # TODO: implement proper 3D distance
        # distance in (x,y) plane
        dist = self.geometry.distance(geometry)
        if dist == 0 and geometry.has_z:
            # it is 3D and they intersect in x,y plane
            intersection = self.geometry.intersection(geometry)
            intersection_points = []
            try:  # single part intersection
                intersection_points = intersection.coords
            except:
                try:  # multipart intersection
                    for geom in intersection.geoms:
                        intersection_points.extend(geom.coords)
                except:
                    pass
            if len(intersection_points) > 0:
                min_z = min(p[2] for p in intersection_points)
                if min_z > self.size.h:
                    # flight trajectory is above the obstacle
                    dist = min_z - self.size.h
        return dist

    def to_dict(self):
        if self.pattern_design is None:
            return {
                "size": {
                    "l": self.size.l,
                    "w": self.size.w,
                    "h": self.size.h
                },
                "position": {
                    "x": self.position.x,
                    "y": self.position.y,
                    "z": self.position.z,
                    "angle": self.position.r,
                },
                "shape": self.shape
            }
        else:
            return {
                "size": {
                    "l": self.size.l,
                    "w": self.size.w,
                    "h": self.size.h
                },
                "position": {
                    "x": self.position.x,
                    "y": self.position.y,
                    "z": self.position.z,
                    "angle": self.position.r,
                },
                "shape": self.shape,
                "pattern_design": self.pattern_design
            }



    @classmethod
    def distance_to_many(cls, obstacles: List[Obstacle], line: LineString):
        boxes = [o.geometry for o in obstacles]
        dist = min([sum([b.distance(Point(*p)) for b in boxes]) for p in line.coords])
        dist_list = [sum([b.distance(Point(*p)) for b in boxes]) for p in line.coords]
        dist_avg = (sum(dist_list)/len(dist_list))
        return dist, dist_list

    @classmethod
    def from_coordinates(cls, coordinates: List[float]):
        size = Obstacle.Size(coordinates[0], coordinates[1], coordinates[2])
        position = Obstacle.Position(
            coordinates[3], coordinates[4], coordinates[5], coordinates[6]
        )
        return Obstacle(size, position)

    @classmethod
    def from_coordinates_multiple(cls, coordinates: List[float]):
        obst = []
        for i in range(0, len(coordinates), 7):
            obst.append(Obstacle.from_coordinates(coordinates[i: i + 7]))
        return obst

    # @classmethod
    # def from_obstacle_def(cls, obstacle: munch.DefaultMunch):
    #     # print(f'Printing the obstacles shape here {obstacle.shape}')
    #     size_object = Obstacle.Size(obstacle.size.l, obstacle.size.w, obstacle.size.h)
    #     # pprint(f'This is the size object--> {vars(size_object)}')
    #     position_object = Obstacle.Position(obstacle.position.x, obstacle.position.y, obstacle.position.z)
    #     # pprint(f'This is the position object--> {vars(position_object)}')
    #     if obstacle.pattern_design is not None:
    #         obstacle_obj = Obstacle(size_object, position_object, obstacle.position.angle, obstacle.shape,
    #                                 obstacle.pattern_design)
    #     else:
    #         obstacle_obj = Obstacle(size_object, position_object, obstacle.position.angle, obstacle.shape)
    #     # pprint(f'The created obstacle object is --> {vars(obstacle_obj)}')
    #     return obstacle_obj

    # @classmethod
    # def from_obstacle_list(cls, obstacle_list: List[munch.DefaultMunch]):
    #     obst = []
    #     for obstacle in obstacle_list:
    #         # print(f"Printing the obstacle here {obstacle}")
    #         obst.append(Obstacle.from_obstacle_def(obstacle))
    #     return obst
    
    @classmethod
    def from_dict(cls, obstacle: dict):
        size = Obstacle.Size(obstacle.size.l, obstacle.size.w, obstacle.size.h)
        position = Obstacle.Position(
            obstacle.position.x,
            obstacle.position.y,
            obstacle.position.z,
            obstacle.position.angle,
        )
        if obstacle.pattern_design is not None:
            obstacle_obj = Obstacle(size, position, obstacle.shape,
                                    obstacle.pattern_design)
        else:
            obstacle_obj = Obstacle(size, position, obstacle.shape)
        
        # pprint(f'****The created obstacle object is --> {vars(obstacle_obj)}')
        
        return obstacle_obj
        

    @classmethod
    def from_dict_multiple(cls, obstacles: List[dict]):
        return [cls.from_dict(obs) for obs in obstacles]
