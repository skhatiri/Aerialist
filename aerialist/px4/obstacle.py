from __future__ import annotations
from typing import List, NamedTuple
from shapely.geometry import box, LineString, Point
from shapely import affinity
import matplotlib.patches as mpatches
import logging

logger = logging.getLogger(__name__)


class Obstacle(object):
    class Size(NamedTuple):
        l: float
        w: float
        h: float
        r: float = 0

    class Position(NamedTuple):
        x: float
        y: float
        z: float = 0
        r: float = 0

    BOX = "box"
    CENTER_POSITION = True
    CYLINDER = "cylinder"

    def __init__(
        self,
        size: Size,
        position: Position,
        shape: str = BOX,
    ) -> None:
        super().__init__()

        if shape is None:
            shape = self.BOX
        self.size = size
        self.position = position
        self.shape = shape

        if shape == self.BOX:
            if self.CENTER_POSITION:
                rect = box(-size.l / 2, -size.w / 2, size.l / 2, size.w / 2)
            else:
                rect = box(0, 0, size.l, size.w)
            self.geometry = affinity.rotate(rect, position.r, "centroid")
            self.geometry = affinity.translate(self.geometry, position.x, position.y)
            self.unrotated_geometry = affinity.translate(rect, position.x, position.y)
        if shape == self.CYLINDER:
            center = Point(position.x, position.y)
            self.geometry = center.buffer(self.size.r)
            self.unrotated_geometry = self.geometry

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
        if self.shape == self.BOX:
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
        elif self.shape == self.CYLINDER:
            obst_patch = mpatches.Circle(
                (
                    self.position.x,
                    self.position.y,
                ),
                self.size.r,
                edgecolor="gray",
                facecolor="gray",
                fill=True,
                alpha=0.5,
            )
            return obst_patch
        else:
            return None

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
        return {
            "shape": self.shape,
            "size": self.size._asdict(),
            "position": self.position._asdict(),
        }

    @classmethod
    def distance_to_many(cls, obstacles: List[Obstacle], line: LineString):
        boxes = [o.geometry for o in obstacles]
        dist = min([sum([b.distance(Point(*p)) for b in boxes]) for p in line.coords])
        return dist

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
            obst.append(Obstacle.from_coordinates(coordinates[i : i + 7]))
        return obst

    @classmethod
    def from_dict(cls, obstacle: dict):
        size = Obstacle.Size(**obstacle.size)
        position = Obstacle.Position(**obstacle.position)
        return Obstacle(size, position, obstacle.get("shape"))

    @classmethod
    def from_dict_multiple(cls, obstacles: List[dict]):
        return [cls.from_dict(obs) for obs in obstacles]
