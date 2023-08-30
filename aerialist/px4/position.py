from __future__ import annotations
import math
from statistics import mean
from typing import List
from shapely.geometry import Point


class Position(object):
    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        r: float = None,
        timestamp: int = None,
    ) -> None:
        super().__init__()
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.r = r

    def convert_jmavsim(self):
        jmav_x = self.x
        jmav_y = self.y
        self.x = -jmav_y
        self.y = jmav_x

    def get_position_in_relative_distance(self, distance: float):
        delta_x = distance * math.cos(self.r)
        delta_y = distance * math.sin(self.r)
        return Position(self.x + delta_x, self.y + delta_y, self.z, self.r)

    def to_geometry(self):
        return Point(self.x, self.y)

    def to_dict(self):
        return {
            "timestamp": self.timestamp,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "r": self.r,
        }

    def __repr__(self):
        return str(self)

    def __str__(self) -> str:
        return f"{self.timestamp}\t ({self.x},{self.y},{self.z})\t{self.r}\n"

    @classmethod
    def average(cls, positions: List[Position]):
        return Position(
            x=mean([p.x for p in positions]),
            y=mean([p.y for p in positions]),
            z=mean([p.z for p in positions]),
            r=mean([p.r for p in positions]),
            timestamp=mean([p.timestamp for p in positions]),
        )
