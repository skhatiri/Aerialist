from __future__ import annotations
import math
from statistics import mean
from typing import List


class Position(object):
    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        r: float = None,
        timestamp: int = None,
        is_jmavsim: bool = False,
    ) -> None:
        super().__init__()
        self.timestamp = timestamp
        if not is_jmavsim:
            self.x = x
            self.y = y
            self.z = z
        else:
            self.x = -y
            self.y = x
            self.z = z

        self.r = r

    def get_position_in_relative_distance(self, distance: float):
        delta_x = distance * math.cos(self.r)
        delta_y = distance * math.sin(self.r)
        return Position(self.x + delta_x, self.y + delta_y, self.z, self.r)

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
            timestamp=mean([p.timestamp for p in positions]),
        )
