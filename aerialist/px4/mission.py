from __future__ import annotations
from typing import List, NamedTuple
import logging

logger = logging.getLogger(__name__)


class Mission(object):
    class Position(NamedTuple):
        x: float
        y: float
        z: float = 0
        r: float = 0

    def __init__(self, waypoints: List[Position]) -> None:
        super().__init__()
        self.waypoints = waypoints

    def to_dict(self):
        return {"waypoints": [wp._asdict() for wp in self.waypoints]}

    @classmethod
    def from_dict(cls, mission: dict):
        waypoints: List[cls.Position] = []
        for wp in mission.waypoints:
            waypoints.append(
                cls.Position(
                    wp.x,
                    wp.y,
                    wp.z,
                    wp.r,
                )
            )
        return cls(waypoints)
