from statistics import median
from typing import List
from .trajectory import Trajectory
from .drone_test import DroneTest, DroneTestResult


class TestAgent(object):
    def __init__(self, config: DroneTest) -> None:
        self.config = config
        self.results: List[DroneTestResult] = []

    def Run(self, config: DroneTest):
        pass

    def Plot(self):
        if self.results is not None and len(self.results) >= 1:
            Trajectory.plot_multiple(
                [r.record for r in self.results],
                self.config.assertion.expectation,
                distance=None
                if self.config.assertion.expectation is None
                else median(
                    [
                        r.record.distance(self.config.assertion.expectation)
                        for r in self.results
                    ]
                ),
                obstacles=self.config.simulation.obstacles,
            )
