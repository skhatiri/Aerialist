from typing import List
from .drone_test import DroneTest, DroneTestResult


class TestAgent(object):
    def __init__(self, config: DroneTest) -> None:
        self.config = config
        self.results: List[DroneTestResult] = []

    def run(self) -> DroneTestResult:
        pass
