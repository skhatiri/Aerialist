from typing import List
from .aerialist_test import AerialistTest, AerialistTestResult


class TestAgent(object):
    def __init__(self, config: AerialistTest) -> None:
        self.config = config
        self.results: List[AerialistTestResult] = []

    def run(self) -> AerialistTestResult:
        pass
