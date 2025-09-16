from . import file_helper
from .test_agent import TestAgent
from .drone import Drone
from .simulator import Simulator
from .aerialist_test import AssertionConfig, AerialistTest, AerialistTestResult
from . import tools


class LocalAgent(TestAgent):
    def __init__(self, config: AerialistTest) -> None:
        super().__init__(config)
        self.simulator = None
        if self.config.simulation is not None:
            self.simulator = Simulator(self.config.simulation)

        self.drone = None
        if self.config.robot is not None:
            self.drone = Drone(self.config.robot)
            if self.config.mission is not None:
                self.drone.schedule_test(self.config.mission)

    def run(self):
        try:
            if self.drone is not None:
                self.drone.run_scheduled()
            log = self.simulator.get_log()
            if self.config.agent is not None and self.config.agent.path is not None:
                file_helper.upload(log, self.config.agent.path)
            self.results.append(AerialistTestResult(log, AssertionConfig.TRAJECTORY))
            return self.results
        except Exception as e:
            self.simulator.kill()
            raise (e)

    def manual(self):
        tools.manual_flight(self.drone)
        self.log = Simulator.get_log()
