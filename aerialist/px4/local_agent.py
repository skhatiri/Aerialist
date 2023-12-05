from . import file_helper
from .get_file_path import check_intermediate_representation
from .get_file_path import get_result_path
from .test_agent import TestAgent
from .drone import Drone
from .simulator import Simulator
from .drone_test import AssertionConfig, DroneTest, DroneTestResult


class LocalAgent(TestAgent):
    def __init__(self, config: DroneTest) -> None:
        super().__init__(config)
        self.simulator = None
        if self.config.simulation is not None:
            self.simulator = Simulator(self.config.simulation)

        self.drone = None
        if self.config.drone is not None:
            self.drone = Drone(self.config.drone)
            if self.config.test is not None:
                self.drone.schedule_test(self.config.test)

    def run(self, config: DroneTest):
        try:
            self.drone.run_scheduled()
            log = self.simulator.get_log()
            if self.config.agent is not None and self.config.agent.path is not None:
                file_helper.upload(log, self.config.agent.path)
                returned_val = check_intermediate_representation()
                result_zip = file_helper.zip_folder(get_result_path())
                file_helper.upload(result_zip, self.config.agent.path)
                if returned_val[0] != "False":
                    zip_list = file_helper.zip_folder(returned_val)
                    if len(zip_list) > 0:
                        for temp_zip_folder in zip_list:
                            file_helper.upload(temp_zip_folder, self.config.agent.path)
            self.results.append(DroneTestResult(log, AssertionConfig.TRAJECTORY))
            return self.results
        except Exception as e:
            self.simulator.kill()
            raise (e)

    def manual(self):
        self.drone.manual()
        self.log = Simulator.get_log()
