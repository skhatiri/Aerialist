import unittest
import os
import logging
from aerialist.px4.drone import Drone
from aerialist.px4.drone_test import (
    AssertionConfig,
    DroneConfig,
    DroneTest,
    SimulationConfig,
    TestConfig,
)
from aerialist.px4.local_agent import LocalAgent
from aerialist.px4.simulator import Simulator


class TestLocalAgent(unittest.TestCase):
    def setUp(self) -> None:
        os.makedirs("logs/", exist_ok=True)
        logging.basicConfig(
            level=logging.DEBUG,
            filename="logs/root.txt",
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )
        return super().setUp()

    def test_simulator(self):
        simulation_config = SimulationConfig(
            headless=False, simulator=SimulationConfig.GAZEBO
        )
        simulator = Simulator(simulation_config)
        log = simulator.get_log()

    def test_drone(self):
        """needs the simulator to be already running"""
        drone_config = DroneConfig(
            port=DroneConfig.SITL_PORT,
            params={},
            mission_file=None,
        )
        test_config = TestConfig(commands="aerialist/resources/logs/t0.ulg")
        drone = Drone(drone_config)
        drone.schedule_test(test_config)
        drone.run_scheduled()

    def test_agent(self):
        simulation_config = SimulationConfig(
            headless=False, simulator=SimulationConfig.GAZEBO
        )
        drone_config = DroneConfig(
            port=DroneConfig.SITL_PORT,
            params={},
            mission_file=None,
        )
        test_config = TestConfig("aerialist/resources/logs/t0.ulg")
        assertion_config = AssertionConfig(
            "aerialist/resources/logs/t0.ulg", variable=AssertionConfig.TRAJECTORY
        )
        test = DroneTest(drone_config, simulation_config, test_config, assertion_config)
        agent = LocalAgent(test)
        agent.run(test)


if __name__ == "__main__":
    test = TestLocalAgent()
    test.test_simulator()
