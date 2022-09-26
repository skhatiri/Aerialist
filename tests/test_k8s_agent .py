import unittest
import os
import logging
from aerialist.px4.drone_test import (
    AssertionConfig,
    DroneConfig,
    DroneTest,
    AgentConfig,
    SimulationConfig,
    TestConfig,
)
from aerialist.px4.k8s_agent import K8sAgent
from aerialist.px4.local_agent import LocalAgent


class TestK8sAgent(unittest.TestCase):
    def setUp(self) -> None:
        os.makedirs("logs/", exist_ok=True)
        logging.basicConfig(
            level=logging.DEBUG,
            filename="logs/root.txt",
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )
        return super().setUp()

    def test_agent(self):
        simulation_config = SimulationConfig(
            headless=True, simulator=SimulationConfig.GAZEBO
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
        runner = AgentConfig(
            engine="k8s",
            count=2,
            path="https://filer.cloudlab.zhaw.ch/remote.php/webdav/resources/logs/",
        )
        test = DroneTest(
            drone_config, simulation_config, test_config, assertion_config, runner
        )
        agent = K8sAgent(test)
        agent.run(test)

    def test_runner(self):
        simulation_config = SimulationConfig(
            headless=True, simulator=SimulationConfig.GAZEBO
        )
        drone_config = DroneConfig(
            port=DroneConfig.SITL_PORT,
            params={},
            mission_file=None,
        )
        test_config = TestConfig(
            "https://filer.cloudlab.zhaw.ch/remote.php/webdav/resources/logs/t0.ulg"
        )
        assertion_config = AssertionConfig(
            "https://filer.cloudlab.zhaw.ch/remote.php/webdav/resources/logs/t0.ulg",
            variable=AssertionConfig.TRAJECTORY,
        )
        runner = AgentConfig(
            engine="local",
            count=1,
            path="https://filer.cloudlab.zhaw.ch/remote.php/webdav/tests/",
        )
        test = DroneTest(
            drone_config, simulation_config, test_config, assertion_config, runner
        )
        agent = LocalAgent(test)
        agent.run(test)
