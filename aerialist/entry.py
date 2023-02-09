#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import os
import sys
from decouple import config
from config_yaml_file import reform_args

try:
    from .px4.k8s_agent import K8sAgent
    from .px4.local_agent import LocalAgent
    from .px4.docker_agent import DockerAgent
    from .px4.drone_test import (
        AssertionConfig,
        DroneConfig,
        DroneTest,
        SimulationConfig,
        TestConfig,
        AgentConfig,
        Plot,
    )
except:
    from px4.k8s_agent import K8sAgent
    from px4.local_agent import LocalAgent
    from px4.docker_agent import DockerAgent
    from px4.drone_test import (
        AssertionConfig,
        DroneConfig,
        DroneTest,
        SimulationConfig,
        TestConfig,
        AgentConfig,
        Plot,
    )

logger = logging.getLogger(__name__)


def arg_parse():
    parser = ArgumentParser(description="Test Execution on Drones")
    parser.add_argument("--test", default=None, help="test description yaml file")

    # drone configs
    parser.add_argument(
        "--drone",
        default=config("DRONE", default="sitl"),
        # choices=["sitl", "cf", "ros"],
        help="type of the drone to conect to",
    )
    parser.add_argument(
        "--mission",
        default=None,
        help="input mission file address",
    )
    parser.add_argument(
        "--params",
        default=None,
        help="params file address",
    )

    # simulator configs
    parser.add_argument(
        "--simulator",
        default=config("SIMULATOR", default="gazebo"),
        choices=["gazebo", "jmavsim", "ros"],
        help="the simulator environment to run",
    )
    parser.add_argument(
        "--obstacle",
        nargs=7,
        type=float,
        help="obstacle position and size to put in simulation environment: [x1,y1,z1,x2,y2,z2] in order",
        default=[],
    )
    parser.add_argument(
        "--obstacle2",
        nargs=7,
        type=float,
        help="obstacle position and size to put in simulation environment: [x1,y1,z1,x2,y2,z2] in order",
        default=[],
    )
    parser.add_argument(
        "--pattern",
        nargs=1,
        help="flag to put a patten on the obstacle 1 being spawned",
        default="_"
    )
    parser.add_argument(
        "--pattern2",
        nargs=1,
        help="flag put a patten on the obstacle 2 being spawned",
        default="_"
    )
    # parser.add_argument(
    #     "--pattern_design2",
    #     nargs=1,
    #     help="name of the second pattern type"
    # )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=config("HEADLESS", default=False, cast=bool),
        help="whether to run the simulator headless",
    )

    parser.add_argument(
        "--speed",
        default=config("SPEED", default=1, cast=float),
        type=float,
        help="the simulator speed relative to real time",
    )

    parser.add_argument(
        "--home",
        nargs=3,
        type=float,
        help="home position to place the drone: [lat,lon,alt] in order",
        default=None,
    )

    # test configs
    parser.add_argument(
        "--commands",
        default=None,
        help="input commands file address",
    )

    # assertion configs
    parser.add_argument(
        "--log",
        default=None,
        help="input log file address",
    )

    # agent configs
    parser.add_argument(
        "--agent",
        default=config("AGENT", default="local"),
        choices=["local", "docker", "k8s"],
        help="where to run the tests",
    )
    parser.add_argument(
        "-n",
        default=1,
        type=int,
        help="no. of parallel runs (in Docker)",
    )
    parser.add_argument(
        "--path",
        default=None,
        help="cloud output path to copy logs",
    )
    parser.add_argument(
        "--id",
        default=None,
        help="k8s job id",
    )
    parser.add_argument(
        "--config",
        default=None,
        nargs=1,
        help="Config file to run the whole system"
    )

    parser.set_defaults(func=run_experiment)

    args = parser.parse_args()
    return args


def run_experiment(args):
    if args.test is not None:
        test = DroneTest.from_yaml(args.test)
    else:
        drone_config = DroneConfig(
            port=args.drone,
            params_file=args.params,
            mission_file=args.mission,
        )
        simulation_config = SimulationConfig(
            simulator=args.simulator,
            world="default",
            speed=args.speed,
            headless=args.headless,
            obstacles=args.obstacle + args.obstacle2,
            pattern=args.pattern + args.pattern2,
            home_position=args.home,
        )
        test_config = TestConfig(
            commands_file=args.commands,
            speed=args.speed,
        )
        assertion_config = AssertionConfig(
            log_file=args.log,
        )
        agent_config = AgentConfig(
            engine=args.agent,
            count=args.n,
            path=args.path,
            id=args.id,
        )
        test = DroneTest(
            drone=drone_config,
            simulation=simulation_config,
            test=test_config,
            assertion=assertion_config,
            agent=agent_config,
        )

    test_results = execute_test(test)
    logger.info(f"LOG:{test_results[0].log_file}")
    Plot(test, test_results)
    # if args.cloud:
    #         exp.log = ulog_helper.upload(exp.log, args.output)
    #     print(f"LOG:{exp.log}")


def execute_test(test: DroneTest):
    logger.info("setting up the test environment...")
    if test.agent.engine == AgentConfig.LOCAL:
        agent = LocalAgent(test)
    if test.agent.engine == AgentConfig.DOCKER:
        agent = DockerAgent(test)
    if test.agent.engine == AgentConfig.K8S:
        agent = K8sAgent(test)

    logger.info("running the test...")
    test_results = agent.run(test)

    logger.info("test finished...")
    return test_results


def config_loggers():
    os.makedirs("logs/", exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG,
        filename="logs/root.txt",
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    c_handler = logging.StreamHandler()
    f_handler = logging.FileHandler("logs/lib.txt")
    c_handler.setLevel(logging.INFO)
    f_handler.setLevel(logging.DEBUG)

    c_format = logging.Formatter("%(name)s - %(levelname)s - %(message)s")
    f_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    c_handler.setFormatter(c_format)
    f_handler.setFormatter(f_format)
    px4 = logging.getLogger("px4")
    main = logging.getLogger("__main__")
    entry = logging.getLogger("entry")
    px4.addHandler(c_handler)
    main.addHandler(c_handler)
    entry.addHandler(c_handler)
    px4.addHandler(f_handler)
    main.addHandler(f_handler)
    entry.addHandler(f_handler)


def main():
    try:
        config_loggers()
        args = arg_parse()
        logger.info(f"preparing the test ...{args}")
        if args.config is not None:
            args = reform_args(args)
        run_experiment(args)

    except Exception as e:
        logger.exception("program terminated:" + str(e), exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
