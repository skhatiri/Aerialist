#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import os
import sys
from decouple import config


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
        DroneTestResult,
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
        DroneTestResult,
    )


logger = logging.getLogger(__name__)


def arg_parse():
    main_parser = ArgumentParser(
        description="UAV Test Bench",
    )
    subparsers = main_parser.add_subparsers()
    parser = subparsers.add_parser(name="exec", description="executes a UAV test")
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
        help="obstacle poisition and size to put in simulation environment: [l,w,h,x,y,z,r] in order",
        default=[],
    )
    parser.add_argument(
        "--obstacle2",
        nargs=7,
        type=float,
        help="obstacle poisition and size to put in simulation environment: [l,w,h,x,y,z,r] in order",
        default=[],
    )
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
        help="no. of parallel runs (in k8s)",
    )
    parser.add_argument(
        "--path",
        default=None,
        help="cloud output path to copy logs (in k8s)",
    )
    parser.add_argument(
        "--id",
        default=None,
        help="k8s job id",
    )

    parser.set_defaults(func=run_experiment)

    # plotting parser
    plot_parser = subparsers.add_parser(
        name="plot", description="plot an executed test"
    )
    plot_parser.add_argument("--test", default=None, help="test description yaml file")
    plot_parser.add_argument(
        "--log",
        "--logs",
        default=None,
        help="test log file address / parallel tests logs folder address",
    )
    plot_parser.set_defaults(func=plot_test)

    args = main_parser.parse_args()
    return args


def run_experiment(args):
    if args.test is not None:
        test = DroneTest.from_yaml(args.test)
        if test.agent is None:
            test.agent = AgentConfig(
                engine=args.agent,
                count=args.n,
                path=args.path,
                id=args.id,
            )

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
    DroneTest.plot(test, test_results)
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
    test_results = agent.run()

    logger.info("test finished...")
    return test_results


def plot_test(args):
    if args.test is not None:
        test = DroneTest.from_yaml(args.test)
    else:
        test = DroneTest()
    if args.log is not None:
        if args.log.endswith(".ulg"):
            test_results = [DroneTestResult(args.log)]
        else:
            test_results = DroneTestResult.load_folder(args.log)

    DroneTest.plot(test, test_results)


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
        args.func(args)

    except Exception as e:
        logger.exception("program terminated:" + str(e), exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
