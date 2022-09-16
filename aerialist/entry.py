#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import os
import sys
import time
from decouple import config

try:
    from .px4.k8s_agent import K8sAgent
    from .px4.local_agent import LocalAgent
    from .px4.docker_agent import DockerAgent
    from .px4 import ulog_helper
    from .px4.drone_test import (
        AssertionConfig,
        DroneConfig,
        DroneTest,
        SimulationConfig,
        TestConfig,
        RunnerConfig,
    )
except:
    from px4.k8s_agent import K8sAgent
    from px4.local_agent import LocalAgent
    from px4.docker_agent import DockerAgent
    from px4 import ulog_helper
    from px4.drone_test import (
        AssertionConfig,
        DroneConfig,
        DroneTest,
        SimulationConfig,
        TestConfig,
        RunnerConfig,
    )


logger = logging.getLogger(__name__)


def arg_parse():
    parser = ArgumentParser(description="Test Execution on Drones")
    parser.add_argument(
        "--drone",
        default=config("DRONE", default="sitl"),
        choices=["sitl", "cf", "ros"],
        help="type of the drone to conect to",
    )
    parser.add_argument(
        "--simulator",
        default=config("SIMULATOR", default="gazebo"),
        choices=["gazebo", "jmavsim", "ros"],
        help="the simulator environment to run",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=config("HEADLESS", default=False, cast=bool),
        help="whether to run the simulator headless",
    )
    parser.add_argument(
        "--cloud",
        action="store_true",
        help="whether to read and write files to the cloud",
    )
    parser.add_argument(
        "--speed",
        default=config("SPEED", default=1, cast=float),
        type=float,
        help="the simulator speed relative to real time",
    )

    # parser.add_argument(
    #     "--sleep",
    #     default=0,
    #     type=float,
    #     help="wait # seconds before starting the process",
    # )

    parser.add_argument("-l", "--log", default=None, help="input log file address")
    parser.add_argument("--commands", default=None, help="input commands file address")
    parser.add_argument(
        "--trajectory", default=None, help="expected trajectory file address"
    )
    parser.add_argument("--mission", default=None, help="input mission file address")
    parser.add_argument("--params", default=None, help="params file address")
    parser.add_argument("--output", default=None, help="cloud output path to copy logs")
    # parser.add_argument(
    #     "--jmavsim",
    #     action="store_true",
    #     help="whether the original log is from jmavsim (to take into account local positioning differences with gazebo)",
    # )

    parser.add_argument(
        "--agent",
        default=config("AGENT", default="local"),
        choices=["local", "docker", "k8s"],
        help="the simulator environment to run",
    )

    parser.add_argument(
        "-n",
        default=1,
        type=int,
        help="no. of parallel runs (in Docker)",
    )

    parser.add_argument(
        "--obst",
        nargs=6,
        type=float,
        help="obstacle poisition and size to put in simulation environment: [x1,y1,z1,x2,y2,z2] in order",
        default=[],
    )
    parser.add_argument(
        "--obst2",
        nargs=6,
        type=float,
        help="obstacle poisition and size to put in simulation environment: [x1,y1,z1,x2,y2,z2] in order",
        default=[],
    )
    # sub_parsers = parser.add_subparsers(help="sub-command help")
    # exp_parser = sub_parsers.add_parser(
    #     "experiment", help="run specific experiments on the drone"
    # )
    # exp_parser.add_argument("method", help="method to run")
    parser.set_defaults(func=run_experiment)

    args = parser.parse_args()
    # if args.sleep > 0:
    #     time.sleep(args.sleep)

    if args.cloud:
        handle_cloud_files(args)
    return args


def handle_cloud_files(args):
    logger.info("downloading cloud files")
    folder = config("WEBDAV_DL_FLD")
    if args.log:
        args.log = ulog_helper.download(args.log, folder)
    if args.mission:
        args.mission = ulog_helper.download(args.mission, folder)
    if args.params:
        args.params = ulog_helper.download(args.params, folder)


def run_experiment(args):
    drone_config = DroneConfig(
        args.drone,
        args.params,
        args.mission,
    )
    simulation_config = SimulationConfig(
        args.simulator,
        "default",
        args.speed,
        args.headless,
        args.obst + args.obst2,
    )
    test_config = TestConfig(args.commands, args.speed)
    assertion_config = AssertionConfig(args.log)
    runner_config = RunnerConfig(args.agent, args.n, args.output)
    test = DroneTest(
        drone_config, simulation_config, test_config, assertion_config, runner_config
    )

    logger.info(f"setting up the test environment...")
    if args.agent == "local":
        agent = LocalAgent(test)
    if args.agent == "docker":
        agent = DockerAgent
    if args.agent == "k8s":
        agent = K8sAgent

    logger.info(f"running the test...")
    test_log = agent.run(test)

    logger.info(f"test finished...")
    logger.info(f"LOG:{test_log}")
    # if args.cloud:
    #         exp.log = ulog_helper.upload(exp.log, args.output)
    #     print(f"LOG:{exp.log}")


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
        run_experiment(args)
        # args.func(args)

    except Exception as e:
        logger.exception("program terminated:" + str(e), exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
