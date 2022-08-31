#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import os
import sys
import time
from decouple import config
from px4.k8s_experiment import K8sExperiment
from px4.experiment import Experiment
from px4.docker_experiment import DockerExperiment
from px4 import ulog_helper

logger = logging.getLogger(__name__)


def arg_parse():
    parser = ArgumentParser(
        description="Comunicate with and send control commands to drones"
    )
    parser.add_argument(
        "-d",
        "--drone",
        default=config("DRONE", default="sim"),
        choices=["sim", "cf", "ros", "none"],
        help="type of the drone to conect to",
    )
    parser.add_argument(
        "-e",
        "--env",
        default=config("SIMULATOR", default="gazebo"),
        choices=["gazebo", "jmavsim", "avoidance"],
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
        "-s",
        "--speed",
        default=config("SPEED", default=1, cast=float),
        type=float,
        help="the simulator speed relative to real time",
    )
    parser.add_argument(
        "--sleep",
        default=0,
        type=float,
        help="wait # seconds before starting the process",
    )

    parser.add_argument("-l", "--log", default=None, help="input log file address")
    parser.add_argument("--commands", default=None, help="input commands file address")
    parser.add_argument(
        "--trajectory", default=None, help="expected trajectory file address"
    )
    parser.add_argument(
        "-m", "--mission", default=None, help="input mission file address"
    )
    parser.add_argument("--params", default=None, help="params file address")
    parser.add_argument("--output", default=None, help="cloud output path to copy logs")
    parser.add_argument(
        "--jmavsim",
        action="store_true",
        help="whether the original log is from jmavsim (to take into account local positioning differences with gazebo)",
    )

    parser.add_argument(
        "--docker",
        action="store_true",
        help="whether to run the experiments in docker containers",
    )

    parser.add_argument(
        "--k8s",
        action="store_true",
        help="whether to run the experiments in k8s",
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
    sub_parsers = parser.add_subparsers(help="sub-command help")
    exp_parser = sub_parsers.add_parser(
        "experiment", help="run specific experiments on the drone"
    )
    exp_parser.add_argument("method", help="method to run")
    exp_parser.set_defaults(func=run_experiment)

    args = parser.parse_args()
    if args.sleep > 0:
        time.sleep(args.sleep)

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
    if args.k8s:
        K8sExperiment.replay_multiple(
            args.n,
            args.drone,
            args.env,
            args.headless,
            args.speed,
            args.log,
            args.jmavsim,
            args.params,
            args.obst + args.obst2,
            args.mission,
        )
        return
    if args.docker:
        if args.n == 1:
            exp = DockerExperiment(
                args.drone,
                args.env,
                args.headless,
                args.speed,
                args.log,
                args.jmavsim,
                args.params,
                args.obst + args.obst2,
                args.mission,
            )
        else:
            DockerExperiment.replay_multiple(
                args.n,
                args.drone,
                args.env,
                args.headless,
                args.speed,
                args.log,
                args.jmavsim,
                args.params,
                args.obst + args.obst2,
                args.mission,
            )
            return
    else:
        exp = Experiment(
            args.drone,
            args.env,
            args.headless,
            args.speed,
            args.log,
            args.jmavsim,
            args.params,
            args.obst + args.obst2,
            args.mission,
        )

    if args.method == "none":
        exp.simulator.bkgnd.join()
    else:
        method = getattr(exp, args.method)
        logger.debug(f'running the "{method}" experiment...')
        method()
        logger.info(f"LOG:{exp.log}")
        if args.cloud:
            exp.log = ulog_helper.upload(exp.log, args.output)
        print(f"LOG:{exp.log}")


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
    px4.addHandler(c_handler)
    main.addHandler(c_handler)
    px4.addHandler(f_handler)
    main.addHandler(f_handler)


def main():
    try:
        config_loggers()
        args = arg_parse()
        logger.info(f"preparing the experiment environment...{args}")
        args.func(args)

    except Exception as e:
        logger.exception("program terminated:" + str(e), exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
