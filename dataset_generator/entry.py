#!/usr/bin/python3
from argparse import ArgumentParser
import logging
import sys
sys.path.append("..")
# from ..aerialist.px4.obstacle import Obstacle
from aerialist.px4.obstacle import Obstacle

from dataset_generator import DatasetGenerator
from log_parser import LogParser

logger = logging.getLogger(__name__)


def arg_parse():
    main_parser = ArgumentParser(
        description="UAV flight supervisor",
    )
    subparsers = main_parser.add_subparsers()
    parser = subparsers.add_parser(name="generate", description="generate dataset")
    parser.add_argument("folder", help="flight logs folder")

    parser.add_argument(
        "--window",
        default=5,
        type=float,
        help="the window size (in seconds)",
    )
    parser.add_argument(
        "--obstacle",
        nargs=7,
        type=float,
        help="obstacle poisition and size to put in simulation environment: [l,w,h,x,y,z,r] in order",
        default=[],
    )
    parser.add_argument(
        "--version",
        default=1,
        type=int,
        help="surrealist version used for generating the dataset",
    )

    parser.set_defaults(func=generate_dateset)

    args = main_parser.parse_args()
    return args


def generate_dateset(args):
    generator = DatasetGenerator(
        data_folder=args.folder,
        metadata=args.folder + "log.csv",
        window_length=args.window * 1000000,
        window_overlap=args.window * 500000,
    )
    if len(args.obstacle) > 0:
        LogParser.fix_obstacle = Obstacle.from_coordinates(args.obstacle)
    LogParser.SURREALIST_VERSION = args.version
    generator.run_parallel_extraction()


def main():
    try:
        args = arg_parse()
        args.func(args)

    except Exception as e:
        logger.exception("program terminated:" + str(e), exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
