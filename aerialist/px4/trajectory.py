from __future__ import annotations
import copy
import math
import os
from typing import List
import numpy as np
import pandas as pd
import similaritymeasures
import matplotlib.pyplot as plt
import ruptures as rpt
import csv
import pyulog
from shapely.geometry import LineString
from tslearn.barycenters import softdtw_barycenter
import warnings
from .obstacle import Obstacle
from .position import Position
from . import file_helper, timeserie_helper
from datetime import datetime
from itertools import zip_longest
from decouple import config
import logging
import random

logger = logging.getLogger(__name__)


class Trajectory(object):
    DIR = config("RESULTS_DIR", default="results/")
    WEBDAV_DIR = config("WEBDAV_UP_FLD", default=None)
    USE_GPS = config("USE_GPS", default=False, cast=bool)
    IGNORE_AUTO_MODES = False
    REMOVE_OFFSET = True
    PLOT_TESTS_XYZ = config("PLOT_TESTS_XYZ", default=True, cast=bool)
    PLOT_R = config("PLOT_R", default=False, cast=bool)
    TIME_RANGE = None
    DISTANCE_METHOD = config("DISTANCE_METHOD", default="dtw")
    AVERAGE_METHOD = config("AVERAGE_METHOD", default="dtw")
    AVE_GAMMA = config("AVE_GAMMA", default=20, cast=float)
    ALLIGN_ORIGIN = config("ALLIGN_ORIGIN", default=True, cast=bool)
    SAMPLING_PERIOD = config("TRJ_SMPL_PRD", cast=float, default=500000)
    RESAMPLE = config("RESAMPLE", default=True, cast=bool)
    AVE_CUT_LAND = config("AVE_CUT_LAND", default=True, cast=bool)
    HIGHLIGHT_COLOR = "red"
    HIGHLIGHT_ALPHA = 0.25
    HIGHLIGHT_SIZE = 25

    def __init__(self, positions: List[Position]) -> None:
        super().__init__()
        self.positions = positions

    def plot(
            self,
            goal: Trajectory = None,
            save: bool = True,
            obstacles: List[Obstacle] = None,
            file_prefix="",
            highlights: List[float] = None,
            filename=None,
    ):
        distance = True
        if goal is not None:
            distance = self.distance(goal)
        return self.plot_multiple(
            [self],
            goal,
            save,
            distance,
            highlights,
            obstacles,
            file_prefix,
            None,
            filename,
        )

    def save_csv(self, address: str) -> None:
        """saves trajectory to file"""
        data_frame = pd.DataFrame.from_records([p.to_dict() for p in self.positions])
        data_frame.to_csv(address, index=False)

    @classmethod
    def plot_multiple(
            cls,
            trajectories: List[Trajectory],
            goal: Trajectory = None,
            save: bool = True,
            distance: float | bool = None,
            highlights: List[float] = None,
            obstacles: List[Obstacle] = None,
            file_prefix="",
            ave_trajectory: Trajectory = None,
            wind: int = 0,
            light: float = 0.4,
            filename=None,
            upload_dir=None,
    ):
        fig = plt.figure(tight_layout=True)

        if cls.PLOT_R:
            gs = fig.add_gridspec(4, 4)
            r_plt = fig.add_subplot(gs[3, :2])
        else:
            gs = fig.add_gridspec(3, 4)
        x_plt = fig.add_subplot(gs[0, :2])
        y_plt = fig.add_subplot(gs[1, :2])
        z_plt = fig.add_subplot(gs[2, :2])
        xy_plt = fig.add_subplot(gs[:, 2:])
        # xyz_plt = fig.add_subplot(gs[:, 2:], projection="3d")
        # annotations
        fig.suptitle(" ")

        x_plt.set_ylabel("X (m)")
        y_plt.set_ylabel("Y (m)")
        z_plt.set_ylabel("Z (m)")
        if cls.PLOT_R:
            r_plt.set_ylabel("Yaw (\u00b0)")
            r_plt.set_xlabel("flight time (s)")
        else:
            z_plt.set_xlabel("flight time (s)")
        xy_plt.set_ylabel("Y (m)", loc="bottom")
        xy_plt.yaxis.set_label_position("right")
        xy_plt.yaxis.tick_right()
        xy_plt.set_xlabel("X (m)", loc="right")
        xy_plt.set_aspect("equal", "datalim")
        circle_legend = None
        wind_legend = None

        if wind != 0:
            fig.text(
                0.85,
                0.97,
                f"wind:{wind}km/hr",
                ha="left",
                va="top",
                bbox=dict(facecolor="none", edgecolor="lightgray", boxstyle="round"),
            )

        if light > 0:
            fig.text(
                0.15,
                0.97,
                f"Light:{light}",
                ha="right",
                va="top",
                bbox=dict(facecolor="none", edgecolor="lightgray", boxstyle="round"),
            )

        if obstacles is not None:
            labeled = False
            labeled_tree = False
            labeled_apartment = False
            for obst in obstacles:
                if obst.shape == "BOX":
                    obst_patch = obst.plt_patch("grey")
                    if not labeled:
                        obst_patch.set_label("Box")
                        labeled = True
                elif obst.shape == "TREE":
                    radius = 0.5
                    color = 'green'
                    obst_patch = obst.plt_patch_circle(radius, color)
                    if not labeled_tree:
                        obst_patch.set_label("Trees")
                        # circle_legend = Line2D([0], [0], marker='o', color='w', markersize=10, markerfacecolor='g',
                        #                        label='Trees')
                        labeled_tree = True
                elif obst.shape == "APARTMENT":
                    radius = 2.5
                    color = 'blue'
                    obst_patch = obst.plt_patch(color)
                    if not labeled_apartment:
                        obst_patch.set_label("Apartment")
                        labeled_apartment = True

                xy_plt.add_patch(obst_patch)

        alpha = 1 if len(trajectories) <= 1 else 0.25
        for i in range(len(trajectories)):
            data_frame = trajectories[i].to_data_frame()

            if len(trajectories) <= 1 or cls.PLOT_TESTS_XYZ:
                x_plt.plot(data_frame[:, 0], data_frame[:, 1], alpha=alpha)
                y_plt.plot(data_frame[:, 0], data_frame[:, 2], alpha=alpha)
                z_plt.plot(data_frame[:, 0], data_frame[:, 3], alpha=alpha)
                if cls.PLOT_R:
                    r_plt.plot(data_frame[:, 0], data_frame[:, 4], alpha=alpha)

            label = None
            if i == 0:
                label = "tests" if len(trajectories) > 1 else "test"

            xy_plt.plot(
                data_frame[:, 1],
                data_frame[:, 2],
                label=label,
                alpha=alpha,
            )
            if highlights is not None:
                for timestamp in highlights:
                    point = data_frame[
                        abs(data_frame[:, 0] - (timestamp / 1000000.0)).argsort()[0]
                    ]
                    xy_plt.scatter(
                        [point[1]],
                        [point[2]],
                        color=cls.HIGHLIGHT_COLOR,
                        alpha=cls.HIGHLIGHT_ALPHA,
                        s=cls.HIGHLIGHT_SIZE,
                        label="uncertainty",
                    )
            # xyz_plt.plot3D(
            #     [p.x for p in trj.positions],
            #     [p.y for p in trj.positions],
            #     [p.z for p in trj.positions],
            # )

        if len(trajectories) > 1:
            if ave_trajectory is None:
                ave_trajectory = Trajectory.average(trajectories)
            data_frame = ave_trajectory.to_data_frame()
            x_plt.plot(
                data_frame[:, 0], data_frame[:, 1], label="test ave.", color="red"
            )
            y_plt.plot(data_frame[:, 0], data_frame[:, 2], color="red")
            z_plt.plot(data_frame[:, 0], data_frame[:, 3], color="red")
            if cls.PLOT_R:
                r_plt.plot(data_frame[:, 0], data_frame[:, 4], color="red")
            xy_plt.plot(data_frame[:, 1], data_frame[:, 2], color="red")
        else:
            ave_trajectory = trajectories[0]

        if goal != None:
            data_frame = goal.to_data_frame()
            x_plt.plot(
                data_frame[:, 0], data_frame[:, 1], label="original", color="blue"
            )
            y_plt.plot(data_frame[:, 0], data_frame[:, 2], color="blue")
            z_plt.plot(data_frame[:, 0], data_frame[:, 3], color="blue")
            if cls.PLOT_R:
                r_plt.plot(data_frame[:, 0], data_frame[:, 4], color="blue")

            xy_plt.plot(data_frame[:, 1], data_frame[:, 2], color="blue")

            # xyz_plt.plot3D(
            #     [p.x for p in goal.positions],
            #     [p.y for p in goal.positions],
            #     [p.z for p in goal.positions],
            # )

        if distance is True and obstacles is not None and len(obstacles) > 0:
            distance = ave_trajectory.min_distance_to_obstacles(obstacles)
        if distance is not None and distance is not False:
            fig.text(
                0.71,
                0.03,
                f"distance:{round(distance, 2)}",
                ha="center",
                bbox=dict(facecolor="none", edgecolor="lightgray", boxstyle="round"),
            )
        if highlights is not None:
            for timestamp in highlights:
                x_plt.axvline(
                    timestamp / 1000000.0,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                y_plt.axvline(
                    timestamp / 1000000.0,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                z_plt.axvline(
                    timestamp / 1000000.0,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                if cls.PLOT_R:
                    r_plt.axvline(
                        timestamp / 1000000.0,
                        color=cls.HIGHLIGHT_COLOR,
                        alpha=cls.HIGHLIGHT_ALPHA,
                        linewidth=1.75,
                    )
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        fig.legend(
            by_label.values(),
            by_label.keys(),
            loc="upper center",
            ncol=3 if obstacles is None else 3,
        )
        if save:
            if filename is None:
                filename = file_prefix + file_helper.time_filename(add_host=True)
            os.makedirs(cls.DIR, exist_ok=True)
            plot_file = f"{cls.DIR}{filename}.png"
            fig.savefig(plot_file)
            plt.close(fig)
            if cls.WEBDAV_DIR is not None:
                file_helper.upload(f"{cls.DIR}{filename}.png", upload_dir)
            return plot_file
        else:
            plt.ion()
            plt.show()

    @classmethod
    def log_threshold_limit(cls, results: List[DroneTestResult], upload_dir: str):
        result_dir = config("RESULTS_DIR", default="results/")
        threshold_file = config("THRESHOLD_FILE", default="threshold")
        file_extension = config("FILE_EXTENSION", default=".csv")
        threshold_file_edit_mode = config("THRESHOLD_FILE_EDIT_MODE", default="a")
        file_ts = str(datetime.now().strftime("%Y%m%d%H%M%S"))
        average_traj = Trajectory.average([r.record for r in results])
        distance_list = [r.record.distance(average_traj) for r in results]
        threshold_file_combined = threshold_file + file_ts + file_extension
        f = open(result_dir + threshold_file_combined, threshold_file_edit_mode)
        writer = csv.writer(f)
        writer.writerow(distance_list)
        f.close()
        if cls.WEBDAV_DIR is not None:
            file_helper.upload(result_dir + threshold_file_combined, upload_dir)

    @classmethod
    def log_csv(cls,
                test: DroneTest,
                results: List[DroneTestResult],
                wind: int = 0,
                obstacles: List[Obstacle] = None,
                light: float = 0.4,
                ) -> None:
        result_dir = config("RESULTS_DIR", default="results/")
        dataset_file_combined = config("DATASET_FILE", default="dataset_file_combined")
        dataset_file_edit_mode = config("DATA_FILE_EDIT_MODE", default="a")
        cpu_file = config("CPU_FILE", default="cpu_file")
        cpu_file_edit_mode = config("CPU_FILE_EDIT_MODE", default="a")
        file_extension = config("FILE_EXTENSION", default=".csv")
        default_separation = config("DEFAULT_SEPARATION", default=",")
        file_ts = str(datetime.now().strftime("%Y%m%d%H%M%S"))
        dataset_file = ""
        # print(f"***LOG:{results[0].log_file}")
        log = pyulog.ULog(results[0].log_file)
        # print("**Printing cpu data list")
        cpu_data = log.get_dataset('cpuload')
        # print(cpu_data)
        cpu_load = cpu_data.data['load']
        ram_usage = cpu_data.data['ram_usage']
        cpu_timestamp = cpu_data.data['timestamp']
        cpu_timestamp_list = []
        cpu_header = False
        unsafe_flag = 0
        # print(f'**keys are {cpu_data.data.keys()}')
        # print(f'cpu load and ram usage length are {len(cpu_load)},{len(ram_usage)}')
        for temp_cpu_load, temp_ram_usage, temp_cpu_timestamp in zip_longest(cpu_load, ram_usage, cpu_timestamp):
            cpu_row = [temp_cpu_timestamp, temp_cpu_load, temp_ram_usage]
            cpu_timestamp_list.append(temp_cpu_timestamp)
            f = open(result_dir + cpu_file + "_" + file_ts + file_extension + str(random.randrange(0,100)),
                     cpu_file_edit_mode)
            writer = csv.writer(f)
            if not cpu_header:
                writer.writerow(["timestamp", "cpu_usage", "ram_usage"])
                cpu_header = True
            writer.writerow(cpu_row)
            f.close()

        trajectories: List[Trajectory] = [r.record for r in results]
        for trajectory in trajectories:
            positions = trajectory.positions
            obstacle_flag = False
            obstacles_present = 0
            obstacle_list = []
            # number_of_obstacles = -1
            number_of_trees = 0
            number_of_apartments = 0
            number_of_boxes = 0
            tree_cumm_list = []
            apt_cumm_list = []
            box_cumm_list = []
            box_min_distance = []
            tree_min_distance = []
            apt_min_distance = []
            obstacle_header_distance = []
            tree_count = apt_count = box_count = 0
            if test.simulation.obstacles is None:
                return
            for obs in test.simulation.obstacles:
                min_distance, returned_list = trajectory.distance_to_obstacles([obs])
                if min_distance < 1.5:
                    unsafe_flag = 1
                if obs.shape == "TREE":
                    tree_cumm_list.append(returned_list)
                    tree_count += 1
                    tree_index = "tree_" + str(tree_count) + "_min_distance"
                    obstacle_header_distance.append(tree_index)
                    tree_min_distance.append(min_distance)
                elif obs.shape == "APARTMENT":
                    apt_cumm_list.append(returned_list)
                    apt_count += 1
                    apartment_index = "apt_" + str(apt_count) + "_min_distance"
                    obstacle_header_distance.append(apartment_index)
                    apt_min_distance.append(min_distance)
                elif obs.shape == "BOX":
                    box_cumm_list.append(returned_list)
                    box_count += 1
                    box_index = "box_" + str(box_count) + "_min_distance"
                    obstacle_header_distance.append(box_index)
                    box_min_distance.append(min_distance)

            sum_tree = [sum(elts) for elts in zip(*tree_cumm_list)]
            sum_apt = [sum(elts) for elts in zip(*apt_cumm_list)]
            sum_box = [sum(elts) for elts in zip(*box_cumm_list)]
            avg_tree = [divmod(x, len(tree_cumm_list))[0] for x in sum_tree]
            avg_apt = [divmod(x, len(apt_cumm_list))[0] for x in sum_apt]
            avg_box = [divmod(x, len(box_cumm_list))[0] for x in sum_box]
            i = 0

            csv_header = ["x", "y", "z", "r", "timestamp", "wind", "light",
                          "obstacle_present"]
            csv_header_obstacles = ["no_of_obst", "no_of_boxes", "no_of_trees", "no_of_apt", "avg_dist_boxes",
                                    "avg_dist_trees", "avg_dist_apt"]
            csv_header_obst_end = ["obst_details", "unsafe"]
            header_flag = False
            if len(obstacles) > 0:
                obstacle_flag = True
                obstacles_present = 1
                for obstacle in test.simulation.obstacles:
                    if obstacle.shape == "TREE":
                        number_of_trees += 1
                    elif obstacle.shape == "APARTMENT":
                        number_of_apartments += 1
                    elif obstacle.shape == "BOX":
                        number_of_boxes += 1
                    obstacle_type = obstacle.shape
                    obst_x = obstacle.position.x
                    obst_y = obstacle.position.y
                    obst_z = obstacle.position.z
                    obst_r = obstacle.position.r
                    temp_obst_row = (obstacle_type, obst_x, obst_y, obst_z, obst_r)
                    obstacle_list.append(temp_obst_row)
            for position in positions:
                x = position.x
                y = position.y
                z = position.z
                r = position.r
                timestamp = position.timestamp
                average_tree_distance = average_box_distance = average_apt_distance = 0

                if len(avg_tree) > 0:
                    average_tree_distance = avg_tree[i]
                if len(avg_box) > 0:
                    average_box_distance = avg_box[i]
                if len(avg_apt) > 0:
                    average_apt_distance = avg_apt[i]
                i += 1
                header_final = []
                if obstacle_flag:
                    number_of_obstacles = len(obstacle_list)
                    if not header_flag:
                        header_final = csv_header + csv_header_obstacles + obstacle_header_distance + csv_header_obst_end
                    row = [x, y, z, r, timestamp, wind, light, obstacles_present, number_of_obstacles, number_of_trees,
                           number_of_boxes,
                           number_of_apartments, average_box_distance, average_tree_distance,
                           average_apt_distance] + tree_min_distance + box_min_distance + apt_min_distance + [
                              obstacle_list] + [unsafe_flag]
                else:
                    if not header_flag:
                        header_final = csv_header
                    row = [x, y, z, r, timestamp, wind, light, obstacles_present]
                dataset_file = result_dir + dataset_file_combined + "_" + file_ts + file_extension
                f = open(dataset_file, dataset_file_edit_mode)
                write = csv.writer(f)
                if not header_flag:
                    write.writerow(header_final)
                    header_flag = True
                write.writerow(row)
                f.close()
        if cls.WEBDAV_DIR is not None:
            file_helper.upload(dataset_file, test.agent.path)

    def allign_origin(self):
        origin = copy.deepcopy(self.positions[0])
        for p in self.positions:
            p.x -= origin.x
            p.y -= origin.y
            p.z -= origin.z
            if origin.timestamp is not None:
                p.timestamp -= origin.timestamp
            if origin.r is not None:
                p.r -= origin.r

    def handle_rotation(self, threshold=np.pi):
        for i in range(1, len(self.positions)):
            diff = self.positions[i].r - self.positions[i - 1].r
            if diff > threshold:
                self.positions[i].r -= 2 * np.pi
                # for p in self.positions[i:]:
                #     p.r -= 2 * np.pi
            elif diff < -threshold:
                self.positions[i].r += 2 * np.pi
                # for p in self.positions[i:]:
                #     p.r += 2 * np.pi
        return

    def distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectoryies using Dynamic Time Warping and normalized datapoints"""

        if self.DISTANCE_METHOD == "dtw-t":
            return self.dtw_t_distance(other)
        elif self.DISTANCE_METHOD == "frechet":
            return self.frechet_distance(other)
        else:
            return self.dtw_distance(other)

    def dtw_t_distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectoryies using Dynamic Time Warping and normalized datapoints"""
        dtw, d = similaritymeasures.dtw(
            timeserie_helper.normalize(self.to_data_frame()[:, 0:4]),
            timeserie_helper.normalize(other.to_data_frame()[:, 0:4]),
        )
        return dtw

    def dtw_distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectories using Dynamic Time Warping and not considering the timestamps"""

        dtw, d = similaritymeasures.dtw(
            self.to_data_frame()[:, 1:4],
            other.to_data_frame()[:, 1:4],
        )
        return dtw

    def frechet_distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectories using Frechete and not considering the timestamps"""
        d = similaritymeasures.frechet_dist(
            self.to_data_frame()[:, 1:4], other.to_data_frame()[:, 1:4]
        )
        return d

    def to_data_frame(self):
        positions = self.positions
        if self.TIME_RANGE is not None:
            positions = [
                p
                for p in positions
                if p.timestamp >= self.TIME_RANGE[0]
                   and p.timestamp <= self.TIME_RANGE[1]
            ]

        data = np.zeros(
            (len(positions), 5),
        )
        data[:, 0] = [p.timestamp / 1000000.0 for p in positions]
        data[:, 1] = [p.x for p in positions]
        data[:, 2] = [p.y for p in positions]
        data[:, 3] = [p.z for p in positions]
        data[:, 4] = [p.r for p in positions]
        return data

    def downsample(self, count: int) -> Trajectory:
        if count > len(self.positions):
            return None
        if count == len(self.positions):
            return self
        scale = len(self.positions) / float(count)
        down_sampled: List[Position] = []
        for i in range(count):
            down_sampled.append(self.positions[math.ceil(i * scale)])

        return Trajectory(down_sampled)

    def downsample_time(self, period: float = SAMPLING_PERIOD) -> Trajectory:
        down_sampled: List[Position] = []
        period_start = self.positions[0].timestamp
        i = 0
        while i < len(self.positions):
            period_points: List[Position] = []
            while (
                    i < len(self.positions)
                    and self.positions[i].timestamp <= period_start + period
            ):
                period_points.append(self.positions[i])
                i += 1
            if len(period_points) > 0:
                down_sampled.append(Position.average(period_points))
            period_start += period
        return Trajectory(down_sampled)

    def min_distance_to_obstacles(self, obstacles: List[Obstacle]):
        line = self.to_line()
        return min(obst.distance(line) for obst in obstacles)

    def distance_to_obstacles(self, obstacles: List[Obstacle]):
        return Obstacle.distance_to_many(obstacles, self.to_line())

    def to_line(self) -> LineString:
        points = self.to_data_frame()[:, 1:4]
        line = LineString(points)
        return line

    @classmethod
    def extract(cls, address: str) -> Trajectory:
        """extracts and returns trajectory from the file"""
        if address.endswith(".csv"):
            return cls.extract_from_csv(address)
        if address.endswith(".ulg"):
            return cls.extract_from_log(address)
        return None

    @classmethod
    def extract_from_csv(cls, address: str) -> Trajectory:
        """extracts and returns trajectory from the saved log"""
        positions = []

        pos_csv = pd.read_csv(address)
        for row in pos_csv.itertuples():
            positions.append(
                Position(
                    row.x,
                    row.y,
                    row.z,
                    row.r,
                    row.timestamp,
                )
            )
        trj = Trajectory(positions)
        return trj

    @classmethod
    def extract_from_log(
            cls,
            log_address: str,
            ignore_automodes=False,
            is_jmavsim=False,
    ):
        """extracts and returns trajectory logs from the input log"""
        positions: List[Position] = []

        # global position
        if cls.USE_GPS:
            global_position = file_helper.extract(
                log_address, "vehicle_global_position"
            )
            global_position = global_position[["timestamp", "lat", "lon", "alt"]]
            for row in global_position.itertuples():
                positions.append(
                    Position(
                        row.lat,
                        row.lon,
                        row.alt,
                        timestamp=row.timestamp,
                    )
                )

        # local position
        else:
            local_position = file_helper.extract(log_address, "vehicle_local_position")
            local_position = local_position[["timestamp", "x", "y", "z", "heading"]]
            for row in local_position.itertuples():
                pos = Position(
                    row.x,
                    row.y,
                    -row.z,
                    row.heading,
                    timestamp=row.timestamp,
                )
                if is_jmavsim:
                    pos.convert_jmavsim()
                positions.append(pos)

        if cls.REMOVE_OFFSET:
            offset = positions[0].timestamp
            for p in positions:
                p.timestamp -= offset

        if cls.ALLIGN_ORIGIN:
            origin = copy.deepcopy(positions[0])
            for p in positions:
                p.x -= origin.x
                p.y -= origin.y
                p.z -= origin.z
                p.r -= origin.r

        if ignore_automodes or cls.IGNORE_AUTO_MODES:
            in_auto_modes = True
            filtered_positions = []
            period_start = None
            commander_state = file_helper.extract(log_address, "commander_state")
            for row in commander_state.itertuples():
                # if row.main_state_changes == 0:
                #     # skip invalid states before the first state change
                #     continue
                mode = row.main_state
                if mode >= 0 and mode <= 2:  # manual,altitude and position modes
                    if in_auto_modes:
                        period_start = row.timestamp
                        in_auto_modes = False

                elif not in_auto_modes and period_start != None:
                    period_end = row.timestamp
                    filtered_positions += list(
                        filter(
                            lambda p: (
                                    p.timestamp >= period_start
                                    and p.timestamp <= period_end
                            ),
                            positions,
                        )
                    )
                    in_auto_modes = True
                    period_start = None

            filtered_positions.append(positions[-1])

            positions = filtered_positions

        traj = cls(positions)
        if cls.RESAMPLE:
            traj = traj.downsample_time()
        return traj

    @classmethod
    def extract_waypoints(cls, log_address: str):
        """extracts and returns trajectory logs from the input log"""
        positions: List[Position] = []

        # waypoints
        waypoints = file_helper.extract(log_address, "vehicle_trajectory_waypoint")
        waypoints = waypoints[
            [
                "timestamp",
                "waypoints[0].position[0]",
                "waypoints[0].position[1]",
                "waypoints[0].position[2]",
                "waypoints[0].yaw",
            ]
        ]
        waypoints.columns = ["timestamp", "x", "y", "z", "heading"]
        for row in waypoints.itertuples():
            pos = Position(
                row.x,
                row.y,
                -row.z,
                row.heading,
                timestamp=row.timestamp,
            )
            positions.append(pos)

        if cls.REMOVE_OFFSET:
            offset = positions[0].timestamp
            for p in positions:
                p.timestamp -= offset

        if cls.ALLIGN_ORIGIN:
            origin = copy.deepcopy(positions[0])
            for p in positions:
                p.x -= origin.x
                p.y -= origin.y
                p.z -= origin.z
                p.r -= origin.r

        traj = cls(positions)
        # if cls.RESAMPLE:
        #     traj = traj.downsample_time()
        return traj

    @classmethod
    def extract_groundtruth(cls, log_address: str):
        """extracts and returns trajectory logs from the input log"""
        positions: List[Position] = []

        # waypoints
        waypoints = file_helper.extract(
            log_address, "vehicle_local_position_groundtruth"
        )
        waypoints = waypoints[["timestamp", "x", "y", "z", "heading"]]
        for row in waypoints.itertuples():
            pos = Position(
                row.x,
                row.y,
                -row.z,
                row.heading,
                timestamp=row.timestamp,
            )
            positions.append(pos)

        if cls.REMOVE_OFFSET:
            offset = positions[0].timestamp
            for p in positions:
                p.timestamp -= offset

        if cls.ALLIGN_ORIGIN:
            origin = copy.deepcopy(positions[0])
            for p in positions:
                p.x -= origin.x
                p.y -= origin.y
                p.z -= origin.z
                p.r -= origin.r

        traj = cls(positions)
        # if cls.RESAMPLE:
        #     traj = traj.downsample_time()
        return traj

    @classmethod
    def get_home(cls, log_address: str):
        global_position = file_helper.extract(log_address, "vehicle_global_position")
        global_position = global_position[["timestamp", "lat", "lon", "alt"]]
        for row in global_position.itertuples():
            home = [
                row.lat,
                row.lon,
                row.alt,
            ]
            return home

    def extract_segments(self) -> List[Trajectory]:
        data = np.array([[c.x, c.y, c.z] for c in self.positions])
        alg = rpt.Pelt(model="rbf").fit(data)
        change_idx = [0] + alg.predict(pen=5)
        segments: List[Trajectory] = []
        for i in range(len(change_idx) - 1):
            seg_pos = self.positions[change_idx[i]: change_idx[i + 1]]
            segments.append(Trajectory(seg_pos))

        return segments

    @classmethod
    def average(cls, trajectories: List[Trajectory]) -> Trajectory:
        if cls.AVERAGE_METHOD == "dtw":
            return cls.dtw_average(trajectories)
        else:
            return cls.simple_average(trajectories)

    @classmethod
    def simple_average(cls, trajectories: List[Trajectory]) -> Trajectory:
        min_len = min([len(t.positions) for t in trajectories])
        down_sampled = [t.downsample(min_len) for t in trajectories]
        ave_positions: List[Position] = []
        for i in range(min_len):
            ave_positions.append(
                Position.average([t.positions[i] for t in down_sampled])
            )

        return Trajectory(ave_positions)

    @classmethod
    def dtw_average(cls, trajectories: List[Trajectory]) -> Trajectory:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            # resampled = [t.downsample_time() for t in trajectories]
            resampled = trajectories
            dataset = [t.to_data_frame()[:, 1:4] for t in resampled]
            average_data = softdtw_barycenter(dataset, gamma=cls.AVE_GAMMA)
            time_dataset = [t.to_data_frame()[:, 0] for t in resampled]
            average_time = softdtw_barycenter(time_dataset, gamma=cls.AVE_GAMMA)
            r_dataset = [t.to_data_frame()[:, 4] for t in resampled]
            average_r = softdtw_barycenter(r_dataset, gamma=cls.AVE_GAMMA)

        ave_positions: List[Position] = []
        if len(average_time) == len(average_data):
            for i in range(len(average_time)):
                ave_positions.append(
                    Position(
                        timestamp=average_time[i, 0] * 1000000,
                        x=average_data[i, 0],
                        y=average_data[i, 1],
                        z=average_data[i, 2],
                        r=average_r[i, 0],
                    )
                )
        ave_trj = Trajectory(ave_positions)
        if cls.AVE_CUT_LAND:
            ave_trj = ave_trj.cut_landed()
        return ave_trj

    def cut_landed(self):
        data = np.array([[c.x, c.y, c.z] for c in self.positions])
        alg = rpt.Pelt(model="rbf").fit(data)
        cut_idx = (alg.predict(pen=0.01)[-2] + alg.predict(pen=0.01)[-1]) // 2
        cut_list = self.positions[cut_idx:]
        cut_ave = Position.average(cut_list)
        positions = self.positions[0:cut_idx]
        positions.append(cut_ave)
        return Trajectory(positions)

    @classmethod
    def load_folder(cls, path, ignore_automodes=False):
        files = [path + f for f in os.listdir(path) if f.endswith(".ulg")]
        trjs = [Trajectory.extract_from_log(f, ignore_automodes) for f in files]
        return trjs
