from __future__ import annotations
import copy
import math
import os
from typing import List
import numpy as np
import pandas as pd
import similaritymeasures
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import ruptures as rpt
from shapely.geometry import LineString
from decouple import config
from .obstacle import Obstacle
from .position import Position
from . import file_helper, timeserie_helper


class Trajectory(object):
    DIR = config("RESULTS_DIR", default="results/")
    WEBDAV_DIR = config("WEBDAV_UP_FLD", default=None)
    USE_GPS = config("USE_GPS", default=False, cast=bool)
    IGNORE_AUTO_MODES = False
    REMOVE_OFFSET = True
    PLOT_TESTS_XYZ = config("PLOT_TESTS_XYZ", default=True, cast=bool)
    TIME_RANGE = None
    DISTANCE_METHOD = config("DISTANCE_METHOD", default="dtw")
    ALLIGN_ORIGIN = config("ALLIGN_ORIGIN", default=True, cast=bool)

    def __init__(
        self, positions: List[Position], highlights: List[tuple[int, int]] = []
    ) -> None:
        super().__init__()
        self.positions = positions
        self.highlights = highlights

    def plot(
        self,
        goal: Trajectory = None,
        save: bool = True,
        obstacles: List[Obstacle] = None,
        file_prefix="",
    ):
        distance = None
        if goal is not None:
            distance = self.distance(goal)
        self.plot_multiple(
            [self], goal, save, distance, self.highlights, obstacles, file_prefix
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
        distance: float = None,
        highlights: bool = None,
        obstacles: List[Obstacle] = None,
        file_prefix="",
        ave_trajectory: Trajectory = None,
    ):
        fig = plt.figure(tight_layout=True)
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
        z_plt.set_xlabel("flight time (s)")
        xy_plt.set_ylabel("Y (m)")
        xy_plt.yaxis.set_label_position("right")
        xy_plt.yaxis.tick_right()
        xy_plt.set_xlabel("X (m)")

        if obstacles is not None:
            labeled = False
            for obst in obstacles:
                obst_patch = mpatches.Rectangle(
                    (
                        obst.min().x,
                        obst.min().y,
                    ),
                    obst.size().x,
                    obst.size().y,
                    edgecolor="gray",
                    facecolor="gray",
                    fill=True,
                    alpha=0.5,
                    label="obstacle" if not labeled else None,
                )
                labeled = True
                xy_plt.add_patch(obst_patch)

                # obst_patch = mpatches.Rectangle(
                #     (
                #         obstacle[0][0] - obstacle[1][0] / 2,
                #         obstacle[0][2] - obstacle[1][2] / 2,
                #     ),
                #     obstacle[1][0],
                #     obstacle[1][2],
                #     edgecolor="black",
                #     fill=False,
                #     alpha=0.5,
                # )
                # xyz_plt.add_patch(obst_patch)
                # mplot3d.art3d.pathpatch_2d_to_3d(
                #     obst_patch, z=obstacle[0][1] - obstacle[1][1] / 2, zdir="y"
                # )

                # patch2 = mpatches.Rectangle(
                #     (
                #         obstacle[0][0] - obstacle[1][0] / 2,
                #         obstacle[0][2] - obstacle[1][2] / 2,
                #     ),
                #     obstacle[1][0],
                #     obstacle[1][2],
                #     edgecolor="black",
                #     fill=False,
                #     alpha=0.5,
                # )
                # xyz_plt.add_patch(patch2)
                # mplot3d.art3d.pathpatch_2d_to_3d(
                #     patch2, z=obstacle[0][1] + obstacle[1][1] / 2, zdir="y"
                # )

        alpha = 1 if len(trajectories) <= 1 else 0.25
        for i in range(len(trajectories)):

            data_frame = trajectories[i].to_data_frame()

            if len(trajectories) <= 1 or cls.PLOT_TESTS_XYZ:
                x_plt.plot(data_frame[:, 0], data_frame[:, 1], alpha=alpha)
                y_plt.plot(data_frame[:, 0], data_frame[:, 2], alpha=alpha)
                z_plt.plot(data_frame[:, 0], data_frame[:, 3], alpha=alpha)
            label = None
            if i == 0:
                # if len(trajectories) > 1:
                label = "tests"
                # elif goal is None:
                #     label = "flight trajectory"

            xy_plt.plot(
                data_frame[:, 1],
                data_frame[:, 2],
                label=label,
                alpha=alpha,
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

            xy_plt.plot(data_frame[:, 1], data_frame[:, 2], color="red")

        if goal != None:
            data_frame = goal.to_data_frame()
            x_plt.plot(
                data_frame[:, 0], data_frame[:, 1], label="original", color="blue"
            )

            y_plt.plot(data_frame[:, 0], data_frame[:, 2], color="blue")

            z_plt.plot(data_frame[:, 0], data_frame[:, 3], color="blue")

            xy_plt.plot(data_frame[:, 1], data_frame[:, 2], color="blue")

            # xyz_plt.plot3D(
            #     [p.x for p in goal.positions],
            #     [p.y for p in goal.positions],
            #     [p.z for p in goal.positions],
            # )

        if distance is not None:
            fig.text(
                0.5,
                0.03,
                f"distance:{round(distance,2)}",
                ha="center",
                bbox=dict(facecolor="none", edgecolor="lightgray", boxstyle="round"),
            )

        if highlights is not None:
            for period in highlights:
                x_plt.axvspan(
                    period[0] / 1000000.0,
                    period[1] / 1000000.0,
                    color="yellow",
                    alpha=0.5,
                )
                y_plt.axvspan(
                    period[0] / 1000000.0,
                    period[1] / 1000000.0,
                    color="yellow",
                    alpha=0.5,
                )
                z_plt.axvspan(
                    period[0] / 1000000.0,
                    period[1] / 1000000.0,
                    color="yellow",
                    alpha=0.5,
                )

        fig.legend(loc="upper center", ncol=3 if obstacles is None else 4)
        if save:
            filename = file_prefix + file_helper.time_filename()
            os.makedirs(cls.DIR, exist_ok=True)
            fig.savefig(f"{cls.DIR}{filename}.png")
            plt.close(fig)
            if cls.WEBDAV_DIR is not None:
                file_helper.upload(f"{cls.DIR}{filename}.png", cls.WEBDAV_DIR)

        else:
            plt.ion()
            plt.show()

    def allign_origin(self):
        origin = copy.deepcopy(self.positions[0])
        for p in self.positions:
            p.x -= origin.x
            p.y -= origin.y
            p.z -= origin.z
            p.timestamp -= origin.timestamp
            p.r -= origin.r

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
            timeserie_helper.normalize(self.to_data_frame()),
            timeserie_helper.normalize(other.to_data_frame()),
        )
        return dtw

    def dtw_distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectories using Dynamic Time Warping and not considering the timestamps"""

        dtw, d = similaritymeasures.dtw(
            self.to_data_frame()[:, 1:],
            other.to_data_frame()[:, 1:],
        )
        return dtw

    def frechet_distance(self, other: Trajectory) -> float:
        """quantify the difference between the two trajectories using Frechete and not considering the timestamps"""
        d = similaritymeasures.frechet_dist(
            self.to_data_frame()[:, 1:], other.to_data_frame()[:, 1:]
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
            (len(positions), 4),
        )
        data[:, 0] = [p.timestamp / 1000000.0 for p in positions]
        data[:, 1] = [p.x for p in positions]
        data[:, 2] = [p.y for p in positions]
        data[:, 3] = [p.z for p in positions]
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

    def distance_to_obstacles(self, obstacles: List[Obstacle]):
        return Obstacle.distance_to_many(obstacles, self.to_line())

    def to_line(self) -> LineString:
        points = self.to_data_frame()[:, 1:3]
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
        load_CP_activations=True,
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
                positions.append(
                    Position(
                        row.x,
                        row.y,
                        -row.z,
                        row.heading,
                        timestamp=row.timestamp,
                        is_jmavsim=is_jmavsim,
                    )
                )

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

        cp_activations = None
        if load_CP_activations:
            cp_activations = cls.extract_CP_active_periods(log_address)

        traj = cls(positions, cp_activations)
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
            seg_pos = self.positions[change_idx[i] : change_idx[i + 1]]
            segments.append(Trajectory(seg_pos))

        return segments

    @classmethod
    def extract_CP_active_periods(cls, log_address):
        collision_constraints = file_helper.extract(
            log_address, "collision_constraints"
        )
        if collision_constraints is None:
            return []
        avoidance_active_periods = []
        period_start = -1
        for ind, row in collision_constraints.iterrows():
            # skip the rows where CP did not change setpoints (where in-active)
            if (
                row["original_setpoint[0]"] != row["adapted_setpoint[0]"]
                or row["original_setpoint[1]"] != row["adapted_setpoint[1]"]
            ):
                if period_start < 0:
                    period_start = int(row["timestamp"])
                    avoidance_active_periods.append(
                        (period_start, int(row["timestamp"]))
                    )
                else:
                    avoidance_active_periods[-1] = (
                        period_start,
                        int(row["timestamp"]),
                    )
            else:
                period_start = -1

        return avoidance_active_periods

    @classmethod
    def average(cls, trajectories: List[Trajectory]) -> Trajectory:
        min_len = min([len(t.positions) for t in trajectories])
        down_sampled = [t.downsample(min_len) for t in trajectories]
        ave_positions: List[Position] = []
        for i in range(min_len):
            ave_positions.append(
                Position.average([t.positions[i] for t in down_sampled])
            )

        return Trajectory(ave_positions)

    @classmethod
    def get_average_folder(cls, path, ignore_automodes=False):
        files = [path + f for f in os.listdir(path) if f.endswith(".ulg")]
        trjs = [Trajectory.extract_from_log(f, ignore_automodes) for f in files]
        ave = Trajectory.average(trjs)
        return ave
