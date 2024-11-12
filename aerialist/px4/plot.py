from __future__ import annotations

import os
from statistics import median
from typing import List
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from decouple import config

from .drone_test import DroneTest, DroneTestResult
from .trajectory import Trajectory
from .obstacle import Obstacle
from . import file_helper


class Plot(object):
    DIR = config("RESULTS_DIR", default="results/")
    WEBDAV_DIR = config("WEBDAV_UP_FLD", default=None)
    PLOT_TESTS_XYZ = config("PLOT_TESTS_XYZ", default=True, cast=bool)
    HIGHLIGHT_COLOR = "red"
    HIGHLIGHT_ALPHA = 0.25
    HIGHLIGHT_SIZE = 25
    WAYPOINT_COLOR = "green"
    WAYPOINT_ALPHA = 0.25

    @classmethod
    def plot_test(
        cls,
        test: DroneTest,
        results: List[DroneTestResult],
        obstacle_distance=True,
        filename=None,
    ) -> str:
        distance = None
        if obstacle_distance:
            distance = True
        elif test.assertion is not None and test.assertion.expectation is not None:
            distance = median(
                [r.record.distance(test.assertion.expectation) for r in results]
            )
        if results is not None and len(results) >= 1:
            return cls.plot_trajectory(
                [r.record for r in results],
                goal=None if test.assertion is None else test.assertion.expectation,
                distance=distance,
                obstacles=(
                    None if test.simulation is None else test.simulation.obstacles
                ),
                filename=filename,
                waypoints=(None if test.mission is None else test.mission.waypoints),
            )

    # @classmethod
    # def plot(
    #         self,
    #         goal: Trajectory = None,
    #         save: bool = True,
    #         obstacles: List[Obstacle] = None,
    #         file_prefix="",
    #         highlights: List[float] = None,
    #         filename=None,
    #     ):
    #         distance = True
    #         if goal is not None:
    #             distance = self.distance(goal)
    #         return self.plot_multiple(
    #             [self],
    #             goal,
    #             save,
    #             distance,
    #             highlights,
    #             obstacles,
    #             file_prefix,
    #             None,
    #             filename,
    #         )

    @classmethod
    def plot_trajectory(
        cls,
        trajectories: List[Trajectory],
        goal: Trajectory = None,
        save: bool = True,
        distance: float | bool = None,
        highlights: List[float] = None,
        obstacles: List[Obstacle] = None,
        file_prefix="",
        ave_trajectory: Trajectory = None,
        filename=None,
        waypoints: List[Obstacle.Position] = None,
        gap: float = None,
    ):
        fig = plt.figure(tight_layout=True)

        gs = fig.add_gridspec(4, 4)
        r_plt = fig.add_subplot(gs[3, :2])
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
        r_plt.set_ylabel("Yaw (rad)")
        r_plt.set_xlabel("Time (s)")
        xy_plt.set_ylabel("Y (m)", loc="bottom")
        xy_plt.yaxis.set_label_position("right")
        xy_plt.yaxis.tick_right()
        xy_plt.set_xlabel("X (m)", loc="right")
        xy_plt.set_aspect("equal", "datalim")

        if obstacles is not None:
            for obst in obstacles:
                obst_patch = obst.plt_patch()
                obst_patch.set_label("obstacle")
                xy_plt.add_patch(obst_patch)

        if waypoints is not None:
            for wp in waypoints:
                waypint_patch = mpatches.Circle(
                    (
                        wp.x,
                        wp.y,
                    ),
                    trajectories[0].WAYPOINT_WIDTH,
                    edgecolor=cls.WAYPOINT_COLOR,
                    facecolor=cls.WAYPOINT_COLOR,
                    fill=True,
                    alpha=cls.WAYPOINT_ALPHA,
                    # label="waypoint",
                )
                xy_plt.add_patch(waypint_patch)

            # workaround for circular legend item
            xy_plt.scatter(
                [],
                [],
                color=cls.WAYPOINT_COLOR,
                alpha=cls.WAYPOINT_ALPHA,
                s=49,
                label="waypoint",
            )

        alpha = 1 if len(trajectories) <= 1 else 0.25
        for i in range(len(trajectories)):
            data_frame = trajectories[i].to_data_frame()

            if len(trajectories) <= 1 or cls.PLOT_TESTS_XYZ:
                x_plt.plot(data_frame[:, 0], data_frame[:, 1], alpha=alpha)
                y_plt.plot(data_frame[:, 0], data_frame[:, 2], alpha=alpha)
                z_plt.plot(data_frame[:, 0], data_frame[:, 3], alpha=alpha)
                r_plt.plot(data_frame[:, 0], data_frame[:, 4], alpha=alpha)

            label = None
            if i == 0:
                label = "tests" if len(trajectories) > 1 else "test"

            cls.data_linewidth_plot(
                data_frame[:, 1],
                data_frame[:, 2],
                ax=xy_plt,
                linewidth=trajectories[0].VEHICLE_WIDTH,
                label=label,
                alpha=alpha,
            )

            if highlights is not None:
                for timestamp in highlights:
                    point = data_frame[
                        abs(
                            data_frame[:, 0] - (timestamp / trajectories[0].TIME_SCALE)
                        ).argsort()[0]
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
                ave_trajectory = trajectories[0].average(trajectories)
            data_frame = ave_trajectory.to_data_frame()
            x_plt.plot(
                data_frame[:, 0], data_frame[:, 1], label="test ave.", color="red"
            )
            y_plt.plot(data_frame[:, 0], data_frame[:, 2], color="red")
            z_plt.plot(data_frame[:, 0], data_frame[:, 3], color="red")
            r_plt.plot(data_frame[:, 0], data_frame[:, 4], color="red")
            cls.data_linewidth_plot(
                data_frame[:, 1],
                data_frame[:, 2],
                ax=xy_plt,
                linewidth=trajectories[0].VEHICLE_WIDTH,
                color="red",
            )
        else:
            ave_trajectory = trajectories[0]

        if goal != None:
            data_frame = goal.to_data_frame()
            x_plt.plot(data_frame[:, 0], data_frame[:, 1], color="blue")
            y_plt.plot(data_frame[:, 0], data_frame[:, 2], color="blue")
            z_plt.plot(data_frame[:, 0], data_frame[:, 3], color="blue")
            r_plt.plot(data_frame[:, 0], data_frame[:, 4], color="blue")

            cls.data_linewidth_plot(
                data_frame[:, 1],
                data_frame[:, 2],
                ax=xy_plt,
                linewidth=trajectories[0].VEHICLE_WIDTH,
                color="blue",
                label="baseline",
            )

            # xyz_plt.plot3D(
            #     [p.x for p in goal.positions],
            #     [p.y for p in goal.positions],
            #     [p.z for p in goal.positions],
            # )

        if distance is True and obstacles is not None and len(obstacles) > 0:
            distance = ave_trajectory.min_distance_to_obstacles(obstacles)
        if gap is None and obstacles is not None and len(obstacles) > 1:
            gap = obstacles[0].minimum_gap(obstacles)
        if distance is not None and type(distance) is not bool:
            dist_text = f"distance:{round(distance,2)}"
            if gap is not None:
                dist_text += f" | gap:{round(gap,2)}"
            fig.text(
                0.67,
                0.03,
                dist_text,
                ha="center",
                bbox=dict(facecolor="none", edgecolor="lightgray", boxstyle="round"),
            )
        if highlights is not None:
            for timestamp in highlights:
                x_plt.axvline(
                    timestamp / trajectories[0].TIME_SCALE,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                y_plt.axvline(
                    timestamp / trajectories[0].TIME_SCALE,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                z_plt.axvline(
                    timestamp / trajectories[0].TIME_SCALE,
                    color=cls.HIGHLIGHT_COLOR,
                    alpha=cls.HIGHLIGHT_ALPHA,
                    linewidth=1.75,
                )
                r_plt.axvline(
                    timestamp / trajectories[0].TIME_SCALE,
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
            ncol=3 if obstacles is None else 4,
        )
        if save:
            return cls.save_file(fig, file_prefix, filename)
        else:
            plt.ion()
            plt.show()

    @classmethod
    def save_file(cls, fig, file_prefix, filename):
        if filename is None:
            filename = file_prefix + file_helper.time_filename(add_host=True)
        os.makedirs(cls.DIR, exist_ok=True)
        plot_file = f"{cls.DIR}{filename}.png"
        fig.savefig(plot_file)
        plt.close(fig)
        if cls.WEBDAV_DIR is not None:
            file_helper.upload(f"{cls.DIR}{filename}.png", cls.WEBDAV_DIR)
        return plot_file

    class data_linewidth_plot:
        def __init__(self, x, y, **kwargs):
            self.ax = kwargs.pop("ax", plt.gca())
            self.fig = self.ax.get_figure()
            self.lw_data = kwargs.pop("linewidth", 1)
            self.lw = 1
            self.fig.canvas.draw()

            self.ppd = 72.0 / self.fig.dpi
            self.trans = self.ax.transData.transform
            (self.linehandle,) = self.ax.plot([], [], **kwargs)
            if "label" in kwargs:
                kwargs.pop("label")
            (self.line,) = self.ax.plot(x, y, **kwargs)
            self.line.set_color(self.linehandle.get_color())
            self._resize()
            self.cid = self.fig.canvas.mpl_connect("draw_event", self._resize)

        def _resize(self, event=None):
            lw = ((self.trans((1, self.lw_data)) - self.trans((0, 0))) * self.ppd)[1]
            if lw != self.lw:
                self.line.set_linewidth(lw)
                self.lw = lw
                self._redraw_later()

        def _redraw_later(self):
            self.timer = self.fig.canvas.new_timer(interval=10)
            self.timer.single_shot = True
            self.timer.add_callback(lambda: self.fig.canvas.draw_idle())
            self.timer.start()
