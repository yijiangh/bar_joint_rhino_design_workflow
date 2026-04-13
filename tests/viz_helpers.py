"""Shared visualization helpers for pytest-based geometric debugging."""

from __future__ import annotations

from typing import Iterable

import numpy as np


class VizContext:
    """Collect draw commands during a test and render them at teardown."""

    def __init__(self, enabled: bool):
        self.enabled = bool(enabled)
        self._commands: list[tuple[str, dict]] = []

    def plot_line(self, start, end, color="black", linewidth=2, label=None, linestyle="-"):
        if not self.enabled:
            return
        self._commands.append(
            (
                "line",
                {
                    "start": np.asarray(start, dtype=float),
                    "end": np.asarray(end, dtype=float),
                    "color": color,
                    "linewidth": linewidth,
                    "label": label,
                    "linestyle": linestyle,
                },
            )
        )

    def plot_point(self, point, color="red", size=40, label=None):
        if not self.enabled:
            return
        self._commands.append(
            (
                "point",
                {
                    "point": np.asarray(point, dtype=float),
                    "color": color,
                    "size": size,
                    "label": label,
                },
            )
        )

    def plot_segment(self, start, end, color="blue", linewidth=1.5, linestyle="--", label=None):
        self.plot_line(start, end, color=color, linewidth=linewidth, linestyle=linestyle, label=label)

    def plot_frame(self, frame_4x4, scale=20.0, label=None):
        if not self.enabled:
            return
        self._commands.append(
            (
                "frame",
                {
                    "frame": np.asarray(frame_4x4, dtype=float),
                    "scale": float(scale),
                    "label": label,
                },
            )
        )

    def plot_box_wireframe(self, frame_4x4, size_xyz, color="pink", label=None):
        if not self.enabled:
            return
        self._commands.append(
            (
                "box",
                {
                    "frame": np.asarray(frame_4x4, dtype=float),
                    "size": np.asarray(size_xyz, dtype=float),
                    "color": color,
                    "label": label,
                },
            )
        )

    def plot_text(self, point, text, color="black", fontsize=10):
        if not self.enabled:
            return
        self._commands.append(
            (
                "text",
                {
                    "point": np.asarray(point, dtype=float),
                    "text": str(text),
                    "color": color,
                    "fontsize": fontsize,
                },
            )
        )

    def set_title(self, title: str):
        if not self.enabled:
            return
        self._commands.append(("title", {"title": str(title)}))

    def show(self):
        if not self.enabled or not self._commands:
            return

        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        all_points: list[np.ndarray] = []
        title = ""
        frame_label_used = False
        box_label_used = False

        for cmd_type, data in self._commands:
            if cmd_type == "line":
                start = data["start"]
                end = data["end"]
                ax.plot(
                    [start[0], end[0]],
                    [start[1], end[1]],
                    [start[2], end[2]],
                    color=data["color"],
                    linewidth=data["linewidth"],
                    linestyle=data["linestyle"],
                    label=data["label"],
                )
                all_points.extend([start, end])
            elif cmd_type == "point":
                point = data["point"]
                ax.scatter(*point, color=data["color"], s=data["size"], label=data["label"], zorder=5)
                all_points.append(point)
            elif cmd_type == "frame":
                frame = data["frame"]
                origin = frame[:3, 3]
                scale = data["scale"]
                colors = ("red", "green", "blue")
                axis_labels = ("X", "Y", "Z")
                plot_label = data["label"] if data["label"] and not frame_label_used else None
                frame_label_used = frame_label_used or bool(plot_label)
                for axis_index, color in enumerate(colors):
                    tip = origin + scale * frame[:3, axis_index]
                    ax.plot(
                        [origin[0], tip[0]],
                        [origin[1], tip[1]],
                        [origin[2], tip[2]],
                        color=color,
                        linewidth=2,
                        label=plot_label if axis_index == 0 else None,
                    )
                    ax.text(*tip, axis_labels[axis_index], color=color, fontsize=8)
                    all_points.append(tip)
                if data["label"]:
                    ax.text(*origin, data["label"], fontsize=9, color="black")
                all_points.append(origin)
            elif cmd_type == "box":
                frame = data["frame"]
                size = data["size"]
                origin = frame[:3, 3]
                rotation = frame[:3, :3]
                half = size / 2.0
                corners_local = np.array(
                    [
                        [-1.0, -1.0, -1.0],
                        [1.0, -1.0, -1.0],
                        [1.0, 1.0, -1.0],
                        [-1.0, 1.0, -1.0],
                        [-1.0, -1.0, 1.0],
                        [1.0, -1.0, 1.0],
                        [1.0, 1.0, 1.0],
                        [-1.0, 1.0, 1.0],
                    ],
                    dtype=float,
                ) * half
                corners_world = (rotation @ corners_local.T).T + origin
                edges = (
                    (0, 1),
                    (1, 2),
                    (2, 3),
                    (3, 0),
                    (4, 5),
                    (5, 6),
                    (6, 7),
                    (7, 4),
                    (0, 4),
                    (1, 5),
                    (2, 6),
                    (3, 7),
                )
                plot_label = data["label"] if data["label"] and not box_label_used else None
                box_label_used = box_label_used or bool(plot_label)
                for edge_index, (start_index, end_index) in enumerate(edges):
                    ax.plot(
                        [corners_world[start_index, 0], corners_world[end_index, 0]],
                        [corners_world[start_index, 1], corners_world[end_index, 1]],
                        [corners_world[start_index, 2], corners_world[end_index, 2]],
                        color=data["color"],
                        linewidth=0.8,
                        label=plot_label if edge_index == 0 else None,
                    )
                all_points.extend(corners_world.tolist())
            elif cmd_type == "text":
                ax.text(*data["point"], data["text"], color=data["color"], fontsize=data["fontsize"])
                all_points.append(data["point"])
            elif cmd_type == "title":
                title = data["title"]

        if all_points:
            self._set_equal_limits(ax, all_points)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        if title:
            ax.set_title(title)
        handles, labels = ax.get_legend_handles_labels()
        if any(label for label in labels):
            ax.legend(loc="upper left", fontsize=8)
        plt.tight_layout()
        plt.show()

    @staticmethod
    def _set_equal_limits(ax, points: Iterable[np.ndarray]):
        pts = np.asarray(list(points), dtype=float)
        center = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
        radius = max(0.55 * np.max(pts.max(axis=0) - pts.min(axis=0)), 1.0)
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)
