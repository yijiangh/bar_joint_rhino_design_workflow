"""Interactive matplotlib viewer for a hierarchical joint FK chain."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config


BAR_COLOR_EXISTING = "#7f7f7f"
BAR_COLOR_NEW = "#d9a066"
FEMALE_COLOR = "#e67e8c"
MALE_COLOR = "#6fd98c"
AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")


@dataclass(frozen=True)
class ViewerGeometry:
    le_start: np.ndarray
    le_end: np.ndarray
    ln_length: float
    female_box_size: tuple[float, float, float]
    male_box_size: tuple[float, float, float]
    female_visual_offset: float
    male_visual_offset: float


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-12:
        raise ValueError("Cannot normalize a near-zero vector.")
    return vector / norm


def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = _unit(axis)
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def _make_frame(origin: np.ndarray, x_axis: np.ndarray, y_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    frame = np.eye(4, dtype=float)
    frame[:3, 0] = _unit(x_axis)
    frame[:3, 1] = _unit(y_axis)
    frame[:3, 2] = _unit(z_axis)
    frame[:3, 3] = origin
    return frame


def _translate_local(frame: np.ndarray, offset_xyz: tuple[float, float, float]) -> np.ndarray:
    translated = np.array(frame, dtype=float, copy=True)
    translated[:3, 3] = translated[:3, 3] + translated[:3, :3] @ np.asarray(offset_xyz, dtype=float)
    return translated


def _rotate_about_local_axis(frame: np.ndarray, axis_index: int, angle: float) -> np.ndarray:
    rotated = np.array(frame, dtype=float, copy=True)
    axis_world = rotated[:3, axis_index]
    rotation = _rotation_matrix(axis_world, angle)
    rotated[:3, :3] = rotation @ rotated[:3, :3]
    return rotated


def build_viewer_geometry() -> ViewerGeometry:
    return ViewerGeometry(
        le_start=np.array([-220.0, 0.0, 0.0], dtype=float),
        le_end=np.array([220.0, 0.0, 0.0], dtype=float),
        ln_length=420.0,
        female_box_size=(46.0, 28.0, 18.0),
        male_box_size=(40.0, 24.0, 16.0),
        female_visual_offset=-12.0,
        male_visual_offset=12.0,
    )


def box_vertices(frame: np.ndarray, size_xyz: tuple[float, float, float]) -> np.ndarray:
    half_sizes = 0.5 * np.array(size_xyz, dtype=float)
    local_corners = np.array(
        [
            [-1, -1, -1],
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1],
        ],
        dtype=float,
    ) * half_sizes
    rotation = frame[:3, :3]
    origin = frame[:3, 3]
    return local_corners @ rotation.T + origin


def box_faces(vertices: np.ndarray) -> list[np.ndarray]:
    face_indices = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [1, 2, 6, 5],
        [0, 3, 7, 4],
    ]
    return [vertices[index_list] for index_list in face_indices]


def set_equal_aspect(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.55 * np.max(maxs - mins)
    radius = max(radius, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def draw_frame_axes(ax, frame: np.ndarray, scale: float = 35.0) -> None:
    origin = frame[:3, 3]
    for axis_index, color in enumerate(AXIS_COLORS):
        axis_vec = frame[:3, axis_index] * scale
        ax.plot(
            [origin[0], origin[0] + axis_vec[0]],
            [origin[1], origin[1] + axis_vec[1]],
            [origin[2], origin[2] + axis_vec[2]],
            color=color,
            linewidth=2.0,
        )


def compute_chain_state(geometry: ViewerGeometry, values: dict[str, float]) -> dict[str, np.ndarray]:
    le_axis = _unit(geometry.le_end - geometry.le_start)
    base_frame = _make_frame(
        origin=geometry.le_start,
        x_axis=le_axis,
        y_axis=np.array([0.0, 1.0, 0.0], dtype=float),
        z_axis=np.array([0.0, 0.0, 1.0], dtype=float),
    )

    female_mount_point = geometry.le_start + values["fjp"] * le_axis
    female_mount_frame = np.array(base_frame, dtype=float, copy=True)
    female_mount_frame[:3, 3] = female_mount_point
    female_mount_frame = _rotate_about_local_axis(female_mount_frame, 0, values["fjr"])

    female_interface_frame = _translate_local(female_mount_frame, (0.0, 0.0, config.FEMALE_RADIAL_OFFSET))
    female_visual_frame = _translate_local(female_interface_frame, (0.0, 0.0, geometry.female_visual_offset))

    male_joint_frame = _rotate_about_local_axis(female_interface_frame, 2, values["jjr"])
    male_joint_frame = _rotate_about_local_axis(male_joint_frame, 0, values["mjr"])
    male_visual_frame = _translate_local(male_joint_frame, (0.0, 0.0, geometry.male_visual_offset))

    ln_axis_contact = _translate_local(male_joint_frame, (0.0, 0.0, config.MALE_RADIAL_OFFSET))
    ln_axis_direction = ln_axis_contact[:3, 0]
    ln_axis_point = ln_axis_contact[:3, 3]
    ln_start = ln_axis_point - values["mjp"] * ln_axis_direction
    ln_end = ln_start + geometry.ln_length * ln_axis_direction
    ln_mid = 0.5 * (ln_start + ln_end)
    ln_frame = _make_frame(
        origin=ln_mid,
        x_axis=ln_axis_contact[:3, 0],
        y_axis=ln_axis_contact[:3, 1],
        z_axis=ln_axis_contact[:3, 2],
    )

    return {
        "base_frame": base_frame,
        "female_mount_point": female_mount_point,
        "female_interface_frame": female_interface_frame,
        "female_visual_frame": female_visual_frame,
        "male_joint_frame": male_joint_frame,
        "male_visual_frame": male_visual_frame,
        "ln_axis_point": ln_axis_point,
        "ln_frame": ln_frame,
        "ln_start": ln_start,
        "ln_end": ln_end,
    }


def main() -> None:
    geometry = build_viewer_geometry()
    initial_state = {
        "fjp": 240.0,
        "fjr": np.deg2rad(55.0),
        "mjp": 180.0,
        "mjr": np.deg2rad(110.0),
        "jjr": np.deg2rad(-25.0),
    }

    plt.close("all")
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_axes([0.05, 0.18, 0.62, 0.78], projection="3d")
    ax.set_title("Hierarchical T20-5 FK Viewer")

    slider_specs = [
        ("fjp", "FJP", config.FJP_RANGE, initial_state["fjp"], "%.1f mm"),
        ("fjr", "FJR", config.FJR_RANGE, initial_state["fjr"], "%.3f rad"),
        ("mjp", "MJP", config.MJP_RANGE, initial_state["mjp"], "%.1f mm"),
        ("mjr", "MJR", config.MJR_RANGE, initial_state["mjr"], "%.3f rad"),
        ("jjr", "JJR", config.JJR_RANGE, initial_state["jjr"], "%.3f rad"),
    ]

    sliders = {}
    slider_top = 0.86
    slider_step = 0.105
    for index, (key, label, bounds, initial_value, valfmt) in enumerate(slider_specs):
        slider_ax = fig.add_axes([0.74, slider_top - index * slider_step, 0.22, 0.03])
        sliders[key] = Slider(slider_ax, label, bounds[0], bounds[1], valinit=initial_value, valfmt=valfmt)

    status_text = fig.text(0.72, 0.24, "", fontsize=11, family="monospace")
    guide_text = fig.text(
        0.72,
        0.13,
        "Hierarchy:\nGrey bar (fixed base) -> Female -> Male -> Orange bar\n"
        "Upstream motion propagates to all children.",
        fontsize=10,
    )
    guide_text.set_bbox({"facecolor": "white", "alpha": 0.9, "edgecolor": "#cccccc"})

    reset_ax = fig.add_axes([0.74, 0.05, 0.1, 0.05])
    reset_button = Button(reset_ax, "Reset")
    zero_ax = fig.add_axes([0.86, 0.05, 0.1, 0.05])
    zero_button = Button(zero_ax, "Zero")

    def current_values() -> dict[str, float]:
        return {key: slider.val for key, slider in sliders.items()}

    def redraw(_=None):
        values = current_values()
        state = compute_chain_state(geometry, values)

        ax.cla()
        ax.set_title("Hierarchical T20-5 FK Viewer")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.plot(
            [geometry.le_start[0], geometry.le_end[0]],
            [geometry.le_start[1], geometry.le_end[1]],
            [geometry.le_start[2], geometry.le_end[2]],
            color=BAR_COLOR_EXISTING,
            linewidth=5.0,
        )
        ax.plot(
            [state["ln_start"][0], state["ln_end"][0]],
            [state["ln_start"][1], state["ln_end"][1]],
            [state["ln_start"][2], state["ln_end"][2]],
            color=BAR_COLOR_NEW,
            linewidth=5.0,
        )

        female_box = box_vertices(state["female_visual_frame"], geometry.female_box_size)
        male_box = box_vertices(state["male_visual_frame"], geometry.male_box_size)
        ax.add_collection3d(
            Poly3DCollection(box_faces(female_box), facecolors=FEMALE_COLOR, edgecolors="#8a3d4a", alpha=0.65)
        )
        ax.add_collection3d(
            Poly3DCollection(box_faces(male_box), facecolors=MALE_COLOR, edgecolors="#287c4e", alpha=0.65)
        )

        draw_frame_axes(ax, state["female_interface_frame"], scale=32.0)
        draw_frame_axes(ax, state["male_joint_frame"], scale=32.0)
        draw_frame_axes(ax, state["ln_frame"], scale=28.0)

        female_chain = np.vstack([state["female_mount_point"], state["female_interface_frame"][:3, 3]])
        male_chain = np.vstack([state["female_interface_frame"][:3, 3], state["male_joint_frame"][:3, 3]])
        bar_chain = np.vstack([state["male_joint_frame"][:3, 3], state["ln_axis_point"]])
        ax.plot(female_chain[:, 0], female_chain[:, 1], female_chain[:, 2], color="#7a2233", linestyle="--", linewidth=1.5)
        ax.plot(male_chain[:, 0], male_chain[:, 1], male_chain[:, 2], color="#17653b", linestyle="--", linewidth=1.5)
        ax.plot(bar_chain[:, 0], bar_chain[:, 1], bar_chain[:, 2], color="#444444", linestyle="--", linewidth=1.5)

        ax.scatter(*state["female_mount_point"], color="#7a2233", s=28)
        ax.scatter(*state["ln_axis_point"], color="#8a5b14", s=28)

        ax.text(*(0.5 * (geometry.le_start + geometry.le_end) + np.array([0.0, 12.0, 0.0])), "Le", color=BAR_COLOR_EXISTING, fontsize=12)
        ax.text(*(0.5 * (state["ln_start"] + state["ln_end"]) + np.array([0.0, 12.0, 0.0])), "Ln", color="#8a5b14", fontsize=12)
        ax.text(*state["female_visual_frame"][:3, 3], "Female", color="#7a2233", fontsize=12)
        ax.text(*state["male_visual_frame"][:3, 3], "Male", color="#17653b", fontsize=12)

        points_for_bounds = np.vstack(
            [
                geometry.le_start,
                geometry.le_end,
                state["ln_start"],
                state["ln_end"],
                female_box,
                male_box,
            ]
        )
        set_equal_aspect(ax, points_for_bounds)
        ax.view_init(elev=23.0, azim=-58.0)

        status_text.set_text(
            "Chain Mode\n"
            "FJP: {:8.2f} mm\n"
            "FJR: {:8.3f} rad\n"
            "MJP: {:8.2f} mm\n"
            "MJR: {:8.3f} rad\n"
            "JJR: {:8.3f} rad".format(
                values["fjp"],
                values["fjr"],
                values["mjp"],
                values["mjr"],
                values["jjr"],
            )
        )
        fig.canvas.draw_idle()

    def reset_to_default(_event):
        for key, *_rest in slider_specs:
            sliders[key].set_val(initial_state[key])

    def zero_all(_event):
        for key in sliders:
            sliders[key].set_val(0.0)

    for slider in sliders.values():
        slider.on_changed(redraw)
    reset_button.on_clicked(reset_to_default)
    zero_button.on_clicked(zero_all)

    redraw()
    plt.show()


if __name__ == "__main__":
    main()
