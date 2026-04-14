"""Interactive PyBullet viewer for the CAD-backed connector chain."""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.kinematics import fk_female_side, fk_male_side, frame_distance, predict_male_from_female
from generate_urdf import DEFAULT_URDF_PATH, LE_BAR_LENGTH, LN_BAR_LENGTH, SCALE, generate_urdf


BASE_POSITION = np.array([0.0, 0.0, 0.3], dtype=float)
BASE_YAW_RAD = -0.5 * math.pi
BASE_ORIENTATION_QUAT = p.getQuaternionFromEuler([0.0, 0.0, BASE_YAW_RAD])
ANNOTATED_LINKS = ("female_link", "female_screw_hole_link", "male_screw_hole_link", "male_link", "ln_bar_link")


def _degrees(bounds_rad: tuple[float, float]) -> tuple[float, float]:
    return tuple(math.degrees(value) for value in bounds_rad)


def _frame_from_pose(position, quaternion) -> np.ndarray:
    frame = np.eye(4, dtype=float)
    frame[:3, :3] = np.array(p.getMatrixFromQuaternion(quaternion), dtype=float).reshape(3, 3)
    frame[:3, 3] = np.asarray(position, dtype=float)
    return frame


def _get_link_frame(robot_id: int, link_map: dict[str, int], link_name: str) -> np.ndarray:
    if link_name == "le_bar_link":
        position, quaternion = p.getBasePositionAndOrientation(robot_id)
    else:
        state = p.getLinkState(robot_id, link_map[link_name], computeForwardKinematics=True)
        position, quaternion = state[0], state[1]
    return _frame_from_pose(position, quaternion)


def _line_endpoints_mm(start_frame_m: np.ndarray, length_mm: float) -> tuple[np.ndarray, np.ndarray]:
    start_mm = start_frame_m[:3, 3] / SCALE
    axis = start_frame_m[:3, 2]
    end_mm = start_mm + length_mm * axis
    return start_mm, end_mm


def _draw_pose(frame_m: np.ndarray, line_ids: dict[str, int], prefix: str, length: float = 0.03) -> None:
    origin = frame_m[:3, 3]
    axes = [
        ("x", frame_m[:3, 0], [1, 0, 0]),
        ("y", frame_m[:3, 1], [0, 1, 0]),
        ("z", frame_m[:3, 2], [0, 0, 1]),
    ]
    for axis_name, direction, color in axes:
        key = f"{prefix}_{axis_name}"
        tip = origin + length * direction
        line_ids[key] = p.addUserDebugLine(
            origin.tolist(),
            tip.tolist(),
            color,
            lineWidth=3.0,
            lifeTime=0.0,
            replaceItemUniqueId=line_ids.get(key, -1),
        )


def _read_slider_values(slider_ids: dict[str, tuple[int, str]]) -> dict[str, float]:
    values = {}
    for joint_name, (slider_id, joint_kind) in slider_ids.items():
        raw = p.readUserDebugParameter(slider_id)
        values[joint_name] = raw if joint_kind == "prismatic" else math.radians(raw)
    return values


def _apply_joint_values(robot_id: int, joint_map: dict[str, int], values: dict[str, float]) -> None:
    for joint_name, value in values.items():
        joint_index = joint_map[joint_name]
        p.resetJointState(robot_id, joint_index, value if "joint" in joint_name and joint_name in ("fjp_joint", "mjp_joint") else value)


def _set_joint_state(robot_id: int, joint_map: dict[str, int], values: dict[str, float]) -> None:
    for joint_name, value in values.items():
        joint_index = joint_map[joint_name]
        joint_info = p.getJointInfo(robot_id, joint_index)
        if joint_info[2] == p.JOINT_PRISMATIC:
            p.resetJointState(robot_id, joint_index, value * SCALE)
        else:
            p.resetJointState(robot_id, joint_index, value)


def _status_line(
    female_screw_predicted_m: np.ndarray,
    male_screw_actual_m: np.ndarray,
    predicted_male_frame_m: np.ndarray,
    actual_male_frame_m: np.ndarray,
) -> str:
    screw_err_mm = float(np.linalg.norm(female_screw_predicted_m[:3, 3] - male_screw_actual_m[:3, 3])) / SCALE
    male_err = frame_distance(predicted_male_frame_m, actual_male_frame_m)
    return f"\rScrew-hole pos err: {screw_err_mm:7.3f} mm | Male-frame mismatch: {male_err:9.4e}"


def main() -> None:
    urdf_path = generate_urdf(DEFAULT_URDF_PATH)
    physics_client = p.connect(p.GUI)

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=BASE_POSITION.tolist(),
            baseOrientation=BASE_ORIENTATION_QUAT,
            useFixedBase=True,
        )

        joint_map: dict[str, int] = {}
        link_map: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            link_name = joint_info[12].decode("utf-8")
            joint_type = joint_info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                joint_map[joint_name] = joint_index
            link_map[link_name] = joint_index

        slider_specs = [
            ("fjp_joint", "FJP (mm)", config.FJP_RANGE[0], config.FJP_RANGE[1], 0.0, "prismatic"),
            ("fjr_joint", "FJR (deg)", *_degrees(config.FJR_RANGE), 0.0, "revolute"),
            ("jjr_joint", "JJR (deg)", *_degrees(config.JJR_RANGE), 0.0, "revolute"),
            ("mjr_joint", "MJR (deg)", *_degrees(config.MJR_RANGE), 0.0, "revolute"),
            ("mjp_joint", "MJP (mm)", config.MJP_RANGE[0], config.MJP_RANGE[1], 0.0, "prismatic"),
        ]

        slider_ids: dict[str, tuple[int, str]] = {}
        for joint_name, label, lower, upper, default, joint_kind in slider_specs:
            slider_ids[joint_name] = (p.addUserDebugParameter(label, lower, upper, default), joint_kind)

        line_ids: dict[str, int] = {}
        last_status_line = ""
        last_status_width = 0

        while p.isConnected(physics_client):
            slider_values = _read_slider_values(slider_ids)
            _set_joint_state(robot_id, joint_map, slider_values)

            le_bar_frame_m = _get_link_frame(robot_id, link_map, "le_bar_link")
            ln_bar_frame_m = _get_link_frame(robot_id, link_map, "ln_bar_link")
            le_start_mm, le_end_mm = _line_endpoints_mm(le_bar_frame_m, LE_BAR_LENGTH)
            ln_start_mm, ln_end_mm = _line_endpoints_mm(ln_bar_frame_m, LN_BAR_LENGTH)

            female_side = fk_female_side(le_start_mm, le_end_mm, slider_values["fjp_joint"], slider_values["fjr_joint"], config)
            male_side = fk_male_side(ln_start_mm, ln_end_mm, slider_values["mjp_joint"], slider_values["mjr_joint"], config)
            predicted = predict_male_from_female(female_side["female_screw_hole_frame"], slider_values["jjr_joint"], config)

            female_screw_predicted_m = np.array(predicted["predicted_male_screw_hole_frame"], dtype=float, copy=True)
            female_screw_predicted_m[:3, 3] *= SCALE
            predicted_male_frame_m = np.array(predicted["predicted_male_frame"], dtype=float, copy=True)
            predicted_male_frame_m[:3, 3] *= SCALE

            actual_male_screw_frame_m = _get_link_frame(robot_id, link_map, "male_screw_hole_link")
            actual_male_frame_m = _get_link_frame(robot_id, link_map, "male_link")

            for link_name in ANNOTATED_LINKS:
                _draw_pose(_get_link_frame(robot_id, link_map, link_name), line_ids, link_name)

            status = _status_line(
                female_screw_predicted_m,
                actual_male_screw_frame_m,
                predicted_male_frame_m,
                actual_male_frame_m,
            )
            padding = max(0, last_status_width - len(status))
            print(status + (" " * padding), end="", flush=True)
            last_status_line = status
            last_status_width = len(status)
            time.sleep(1.0 / 120.0)
    finally:
        if last_status_line:
            print()
        p.disconnect(physics_client)


if __name__ == "__main__":
    main()
