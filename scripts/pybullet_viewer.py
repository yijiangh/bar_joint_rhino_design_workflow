"""Interactive PyBullet viewer for the CAD-backed connector chain."""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_planning as pp


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.kinematics import fk_female_side, fk_male_side, optimize_joint_placement
from generate_urdf import DEFAULT_URDF_PATH, LE_BAR_LENGTH, LN_BAR_LENGTH, SCALE, generate_urdf


BASE_POSITION = np.array([0.0, 0.0, 0.3], dtype=float)
BASE_YAW_RAD = -0.5 * math.pi
BASE_ORIENTATION_QUAT = p.getQuaternionFromEuler([0.0, 0.0, BASE_YAW_RAD])
ANNOTATED_LINKS = (
    "le_bar_link",
    "female_link",
    "female_screw_hole_link",
    "male_screw_hole_link",
    "male_link",
    "ln_bar_link",
)
BODY_ALPHA = 0.35
GHOST_ALPHA = 0.5
OPTIMIZATION_MIN_INTERVAL_S = 0.25
HELP_TEXT = """
PyBullet viewer controls:
  Move sliders: Test 1 and Test 2 update continuously.
  Press 1: print the detailed Test 1 FK sanity report for the current slider state.
  Press 2: print the detailed Test 2 optimizer report for the current slider state.
  Press h: print this help again.
""".strip()


def _degrees(bounds_rad: tuple[float, float]) -> tuple[float, float]:
    return tuple(math.degrees(value) for value in bounds_rad)


def _frame_from_pose(position, quaternion) -> np.ndarray:
    frame = np.eye(4, dtype=float)
    frame[:3, :3] = np.array(p.getMatrixFromQuaternion(quaternion), dtype=float).reshape(3, 3)
    frame[:3, 3] = np.asarray(position, dtype=float)
    return frame


def _scaled_frame_translation(frame: np.ndarray, scale_factor: float) -> np.ndarray:
    scaled = np.array(frame, dtype=float, copy=True)
    scaled[:3, 3] *= float(scale_factor)
    return scaled


def _frame_m_to_mm(frame_m: np.ndarray) -> np.ndarray:
    return _scaled_frame_translation(frame_m, 1.0 / SCALE)


def _frame_mm_to_m(frame_mm: np.ndarray) -> np.ndarray:
    return _scaled_frame_translation(frame_mm, SCALE)


def _get_link_frame(robot_id: int, link_map: dict[str, int], link_name: str) -> np.ndarray:
    if link_name == "le_bar_link":
        position, quaternion = p.getBasePositionAndOrientation(robot_id)
    else:
        state = p.getLinkState(robot_id, link_map[link_name], computeForwardKinematics=True)
        position, quaternion = state[4], state[5]
    return _frame_from_pose(position, quaternion)


def _set_body_transparency(robot_id: int, alpha: float = BODY_ALPHA) -> None:
    for visual_shape in p.getVisualShapeData(robot_id):
        link_index = visual_shape[1]
        rgba = visual_shape[7]
        p.changeVisualShape(robot_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], alpha])


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


def _set_joint_state(robot_id: int, joint_map: dict[str, int], values: dict[str, float]) -> None:
    for joint_name, value in values.items():
        joint_index = joint_map[joint_name]
        joint_info = p.getJointInfo(robot_id, joint_index)
        if joint_info[2] == p.JOINT_PRISMATIC:
            p.resetJointState(robot_id, joint_index, value * SCALE)
        else:
            p.resetJointState(robot_id, joint_index, value)


def _rotation_error_deg(frame_a: np.ndarray, frame_b: np.ndarray) -> float:
    relative = frame_a[:3, :3].T @ frame_b[:3, :3]
    cosine = float(np.clip(0.5 * (np.trace(relative) - 1.0), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _frame_match_metrics(predicted_frame_mm: np.ndarray, actual_frame_m: np.ndarray) -> tuple[float, float]:
    actual_frame_mm = _frame_m_to_mm(actual_frame_m)
    position_error_mm = float(np.linalg.norm(predicted_frame_mm[:3, 3] - actual_frame_mm[:3, 3]))
    rotation_error_deg = _rotation_error_deg(predicted_frame_mm, actual_frame_mm)
    return position_error_mm, rotation_error_deg


def _create_overlay_body(filename: str, color: tuple[float, float, float, float]) -> int:
    body = pp.create_obj(os.path.join(SCRIPT_DIR, filename), scale=0.001, collision=False)
    pp.set_color(body, pp.apply_alpha(color, GHOST_ALPHA))
    return body


def _pose_body_from_frame_mm(body: int, frame_mm: np.ndarray) -> None:
    pp.set_pose(body, pp.pose_from_tform(_frame_mm_to_m(frame_mm)))


def _slider_key(values: dict[str, float]) -> tuple[float, float, float, float, float]:
    return tuple(round(float(values[name]), 6) for name in ("fjp_joint", "fjr_joint", "jjr_joint", "mjr_joint", "mjp_joint"))


def _joint_snapshot_line(values: dict[str, float]) -> str:
    return (
        f"FJP={values['fjp_joint']:.3f} mm, "
        f"FJR={math.degrees(values['fjr_joint']):.3f} deg, "
        f"JJR={math.degrees(values['jjr_joint']):.3f} deg, "
        f"MJR={math.degrees(values['mjr_joint']):.3f} deg, "
        f"MJP={values['mjp_joint']:.3f} mm"
    )


def _print_help() -> None:
    print()
    print(HELP_TEXT)


def _print_fk_report(
    slider_values: dict[str, float],
    female_side: dict[str, np.ndarray],
    male_side: dict[str, np.ndarray],
    actual_female_screw_frame_m: np.ndarray,
    actual_male_screw_frame_m: np.ndarray,
) -> None:
    female_fk_pos_err_mm, female_fk_rot_err_deg = _frame_match_metrics(
        female_side["female_screw_hole_frame"],
        actual_female_screw_frame_m,
    )
    male_fk_pos_err_mm, male_fk_rot_err_deg = _frame_match_metrics(
        male_side["male_screw_hole_frame"],
        actual_male_screw_frame_m,
    )
    print()
    print("[Test 1] FK sanity against the live URDF")
    print(f"  Slider state: {_joint_snapshot_line(slider_values)}")
    print(
        f"  female_screw_hole_link: "
        f"pos err={female_fk_pos_err_mm:.6f} mm, rot err={female_fk_rot_err_deg:.6f} deg"
    )
    print(
        f"  male_screw_hole_link: "
        f"pos err={male_fk_pos_err_mm:.6f} mm, rot err={male_fk_rot_err_deg:.6f} deg"
    )
    print("  This check is always live in the status line; key 1 prints this detailed snapshot.")


def _print_optimizer_refresh(slider_values: dict[str, float], opt_result: dict[str, object]) -> None:
    debug = opt_result.get("optimizer_debug")
    if not isinstance(debug, dict):
        return
    best = debug["best_seed_report"]
    print()
    print(
        "[opt] "
        f"{_joint_snapshot_line(slider_values)} | "
        f"best seed {int(debug['best_seed_index']) + 1}/{int(debug['seed_count'])} | "
        f"residual={float(opt_result['residual']):.6e} | "
        f"origin={float(opt_result['origin_error_mm']):.6f} mm | "
        f"z={math.degrees(float(opt_result['z_axis_error_rad'])):.6f} deg | "
        f"nit={int(best['nit'])} | nfev={int(best['nfev'])} | success={bool(best['success'])}"
    )


def _print_optimizer_report(slider_values: dict[str, float], opt_result: dict[str, object]) -> None:
    debug = opt_result.get("optimizer_debug")
    print()
    print("[Test 2] Optimization check and ghost-mesh overlay")
    print(f"  Current slider state: {_joint_snapshot_line(slider_values)}")
    print(
        f"  Solved DOFs: "
        f"FJP={float(opt_result['fjp']):.6f} mm, "
        f"FJR={math.degrees(float(opt_result['fjr'])):.6f} deg, "
        f"MJR={math.degrees(float(opt_result['mjr'])):.6f} deg, "
        f"MJP={float(opt_result['mjp']):.6f} mm"
    )
    print(
        f"  Interface match: "
        f"origin err={float(opt_result['origin_error_mm']):.6f} mm, "
        f"z-axis err={math.degrees(float(opt_result['z_axis_error_rad'])):.6f} deg, "
        f"residual={float(opt_result['residual']):.6e}"
    )
    print("  Ghost meshes: red=female optimizer result, green=male optimizer result.")
    print("  Note: the optimizer intentionally does not solve JJR; twist about the shared screw-hole axis is free.")
    if isinstance(debug, dict):
        best = debug["best_seed_report"]
        print(
            f"  Best run: seed {int(debug['best_seed_index']) + 1}/{int(debug['seed_count'])}, "
            f"success={bool(best['success'])}, nit={int(best['nit'])}, nfev={int(best['nfev'])}"
        )
        print(f"  Best message: {best['message']}")
        sorted_reports = sorted(debug["seed_reports"], key=lambda report: float(report["residual"]))
        for report in sorted_reports[:3]:
            print(
                f"  Seed {int(report['seed_index']) + 1}: "
                f"residual={float(report['residual']):.6e}, "
                f"nit={int(report['nit'])}, nfev={int(report['nfev'])}, success={bool(report['success'])}"
            )


def _was_key_triggered(keys: dict[int, int], *key_codes: int) -> bool:
    return any((keys.get(key_code, 0) & p.KEY_WAS_TRIGGERED) != 0 for key_code in key_codes)


def _status_line(
    female_fk_pos_err_mm: float,
    female_fk_rot_err_deg: float,
    male_fk_pos_err_mm: float,
    male_fk_rot_err_deg: float,
    opt_result: dict[str, float] | None,
) -> str:
    fk_summary = (
        f"FK female: {female_fk_pos_err_mm:6.3f} mm / {female_fk_rot_err_deg:5.2f} deg | "
        f"FK male: {male_fk_pos_err_mm:6.3f} mm / {male_fk_rot_err_deg:5.2f} deg"
    )
    if opt_result is None:
        return f"\r{fk_summary} | Opt: pending"
    return (
        f"\r{fk_summary} | "
        f"Opt interface: {opt_result['origin_error_mm']:6.3f} mm / "
        f"{math.degrees(opt_result['z_axis_error_rad']):5.2f} deg"
    )


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
        _set_body_transparency(robot_id)

        female_body = _create_overlay_body("female_joint_mesh_mm.obj", pp.RED)
        male_body = _create_overlay_body("male_joint_mesh_mm.obj", pp.GREEN)
        _print_help()

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
        last_opt_key = None
        last_opt_time = 0.0
        opt_result = None

        while p.isConnected(physics_client):
            slider_values = _read_slider_values(slider_ids)
            _set_joint_state(robot_id, joint_map, slider_values)

            le_bar_frame_m = _get_link_frame(robot_id, link_map, "le_bar_link")
            ln_bar_frame_m = _get_link_frame(robot_id, link_map, "ln_bar_link")
            le_bar_frame_mm = _frame_m_to_mm(le_bar_frame_m)
            ln_bar_frame_mm = _frame_m_to_mm(ln_bar_frame_m)

            female_side = fk_female_side(le_bar_frame_mm, slider_values["fjp_joint"], slider_values["fjr_joint"], config)
            male_side = fk_male_side(ln_bar_frame_mm, slider_values["mjp_joint"], slider_values["mjr_joint"], config)

            actual_female_screw_frame_m = _get_link_frame(robot_id, link_map, "female_screw_hole_link")
            actual_male_screw_frame_m = _get_link_frame(robot_id, link_map, "male_screw_hole_link")

            female_fk_pos_err_mm, female_fk_rot_err_deg = _frame_match_metrics(
                female_side["female_screw_hole_frame"],
                actual_female_screw_frame_m,
            )
            male_fk_pos_err_mm, male_fk_rot_err_deg = _frame_match_metrics(
                male_side["male_screw_hole_frame"],
                actual_male_screw_frame_m,
            )

            le_start_mm, le_end_mm = _line_endpoints_mm(le_bar_frame_m, LE_BAR_LENGTH)
            ln_start_mm, ln_end_mm = _line_endpoints_mm(ln_bar_frame_m, LN_BAR_LENGTH)

            slider_key = _slider_key(slider_values)
            now = time.time()
            optimizer_reran = False
            if opt_result is None or (slider_key != last_opt_key and (now - last_opt_time) >= OPTIMIZATION_MIN_INTERVAL_S):
                opt_result = optimize_joint_placement(
                    le_start_mm,
                    le_end_mm,
                    ln_start_mm,
                    ln_end_mm,
                    config,
                    return_debug=True,
                )
                last_opt_key = slider_key
                last_opt_time = now
                optimizer_reran = True

            if opt_result is not None:
                _pose_body_from_frame_mm(female_body, opt_result["female_frame"])
                _pose_body_from_frame_mm(male_body, opt_result["male_frame"])
            if optimizer_reran:
                _print_optimizer_refresh(slider_values, opt_result)

            for link_name in ANNOTATED_LINKS:
                _draw_pose(_get_link_frame(robot_id, link_map, link_name), line_ids, link_name)
            _draw_pose(_frame_mm_to_m(female_side["female_screw_hole_frame"]), line_ids, "fk_female_screw", length=0.02)
            _draw_pose(_frame_mm_to_m(male_side["male_screw_hole_frame"]), line_ids, "fk_male_screw", length=0.02)

            keys = p.getKeyboardEvents()
            if _was_key_triggered(keys, ord("h"), ord("H")):
                _print_help()
            if _was_key_triggered(keys, ord("1")):
                _print_fk_report(
                    slider_values,
                    female_side,
                    male_side,
                    actual_female_screw_frame_m,
                    actual_male_screw_frame_m,
                )
            if _was_key_triggered(keys, ord("2")) and opt_result is not None:
                _print_optimizer_report(slider_values, opt_result)

            status = _status_line(
                female_fk_pos_err_mm,
                female_fk_rot_err_deg,
                male_fk_pos_err_mm,
                male_fk_rot_err_deg,
                opt_result,
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
