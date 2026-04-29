"""Pure-Python replay of an RSIKSupportKeyframe capture (no Rhino).

Runs the support-IK pipeline against a JSON capture written by
`scripts/rs_ik_support_keyframe.py::_save_capture`:

  1. Start the PyBullet client headless (connection_type=direct).
  2. Load the single-arm support RobotCell (Husky-Alice + dual-arm-as-tool).
  3. Configure the dual-arm tool obstacle from `capture["assembled"]`.
  4. Recompute tool0 via `BAR_GRASP_TO_TOOL0[gripper_kind]` (catches drift),
     fall back to the captured `tool0_frame_world_mm` if the table entry
     is missing.
  5. Run `solve_support_ik` for the captured `base_frame_world_mm`.
  6. Compare the extracted SUPPORT_GROUP joint config against
     `capture["expected"]["final"]` within a per-joint tolerance.
  7. Exit 0 on success, 1 on first failure.

Capture schema (schema_version 1, written by `_save_capture`):
    robot_id, captured_at, doc_unit_scale_to_mm
    held_bar_id, assembled_bar_id, gripper_kind
    grasp_frame_world_mm  (4x4 mm)
    tool0_frame_world_mm  (4x4 mm, already composed)
    base_frame_world_mm   (4x4 mm, single-arm support base)
    assembled: {
        base_frame_world_mm, joint_values_left, joint_values_right,
        joint_names_left, joint_names_right
    }
    include_self_collision, include_env_collision
    expected.final: {joint_names: [...], joint_values: [...]}
        (absent on failure-mode captures saved with `_ik_fail` suffix)

Usage:

    python tests/replay_ik_support.py tests/captures/<file>.json [--tol 1e-3]

Cannot be run end-to-end without a real capture JSON, which is produced
only by clicking through `RSIKSupportKeyframe` inside Rhino.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


def _config_diff_max(actual_values, expected_values) -> float:
    a = np.asarray(actual_values, dtype=float)
    e = np.asarray(expected_values, dtype=float)
    return float(np.max(np.abs(a - e)))


def replay_capture(capture_path: str, tol_rad: float = 1e-3, verbose: bool = True) -> int:
    if not os.path.isfile(capture_path):
        print(f"[X] capture not found: {capture_path}")
        return 1

    with open(capture_path, "r", encoding="utf-8") as stream:
        capture = json.load(stream)

    from core import config
    from core import robot_cell as rc
    from core import robot_cell_support as rcs

    if verbose:
        print(f"replay_ik_support: loaded capture {os.path.basename(capture_path)}")
        print(f"                   captured_at   = {capture.get('captured_at', '?')}")
        print(f"                   held_bar_id   = {capture.get('held_bar_id', '?')}")
        print(f"                   assembled_bar = {capture.get('assembled_bar_id', '?')}")
        print(f"                   gripper_kind  = {capture.get('gripper_kind', '?')}")
        print(f"                   collisions    = self={capture['include_self_collision']} "
              f"env={capture['include_env_collision']}")

    grasp_mm = np.array(capture["grasp_frame_world_mm"], dtype=float)
    captured_tool0_mm = np.array(capture["tool0_frame_world_mm"], dtype=float)
    base_mm = np.array(capture["base_frame_world_mm"], dtype=float)
    gripper_kind = capture["gripper_kind"]

    table = config.BAR_GRASP_TO_TOOL0
    if gripper_kind in table and table[gripper_kind] is not None:
        tool0_mm = grasp_mm @ np.asarray(table[gripper_kind], dtype=float)
        if verbose:
            drift = float(np.max(np.abs(tool0_mm - captured_tool0_mm)))
            print(f"replay_ik_support: recomputed tool0 from grasp; max drift vs capture = {drift:.4g} mm")
    else:
        print(
            f"[!] BAR_GRASP_TO_TOOL0['{gripper_kind}'] is missing/empty; "
            "falling back to captured tool0_frame_world_mm."
        )
        tool0_mm = captured_tool0_mm

    check_collision = bool(
        capture["include_self_collision"] or capture["include_env_collision"]
    )

    started_here = False
    if rc.is_pb_running():
        if verbose:
            print("replay_ik_support: PyBullet client already running; reusing it.")
    else:
        if verbose:
            print("replay_ik_support: starting PyBullet (direct, verbose=True) ...")
        rc.start_pb_client(use_gui=False, verbose=True)
        started_here = True

    try:
        cell = rcs.get_or_load_support_cell()
        template = rcs.default_support_cell_state()

        assembled = capture["assembled"]
        template = rcs.configure_dual_arm_obstacle(
            template,
            base_frame_world_mm=np.asarray(assembled["base_frame_world_mm"], dtype=float),
            joint_values_left=assembled["joint_values_left"],
            joint_values_right=assembled["joint_values_right"],
            joint_names_left=assembled["joint_names_left"],
            joint_names_right=assembled["joint_names_right"],
        )

        _client, planner = rc.get_planner()
        rcs.set_cell_state(planner, template)

        if verbose:
            origin = base_mm[:3, 3]
            print(
                f"replay_ik_support: solving support IK at base "
                f"({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
            )

        t0 = time.time()
        state = rcs.solve_support_ik(
            planner, template,
            base_frame_world_mm=base_mm,
            tool0_world_mm=tool0_mm,
            check_collision=check_collision,
        )
        dt = time.time() - t0

        if state is None:
            print(f"[X] support IK failed (took {dt:.2f}s).")
            return 1
        print(f"[OK] support IK solved in {dt:.2f}s.")

        expected_root = capture.get("expected") or {}
        expected = expected_root.get("final")
        if not expected:
            print("  (no `expected.final` in capture; skipping config comparison)")
            print("[OK] replay_ik_support PASS (IK re-solved; no expected to compare)")
            return 0

        actual = rcs.extract_group_config(state, config.SUPPORT_GROUP, cell)
        if actual["joint_names"] != expected["joint_names"]:
            print(
                f"[X] joint_names differ:\n"
                f"    actual   = {actual['joint_names']}\n"
                f"    expected = {expected['joint_names']}"
            )
            print("[X] replay_ik_support FAIL")
            return 1

        max_err = _config_diff_max(actual["joint_values"], expected["joint_values"])
        status = "OK" if max_err <= tol_rad else "X"
        print(f"[{status}] joint values max-error = {max_err:.4g} rad (tol={tol_rad:.4g}).")
        if max_err > tol_rad:
            print("[X] replay_ik_support FAIL")
            return 1
        print("[OK] replay_ik_support PASS")
        return 0
    finally:
        if started_here:
            try:
                rc.stop_pb_client()
            except Exception as exc:
                print(f"replay_ik_support: stop_pb_client raised ({exc})")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="Path to a JSON capture file in tests/captures/.")
    parser.add_argument("--tol", type=float, default=1e-3, help="Joint angle tolerance in radians.")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-step prints.")
    args = parser.parse_args()
    sys.exit(replay_capture(args.capture, tol_rad=args.tol, verbose=not args.quiet))


if __name__ == "__main__":
    main()
