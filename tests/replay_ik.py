"""Pure-Python replay of an RSIKKeyframe capture (no Rhino, no rhinoinside).

Runs the full IK keyframe pipeline against a JSON capture written by
`scripts/rs_ik_keyframe.py::_save_capture`:

  1. Start the PyBullet client headless (connection_type=direct).
  2. Load the dual-arm Husky RobotCell from the in-repo URDF/SRDF submodule.
  3. Recompute the per-arm tool0 frames via `MALE_JOINT_OCF_TO_TOOL0[name][side]`.
  4. Run `solve_dual_arm_ik` for the FINAL target.
  5. Compute the approach offset = -unit(avg(male_z_L, male_z_R)) * LM_DISTANCE.
  6. Run `solve_dual_arm_ik` for the APPROACH target.
  7. Compare extracted left/right joint configs against the captured
     `expected.{final,approach}` within a per-joint tolerance.
  8. Exit 0 on success, 1 on first failure.

Usage:

    python tests/replay_ik.py tests/captures/<file>.json [--tol 1e-3]

This script intentionally avoids any Rhino API. It exists so Claude can
self-debug IK math, OCF -> tool0 dispatch, approach offset, and the IK
solver itself without asking the user to click Rhino buttons.
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

# Pull Rhino's `scaffolding_env` site-env onto sys.path so plain `python.exe`
# can see compas / pybullet / etc. installed via `# r:` directives.
from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


# ---------------------------------------------------------------------------
# Math helpers (mirrors rs_ik_keyframe.py — kept duplicate-but-tiny so this
# module stays Rhino-free without forcing a refactor of the entry script)
# ---------------------------------------------------------------------------


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot unitize a zero-length vector.")
    return np.asarray(vector, dtype=float) / norm


def _translate_frame(frame_mm: np.ndarray, offset_mm: np.ndarray) -> np.ndarray:
    out = np.array(frame_mm, dtype=float, copy=True)
    out[:3, 3] = out[:3, 3] + np.asarray(offset_mm, dtype=float)
    return out


def _tool0_from_ocf(ocf_world_mm: np.ndarray, block_name: str, arm_side: str, table) -> np.ndarray:
    if block_name not in table:
        raise RuntimeError(
            f"MALE_JOINT_OCF_TO_TOOL0 has no entry for '{block_name}'. "
            "Re-run RSExportJointTool0TF in Rhino, or check that "
            "scripts/core/config_generated_ik.py is up to date."
        )
    per_side = table[block_name]
    if arm_side not in per_side:
        raise RuntimeError(
            f"MALE_JOINT_OCF_TO_TOOL0['{block_name}'] has no '{arm_side}' arm entry."
        )
    return np.asarray(ocf_world_mm, dtype=float) @ np.asarray(per_side[arm_side], dtype=float)


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------


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

    # Imports need scripts/ on sys.path; deferred so error messages above are
    # printable even if imports fail later.
    from core import config
    from core import robot_cell as rc

    if verbose:
        print(f"replay_ik: loaded capture {os.path.basename(capture_path)}")
        print(f"           captured_at = {capture.get('captured_at', '?')}")
        print(f"           target bar  = {capture.get('target_bar_id', '?')}")
        print(f"           collisions  = self={capture['include_self_collision']} "
              f"env={capture['include_env_collision']}")

    ocf_left = np.array(capture["left"]["ocf_world_mm"], dtype=float)
    ocf_right = np.array(capture["right"]["ocf_world_mm"], dtype=float)
    base = np.array(capture["base_frame_world_mm"], dtype=float)
    lm_distance = float(capture["lm_distance_mm"])

    # Resolve tool0 frames through the LIVE config table — this catches drift
    # between captured and current MALE_JOINT_OCF_TO_TOOL0.
    table = config.MALE_JOINT_OCF_TO_TOOL0
    tool0_left = _tool0_from_ocf(
        ocf_left, capture["left"]["block_name"], capture["left"]["arm_side"], table
    )
    tool0_right = _tool0_from_ocf(
        ocf_right, capture["right"]["block_name"], capture["right"]["arm_side"], table
    )

    # Approach offset along -unit(avg(male z))
    z_avg = (ocf_left[:3, 2] + ocf_right[:3, 2]) / 2.0
    approach_dir = -_unit(z_avg)
    offset = approach_dir * lm_distance
    tool0_left_app = _translate_frame(tool0_left, offset)
    tool0_right_app = _translate_frame(tool0_right, offset)

    check_collision = bool(
        capture["include_self_collision"] or capture["include_env_collision"]
    )

    if rc.is_pb_running():
        if verbose:
            print("replay_ik: PyBullet client already running; reusing it.")
    else:
        if verbose:
            print("replay_ik: starting PyBullet (direct, verbose=True) ...")
        rc.start_pb_client(use_gui=False, verbose=True)

    started_here = not rc.is_pb_running.__closure__  # placeholder; we always own teardown
    started_here = True  # see finally — we always tear down what we started in this run

    try:
        rcell = rc.get_or_load_robot_cell()
        template = rc.default_cell_state()
        _client, planner = rc.get_planner()

        for phase, target_l, target_r, expected in (
            ("FINAL",    tool0_left,     tool0_right,     capture["expected"]["final"]),
            ("APPROACH", tool0_left_app, tool0_right_app, capture["expected"]["approach"]),
        ):
            if verbose:
                origin = base[:3, 3]
                print(
                    f"replay_ik: solving {phase} IK at base "
                    f"({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
                )
            t0 = time.time()
            state = rc.solve_dual_arm_ik(
                planner, template, base, target_l, target_r,
                check_collision=check_collision,
            )
            dt = time.time() - t0
            if state is None:
                print(f"[X] {phase} IK failed (took {dt:.2f}s).")
                return 1
            print(f"[OK] {phase} IK solved in {dt:.2f}s.")

            for side, group_attr in (("left", "LEFT_GROUP"), ("right", "RIGHT_GROUP")):
                group = getattr(config, group_attr)
                actual = rc.extract_group_config(state, group, rcell)
                exp = expected[side]
                if actual["joint_names"] != exp["joint_names"]:
                    print(
                        f"[X] {phase}/{side} joint_names differ:\n"
                        f"    actual   = {actual['joint_names']}\n"
                        f"    expected = {exp['joint_names']}"
                    )
                    return 1
                max_err = _config_diff_max(actual["joint_values"], exp["joint_values"])
                status = "OK" if max_err <= tol_rad else "X"
                print(f"[{status}] {phase}/{side} joint values max-error = {max_err:.4g} rad "
                      f"(tol={tol_rad:.4g}).")
                if max_err > tol_rad:
                    return 1

        print("[OK] replay_ik PASS")
        return 0
    finally:
        if started_here:
            try:
                rc.stop_pb_client()
            except Exception as exc:
                print(f"replay_ik: stop_pb_client raised ({exc})")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="Path to a JSON capture file in tests/captures/.")
    parser.add_argument("--tol", type=float, default=1e-3, help="Joint angle tolerance in radians.")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-step prints.")
    args = parser.parse_args()
    sys.exit(replay_capture(args.capture, tol_rad=args.tol, verbose=not args.quiet))


if __name__ == "__main__":
    main()
