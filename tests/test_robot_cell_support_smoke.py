"""Headless smoke test: load the support cell with the dual-arm robot
attached as a ToolModel obstacle, freeze it at a captured pose, and verify
the planner accepts the resulting state without raising.

Primary goal: exercise the URDF/SRDF + ToolModel.from_robot_model code path
end-to-end. Collision pass/fail is reported but not asserted — that's
covered by the more targeted unit test in the design plan.

Run:

    python tests/test_robot_cell_support_smoke.py

Exits 0 on pass, 1 on failure.
"""

from __future__ import annotations

import glob
import json
import os
import sys
import traceback


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


def _pick_capture():
    candidates = sorted(glob.glob(os.path.join(TESTS_DIR, "captures", "*.json")), reverse=True)
    for path in candidates:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            continue
        final = (data.get("expected") or {}).get("final") or {}
        if final.get("left", {}).get("joint_values") and final.get("right", {}).get("joint_values"):
            return path, data
    return None, None


def main() -> int:
    capture_path, capture = _pick_capture()
    if capture is None:
        print("[X] no usable capture with full expected.final.{left,right} payload found.")
        return 1
    print(f"smoke: using capture {os.path.basename(capture_path)}")

    from core import robot_cell as rc
    from core import robot_cell_support as rcs

    started_here = False
    try:
        if not rc.is_pb_running():
            print("smoke: starting PyBullet (direct, verbose=True) ...")
            rc.start_pb_client(use_gui=False, verbose=True)
            started_here = True

        cell = rcs.get_or_load_support_cell()
        print(f"smoke: support cell loaded; robot_model.name = {cell.robot_model.name}")
        print(f"       tool_models = {sorted(cell.tool_models.keys())}")

        template = rcs.default_support_cell_state()

        left_final = capture["expected"]["final"]["left"]
        right_final = capture["expected"]["final"]["right"]
        rcs.configure_dual_arm_obstacle(
            template,
            base_frame_world_mm=capture["base_frame_world_mm"],
            joint_values_left=left_final["joint_values"],
            joint_values_right=right_final["joint_values"],
            joint_names_left=left_final["joint_names"],
            joint_names_right=right_final["joint_names"],
        )
        print("smoke: DualArm tool configured at captured pose.")

        _client, planner = rc.get_planner()
        rcs.set_cell_state(planner, template)
        print("smoke: planner.set_robot_cell + set_robot_cell_state succeeded.")

        try:
            planner.check_collision(template)
            print("smoke: check_collision returned (no collision) for default support config.")
        except Exception as exc:
            print(f"smoke: check_collision reports collision (informational): {exc}")

        print("smoke test passed")
        return 0
    except Exception:
        print("[X] smoke test FAILED with exception:")
        traceback.print_exc()
        return 1
    finally:
        if started_here:
            try:
                rc.stop_pb_client()
            except Exception as exc:
                print(f"smoke: stop_pb_client raised ({exc})")


if __name__ == "__main__":
    sys.exit(main())
