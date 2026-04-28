"""Replay an RSIKKeyframe capture with `rhinoinside` so the scene-viz path runs.

Same IK pipeline as `replay_ik.py`, then additionally exercises
`core.ik_viz.show_state(...)` for both the FINAL and APPROACH solutions
so any regression in the Rhino-side mesh-bake / scene-object code path
is caught from a plain terminal — no manual click required.

Usage:

    python tests/replay_ik_with_viz.py tests/captures/<file>.json

Caveats:

  * Requires `pip install rhinoinside` and a local Rhino 8 install.
  * `rhinoinside.load()` boots Rhino's runtime in this CPython process; it
    is Windows-only and adds ~3-5 seconds startup.
  * `compas_robots.rhino.scene` registers `RobotModelObject` for the Rhino
    context only AFTER `Rhino.RhinoApp` is alive; we trigger the
    registration explicitly post-load.
  * Anything that reads `scriptcontext.doc` UI state (interactive picks,
    layer prompts) is still skipped — those paths are exercised only when
    the script runs inside the real ScriptEditor.
"""

from __future__ import annotations

import argparse
import os
import sys
import time


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


def _bootstrap_rhinoinside() -> None:
    """Load Rhino runtime via rhinoinside and stub out missing scriptcontext bits.

    Raises a helpful error if `rhinoinside` is not installed.
    """
    try:
        import rhinoinside  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "replay_ik_with_viz: `rhinoinside` is not installed. "
            "Install with `pip install rhinoinside` and ensure Rhino 8 is on this machine."
        ) from exc

    rhinoinside.load()
    import Rhino  # noqa: F401  - validates the load
    import types

    # `scriptcontext` is part of rhinoscriptsyntax; rhinoinside ships
    # rhinoscriptsyntax stubs but `sticky` and `doc` may need hand-feeding.
    sc = sys.modules.get("scriptcontext")
    if sc is None:
        sc = types.ModuleType("scriptcontext")
        sys.modules["scriptcontext"] = sc
    if not hasattr(sc, "sticky") or sc.sticky is None:
        sc.sticky = {}
    if not hasattr(sc, "doc") or sc.doc is None:
        try:
            sc.doc = Rhino.RhinoDoc.ActiveDoc  # type: ignore[name-defined]
        except Exception:
            sc.doc = None  # last resort — ik_viz will hit AttributeError loudly

    # Trigger compas_robots Rhino scene plugin registration (its `@plugin`
    # decorator runs when the module is imported).
    try:
        import compas_robots.rhino.scene  # noqa: F401
    except Exception as exc:
        print(f"replay_ik_with_viz: compas_robots.rhino.scene import failed ({exc}); "
              "RobotModel SceneObject may not be registered for Rhino context.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", help="Path to a JSON capture file in tests/captures/.")
    parser.add_argument("--tol", type=float, default=1e-3, help="Joint angle tolerance in radians.")
    args = parser.parse_args()

    _bootstrap_rhinoinside()

    # Now that Rhino's runtime is alive, import the project modules. Importing
    # before rhinoinside.load() would crash in `core.robot_cell` if it tried
    # to bind RhinoCommon types eagerly.
    import numpy as np  # noqa: F401
    from core import config
    from core import robot_cell as rc
    from core import ik_viz

    from tests.replay_ik import (  # type: ignore  - sibling import
        _tool0_from_ocf,
        _translate_frame,
        _unit,
    )
    import json

    with open(args.capture, "r", encoding="utf-8") as stream:
        capture = json.load(stream)

    ocf_left = np.array(capture["left"]["ocf_world_mm"], dtype=float)
    ocf_right = np.array(capture["right"]["ocf_world_mm"], dtype=float)
    base = np.array(capture["base_frame_world_mm"], dtype=float)
    lm_distance = float(capture["lm_distance_mm"])
    table = config.MALE_JOINT_OCF_TO_TOOL0
    tool0_l = _tool0_from_ocf(ocf_left, capture["left"]["block_name"], capture["left"]["arm_side"], table)
    tool0_r = _tool0_from_ocf(ocf_right, capture["right"]["block_name"], capture["right"]["arm_side"], table)
    z_avg = (ocf_left[:3, 2] + ocf_right[:3, 2]) / 2.0
    offset = -_unit(z_avg) * lm_distance
    tool0_l_app = _translate_frame(tool0_l, offset)
    tool0_r_app = _translate_frame(tool0_r, offset)
    check_collision = bool(capture["include_self_collision"] or capture["include_env_collision"])

    if not rc.is_pb_running():
        print("replay_ik_with_viz: starting PyBullet (direct) ...")
        rc.start_pb_client(use_gui=False, verbose=True)

    try:
        rcell = rc.get_or_load_robot_cell()
        template = rc.default_cell_state()
        _client, planner = rc.get_planner()

        for phase, target_l, target_r in (
            ("FINAL",    tool0_l,     tool0_r),
            ("APPROACH", tool0_l_app, tool0_r_app),
        ):
            print(f"replay_ik_with_viz: {phase} IK ...")
            t0 = time.time()
            state = rc.solve_dual_arm_ik(
                planner, template, base, target_l, target_r,
                check_collision=check_collision,
            )
            if state is None:
                print(f"[X] {phase} IK failed.")
                return 1
            print(f"[OK] {phase} IK solved in {time.time() - t0:.2f}s.")

            try:
                ik_viz.show_state(state)
                print(f"[OK] {phase} ik_viz.show_state() returned without raising.")
            except Exception as exc:
                print(f"[X] {phase} ik_viz.show_state() raised: {exc}")
                return 1

        ik_viz.clear_scene()
        print("[OK] replay_ik_with_viz PASS")
        return 0
    finally:
        try:
            rc.stop_pb_client()
        except Exception as exc:
            print(f"replay_ik_with_viz: stop_pb_client raised ({exc})")


if __name__ == "__main__":
    sys.exit(main())
