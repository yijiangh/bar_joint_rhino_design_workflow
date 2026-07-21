#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
# r: txaio
"""RSPBStart - Start the shared PyBullet client for IK / FK workflows.

Left-click starts a Direct (headless) connection; the RSPBStartGUI right-click
entry starts a GUI connection. Either way the dual-arm Husky robot cell is
loaded into a cached PyBullet planner. Subsequent IK scripts (RSIKKeyframe,
RSShowIK) reuse this same client via `sc.sticky`.

After the client is up, this also auto-builds the static assembly collision
cell (the same work as the RSRebuildRobotCell button, including the
WalkableGround auto-assign + seed-base passes), so you no longer have to click
RSRebuildRobotCell by hand before the first IK / ShowIK / export. On success it
pops up the same summary window RSRebuildRobotCell shows. The auto-build is
best-effort: if it can't run yet (e.g. the two arm tools aren't defined), it
just warns and leaves PyBullet running -- click RSRebuildRobotCell later once
the geometry + tools exist.
"""

from __future__ import annotations

import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.robot_cell import (
    get_or_load_robot_cell,
    is_pb_running,
    rebuild_assembly_cell,
    start_pb_client,
)
from core.rhino_bar_registry import repair_on_entry
from core.rhino_walkable_ground import rebuild_summary_after_assign


def main(use_gui: bool = False) -> None:
    if is_pb_running():
        print("RSPBStart: PyBullet client already running. Run RSPBStop first if you want to restart.")
        return

    print(f"RSPBStart: Starting PyBullet ({'GUI' if use_gui else 'Direct'}) and loading robot cell...")
    client, planner = start_pb_client(use_gui=use_gui)
    print(f"RSPBStart: Robot cell loaded. Client id={client.client_id}.")

    # ! Build the static assembly cell right after startup so the user doesn't
    # ! have to click RSRebuildRobotCell manually before the first IK command.
    _auto_build_robot_cell(planner)


def _auto_build_robot_cell(planner) -> None:
    """Build the static assembly cell (same work as the RSRebuildRobotCell button).

    Kept as its own function and wrapped in try/except so any build failure --
    most commonly "the two arm tools aren't defined yet" -- is a non-fatal
    warning rather than aborting the PyBullet session that just started fine.
    The manual RSRebuildRobotCell button still works for later geometry edits.

    On success it shows the SAME pop-up summary as RSRebuildRobotCell (bar /
    joint / obstacle / tool counts + WalkableGround + seed-base lines), built by
    the shared ``rhino_walkable_ground.rebuild_summary_after_assign`` helper so
    the two windows never drift apart.
    """
    rcell = get_or_load_robot_cell()
    try:
        # ! Same bar-registry hygiene the manual RSRebuildRobotCell runs on entry.
        repair_on_entry(float(config.BAR_RADIUS), "RSPBStart")
        collision_bodies = rebuild_assembly_cell(rcell, planner)
        # ! Same WalkableGround auto-assign + seed-base passes + summary text that
        # ! RSRebuildRobotCell produces, so the pop-up window matches exactly.
        msg, console_line = rebuild_summary_after_assign(
            collision_bodies, sorted(rcell.tool_models.keys())
        )
    except Exception as exc:  # noqa: BLE001 -- never let a cell-build issue kill startup
        print(
            f"RSPBStart: skipped auto-building the robot cell "
            f"({type(exc).__name__}: {exc}). "
            "Define the arm tools + geometry, then click RSRebuildRobotCell."
        )
        return

    # ---- Same pop-up window + console echo as the RSRebuildRobotCell button.
    print(f"RSPBStart: {console_line}")
    rs.MessageBox(msg, 0, "RSPBStart")


if __name__ == "__main__":
    main(use_gui=False)
