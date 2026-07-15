#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSRebuildRobotCell - (Re)build the static assembly collision cell.

Scans the document and registers the FULL canonical assembly into the cached
``RobotCell``: every bar (``bar_<bid>``), every joint half
(``joint_<jid>_<sub>``), every environment obstacle mesh on
``LAYER_ENVIRONMENT`` (``obstacle_<name>``), and the two arm tools as
``ToolModel``s (``AT3L`` / ``AT3R``). Pushes the cell to the planner once and
caches a snapshot (+ a cheap fingerprint) so every later IK / ShowIK / export
command reuses it without re-scanning.

It also AUTO-ASSIGNS WalkableGround surfaces to bars: every WalkableGround brep
gets a stable id (``WG0``, ``WG1``, ...) and every bar that has no association
yet is linked to its nearest ground(s) by distance. Bars you already edited with
RSAssignAndShowWalkableGround are left untouched, so manual picks survive a rebuild;
run RSAssignAndShowWalkableGround to override an auto-assignment.

Finally it AUTO-POPULATES a heuristic seed base pose on every bar that has no base
yet (stand a standoff behind the bar, face the average male-joint insertion
direction, on the bar's assigned ground -- the same heuristic as
RSAssignAndShowWalkableGround). So the exported BarAction + every movement therein
already start from a real base instead of a placeholder. Non-destructive: bars
that already carry a base (e.g. an IK-solved one) are kept.

Run this after you ADD / DELETE / MOVE / RESIZE bars, joints, or environment
meshes. The collision cell is a manual snapshot -- edits made after the last
rebuild are NOT reflected until you click this again (IK / ShowIK warn when
they detect the document changed).

Requires PyBullet (RSPBStart).
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import env_collision as _env_collision_module
from core import robot_cell as _robot_cell_module
from core import rhino_walkable_ground as _walkable_module
from core.rhino_bar_registry import repair_on_entry


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    config = importlib.reload(_config_module)
    importlib.reload(_env_collision_module)
    walkable = importlib.reload(_walkable_module)

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.",
            0,
            "RSRebuildRobotCell",
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    repair_on_entry(float(config.BAR_RADIUS), "RSRebuildRobotCell")

    try:
        collision_bodies = robot_cell.rebuild_assembly_cell(rcell, planner)
    except Exception as exc:  # noqa: BLE001 -- surface any build error to the user
        rs.MessageBox(
            f"RSRebuildRobotCell failed:\n{type(exc).__name__}: {exc}",
            0,
            "RSRebuildRobotCell",
        )
        return

    # Auto-assign WalkableGround surfaces to bars (non-destructive; keeps manual
    # picks). Shared with RSExportAllBarActions so both do the same thing.
    n_grounds, n_assigned, n_kept, n_noground = walkable.auto_assign_walkable_ground_ids_all_bars()

    # Auto-populate a heuristic seed base pose on every bar that lacks one, using
    # the shared WalkableGround heuristic (stand behind the bar, face the male-joint
    # insertion direction). NON-DESTRUCTIVE: any bar that already has a base frame
    # (hand-picked in RSIKKeyframe / RSAssignAndShowWalkableGround, or IK-solved) is
    # KEPT -- a human-chosen base always wins over the auto seed. This means the
    # exported BarAction + every movement therein already start from a real base
    # instead of a placeholder identity.
    n_base_pop, n_base_kept, n_base_fail = walkable.auto_populate_base_frames_all_bars()

    n_bar = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "bar")
    n_joint = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "joint")
    n_env = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "environment")
    n_tools = len(rcell.tool_models)
    # WalkableGround summary line: only surface a warning about un-grounded bars
    # when there are grounds to assign but some bar still got none.
    if n_grounds == 0:
        ground_line = "  WalkableGround: none on layer (skipped auto-assign)"
    else:
        ground_line = (
            f"  WalkableGround: {n_grounds} surface(s); "
            f"{n_assigned} bar(s) auto-assigned, {n_kept} kept, {n_noground} still none"
        )
    base_line = (
        f"  Base pose: {n_base_pop} bar(s) auto-populated, {n_base_kept} kept, "
        f"{n_base_fail} skipped (no ground)"
    )
    msg = (
        f"Rebuilt the robot cell:\n"
        f"  {n_bar} bar(s)\n"
        f"  {n_joint} joint half/halves\n"
        f"  {n_env} environment obstacle(s)\n"
        f"  {n_tools} arm tool(s): {', '.join(sorted(rcell.tool_models.keys())) or '-'}\n"
        f"{ground_line}\n"
        f"{base_line}"
    )
    print(
        f"RSRebuildRobotCell: {n_bar} bars, {n_joint} joints, {n_env} obstacles, "
        f"{n_tools} tools; WalkableGround {n_grounds} surf / {n_assigned} assigned / "
        f"{n_kept} kept / {n_noground} none; Base pose {n_base_pop} populated / "
        f"{n_base_kept} kept / {n_base_fail} skipped."
    )
    rs.MessageBox(msg, 0, "RSRebuildRobotCell")


if __name__ == "__main__":
    main()
