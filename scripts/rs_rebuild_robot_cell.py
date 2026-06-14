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
from core.rhino_bar_registry import repair_on_entry


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    config = importlib.reload(_config_module)
    importlib.reload(_env_collision_module)

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

    n_bar = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "bar")
    n_joint = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "joint")
    n_env = sum(1 for bi in collision_bodies.values() if bi.get("kind") == "environment")
    n_tools = len(rcell.tool_models)
    msg = (
        f"Rebuilt the robot cell:\n"
        f"  {n_bar} bar(s)\n"
        f"  {n_joint} joint half/halves\n"
        f"  {n_env} environment obstacle(s)\n"
        f"  {n_tools} arm tool(s): {', '.join(sorted(rcell.tool_models.keys())) or '-'}"
    )
    print(f"RSRebuildRobotCell: {n_bar} bars, {n_joint} joints, {n_env} obstacles, {n_tools} tools.")
    rs.MessageBox(msg, 0, "RSRebuildRobotCell")


if __name__ == "__main__":
    main()
