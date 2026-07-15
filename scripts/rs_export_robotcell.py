#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportRobotCell - Save the loaded `compas_fab.RobotCell` to JSON.

Writes the canonical layout under a user-chosen root folder:

    <root>/RobotCell.json
    <root>/BarActions/           (created empty; populated by RSExportBarAction)

Rigid-body names in the JSON are already state-independent (``bar_<bid>`` /
``joint_<jid>_<sub>`` / ``obstacle_<name>``) and tools live under
``tool_models`` (``AT3L`` / ``AT3R``) -- the cached cell is the persistent
static registry, so the dump is a plain ``compas.json_dump`` (no
canonicalization, no snapshot/restore).

``robot_cell.ensure_assembly_cell`` is called first: it reuses the cached
snapshot built by RSRebuildRobotCell (building once if absent) and warns if
the document geometry changed since the last rebuild. Requires PyBullet
(RSPBStart); run RSRebuildRobotCell after geometry edits.

The last-used root is remembered in ``sc.sticky`` and pre-selected in the
folder dialog; the user can always pick a different folder.
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import robot_cell as _robot_cell_module

from compas import json_dump


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"


def _prompt_export_root() -> str | None:
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder",
        title="RSExportRobotCell",
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first, then RSRebuildRobotCell.",
            0,
            "RSExportRobotCell",
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSExportRobotCell: aborted (stale collision cell).")
        return

    # The cell is the persistent static registry. Reuse the cached snapshot
    # (building once if absent; warns if geometry changed since the last
    # RSRebuildRobotCell). No snapshot/restore -- the cell is meant to carry the
    # full assembly.
    robot_cell.ensure_assembly_cell(rcell, planner)
    n_rb = len(rcell.rigid_body_models)
    n_tools = len(rcell.tool_models)
    if n_rb == 0:
        ans = rs.MessageBox(
            "RobotCell has 0 rigid bodies registered.\n\n"
            "Recommended: cancel, run RSRebuildRobotCell first, then re-run "
            "RSExportRobotCell.\n\nSave anyway?",
            4 | 48,  # YesNo | Warning
            "RSExportRobotCell",
        )
        if ans != 6:  # 6 == Yes
            print("RSExportRobotCell: cancelled (run RSRebuildRobotCell first).")
            return
    else:
        print(f"RSExportRobotCell: cell carries {n_rb} rigid_body_models + {n_tools} tool_models.")

    root = _prompt_export_root()
    if not root:
        print("RSExportRobotCell: cancelled.")
        return

    os.makedirs(root, exist_ok=True)
    os.makedirs(os.path.join(root, "BarActions"), exist_ok=True)

    out = os.path.join(root, "RobotCell.json")
    if os.path.exists(out):
        ans = rs.MessageBox(
            f"'{out}' exists. Overwrite?",
            4 | 32,  # YesNo | Question
            "RSExportRobotCell",
        )
        if ans != 6:  # 6 == Yes
            print("RSExportRobotCell: cancelled (kept existing file).")
            return

    with open(out, "w") as f:
        # Names are already canonical (bar_<bid> / joint_<jid>_<sub> /
        # obstacle_<name>); tools under tool_models. Plain dump.
        json_dump(rcell, f, pretty=True)
    print(
        f"RSExportRobotCell: saved {out} "
        f"({n_rb} rigid_body_models, {n_tools} tool_models)."
    )


if __name__ == "__main__":
    main()
