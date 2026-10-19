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
    <root>/MoCapMarkers.json     (designed MoCap markers; only when any are placed)
    <root>/BarActions/           (created empty; populated by RSExportBarAction)

``MoCapMarkers.json`` lists, per bar, its MoCap marker spheres in the bar
body's own frame (metres), named ``J20-21_mocap_M1``; the same markers are
labelled in Rhino on the "MoCap Marker Labels" layer, to name the Motive
markers by.  A bar whose marker layout could be seen in two poses stops the
export before anything is written.  See ``docs/mocap_marker_export.md``.

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


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import export_root as _export_root_module
from core import mocap_markers as _mocap_markers_module
from core import rhino_mocap_markers as _rhino_mocap_markers_module
from core import robot_cell as _robot_cell_module



def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    export_root = importlib.reload(_export_root_module)
    importlib.reload(_mocap_markers_module)
    rhino_mocap_markers = importlib.reload(_rhino_mocap_markers_module)

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
    bodies = robot_cell.ensure_assembly_cell(rcell, planner)
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

    # The MoCap marker file is built BEFORE anything is written: its layout
    # check may refuse the export, and then the folder must stay untouched.
    try:
        mocap_payload = rhino_mocap_markers.build_mocap_sidecar(bodies or {}, "RSExportRobotCell")
    except RuntimeError as exc:
        rs.MessageBox(f"Export stopped, nothing was written:\n\n{exc}", 0, "RSExportRobotCell")
        print(f"RSExportRobotCell: stopped, nothing written ({exc}).")
        return

    root = export_root.prompt_export_root("RSExportRobotCell")
    if not root:
        print("RSExportRobotCell: cancelled.")
        return

    os.makedirs(root, exist_ok=True)
    os.makedirs(os.path.join(root, export_root.BAR_ACTIONS_DIR), exist_ok=True)

    out = os.path.join(root, export_root.ROBOT_CELL_FILE)
    if os.path.exists(out):
        ans = rs.MessageBox(
            f"'{out}' exists. Overwrite?",
            4 | 32,  # YesNo | Question
            "RSExportRobotCell",
        )
        if ans != 6:  # 6 == Yes
            print("RSExportRobotCell: cancelled (kept existing file).")
            return

    export_root.write_robot_cell(rcell, root)
    print(
        f"RSExportRobotCell: saved {out} "
        f"({n_rb} rigid_body_models, {n_tools} tool_models)."
    )
    rhino_mocap_markers.write_mocap_sidecar(mocap_payload, bodies or {}, root, "RSExportRobotCell")


if __name__ == "__main__":
    main()
