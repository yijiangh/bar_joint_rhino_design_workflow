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
    <root>/Trajectories/         (created empty for future use)

Rigid-body names in the JSON are state-independent (``bar_<bid>`` /
``joint_<jid>_<sub>``) via ``core.bar_action.dump_cell_canonical``; the
cached in-Rhino cell keeps the upstream ``active_*`` / ``env_*`` prefixes
so the IK pipeline is unaffected.

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

from core import bar_action as _bar_action_module
from core import robot_cell as _robot_cell_module


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
    bar_action = importlib.reload(_bar_action_module)

    rcell = robot_cell.get_or_load_robot_cell()

    n_rb = len(rcell.rigid_body_models)
    if n_rb == 0:
        ans = rs.MessageBox(
            "RobotCell has 0 rigid bodies registered (no env bars/joints, no "
            "arm-tool RBs). Per-bar BarActions will reference RBs that aren't "
            "on this cell.\n\n"
            "Recommended: cancel, run RSExportBarAction on a bar first "
            "(it registers env + arm-tool RBs onto the cached cell), then "
            "re-run RSExportRobotCell.\n\n"
            "Save anyway?",
            4 | 48,  # YesNo | Warning
            "RSExportRobotCell",
        )
        if ans != 6:  # 6 == Yes
            print("RSExportRobotCell: cancelled (run RSExportBarAction first).")
            return
    else:
        print(f"RSExportRobotCell: cell carries {n_rb} rigid_body_models.")

    root = _prompt_export_root()
    if not root:
        print("RSExportRobotCell: cancelled.")
        return

    os.makedirs(root, exist_ok=True)
    os.makedirs(os.path.join(root, "BarActions"), exist_ok=True)
    os.makedirs(os.path.join(root, "Trajectories"), exist_ok=True)

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
        # State-independent rigid-body names: strip active_*/env_* prefixes
        # on the dump only; cached rcell keeps upstream prefixes so the
        # in-Rhino IK pipeline (env_collision / ik_collision_setup) is
        # unaffected. See core.bar_action.dump_cell_canonical.
        bar_action.dump_cell_canonical(rcell, f, pretty=True)
    print(
        f"RSExportRobotCell: saved {out} "
        f"(rigid-body names canonicalized: bar_<bid> / joint_<jid>_<sub>)."
    )


if __name__ == "__main__":
    main()
