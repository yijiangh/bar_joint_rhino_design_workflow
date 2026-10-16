"""The export folder the RSExport* commands write into, and the files in it
that more than one command writes.

::

    <export root>/
        RobotCell.json        RSExportRobotCell, RSExportAllBarActions
        ActionSchedule.json   RSExportAllBarActions, RSExportBarAction
        BarActions/           one file per bar action
        ...

The folder is remembered for the Rhino session in ``sc.sticky`` under
:data:`EXPORT_ROOT_STICKY_KEY`, so every command offers the last one.  It is
the same folder the teleop monitor reads as its design problem.

Rhino-runtime only.
"""

from __future__ import annotations

import os

import rhinoscriptsyntax as rs
import scriptcontext as sc

EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"
ROBOT_CELL_FILE = "RobotCell.json"
BAR_ACTIONS_DIR = "BarActions"


def last_export_root():
    """The remembered export root, or ``None`` when unset or gone."""
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    return last if last and os.path.isdir(last) else None


def prompt_export_root(title: str, message: str = "Select export root folder"):
    """Browse for the export root (starting at the last one); remember it.

    Returns the chosen folder, or ``None`` on cancel.
    """
    chosen = rs.BrowseForFolder(folder=last_export_root(), message=message, title=title)
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def resolve_actions_root(title: str):
    """The remembered root if it holds ``BarActions/``, else browse for one."""
    last = last_export_root()
    if last and os.path.isdir(os.path.join(last, BAR_ACTIONS_DIR)):
        return last
    return prompt_export_root(
        title, f"Select export root (the folder that contains {BAR_ACTIONS_DIR}/)"
    )


def write_robot_cell(rcell, root: str) -> str:
    """Write *rcell* to ``<root>/RobotCell.json`` (compas JSON); return the path."""
    from compas import json_dump  # noqa: PLC0415

    path = os.path.join(root, ROBOT_CELL_FILE)
    with open(path, "w") as f:
        # Names are already canonical (bar_<bid> / joint_<jid>_<sub> /
        # obstacle_<name>); tools under tool_models.  Plain dump.
        json_dump(rcell, f, pretty=True)
    return path
