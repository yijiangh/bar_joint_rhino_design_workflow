#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSShowPlanningWorld - Open the assembly robot's planning world in PyBullet's window (debugging).

Reads the document as a process, syncs the scene at the first movement into the assembly robot's planning mirror, and
shows that PyBullet world in its own window: what the IK commands check collisions against. Nothing in the document
changes. RSResetPlanning closes it.

- ! Closing PyBullet's window by hand ends that world; the next IK command builds a new one.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

from core import config as _config_module
from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path; never reloaded)
from core import process_builder as _process_builder_module
from core import process_planning as _process_planning_module
from core import process_source as _process_source_module
from core.rhino_bar_registry import repair_on_entry

COMMAND = "RSShowPlanningWorld"


def main() -> None:
    config = importlib.reload(_config_module)
    importlib.reload(_process_builder_module)
    process_source = importlib.reload(_process_source_module)
    process_planning = importlib.reload(_process_planning_module)
    repair_on_entry(float(config.BAR_RADIUS), COMMAND)

    try:
        result = process_source.RhinoProcessSource(COMMAND, process_source.geometry_cache(sc.sticky)).build()
    except Exception as exc:  # noqa: BLE001 -- surface any build error to the user
        session_log.caught(exc)
        rs.MessageBox(f"The document cannot become a process:\n{type(exc).__name__}: {exc}", 0, COMMAND)
        return
    process = result.process
    for problem in result.problems:
        print(f"{COMMAND}: {problem}")
    movement = next((m for _action, m in process.movements()), None)
    if movement is None:
        rs.MessageBox("Nothing to show yet: no bar has a left and a right tool, so the process has no movement.",
                      0, COMMAND)
        return
    robot_id = process_source.robot_id(config.ASSEMBLY_ROBOT_NAME)
    try:
        with session_log.step("open planning world", robot_id):
            process_planning.mirror_for(sc.sticky, robot_id).set_gui(True)
            process_planning.sync_seeded(sc.sticky, process.scene_at(movement), robot_id, report_parked=False)
    except Exception as exc:  # noqa: BLE001 -- PyBullet allows one window per process, and may have none
        session_log.caught(exc)
        rs.MessageBox(f"Could not open PyBullet's window:\n{type(exc).__name__}: {exc}", 0, COMMAND)
        return
    print(f"{COMMAND}: {robot_id}'s planning world at the start of {movement.id} is open in PyBullet's window.")


if __name__ == "__main__":
    session_log.run(__file__, main)
