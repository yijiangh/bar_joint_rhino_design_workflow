#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSRebuildRobotCell - Refresh the planning cells: re-read the document and rebuild them through the core.

The planning cells are built by the shared core from the document
read as a schema 2 design (``core.design_source``): every bar, joint half,
walkable ground and environment obstacle, the robots and their tools. The IK
commands and viewers keep them in the session (``sc.sticky``) and rebuild one
only when its geometry changed, so no manual rebuild is needed after an edit.
This button forces a clean start and pays the load up front:

1. AUTO-ASSIGNS WalkableGround surfaces to bars: every WalkableGround brep gets a
   stable id (``WG0``, ``WG1``, ...) and every bar with no association yet is
   linked to its nearest ground(s). Bars edited with
   RSAssignAndShowWalkableGround are left untouched.
2. AUTO-POPULATES a heuristic seed base pose on every bar that has none yet
   (non-destructive: hand-picked and IK-solved bases are kept).
3. Drops every cached cell, mirror and preview mesh, re-reads the document, and
   builds each robot's planning mirror (its own PyBullet world) and reads the
   robots' meshes, so the next IK command or viewer starts warm.

RSPBStart (left-click) runs the same; its right-click, RSPBStartGUI, also opens
the assembly robot's planning world in PyBullet's window. RSPBStop closes them.
"""

from __future__ import annotations

import importlib
import os
import sys
import time

import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

from core import config as _config_module
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import design_builder as _design_builder_module
from core import design_display as _design_display_module
from core import design_planning as _design_planning_module
from core import design_source as _design_source_module
from core import ik_viz as _ik_viz_module
from core import rhino_walkable_ground as _walkable_module
from core.rhino_bar_registry import repair_on_entry


def close_all(store) -> None:
    """Close every planning mirror, forget the geometry cache and the preview meshes."""
    design_planning = importlib.reload(_design_planning_module)
    design_source = importlib.reload(_design_source_module)
    ik_viz = importlib.reload(_ik_viz_module)
    design_planning.close_mirrors(store)
    store.pop(design_source.GEOMETRY_CACHE_KEY, None)
    ik_viz.discard_cache()


def refresh(command: str = "RSRebuildRobotCell", gui: bool = False) -> None:
    """Run the ground and base passes, rebuild every cell from the document, and report.

    Args:
        command: The command name for messages.
        gui: Also open the assembly robot's planning world in PyBullet's window.
    """
    config = importlib.reload(_config_module)
    walkable = importlib.reload(_walkable_module)
    importlib.reload(_design_builder_module)
    design_source = importlib.reload(_design_source_module)
    importlib.reload(_design_display_module)
    design_planning = importlib.reload(_design_planning_module)

    repair_on_entry(float(config.BAR_RADIUS), command)
    ground_lines, ground_console = walkable.assign_and_seed_summary()
    close_all(sc.sticky)

    started = time.monotonic()
    try:
        result = design_source.RhinoDesignSource(command, design_source.geometry_cache(sc.sticky)).build()
    except Exception as exc:  # noqa: BLE001 -- surface any build error to the user
        session_log.caught(exc)
        rs.MessageBox(f"{command}: the document cannot become a design:\n{type(exc).__name__}: {exc}", 0, command)
        return
    design = result.design
    for problem in result.problems:
        print(f"{command}: {problem}")

    built, failed = [], []
    movement = next((m for _action, m in design.movements()), None)
    if movement is not None:
        for robot_id in sorted(design.robots):
            try:
                with session_log.step("build planning cell", robot_id):
                    scene = design.scene_at(movement)
                    if gui and robot_id == design_source.robot_id(config.ASSEMBLY_ROBOT_NAME):
                        design_planning.mirror_for(sc.sticky, robot_id).set_gui(True)
                    design_planning.sync_seeded(sc.sticky, scene, robot_id, report_parked=False)
                    # The meshes the previews draw (read once per file, kept by the core).
                    with session_log.step("read robot meshes"):
                        scene.robots[robot_id].model.link_geometry
                    built.append(robot_id)
            except Exception as exc:  # noqa: BLE001 -- one robot must not stop the others
                session_log.caught(exc)
                failed.append(f"{robot_id}: {type(exc).__name__}: {exc}")

    count = {prefix: sum(1 for b in design.bodies if b.startswith(prefix + "/"))
             for prefix in ("bars", "joints", "ground", "obstacles")}
    lines = [f"{count['bars']} bar(s), {count['joints']} joint half/halves, {count['ground']} walkable ground(s), "
             f"{count['obstacles']} environment obstacle(s)",
             f"Tools: {', '.join(sorted(design.tools)) or '-'}",
             f"Planning cells: {', '.join(built) or 'none'} ({time.monotonic() - started:.1f} s)"]
    if movement is None:
        lines.append("No bar has a left and a right tool yet: the cells are built by the first IK command.")
    lines += [f"FAILED {line}" for line in failed] + ground_lines
    if result.problems:
        lines.append(f"{len(result.problems)} note(s) on what the design leaves out: see the command history.")
    print(f"{command}: {'; '.join(lines[:3])}; {ground_console}.")
    rs.MessageBox("Refreshed the planning cells:\n  " + "\n  ".join(lines), 0, command)


def main() -> None:
    refresh("RSRebuildRobotCell")


if __name__ == "__main__":
    session_log.run(__file__, main)
