#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSExportDesign - write the document as a schema 2 design folder, then run the file and plan checks.

The design comes from `core.design_source.RhinoDesignSource` (bars, halves, ground, obstacles, robots, tools, mates,
and the actions built from each bar's keyframes) and is written by the core's `write`. Bars without IK are exported
with `null` bases and joints. `source/` gets the .3dm as last saved and the two registries. Needs no PyBullet,
compas_fab or tamp. It is the only export command.
"""

from __future__ import annotations

import importlib
import os
import shutil
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

from core import config as _config_module  # noqa: E402
from core import core_lib  # noqa: E402,F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import design_builder as _design_builder_module  # noqa: E402
from core import design_source as _design_source_module  # noqa: E402
from core import rhino_walkable_ground as _walkable_module  # noqa: E402
from core.ik_status import stale_from, stale_ik_lines  # noqa: E402
from core.rhino_bar_registry import repair_on_entry  # noqa: E402
from bar_assembly_core.design import DesignError, write  # noqa: E402
from bar_assembly_core.design.plan_check import check_plan  # noqa: E402
from rs_check_design import DESIGN_FOLDER_STICKY_KEY, report_lines  # noqa: E402


def _pick_folder(command: str):
    """Ask for the parent folder and the design folder's name; None if cancelled or refused."""
    last = sc.sticky.get(DESIGN_FOLDER_STICKY_KEY)
    parent = rs.BrowseForFolder(folder=os.path.dirname(last) if last else None,
                                message="Select the folder to write the design folder into", title=command)
    if not parent:
        return None
    doc_name = os.path.splitext(sc.doc.Name or "design")[0]
    default = os.path.basename(last) if last else f"{doc_name}_design"
    name = rs.StringBox("Design folder name", default, command)
    if not name:
        return None
    folder = os.path.join(parent, name)
    if os.path.isdir(folder) and os.listdir(folder):
        if not os.path.isfile(os.path.join(folder, "design.json")):
            rs.MessageBox(f"{folder} is not empty and holds no design.json; pick another name.", 0, command)
            return None
        if rs.MessageBox(f"Replace the design in\n{folder}?\n(solutions/ and runs/ stay)", 4 | 32, command) != 6:
            return None
    sc.sticky[DESIGN_FOLDER_STICKY_KEY] = folder
    return folder


def _copy_sources(folder: str, config) -> None:
    """Copy the .3dm as last saved and the joint and tool registries into `source/` (never read back)."""
    source = os.path.join(folder, "source")
    os.makedirs(source, exist_ok=True)
    paths = [os.path.join(config.SCRIPTS_DIR, "core", "joint_pairs.json"),
             os.path.join(config.SCRIPTS_DIR, "core", "robotic_tools.json")]
    if sc.doc.Path and os.path.isfile(sc.doc.Path):
        paths.append(sc.doc.Path)
    for path in paths:
        shutil.copy2(path, source)


def export(command: str = "RSExportDesign") -> None:
    """Build, write and check the design; print everything and show a summary."""
    config = importlib.reload(_config_module)
    importlib.reload(_design_builder_module)
    design_source = importlib.reload(_design_source_module)
    repair_on_entry(float(config.BAR_RADIUS), command)
    importlib.reload(_walkable_module).ensure_ground_and_base(command)

    folder = _pick_folder(command)
    if not folder:
        print(f"{command}: cancelled.")
        return
    started = time.perf_counter()
    try:
        result = design_source.RhinoDesignSource(command, design_source.geometry_cache(sc.sticky),
                                                 fresh_versions=True).build()
    except DesignError as exc:
        print(f"{command}: the document cannot be exported:\n  " + "\n  ".join(exc.problems))
        rs.MessageBox(f"Not exported: {len(exc.problems)} problem(s) in the document.\n\nDetails in the command "
                      "history.", 0, command)
        return
    built = time.perf_counter()
    lines = [f"{command}: built in {built - started:.1f} s"]
    if result.problems:
        lines += [f"Notes ({len(result.problems)}):"] + [f"  {p}" for p in result.problems]
    try:
        with session_log.step("write design folder", folder):
            written = write(result.design, folder, overwrite=True, package_dirs=[config.HUSKY_PKG_PATH])
    except DesignError as exc:
        lines += [f"A (file checks): FAILED, nothing usable written; {len(exc.problems)} problem(s)"]
        lines += [f"  {p}" for p in exc.problems]
        print("\n".join(lines))
        rs.MessageBox(f"A (file checks) failed: {len(exc.problems)} problem(s).\n\nDetails in the command history.",
                      0, command)
        return
    with session_log.step("copy sources"):
        _copy_sources(folder, config)
    with session_log.step("check plan (B)"):
        report = check_plan(written)
    # The core's verdict on the saved IK (B15), by bar: these bars export as solved, with poses that miss.
    stale = stale_from(report.unreached)
    n = sum(len(a.movements) for a in written.actions.values())
    solved = sum(1 for a in written.actions.values() if a.type == "bar_jointing" and a.movements[-1].target.joints)
    jointing = sum(1 for a in written.actions.values() if a.type == "bar_jointing")
    lines += [f"Wrote {folder} in {time.perf_counter() - built:.1f} s: {len(written.bodies)} bodies, "
              f"{len(written.mates)} mates, {len(written.actions)} actions, {n} movements; "
              f"{solved}/{jointing} bars with IK", "A (file checks): passed"] + report_lines(report)
    lines += stale_ik_lines(stale)
    print("\n".join(lines))
    stale_note = (f"\n\nSTALE IK on {len(stale)} bar(s) (plan check B15): their saved robot poses no longer reach "
                  "the bars' tools. Re-run RSIKKeyframe on them, then export again." if stale else "")
    rs.MessageBox(f"Wrote {folder}\n\n{len(written.actions)} actions ({solved}/{jointing} bars with IK), "
                  f"{len(result.problems)} note(s)\nA passed; B {len(report.errors)} error(s), "
                  f"{len(report.warnings)} warning(s){stale_note}\n\nDetails in the command history.", 0, command)


if __name__ == "__main__":
    session_log.run(__file__, export)
