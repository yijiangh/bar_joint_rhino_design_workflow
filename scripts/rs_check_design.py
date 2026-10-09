#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSCheckDesign - read a schema 2 design folder and print its file checks (A) and plan checks (B).

The core (`external/bar_assembly_core`) does both: `read` runs A1-A13 and refuses a broken folder, `check_plan` runs
B1-B14. Needs no PyBullet, compas_fab or tamp.
"""

from __future__ import annotations

import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

from core import core_lib  # noqa: E402,F401  (puts bar_assembly_core on sys.path; never reloaded)
from bar_assembly_core.design import DesignError, read  # noqa: E402
from bar_assembly_core.design.plan_check import check_plan  # noqa: E402

#: Last design folder, shared with RSExportDesign.
DESIGN_FOLDER_STICKY_KEY = "bar_joint:design_folder"
#: Messages printed per check before "... and N more".
SHOWN_PER_CHECK = 6


def report_lines(report) -> list:
    """The plan check result as printable lines: a count, then each check's messages, errors first."""
    lines = [f"B (plan checks): {len(report.errors)} error(s), {len(report.warnings)} warning(s)"]
    for kind, messages in (("error", report.errors), ("warning", report.warnings)):
        by_check = {}
        for message in messages:
            by_check.setdefault(message.split(":")[0], []).append(message)
        for check, found in by_check.items():
            lines.append(f"  {check} {kind} x{len(found)}:")
            lines += [f"    {m}" for m in found[:SHOWN_PER_CHECK]]
            if len(found) > SHOWN_PER_CHECK:
                lines.append(f"    ... and {len(found) - SHOWN_PER_CHECK} more")
    return lines


def check_folder(folder: str) -> tuple:
    """Read a design folder and plan-check it.

    Returns:
        tuple: (summary line, printable lines).
    """
    try:
        design = read(folder)
    except DesignError as exc:
        lines = [f"A (file checks): FAILED, {len(exc.problems)} problem(s)"] + [f"  {p}" for p in exc.problems]
        return f"A failed: {len(exc.problems)} problem(s)", lines
    report = check_plan(design)
    n = sum(len(a.movements) for a in design.actions.values())
    lines = [f"design {folder}: {len(design.bodies)} bodies, {len(design.mates)} mates, {len(design.actions)} actions,"
             f" {n} movements", "A (file checks): passed"] + report_lines(report)
    return f"A passed; B {len(report.errors)} error(s), {len(report.warnings)} warning(s)", lines


def main() -> None:
    last = sc.sticky.get(DESIGN_FOLDER_STICKY_KEY)
    folder = rs.BrowseForFolder(folder=last if last and os.path.isdir(last) else None,
                                message="Select a design folder (it holds design.json)", title="RSCheckDesign")
    if not folder:
        print("RSCheckDesign: cancelled.")
        return
    sc.sticky[DESIGN_FOLDER_STICKY_KEY] = folder
    summary, lines = check_folder(folder)
    print("RSCheckDesign:\n" + "\n".join(lines))
    rs.MessageBox(f"{summary}\n\nDetails in the command history.", 0, "RSCheckDesign")


if __name__ == "__main__":
    main()
