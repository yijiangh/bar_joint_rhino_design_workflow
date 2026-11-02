#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSIKMaintenance - check the saved IK keyframes, or clear them.

One button, three jobs, asked first (Enter = CheckIK):

* **CheckIK** -- finds every bar whose saved IK no longer fits the model and
  must be recalculated. A saved keyframe is only joint angles and a base, so it
  goes stale silently when a joint or a bar moves, the walkable ground changes
  height, or another bar's IK changes (a support robot now standing where this
  robot's arm goes). The check reads the document as a process (the same read
  RSExportProcess does, so a moved joint is seen), then asks the solver's own
  questions of every saved pose -- assembly (approach / assembled / retreat /
  home) and support (held / release): do the arms still reach the tools (the
  core's plan check B15), is the pose collision-free on the robots' planning
  worlds, does the base still stand on the ground, is the support grasp still
  on its bar. The rule is ``core.ik_check``; the tolerances are
  ``config.IK_CHECK_*``.

  Every flagged bar turns amber (``config.IK_STALE_COLOR``) and is selected,
  the command line says why, and you are asked whether to clean their IK: all
  of them, or the ones you pick (the same Keyframe / BasePosition / Support
  toggles as below). The planning worlds are built on first use (PyBullet,
  nothing to start by hand).

* **ClearOneBar** / **ClearAllBars** -- the old RSClearIKKeyframe /
  RSClearIKKeyframeAll, unchanged (pick a bar or take them all, then choose what
  to erase). Those two can still be typed. No planning world needed.

The amber stays until the next CheckIK (which puts back what it painted over),
RSUpdatePreview or RSClearColorPreview.
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

import rs_clear_ik_keyframe as _clear_module

from core import config
from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from core import ik_check as _ik_check_module
from core import process_source as _process_source_module
from core import rhino_bar_registry as _registry_module
from core import rhino_walkable_ground as _walkable_module
from core.rhino_bar_pick import bar_or_tube_filter
from core.rhino_bar_registry import BAR_ID_KEY, TUBE_BAR_ID_KEY, repair_on_entry
from core.rhino_helpers import ask_option, paint_remembered, restore_painted, suspend_redraw
from scaffolding_assembly_core.process import ProcessError

# Command name used in every command-line message + dialog title.
CMD = "RSIKMaintenance"

#: ``sc.sticky`` key: what the last CheckIK painted over, so the next one puts it back.
_STICKY_PAINTED = "rs_ik_maintenance:painted"


def _reload():
    """Re-import the modules this command uses so ScriptEditor picks up changes."""
    global clear_cmd, ik_check, registry, process_source, walkable
    importlib.reload(config)
    clear_cmd = importlib.reload(_clear_module)
    ik_check = importlib.reload(_ik_check_module)
    registry = importlib.reload(_registry_module)
    process_source = importlib.reload(_process_source_module)
    walkable = importlib.reload(_walkable_module)


_reload()


def _ask_mode():
    choice = ask_option(
        "IK maintenance: check the saved IK, or clear it",
        ("CheckIK", "ClearOneBar", "ClearAllBars"),
        default="CheckIK",
    )
    return {"CheckIK": "check", "ClearOneBar": "one", "ClearAllBars": "all"}.get(choice)


# ---------------------------------------------------------------------------
# CheckIK
# ---------------------------------------------------------------------------


def _print_report(reports, stale, elapsed_s) -> None:
    print(f"{CMD}: checked the saved IK of {len(reports)} bar(s) in {elapsed_s:.0f} s.")
    for report in stale:
        print(f"  {report.bar_id} (step {report.seq}) -- needs recalculation:")
        for line in report.assembly:
            print(f"    assembly: {line}")
        for line in report.support:
            print(f"    {line}")
        for line in report.notes:
            print(f"    note: {line}")
    for report in reports:
        if not report.stale and report.notes:
            for line in report.notes:
                print(f"  {report.bar_id}: note: {line}")
    if not stale:
        print(f"{CMD}: every saved IK still fits the model.")
        return
    print(
        f"{CMD}: {len(stale)} of {len(reports)} bar(s) need recalculation (amber, selected): "
        f"{', '.join(r.bar_id for r in stale)}. Re-solve them in assembly order -- "
        "re-solving one bar changes the scene for the bars after it."
    )


def _chosen_bar_ids(bar_map) -> list:
    """The bars the user picks (the flagged ones are still amber to pick from).

    Picked from an empty selection on purpose: with the flagged bars left
    selected, Rhino's GetObjects would accept them at once without letting the
    user change the set.
    """
    rs.UnselectAllObjects()
    picked = rs.GetObjects(
        "Pick the bars to clean (amber = needs recalculation), Enter when done",
        preselect=False, select=True, custom_filter=bar_or_tube_filter,
    )
    out = []
    for oid in picked or []:
        bar_id = rs.GetUserText(oid, BAR_ID_KEY) or rs.GetUserText(oid, TUBE_BAR_ID_KEY)
        if bar_id in bar_map and bar_id not in out:
            out.append(bar_id)
    return out


def _offer_clean(stale, bar_map) -> None:
    choice = ask_option(
        f"{len(stale)} bar(s) need recalculation. Clean their IK?",
        ("Keep", "CleanAll", "CleanSelected"),
        default="Keep",
    )
    if choice in (None, "Keep"):
        print(f"{CMD}: kept every saved IK.")
        return
    if choice == "CleanAll":
        bar_ids = [report.bar_id for report in stale]
    else:
        bar_ids = _chosen_bar_ids(bar_map)
        if not bar_ids:
            print(f"{CMD}: no bar selected; nothing cleaned.")
            return
    clear_cmd.clear_ik(all_bars=False, bar_ids=bar_ids)


def _check_ik() -> None:
    repair_on_entry(float(config.BAR_RADIUS), CMD)
    # Same entry pass as the IK and export commands: a bar without a ground gets one, so the process can be read.
    walkable.ensure_ground_and_base(CMD)

    # Always from today's document: the process is read fresh, so a moved joint
    # (one of the things to find) moves the targets the saved poses are checked against.
    started = time.perf_counter()
    print(f"{CMD}: reading the document as a process ...")
    try:
        result = process_source.RhinoProcessSource(CMD, process_source.geometry_cache(sc.sticky)).build()
    except ProcessError as exc:
        # ! No check without a process: say what is wrong with the document instead of guessing.
        problems = list(getattr(exc, "problems", None) or [str(exc)])
        print(f"{CMD}: the saved IK cannot be checked, the document cannot become a process:\n  "
              + "\n  ".join(problems))
        rs.MessageBox(f"CheckIK: {len(problems)} problem(s) stop the document from becoming a process.\n\n"
                      "Details in the command history.", 0, CMD)
        return
    for line in result.problems:
        print(f"{CMD}: note: {line}")

    bar_map = registry.get_bar_seq_map()
    reports = ik_check.check_all(result.process, sc.sticky, registry.get_real_bar_seq_map(bar_map), progress=print)
    stale = [report for report in reports if report.stale]

    with suspend_redraw():
        restore_painted(_STICKY_PAINTED)
        rs.UnselectAllObjects()
        color_by_oid = {}
        for report in stale:
            for oid in registry._bar_curve_and_tube(bar_map[report.bar_id][0]):
                color_by_oid[oid] = config.IK_STALE_COLOR
        paint_remembered(color_by_oid, _STICKY_PAINTED)
        if color_by_oid:
            rs.SelectObjects(list(color_by_oid))

    _print_report(reports, stale, time.perf_counter() - started)
    if stale:
        _offer_clean(stale, bar_map)


def main() -> None:
    _reload()
    mode = _ask_mode()
    if mode is None:
        return
    if mode == "check":
        _check_ik()
        return
    clear_cmd.clear_ik(all_bars=(mode == "all"))


if __name__ == "__main__":
    main()
