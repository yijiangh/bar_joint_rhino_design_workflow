#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSExportProcess - write the document as a schema 2 process folder, then run the file and plan checks.

The process comes from `core.process_source.RhinoProcessSource` (bars, halves, ground, obstacles, robots, tools, mates,
and the actions built from each bar's keyframes) and is written by the core's `write`. Bars without IK are exported
with `null` bases and joints. `source/` gets the .3dm as last saved and the two registries. Needs no PyBullet,
compas_fab or tamp. It is the only export command.

First it asks which bars to export (`core.rhino_export_range`): Enter or All exports every bar; FromBar / UntilBar pick
a range. A range export holds the bars up to Until and the actions of From..Until, the earlier bars built in its first
state (`process_builder.cut_to_range`); the range goes into `producer.command`.

Then a folder dialog asks for the process folder itself: pick one, or make a new folder; the files go into it. If it
already holds any of process.json, actions/, meshes/, robots/, runs/, source/, a window offers Overwrite, Pick another
folder or Cancel export. Other files in it (desktop.ini, ...) are never looked at.

The MoCap markers travel in `process.json` on their halves. Before anything is written, `core.process_markers` checks
them: a MoCap half without markers, or a bar the monitor could not fit uniquely (fewer than 3 markers, or a layout that
fits itself in a second pose within `config.MOCAP_LAYOUT_TOL_MM`), stops the export with a message naming it. After
the write, every designed marker is labelled on the layer "MoCap Marker Labels" (`core.rhino_mocap_markers`).
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
from core import core_lib  # noqa: E402,F401  (puts scaffolding_assembly_core on sys.path; never reloaded)
from core import process_builder as _process_builder_module  # noqa: E402
from core import process_markers as _process_markers_module  # noqa: E402
from core import process_source as _process_source_module  # noqa: E402
from core import rhino_export_range as _export_range_module  # noqa: E402
from core import rhino_mocap_markers as _rhino_mocap_markers_module  # noqa: E402
from core import rhino_walkable_ground as _walkable_module  # noqa: E402
from core.ik_status import stale_from, stale_ik_lines  # noqa: E402
from core.rhino_bar_registry import get_real_bar_seq_map, repair_on_entry  # noqa: E402
from scaffolding_assembly_core.process import ProcessError, write  # noqa: E402
from scaffolding_assembly_core.process.plan_check import check_plan  # noqa: E402
from rs_check_process import PROCESS_FOLDER_STICKY_KEY, report_lines  # noqa: E402


# What an export puts in the process folder (`doc/process_format.md` §2): the core's `write` makes process.json,
# actions/, meshes/ and robots/ (emptying actions/*.json and meshes/ first), `_copy_sources` makes source/, and the
# export adds an empty runs/ (the monitor's). Anything else in the folder (desktop.ini, notes, ...) is never looked at.
PROCESS_ENTRIES = ("process.json", "actions", "meshes", "robots", "runs", "source")


def _entry_label(name: str) -> str:
    """`actions` -> `actions/`; `process.json` stays."""
    return name if "." in name else f"{name}/"


def _ask_existing(folder: str, found: list, command: str):
    """Ask what to do with a folder that already holds process entries: "overwrite", "repick", or None (give up).

    Args:
        folder: The picked folder.
        found: The `PROCESS_ENTRIES` already in it.
        command: The command name, for the window title.
    """
    import Eto.Drawing as drawing
    import Eto.Forms as forms
    import Rhino

    # Eto constructors take no keyword arguments under Rhino's CPython: construct, then assign.
    def label(text):
        control = forms.Label()
        control.Text = text
        return control

    def button(text, choice):
        control = forms.Button()
        control.Text = text

        def on_click(_sender, _args):
            chosen[0] = choice
            dialog.Close(True)

        control.Click += on_click
        return control

    chosen = [None]  # * closing the window or Esc: give up
    dialog = forms.Dialog[bool]()
    dialog.Title = command
    dialog.Padding = drawing.Padding(12)
    text = (f"{folder}\n\nalready holds: {', '.join(_entry_label(n) for n in found)}\n\n"
            "Overwrite: this export replaces process.json, actions/, meshes/, robots/ and source/\n"
            "    (runs/ and solutions/ keep what is in them).\n"
            "Pick another folder: back to the folder dialog.\n"
            "Cancel export: nothing is written.")
    overwrite, repick, give_up = (button("Overwrite", "overwrite"), button("Pick another folder", "repick"),
                                  button("Cancel export", None))
    buttons = forms.DynamicLayout()
    buttons.Spacing = drawing.Size(8, 0)
    buttons.AddRow(None, overwrite, repick, give_up)
    layout = forms.DynamicLayout()
    layout.Spacing = drawing.Size(0, 12)
    layout.AddRow(label(text))
    layout.AddRow(buttons)
    dialog.Content = layout
    dialog.DefaultButton = repick  # ? Enter never overwrites
    dialog.AbortButton = give_up
    try:
        parent = Rhino.UI.RhinoEtoApp.MainWindow
    except Exception:  # noqa: BLE001
        parent = None
    dialog.ShowModal(parent) if parent is not None else dialog.ShowModal()
    return chosen[0]


def _pick_folder(command: str):
    """Ask for the process folder itself (pick one, or make a new folder); None if cancelled.

    The files go straight into the picked folder. When it already holds any of `PROCESS_ENTRIES`, a window offers
    Overwrite, Pick another folder (the folder dialog again) or Cancel export; other files never count.

    Args:
        command: The command name, for the dialog title.
    """
    last = sc.sticky.get(PROCESS_FOLDER_STICKY_KEY)
    # ? Starting at the last process folder: Windows opens its parent with it selected, so Select Folder re-exports
    # ? there and New folder makes a sibling. Without one (or once it is gone), start at the .3dm's folder.
    start = last if last and os.path.isdir(os.path.dirname(last)) else (
        os.path.dirname(sc.doc.Path) if sc.doc.Path else None)
    # ? BrowseForFolder titles the dialog "<title>: <message>", so the message leaves out the command name.
    message = "select the process folder, or make a new folder"
    print(f"{command}: {message}")
    while True:
        folder = rs.BrowseForFolder(folder=start, message=message, title=command)
        if not folder:
            return None
        start = folder
        found = [name for name in PROCESS_ENTRIES if os.path.exists(os.path.join(folder, name))]
        if found:
            choice = _ask_existing(folder, found, command)
            if choice is None:
                return None
            if choice == "repick":
                continue
        sc.sticky[PROCESS_FOLDER_STICKY_KEY] = folder
        return folder


def _copy_sources(folder: str, config) -> None:
    """Copy the .3dm as last saved and the joint and tool registries into a fresh `source/` (never read back)."""
    source = os.path.join(folder, "source")
    # * Emptied first: an overwrite from a renamed .3dm would otherwise keep the old copy next to the new one.
    shutil.rmtree(source, ignore_errors=True)
    os.makedirs(source, exist_ok=True)
    paths = [os.path.join(config.SCRIPTS_DIR, "core", "joint_pairs.json"),
             os.path.join(config.SCRIPTS_DIR, "core", "robotic_tools.json")]
    if sc.doc.Path and os.path.isfile(sc.doc.Path):
        paths.append(sc.doc.Path)
    for path in paths:
        shutil.copy2(path, source)


def export(command: str = "RSExportProcess") -> None:
    """Build, write and check the process; print everything and show a summary."""
    config = importlib.reload(_config_module)
    importlib.reload(_process_builder_module)
    process_source = importlib.reload(_process_source_module)
    process_markers = importlib.reload(_process_markers_module)
    rhino_mocap_markers = importlib.reload(_rhino_mocap_markers_module)
    export_range_prompt = importlib.reload(_export_range_module)
    repair_on_entry(float(config.BAR_RADIUS), command)
    importlib.reload(_walkable_module).ensure_ground_and_base(command)

    seq_map = get_real_bar_seq_map()
    export_range = None
    if seq_map:
        picked = export_range_prompt.prompt_export_range(seq_map, command)
        if picked is None:
            print(f"{command}: cancelled.")
            return
        ordered = sorted(seq_map, key=lambda bar: seq_map[bar][1])
        export_range = None if picked == (ordered[0], ordered[-1]) else picked  # * every bar: no range
    folder = _pick_folder(command)
    if not folder:
        print(f"{command}: cancelled.")
        return
    started = time.perf_counter()
    try:
        result = process_source.RhinoProcessSource(command, process_source.geometry_cache(sc.sticky),
                                                 fresh_versions=True).build(export_range=export_range)
    except ProcessError as exc:
        print(f"{command}: the document cannot be exported:\n  " + "\n  ".join(exc.problems))
        rs.MessageBox(f"Not exported: {len(exc.problems)} problem(s) in the document.\n\nDetails in the command "
                      "history.", 0, command)
        return
    # ! The MoCap layout check runs before anything is written: a refused export leaves the folder untouched.
    mocap_problems = process_markers.check_layouts(result.process, float(config.MOCAP_LAYOUT_TOL_MM))
    if mocap_problems:
        print(f"{command}: export stopped, nothing was written: the MoCap markers cannot be read by the monitor:\n  "
              + "\n  ".join(mocap_problems) + f"\n  {process_markers.LAYOUT_HINT}")
        rs.MessageBox("Export stopped, nothing was written:\n\n" + "\n\n".join(mocap_problems)
                      + f"\n\n{process_markers.LAYOUT_HINT}", 0, command)
        return
    built = time.perf_counter()
    lines = [f"{command}: built in {built - started:.1f} s"]
    if result.problems:
        lines += [f"Notes ({len(result.problems)}):"] + [f"  {p}" for p in result.problems]
    try:
        with session_log.step("write process folder", folder):
            written = write(result.process, folder, overwrite=True, package_dirs=[config.HUSKY_PKG_PATH])
    except ProcessError as exc:
        lines += [f"A (file checks): FAILED, nothing usable written; {len(exc.problems)} problem(s)"]
        lines += [f"  {p}" for p in exc.problems]
        print("\n".join(lines))
        rs.MessageBox(f"A (file checks) failed: {len(exc.problems)} problem(s).\n\nDetails in the command history.",
                      0, command)
        return
    with session_log.step("copy sources"):
        _copy_sources(folder, config)
    # ? Empty until the monitor records a run there; made here so every process folder has the full layout.
    os.makedirs(os.path.join(folder, "runs"), exist_ok=True)
    with session_log.step("label MoCap markers"):
        n_markers = rhino_mocap_markers.label_markers_in_rhino(written)
    with session_log.step("check plan (B)"):
        report = check_plan(written)
    # The core's verdict on the saved IK (B15), by bar: these bars export as solved, with poses that miss.
    stale = stale_from(report.unreached)
    n = sum(len(a.movements) for a in written.actions.values())
    # ? By the insert, not the last movement: a ground bar's jointing ends with the operator's fix_foundation step.
    solved = sum(1 for a in written.actions.values() if a.type == "bar_jointing"
                 and any(m.id.endswith("_J_insert") and m.target.joints for m in a.movements))
    jointing = sum(1 for a in written.actions.values() if a.type == "bar_jointing")
    lines += [f"Wrote {folder} in {time.perf_counter() - built:.1f} s: {len(written.bodies)} bodies, "
              f"{len(written.mates)} mates, {len(written.actions)} actions, {n} movements; "
              f"{solved}/{jointing} bars with IK", "A (file checks): passed"] + report_lines(report)
    if n_markers:
        lines.append(f"MoCap: {n_markers} markers labelled on layer '{config.LAYER_MOCAP_MARKER_LABELS}'")
    lines += stale_ik_lines(stale)
    print("\n".join(lines))
    stale_note = (f"\n\nSTALE IK on {len(stale)} bar(s) (plan check B15): their saved robot poses no longer reach "
                  "the bars' tools. Re-run RSIKKeyframe on them, then export again." if stale else "")
    rs.MessageBox(f"Wrote {folder}\n\n{len(written.actions)} actions ({solved}/{jointing} bars with IK), "
                  f"{len(result.problems)} note(s)\nA passed; B {len(report.errors)} error(s), "
                  f"{len(report.warnings)} warning(s){stale_note}\n\nDetails in the command history.", 0, command)


if __name__ == "__main__":
    session_log.run(__file__, export)
