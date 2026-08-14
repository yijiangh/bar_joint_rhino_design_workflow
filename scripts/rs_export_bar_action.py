#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportBarAction - Save one picked bar's action files to JSON.

Pick a bar; the script reads its IK keyframe data (`KEY_ASSEMBLY_*` user-text
written by ``rs_ik_keyframe.py``), builds both assembly halves via
``core.bar_action.build_bar_assembly_actions`` (which reuses the cached static
cell built by RSRebuildRobotCell) and writes ``<root>/BarActions/<bar>__J.json``
+ ``<bar>__R.json``. When the bar is in the hold plan and its support keyframe
is solved, the holding + holding-release actions are exported too
(``<bar>__H.json`` / ``<bar>__HR.json``).

A bar WITHOUT saved IK keyframes is still exported (``allow_missing_ik=True``,
same as RSExportAllBarActions): placeholder base + no configs, so the headless
keyframe solver can sample its base + IK later. Unlike the batch command, this
single-bar export does NOT emit ``RobotCell*.json`` / ``WalkableGround.json``,
so it is meant to refresh one bar inside a bundle a prior batch export
produced — but it DOES refresh ``ActionSchedule.json`` when one exists (pure
metadata, cheap to rebuild).

Run RSRebuildRobotCell after any geometry edit so the export reflects it. Root
folder is shared with RSExportRobotCell via ``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
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
from core import config as _config_module
from core import hold_action_builder as _hold_action_builder_module
from core import ik_collision_setup as _ik_collision_setup_module
from core import robot_cell as _robot_cell_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, is_fake_bar, repair_on_entry

from compas import json_dump


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"


def _prompt_export_root() -> str | None:
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder",
        title="RSExportBarAction",
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    config = importlib.reload(_config_module)
    importlib.reload(_ik_collision_setup_module)
    bar_action = importlib.reload(_bar_action_module)
    hold_action_builder = importlib.reload(_hold_action_builder_module)

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.",
            0,
            "RSExportBarAction",
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    repair_on_entry(float(config.BAR_RADIUS), "RSExportBarAction")

    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSExportBarAction: aborted (stale collision cell).")
        return

    rs.UnselectAllObjects()
    bar_oid = pick_bar(
        "Pick a bar to export its BarAssemblyAction (Esc to cancel)"
    )
    if bar_oid is None:
        return
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)
    if not bar_id:
        rs.MessageBox(
            "Picked curve is not a registered bar (no 'bar_id' user-text).",
            0,
            "RSExportBarAction",
        )
        return

    # A fake bar is staging the robot never assembles, so it has no action plan
    # to export.  Refuse rather than emit a meaningless one -- and say how to
    # undo the mark, since a bar marked by mistake looks identical otherwise.
    if is_fake_bar(bar_oid):
        rs.MessageBox(
            f"Bar '{bar_id}' is marked as a fake (non-fabricated) staging bar, so "
            "the robot never assembles it and it has no action plan.\n\n"
            "Un-mark it in RSBarEdit > FakeBar > Delete if that is wrong.",
            0,
            "RSExportBarAction",
        )
        return

    # The robot cell is now a persistent static registry (the full canonical
    # assembly + env obstacles + arm ToolModels), built by RSRebuildRobotCell
    # and reused by every command. Building the BarAction just reads that cached
    # cell -- no snapshot/restore needed. If you edited geometry since the last
    # RSRebuildRobotCell, rebuild first so the export reflects it.
    try:
        # allow_missing_ik=True (matches RSExportAllBarActions): a bar without a
        # saved IK keyframe is still exported with a placeholder base + no configs
        # so the headless solver can fill in its base + IK later, instead of hard-
        # erroring here. The target EE frames come from the placed tool blocks
        # (pure geometry), so they are complete either way.
        jointing_action, release_action = bar_action.build_bar_assembly_actions(
            rcell, planner, bar_id, bar_oid, allow_missing_ik=True
        )
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSExportBarAction")
        return

    # Holding + holding-release for a bar in the hold plan (skipped with a
    # clear note when its support keyframe is not solved yet).
    from core.rhino_bar_registry import collect_hold_inputs, get_bar_seq_map
    from core.hold_schedule import derive_hold_plan
    hold_actions = []
    hold_note = None
    seq_map = get_bar_seq_map()
    try:
        bar_seq, supported = collect_hold_inputs(seq_map)
        hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    except RuntimeError as exc:
        hold_plan = {}
        hold_note = f"hold plan derivation failed: {exc}"
    if bar_id in hold_plan:
        try:
            hold_actions = [
                ("__H", hold_action_builder.build_bar_holding_action(
                    bar_id, bar_oid, hold_plan, bar_map=seq_map)),
                ("__HR", hold_action_builder.build_bar_holding_release_action(
                    bar_id, bar_oid, hold_plan, bar_map=seq_map)),
            ]
        except RuntimeError as exc:
            hold_note = str(exc)

    to_write = [("__J", jointing_action), ("__R", release_action)] + hold_actions
    n_seq = len(jointing_action.assembly_seq)
    try:
        idx = jointing_action.assembly_seq.index(bar_id)
    except ValueError:
        idx = -1
    print(
        f"RSExportBarAction: built {len(to_write)} action(s) for bar "
        f"'{bar_id}' (assembly index {idx}/{n_seq})."
    )
    for _suffix, action in to_write:
        print(f"  {action.action_id} ({type(action).__name__}):")
        for mv in action.movements:
            n_rbs = len(mv.start_state.rigid_body_states) if mv.start_state else 0
            cfg_status = "None" if (mv.start_state is None or mv.start_state.robot_configuration is None) else "set"
            print(
                f"  - {mv.movement_id}: {type(mv).__name__}, "
                f"start_state.config={cfg_status}, rb_states={n_rbs}, "
                f"target_ee_frames={'yes' if mv.target_ee_frames else 'no'}, "
                f"target_configuration={'yes' if mv.target_configuration is not None else 'no'}"
            )
    if hold_note:
        print(f"RSExportBarAction: holding actions NOT exported — {hold_note}")

    root = _prompt_export_root()
    if not root:
        print("RSExportBarAction: cancelled.")
        return

    out_dir = os.path.join(root, "BarActions")
    os.makedirs(out_dir, exist_ok=True)

    existing = [
        os.path.join(out_dir, f"{bar_id}{suffix}.json")
        for suffix, _a in to_write
        if os.path.exists(os.path.join(out_dir, f"{bar_id}{suffix}.json"))
    ]
    if existing:
        ans = rs.MessageBox(
            f"{len(existing)} file(s) for bar '{bar_id}' exist. Overwrite?",
            4 | 32,  # YesNo | Question
            "RSExportBarAction",
        )
        if ans != 6:  # 6 == Yes
            print("RSExportBarAction: cancelled (kept existing files).")
            return

    for suffix, action in to_write:
        out = os.path.join(out_dir, f"{bar_id}{suffix}.json")
        with open(out, "w") as f:
            json_dump(action, f, pretty=True)
        print(f"RSExportBarAction: saved {out}.")

    # Refresh the schedule manifest when a prior batch export created one —
    # pure metadata, so a single-bar refresh keeps it consistent for free.
    schedule_path = os.path.join(root, "ActionSchedule.json")
    if os.path.isfile(schedule_path):
        try:
            import json as _json
            with open(schedule_path, "w") as f:
                _json.dump(hold_action_builder.build_action_schedule_payload(seq_map), f, indent=2)
            print(f"RSExportBarAction: refreshed {schedule_path}.")
        except RuntimeError as exc:
            print(f"RSExportBarAction: ActionSchedule refresh failed ({exc}).")


if __name__ == "__main__":
    main()
