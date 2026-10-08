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

Range bundles: when the folder's ``ActionSchedule.json`` records the cell's
body names (``cell_rigid_bodies``, written by RSExportAllBarActions, also for
a range From .. Until), the fresh actions are cut the same way the batch
export cut the bundle: the bodies its ``RobotCell.json`` lacks are dropped and
``assembly_seq`` stops at the bundle's last bar. A bar outside the bundle's
range is refused, and so is a bundle whose bar sequence no longer matches the
document (re-export it with the right-click instead). A hold releasing after
the range's last bar keeps its holding action but gets no release file.

Run RSRebuildRobotCell after any geometry edit so the export reflects it. Root
folder is shared with RSExportRobotCell via ``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
"""

from __future__ import annotations

import importlib
import json
import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# ! Reload the "provider" modules FIRST: the ones other core modules import
# NAMES from (`from core.env_collision import STATIC_KINDS`, ...). A module
# imported below for the first time in this Rhino session would otherwise bind
# those names against a stale copy still in sys.modules, and fail with an
# ImportError on any newly added name before main() could reload anything.
from core import env_collision as _env_collision_module
from core import export_subset as _export_subset_module
from core import hold_schedule as _hold_schedule_module
from core import rhino_bar_registry as _rhino_bar_registry_module

importlib.reload(_env_collision_module)
# After env_collision: it imports the body-name prefixes from there.
importlib.reload(_export_subset_module)
importlib.reload(_hold_schedule_module)
importlib.reload(_rhino_bar_registry_module)

from core import bar_action as _bar_action_module
from core import config as _config_module
from core import hold_action_builder as _hold_action_builder_module
from core import ik_collision_setup as _ik_collision_setup_module
from core import robot_cell as _robot_cell_module
from core import robot_obstacles as _robot_obstacles_module
from core.export_subset import (
    assert_states_name_bodies,
    bodies_outside_cell,
    cut_assembly_seq,
    range_bar_ids,
    trim_action_states,
)
from core.hold_schedule import derive_hold_plan
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    collect_hold_inputs,
    get_real_bar_seq_map,
    is_fake_bar,
    repair_on_entry,
)

from compas import json_dump


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"
CALLER = "RSExportBarAction"


def _prompt_export_root() -> str | None:
    """Ask for the export folder (remembered for the next export).

    Returns:
        str | None: the chosen folder, or None when cancelled.
    """
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder",
        title=CALLER,
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def _read_bundle_schedule(root: str) -> dict | None:
    """The folder's ``ActionSchedule.json``, or None when there is none.

    Args:
        root (str): the export folder.

    Returns:
        dict | None: the schedule payload.
    """
    schedule_path = os.path.join(root, "ActionSchedule.json")
    if not os.path.isfile(schedule_path):
        return None
    with open(schedule_path) as f:
        return json.load(f)


def _fit_to_bundle(to_write: list, bar_id: str, seq_map: dict, hold_plan: dict, schedule: dict):
    """Cut a freshly built bar's actions to the bundle they are written into.

    Args:
        to_write (list): ``[(suffix, action), ...]`` built for ``bar_id``.
        bar_id (str): the picked bar.
        seq_map (dict): ``{bar_id: (oid, seq)}`` of the real bars.
        hold_plan (dict): the full hold plan (``derive_hold_plan``).
        schedule (dict): the bundle's ``ActionSchedule.json``; must carry
            ``cell_rigid_bodies``.

    Returns:
        tuple: ``(to_write, export_range, notes)`` -- the actions to write
        (a hold release after the range is left out), the bundle's
        ``(from_bar_id, until_bar_id)`` and printable notes.

    Raises:
        RuntimeError: the bar is outside the bundle's range, the document's
            sequence no longer matches the bundle, or a cut state still does
            not name exactly the bundle's cell bodies.
    """
    ordered = schedule["assembly_seq"]
    recorded_range = schedule.get("export_range") or {}
    from_bar_id = recorded_range.get("from_bar_id") or ordered[0]
    until_bar_id = recorded_range.get("until_bar_id") or ordered[-1]
    # The bundle's scene must still be the document's: same bars, same order.
    if until_bar_id not in seq_map or cut_assembly_seq(seq_map, until_bar_id) != ordered:
        raise RuntimeError(
            "The bar sequence changed since this bundle was exported (bars added, "
            "removed, reordered or marked fake up to its last bar "
            f"{until_bar_id}). Re-export the bundle with the right-click "
            "(RSExportAllBarActions) instead of refreshing one bar."
        )
    if bar_id not in range_bar_ids(seq_map, from_bar_id, until_bar_id):
        raise RuntimeError(
            f"Bar {bar_id!r} is outside this bundle's range {from_bar_id} .. "
            f"{until_bar_id}. Re-export the bundle with the right-click "
            "(RSExportAllBarActions) to change the range."
        )
    notes = []
    # The bar's own hold always starts inside the range (at the bar's step);
    # its release is only part of the bundle when it comes by the range's end.
    entry = hold_plan.get(bar_id)
    if entry is not None and entry["release_after_seq"] > int(seq_map[until_bar_id][1]):
        to_write = [(suffix, action) for suffix, action in to_write if suffix != "__HR"]
        notes.append(
            f"hold release NOT exported: {bar_id} is released after "
            f"{entry['release_after_bar_id']}, past the bundle's last bar {until_bar_id}."
        )
    # The same cut the batch export made: drop what the bundle's cell lacks.
    cell_bodies = schedule["cell_rigid_bodies"]
    actions = [action for _suffix, action in to_write]
    dropped = bodies_outside_cell(actions, cell_bodies)
    for action in actions:
        trim_action_states(action, dropped, ordered)
    assembly_actions = [action for suffix, action in to_write if suffix in ("__J", "__R")]
    assert_states_name_bodies(assembly_actions, cell_bodies, "the bundle's RobotCell.json")
    if dropped:
        notes.append(f"cut to the bundle: {len(dropped)} bod(ies) after {until_bar_id} left out.")
    return to_write, (from_bar_id, until_bar_id), notes


def main() -> None:
    robot_cell = importlib.reload(_robot_cell_module)
    config = importlib.reload(_config_module)
    # Providers before the modules that import names from them.
    importlib.reload(_env_collision_module)
    importlib.reload(_export_subset_module)
    importlib.reload(_hold_schedule_module)
    importlib.reload(_rhino_bar_registry_module)
    importlib.reload(_robot_obstacles_module)
    importlib.reload(_ik_collision_setup_module)
    bar_action = importlib.reload(_bar_action_module)
    hold_action_builder = importlib.reload(_hold_action_builder_module)

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, CALLER)
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    repair_on_entry(float(config.BAR_RADIUS), CALLER)

    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print(f"{CALLER}: aborted (stale collision cell).")
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
            "Picked curve is not a registered bar (no 'bar_id' user-text).", 0, CALLER,
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
            CALLER,
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
        rs.MessageBox(str(exc), 0, CALLER)
        return

    # Holding + holding-release for a bar in the hold plan (skipped with a
    # clear note when its support keyframe is not solved yet).
    hold_actions = []
    hold_note = None
    # Real bars only (no fake bars): the same map the batch export uses.
    seq_map = get_real_bar_seq_map()
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

    root = _prompt_export_root()
    if not root:
        print(f"{CALLER}: cancelled.")
        return

    # * ---- Fit the fresh actions to the bundle they go into (range bundles).
    schedule = _read_bundle_schedule(root)
    export_range = None
    if schedule is not None and schedule.get("cell_rigid_bodies") is not None:
        try:
            to_write, export_range, notes = _fit_to_bundle(
                to_write, bar_id, seq_map, hold_plan, schedule,
            )
        except RuntimeError as exc:
            rs.MessageBox(f"Nothing was written:\n\n{exc}", 0, CALLER)
            print(f"{CALLER}: nothing written ({exc}).")
            return
        for note in notes:
            print(f"{CALLER}: {note}")

    n_seq = len(jointing_action.assembly_seq)
    try:
        idx = jointing_action.assembly_seq.index(bar_id)
    except ValueError:
        idx = -1
    print(
        f"{CALLER}: built {len(to_write)} action(s) for bar "
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
        print(f"{CALLER}: holding actions NOT exported — {hold_note}")

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
            CALLER,
        )
        if ans != 6:  # 6 == Yes
            print(f"{CALLER}: cancelled (kept existing files).")
            return

    for suffix, action in to_write:
        out = os.path.join(out_dir, f"{bar_id}{suffix}.json")
        with open(out, "w") as f:
            json_dump(action, f, pretty=True)
        print(f"{CALLER}: saved {out}.")

    # Refresh the schedule manifest when a prior batch export created one —
    # pure metadata, so a single-bar refresh keeps it consistent for free. Only
    # files that exist in this bundle are scheduled (the rest are listed under
    # "not_exported"). A range bundle keeps its range and its cell record.
    if schedule is not None:
        schedule_path = os.path.join(root, "ActionSchedule.json")
        try:
            written_files = {
                f"BarActions/{name}" for name in os.listdir(out_dir) if name.endswith(".json")
            }
            payload = hold_action_builder.build_action_schedule_payload(
                seq_map, written_files=written_files, export_range=export_range,
            )
            if schedule.get("cell_rigid_bodies") is not None:
                payload["cell_rigid_bodies"] = schedule["cell_rigid_bodies"]
            with open(schedule_path, "w") as f:
                json.dump(payload, f, indent=2)
            print(f"{CALLER}: refreshed {schedule_path}.")
        except RuntimeError as exc:
            print(f"{CALLER}: ActionSchedule refresh failed ({exc}).")


if __name__ == "__main__":
    main()
