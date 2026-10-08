#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportAllBarActions - Batch-export the bars' action files + the schedule.

Right-click companion to ``RSExportBarAction`` (left-click = single picked bar).
First asks which bars to export: press Enter for EVERY bar, or set a range
with the ``FromBar`` / ``UntilBar`` options (click a bar or type its id / step
number; the scene built up to that bar is shown for Accept / Repick). Then
walks the chosen bars in assembly-sequence order and writes per-action files
under ``<root>/BarActions/``:

- ``<bar>__J.json`` / ``<bar>__R.json`` — the assembly robot's jointing and
  release halves (``core.bar_action.build_bar_assembly_actions``). Bars with
  IK keyframe user-text carry a real base + configs; bars WITHOUT IK are still
  exported (placeholder base, no configs) for the headless keyframe solver.
- ``<bar>__H.json`` / ``<bar>__HR.json`` — the support robot's holding and
  holding-release actions for every bar in the hold plan whose support
  keyframe is solved (``core.hold_action_builder``); unsolved holds are
  reported, not silently skipped.

Plus the bundle files at the root: ``RobotCell.json`` (Cindy's cell),
``RobotCell_<robot>.json`` for each support robot actually used,
``WalkableGround.json``, and ``ActionSchedule.json`` — the global interleaved
action order across all robots with explicit robot assignments.

A range export (From .. Until) is a smaller, self-consistent bundle
(``core.export_subset``):

- actions for the bars From .. Until only; a hold is exported when it starts
  in the range (``__H``) and/or releases right after a bar of the range
  (``__HR``);
- the cells and every state keep the bars up to Until and the joint halves
  mounted on them; later bars, their joint halves and the floors no exported
  action stands on are left out;
- every action's ``assembly_seq`` (and the schedule's) stops at Until; the
  bars before From stay in it, as the scene the first bar is built into;
- ``ActionSchedule.json`` records the range and the cell's body names, so a
  later single-bar refresh (left-click) keeps the bundle consistent.

PyBullet must be running (RSPBStart); run RSRebuildRobotCell after geometry
edits. Root folder is shared with RSExportBarAction / RSExportRobotCell via
``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
"""

from __future__ import annotations

import importlib
import json
import os
import re
import sys
import traceback

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

from core import rhino_export_range as _rhino_export_range_module

# After rhino_bar_registry: it imports the sequence-colour view from there.
importlib.reload(_rhino_export_range_module)

from core import bar_action as _bar_action_module
from core import config as _config_module
from core import hold_action_builder as _hold_action_builder_module
from core import ik_collision_setup as _ik_collision_setup_module
from core import robot_cell as _robot_cell_module
from core import robot_cell_support as _robot_cell_support_module
from core import robot_obstacles as _robot_obstacles_module
from core.export_subset import (
    assert_states_match_cell,
    cut_assembly_seq,
    dropped_body_names,
    range_bar_ids,
    trim_action_states,
    trim_robot_cell,
    used_ground_ids,
)
from core.hold_schedule import derive_hold_plan
from core.rhino_bar_registry import (
    collect_hold_inputs,
    get_bar_seq_map,
    get_fake_bar_ids,
    get_real_bar_seq_map,
    repair_on_entry,
)
from core.rhino_export_range import prompt_export_range
from core.rhino_walkable_ground import (
    auto_assign_walkable_ground_ids_all_bars,
    brep_to_compas_mesh,
    get_all_walkable_grounds,
    get_bar_ground_ids,
)

from compas import json_dump


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"
CALLER = "RSExportAllBarActions"

#: An action file this exporter writes: ``<bar>__J|R|H|HR.json``. Other files
#: in ``BarActions/`` (planned motions, ...) are never touched.
_ACTION_FILE_RE = re.compile(r"^.+__(J|R|H|HR)\.json$")


def _prompt_export_root() -> str | None:
    """Ask for the export folder (remembered for the next export).

    Returns:
        str | None: the chosen folder, or None when cancelled.
    """
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder (all BarActions + RobotCell go here)",
        title=CALLER,
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def _ask_remove_leftovers(root: str, planned_files: set) -> list:
    """Offer to delete action / support-cell files an earlier export left behind.

    A range export into a folder that held a bigger export would otherwise
    leave the later bars' files next to the new ones. ``ActionSchedule.json``
    never names them, but other readers (the bundle checker, a folder scan)
    would pick them up with a sequence that no longer matches.

    Args:
        root (str): the export folder.
        planned_files (set): the files (relative to ``root``, forward slashes)
            this export is about to write.

    Returns:
        list: the leftover files that were KEPT (empty when there were none,
        or when the user agreed to delete them).
    """
    leftovers = []
    actions_dir = os.path.join(root, "BarActions")
    for name in sorted(os.listdir(actions_dir)):
        if _ACTION_FILE_RE.match(name) and f"BarActions/{name}" not in planned_files:
            leftovers.append(f"BarActions/{name}")
    for name in sorted(os.listdir(root)):
        if name.startswith("RobotCell_") and name.endswith(".json") and name not in planned_files:
            leftovers.append(name)
    if not leftovers:
        return []
    answer = rs.MessageBox(
        f"The folder holds {len(leftovers)} file(s) from an earlier export that this "
        "export does not write:\n\n"
        + "\n".join(f"  {name}" for name in leftovers[:15])
        + ("\n  ..." if len(leftovers) > 15 else "")
        + "\n\nDelete them? (No keeps them: ActionSchedule.json will not name them, "
        "but anything that reads every file in the folder will.)",
        4 | 32 | 256,  # Yes/No | Question | "No" is the default button
        CALLER,
    )
    if answer != 6:  # 6 == Yes
        return leftovers
    for name in leftovers:
        os.remove(os.path.join(root, name))
        print(f"  [deleted] {name}")
    return []


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
    robot_cell_support = importlib.reload(_robot_cell_support_module)
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

    all_bars_map = get_bar_seq_map()
    if not all_bars_map:
        rs.MessageBox("No registered bars found.", 0, CALLER)
        return
    # Fake bars are staging the robot never assembles -- no action plan to
    # export.  Dropped here rather than inside the loop so the count is reported
    # once, up front: a silently shorter export looks like a partial failure.
    # The same fake-free map feeds every action, the hold plan and the schedule.
    fake_ids = get_fake_bar_ids(all_bars_map)
    seq_map = get_real_bar_seq_map(all_bars_map)
    if fake_ids:
        print(
            f"{CALLER}: skipping {len(fake_ids)} fake bar(s) -- "
            f"{', '.join(sorted(fake_ids))} (RSBarEdit > FakeBar to change)."
        )
        if not seq_map:
            rs.MessageBox("Every registered bar is marked fake; nothing to export.", 0, CALLER)
            return

    # * ---- Which bars: Enter = every bar, or a From .. Until range.
    export_range = prompt_export_range(seq_map, CALLER)
    if export_range is None:
        print(f"{CALLER}: cancelled.")
        return
    from_bar_id, until_bar_id = export_range
    try:
        # The bars whose actions are written, and the built scene they live in
        # (every bar up to Until, the ones before From included).
        range_ids = range_bar_ids(seq_map, from_bar_id, until_bar_id)
        assembly_seq_cut = cut_assembly_seq(seq_map, until_bar_id)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, CALLER)
        return
    is_full = len(range_ids) == len(seq_map)
    from_step = int(seq_map[from_bar_id][1])
    until_step = int(seq_map[until_bar_id][1])
    range_text = (
        f"all {len(range_ids)} bars" if is_full
        else f"bars {from_bar_id} .. {until_bar_id} ({len(range_ids)} of {len(seq_map)})"
    )

    # Assembly-sequence order so the build / output order is deterministic.
    # Export EVERY bar of the range, not just the ones with IK. Bars without IK
    # are built with a placeholder base (allow_missing_ik=True) so the headless
    # solver can sample their base + IK later; we only track which had IK for
    # the report. Use the SAME predicate the build uses to decide real-IK vs
    # placeholder, so the reported split can't drift from what actually gets
    # exported.
    all_bars = [(bid, seq_map[bid][0]) for bid in range_ids]
    with_ik_ids = [bid for bid, oid in all_bars if bar_action.has_ik_keyframe(oid)]
    without_ik_ids = [bid for bid, oid in all_bars if not bar_action.has_ik_keyframe(oid)]

    root = _prompt_export_root()
    if not root:
        print(f"{CALLER}: cancelled.")
        return

    actions_dir = os.path.join(root, "BarActions")
    os.makedirs(actions_dir, exist_ok=True)

    print(
        f"{CALLER}: exporting {range_text} "
        f"({len(with_ik_ids)} with IK, {len(without_ik_ids)} without IK: "
        f"{without_ik_ids or '-'})"
    )

    n_ok = 0
    failures = []
    # Every action built, written only after the whole bundle is cut and
    # checked: (file stem, action, robot name).
    outputs = []
    total = len(all_bars)
    # The cell is the persistent static registry; ensure it once up front so
    # the RobotCell.json dumped below is the assembly (cut to the range). No
    # snapshot/restore -- the cell is meant to carry everything.
    collision_bodies = robot_cell.ensure_assembly_cell(rcell, planner)

    # Auto-assign WalkableGround to any bar that has none, up front, so each
    # exported BarAction carries a non-empty `walkable_ground_ids` even if
    # RSRebuildRobotCell's auto-assign was skipped. Non-destructive (keeps manual
    # picks); shares one definition with RSRebuildRobotCell. `grounds` is reused
    # below for the WalkableGround.json dump so the layer is only scanned once.
    grounds = get_all_walkable_grounds()  # {ground_id: oid}; also stamps ids
    n_grounds, n_assigned, n_kept, n_noground = auto_assign_walkable_ground_ids_all_bars(grounds)
    print(
        f"{CALLER}: WalkableGround {n_grounds} surface(s); "
        f"{n_assigned} bar(s) auto-assigned, {n_kept} kept, {n_noground} still none."
    )

    for i, (bar_id, bar_oid) in enumerate(all_bars, start=1):
        print(f"  [{i}/{total}] building bar '{bar_id}' ...")
        try:
            jointing_action, release_action = bar_action.build_bar_assembly_actions(
                rcell, planner, bar_id, bar_oid, allow_missing_ik=True
            )
        except Exception as exc:  # noqa: BLE001 -- one bad bar must not abort the batch
            tb = traceback.format_exc().strip().splitlines()
            failures.append((bar_id, f"{type(exc).__name__}: {exc}"))
            print(f"  [x] {bar_id}: {type(exc).__name__}: {exc}")
            print(f"      (last frame: {tb[-2] if len(tb) >= 2 else tb[-1]})")
            continue
        outputs.append((f"{bar_id}__J", jointing_action, config.ASSEMBLY_ROBOT_NAME))
        outputs.append((f"{bar_id}__R", release_action, config.ASSEMBLY_ROBOT_NAME))
        n_ok += 1

    # * ---- Holding + holding-release actions for every bar in the hold plan.
    # The plan always comes from the FULL sequence (a bar may name a stabilizer
    # after Until). In a range, a hold contributes its holding action when it
    # starts in the range, and its release when it releases right after a bar
    # of the range. Unsolved holds are listed loudly — never silently skipped.
    holds_exported = []
    holds_pending = []
    # Holds exported with their holding action but still on at Until.
    holds_open = []
    hold_plan = {}
    try:
        bar_seq, supported = collect_hold_inputs(seq_map)
        hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    except RuntimeError as exc:
        failures.append(("<hold plan>", str(exc)))
        print(f"  [x] hold plan derivation failed: {exc}")
    env_union = hold_action_builder.get_env_union(seq_map) if hold_plan else None
    for held_bar_id in sorted(hold_plan, key=lambda b: hold_plan[b]["hold_start_seq"]):
        entry = hold_plan[held_bar_id]
        want_hold = from_step <= entry["hold_start_seq"] <= until_step
        want_release = from_step <= entry["release_after_seq"] <= until_step
        if not (want_hold or want_release):
            continue
        held_oid = seq_map[held_bar_id][0]
        built = []
        try:
            if want_hold:
                built.append(("__H", hold_action_builder.build_bar_holding_action(
                    held_bar_id, held_oid, hold_plan, bar_map=seq_map, env_union=env_union,
                )))
            if want_release:
                built.append(("__HR", hold_action_builder.build_bar_holding_release_action(
                    held_bar_id, held_oid, hold_plan, bar_map=seq_map, env_union=env_union,
                )))
        except RuntimeError as exc:
            holds_pending.append((held_bar_id, str(exc)))
            print(f"  [x] hold {held_bar_id}: {exc}")
            continue
        for suffix, action in built:
            outputs.append((f"{held_bar_id}{suffix}", action, entry["robot_name"]))
        holds_exported.append(held_bar_id)
        if want_hold and not want_release:
            holds_open.append((held_bar_id, entry["release_after_bar_id"]))

    # * ---- Cut the bundle to the range: one set of dropped bodies for the
    # cells and every state (compas_fab needs them to name the same bodies).
    # Floors stay for the grounds the exported actions stand on, plus the
    # grounds of the range's bars (so a bar that failed to build above does
    # not take its floor away from the others).
    used_grounds = used_ground_ids(action for _stem, action, _robot in outputs)
    for _bar_id, bar_oid in all_bars:
        used_grounds.update(get_bar_ground_ids(bar_oid))
    used_support_robots = sorted(
        {robot for _stem, _action, robot in outputs if robot != config.ASSEMBLY_ROBOT_NAME}
    )
    try:
        dropped = dropped_body_names(collision_bodies, seq_map, until_bar_id, used_grounds)
        for _stem, action, _robot in outputs:
            trim_action_states(action, dropped, assembly_seq_cut)
        cells_out = {"RobotCell.json": trim_robot_cell(rcell, dropped)}
        assert_states_match_cell(
            cells_out["RobotCell.json"],
            [a for _s, a, robot in outputs if robot == config.ASSEMBLY_ROBOT_NAME],
            "RobotCell.json",
        )
        for support_name in used_support_robots:
            cell_name = f"RobotCell_{support_name}.json"
            cells_out[cell_name] = trim_robot_cell(
                robot_cell_support.get_or_load_support_cell(support_name), dropped,
            )
            assert_states_match_cell(
                cells_out[cell_name],
                [a for _s, a, robot in outputs if robot == support_name],
                cell_name,
            )
    except RuntimeError as exc:
        rs.MessageBox(f"Export stopped, nothing was written:\n\n{exc}", 0, CALLER)
        print(f"{CALLER}: stopped, nothing written ({exc}).")
        return
    n_dropped_bars = sum(1 for name in dropped if collision_bodies[name].get("kind") == "bar")
    print(
        f"{CALLER}: scene cut at {until_bar_id}: {len(dropped)} bod(ies) left out "
        f"({n_dropped_bars} bar(s) after {until_bar_id}, their joint halves, unused floors)."
    )

    # * ---- Write the bundle.
    planned_files = {f"BarActions/{stem}.json" for stem, _a, _r in outputs}
    planned_files.update(cells_out)
    leftovers_kept = _ask_remove_leftovers(root, planned_files)

    written_files = set()
    for stem, action, _robot in outputs:
        out = os.path.join(actions_dir, f"{stem}.json")
        with open(out, "w") as f:
            json_dump(action, f, pretty=True)
        written_files.add(f"BarActions/{stem}.json")
        print(f"  [OK] {stem} -> {out} ({len(action.movements)} movements)")

    # * ---- The global interleaved schedule (pure metadata, rebuilt fresh).
    # Only the files written THIS run are scheduled; a skipped bar / hold is
    # listed under "not_exported" instead of pointing at a missing file.
    schedule_out = os.path.join(root, "ActionSchedule.json")
    not_exported = []
    try:
        payload = hold_action_builder.build_action_schedule_payload(
            seq_map, written_files=written_files, export_range=(from_bar_id, until_bar_id),
        )
        # The cell's body names, so a later single-bar refresh (RSExportBarAction)
        # can cut its fresh states the same way.
        payload["cell_rigid_bodies"] = sorted(cells_out["RobotCell.json"].rigid_body_models)
        not_exported = payload["not_exported"]
        with open(schedule_out, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  [OK] ActionSchedule -> {schedule_out}")
    except RuntimeError as exc:
        failures.append(("<schedule>", str(exc)))
        print(f"  [x] ActionSchedule failed: {exc}")

    # Cindy's cell, then one cell per support robot this export uses.
    for cell_name, cell in cells_out.items():
        cell_path = os.path.join(root, cell_name)
        with open(cell_path, "w") as f:
            json_dump(cell, f, pretty=True)
        print(
            f"  [OK] {cell_name} -> {cell_path} "
            f"({len(cell.rigid_body_models)} rigid bodies, {len(cell.tool_models)} tools)"
        )

    # Dump the WalkableGround breps as meshed surfaces keyed by their stable
    # ids, so the headless base sampler can snap to them. Bars reference these
    # ids via their `walkable_ground_ids`. A full export keeps every surface (as
    # before); a range export only the ones its actions may stand on.
    ground_meshes = {}
    for gid, oid in grounds.items():
        if not is_full and gid not in used_grounds:
            continue
        try:
            ground_meshes[gid] = brep_to_compas_mesh(oid)
        except RuntimeError as exc:
            print(f"  [x] WalkableGround {gid}: {exc}")
    wg_out = os.path.join(root, "WalkableGround.json")
    with open(wg_out, "w") as f:
        json_dump({"grounds": ground_meshes}, f, pretty=True)
    print(f"  [OK] WalkableGround -> {wg_out} ({len(ground_meshes)} ground surface(s))")

    os.makedirs(os.path.join(root, "Trajectories"), exist_ok=True)

    support_cells = [name for name in cells_out if name != "RobotCell.json"]
    msg = (
        f"Exported {range_text}: {n_ok}/{len(all_bars)} bar(s) (jointing + release) + "
        f"{len(holds_exported)} hold(s) + ActionSchedule.json + RobotCell.json + "
        f"WalkableGround.json ({len(ground_meshes)} ground(s)) to:\n{root}"
    )
    if not is_full:
        msg += (
            f"\n\nScene cut at {until_bar_id}: {n_dropped_bars} later bar(s), their joint "
            f"halves and unused floors left out ({len(dropped)} bodies). Every "
            f"assembly_seq ends at {until_bar_id}."
        )
    if support_cells:
        msg += f"\n\nSupport cells: {', '.join(support_cells)}"
    if without_ik_ids:
        msg += f"\n\nNo IK Computed: {', '.join(without_ik_ids)}"
    if holds_open:
        msg += "\n\nHolds still on at the end of the range (no release exported):\n" + "\n".join(
            f"  {held} (released after {release_bar})" for held, release_bar in holds_open
        )
    if holds_pending:
        msg += "\n\nHolds NOT exported (solve their support keyframes first):\n" + "\n".join(
            f"  {b}: {e}" for b, e in holds_pending
        )
    if failures:
        msg += "\n\nFailed:\n" + "\n".join(f"  {b}: {e}" for b, e in failures)
    if not_exported:
        msg += (
            f"\n\nLeft OUT of ActionSchedule.json (not exported): {len(not_exported)} file(s)\n"
            + "\n".join(f"  {name}" for name in not_exported[:12])
            + ("\n  ..." if len(not_exported) > 12 else "")
        )
    if leftovers_kept:
        msg += (
            f"\n\nKept {len(leftovers_kept)} file(s) from an earlier export that this "
            "export did not write (not in ActionSchedule.json)."
        )
    rs.MessageBox(msg, 0, CALLER)
    print(
        f"{CALLER}: done ({range_text}: {n_ok} bars, {len(holds_exported)} holds, "
        f"{len(holds_pending)} holds pending, {len(failures)} failed, "
        f"{len(without_ik_ids)} without IK)."
    )


if __name__ == "__main__":
    main()
