#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportAllBarActions - Batch-export BarAssemblyAction JSON for every bar.

Right-click companion to ``RSExportBarAction`` (left-click = single picked bar).
Walks every registered bar in assembly-sequence order and builds the five-movement
``BarAssemblyAction`` via ``core.bar_action.build_bar_assembly_action``, writing
``<root>/BarActions/<bar_id>.json``. Bars that already have IK keyframe user-text
(``KEY_ASSEMBLY_BASE_FRAME`` + ``KEY_ASSEMBLY_IK_APPROACH`` +
``KEY_ASSEMBLY_IK_ASSEMBLED``) carry a real base + configs; bars WITHOUT IK are
still exported (placeholder base, no configs) so the headless keyframe solver can
sample their base + IK later. Each BarAction also carries the bar's
``walkable_ground_ids`` (set via RSAssignAndShowWalkableGround).

The cached ``RobotCell`` is the persistent static registry (full canonical
assembly + env obstacles + arm ToolModels, built by RSRebuildRobotCell), so
after the loop this script also dumps ``<root>/RobotCell.json`` and
``<root>/WalkableGround.json`` (every WalkableGround brep meshed, keyed by its
stable id) via plain ``compas.json_dump`` so the cell + grounds + every BarAction
form a consistent bundle for the downstream headless solver / motion planner.

PyBullet must be running (RSPBStart); run RSRebuildRobotCell after geometry
edits. Root folder is shared with RSExportBarAction / RSExportRobotCell via
``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
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
from core import ik_collision_setup as _ik_collision_setup_module
from core import robot_cell as _robot_cell_module
from core.rhino_bar_registry import repair_on_entry

from compas import json_dump


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"


def _prompt_export_root() -> str | None:
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder (all BarActions + RobotCell go here)",
        title="RSExportAllBarActions",
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

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.",
            0,
            "RSExportAllBarActions",
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()

    repair_on_entry(float(config.BAR_RADIUS), "RSExportAllBarActions")

    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSExportAllBarActions: aborted (stale collision cell).")
        return

    from core.rhino_bar_registry import get_bar_seq_map, get_fake_bar_ids
    seq_map = get_bar_seq_map()
    if not seq_map:
        rs.MessageBox("No registered bars found.", 0, "RSExportAllBarActions")
        return
    # Fake bars are staging the robot never assembles -- no action plan to
    # export.  Dropped here rather than inside the loop so the count is reported
    # once, up front: a silently shorter export looks like a partial failure.
    fake_ids = get_fake_bar_ids(seq_map)
    if fake_ids:
        seq_map = {b: v for b, v in seq_map.items() if b not in fake_ids}
        print(
            f"RSExportAllBarActions: skipping {len(fake_ids)} fake bar(s) -- "
            f"{', '.join(sorted(fake_ids))} (RSBarEdit > FakeBar to change)."
        )
        if not seq_map:
            rs.MessageBox(
                "Every registered bar is marked fake; nothing to export.",
                0, "RSExportAllBarActions",
            )
            return
    # Assembly-sequence order so the build / output order is deterministic.
    ordered = sorted(seq_map.items(), key=lambda kv: kv[1][1])  # [(bar_id, (oid, seq)), ...]

    # Export EVERY registered bar, not just the ones with IK. Bars without IK are
    # built with a placeholder base (allow_missing_ik=True) so the headless solver
    # can sample their base + IK later; we only track which had IK for the report.
    # Use the SAME predicate the build uses to decide real-IK vs placeholder, so
    # the reported split can't drift from what actually gets exported.
    all_bars = [(bid, oid) for bid, (oid, _seq) in ordered]
    with_ik_ids = [bid for bid, oid in all_bars if bar_action.has_ik_keyframe(oid)]
    without_ik_ids = [bid for bid, oid in all_bars if not bar_action.has_ik_keyframe(oid)]

    root = _prompt_export_root()
    if not root:
        print("RSExportAllBarActions: cancelled.")
        return

    actions_dir = os.path.join(root, "BarActions")
    os.makedirs(actions_dir, exist_ok=True)

    print(
        f"RSExportAllBarActions: exporting {len(all_bars)} bar(s) "
        f"({len(with_ik_ids)} with IK, {len(without_ik_ids)} without IK: "
        f"{without_ik_ids or '-'})"
    )

    n_ok = 0
    failures = []
    total = len(all_bars)
    # The cell is the persistent static registry; ensure it once up front so the
    # RobotCell.json dumped after the loop is the full assembly. No snapshot/
    # restore -- the cell is meant to carry everything.
    robot_cell.ensure_assembly_cell(rcell, planner)

    # Auto-assign WalkableGround to any bar that has none, up front, so each
    # exported BarAction carries a non-empty `walkable_ground_ids` even if
    # RSRebuildRobotCell's auto-assign was skipped. Non-destructive (keeps manual
    # picks); shares one definition with RSRebuildRobotCell. `grounds` is reused
    # below for the WalkableGround.json dump so the layer is only scanned once.
    from core.rhino_walkable_ground import (
        auto_assign_walkable_ground_ids_all_bars,
        brep_to_compas_mesh,
        get_all_walkable_grounds,
    )
    grounds = get_all_walkable_grounds()  # {ground_id: oid}; also stamps ids
    n_grounds, n_assigned, n_kept, n_noground = auto_assign_walkable_ground_ids_all_bars(grounds)
    print(
        f"RSExportAllBarActions: WalkableGround {n_grounds} surface(s); "
        f"{n_assigned} bar(s) auto-assigned, {n_kept} kept, {n_noground} still none."
    )

    for i, (bar_id, bar_oid) in enumerate(all_bars, start=1):
        print(f"  [{i}/{total}] exporting bar '{bar_id}' ...")
        try:
            action = bar_action.build_bar_assembly_action(
                rcell, planner, bar_id, bar_oid, allow_missing_ik=True
            )
        except Exception as exc:  # noqa: BLE001 -- one bad bar must not abort the batch
            import traceback
            tb = traceback.format_exc().strip().splitlines()
            failures.append((bar_id, f"{type(exc).__name__}: {exc}"))
            print(f"  [x] {bar_id}: {type(exc).__name__}: {exc}")
            print(f"      (last frame: {tb[-2] if len(tb) >= 2 else tb[-1]})")
            continue
        out = os.path.join(actions_dir, f"{bar_id}.json")
        with open(out, "w") as f:
            json_dump(action, f, pretty=True)
        n_ok += 1
        print(f"  [OK] {bar_id} -> {out} ({len(action.movements)} movements)")

    cell_out = os.path.join(root, "RobotCell.json")
    with open(cell_out, "w") as f:
        json_dump(rcell, f, pretty=True)
    print(
        f"  [OK] RobotCell -> {cell_out} "
        f"({len(rcell.rigid_body_models)} rigid bodies, {len(rcell.tool_models)} tools)"
    )

    # Dump every WalkableGround brep as a meshed surface keyed by its stable id,
    # so the headless base sampler can snap to it. Bars reference these ids via
    # their `walkable_ground_ids`. `grounds` was scanned once up front (above).
    ground_meshes = {}
    for gid, oid in grounds.items():
        try:
            ground_meshes[gid] = brep_to_compas_mesh(oid)
        except RuntimeError as exc:
            print(f"  [x] WalkableGround {gid}: {exc}")
    wg_out = os.path.join(root, "WalkableGround.json")
    with open(wg_out, "w") as f:
        json_dump({"grounds": ground_meshes}, f, pretty=True)
    print(f"  [OK] WalkableGround -> {wg_out} ({len(ground_meshes)} ground surface(s))")

    os.makedirs(os.path.join(root, "Trajectories"), exist_ok=True)

    msg = (
        f"Exported {n_ok}/{len(all_bars)} BarAction(s) + RobotCell.json + "
        f"WalkableGround.json ({len(ground_meshes)} ground(s)) to:\n{root}"
    )
    if without_ik_ids:
        msg += f"\n\nNo IK Computed: {', '.join(without_ik_ids)}"
    if failures:
        msg += "\n\nFailed:\n" + "\n".join(f"  {b}: {e}" for b, e in failures)
    rs.MessageBox(msg, 0, "RSExportAllBarActions")
    print(
        f"RSExportAllBarActions: done ({n_ok} exported, {len(failures)} failed, "
        f"{len(without_ik_ids)} without IK)."
    )


if __name__ == "__main__":
    main()
