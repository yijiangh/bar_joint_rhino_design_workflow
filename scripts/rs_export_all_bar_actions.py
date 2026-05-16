#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportAllBarActions - Batch-export BarAssemblyAction JSON for every bar with IK results.

Right-click companion to ``RSExportBarAction`` (left-click = single picked bar).
Walks every registered bar in assembly-sequence order; for each one that
carries the IK keyframe user-text (``KEY_ASSEMBLY_BASE_FRAME`` +
``KEY_ASSEMBLY_IK_APPROACH`` + ``KEY_ASSEMBLY_IK_ASSEMBLED``) it builds the
four-movement ``BarAssemblyAction`` via ``core.bar_action.build_bar_assembly_action``
and writes ``<root>/BarActions/<bar_id>.json``. Bars without IK results are
skipped with a note.

Each ``build_bar_assembly_action`` call also registers the FULL assembly
(every bar + joint) onto the cached ``RobotCell`` (see
``core.bar_action._register_full_assembly_geom``), so after the loop the cell
carries the complete state-independent body set; this script then also dumps
``<root>/RobotCell.json`` (canonical rigid-body names) so the cell + every
BarAction form a consistent bundle for the downstream motion planner.

PyBullet must be running (RSPBStart). Root folder is shared with
RSExportBarAction / RSExportRobotCell via ``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
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


def _has_ik_results(bar_oid, config) -> bool:
    return bool(
        rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
        and rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
        and rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    )


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

    from core.rhino_bar_registry import get_bar_seq_map
    seq_map = get_bar_seq_map()
    if not seq_map:
        rs.MessageBox("No registered bars found.", 0, "RSExportAllBarActions")
        return
    # Assembly-sequence order so the build / output order is deterministic.
    ordered = sorted(seq_map.items(), key=lambda kv: kv[1][1])  # [(bar_id, (oid, seq)), ...]

    with_ik = [(bid, oid) for bid, (oid, _seq) in ordered if _has_ik_results(oid, config)]
    skipped = [bid for bid, (oid, _seq) in ordered if not _has_ik_results(oid, config)]
    if not with_ik:
        rs.MessageBox(
            "No bars have IK results attached. Run RSIKKeyframe on at least one bar first.",
            0,
            "RSExportAllBarActions",
        )
        return

    root = _prompt_export_root()
    if not root:
        print("RSExportAllBarActions: cancelled.")
        return

    actions_dir = os.path.join(root, "BarActions")
    os.makedirs(actions_dir, exist_ok=True)

    print(
        f"RSExportAllBarActions: {len(with_ik)} bar(s) with IK results; "
        f"{len(skipped)} skipped (no IK): {skipped or '-'}"
    )

    n_ok = 0
    failures = []
    total = len(with_ik)
    for i, (bar_id, bar_oid) in enumerate(with_ik, start=1):
        print(f"  [{i}/{total}] exporting bar '{bar_id}' ...")
        try:
            action = bar_action.build_bar_assembly_action(rcell, planner, bar_id, bar_oid)
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

    # Re-assert the full-assembly registration before dumping the cell --
    # belt-and-suspenders, in case the last loop iteration failed AFTER
    # `prepare_assembly_collision_state` (which shrinks the cell to that
    # bar's active context) but BEFORE `_register_full_assembly_geom`.
    bar_action._register_full_assembly_geom(rcell, planner)
    bar_action._attach_arm_tools_to_cell(rcell, planner)
    cell_out = os.path.join(root, "RobotCell.json")
    with open(cell_out, "w") as f:
        bar_action.dump_cell_canonical(rcell, f, pretty=True)
    print(f"  [OK] RobotCell -> {cell_out} ({len(rcell.rigid_body_models)} rigid bodies, names canonicalized)")

    os.makedirs(os.path.join(root, "Trajectories"), exist_ok=True)

    msg = f"Exported {n_ok}/{len(with_ik)} BarAction(s) + RobotCell.json to:\n{root}"
    if skipped:
        msg += f"\n\nSkipped (no IK results): {', '.join(skipped)}"
    if failures:
        msg += "\n\nFailed:\n" + "\n".join(f"  {b}: {e}" for b, e in failures)
    rs.MessageBox(msg, 0, "RSExportAllBarActions")
    print(f"RSExportAllBarActions: done ({n_ok} exported, {len(failures)} failed, {len(skipped)} skipped).")


if __name__ == "__main__":
    main()
