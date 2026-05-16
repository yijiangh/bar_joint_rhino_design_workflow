#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSExportBarAction - Save the per-bar `BarAssemblyAction` to JSON.

Pick a bar; the script reads its IK keyframe data (`KEY_ASSEMBLY_*` user-text
written by ``rs_ik_keyframe.py``), reuses
``core.ik_collision_setup.prepare_assembly_collision_state`` to seed the
per-bar collision context (tool RBs + env_*/active_* RBs + ACM via
``configure_active_assembly_acm``), then builds the four movements via
``core.bar_action.build_bar_assembly_action`` and writes them to
``<root>/BarActions/<bar_id>.json`` using ``compas.json_dump``.

Side effect: ``prepare_assembly_collision_state`` registers env bars +
arm-tool RBs into ``rcell.rigid_body_models`` (cached). A subsequent
``RSExportRobotCell`` will therefore save a cell that carries those RBs
(with names canonicalized via ``core.bar_action.dump_cell_canonical``).

Root folder is shared with RSExportRobotCell via
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
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry

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

    # B11: build_bar_assembly_action pollutes the cached cell with future-bar
    # + arm-tool RBs (needed to make the per-bar M states reference a stable
    # superset cell). Snapshot BEFORE the build call and restore at the end so
    # subsequent ShowIK / IK keyframe in this Rhino session see the cell as
    # it was before the export.
    rb_snapshot = bar_action.snapshot_cell_rigid_bodies(rcell)
    try:
        try:
            action = bar_action.build_bar_assembly_action(rcell, planner, bar_id, bar_oid)
        except RuntimeError as exc:
            rs.MessageBox(str(exc), 0, "RSExportBarAction")
            return

        n_seq = len(action.assembly_seq)
        try:
            idx = action.assembly_seq.index(action.active_bar_id)
        except ValueError:
            idx = -1
        print(
            f"RSExportBarAction: built {len(action.movements)} movement(s) for bar "
            f"'{action.active_bar_id}' (assembly index {idx}/{n_seq})."
        )
        for mv in action.movements:
            n_rbs = len(mv.start_state.rigid_body_states) if mv.start_state else 0
            cfg_status = "None" if (mv.start_state is None or mv.start_state.robot_configuration is None) else "set"
            print(
                f"  - {mv.movement_id}: {type(mv).__name__}, "
                f"start_state.config={cfg_status}, rb_states={n_rbs}, "
                f"target_ee_frames={'yes' if mv.target_ee_frames else 'no'}, "
                f"target_configuration={'yes' if mv.target_configuration is not None else 'no'}"
            )

        root = _prompt_export_root()
        if not root:
            print("RSExportBarAction: cancelled.")
            return

        out_dir = os.path.join(root, "BarActions")
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, f"{bar_id}.json")

        if os.path.exists(out):
            ans = rs.MessageBox(
                f"'{out}' exists. Overwrite?",
                4 | 32,  # YesNo | Question
                "RSExportBarAction",
            )
            if ans != 6:  # 6 == Yes
                print("RSExportBarAction: cancelled (kept existing file).")
                return

        with open(out, "w") as f:
            json_dump(action, f, pretty=True)
        print(f"RSExportBarAction: saved {out} for bar '{bar_id}'.")
    finally:
        bar_action.restore_cell_rigid_bodies(rcell, rb_snapshot, planner)
        print(
            f"RSExportBarAction: restored cached cell to pre-export state "
            f"({len(rcell.rigid_body_models)} rigid_body_models)."
        )


if __name__ == "__main__":
    main()
