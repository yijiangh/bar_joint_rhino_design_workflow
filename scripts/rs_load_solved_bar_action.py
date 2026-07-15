#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSLoadSolvedBarAction - Load headless-solved BarAction JSON(s) and view them.

This button ONLY loads files; the visualization lives in RSShowBarActionPlan. On load it
(1) caches the loaded ``BarAssemblyAction`` object(s) for the viewer (trajectories
for the motion view), (2) syncs the condensed IK info (base frame + approach /
assembled / retreat configs) back onto each bar's Rhino user-text -- the reverse
of the export -- so RSShowBarActionPlan's keyframe view + a later re-export stay consistent,
and (3) launches RSShowBarActionPlan.

Left-click (this script): pick ONE bar; load its ``<bar_id>.solved_<kind>.json``.
Right-click (``rs_load_solved_bar_action_all.py``): load EVERY
``<bar>.solved_<kind>.json`` in ``<root>/BarActions/``.

Both prompt whether to load the ``solved_keyframe`` sidecar (base + IK keyframe
configs) or the ``solved_motion`` sidecar (planned trajectories). The export root
is shared with the RSExport* commands via ``sc.sticky[EXPORT_ROOT_STICKY_KEY]``.
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc

from compas.data import json_load


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import bar_action as _bar_action_module
from core import config as _config_module
from core import robot_cell as _robot_cell_module
from core import solved_action_cache as _cache_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, get_bar_seq_map, repair_on_entry


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"


def _prompt_kind() -> str | None:
    """Ask whether to load the keyframe or motion sidecar. Returns "keyframe"/"motion"/None."""
    choice = rs.ListBox(
        ["solved_keyframe (base + IK keyframes)", "solved_motion (planned trajectories)"],
        "Which solved sidecar to load?",
        "RSLoadSolvedBarAction",
    )
    if not choice:
        return None
    return "motion" if "motion" in choice else "keyframe"


def _resolve_root() -> str | None:
    """Return the export root (shared sticky), browsing for it if unset/invalid."""
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    if last and os.path.isdir(os.path.join(last, "BarActions")):
        return last
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root (the folder that contains BarActions/)",
        title="RSLoadSolvedBarAction",
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def load_and_view(batch: bool) -> None:
    """Load solved BarAction JSON(s), sync user-text, cache, and launch RSShowBarActionPlan.

    Args:
        batch (bool): False = pick one bar (left-click); True = all bars in the
            folder (right-click).

    Returns:
        None.
    """
    config = importlib.reload(_config_module)
    bar_action = importlib.reload(_bar_action_module)
    robot_cell = importlib.reload(_robot_cell_module)
    cache = importlib.reload(_cache_module)

    repair_on_entry(float(config.BAR_RADIUS), "RSLoadSolvedBarAction")

    kind = _prompt_kind()
    if kind is None:
        return
    root = _resolve_root()
    if not root:
        return
    actions_dir = os.path.join(root, "BarActions")
    if not os.path.isdir(actions_dir):
        rs.MessageBox(f"No BarActions/ under:\n{root}", 0, "RSLoadSolvedBarAction")
        return

    # Which sidecar file(s) to load.
    suffix = f".solved_{kind}.json"
    if batch:
        files = sorted(f for f in os.listdir(actions_dir) if f.endswith(suffix))
        if not files:
            rs.MessageBox(f"No *{suffix} files in:\n{actions_dir}", 0, "RSLoadSolvedBarAction")
            return
    else:
        rs.UnselectAllObjects()
        bar_oid = pick_bar("Pick a bar to load its solved BarAction (Esc to cancel)")
        if bar_oid is None:
            return
        bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)
        if not bar_id:
            rs.MessageBox("Picked curve is not a registered bar.", 0, "RSLoadSolvedBarAction")
            return
        one = f"{bar_id}{suffix}"
        if not os.path.isfile(os.path.join(actions_dir, one)):
            rs.MessageBox(
                f"No solved file for bar '{bar_id}':\n{one}\nunder {actions_dir}",
                0, "RSLoadSolvedBarAction",
            )
            return
        files = [one]

    # rcell for the user-text sync (extract_group_config); needs no PyBullet.
    rcell = robot_cell.get_or_load_robot_cell()
    seq_map = get_bar_seq_map()  # {bar_id: (oid, seq)}

    loaded = {}
    n_synced = 0
    for f in files:
        try:
            action = json_load(os.path.join(actions_dir, f))
        except Exception as exc:  # noqa: BLE001 -- one bad file must not abort the batch
            print(f"  [x] {f}: {type(exc).__name__}: {exc}")
            continue
        bar_id = getattr(action, "active_bar_id", "") or f.split(suffix)[0]
        loaded[bar_id] = action
        oid_seq = seq_map.get(bar_id)
        if oid_seq is not None:
            if bar_action.write_bar_keyframe_from_action(oid_seq[0], action, rcell):
                n_synced += 1
        else:
            print(f"  [note] bar '{bar_id}' from {f} is not in this document; cached but not synced.")
    if not loaded:
        rs.MessageBox("Nothing loaded.", 0, "RSLoadSolvedBarAction")
        return

    cache.set_loaded(loaded, kind)
    print(
        f"RSLoadSolvedBarAction: loaded {len(loaded)} bar(s) ({kind}); "
        f"synced {n_synced} to user-text. Launching RSShowBarActionPlan ..."
    )

    # Launch the viewer. It reads the cache (via solved_action_cache) to know which
    # bar(s) to show. Requires PyBullet; if it isn't running the viewer says so.
    if not robot_cell.is_pb_running():
        rs.MessageBox(
            f"Loaded {len(loaded)} bar(s) and synced their IK to user-text.\n\n"
            "Start PyBullet (RSPBStart), then run RSShowBarActionPlan to view them.",
            0, "RSLoadSolvedBarAction",
        )
        return
    import rs_show_bar_action_plan
    importlib.reload(rs_show_bar_action_plan).main()


def main() -> None:
    # Left-click: single bar.
    load_and_view(batch=False)


if __name__ == "__main__":
    main()
