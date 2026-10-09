#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.core_lib`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
# r: trimesh==4.12.2
"""RSLoadSolvedBarAction - Read a design folder's planned solutions back into the document, then view them.

The planner (``scripts/plan_design.py``) writes ``<design folder>/solutions/``.
This button reads them through the shared core and, for every bar whose insert
is solved, writes the keyframe the plan ended at back onto the bar's user text --
the base frame (the planner's, else the design's) and the approach / assembled /
retreat configs (the ends of the transfer, the insert and the retreat) -- so
RSIKKeyframe, the keyframe viewer and the next RSExportDesign agree with the plan.
Then it opens RSShowBarActionPlan on the loaded bars: their keyframe poses, or
their planned motion.

Left-click (this script): pick ONE bar. Right-click
(``rs_load_solved_bar_action_all.py``): every bar with a solved insert.

The design folder is shared with RSExportDesign / RSCheckDesign (the last one
used is offered).
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

from core import config as _config_module
from core import keyframe_store as _keyframe_store_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, get_bar_seq_map, repair_on_entry


def _prompt_kind() -> str | None:
    """Ask what to view after loading. Returns "keyframe" / "motion" / None."""
    choice = rs.ListBox(["keyframes (base + IK keyframe poses)", "motion (planned trajectories)"],
                        "What to view after loading?", "RSLoadSolvedBarAction")
    if not choice:
        return None
    return "motion" if "motion" in choice else "keyframe"


def load_and_view(batch: bool) -> None:
    """Write solved keyframes back to the document's bars, then launch RSShowBarActionPlan.

    Args:
        batch (bool): False = pick one bar (left-click); True = every bar with a solved insert (right-click).
    """
    import rs_show_bar_action_plan as viewer
    viewer = importlib.reload(viewer)
    config = importlib.reload(_config_module)
    store = importlib.reload(_keyframe_store_module)
    from bar_assembly_core.design import read
    from bar_assembly_core.design.solutions import is_stale, read_solutions

    repair_on_entry(float(config.BAR_RADIUS), "RSLoadSolvedBarAction")
    kind = _prompt_kind()
    if kind is None:
        return
    folder = viewer.design_folder("RSLoadSolvedBarAction")
    if folder is None:
        return
    try:
        design = read(folder)
        solutions = read_solutions(design)
    except Exception as exc:  # noqa: BLE001 -- a broken folder is reported, not raised
        rs.MessageBox(f"Cannot read the design folder:\n{exc}", 0, "RSLoadSolvedBarAction")
        return
    if not solutions:
        rs.MessageBox(f"No solutions in\n{os.path.join(folder, 'solutions')}\n\n"
                      "Plan the design first: python scripts/plan_design.py <design folder>.",
                      0, "RSLoadSolvedBarAction")
        return

    seq_map = get_bar_seq_map()  # {bar_id: (oid, seq)}
    if batch:
        bar_ids = sorted({viewer.plan_steps.short(design.actions[a].bar) for a in solutions},
                         key=lambda b: seq_map.get(b, (None, 1e9))[1])
    else:
        rs.UnselectAllObjects()
        oid = pick_bar("Pick a bar to load its solved plan (Esc to cancel)")
        if oid is None:
            return
        bar_id = rs.GetUserText(oid, BAR_ID_KEY)
        if not bar_id:
            rs.MessageBox("Picked curve is not a registered bar.", 0, "RSLoadSolvedBarAction")
            return
        bar_ids = [bar_id]

    loaded = []
    for bar_id in bar_ids:
        bar = f"bars/{bar_id}"
        stale = [a for a in (f"{bar_id}_J", f"{bar_id}_R") if a in solutions and is_stale(design, solutions[a])]
        if stale:
            print(f"  [note] {bar_id}: stale solution(s) {', '.join(stale)} (the design changed since planning).")
        keyframes = viewer.plan_steps.solved_keyframes(design, solutions, bar)
        if keyframes is None:
            print(f"  [x] {bar_id}: no solved insert in the solutions; nothing written.")
            continue
        if bar_id not in seq_map:
            print(f"  [note] bar '{bar_id}' is not in this document; not written.")
            continue
        oid = seq_map[bar_id][0]
        groups = viewer.plan_steps.arm_groups
        store.write_assembly_base_frame(oid, viewer.base_frame_mm(keyframes["base"]))
        store.write_assembly_keyframes(oid, groups(keyframes["approach"]), groups(keyframes["assembled"]),
                                       groups(keyframes["retreat"]) if keyframes["retreat"] else None)
        if not keyframes["retreat"]:
            print(f"  [note] {bar_id}: no solved retreat; its stored retreat keyframe (if any) is kept as it was.")
        loaded.append(bar_id)
    if not loaded:
        rs.MessageBox("Nothing loaded: no picked bar has a solved insert in this design folder.",
                      0, "RSLoadSolvedBarAction")
        return
    print(f"RSLoadSolvedBarAction: wrote the solved keyframes of {len(loaded)} bar(s) to user text: "
          f"{', '.join(loaded)}. Launching RSShowBarActionPlan ...")

    if kind == "motion":
        first = loaded[0]
        viewer.main_motion(folder, (first, seq_map[first][0]) if not batch else None)
        return
    sc.sticky[viewer.LOADED_STICKY_KEY] = {"bars": loaded, "folder": folder}
    viewer.main()


def main() -> None:
    # Left-click: single bar.
    load_and_view(batch=False)


if __name__ == "__main__":
    main()
