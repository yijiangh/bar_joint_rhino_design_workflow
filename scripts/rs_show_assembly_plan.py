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
"""RSShowAssemblyPlan - Step through the WHOLE assembly, pose by pose.

RSShowBarActionPlan shows ONE bar's steps. This is the same viewer with every
bar's steps concatenated in the design's schedule order (``core.plan_steps``), so
Enter walks the build from the first bar to the last:

    B20 approach -> B20 assembled -> B20 retreat -> B20 home
    -> B21 approach -> B21 assembled -> B21 hold -> B21 retreat -> B21 home
    -> ... -> B37 home -> B37 release B21 -> B37 release B35 -> ...

Each step is a moment of the design (a movement's start or end), drawn with the
robots exactly where the schedule has them: holders from earlier bars stand
frozen throughout, a bar's own holder joins at its "hold" step, and a hold's
release plays right after its last stabilizing bar.

Bars that the design leaves out (fake, or without a left and a right tool), or
whose keyframe is unsolved, add no assembly steps and are listed up front.

Command line: Enter/Next advances, Prev goes back, Jump asks for a step index,
GoToBar jumps to a bar's first step, clicking a bar jumps there too, MeshMode
flips visual/collision, CheckCollision reports on the current step, and Esc
exits.
"""

from __future__ import annotations

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# The per-bar viewer owns the rendering session; this command only sequences it
# (as rs_ik_keyframe_all reuses rs_ik_keyframe).
import rs_show_bar_action_plan as viewer
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)
from core.rhino_bar_registry import BAR_ID_KEY, get_bar_seq_map


def _reload():
    """Reload the shared viewer so edits land without restarting Rhino."""
    importlib.reload(viewer)


_reload()


def _show(session, idx, bar_map):
    """Show step ``idx``; on a bar change, draw that bar's base frame and walkable grounds."""
    bar_id = session.views[idx].bar_id
    session.index = idx
    if bar_id != session.active_bar_id:
        session.active_bar_id = bar_id
        session.active_bar_oid = bar_map.get(bar_id, (None,))[0]
        viewer.draw_base_frames(session.design, [bar_id])  # replaces the previous bar's marker
        session._show_walkable_grounds()
    session.refresh()


def _ask_jump(steps, current_idx):
    """Prompt for a 1-based step index; returns a 0-based index or None."""
    value = rs.GetInteger(f"Jump to step (1-{len(steps)})", current_idx + 1, 1, len(steps))
    return None if value is None else int(value) - 1


def _first_step_of(session, bar_id):
    """The index of a bar's first step, or None."""
    return next((i for i, view in enumerate(session.views) if view.bar_id == bar_id), None)


def _build_prompt(session):
    """The command-line prompt for the stepper loop."""
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(
        f"[step {session.index + 1}/{len(session.views)} | {session.view.label}] Enter=next, or pick a bar to jump "
        f"(mesh: {session.mesh_mode}), Esc to exit"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)
    go.SetCustomGeometryFilter(_bar_or_tube_filter)
    for name in ("Next", "Prev", "Jump", "GoToBar", "MeshMode", "CheckCollision"):
        go.AddOption(name)
    go.AddOption("HideUnbuilt" if session.show_unbuilt else "ShowUnbuilt")
    return go


def main() -> None:
    """Walk the whole assembly timeline from the Rhino command line."""
    _reload()
    bar_map = get_bar_seq_map()
    if not bar_map:
        rs.MessageBox("No registered bars in this document.", 0, "RSShowAssemblyPlan")
        return

    design = viewer.design_from_document("RSShowAssemblyPlan")
    steps, skipped = viewer.plan_steps.timeline(design)
    if not steps:
        rs.MessageBox(
            "No bar in this document has a solved IK keyframe, so there is "
            "nothing to step through.\n\nRun RSIKKeyframe (or RSIKKeyframeAll) first.",
            0, "RSShowAssemblyPlan",
        )
        return

    # Say what is NOT in the walkthrough, rather than quietly dropping it.
    in_design = {viewer.plan_steps.short(a.bar) for a in design.actions.values()}
    left_out = sorted((b for b in bar_map if b not in in_design), key=lambda b: bar_map[b][1])
    holds = [a for a in design.actions.values() if a.type == "bar_holding"]
    print(f"RSShowAssemblyPlan: {len(steps)} step(s) across "
          f"{len({s.bar for s in steps})} bar(s); {len(holds)} hold(s) in the design.")
    for bar, reason in skipped:
        print(f"  - {viewer.plan_steps.short(bar)}: {reason}")
    if left_out:
        print(f"RSShowAssemblyPlan: not in the design (fake, or no tool pair): {', '.join(left_out)}")

    session = viewer.PlanSession(design, "RSShowAssemblyPlan")
    session.views = viewer.keyframe_views(design, steps)
    try:
        _show(session, 0, bar_map)
        while True:
            go = _build_prompt(session)
            result = go.Get()
            if result == Rhino.Input.GetResult.Cancel:
                break
            if result == Rhino.Input.GetResult.Nothing:
                _show(session, (session.index + 1) % len(steps), bar_map)
                continue
            if result == Rhino.Input.GetResult.Object:
                bar_curve_id = _resolve_picked_to_bar_curve(go.Object(0).ObjectId)
                picked_bar = rs.GetUserText(bar_curve_id, BAR_ID_KEY) if bar_curve_id else None
                target = _first_step_of(session, picked_bar)
                if target is None:
                    print("RSShowAssemblyPlan: picked bar has no steps in this timeline "
                          "(not in the design, or no solved IK keyframe).")
                    continue
                _show(session, target, bar_map)
                continue
            if result == Rhino.Input.GetResult.Option:
                name = go.Option().EnglishName
                if name == "Next":
                    _show(session, (session.index + 1) % len(steps), bar_map)
                elif name == "Prev":
                    _show(session, (session.index - 1) % len(steps), bar_map)
                elif name == "Jump":
                    target = _ask_jump(steps, session.index)
                    if target is not None:
                        _show(session, target, bar_map)
                elif name == "GoToBar":
                    answer = rs.GetString(f"Bar id to jump to (e.g. {session.views[0].bar_id})")
                    target = _first_step_of(session, str(answer).strip()) if answer else None
                    if target is None:
                        print(f"RSShowAssemblyPlan: bar '{answer}' has no steps in this timeline.")
                    else:
                        _show(session, target, bar_map)
                elif name == "MeshMode":
                    session.cycle_mesh_mode()
                elif name == "CheckCollision":
                    session.check_collision()
                elif name in ("ShowUnbuilt", "HideUnbuilt"):
                    session.toggle_unbuilt()
    finally:
        session.cleanup()
        viewer.base_frame_viz.clear_base_frames()


if __name__ == "__main__":
    main()
