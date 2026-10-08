#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowAssemblyPlan - Step through the WHOLE assembly, movement by movement.

RSShowBarActionPlan shows ONE bar's timeline. This is the same viewer with
every bar concatenated in assembly-sequence order, so Enter walks the build
from the first bar to the last exactly as ``ActionSchedule.json`` records it:

    B20 approach -> B20 assembled -> B20 retreat -> B20 home
    -> B21 approach -> B21 assembled -> B21 hold -> B21 retreat -> B21 home
    -> ... -> B37 home -> release B21 -> release B35 -> ...

Each step is one movement ENDPOINT (the poses that have a solved keyframe);
the tool and manual movements between them -- gripper open/close, the
screw-driving, the operator mounting the bar -- are printed alongside each
step rather than drawn, since they are instants at a pose already on screen.

Support robots appear per pose exactly as in the per-bar viewer: holders from
earlier bars stand frozen throughout, a bar's own holder joins at its "hold"
step, and a hold's release plays right after its last stabilizing bar.

Bars that are fake, or that have no solved IK keyframe yet, contribute no
steps and are listed up front.

Command line: Enter/Next advances, Prev goes back, Jump asks for a step index,
GoToBar jumps to a bar's first step, clicking a bar jumps there too, MeshMode
flips visual/collision, CheckCollision reports on the current pose, and
Done/Esc exits.
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

# The per-bar viewer owns the rendering session, the pose taxonomy and the
# timeline builder; this command only sequences them (mirrors how
# rs_ik_keyframe_all reuses rs_ik_keyframe).
import rs_show_bar_action_plan as viewer
from core import base_frame_viz as _base_frame_viz_module
from core import robot_cell as _robot_cell_module
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)
from core.rhino_bar_registry import BAR_ID_KEY, get_bar_seq_map


def _reload():
    """Reload the shared modules so edits land without restarting Rhino."""
    global base_frame_viz, robot_cell
    importlib.reload(viewer)
    viewer._reload()
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)


_reload()


def _step_caption(steps, idx) -> str:
    """One-line progress caption for step ``idx``."""
    bar_id, _oid, pose = steps[idx]
    return (
        f"step {idx + 1}/{len(steps)}  |  bar {bar_id}  |  "
        f"{viewer._pose_label(pose)}"
    )


def _apply_step(session, steps, idx, hold_plan):
    """Render step ``idx``, switching bars when the step crosses into a new one.

    Args:
        session (viewer._PreviewSession): the rendering session.
        steps (list): the ``(bar_id, bar_oid, pose)`` timeline.
        idx (int): which step to show.
        hold_plan (dict): the run's hold plan (reused, not re-derived per bar).
    """
    bar_id, bar_oid, pose = steps[idx]
    if bar_id != session.active_bar_id:
        # A bar switch rebuilds the collision cell + preview cache, so land on
        # the wanted pose in that same pass rather than rendering twice.
        session.set_active_bar(bar_id, bar_oid, hold_plan=hold_plan, pose=pose)
        # Draw only the active bar's base frame (draw_base_frames clears its
        # own layer first, so the previous bar's marker goes away).
        payload = viewer._load_assembly_payload(bar_oid)
        if payload is not None:
            base_frame_viz.draw_base_frames({bar_id: payload["base_frame_world_mm"]})
    elif session.pose != pose:
        session.pose = pose
        session.refresh()
    print(f"RSShowAssemblyPlan: {_step_caption(steps, idx)}")


def _ask_jump(steps, current_idx):
    """Prompt for a 1-based step index; returns a 0-based index or None."""
    value = rs.GetInteger(
        f"Jump to step (1-{len(steps)})", current_idx + 1, 1, len(steps)
    )
    if value is None:
        return None
    return int(value) - 1


def _ask_go_to_bar(steps, bar_ids):
    """Prompt for a bar id; returns the index of its FIRST step, or None."""
    answer = rs.GetString(f"Bar id to jump to (e.g. {bar_ids[0]})")
    if not answer:
        return None
    wanted = str(answer).strip()
    for i, (bar_id, _oid, _pose) in enumerate(steps):
        if bar_id == wanted:
            return i
    print(f"RSShowAssemblyPlan: bar '{wanted}' has no steps in this timeline.")
    return None


def _build_prompt(session, steps, idx):
    """The command-line prompt for the stepper loop."""
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(
        f"[{_step_caption(steps, idx)}] Enter=next, or pick a bar to jump "
        f"(mesh: {session.mesh_mode}), Esc to exit"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)
    go.SetCustomGeometryFilter(_bar_or_tube_filter)
    go.AddOption("Next")
    go.AddOption("Prev")
    go.AddOption("Jump")
    go.AddOption("GoToBar")
    go.AddOption("MeshMode")
    go.AddOption("CheckCollision")
    if session.show_unbuilt:
        go.AddOption("HideUnbuilt")
    else:
        go.AddOption("ShowUnbuilt")
    return go


def main() -> None:
    """Walk the whole assembly timeline from the Rhino command line."""
    _reload()

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, "RSShowAssemblyPlan")
        return
    _client, planner = robot_cell.get_planner()

    rcell = robot_cell.get_or_load_robot_cell()
    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSShowAssemblyPlan: aborted (stale collision cell).")
        return

    bar_map = get_bar_seq_map()
    if not bar_map:
        rs.MessageBox("No registered bars in this document.", 0, "RSShowAssemblyPlan")
        return

    hold_plan = viewer.derive_hold_plan_safe("RSShowAssemblyPlan")
    steps, skipped = viewer.build_global_timeline(hold_plan=hold_plan, bar_map=bar_map)
    if not steps:
        rs.MessageBox(
            "No bar in this document has a solved IK keyframe, so there is "
            "nothing to step through.\n\nRun RSIKKeyframe (or RSIKKeyframeAll) first.",
            0, "RSShowAssemblyPlan",
        )
        return

    # Say what is NOT in the walkthrough, rather than quietly dropping it.
    bar_ids = sorted({bar_id for bar_id, _oid, _pose in steps},
                     key=lambda b: bar_map[b][1])
    print(
        f"RSShowAssemblyPlan: {len(steps)} step(s) across {len(bar_ids)} bar(s); "
        f"{len(hold_plan)} hold(s) in the plan."
    )
    if skipped:
        print(f"RSShowAssemblyPlan: {len(skipped)} bar(s) contribute no steps:")
        for bar_id, reason in skipped:
            print(f"  - {bar_id}: {reason}")
    unsolved_holds = [
        b for b in hold_plan
        if b in bar_map and viewer._load_support_payload(bar_map[b][0]) is None
    ]
    if unsolved_holds:
        print(
            "RSShowAssemblyPlan: hold/release steps are missing for "
            f"{', '.join(sorted(unsolved_holds))} (no solved support keyframe yet)."
        )

    session = viewer._PreviewSession(planner)
    idx = 0
    try:
        _apply_step(session, steps, idx, hold_plan)

        while True:
            go = _build_prompt(session, steps, idx)
            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                break

            if result == Rhino.Input.GetResult.Nothing:
                idx = (idx + 1) % len(steps)
                _apply_step(session, steps, idx, hold_plan)
                continue

            if result == Rhino.Input.GetResult.Object:
                bar_curve_id = _resolve_picked_to_bar_curve(go.Object(0).ObjectId)
                picked_bar = rs.GetUserText(bar_curve_id, BAR_ID_KEY) if bar_curve_id else None
                target = next(
                    (i for i, (b, _o, _p) in enumerate(steps) if b == picked_bar), None
                )
                if target is None:
                    print(
                        "RSShowAssemblyPlan: picked bar has no steps in this "
                        "timeline (fake, or no solved IK keyframe)."
                    )
                    continue
                idx = target
                _apply_step(session, steps, idx, hold_plan)
                continue

            if result == Rhino.Input.GetResult.Option:
                name = go.Option().EnglishName
                if name == "Next":
                    idx = (idx + 1) % len(steps)
                    _apply_step(session, steps, idx, hold_plan)
                elif name == "Prev":
                    idx = (idx - 1) % len(steps)
                    _apply_step(session, steps, idx, hold_plan)
                elif name == "Jump":
                    target = _ask_jump(steps, idx)
                    if target is not None:
                        idx = target
                        _apply_step(session, steps, idx, hold_plan)
                elif name == "GoToBar":
                    target = _ask_go_to_bar(steps, bar_ids)
                    if target is not None:
                        idx = target
                        _apply_step(session, steps, idx, hold_plan)
                elif name == "MeshMode":
                    session.cycle_mesh_mode()
                elif name == "CheckCollision":
                    session.check_collision()
                elif name in ("ShowUnbuilt", "HideUnbuilt"):
                    session.toggle_unbuilt()
                continue
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()


if __name__ == "__main__":
    main()
