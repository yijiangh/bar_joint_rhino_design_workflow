#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSInspectRoboticTool - inspect robotic tool candidates in PyBullet.

Opens a PyBullet GUI with a slider that steps through the tool candidates in
``robotic_tools.json`` one left/right pair at a time. For the selected pair it:

  - loads the left tool's collision ``.obj`` at the world origin and the right
    tool's ``.obj`` 0.30 m along +X (the ``.obj`` files are in millimetres, so
    they are scaled by 0.001 into the metre-based PyBullet world),
  - draws two coordinate triads per tool:
      * a LONGER triad at the mesh base = the tool0 / robot-flange frame (the
        ``.obj`` is baked at the flange), and
      * a SHORTER triad at the TCP frame taken from ``M_tcp_from_block`` (the
        TCP expressed in the tool0 frame, so its world pose is
        ``base_pose @ M_tcp_from_block`` with the translation in metres).

Deletion uses a two-step, GUI-only confirmation (no terminal needed): click
``DELETE candidate (step 1: arm)`` to arm -- a red warning lists exactly what
will be removed -- then click ``CONFIRM DELETE (step 2)`` to actually remove
both JSON entries and their ``.3dm`` + ``.obj`` files. Moving the slider cancels
a pending delete. A file that another remaining tool still references is kept.
Deleting the registry's ``active`` tool is not blocked; that ``active`` side is
simply cleared (refill it later in Rhino with RSSwapRoboticTool).

How to run
----------
Primary use is the **RSInspectRoboticTool** toolbar button (RSSetup group),
which runs this file through the Rhino ScriptEditor. The ``# r:`` header above
provisions numpy + pybullet + pybullet_planning into the ``scaffolding_env``
venv (the same one ``rs_ik_keyframe.py`` uses); a separate PyBullet window
opens. No Rhino geometry is touched -- it only reads / edits robotic_tools.json.

It also runs standalone from any interpreter that already has those packages
(e.g. the tamp venv), where ``--no-gui`` renders every candidate once as a
headless smoke test::

    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe scripts\\rs_inspect_robotic_tool.py --no-gui

Distances shown in the GUI are in metres; the registry stores millimetres.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np


# ---- Make the sibling ``core`` package importable ----------------------------
# The tool data model + JSON I/O live in scripts/core/robotic_tool.py (pure
# Python + numpy, no Rhino). This file sits in scripts/, so adding its own
# directory to sys.path is enough to import the ``core`` package.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import pybullet as p
import pybullet_planning as pp

from core.robotic_tool import (
    RoboticToolDef,
    arm_side_from_tool_name,
    delete_robotic_tools,
    load_robotic_tools,
)


# ---- Layout / appearance constants -------------------------------------------
LEFT_BASE_XYZ = (0.0, 0.0, 0.0)         # left tool sits at the world origin
RIGHT_BASE_XYZ = (0.30, 0.0, 0.0)       # right tool sits 0.30 m along +X
LEFT_COLOR = (0.90, 0.10, 0.10, 1.0)    # RED left tool mesh + labels
RIGHT_COLOR = (0.10, 0.75, 0.15, 1.0)   # GREEN right tool mesh + labels
BASE_FRAME_LEN = 0.06                   # tool0 / mesh-base triad axis length (m)
TCP_FRAME_LEN = 0.04                    # TCP triad axis length (m); shorter on purpose
MM_TO_M = 0.001                         # registry is in mm; PyBullet world is in m


# ---- Data holders ------------------------------------------------------------
@dataclass
class Candidate:
    """One tool candidate = a left/right pair sharing a name prefix.

    Attributes:
        prefix (str): shared name prefix, e.g. ``"AT4_E3_"`` (the name minus
            its trailing L/R). Used only as a label.
        left (RoboticToolDef | None): the ...L tool, or None if not registered.
        right (RoboticToolDef | None): the ...R tool, or None if not registered.
    """

    prefix: str
    left: "RoboticToolDef | None"
    right: "RoboticToolDef | None"


@dataclass
class Scene:
    """Tracks what is currently drawn so we can clear it before redrawing.

    Attributes:
        bodies (list): PyBullet body unique ids of loaded tool meshes.
        handles (list): PyBullet debug-item ids (frame lines + text labels).
    """

    bodies: list = field(default_factory=list)
    handles: list = field(default_factory=list)


# ---- Candidate discovery -----------------------------------------------------
def build_candidates() -> list:
    """Group every registered tool into left/right candidate pairs.

    Tools are grouped by their name prefix (the name minus its trailing L/R
    suffix). A tool whose name has no L/R suffix is shown on its own at the
    origin (treated as a lone "left").

    Returns:
        list[Candidate]: candidates sorted by prefix.
    """
    tools = load_robotic_tools()
    # prefix -> {"left": def|None, "right": def|None}
    groups: dict[str, dict] = {}
    for name, tool in tools.items():
        side = arm_side_from_tool_name(name)
        prefix = name[:-1] if side is not None else name
        slot = groups.setdefault(prefix, {"left": None, "right": None})
        if side == "right":
            slot["right"] = tool
        else:
            # "left" or suffix-less -> render at the origin.
            slot["left"] = tool

    return [
        Candidate(prefix=prefix, left=groups[prefix]["left"], right=groups[prefix]["right"])
        for prefix in sorted(groups)
    ]


# ---- Frame math --------------------------------------------------------------
def tcp_pose_in_world(base_pose, m_tcp_from_block: np.ndarray):
    """Compute the world TCP pose from a tool's base pose and its TCP matrix.

    ``M_tcp_from_block`` is the TCP frame expressed in the tool0/mesh-base
    frame, with translation in millimetres. We convert the translation to
    metres and compose it onto the base pose.

    Args:
        base_pose: PyBullet pose ``(point, quat_xyzw)`` where the mesh sits.
        m_tcp_from_block (np.ndarray): 4x4 TCP-in-block transform (mm).

    Returns:
        PyBullet pose ``(point, quat_xyzw)`` of the TCP frame in the world.
    """
    matrix_m = np.array(m_tcp_from_block, dtype=float, copy=True)
    matrix_m[:3, 3] *= MM_TO_M  # mm -> m on the translation column only
    local_tcp = pp.pose_from_tform(matrix_m)
    return pp.multiply(base_pose, local_tcp)


def _text_pos(base_xyz, dz: float = 0.015):
    """Return a point slightly above ``base_xyz`` for a readable text label."""
    return (base_xyz[0], base_xyz[1], base_xyz[2] + dz)


# ---- Rendering ---------------------------------------------------------------
def _render_side(tool: RoboticToolDef, base_xyz, color, scene: Scene) -> None:
    """Load one tool's mesh at ``base_xyz`` and draw its tool0 + TCP frames.

    Args:
        tool (RoboticToolDef): the tool to render.
        base_xyz: world XYZ (metres) for the tool's mesh base.
        color: RGBA tuple for the mesh and its text labels.
        scene (Scene): scene tracker to append the new body + handles to.
    """
    base_pose = (tuple(base_xyz), (0.0, 0.0, 0.0, 1.0))

    # Load the collision mesh (mm -> m). Missing OBJ is non-fatal: we still draw
    # the frames so you can at least inspect the geometry-free transform.
    col_path = tool.collision_path()
    if col_path and os.path.exists(col_path):
        body = pp.create_obj(col_path, scale=MM_TO_M, color=color)
        pp.set_pose(body, base_pose)
        scene.bodies.append(body)
    else:
        print(f"[warn] {tool.name}: collision OBJ not found ({col_path!r}); drawing frames only.")

    # tool0 / mesh-base frame -- the OBJ is baked at the robot flange, so the
    # mesh origin IS the tool0 frame. Draw the longer triad here.
    scene.handles.extend(pp.draw_pose(base_pose, length=BASE_FRAME_LEN))
    scene.handles.append(pp.add_text(f"{tool.name} tool0", position=_text_pos(base_xyz), color=color))

    # TCP frame from M_tcp_from_block, drawn relative to the base. Shorter triad.
    tcp_pose = tcp_pose_in_world(base_pose, tool.M_tcp_from_block)
    scene.handles.extend(pp.draw_pose(tcp_pose, length=TCP_FRAME_LEN))
    scene.handles.append(pp.add_text(f"{tool.name} tcp", position=tcp_pose[0], color=color))


def clear_scene(scene: Scene) -> None:
    """Remove every body + debug item recorded in ``scene`` (leaves controls)."""
    for body in scene.bodies:
        try:
            p.removeBody(body)
        except p.error:
            pass  # already gone (e.g. after a disconnect) -- ignore
    if scene.handles:
        pp.remove_handles(scene.handles)
    scene.bodies = []
    scene.handles = []


def render_candidate(cand: Candidate, scene: Scene) -> None:
    """Clear the scene and draw both sides of one candidate pair.

    Args:
        cand (Candidate): the candidate to display.
        scene (Scene): scene tracker (cleared first, then repopulated).
    """
    clear_scene(scene)

    shown = []
    if cand.left is not None:
        _render_side(cand.left, LEFT_BASE_XYZ, LEFT_COLOR, scene)
        shown.append(f"L={cand.left.name}")
    if cand.right is not None:
        _render_side(cand.right, RIGHT_BASE_XYZ, RIGHT_COLOR, scene)
        shown.append(f"R={cand.right.name}")

    missing = [side for side, tool in (("left", cand.left), ("right", cand.right)) if tool is None]
    msg = f"[show] candidate '{cand.prefix}': " + (", ".join(shown) if shown else "(nothing to show)")
    if missing:
        msg += f"  (no {'/'.join(missing)} side registered)"
    print(msg)


# ---- Delete flow (two-step, GUI-only confirmation) ---------------------------
def _delete_targets(cand: Candidate):
    """Return ``(names, file_paths)`` that deleting this candidate would touch.

    Args:
        cand (Candidate): the candidate to inspect.

    Returns:
        tuple[list[str], list[str]]: registry names and the absolute .3dm/.obj
        paths of both present sides (empty ``names`` => nothing to delete).
    """
    tools = [tool for tool in (cand.left, cand.right) if tool is not None]
    names = [tool.name for tool in tools]
    files = [
        file_path
        for tool in tools
        for file_path in (tool.asset_path(), tool.collision_path())
        if file_path
    ]
    return names, files


def _draw_delete_warning(cand: Candidate) -> list:
    """Draw a red in-GUI banner describing the armed delete; return its handles."""
    names, _ = _delete_targets(cand)
    lines = [
        f"ARMED: delete candidate '{cand.prefix}' -> {', '.join(names)}",
        "Click 'CONFIRM DELETE (step 2)' to remove entries + .3dm/.obj files.",
        "Move the slider to CANCEL.",
    ]
    red = (1.0, 0.2, 0.2, 1.0)
    handles = []
    for i, line in enumerate(lines):
        handles.append(pp.add_text(line, position=(0.0, 0.0, 0.34 - i * 0.03), color=red))
    return handles


def do_delete(cand: Candidate) -> bool:
    """Delete the whole candidate pair (entries + files) and print a summary.

    Confirmation is handled by the caller's two-button flow, so this deletes
    unconditionally. Any ``active`` side pointing at a deleted tool is cleared
    (not blocked); a file another remaining tool still references is kept.

    Args:
        cand (Candidate): the candidate to delete.

    Returns:
        bool: True if the registry changed on disk, else False.
    """
    names, _ = _delete_targets(cand)
    if not names:
        print("[delete] nothing to delete for this candidate.")
        return False

    summary = delete_robotic_tools(names, remove_files=True)
    print(f"[delete] removed entries : {summary['removed']}")
    print(f"[delete] deleted files   : {summary['files_deleted']}")
    if summary["files_kept_referenced"]:
        print(f"[delete] kept (shared)   : {summary['files_kept_referenced']}")
    if summary["files_missing"]:
        print(f"[delete] files not found : {summary['files_missing']}")
    if summary["cleared_active_sides"]:
        print(
            "[delete] cleared active sides "
            f"{summary['cleared_active_sides']} -- refill in Rhino via RSSwapRoboticTool."
        )
    return True


# ---- Controls ----------------------------------------------------------------
def _make_controls(n_candidates: int) -> dict:
    """Create the slider + the two delete buttons; return ids and baselines.

    Args:
        n_candidates (int): number of candidates (>= 1) the slider spans.

    Returns:
        dict: ``slider`` / ``arm_btn`` / ``confirm_btn`` debug-item ids plus
        ``arm_base`` / ``confirm_base`` -- each button's current click count (a
        press makes the count exceed its baseline).
    """
    # Slider spans 0..N so every candidate gets a full unit-wide bucket; reading
    # it and flooring turns each whole step into the next tool (see _read_index).
    slider = p.addUserDebugParameter("candidate index", 0, n_candidates, 0)
    # min > max makes these buttons; reading one returns its click counter.
    arm_btn = p.addUserDebugParameter("DELETE candidate (step 1: arm)", 1, 0, 0)
    confirm_btn = p.addUserDebugParameter("CONFIRM DELETE (step 2)", 1, 0, 0)
    return {
        "slider": slider,
        "arm_btn": arm_btn,
        "confirm_btn": confirm_btn,
        "arm_base": p.readUserDebugParameter(arm_btn),
        "confirm_base": p.readUserDebugParameter(confirm_btn),
    }


def _read_index(slider, n_candidates: int) -> int:
    """Read the slider and floor it to a whole candidate index.

    Flooring (not rounding) makes the selection change exactly at each whole
    number, so dragging one full unit steps to the next tool -- no half-step
    (X.5) boundaries. ``n_candidates`` is the CURRENT count, so the index stays
    valid even after a delete shrinks the list under the (fixed) slider range.
    """
    value = p.readUserDebugParameter(slider)
    return max(0, min(n_candidates - 1, int(value)))


def _help_text() -> str:
    return (
        "\n[inspect] Drag 'candidate index' to switch candidate pairs.\n"
        "[inspect]   left tool  -> world origin,  right tool -> +0.30 m along X\n"
        "[inspect]   long triad = tool0 / mesh-base frame; short triad = TCP (M_tcp_from_block)\n"
        "[inspect] To delete the shown pair: click 'DELETE candidate (step 1: arm)',\n"
        "[inspect]   then 'CONFIRM DELETE (step 2)'. Move the slider to cancel.\n"
        "[inspect] Close the PyBullet window to quit.\n"
    )


# ---- Entry point -------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Render every candidate once headless (a smoke test), then exit.",
    )
    # parse_known_args so a Rhino ScriptEditor launch (which may add its own
    # argv) never crashes on an unexpected argument.
    args, _ = parser.parse_known_args()

    pp.connect(use_gui=not args.no_gui)
    scene = Scene()
    try:
        # A world-origin triad for spatial reference.
        pp.draw_pose(pp.unit_pose(), length=0.1)
        if not args.no_gui:
            # pybullet_planning's connect() hides the whole GUI overlay
            # (COV_ENABLE_GUI False), which also hides the parameter-slider
            # panel. Turn the debug GUI back on so the candidate slider + the
            # two delete buttons are actually visible and draggable.
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.8, cameraYaw=50, cameraPitch=-35,
                cameraTargetPosition=[0.15, 0.0, 0.05],
            )

        candidates = build_candidates()
        if not candidates:
            print("[X] No robotic tools registered in robotic_tools.json - nothing to inspect.")
            return 0

        # Headless smoke test: build each candidate once to exercise the mesh
        # loading + frame math, then quit.
        if args.no_gui:
            for cand in candidates:
                render_candidate(cand, scene)
                clear_scene(scene)
            print(f"[smoke] rendered {len(candidates)} candidate(s) headless OK.")
            return 0

        controls = _make_controls(len(candidates))
        print(_help_text())

        last_idx = -1
        armed_idx = None          # candidate index armed for deletion (or None)
        warn_handles: list = []   # red banner handles shown while armed
        while True:
            try:
                idx = _read_index(controls["slider"], len(candidates))
                if idx != last_idx:
                    render_candidate(candidates[idx], scene)
                    last_idx = idx
                    # Changing the selection cancels any pending delete.
                    if armed_idx is not None and idx != armed_idx:
                        pp.remove_handles(warn_handles)
                        warn_handles = []
                        armed_idx = None
                        print("[delete] disarmed (candidate selection changed).")

                # Step 1: arm -- show exactly what a confirm would remove.
                if p.readUserDebugParameter(controls["arm_btn"]) > controls["arm_base"]:
                    controls["arm_base"] = p.readUserDebugParameter(controls["arm_btn"])
                    names, files = _delete_targets(candidates[idx])
                    if not names:
                        print("[delete] nothing to delete for this candidate.")
                    else:
                        pp.remove_handles(warn_handles)
                        warn_handles = _draw_delete_warning(candidates[idx])
                        armed_idx = idx
                        print("\n[delete] ARMED -- CONFIRM will PERMANENTLY remove:")
                        print(f"    entries : {', '.join(names)}")
                        for file_path in files:
                            print(f"    file    : {file_path}")
                        print("    (a file is kept if another remaining tool still references it.)")
                        print("[delete] click 'CONFIRM DELETE (step 2)', or move the slider to cancel.")

                # Step 2: confirm -- only acts when something is armed.
                if p.readUserDebugParameter(controls["confirm_btn"]) > controls["confirm_base"]:
                    controls["confirm_base"] = p.readUserDebugParameter(controls["confirm_btn"])
                    if armed_idx is None:
                        print("[delete] nothing armed -- click 'DELETE candidate (step 1: arm)' first.")
                    else:
                        changed = do_delete(candidates[armed_idx])
                        pp.remove_handles(warn_handles)
                        warn_handles = []
                        armed_idx = None
                        if changed:
                            # Registry changed on disk -- rebuild the candidate
                            # list but KEEP the same slider + buttons. Recreating
                            # them stacks duplicate widgets in the Params panel
                            # (PyBullet does not reliably remove parameters), so
                            # we deliberately reuse the one set made at startup.
                            candidates = build_candidates()
                            clear_scene(scene)
                            if not candidates:
                                print("[delete] registry is now empty; nothing left to inspect.")
                                break
                            last_idx = -1  # force a re-render on the next poll
            except p.error:
                print("[quit] PyBullet GUI closed.")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[quit] interrupted.")
    finally:
        try:
            pp.disconnect()
        except p.error:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
