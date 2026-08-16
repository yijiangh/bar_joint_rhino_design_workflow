"""Logic behind the ``RSGHSequencePreview`` Grasshopper component.

What it is
----------
A *viewer* that turns the document's assembly sequence into a scrubbable frame
index, so the Rhino viewport can be driven from a Grasshopper slider (and
therefore animated) instead of only from the ``RSSequenceEdit`` command line.

One frame = one ``(bar, pose)`` pair.  ``poses`` is the list of robot movements
to visit at each bar -- ``M1`` home, ``M2`` approach, ``M3`` assembled, ``M4``
retreat -- so ``poses=["M3"]`` gives one frame per bar (the moment it lands) and
``poses=["M1","M2","M3","M4"]`` shows the whole reach-and-retreat per bar.

Frames are ordered **bar-major**: the bar is the slow-moving digit and the pose
the fast one, like a two-digit odometer::

    idx        0        1        2        3       ...
    bar     bar[0]   bar[0]   bar[1]   bar[1]
    pose    poses[0] poses[1] poses[0] poses[1]

    n_frames  = len(bars) * len(poses)
    bar_index = idx // len(poses)          # complete bars passed
    pose      = poses[idx % len(poses)]    # how far into the current bar

**Every registered bar is always in the frame list.**  The persistent
``HideUnbuilt`` latch (``scaffolding.build_stage``) is deliberately ignored: it
is never read to filter the sequence and never written.  Hiding not-yet-built
bars for the camera is the separate ``show_unbuilt`` input, which is a
per-frame view decision, not the document-wide latch.

What it deliberately does NOT do
--------------------------------
* **Never renumbers a sequence.**  ``repair_on_entry`` is never called -- it
  mutates the document, and a viewer that quietly rewrote bar numbers while the
  user dragged a slider would be a disaster.
* **Never writes the build-stage latch.**  Writing it from a preview would leave
  the model permanently filtered after the component was deleted.  See the
  warning on ``rs_show_bar_action_plan._PreviewSession.toggle_unbuilt``.
* **Never starts PyBullet.**  The robot is drawn by forward kinematics only
  (``ik_viz.update_state`` with no ``set_cell_state``), copying the render path
  of ``_render_motion_waypoint``.  ``RSPBStart`` is not required.

Everything it changes is restored by :func:`_teardown`, which runs on
``enable=False``.

Pose sources -- all four readable from the ``.3dm`` alone, no planner:

===== ==========================================================
Pose  Source
===== ==========================================================
M1    the bar's saved base frame + ``config.HOME_CONF_*_6``
M2    ``config.KEY_ASSEMBLY_IK_APPROACH``
M3    ``config.KEY_ASSEMBLY_IK_ASSEMBLED``
M4    ``config.KEY_ASSEMBLY_IK_RETREAT``
===== ==========================================================
"""

from __future__ import annotations

import json

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core import config
from core import gh_bridge
from core import ik_viz
from core import robot_cell
from core.rhino_bar_registry import (
    get_bar_seq_map,
    reset_sequence_colors,
    show_sequence_colors,
)


# The pose column of the frame table, in canonical order.  A user-supplied
# `poses` list is filtered against this but keeps the user's own ordering, so
# feeding ("M4", "M1") really does play retreat-then-home.
POSES = ("M1", "M2", "M3", "M4")
DEFAULT_POSES = ("M3",)

# The document layers `show_bars_and_joints` toggles.  Centerlines are included
# (unlike ik_viz's default hide list, which keeps them pickable) because this
# component is for filming, where a stray centerline is just a wire in shot.
GEOM_LAYERS = (
    config.LAYER_BAR_CENTERLINES,
    config.LAYER_BAR_TUBE_PREVIEWS,
    config.LAYER_JOINT_FEMALE_INSTANCES,
    config.LAYER_JOINT_MALE_INSTANCES,
    config.LAYER_JOINT_GROUND_INSTANCES,
)

# Which saved user-text record each pose reads.  M1 is absent: it is synthesised
# from the config home configuration, not stored per bar.
_POSE_PAYLOAD_KEY = {"M2": "approach", "M3": "assembled", "M4": "retreat"}


# ---------------------------------------------------------------------------
# Payload loading -- lifted from rs_show_bar_action_plan so the GH component and
# the command read the same records the same way.
# ---------------------------------------------------------------------------


def load_assembly_payload(bar_oid):
    """Read the split assembly keys off ``bar_oid``.

    Returns ``{"base_frame_world_mm", "approach", "assembled", "retreat"}`` (any
    of "approach" / "retreat" may be ``None`` if missing -- e.g. bars keyframed
    before the three-movement IK) or ``None`` if the bar has no assembled
    keyframe at all.
    """
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not assembled_raw:
        return None
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    retreat_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT)
    try:
        return {
            "base_frame_world_mm": np.asarray(json.loads(base_raw), dtype=float),
            "approach": json.loads(approach_raw) if approach_raw else None,
            "assembled": json.loads(assembled_raw),
            "retreat": json.loads(retreat_raw) if retreat_raw else None,
        }
    except json.JSONDecodeError as exc:
        print(f"RSGHSequencePreview: malformed user-text on bar ({exc}); skipping IK preview.")
        return None


def _apply_groups(state, groups):
    """Merge ``{left,right}`` group configs into ``state.robot_configuration``."""
    for _side, cfg in groups.items():
        for name, value in zip(cfg["joint_names"], cfg["joint_values"]):
            state.robot_configuration[name] = float(value)


def build_assembly_state(base_state, base_mm, groups, deps):
    """Place *base_state* at the bar's saved base frame and stamp on *groups*.

    ``base_mm`` is the 4x4 world transform in **millimetres** (the repo's
    convention for robot/IK data); compas wants metres, hence the /1000.
    """
    state = base_state
    origin_m = base_mm[:3, 3] / 1000.0
    x_axis = base_mm[:3, 0]
    y_axis = base_mm[:3, 1]
    state.robot_base_frame = deps["Frame"](
        list(map(float, origin_m)),
        list(map(float, x_axis)),
        list(map(float, y_axis)),
    )
    _apply_groups(state, groups)
    return state


# ---------------------------------------------------------------------------
# Inputs
# ---------------------------------------------------------------------------


def normalize_poses(poses):
    """Filter *poses* to the known set, keeping the caller's order.

    Accepts a list (GH List Access), a single string (a user who forgot to set
    List Access), or ``None``.  Anything unrecognised is dropped; an empty
    result falls back to :data:`DEFAULT_POSES` so the component always has at
    least one frame per bar.

    Returns:
        (tuple[str], list[str]): the accepted poses and any warnings.
    """
    notes = []
    if poses is None:
        items = []
    elif isinstance(poses, str):
        items = [poses]
    else:
        items = list(poses)

    accepted = []
    for raw in items:
        if raw is None:
            continue
        # The Script Editor rewrites a `List[str]` annotation to the .NET
        # `System.Collections.Generic.List[object]`, so an item can arrive as a
        # GH_String wrapper rather than a Python str.  `.Value` unwraps those;
        # `str()` covers everything else.
        raw = getattr(raw, "Value", raw)
        name = str(raw).strip().upper()
        if name not in POSES:
            notes.append(f"ignored unknown pose {raw!r}")
            continue
        if name in accepted:
            continue  # a repeat would just film the same frame twice
        accepted.append(name)

    if not accepted:
        accepted = list(DEFAULT_POSES)
        if items:
            notes.append(f"no usable pose in {items!r}; falling back to {accepted}")
    return tuple(accepted), notes


def frame_at(ordered_bars, poses, idx):
    """Resolve a flat frame index to ``(bar_id, pose)`` -- the odometer math."""
    n_poses = len(poses)
    idx = max(0, min(int(idx), len(ordered_bars) * n_poses - 1))
    return ordered_bars[idx // n_poses], poses[idx % n_poses]


def _flag(value, default):
    """Coerce a GH boolean input to a real bool, treating ``None`` as *default*.

    An **unconnected** Grasshopper input arrives as ``None``, not as the Python
    default in the signature -- the host passes the argument explicitly, so the
    default never applies.  Left alone, every unwired switch would read as False
    and the component would come up with the whole legend silently off.
    """
    return default if value is None else bool(value)


def _index(value, default=0):
    """Coerce a GH integer input, treating ``None`` (unconnected) as *default*."""
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# Document layers
# ---------------------------------------------------------------------------


def _set_geom_layers(store, visible):
    """Show/hide the bar + joint layers, recording their prior visibility once.

    The recording is deliberately taken only on the *first* change: if the
    component recorded on every solve it would eventually record its own hidden
    state and ``_teardown`` would restore nothing.
    """
    prev = store.get("layer_prev")
    if prev is None:
        prev = {}
        for name in GEOM_LAYERS:
            if rs.IsLayer(name):
                prev[name] = bool(rs.LayerVisible(name))
        store["layer_prev"] = prev
    for name in GEOM_LAYERS:
        if rs.IsLayer(name):
            rs.LayerVisible(name, bool(visible))


def _restore_geom_layers(store):
    prev = store.pop("layer_prev", None) or {}
    for name, was_visible in prev.items():
        if rs.IsLayer(name):
            rs.LayerVisible(name, was_visible)


# ---------------------------------------------------------------------------
# Robot rendering
# ---------------------------------------------------------------------------


def _base_cell_state(rcell):
    """Return ``(state, how)`` -- the tool-bearing base state when one is available.

    ``base_assembly_cell_state`` needs ``rcell.tool_models`` to be populated,
    which happens when ``RSIKKeyframe`` / ``RSRebuildRobotCell`` has run in this
    Rhino session.  With a bare cell the arms render flange-only -- which is not
    a hole in the picture, because the document's own tool block instances are
    still on screen (``show_sequence_colors`` shows the active step's tools).

    The shared cached cell is never mutated here: registering tools is
    ``rebuild_assembly_cell``'s job, and doing it from a viewer would change what
    every other command sees.
    """
    if getattr(rcell, "tool_models", None):
        return robot_cell.base_assembly_cell_state(), "tools attached"
    return robot_cell.default_cell_state(), "flange-only (no tool models registered)"


def _home_groups(rcell):
    """Synthesise the M1 home configuration groups from ``config.HOME_CONF_*_6``."""
    return {
        "left": {
            "joint_names": list(rcell.get_configurable_joint_names(config.LEFT_GROUP)),
            "joint_values": list(config.HOME_CONF_LEFT_6),
        },
        "right": {
            "joint_names": list(rcell.get_configurable_joint_names(config.RIGHT_GROUP)),
            "joint_values": list(config.HOME_CONF_RIGHT_6),
        },
    }


def _render_assemble_robot(store, bar_oid, bar_id, pose, notes):
    """Draw the dual-arm robot at *bar_id*'s saved base frame in *pose*.

    FK only -- ``ik_viz.update_state`` without ``set_cell_state`` -- so no
    PyBullet connection is needed.  Returns True when something was drawn.
    """
    payload = load_assembly_payload(bar_oid)
    if payload is None:
        notes.append(
            f"bar {bar_id} has no '{config.KEY_ASSEMBLY_IK_ASSEMBLED}' record; robot hidden"
        )
        ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
        return False

    rcell = robot_cell.get_or_load_robot_cell()
    deps = robot_cell.import_compas_stack()

    if pose == "M1":
        groups = _home_groups(rcell)
    else:
        groups = payload.get(_POSE_PAYLOAD_KEY[pose])
        if groups is None:
            notes.append(f"bar {bar_id} has no saved {pose} keyframe; robot hidden")
            ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
            return False

    base_state, how = _base_cell_state(rcell)
    if store.get("tool_note") != how:
        store["tool_note"] = how
        notes.append(f"arms: {how}")

    state = build_assembly_state(base_state, payload["base_frame_world_mm"], groups, deps)

    rs.EnableRedraw(False)
    try:
        if not store.get("session"):
            # hide_doc_layers=() is essential: the default hides tubes / joints /
            # tool blocks for the session, which would override this component's
            # own `show_bars_and_joints` switch and restore them at a moment of
            # ik_viz's choosing rather than ours.
            ik_viz.begin_session(
                robot_cell=rcell,
                mesh_modes=(ik_viz.MESH_MODE_VISUAL,),
                active_mesh_mode=ik_viz.MESH_MODE_VISUAL,
                layer_key=ik_viz.LAYER_KEY_ASSEMBLY,
                hide_doc_layers=(),
            )
            store["session"] = True
        ik_viz.update_state(
            state, robot_cell=rcell, layer_key=ik_viz.LAYER_KEY_ASSEMBLY
        )
        ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, True)
    finally:
        rs.EnableRedraw(True)
    return True


def _render_support_robot(store, bar_oid, bar_id, notes):
    """Support-arm preview -- **not implemented yet**, always hides the layer.

    TODO: read the bar's support keyframe and draw it.  Data source: today's
    single ``config.IK_SUPPORT_KEY`` blob, moving to the split
    ``config.KEY_SUPPORT_BASE_FRAME`` / ``KEY_SUPPORT_IK_APPROACH`` /
    ``KEY_SUPPORT_IK_HELD`` keys.  Reference implementation is
    ``rs_show_bar_action_plan._show_support_state`` **minus**
    ``robot_cell_support.set_cell_state`` (that is the planner call this
    component must stay clear of).

    The shape mirrors :func:`_render_assemble_robot` exactly -- same
    ``begin_session`` / ``update_state`` calls, only
    ``layer_key=ik_viz.LAYER_KEY_SUPPORT`` and
    ``robot_cell_support.get_or_load_support_cell()`` /
    ``default_support_cell_state()`` -- so wiring it later is filling in this one
    function, with no structural change to :func:`run`.

    It is inert for now because the support pipeline is not ready:
    ``rs_ik_support_keyframe.py`` is archived and off the toolbar, the
    ``KEY_SUPPORT_*`` split is unwired, and essentially no bar in the current
    file carries an ``ik_support`` record.
    """
    ik_viz.set_layer_visible(ik_viz.LAYER_KEY_SUPPORT, False)
    notes.append("show_support_robot: pending (support pipeline not wired yet)")
    return False


# ---------------------------------------------------------------------------
# Teardown
# ---------------------------------------------------------------------------


def _teardown(store):
    """Put the document back exactly as the component found it."""
    reset_sequence_colors()
    if store.get("session"):
        ik_viz.end_session()
    ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
    ik_viz.set_layer_visible(ik_viz.LAYER_KEY_SUPPORT, False)
    _restore_geom_layers(store)
    store.pop("session", None)
    store.pop("fingerprint", None)
    store.pop("tool_note", None)
    sc.doc.Views.Redraw()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(
    ghenv,
    enable=False,
    reload=False,
    step=0,
    poses=None,
    show_unbuilt=True,
    show_bars_and_joints=True,
    show_assemble_robot=True,
    show_support_robot=False,
    preview_color_unbuilt=True,
    preview_color_built=True,
    preview_color_current=True,
    preview_color_support=True,
):
    """Drive the Rhino viewport to one frame of the assembly sequence.

    Args:
        ghenv: the component's ``ghenv`` (or ``self`` in SDK mode) -- used for
            its per-component state slot and to find the ``step`` parameter.
        enable (bool): False tears the preview down and returns immediately.
        reload (bool): button; on its rising edge, (re)build the ``step`` slider.
        step (int): frame index, bar-major (see the module docstring).
        poses (list[str]): which movements to visit per bar; default ``["M3"]``.
        show_unbuilt (bool): show bars later in the sequence than the active one.
            Separate from ``preview_color_unbuilt``, which only drops their tint.
        show_bars_and_joints (bool): document bar/joint layer visibility.
        show_assemble_robot (bool): draw the dual-arm robot at this frame.
        show_support_robot (bool): accepted but inert -- see
            :func:`_render_support_robot`.
        preview_color_* (bool): per-class tint switches; False leaves that class
            ByLayer without changing its visibility.

    Returns:
        dict: ``{"bar_id", "seq", "pose", "frame_count", "info"}``.
    """
    # Unconnected GH inputs arrive as None -- the host passes every argument
    # explicitly, so the signature defaults above never fire.  Normalise here,
    # once, so the rest of the function can trust these are real values.
    enable = _flag(enable, False)
    reload = _flag(reload, False)
    step = _index(step, 0)
    show_unbuilt = _flag(show_unbuilt, True)
    show_bars_and_joints = _flag(show_bars_and_joints, True)
    show_assemble_robot = _flag(show_assemble_robot, True)
    show_support_robot = _flag(show_support_robot, False)
    preview_color_unbuilt = _flag(preview_color_unbuilt, True)
    preview_color_built = _flag(preview_color_built, True)
    preview_color_current = _flag(preview_color_current, True)
    preview_color_support = _flag(preview_color_support, True)

    store = gh_bridge.state(ghenv)
    notes = []

    with gh_bridge.rhino_doc():
        if not enable:
            _teardown(store)
            return {
                "bar_id": None,
                "seq": None,
                "pose": None,
                "frame_count": 0,
                "info": "disabled; document restored",
            }

        # bar_map is built once and handed down to show_sequence_colors, so the
        # whole solve makes exactly one rs.AllObjects() scan.  repair_on_entry is
        # NOT called: it renumbers, and a viewer must not.  Every bar it returns
        # becomes a frame -- the build-stage latch never filters this list.
        bar_map = get_bar_seq_map()
        if not bar_map:
            return {
                "bar_id": None,
                "seq": None,
                "pose": None,
                "frame_count": 0,
                "info": "no registered bars in this document",
            }
        ordered = sorted(bar_map, key=lambda b: bar_map[b][1])

        pose_list, pose_notes = normalize_poses(poses)
        notes.extend(pose_notes)
        n_frames = len(ordered) * len(pose_list)

        # The slider is (re)built on the button's rising edge, and its bounds are
        # refreshed silently whenever the frame count changes under it (a bar
        # added / removed, or a different pose set).
        if gh_bridge.rising_edge(store, "reload", reload):
            store.pop("fingerprint", None)  # force a redraw after a reload
            notes.append(gh_bridge.ensure_int_slider(ghenv, "step", 0, n_frames - 1))
        elif store.get("n_frames") != n_frames:
            notes.append(gh_bridge.ensure_int_slider(ghenv, "step", 0, n_frames - 1))
        store["n_frames"] = n_frames

        bar_id, pose = frame_at(ordered, pose_list, step)
        bar_oid, seq = bar_map[bar_id]

        # GH re-solves aggressively (any upstream tick, a canvas move, a
        # neighbouring component expiring).  Re-posing the robot each time is the
        # expensive part, so skip the whole render when nothing that affects it
        # changed.  `reload` clears this, which is the manual escape hatch when
        # the document itself changed underneath us.
        fingerprint = (
            bar_id, pose, n_frames, bool(show_unbuilt), bool(show_bars_and_joints),
            bool(show_assemble_robot), bool(show_support_robot),
            bool(preview_color_unbuilt), bool(preview_color_built),
            bool(preview_color_current), bool(preview_color_support),
        )
        if store.get("fingerprint") == fingerprint:
            return {
                "bar_id": bar_id,
                "seq": seq,
                "pose": pose,
                "frame_count": n_frames,
                "info": "unchanged (render skipped)",
            }

        # Colours + visibility for bars / joints / tools.  set_build_stage is
        # never called: that latch is persistent and this is a viewer.
        show_sequence_colors(
            bar_id,
            show_unbuilt=bool(show_unbuilt),
            bar_map=bar_map,
            highlight_supports=bool(preview_color_support),
            color_flags={
                "built": bool(preview_color_built),
                "active": bool(preview_color_current),
                "unbuilt": bool(preview_color_unbuilt),
                "support": bool(preview_color_support),
            },
        )

        _set_geom_layers(store, show_bars_and_joints)

        if show_assemble_robot:
            _render_assemble_robot(store, bar_oid, bar_id, pose, notes)
        else:
            ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)

        if show_support_robot:
            _render_support_robot(store, bar_oid, bar_id, notes)
        else:
            ik_viz.set_layer_visible(ik_viz.LAYER_KEY_SUPPORT, False)

        store["fingerprint"] = fingerprint
        sc.doc.Views.Redraw()

    info = f"frame {int(step)}/{n_frames - 1} -> {bar_id} (seq {seq}) {pose}"
    if notes:
        info += " | " + " | ".join(notes)
    return {
        "bar_id": bar_id,
        "seq": seq,
        "pose": pose,
        "frame_count": n_frames,
        "info": info,
    }
