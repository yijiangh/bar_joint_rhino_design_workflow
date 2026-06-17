"""Shared assembly-side collision-state setup (static-cell model).

This module builds the per-step ``RobotCellState`` on top of the cached static
cell (bars + joints + environment obstacles + the two arm ``ToolModel``s, built
once by ``robot_cell.rebuild_assembly_cell``). The per-movement collision
context (attachments + allowed-touch policy) lives in ``core.bar_action``;
``rs_ik_keyframe`` and ``rs_show_ik`` both go through
``bar_action.build_assembly_movements`` so the solve and the replay-check share
one source of truth.

State-independent canonical names are used throughout (``bar_<bid>`` /
``joint_<jid>_<sub>`` / ``obstacle_<name>``); the bar currently being
assembled is identified by ``active_bar_id`` + the assembly sequence, not by a
name prefix.

The pure (Rhino-free) helper ``build_full_assembly_state`` is unit-tested
headless (see ``tests/test_bar_action_collisions.py``); the Rhino-only helpers
import ``rhinoscriptsyntax`` lazily so this module stays importable outside Rhino.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from core import config
from core import robot_cell


def _arm_side_from_tool_name(tool_name: str) -> Optional[str]:
    """Classify an arm side from a tool name's L/R suffix.

    Args:
        tool_name (str): e.g. ``"AT3L"`` / ``"AT3R"``.

    Returns:
        str | None: ``"left"`` if the name ends in 'L', ``"right"`` if 'R',
        else ``None``.
    """
    if not tool_name:
        return None
    last = tool_name.strip()[-1].upper()
    if last == "L":
        return "left"
    if last == "R":
        return "right"
    return None


# ---------------------------------------------------------------------------
# Rhino-side tool resolution (lazy rhinoscriptsyntax import)
# ---------------------------------------------------------------------------


def _males_on_bar(bar_id: str) -> list:
    """Find the tool-bearing joint blocks mounted on a bar.

    Args:
        bar_id (str): the bar to scan for.

    Returns:
        list: Rhino object ids of male + ground joint block instances whose
        ``parent_bar_id == bar_id``.
    """
    import rhinoscriptsyntax as rs

    out = []
    for layer in (
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        out.extend(
            oid
            for oid in rs.ObjectsByLayer(layer) or []
            if rs.GetUserText(oid, "parent_bar_id") == bar_id
        )
    return out


def resolve_arm_tools_on_bar(bar_id: str):
    """Resolve which placed tool block sits on each arm for a bar.

    Args:
        bar_id (str): the bar whose two tool-bearing joints to resolve.

    Returns:
        tuple: ``({"left": tool_oid, "right": tool_oid}, None)`` on success, or
        ``(None, error_msg)`` if the bar lacks exactly one left + one right tool.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_tool_place import find_tool_for_joint

    males = _males_on_bar(bar_id)
    if len(males) != 2:
        return None, (
            f"Bar '{bar_id}' has {len(males)} tool-bearing joint(s) (male+ground); need exactly 2."
        )
    sides = {"left": None, "right": None}
    for moid in males:
        jid = rs.GetUserText(moid, "joint_id")
        if not jid:
            return None, f"Joint block on bar '{bar_id}' missing 'joint_id'."
        toid = find_tool_for_joint(jid)
        if toid is None:
            return None, f"Joint '{jid}' on bar '{bar_id}' has no robotic tool placed."
        side = _arm_side_from_tool_name(rs.GetUserText(toid, "tool_name") or "")
        if side is None:
            return None, f"Tool on joint '{jid}' has no L/R suffix; cannot decide arm side."
        if sides[side] is not None:
            return None, f"Bar '{bar_id}' has two {side.upper()}-suffix tools."
        sides[side] = toid
    if sides["left"] is None or sides["right"] is None:
        missing = "left" if sides["left"] is None else "right"
        return None, f"Bar '{bar_id}' missing the {missing}-arm tool."
    return sides, None


def resolve_tool_collision_paths(left_tool_oid, right_tool_oid):
    """Look up each tool's collision-OBJ path from the registry.

    Args:
        left_tool_oid: Rhino object id of the left-arm tool block.
        right_tool_oid: Rhino object id of the right-arm tool block.

    Returns:
        tuple: ``(left_path, right_path)`` -- absolute OBJ paths, empty string
        when the tool has no collision mesh / is not in the registry.
    """
    import rhinoscriptsyntax as rs
    from core.robotic_tool import get_robotic_tool

    out = {"left": "", "right": ""}
    for side, oid in (("left", left_tool_oid), ("right", right_tool_oid)):
        tname = rs.GetUserText(oid, "tool_name") or ""
        if not tname:
            continue
        try:
            tooldef = get_robotic_tool(tname)
        except KeyError:
            continue
        path = tooldef.collision_path()
        if path:
            out[side] = path
    return out["left"], out["right"]


# ---------------------------------------------------------------------------
# Per-step state builder (Rhino-free pure functions)
# ---------------------------------------------------------------------------


def _frame_from_mm4(matrix_mm):
    """Convert a 4x4 mm transform to a compas ``Frame`` in meters.

    Args:
        matrix_mm (array-like): 4x4 homogeneous transform with mm translation.

    Returns:
        compas.geometry.Frame: the same pose with the origin scaled to meters.
    """
    from compas.geometry import Frame

    m = np.asarray(matrix_mm, dtype=float)
    origin = m[:3, 3] / 1000.0
    return Frame(
        list(map(float, origin)),
        list(map(float, m[:3, 0])),
        list(map(float, m[:3, 1])),
    )


def build_full_assembly_state(base_state, collision_bodies, bar_seq_map, active_bar_id):
    """Populate a full-key-set ``RobotCellState`` for assembling ``active_bar_id``.

    Pure function (no Rhino). ``base_state`` carries ``robot_base_frame``,
    ``robot_configuration`` and the attached arm ``tool_states`` (from
    ``robot_cell.base_assembly_cell_state``). Every body in ``collision_bodies``
    becomes a static, unattached obstacle at its world frame, EXCEPT
    not-yet-built bars/joints (parent-bar sequence > active) which are
    ``is_hidden=True``. Environment obstacles (``kind:"environment"``, no
    parent bar) are always static + visible.

    The active bar is left as a static obstacle here (the live-IK form); the
    BarAction export re-classes it to gripper-attached per movement.

    Args:
        base_state (RobotCellState): seed state carrying ``robot_base_frame``,
            ``robot_configuration`` and the attached arm ``tool_states``
            (from ``robot_cell.base_assembly_cell_state``).
        collision_bodies (dict): ``{name: body_info}`` from the static cell;
            each ``body_info`` has ``frame_world_mm`` and ``kind``, and (for
            bars/joints) ``parent_bar_id``.
        bar_seq_map (dict): ``{bar_id: (oid, seq)}`` assembly-sequence map.
        active_bar_id (str): id of the bar being assembled.

    Returns:
        RobotCellState: a copy of ``base_state`` with one ``RigidBodyState``
        per body (static at its world frame; not-yet-built bars/joints
        ``is_hidden=True``). Key-set equals ``collision_bodies``.
    """
    from compas_fab.robots import RigidBodyState

    state = base_state.copy()
    active_seq = None
    if active_bar_id in bar_seq_map:
        active_seq = bar_seq_map[active_bar_id][1]

    for name, body_info in collision_bodies.items():
        frame = _frame_from_mm4(body_info["frame_world_mm"])
        if body_info.get("kind") == "environment":
            is_hidden = False
        else:
            parent = body_info.get("parent_bar_id")
            seq = bar_seq_map[parent][1] if parent in bar_seq_map else None
            is_hidden = active_seq is not None and seq is not None and seq > active_seq
        state.rigid_body_states[name] = RigidBodyState(
            frame=frame,
            attached_to_link=None,
            attached_to_tool=None,
            touch_links=[],
            touch_bodies=[],
            attachment_frame=None,
            is_hidden=is_hidden,
        )

    # Defensive: state's rigid-body key-set must equal the cell's (collision
    # bodies). A mismatch -> assert_cell_state_match would later reject the state.
    extra = set(state.rigid_body_states) - set(collision_bodies)
    if extra:
        raise RuntimeError(
            f"build_full_assembly_state: state has rigid bodies not in the cell: {sorted(extra)}"
        )
    return state


# ---------------------------------------------------------------------------
# Collision-state assembly (Rhino entry; pure helpers above)
# ---------------------------------------------------------------------------


def prepare_assembly_collision_state(rcell, planner, template_state, bar_id):
    """Build the per-step assembly state on top of the cached static cell.

    Args:
        rcell (RobotCell): the cached robot cell.
        planner (PyBulletPlanner): active planner (used by ``ensure_assembly_cell``).
        template_state (RobotCellState): supplies ``robot_base_frame`` and
            ``robot_configuration`` only (arm tools come attached from
            ``base_assembly_cell_state``).
        bar_id (str): id of the bar being assembled.

    Returns:
        tuple: ``(state, collision_bodies)`` -- the full per-step
        ``RobotCellState`` and the cached ``{name: body_info}`` dict.
    """
    from core.rhino_bar_registry import get_bar_seq_map

    collision_bodies = robot_cell.ensure_assembly_cell(rcell, planner)
    base = robot_cell.base_assembly_cell_state()
    base.robot_base_frame = template_state.robot_base_frame
    base.robot_configuration = template_state.robot_configuration
    state = build_full_assembly_state(base, collision_bodies, get_bar_seq_map(), bar_id)
    return state, collision_bodies
