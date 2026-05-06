"""Shared assembly-side collision-state setup.

Both ``rs_ik_keyframe.py`` (where the IK is solved) and ``rs_show_ik.py``
(where the saved keyframe is replayed + collision-checked) need IDENTICAL
collision context for the dual-arm cell:

* per-arm tool collision rigid bodies (``AssemblyLeftArmToolBody`` /
  ``AssemblyRightArmToolBody``) attached to the matching ``*_ur_arm_tool0``
  link with wrist touch-links whitelisted; and
* ``env_*`` rigid bodies for every built bar / joint earlier in the
  sequence than the active bar.

Without the per-arm tool RBs the ShowIK CheckCollision misses the
tool<->env / tool<->link / tool<->tool pair categories (CC.2 / CC.3 / CC.5
involving the tool RBs) that the IK solver was actually screening on,
producing the puzzling "IK fails with collision but ShowIK reports
no collision" mismatch.
"""

from __future__ import annotations

from typing import Optional

import rhinoscriptsyntax as rs

from core import config
from core import env_collision
from core import robot_cell
from core.rhino_bar_registry import get_bar_seq_map
from core.rhino_tool_place import find_tool_for_joint
from core.robotic_tool import get_robotic_tool


def _arm_side_from_tool_name(tool_name: str) -> Optional[str]:
    """``tool_name`` ending in 'L' -> 'left', 'R' -> 'right', else ``None``."""
    if not tool_name:
        return None
    last = tool_name.strip()[-1].upper()
    if last == "L":
        return "left"
    if last == "R":
        return "right"
    return None


def _males_on_bar(bar_id: str) -> list:
    """Block-instance oids of male joints whose ``parent_bar_id == bar_id``."""
    if not rs.IsLayer(config.LAYER_JOINT_MALE_INSTANCES):
        return []
    return [
        oid
        for oid in rs.ObjectsByLayer(config.LAYER_JOINT_MALE_INSTANCES) or []
        if rs.GetUserText(oid, "parent_bar_id") == bar_id
    ]


def resolve_arm_tools_on_bar(bar_id: str):
    """Return ``({"left": tool_oid, "right": tool_oid}, None)`` or ``(None, error_msg)``.

    Mirrors ``rs_ik_keyframe._resolve_arm_tools_on_bar`` but keyed on
    ``bar_id`` only (no need for the bar curve oid here).
    """
    males = _males_on_bar(bar_id)
    if len(males) != 2:
        return None, (
            f"Bar '{bar_id}' has {len(males)} male joint(s); need exactly 2."
        )
    sides = {"left": None, "right": None}
    for moid in males:
        jid = rs.GetUserText(moid, "joint_id")
        if not jid:
            return None, f"Male block on bar '{bar_id}' missing 'joint_id'."
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
    """Return ``(left_path, right_path)`` -- empty string when missing."""
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


def prepare_assembly_collision_state(rcell, planner, template_state, bar_id):
    """Mirror of ``rs_ik_keyframe._prepare_collision_template_state``.

    Attaches per-arm tool rigid bodies + env bars/joints to ``template_state``
    for the bar identified by ``bar_id``. Returns ``(state, env_geom)``.

    On failure (missing tools etc.) prints a diagnostic and returns the
    original ``template_state`` unchanged + empty ``env_geom`` -- the caller
    can still run a partial collision check that omits tool obstacles.
    """
    arm_tools, err = resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        print(f"core.ik_collision_setup: {err}; skipping arm-tool RB attach.")
        arm_tool_rb_names = {"left": None, "right": None}
    else:
        left_path, right_path = resolve_tool_collision_paths(
            arm_tools["left"], arm_tools["right"]
        )
        arm_tool_rb_names = robot_cell.attach_arm_tool_rigid_bodies(
            rcell,
            planner,
            left_collision_path=left_path,
            right_collision_path=right_path,
            native_scale=0.001,
        )
        robot_cell.configure_arm_tool_rigid_body_states(
            template_state, arm_tool_rb_names
        )

    env_geom = env_collision.collect_built_geometry(bar_id, get_bar_seq_map())
    # Add the active bar + its joints (visible + collision) under separate
    # active_* prefixes so callers can hide/attach them independently later.
    active_geom = env_collision.collect_active_geometry(bar_id, get_bar_seq_map())
    env_geom.update(active_geom)
    robot_cell.ensure_env_registered(rcell, env_geom, planner)
    state = env_collision.build_env_state(template_state, env_geom)
    # `build_env_state` returns a fresh copy; re-apply tool-RB attachments.
    robot_cell.configure_arm_tool_rigid_body_states(state, arm_tool_rb_names)
    # Whitelist intentional design contacts: active joint <-> mating env joint,
    # active bodies <-> arm tool RBs (see env_collision.configure_active_assembly_acm).
    env_collision.configure_active_assembly_acm(state, arm_tool_rb_names)
    return state, env_geom
