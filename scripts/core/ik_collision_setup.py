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
    """Block-instance oids of tool-bearing joints (male + ground) whose
    ``parent_bar_id == bar_id``.  Name kept for back-compat; downstream
    only consumes opaque oids."""
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
    """Return ``({"left": tool_oid, "right": tool_oid}, None)`` or ``(None, error_msg)``.

    Mirrors ``rs_ik_keyframe._resolve_arm_tools_on_bar`` but keyed on
    ``bar_id`` only (no need for the bar curve oid here).
    """
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
    return state, env_geom


def compute_assembly_allowed_touches(env_geom, arm_to_male):
    """Compute the 5 legitimate-contact pairs for the active assembly.

    Returns ``{rb_name: [other_rb_name, ...]}`` suitable for merging into
    ``state.rigid_body_states[rb_name].touch_bodies``.

    Partner names use the SAME naming scheme present in ``env_geom``
    (active_* if any active_* keys are present, else canonical bar_/joint_).
    Tool RB names are stable.
    """
    from core.robot_cell import ARM_TOOL_RB_NAMES
    from core import bar_action

    if not env_geom:
        return {}

    has_active_prefix = any(
        k.startswith(bar_action.ACTIVE_RB_BAR_PREFIX)
        or k.startswith(bar_action.ACTIVE_RB_JOINT_PREFIX)
        for k in env_geom
    )
    if has_active_prefix:
        bar_prefix = bar_action.ACTIVE_RB_BAR_PREFIX
        joint_prefix = bar_action.ACTIVE_RB_JOINT_PREFIX
    else:
        bar_prefix = bar_action.CANONICAL_BAR_PREFIX
        joint_prefix = bar_action.CANONICAL_JOINT_PREFIX

    if has_active_prefix:
        active_bar_keys = [k for k in env_geom if k.startswith(bar_action.ACTIVE_RB_BAR_PREFIX)]
    else:
        active_bar_keys = [
            k for k in env_geom
            if k.startswith(bar_action.CANONICAL_BAR_PREFIX) and not k.startswith("bar_env_")
        ]
    if not active_bar_keys:
        print("compute_assembly_allowed_touches: no active bar key found in env_geom; returning {}.")
        return {}
    if len(active_bar_keys) > 1:
        raise ValueError(
            f"compute_assembly_allowed_touches: multiple active bar keys: {active_bar_keys}"
        )
    active_bar_key = active_bar_keys[0]

    out = {}
    left_rb = ARM_TOOL_RB_NAMES["left"]
    right_rb = ARM_TOOL_RB_NAMES["right"]
    out.setdefault(left_rb, []).append(active_bar_key)
    out.setdefault(right_rb, []).append(active_bar_key)

    for jid, arm in arm_to_male.items():
        male_key = f"{joint_prefix}{jid}_male"
        female_key = f"{joint_prefix}{jid}_female"
        if male_key not in env_geom:
            print(f"compute_assembly_allowed_touches: male key {male_key!r} missing from env_geom; skipping.")
            continue
        tool_rb = ARM_TOOL_RB_NAMES.get(arm)
        if tool_rb is None:
            print(f"compute_assembly_allowed_touches: unknown arm {arm!r} for joint {jid}; skipping.")
            continue
        out.setdefault(tool_rb, []).append(male_key)
        # Rigid-bond pair: each joint half is fixed to the active bar's
        # centerline by design (the half-joint blocks live on the bar's
        # local OCF). They will always overlap the bar's tube geometry, so
        # whitelist bar<->male and bar<->female across all movements.
        out.setdefault(active_bar_key, []).append(male_key)
        if female_key in env_geom:
            # Mate pair (male <-> female) only legitimate at the assembled
            # pose. M1 (home -> approach) must strip it back out --
            # see `core.bar_action.MATE_PAIR_TOUCH_BODY_KEYS`.
            out.setdefault(male_key, []).append(female_key)
            out.setdefault(active_bar_key, []).append(female_key)
        else:
            print(f"compute_assembly_allowed_touches: female sibling {female_key!r} missing; skipping mate-pair.")

    for k, lst in out.items():
        seen = set()
        out[k] = [x for x in lst if not (x in seen or seen.add(x))]
    return out


def apply_allowed_touches(state, allowed_dict):
    """Merge ``allowed_dict`` into ``state.rigid_body_states[name].touch_bodies``.

    Unions existing entries with new partners. Prints a warning when
    ``rb_name`` is missing from ``state``. Mutates ``state`` in place.
    Returns the number of (rb_name, partner) pairs newly added.
    """
    added = 0
    rb_states = getattr(state, "rigid_body_states", None) or {}
    for rb_name, partners in (allowed_dict or {}).items():
        rb = rb_states.get(rb_name)
        if rb is None:
            print(f"apply_allowed_touches: rigid_body_state {rb_name!r} not in state; skipping.")
            continue
        existing = set(rb.touch_bodies or [])
        for p in partners:
            if p not in existing:
                existing.add(p)
                added += 1
        rb.touch_bodies = sorted(existing)
    return added


def build_assembly_ik_state(rcell, planner, template_state, bar_id):
    """Shared IK/MP collision-state builder for assembling ``bar_id``.

    Pipeline: prepare_assembly_collision_state -> _classify_male_joints_per_arm
    -> compute_assembly_allowed_touches -> apply_allowed_touches.

    State uses active_*/env_* naming (NOT canonicalized); downstream
    ``canonicalize_state`` remaps touch_bodies entries correctly.

    Returns ``(state, env_geom, arm_to_male, allowed)``.
    """
    from core import bar_action

    state, env_geom = prepare_assembly_collision_state(rcell, planner, template_state, bar_id)
    arm_to_male = bar_action._classify_male_joints_per_arm(bar_id)
    allowed = compute_assembly_allowed_touches(env_geom, arm_to_male)
    apply_allowed_touches(state, allowed)
    return state, env_geom, arm_to_male, allowed
