"""BarAssemblyAction schema + builder.

A `BarAssemblyAction` is a downstream artifact describing one bar's full
dual-arm assembly cycle as four `Movement` records:

    M1  RoboticDualArmConstrainedMovement   home -> approach (gripped bar)
    M2  RoboticLinearMovement               linear mate (gripped bar -> mated)
    M3  RoboticLinearMovement               linear retreat (released, per-arm)
    M4  RoboticFreeMovement                 free home (no grasp)

The `build_bar_assembly_action` factory reads the IK keyframe data already
written on the bar curve user-text by `rs_ik_keyframe.py`
(`KEY_ASSEMBLY_BASE_FRAME`, `KEY_ASSEMBLY_IK_APPROACH`,
`KEY_ASSEMBLY_IK_ASSEMBLED`) and reuses the existing collision context
helpers from `core.ik_collision_setup` / `core.env_collision` / `core.robot_cell`
without modifying them.

Each movement carries a *full* `RobotCellState` start_state (rather than
the more frugal state-diff used by `gramaziokohler/integral_timber_joints`)
because the per-bar artifact stays small and the consumer should not need
to walk diffs to reconstruct intermediate states.

State-independent rigid-body naming
-----------------------------------
The cell is now a static registry with canonical, state-independent names
throughout (`bar_<bid>`, `joint_<jid>_<sub>`, `obstacle_<name>`; tools under
`tool_models` as `AT3L`/`AT3R`), built by `robot_cell.rebuild_assembly_cell`.
There is no active_*/env_* prefixing and no canonicalization step -- the
grasped bar for this action is identified by `bar_id` + the assembly sequence,
and every per-movement state re-classes only those bodies (attach to tool0 in
M1/M2, detach to world in M3/M4).

Module is importable without Rhino; the `build_*` helpers import
`rhinoscriptsyntax` lazily.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

from compas.geometry import Frame
from compas_robots import Configuration

# Data classes were extracted to the shared `rs_data_structure` package so
# this repo, husky-assembly-teleop, and husky_assembly_tamp share one
# interchange schema. Vendored as a git submodule at
# `external/rs_data_structure` (see README "Submodule dependency for BarAction
# export"). Prepend onto sys.path before import so a stale install from a
# sibling Rhino env doesn't shadow it -- same gotcha as compas_fab
# (`tasks/yh_lesson.md`).
_RS_DS_ROOT = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "external", "rs_data_structure",
    )
)
# Tolerate flat layout (<root>/rs_data_structure/__init__.py) and src
# layout (<root>/src/rs_data_structure/__init__.py).
_RS_DS_SRC = (
    _RS_DS_ROOT
    if os.path.isdir(os.path.join(_RS_DS_ROOT, "rs_data_structure"))
    else os.path.join(_RS_DS_ROOT, "src")
)
if not os.path.isdir(os.path.join(_RS_DS_SRC, "rs_data_structure")):
    raise RuntimeError(
        f"rs_data_structure submodule missing at {_RS_DS_ROOT}. "
        "Run `git submodule update --init --recursive`."
    )
if _RS_DS_SRC not in sys.path:
    sys.path.insert(0, _RS_DS_SRC)

from rs_data_structure.bar_action import (  # noqa: E402  (path-prepend gate above)
    Movement,
    RoboticFreeMovement,
    RoboticLinearMovement,
    RoboticDualArmConstrainedMovement,
    Action,
    BarAssemblyAction,
)


# ---------------------------------------------------------------------------
# Geometry helpers (numpy, no Rhino)
# ---------------------------------------------------------------------------


def _mm4_to_frame(matrix_mm) -> Frame:
    """Convert a 4x4 matrix with mm translation to a compas `Frame` (meters)."""
    m = np.asarray(matrix_mm, dtype=float)
    origin = m[:3, 3] / 1000.0
    return Frame(
        list(map(float, origin)),
        list(map(float, m[:3, 0])),
        list(map(float, m[:3, 1])),
    )


def _unit(vector):
    v = np.asarray(vector, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("Cannot unitize a near-zero vector.")
    return v / n


def _grasp_frame_in_tool0(tool0_world_mm, body_world_mm) -> Frame:
    """Return body's pose in tool0's local frame (meters)."""
    inv_t = np.linalg.inv(np.asarray(tool0_world_mm, dtype=float))
    grasp_mm = inv_t @ np.asarray(body_world_mm, dtype=float)
    return _mm4_to_frame(grasp_mm)


def _retreat_tool0_target_mm(tool0_assembled_mm, joint_world_mm, lm_distance_mm: float):
    """tool0 frame at retreat = assembled rotation, origin shifted by -joint_z * d."""
    j = np.asarray(joint_world_mm, dtype=float)
    axis_world = _unit(-j[:3, 2])  # joint local -Z
    out = np.array(tool0_assembled_mm, dtype=float, copy=True)
    out[:3, 3] = out[:3, 3] + axis_world * float(lm_distance_mm)
    return out, axis_world


def _compute_approach_targets_mm(tool0_left_assembled_mm, tool0_right_assembled_mm, lm_distance_mm: float):
    """Approach: both tool0 origins translated by -avg(tool_z) * lm_distance.

    Mirrors `rs_ik_keyframe._compute_approach_targets` (lines 999-1011); the
    upstream helper is private to that script, so the formula is duplicated
    here per the plan ("copy 5-15 LOC each, can't import without mutating
    upstream").
    """
    z_avg = (tool0_left_assembled_mm[:3, 2] + tool0_right_assembled_mm[:3, 2]) / 2.0
    approach_dir = _unit(-z_avg)
    offset = approach_dir * float(lm_distance_mm)
    left = np.array(tool0_left_assembled_mm, dtype=float, copy=True)
    right = np.array(tool0_right_assembled_mm, dtype=float, copy=True)
    left[:3, 3] = left[:3, 3] + offset
    right[:3, 3] = right[:3, 3] + offset
    return left, right


# ---------------------------------------------------------------------------
# State helpers
# ---------------------------------------------------------------------------


def _apply_groups_to_config(state, groups: dict) -> None:
    """Overwrite per-group joint values onto state.robot_configuration in place."""
    for _side, cfg in groups.items():
        names = cfg["joint_names"]
        values = cfg["joint_values"]
        for name, value in zip(names, values):
            state.robot_configuration[name] = float(value)


def _build_home_configuration(template_state, rcell, left_values, right_values, left_group: str, right_group: str) -> Configuration:
    """Return a Configuration with HOME values written onto a copy of template_state.robot_configuration."""
    cfg = template_state.robot_configuration.copy()
    left_names = list(rcell.get_configurable_joint_names(left_group))
    right_names = list(rcell.get_configurable_joint_names(right_group))
    for name, val in zip(left_names, left_values):
        cfg[name] = float(val)
    for name, val in zip(right_names, right_values):
        cfg[name] = float(val)
    return cfg


def _set_robot_base_frame(state, base_frame_world_mm) -> None:
    state.robot_base_frame = _mm4_to_frame(base_frame_world_mm)


# ---------------------------------------------------------------------------
# Attachment helpers
# ---------------------------------------------------------------------------

# State-independent canonical rigid-body name prefixes (the only naming the
# static-cell pipeline uses). Mirror of env_collision.CANONICAL_*.
CANONICAL_BAR_PREFIX = "bar_"
CANONICAL_JOINT_PREFIX = "joint_"

# Arm tool0 link names (mirror robot_cell._ARM_TOOL_LINKS).
_ARM_TOOL_LINKS = {
    "left": "left_ur_arm_tool0",
    "right": "right_ur_arm_tool0",
}


def _attach_body_to_arm_tool0(state, body_world_mm, tool0_arm_assembled_mm, arm_side: str, rb_key: str) -> None:
    """Attach any grasped body (bar or joint half) to an arm's tool0 link.

    Sets ``rb_state`` attached to ``<arm>_ur_arm_tool0`` with
    ``attachment_frame = inv(tool0) @ body_world`` (i.e. ``tool0_from_body``).

    Args:
        state (RobotCellState): state whose ``rigid_body_states[rb_key]`` is
            mutated in place.
        body_world_mm (ndarray): 4x4 mm world pose of the body at the assembled
            keyframe.
        tool0_arm_assembled_mm (ndarray): 4x4 mm world pose of that arm's flange
            at the assembled keyframe.
        arm_side (str): ``"left"`` / ``"right"`` -- which arm's tool0 to attach to.
        rb_key (str): canonical rigid-body name to attach.

    Returns:
        None: mutates ``state``.
    """
    rb = state.rigid_body_states[rb_key]
    rb.attached_to_link = _ARM_TOOL_LINKS[arm_side]
    rb.attached_to_tool = None
    rb.attachment_frame = _grasp_frame_in_tool0(tool0_arm_assembled_mm, body_world_mm)
    rb.frame = None
    rb.is_hidden = False


def _detach_to_world(state, world_mm, rb_key: str) -> None:
    """Clear attachment, place body at world frame."""
    rb = state.rigid_body_states[rb_key]
    rb.attached_to_link = None
    rb.attached_to_tool = None
    rb.attachment_frame = None
    rb.frame = _mm4_to_frame(world_mm)
    rb.is_hidden = False


# ---------------------------------------------------------------------------
# Rhino-side helpers (lazy import)
# ---------------------------------------------------------------------------


def _classify_male_joints_per_arm(bar_id: str) -> dict:
    """Return ``{joint_id: 'left' | 'right'}`` for every male joint on `bar_id`.

    Mirrors the arm-classification done by `rs_ik_keyframe._resolve_arm_tools_on_bar`
    (lines 185-234) and `ik_collision_setup.resolve_arm_tools_on_bar` -- keyed
    on joint_id (rather than oid) so the BarAction builder can drive
    per-joint attachment without depending on Rhino oids surviving the export.
    """
    import rhinoscriptsyntax as rs
    from core import config
    from core.rhino_tool_place import find_tool_for_joint

    out = {}
    if not rs.IsLayer(config.LAYER_JOINT_MALE_INSTANCES):
        return out
    for moid in rs.ObjectsByLayer(config.LAYER_JOINT_MALE_INSTANCES) or []:
        if rs.GetUserText(moid, "parent_bar_id") != bar_id:
            continue
        jid = rs.GetUserText(moid, "joint_id")
        if not jid:
            continue
        toid = find_tool_for_joint(jid)
        if toid is None:
            continue
        tname = rs.GetUserText(toid, "tool_name") or ""
        if not tname:
            continue
        last = tname.strip()[-1].upper()
        if last == "L":
            out[jid] = "left"
        elif last == "R":
            out[jid] = "right"
    return out


def _read_bar_keyframe(bar_oid):
    """Read the bar's saved IK keyframe data from its Rhino user-text.

    Args:
        bar_oid: Rhino object id of the bar curve.

    Returns:
        dict | None: ``{"base_frame_world_mm": np.ndarray(4, 4), "approach":
        {...}, "assembled": {...}}``, or ``None`` if any of the three
        ``KEY_ASSEMBLY_*`` records are missing/malformed.
    """
    import rhinoscriptsyntax as rs
    from core import config

    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not approach_raw or not assembled_raw:
        return None
    try:
        base_mm = np.asarray(json.loads(base_raw), dtype=float)
        if base_mm.shape != (4, 4):
            return None
        approach = json.loads(approach_raw)
        assembled = json.loads(assembled_raw)
    except (json.JSONDecodeError, ValueError):
        return None
    return {"base_frame_world_mm": base_mm, "approach": approach, "assembled": assembled}


# ---------------------------------------------------------------------------
# Per-movement builders
# ---------------------------------------------------------------------------


def _set_active_attachments(
    state,
    active_keys,
    env_geom: dict,
    arm_to_male: dict,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    bar_arm_side: str = "left",
) -> None:
    """Attach every grasped body in ``active_keys`` to its arm's tool0 link.

    - Canonical bar (``bar_<bid>``) attaches to ``bar_arm_side``'s tool0.
    - Canonical male joint (``joint_<jid>_male``) attaches to its classified arm.
    - Canonical female joint (``joint_<jid>_female``) attaches to ``bar_arm_side``
      (rigidly bonded to the bar).

    Each ``attachment_frame`` = ``inv(tool0_<arm>_assembled_world) @
    body_world_at_assembled`` (i.e. ``tool0_from_body``).

    Args:
        state (RobotCellState): movement start_state, mutated in place.
        active_keys (set): canonical names of the grasped bar + its joint halves.
        env_geom (dict): ``{name: body_info}`` with ``frame_world_mm`` world poses.
        arm_to_male (dict): ``{joint_id: 'left' | 'right'}`` for the grasped males.
        tool0_left_assembled_mm, tool0_right_assembled_mm (ndarray): 4x4 mm flange
            poses at the assembled keyframe.
        bar_arm_side (str): arm the bar + carried females attach to (default
            ``"left"``).

    Returns:
        None: mutates ``state``.
    """
    bar_tool0 = tool0_left_assembled_mm if bar_arm_side == "left" else tool0_right_assembled_mm
    for key in active_keys:
        body_info = env_geom.get(key)
        if body_info is None:
            continue
        if key.startswith(CANONICAL_BAR_PREFIX):
            _attach_body_to_arm_tool0(state, body_info["frame_world_mm"], bar_tool0, bar_arm_side, key)
            continue
        if not key.startswith(CANONICAL_JOINT_PREFIX):
            continue
        tag = key[len(CANONICAL_JOINT_PREFIX):]
        if "_" not in tag:
            continue
        jid, sub = tag.rsplit("_", 1)
        if sub == "male":
            arm = arm_to_male.get(jid, bar_arm_side)
        else:
            arm = bar_arm_side  # females rigidly bonded to bar -> bar's gripper
        tool0_arm = tool0_left_assembled_mm if arm == "left" else tool0_right_assembled_mm
        _attach_body_to_arm_tool0(state, body_info["frame_world_mm"], tool0_arm, arm, key)


def _detach_active_to_assembled_world(state, active_keys, env_geom: dict) -> None:
    """Detach every grasped body in ``active_keys`` and place it at its world frame.

    Used by M3/M4, where the bar is released: each body becomes a static
    obstacle at its assembled world pose (``attached_to_link=None``).

    Args:
        state (RobotCellState): movement start_state, mutated in place.
        active_keys (set): canonical names of the (now-released) bar + joints.
        env_geom (dict): ``{name: body_info}`` with ``frame_world_mm`` world poses.

    Returns:
        None: mutates ``state``.
    """
    for key in active_keys:
        body_info = env_geom.get(key)
        if body_info is None:
            continue
        _detach_to_world(state, body_info["frame_world_mm"], key)


def _apply_movement_touch_policy(
    state, movement: str, active_keys, env_geom: dict, arm_to_male: dict,
    bar_key: str, tool_ids: dict,
) -> None:
    """Set per-movement allowed contacts (``touch_bodies``) on the grasped bodies.

    Built directly per movement (no build-full-then-strip):

    - **M1** (grasped, halves apart): ``male<->its arm tool``, ``male<->bar``,
      ``carried_female<->bar``.
    - **M2** (grasped, mating): M1 + ``male<->built_female`` (same joint_id, the
      mate on the already-built bar).
    - **M3** (released, tool peeling off): ``male<->tool`` only -- everything else
      is detached/static, so compas_fab auto-skips it.
    - **M4** (gone): nothing.

    The whitelist is recorded on the male / carried-female side (one side is
    enough); the bar tube carries none (the tool grips the male, not the tube).

    Args:
        state (RobotCellState): movement start_state, mutated in place.
        movement (str): one of ``"M1"`` / ``"M2"`` / ``"M3"`` / ``"M4"``.
        active_keys (set): canonical names of the grasped bar + its joint halves.
        env_geom (dict): ``{name: body_info}`` (used to confirm the mate
            ``joint_<jid>_female`` exists for M2).
        arm_to_male (dict): ``{joint_id: 'left' | 'right'}`` for the grasped males.
        bar_key (str): canonical name of the grasped bar (``bar_<id>``).
        tool_ids (dict): ``{"left": tool_id, "right": tool_id}``.

    Returns:
        None: mutates ``state``.
    """
    for jid, arm in arm_to_male.items():
        male_rb = state.rigid_body_states.get(f"{CANONICAL_JOINT_PREFIX}{jid}_male")
        if male_rb is None:
            continue
        tool = tool_ids.get(arm)
        partners = []
        if movement in ("M1", "M2"):
            if tool:
                partners.append(tool)
            partners.append(bar_key)
            if movement == "M2":
                female_key = f"{CANONICAL_JOINT_PREFIX}{jid}_female"
                if female_key in env_geom:
                    partners.append(female_key)
        elif movement == "M3":
            if tool:
                partners.append(tool)
        male_rb.touch_bodies = sorted(set(partners))

    for key in active_keys:
        if not (key.startswith(CANONICAL_JOINT_PREFIX) and key.endswith("_female")):
            continue
        frb = state.rigid_body_states.get(key)
        if frb is not None:
            frb.touch_bodies = [bar_key] if movement in ("M1", "M2") else []

    bar_rb = state.rigid_body_states.get(bar_key)
    if bar_rb is not None:
        bar_rb.touch_bodies = []


def _build_m1(
    template_state,
    bar_id: str,
    rcell,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
    bar_key: str,
    tool_ids: dict,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    base_frame_world_mm,
    home_left,
    home_right,
    left_group: str,
    right_group: str,
    lm_distance_mm: float,
    bar_arm_side: str = "left",
) -> RoboticDualArmConstrainedMovement:
    """Build M1: home -> approach, gripping the bar (constrained dual-arm).

    Args:
        template_state (RobotCellState): full static-cell template to clone.
        bar_id (str): the bar being assembled.
        rcell (RobotCell): cached cell (for HOME configuration joint names).
        env_geom (dict): ``{name: body_info}`` with world frames.
        active_keys (set): canonical names of the grasped bar + its joint halves.
        arm_to_male (dict): ``{joint_id: 'left' | 'right'}``.
        bar_key (str): canonical grasped-bar name (``bar_<id>``).
        tool_ids (dict): ``{"left": tool_id, "right": tool_id}``.
        tool0_left_assembled_mm, tool0_right_assembled_mm (ndarray): 4x4 mm
            flange poses at the assembled keyframe.
        base_frame_world_mm (ndarray): 4x4 mm robot base frame.
        home_left, home_right (list): HOME joint values per arm.
        left_group, right_group (str): planning-group names.
        lm_distance_mm (float): approach offset distance.
        bar_arm_side (str): arm the bar attaches to (default ``"left"``).

    Returns:
        RoboticDualArmConstrainedMovement: the M1 movement (start_state at HOME,
        gripping the bar; target = approach EE frames).
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    state.robot_configuration = _build_home_configuration(
        template_state, rcell, home_left, home_right, left_group, right_group,
    )
    _set_active_attachments(
        state, active_keys, env_geom, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        bar_arm_side=bar_arm_side,
    )
    _apply_movement_touch_policy(
        state, "M1", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )
    tool0_left_approach_mm, tool0_right_approach_mm = _compute_approach_targets_mm(
        tool0_left_assembled_mm, tool0_right_assembled_mm, lm_distance_mm,
    )
    return RoboticDualArmConstrainedMovement(
        movement_id=f"{bar_id}_M1_CDFM_home_to_approach",
        tag="Home -> Approach (gripped bar, fixed relative EE)",
        start_state=state,
        target_ee_frames={
            "left": _mm4_to_frame(tool0_left_approach_mm),
            "right": _mm4_to_frame(tool0_right_approach_mm),
        },
        target_configuration=None,
        notes={
            "constraint": "fixed_relative_ee_transform",
            "approach_offset_mm": float(lm_distance_mm),
            "bar_arm_side": bar_arm_side,
        },
    )


def _build_m2(
    template_state,
    bar_id: str,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
    bar_key: str,
    tool_ids: dict,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    base_frame_world_mm,
    approach_groups: dict,
    lm_distance_mm: float,
    bar_arm_side: str = "left",
) -> RoboticLinearMovement:
    """Build M2: approach -> assembled (linear mate), still gripping the bar.

    Shared args are as in :func:`_build_m1`.

    Args:
        approach_groups (dict): per-arm approach-keyframe joint config
            (``{side: {"joint_names": [...], "joint_values": [...]}}``) written
            onto the start_state.

    Returns:
        RoboticLinearMovement: the M2 movement (start_state at the approach
        config; target = assembled EE frames; mate touch whitelisted).
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    _apply_groups_to_config(state, approach_groups)
    _set_active_attachments(
        state, active_keys, env_geom, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        bar_arm_side=bar_arm_side,
    )
    _apply_movement_touch_policy(
        state, "M2", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )
    return RoboticLinearMovement(
        movement_id=f"{bar_id}_M2_LM_mate",
        tag="Approach -> Assembled (linear mate)",
        start_state=state,
        target_ee_frames={
            "left": _mm4_to_frame(tool0_left_assembled_mm),
            "right": _mm4_to_frame(tool0_right_assembled_mm),
        },
        target_configuration=None,
        notes={
            "lm_axis": "per_tool0_z_avg",
            "lm_distance_mm": float(lm_distance_mm),
            "bar_arm_side": bar_arm_side,
        },
    )


def _build_m3(
    template_state,
    bar_id: str,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
    bar_key: str,
    tool_ids: dict,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    base_frame_world_mm,
    assembled_groups: dict,
    lm_distance_mm: float,
) -> RoboticLinearMovement:
    """Build M3: assembled -> retreated (per-arm linear), bar released.

    Shared args are as in :func:`_build_m1`.

    Args:
        assembled_groups (dict): per-arm assembled-keyframe joint config written
            onto the start_state.

    Returns:
        RoboticLinearMovement: the M3 movement (start_state at the assembled
        config with the bar/joints detached to their world poses; target =
        per-arm retreated EE frames; only male<->tool whitelisted).
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    _apply_groups_to_config(state, assembled_groups)
    _detach_active_to_assembled_world(state, active_keys, env_geom)
    # Released: bar + joints are static at the assembled world pose. Only the
    # tool is still peeling off the male, so allow male<->tool; everything else
    # is static<->static (compas_fab auto-skips).
    _apply_movement_touch_policy(
        state, "M3", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )

    retreat_axes_world = {}
    target_left_mm = tool0_left_assembled_mm
    target_right_mm = tool0_right_assembled_mm
    for arm, tool0_assembled in (("left", tool0_left_assembled_mm), ("right", tool0_right_assembled_mm)):
        # Find this arm's male joint OCF.
        jid = next((j for j, a in arm_to_male.items() if a == arm), None)
        if jid is None:
            continue
        joint_key = f"{CANONICAL_JOINT_PREFIX}{jid}_male"
        joint_body_info = env_geom.get(joint_key)
        if joint_body_info is None:
            continue
        target_mm, axis_world = _retreat_tool0_target_mm(
            tool0_assembled, joint_body_info["frame_world_mm"], lm_distance_mm,
        )
        if arm == "left":
            target_left_mm = target_mm
        else:
            target_right_mm = target_mm
        retreat_axes_world[arm] = [float(x) for x in axis_world]

    return RoboticLinearMovement(
        movement_id=f"{bar_id}_M3_LM_retreat",
        tag="Assembled -> Retreated (per-arm linear)",
        start_state=state,
        target_ee_frames={
            "left": _mm4_to_frame(target_left_mm),
            "right": _mm4_to_frame(target_right_mm),
        },
        target_configuration=None,
        notes={
            "lm_axis": "per_joint_neg_z",
            "lm_distance_mm": float(lm_distance_mm),
            "retreat_axes_world": retreat_axes_world,
        },
    )


def _build_m4(
    template_state,
    bar_id: str,
    rcell,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
    bar_key: str,
    tool_ids: dict,
    base_frame_world_mm,
    home_left,
    home_right,
    left_group: str,
    right_group: str,
) -> RoboticFreeMovement:
    """Build M4: retreated -> home (free joint-space motion), bar released.

    Shared args are as in :func:`_build_m1`.

    Returns:
        RoboticFreeMovement: the M4 movement. ``start_state.robot_configuration``
        is ``None`` (the planner fills it from M3's end); the bar/joints are
        detached at their world poses; target = HOME configuration.
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    # Per plan: M4.start.robot_configuration is None (planner fills from M3.end).
    state.robot_configuration = None
    _detach_active_to_assembled_world(state, active_keys, env_geom)
    # Fully released, tools retreated: no special allowed contacts.
    _apply_movement_touch_policy(
        state, "M4", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )

    home_cfg = _build_home_configuration(
        template_state, rcell, home_left, home_right, left_group, right_group,
    )
    return RoboticFreeMovement(
        movement_id=f"{bar_id}_M4_free_home",
        tag="Retreated -> Home (free motion)",
        start_state=state,
        target_ee_frames=None,
        target_configuration=home_cfg,
        notes={
            "start_config_is_none": True,
            "planner_fills": "start_state.robot_configuration",
        },
    )


# ---------------------------------------------------------------------------
# Top-level factory
# ---------------------------------------------------------------------------


def build_bar_assembly_action(rcell, planner, bar_id: str, bar_oid):
    """Build a `BarAssemblyAction` for `bar_id`. Rhino-only call.

    `prepare_assembly_collision_state` reuses the cached static cell (the full
    canonical assembly + env obstacles + arm ToolModels, built by
    RSRebuildRobotCell) and returns a full-key-set template state: built bars
    visible static, the grasped bar a static obstacle, not-yet-built bars
    `is_hidden=True`, tools attached. Each of the four movements clones that
    template and re-classes only the grasped (active-bar) bodies + sets its own
    allowed-touch policy. No canonicalization, no snapshot/restore.

    Args:
        rcell (RobotCell): the cached static cell.
        planner (PyBulletPlanner): active planner.
        bar_id (str): id of the bar to assemble.
        bar_oid: Rhino object id of the bar curve (its ``KEY_ASSEMBLY_*``
            user-text supplies the base frame + approach/assembled keyframes).

    Returns:
        BarAssemblyAction: the four-movement (M1-M4) action for ``bar_id``.

    Raises:
        RuntimeError: if the bar is missing IK keyframe user-text or its two
            arm tools can't be resolved.
    """
    import rhinoscriptsyntax as rs  # noqa: F401  (kept; surrounding helpers import lazily)
    from core import config
    from core import env_collision
    from core import ik_collision_setup
    from core import robot_cell
    from core.rhino_bar_registry import get_bar_seq_map

    print(f"core.bar_action.build_bar_assembly_action: building bar '{bar_id}' ...")

    # 1) Read keyframe + base from the bar curve.
    bar_keyframe = _read_bar_keyframe(bar_oid)
    if bar_keyframe is None:
        raise RuntimeError(
            f"Bar '{bar_id}' is missing one of "
            f"'{config.KEY_ASSEMBLY_BASE_FRAME}'/'{config.KEY_ASSEMBLY_IK_APPROACH}'/"
            f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}'. Run RSIKKeyframe first."
        )
    base_frame_world_mm = bar_keyframe["base_frame_world_mm"]

    # TODO the tools should not be separately resolved as objects per bar, but rather there should only be two ToolModel attached to the robot's tool0 links in the RobotCell, and in robot_cell_state we simply attach the bar in action to the left arm of the dual arm. (since in consrained dual arm planning, and the insertion dual arm linear motion, the constraints on two arms'EE is enf)
    # 2) Resolve per-arm tools on the bar.
    arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        raise RuntimeError(f"Bar '{bar_id}': {err}")

    # 3) Tool0 world transforms at IK_ASSEMBLED (= placed tool block instance xforms).
    tool0_left_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["left"])
    tool0_right_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["right"])

    # 4) Build the full static-cell template state. `prepare_assembly_collision_state`
    # calls `ensure_assembly_cell` (registers the full canonical assembly + env
    # obstacles + arm ToolModels onto the cached cell) and returns a full-key-set
    # state: built bars/joints visible static, the active bar a static obstacle,
    # not-yet-built bars/joints `is_hidden=True`, the two arm tools attached. No
    # canonicalization, no snapshot/restore -- the cell IS the persistent static
    # cell now. Every Mi re-classes only the active (grasped) bodies.
    slim_state = robot_cell.default_cell_state()
    _set_robot_base_frame(slim_state, base_frame_world_mm)
    template_state, env_geom = ik_collision_setup.prepare_assembly_collision_state(
        rcell, planner, slim_state, bar_id,
    )
    arm_to_male = _classify_male_joints_per_arm(bar_id)

    # The grasped (active) bodies = bar_<bar_id> + every joint half mounted on it.
    active_keys = {
        name for name, body_info in env_geom.items()
        if body_info.get("parent_bar_id") == bar_id
    }
    bar_key = f"{CANONICAL_BAR_PREFIX}{bar_id}"
    tool_ids = robot_cell.arm_tool_ids()

    # The active bar/joint frames in the template are the assembled-pose world
    # frames; every Mi rewrites them (M1/M2 attach to tool0, M3/M4 detach to
    # world). Clear them so a missed re-class surfaces as a None-frame error.
    for k in active_keys:
        rb = template_state.rigid_body_states.get(k)
        if rb is not None:
            rb.frame = None

    # 6) Build movements.
    seq_map = get_bar_seq_map()
    # Full ordered list of bar ids = assembly sequence (ascending seq number).
    assembly_seq = [
        bid for bid, _oid_seq in sorted(seq_map.items(), key=lambda kv: kv[1][1])
    ]
    active_index = assembly_seq.index(bar_id) if bar_id in assembly_seq else -1

    m1 = _build_m1(
        template_state, bar_id, rcell, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        config.HOME_CONFIG_LEFT, config.HOME_CONFIG_RIGHT,
        config.LEFT_GROUP, config.RIGHT_GROUP,
        config.LM_DISTANCE,
    )
    m2 = _build_m2(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        bar_keyframe["approach"],
        config.LM_DISTANCE,
    )
    m3 = _build_m3(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        bar_keyframe["assembled"],
        config.LM_DISTANCE,
    )
    m4 = _build_m4(
        template_state, bar_id, rcell, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        base_frame_world_mm,
        config.HOME_CONFIG_LEFT, config.HOME_CONFIG_RIGHT,
        config.LEFT_GROUP, config.RIGHT_GROUP,
    )

    return BarAssemblyAction(
        action_id=f"{bar_id}_A0_assemble",
        tag=f"Assemble bar {bar_id} (index {active_index} of {len(assembly_seq)})",
        movements=[m1, m2, m3, m4],
        active_bar_id=bar_id,
        assembly_seq=assembly_seq,
    )
