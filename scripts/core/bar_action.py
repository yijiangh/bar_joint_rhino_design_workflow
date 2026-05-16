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
Upstream (`env_collision`) labels rigid bodies with state-dependent prefixes
(`active_bar_<bid>` for the bar being assembled now, `env_bar_<bid>` for
already-built bars; same for joints). The downstream artifact saves these
under canonical, state-independent names (`bar_<bid>`, `joint_<jid>_<sub>`)
so a single `RobotCell.json` can be reused across every per-bar state.
The renaming happens here, only on the export-bound copies; the in-Rhino
cached `RobotCell` keeps the active_*/env_* prefixes so upstream
collision setup keeps working unchanged. Use
`canonicalize_state` / `dump_cell_canonical` to apply the same renaming
to any state or cell being saved.

Module is importable without Rhino; the `build_*` helpers import
`rhinoscriptsyntax` lazily.
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np

from compas.data import Data
from compas.geometry import Frame
from compas_robots import Configuration


# ---------------------------------------------------------------------------
# Data classes (pure python, no Rhino dependency)
# ---------------------------------------------------------------------------


class Movement(Data):
    """One robot movement in an assembly action.

    Class is a base; the concrete subclass (`RoboticFreeMovement`,
    `RoboticLinearMovement`, `RoboticDualArmConstrainedMovement`) IS the
    discriminator -- no `motion_kind` string field.
    """

    def __init__(
        self,
        movement_id: str = "",
        tag: str = "",
        start_state=None,
        target_ee_frames: Optional[dict] = None,
        target_configuration: Optional[Configuration] = None,
        trajectory=None,
        notes: Optional[dict] = None,
    ):
        super(Movement, self).__init__()
        self.movement_id = movement_id
        self.tag = tag
        self.start_state = start_state
        self.target_ee_frames = target_ee_frames if target_ee_frames is not None else {}
        self.target_configuration = target_configuration
        self.trajectory = trajectory
        self.notes = notes if notes is not None else {}

    @property
    def __data__(self):
        return {
            "movement_id": self.movement_id,
            "tag": self.tag,
            "start_state": self.start_state,
            "target_ee_frames": self.target_ee_frames,
            "target_configuration": self.target_configuration,
            "trajectory": self.trajectory,
            "notes": self.notes,
        }


class RoboticFreeMovement(Movement):
    """Unconstrained joint-space motion (typically free-home)."""


class RoboticLinearMovement(Movement):
    """Cartesian linear motion of the tool flange(s)."""


class RoboticDualArmConstrainedMovement(Movement):
    """Dual-arm motion preserving the relative tool0_left -> tool0_right transform.

    The constraint is implied by the class type AND by the rigid body
    attachments inside `start_state` (one body attached to each arm's
    tool0).
    """


class Action(Data):
    """Base class for any high-level action that groups a list of movements."""

    def __init__(
        self,
        action_id: str = "",
        tag: str = "",
        movements: Optional[list] = None,
    ):
        super(Action, self).__init__()
        self.action_id = action_id
        self.tag = tag
        self.movements = list(movements) if movements is not None else []

    @property
    def __data__(self):
        return {
            "action_id": self.action_id,
            "tag": self.tag,
            "movements": self.movements,
        }


class BarAssemblyAction(Action):
    """One bar's full assemble cycle (M1 .. M4).

    ``assembly_seq`` is the full ordered list of bar ids in the Rhino assembly
    sequence (ascending ``bar_seq``), not just the active bar's index. The
    downstream planner can derive the active bar's position via
    ``assembly_seq.index(active_bar_id)`` and use the prefix list as the set
    of "already-built env" bars.
    """

    def __init__(
        self,
        action_id: str = "",
        tag: str = "",
        movements: Optional[list] = None,
        active_bar_id: str = "",
        assembly_seq: Optional[list] = None,
    ):
        super(BarAssemblyAction, self).__init__(
            action_id=action_id, tag=tag, movements=movements,
        )
        self.active_bar_id = active_bar_id
        self.assembly_seq = list(assembly_seq) if assembly_seq is not None else []

    @property
    def __data__(self):
        d = dict(super(BarAssemblyAction, self).__data__)
        d["active_bar_id"] = self.active_bar_id
        d["assembly_seq"] = self.assembly_seq
        return d


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

# Local mirrors of env_collision prefix constants. Duplicated to keep this
# module importable without mutating upstream. After canonicalization (see
# `canonicalize_state` / `canonical_rb_name`), the bodies are keyed by the
# CANONICAL_*_PREFIX values below. The active_*/env_* prefixes only appear
# transiently between `prepare_assembly_collision_state` and canonicalization.
ACTIVE_RB_BAR_PREFIX = "active_bar_"
ACTIVE_RB_JOINT_PREFIX = "active_joint_"
ENV_RB_BAR_PREFIX = "env_bar_"
ENV_RB_JOINT_PREFIX = "env_joint_"

CANONICAL_BAR_PREFIX = "bar_"
CANONICAL_JOINT_PREFIX = "joint_"

# Arm tool0 link names (mirror robot_cell._ARM_TOOL_LINKS).
_ARM_TOOL_LINKS = {
    "left": "left_ur_arm_tool0",
    "right": "right_ur_arm_tool0",
}


# ---------------------------------------------------------------------------
# State-independent ("canonical") rigid-body naming
# ---------------------------------------------------------------------------


def canonical_rb_name(name: str) -> str:
    """Strip the upstream `active_`/`env_` prefix to produce a state-independent name.

    ``active_bar_B6`` / ``env_bar_B6`` -> ``bar_B6``
    ``active_joint_J3-6_male`` / ``env_joint_J3-6_male`` -> ``joint_J3-6_male``

    Anything else (e.g. ``AssemblyLeftArmToolBody``) is returned unchanged.
    Idempotent.
    """
    for p in (ACTIVE_RB_BAR_PREFIX, ENV_RB_BAR_PREFIX):
        if name.startswith(p):
            return CANONICAL_BAR_PREFIX + name[len(p):]
    for p in (ACTIVE_RB_JOINT_PREFIX, ENV_RB_JOINT_PREFIX):
        if name.startswith(p):
            return CANONICAL_JOINT_PREFIX + name[len(p):]
    return name


def canonical_rigid_body_models(rcell) -> dict:
    """Return a NEW dict mapping canonical names to the same RigidBody instances.

    Does not mutate ``rcell``. Collisions (two upstream prefixes mapping to
    the same canonical name) are detected and reported via RuntimeError --
    in practice this cannot happen because each joint half lives on exactly
    one bar (either active or env, never both).
    """
    out = {}
    for name, rb in rcell.rigid_body_models.items():
        cname = canonical_rb_name(name)
        if cname in out:
            raise RuntimeError(
                f"canonical_rigid_body_models: collision on '{cname}' "
                f"(both '{name}' and a previous entry map to it)"
            )
        out[cname] = rb
    return out


def canonicalize_state(state) -> None:
    """In-place mutation of ``state.rigid_body_states`` and ACM cross-refs to
    canonical names.

    - Renames every key in ``state.rigid_body_states``.
    - Rewrites every ``touch_bodies`` entry on every RigidBodyState
      (including ones whose key did not change, e.g. ``AssemblyLeftArmToolBody``)
      via ``canonical_rb_name``, so cross-references to active_*/env_*
      bodies all land on canonical names.

    Idempotent.
    """
    rename = {k: canonical_rb_name(k) for k in state.rigid_body_states.keys()}
    new_states = {}
    for old, new in rename.items():
        if new in new_states:
            raise RuntimeError(
                f"canonicalize_state: collision on '{new}' (mapped from '{old}')"
            )
        new_states[new] = state.rigid_body_states[old]
    # Apply canonical_rb_name to every touch_body string regardless of rename-dict
    # membership, so cross-refs from tool / non-active RBs also get canonicalized.
    for rb in new_states.values():
        if rb.touch_bodies:
            rb.touch_bodies = sorted({canonical_rb_name(t) for t in rb.touch_bodies})
    state.rigid_body_states = new_states


def canonicalize_env_geom(env_geom: dict) -> dict:
    """Return a new ``env_geom`` dict with canonical keys (same payloads)."""
    out = {}
    for k, v in env_geom.items():
        cname = canonical_rb_name(k)
        if cname in out:
            raise RuntimeError(f"canonicalize_env_geom: collision on '{cname}'")
        out[cname] = v
    return out


def dump_cell_canonical(rcell, fileobj, *, pretty: bool = True) -> None:
    """Write ``rcell`` to ``fileobj`` using canonical rigid-body names.

    Temporarily swaps the cell's ``rigid_body_models`` attribute with a
    canonical-keyed shallow copy, dumps, then restores the original. No
    deep copy of the RobotCell is performed (URDF + meshes stay shared).
    """
    from compas import json_dump

    canonical = canonical_rigid_body_models(rcell)
    original = rcell.rigid_body_models
    rcell.rigid_body_models = canonical
    try:
        json_dump(rcell, fileobj, pretty=pretty)
    finally:
        rcell.rigid_body_models = original


def _attach_active_bar_to_arm(state, body_world_mm, tool0_arm_assembled_mm, arm_side: str, rb_key: str) -> None:
    """Set rb_state to be attached to <arm>_ur_arm_tool0 with grasp = inv(tool0) @ body_world."""
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
# ACM helpers (per-movement)
# ---------------------------------------------------------------------------


def _clear_active_touch_bodies(state, active_keys) -> None:
    """Empty touch_bodies on every key in `active_keys`. M1/M2 ACM is set by
    upstream `configure_active_assembly_acm` and then carried over via
    canonicalization; M3/M4 callers wipe and selectively rewrite."""
    for k in active_keys:
        rb = state.rigid_body_states.get(k)
        if rb is not None:
            rb.touch_bodies = []


def _whitelist_mate_pairs(state) -> int:
    """Whitelist every `joint_<jid>_male` <-> `joint_<jid>_female` pair in the
    state by adding each side to the other's `touch_bodies`.

    Operates on canonical names: every existing male/female pair (regardless
    of whether either side was active or env upstream) is by-design contact
    once mated, so the whitelist prevents false positives in M3/M4.

    Returns the number of pairs whitelisted.
    """
    rb_states = state.rigid_body_states
    seen = set()
    n = 0
    for key in list(rb_states.keys()):
        if not key.startswith(CANONICAL_JOINT_PREFIX):
            continue
        tag = key[len(CANONICAL_JOINT_PREFIX):]
        if "_" not in tag:
            continue
        jid, sub = tag.rsplit("_", 1)
        if sub == "male":
            other = f"{CANONICAL_JOINT_PREFIX}{jid}_female"
        elif sub == "female":
            other = f"{CANONICAL_JOINT_PREFIX}{jid}_male"
        else:
            continue
        if other not in rb_states:
            continue
        pair = tuple(sorted([key, other]))
        if pair in seen:
            continue
        seen.add(pair)
        a_tb = set(rb_states[key].touch_bodies or [])
        e_tb = set(rb_states[other].touch_bodies or [])
        a_tb.add(other)
        e_tb.add(key)
        rb_states[key].touch_bodies = sorted(a_tb)
        rb_states[other].touch_bodies = sorted(e_tb)
        n += 1
    return n


def _whitelist_gripper_male_adjacency(state, arm_to_male: dict, arm_tool_rb_names: dict) -> int:
    """Add `joint_<jid>_male` <-> matching arm tool RB to touch_bodies.

    Used in M3 retreat: gripper is still adjacent to the just-mated joint as
    it pulls back; that contact should not flag.
    """
    rb_states = state.rigid_body_states
    n = 0
    for jid, arm in arm_to_male.items():
        rb_key = f"{CANONICAL_JOINT_PREFIX}{jid}_male"
        tool_rb = arm_tool_rb_names.get(arm)
        if rb_key not in rb_states or not tool_rb:
            continue
        a_tb = set(rb_states[rb_key].touch_bodies or [])
        a_tb.add(tool_rb)
        rb_states[rb_key].touch_bodies = sorted(a_tb)
        if tool_rb in rb_states:
            t_tb = set(rb_states[tool_rb].touch_bodies or [])
            t_tb.add(rb_key)
            rb_states[tool_rb].touch_bodies = sorted(t_tb)
        n += 1
    return n


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


def _read_bar_payload(bar_oid):
    """Return ``{"base_frame_world_mm": np.ndarray(4,4), "approach": {...}, "assembled": {...}}`` or None."""
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
    """Set attached_to_link/_tool/_frame on every canonical key in ``active_keys``.

    - Canonical bar (``bar_<bid>``) attaches to bar_arm_side's tool0.
    - Canonical male joint (``joint_<jid>_male``) attaches to its classified arm.
    - Canonical female joint (``joint_<jid>_female``) attaches to bar_arm_side
      (rigidly bonded to the bar).

    ``attachment_frame`` = inv(tool0_<arm>_assembled_world) @ body_world_at_assembled.
    """
    bar_tool0 = tool0_left_assembled_mm if bar_arm_side == "left" else tool0_right_assembled_mm
    for key in active_keys:
        payload = env_geom.get(key)
        if payload is None:
            continue
        if key.startswith(CANONICAL_BAR_PREFIX):
            _attach_active_bar_to_arm(state, payload["frame_world_mm"], bar_tool0, bar_arm_side, key)
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
        _attach_active_bar_to_arm(state, payload["frame_world_mm"], tool0_arm, arm, key)


def _detach_active_to_assembled_world(state, active_keys, env_geom: dict) -> None:
    """Detach every canonical key in ``active_keys`` and place at its assembled world frame."""
    for key in active_keys:
        payload = env_geom.get(key)
        if payload is None:
            continue
        _detach_to_world(state, payload["frame_world_mm"], key)


def _build_m1(
    template_state,
    bar_id: str,
    rcell,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
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
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    base_frame_world_mm,
    approach_groups: dict,
    lm_distance_mm: float,
    bar_arm_side: str = "left",
) -> RoboticLinearMovement:
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    _apply_groups_to_config(state, approach_groups)
    _set_active_attachments(
        state, active_keys, env_geom, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        bar_arm_side=bar_arm_side,
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
    arm_tool_rb_names: dict,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    base_frame_world_mm,
    assembled_groups: dict,
    lm_distance_mm: float,
) -> RoboticLinearMovement:
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    _apply_groups_to_config(state, assembled_groups)
    _detach_active_to_assembled_world(state, active_keys, env_geom)
    _clear_active_touch_bodies(state, active_keys)
    _whitelist_mate_pairs(state)
    _whitelist_gripper_male_adjacency(state, arm_to_male, arm_tool_rb_names)

    retreat_axes_world = {}
    target_left_mm = tool0_left_assembled_mm
    target_right_mm = tool0_right_assembled_mm
    for arm, tool0_assembled in (("left", tool0_left_assembled_mm), ("right", tool0_right_assembled_mm)):
        # Find this arm's male joint OCF.
        jid = next((j for j, a in arm_to_male.items() if a == arm), None)
        if jid is None:
            continue
        joint_key = f"{CANONICAL_JOINT_PREFIX}{jid}_male"
        joint_payload = env_geom.get(joint_key)
        if joint_payload is None:
            continue
        target_mm, axis_world = _retreat_tool0_target_mm(
            tool0_assembled, joint_payload["frame_world_mm"], lm_distance_mm,
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
    base_frame_world_mm,
    home_left,
    home_right,
    left_group: str,
    right_group: str,
) -> RoboticFreeMovement:
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    # Per plan: M4.start.robot_configuration is None (planner fills from M3.end).
    state.robot_configuration = None
    _detach_active_to_assembled_world(state, active_keys, env_geom)
    _clear_active_touch_bodies(state, active_keys)
    _whitelist_mate_pairs(state)

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
# Full-assembly registration (state-independent RobotCell)
# ---------------------------------------------------------------------------


def _attach_arm_tools_to_cell(rcell, planner) -> bool:
    """Ensure ``AssemblyLeftArmToolBody`` / ``AssemblyRightArmToolBody`` are
    registered on ``rcell.rigid_body_models``.

    Scans the document for ANY bar that has both a left- and right-arm tool
    placed (via ``ik_collision_setup.resolve_arm_tools_on_bar``), reads each
    tool's collision OBJ path from ``robotic_tools.json`` and calls
    ``robot_cell.attach_arm_tool_rigid_bodies``. The arm-tool RBs are global
    (per-arm, geometry-only) -- the same bodies any BarAction state expects --
    so it doesn't matter which bar provides the tool config.

    Returns True if both RBs end up registered; False if no bar with both
    tools could be found (cell will lack the arm-tool RBs and any BarAction
    state will mismatch on ``AssemblyLeftArmToolBody`` / ``AssemblyRightArmToolBody``).
    Idempotent: re-runs that point at the same OBJ are no-ops.
    """
    from core import ik_collision_setup
    from core import robot_cell
    from core.rhino_bar_registry import get_bar_seq_map

    seq_map = get_bar_seq_map()
    arm_tools = None
    for bar_id in seq_map:
        result, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
        if err is None:
            arm_tools = result
            break
    if arm_tools is None:
        print(
            "core.bar_action._attach_arm_tools_to_cell: no bar in the document "
            "carries both left+right arm tools; skipping arm-tool RB attach."
        )
        return False
    left_path, right_path = ik_collision_setup.resolve_tool_collision_paths(
        arm_tools["left"], arm_tools["right"],
    )
    registered = robot_cell.attach_arm_tool_rigid_bodies(
        rcell, planner,
        left_collision_path=left_path,
        right_collision_path=right_path,
        native_scale=0.001,
    )
    return bool(registered.get("left")) and bool(registered.get("right"))


def _register_full_assembly_geom(rcell, planner) -> dict:
    """Register EVERY bar + EVERY joint of the assembly into ``rcell.rigid_body_models``.

    ``env_collision.collect_built_geometry(last_bar)`` yields all bars with
    ``seq < last_seq`` + their joints; ``collect_active_geometry(last_bar)``
    yields the last bar + its joints. The union is the full assembly. Keys
    keep the upstream prefixes (``env_bar_*`` for all but the last, ``active_bar_*``
    for the last bar's bodies); the caller canonicalizes them.

    Pushed via ``robot_cell.ensure_env_registered`` so the in-Rhino cached cell
    and the planner's collision world both carry the full set. (Subsequent
    upstream IK calls re-sync the cell to whatever bar is active then -- this
    full set is transient, only meaningful for the export that follows.)

    Returns the merged prefixed-key geom dict, or ``{}`` if no bars are registered.
    """
    from core import env_collision
    from core import robot_cell
    from core.rhino_bar_registry import get_bar_seq_map

    seq_map = get_bar_seq_map()
    if not seq_map:
        return {}
    last_bar = max(seq_map, key=lambda b: seq_map[b][1])
    full_geom = {}
    full_geom.update(env_collision.collect_built_geometry(last_bar, seq_map))
    full_geom.update(env_collision.collect_active_geometry(last_bar, seq_map))
    if full_geom:
        robot_cell.ensure_env_registered(rcell, full_geom, planner)
    return full_geom


# ---------------------------------------------------------------------------
# Top-level factory
# ---------------------------------------------------------------------------


def build_bar_assembly_action(rcell, planner, bar_id: str, bar_oid):
    """Build a `BarAssemblyAction` for `bar_id`. Rhino-only call.

    Side effects:
    - `prepare_assembly_collision_state` registers env+arm-tool RBs onto
      `rcell.rigid_body_models` and `configure_active_assembly_acm` on the
      returned state. We treat that as the M1/M2 collision template and
      clone+mutate it for each of the four movements.
    - `_register_full_assembly_geom` then registers EVERY bar + joint onto
      `rcell.rigid_body_models` so a subsequent `RSExportRobotCell` writes a
      state-independent cell. To keep the per-movement states' workpiece
      key-set equal to the (full) cell's, the not-yet-built bars/joints are
      added to every movement's start_state as `is_hidden=True` rigid bodies.
    """
    import rhinoscriptsyntax as rs  # noqa: F401  (kept; surrounding helpers import lazily)
    from core import config
    from core import env_collision
    from core import ik_collision_setup
    from core import robot_cell
    from core.rhino_bar_registry import get_bar_seq_map

    print(f"core.bar_action.build_bar_assembly_action: building bar '{bar_id}' ...")

    # 1) Read keyframe + base from the bar curve.
    payload = _read_bar_payload(bar_oid)
    if payload is None:
        raise RuntimeError(
            f"Bar '{bar_id}' is missing one of "
            f"'{config.KEY_ASSEMBLY_BASE_FRAME}'/'{config.KEY_ASSEMBLY_IK_APPROACH}'/"
            f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}'. Run RSIKKeyframe first."
        )
    base_frame_world_mm = payload["base_frame_world_mm"]

    # 2) Resolve per-arm tools on the bar.
    arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        raise RuntimeError(f"Bar '{bar_id}': {err}")

    # 3) Tool0 world transforms at IK_ASSEMBLED (= placed tool block instance xforms).
    tool0_left_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["left"])
    tool0_right_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["right"])

    # 4) Joint -> arm classifier (joint_id keyed).
    arm_to_male = _classify_male_joints_per_arm(bar_id)

    # 5) Build slim base state, then run upstream collision-state prep.
    slim_state = robot_cell.default_cell_state()
    _set_robot_base_frame(slim_state, base_frame_world_mm)
    _apply_groups_to_config(slim_state, payload["assembled"])
    template_state, env_geom = ik_collision_setup.prepare_assembly_collision_state(
        rcell, planner, slim_state, bar_id,
    )

    # 5b) Canonicalize names: identify which canonical keys belong to the
    # active bar (= came from active_* upstream) and which are present at
    # this bar's stage (built or active), then strip the prefixes off
    # template_state.rigid_body_states / env_geom for the export path.
    active_keys = {
        canonical_rb_name(k) for k in env_geom
        if k.startswith(ACTIVE_RB_BAR_PREFIX) or k.startswith(ACTIVE_RB_JOINT_PREFIX)
    }
    present_keys = {canonical_rb_name(k) for k in env_geom}

    # 5c) Register the FULL assembly (every bar + joint) onto the cached cell
    # so RSExportRobotCell writes a state-independent RobotCell.json. Then the
    # not-yet-built bodies (future bars/joints) are added to template_state as
    # hidden, so the state's workpiece key-set matches the full cell.
    full_geom_raw = _register_full_assembly_geom(rcell, planner)
    full_geom_canonical = {canonical_rb_name(k): v for k, v in full_geom_raw.items()}
    future_keys = set(full_geom_canonical.keys()) - present_keys

    canonicalize_state(template_state)
    env_geom = canonicalize_env_geom(env_geom)

    if future_keys:
        from compas_fab.robots import RigidBodyState
        # NB: use a distinct name here -- `payload` is the bar-keyframe dict
        # read above and is still needed below for M2/M3 (approach/assembled).
        for fk in sorted(future_keys):
            fk_geom = full_geom_canonical[fk]
            template_state.rigid_body_states[fk] = RigidBodyState(
                frame=_mm4_to_frame(fk_geom["frame_world_mm"]),
                attached_to_link=None,
                attached_to_tool=None,
                touch_links=[],
                touch_bodies=[],
                attachment_frame=None,
                is_hidden=True,
            )

    # 6) Recover arm tool RB names (mirror of robot_cell.ARM_TOOL_RB_NAMES,
    # filtered to those actually attached for THIS bar's tools).
    left_path, right_path = ik_collision_setup.resolve_tool_collision_paths(
        arm_tools["left"], arm_tools["right"],
    )
    arm_tool_rb_names = {
        "left": robot_cell.ARM_TOOL_RB_NAMES["left"] if left_path else None,
        "right": robot_cell.ARM_TOOL_RB_NAMES["right"] if right_path else None,
    }

    # 7) Build movements.
    seq_map = get_bar_seq_map()
    # Full ordered list of bar ids = assembly sequence (ascending seq number).
    assembly_seq = [
        bid for bid, _oid_seq in sorted(seq_map.items(), key=lambda kv: kv[1][1])
    ]
    active_index = assembly_seq.index(bar_id) if bar_id in assembly_seq else -1

    m1 = _build_m1(
        template_state, bar_id, rcell, env_geom, active_keys, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        config.HOME_CONFIG_LEFT, config.HOME_CONFIG_RIGHT,
        config.LEFT_GROUP, config.RIGHT_GROUP,
        config.LM_DISTANCE,
    )
    m2 = _build_m2(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        payload["approach"],
        config.LM_DISTANCE,
    )
    m3 = _build_m3(
        template_state, bar_id, env_geom, active_keys, arm_to_male, arm_tool_rb_names,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        payload["assembled"],
        config.LM_DISTANCE,
    )
    m4 = _build_m4(
        template_state, bar_id, rcell, env_geom, active_keys,
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
