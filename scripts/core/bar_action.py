"""BarAssemblyAction schema + builder.

A `BarAssemblyAction` is a downstream artifact describing one bar's full
dual-arm assembly cycle as five `Movement` records:

    M0  IndependentDualArmFreeMovement               free move to M1 start (live-planned)
    M1  EndEffectorConstrainedDualArmFreeMovement    bar loading -> approach (gripped bar)
    M2  EndEffectorConstrainedDualArmLinearMovement  linear mate (gripped bar -> mated)
    M3  IndependentDualArmLinearMovement             linear retreat (released, per-arm)
    M4  IndependentDualArmFreeMovement               free home (no grasp)

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
M1/M2, detach to world in M0/M3/M4).

Module is importable without Rhino; the `build_*` helpers import
`rhinoscriptsyntax` lazily.
"""

from __future__ import annotations

import json
import os
import re
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
    IndependentDualArmFreeMovement,
    EndEffectorConstrainedDualArmFreeMovement,
    EndEffectorConstrainedDualArmLinearMovement,
    IndependentDualArmLinearMovement,
    Action,
    BarAssemblyAction,
)

# Single home of the L/R tool-name suffix rule (Rhino-free).
from core.robotic_tool import arm_side_from_tool_name  # noqa: E402


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


def _retreat_tool0_target_mm(tool0_assembled_mm, joint_world_mm, retreat_distance_mm: float):
    """tool0 frame at retreat = assembled rotation, origin shifted by -joint_z * d.

    Distance is ``config.LM_RETREAT_DISTANCE`` -- a separate knob from the approach
    offset below, since the released arms pull clear further than they pushed in.
    """
    j = np.asarray(joint_world_mm, dtype=float)
    axis_world = _unit(-j[:3, 2])  # joint local -Z
    out = np.array(tool0_assembled_mm, dtype=float, copy=True)
    out[:3, 3] = out[:3, 3] + axis_world * float(retreat_distance_mm)
    return out, axis_world


def _compute_approach_targets_mm(tool0_left_assembled_mm, tool0_right_assembled_mm, approach_distance_mm: float):
    """Approach: both tool0 origins translated by -avg(tool_z) * approach_distance.

    Distance is ``config.LM_APPROACH_DISTANCE``. This is the single source of the
    approach offset; ``rs_ik_keyframe`` reads it back off ``M1.target_ee_frames``
    rather than recomputing it. Tool block local +Z points out of the flange toward
    the joint, so -Z is the retreat direction.
    """
    z_avg = (tool0_left_assembled_mm[:3, 2] + tool0_right_assembled_mm[:3, 2]) / 2.0
    approach_dir = _unit(-z_avg)
    offset = approach_dir * float(approach_distance_mm)
    left = np.array(tool0_left_assembled_mm, dtype=float, copy=True)
    right = np.array(tool0_right_assembled_mm, dtype=float, copy=True)
    left[:3, 3] = left[:3, 3] + offset
    right[:3, 3] = right[:3, 3] + offset
    return left, right


# ---------------------------------------------------------------------------
# State helpers
# ---------------------------------------------------------------------------


def _apply_groups_to_config(state, groups: dict) -> None:
    """Overwrite per-group joint values onto ``state.robot_configuration`` in place.

    When ``groups`` is falsy the start config is set to ``None`` (not left at the
    template's zero configuration). This is what lets the M2/M3 start states be
    built *before* IK has been solved: ``rs_ik_keyframe`` builds the movements with
    ``approach_groups``/``assembled_groups`` left as ``None``, and a ``None`` start
    config is the honest "unsolved" marker (matching M0/M1/M4, which also set
    ``None``) -- the IK / headless solver fills it in later, and any consumer can
    tell a solved bar from an unsolved one by whether this config is ``None``.
    Leaving the template's all-zero config here would masquerade as a real IK
    solution in the exported JSON. The export path (``build_bar_assembly_action``)
    passes the saved keyframe groups, so a solved bar's start config is stamped
    as before.

    Args:
        state (RobotCellState): start_state whose configuration is patched in place.
        groups (dict | None): ``{side: {"joint_names": [...], "joint_values": [...]}}``
            or ``None``/empty to null the start config (unsolved).

    Returns:
        None: mutates ``state``.
    """
    if not groups:
        state.robot_configuration = None
        return
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


def _configuration_from_groups(template_state, groups: dict):
    """Return a Configuration built from per-arm keyframe groups, or None.

    Like :func:`_apply_groups_to_config` but returns a fresh ``Configuration``
    (does not mutate a movement's start_state), so the same keyframe config can be
    stored as a movement's ``target_configuration`` and as the next movement's
    start config. Returns ``None`` when ``groups`` is falsy (an unsolved bar), so
    those fields stay ``None`` for the solver to fill later.

    Args:
        template_state (RobotCellState): a state whose ``robot_configuration`` is
            copied as the base (its arm joints are then overwritten).
        groups (dict | None): ``{side: {"joint_names": [...], "joint_values": [...]}}``.

    Returns:
        Configuration | None: the config with the arm joints set, or ``None``.
    """
    if not groups:
        return None
    cfg = template_state.robot_configuration.copy()
    for _side, grp in groups.items():
        for name, value in zip(grp["joint_names"], grp["joint_values"]):
            cfg[name] = float(value)
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
        side = arm_side_from_tool_name(tname)
        if side is not None:
            out[jid] = side
    return out


def _read_bar_keyframe(bar_oid):
    """Read the bar's saved IK keyframe data from its Rhino user-text.

    Args:
        bar_oid: Rhino object id of the bar curve.

    Returns:
        dict | None: ``{"base_frame_world_mm": np.ndarray(4, 4), "approach":
        {...}, "assembled": {...}, "retreat": {...} | None}``, or ``None`` if any of
        the three required ``KEY_ASSEMBLY_*`` records (base/approach/assembled) are
        missing/malformed. ``retreat`` (``KEY_ASSEMBLY_IK_RETREAT``) is optional --
        it is ``None`` for bars solved before retreat was saved.
    """
    import rhinoscriptsyntax as rs
    from core import config

    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    retreat_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT)
    if not base_raw or not approach_raw or not assembled_raw:
        return None
    try:
        base_mm = np.asarray(json.loads(base_raw), dtype=float)
        if base_mm.shape != (4, 4):
            return None
        approach = json.loads(approach_raw)
        assembled = json.loads(assembled_raw)
        retreat = json.loads(retreat_raw) if retreat_raw else None
    except (json.JSONDecodeError, ValueError):
        return None
    return {
        "base_frame_world_mm": base_mm,
        "approach": approach,
        "assembled": assembled,
        "retreat": retreat,
    }


def has_ik_keyframe(bar_oid) -> bool:
    """True if the bar carries a complete, well-formed IK keyframe in user-text.

    This is the single source of truth for "is this bar IK-solved": it is exactly
    the check ``build_bar_assembly_action`` uses to decide between the real-IK
    build and the placeholder build. Export summaries should call this (rather
    than re-reading the user-text keys themselves) so the reported "with IK / no
    IK" split can never drift from what the export actually did.

    Args:
        bar_oid: Rhino object id of the bar curve.

    Returns:
        bool: True when ``_read_bar_keyframe`` returns a usable keyframe (all three
        ``KEY_ASSEMBLY_*`` records present and parseable), else False.
    """
    return _read_bar_keyframe(bar_oid) is not None


_MOVEMENT_ROLE_RE = re.compile(r"_M([0-9])_")


def _movement_by_role(action, role: str):
    """Return the Movement in ``action`` whose id carries the ``_M<n>_`` tag.

    Args:
        action (BarAssemblyAction): the action to search.
        role (str): the wanted role, ``"M0"``..``"M4"``.

    Returns:
        Movement | None: the matching movement, or None.
    """
    for mv in action.movements:
        m = _MOVEMENT_ROLE_RE.search(getattr(mv, "movement_id", "") or "")
        if m and f"M{m.group(1)}" == role:
            return mv
    return None


def _frame_to_mm4(frame) -> np.ndarray:
    """Convert a compas ``Frame`` (meters) to a 4x4 numpy transform with mm origin.

    Inverse of :func:`_mm4_to_frame`. Kept local so this module needs no import of
    the tamp ``keyframe.ik_keyframe`` module (which pulls the robot stack) just to
    serialize a base.
    """
    matrix = np.eye(4, dtype=float)
    matrix[:3, 0] = np.asarray(frame.xaxis, dtype=float)
    matrix[:3, 1] = np.asarray(frame.yaxis, dtype=float)
    matrix[:3, 2] = np.asarray(frame.zaxis, dtype=float)
    matrix[:3, 3] = np.asarray(frame.point, dtype=float) * 1000.0  # m -> mm
    return matrix


def write_bar_keyframe_from_action(bar_oid, action, rcell) -> bool:
    """Sync a loaded BarAssemblyAction's condensed IK info onto the bar user-text.

    Writes the essential IK-related fields -- the robot base frame and the
    approach / assembled / retreat per-arm configs -- into the same
    ``KEY_ASSEMBLY_*`` user-text keys that RSIKKeyframe writes and the export reads.
    This keeps the condensed on-curve record in sync whenever a solved BarAction
    JSON is loaded (the reverse of the export, which reads user-text -> JSON). The
    per-keyframe configs come from the movements' start_states, matching the
    movement model (a movement's start config is the previous movement's goal):
        approach  = M2.start_state (M1's goal),
        assembled = M3.start_state (M2's goal),
        retreat   = M4.start_state (M3's goal).
    The base frame is taken from whichever movement carries one (M1 first).

    Args:
        bar_oid: Rhino object id of the bar curve.
        action (BarAssemblyAction): the loaded (solved) action.
        rcell (RobotCell): used to name the per-group joints.

    Returns:
        bool: True when a base frame + at least approach + assembled were written
        (a usable keyframe); False otherwise (user-text left unchanged).
    """
    import rhinoscriptsyntax as rs
    from core import config
    # The group-config reader lives with the solvers in the tamp submodule now.
    from husky_assembly_tamp.keyframe.dual_arm_ik import extract_group_config

    m1 = _movement_by_role(action, "M1")
    m2 = _movement_by_role(action, "M2")
    m3 = _movement_by_role(action, "M3")
    m4 = _movement_by_role(action, "M4")

    def _group_pair(mv):
        state = getattr(mv, "start_state", None) if mv is not None else None
        if state is None or state.robot_configuration is None:
            return None
        return {
            "left": extract_group_config(state, config.LEFT_GROUP, rcell),
            "right": extract_group_config(state, config.RIGHT_GROUP, rcell),
        }

    base_frame = None
    for mv in (m1, m2, m3, m4):
        state = getattr(mv, "start_state", None) if mv is not None else None
        if state is not None and getattr(state, "robot_base_frame", None) is not None:
            base_frame = state.robot_base_frame
            break

    approach = _group_pair(m2)
    assembled = _group_pair(m3)
    retreat = _group_pair(m4)

    if base_frame is None or approach is None or assembled is None:
        print(
            "core.bar_action.write_bar_keyframe_from_action: action has no usable "
            "base+approach+assembled; user-text left unchanged."
        )
        return False

    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME,
                   json.dumps(_frame_to_mm4(base_frame).tolist()))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH, json.dumps(approach))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED, json.dumps(assembled))
    if retreat is not None:
        rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT, json.dumps(retreat))
    print(
        "core.bar_action.write_bar_keyframe_from_action: synced base + approach + "
        f"assembled{' + retreat' if retreat else ''} to user-text."
    )
    return True


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
    # The bar tube rides on one arm's flange (bar_arm_side); the other arm grips a
    # male joint half. Resolve the tube's flange xform up front.
    bar_tool0 = tool0_left_assembled_mm if bar_arm_side == "left" else tool0_right_assembled_mm
    for key in active_keys:
        body_info = env_geom.get(key)
        if body_info is None:
            continue  # no cached geometry for this key -> nothing to attach
        # * The bar tube attaches to bar_arm_side's tool0.
        if key.startswith(CANONICAL_BAR_PREFIX):
            _attach_body_to_arm_tool0(state, body_info["frame_world_mm"], bar_tool0, bar_arm_side, key)
            continue
        # * Only the bar + its joint halves are grasped; skip anything else.
        if not key.startswith(CANONICAL_JOINT_PREFIX):
            continue
        # Parse the canonical joint key "joint_<jid>_<sub>" (sub = male|female);
        # skip anything that doesn't split cleanly.
        tag = key[len(CANONICAL_JOINT_PREFIX):]
        if "_" not in tag:
            continue
        jid, sub = tag.rsplit("_", 1)
        # A male half goes to the arm whose tool grips it (its classified arm,
        # falling back to bar_arm_side); a female half is rigidly bonded to the
        # bar, so it rides the bar's arm.
        if sub == "male":
            arm = arm_to_male.get(jid, bar_arm_side)
        else:
            arm = bar_arm_side  # females rigidly bonded to bar -> bar's gripper
        # Attach to that arm's assembled flange pose (stores tool0_from_body).
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

    - **M0** (bar not yet grasped): nothing -- same no-whitelist policy as M4.
      Every ``"M0"`` branch below falls through to the "else" cases.
    - **M1** (grasped, halves apart): ``male<->its arm tool``, ``male<->bar``,
      ``carried_female<->bar``.
    - **M2** (grasped, mating): M1 + ``male<->built_female`` (same joint_id, the
      mate on the already-built bar) + ``male<->female_bar`` (the built bar the
      mate female belongs to -- the male's screw tip stops ~2.5 mm from it, a
      false-positive collision on the coarse meshes).
    - **M3** (released, tool peeling off): ``male<->tool`` only -- everything else
      is detached/static, so compas_fab auto-skips it.
    - **M4** (gone): nothing.

    The male<->mate whitelist is recorded on the male / carried-female side (one
    side is enough). The bar tube additionally whitelists BOTH gripper tools while
    the arms are at the joints (M1-M3); cleared once the bar is released (M4).

    Note: the tool<->tube contact is allowed because the simplified (convex)
    collision meshes of the tool head and the tube overlap by ~1-3 mm even when
    the *visual* meshes have clearance -- a pipeline mesh-coarseness artefact, not
    real interference (confirmed via PyBullet getClosestPoints). Whitelisting
    avoids the false positive.

    Args:
        state (RobotCellState): movement start_state, mutated in place.
        movement (str): one of ``"M0"`` / ``"M1"`` / ``"M2"`` / ``"M3"`` / ``"M4"``.
        active_keys (set): canonical names of the grasped bar + its joint halves.
        env_geom (dict): ``{name: body_info}`` (used to confirm the mate
            ``joint_<jid>_female`` exists for M2, and to read that female's
            ``parent_bar_id`` so the male can whitelist the female's bar).
        arm_to_male (dict): ``{joint_id: 'left' | 'right'}`` for the grasped males.
        bar_key (str): canonical name of the grasped bar (``bar_<id>``).
        tool_ids (dict): ``{"left": tool_id, "right": tool_id}``.

    Returns:
        None: mutates ``state``.
    """
    # The three blocks below write per-body ``touch_bodies`` allow-lists -- the
    # only ACM this pipeline authors. compas_fab consumes them ONLY in its CC4
    # (attached body vs other rigid body) and CC5 (tool vs rigid body) checks.
    # They do NOT affect arm<->arm self-collision (that is CC1, driven by the
    # robot SRDF's disabled-collision pairs, and is untouched here).

    # (1) Each grasped MALE joint half: allow the bodies it is meant to be in
    # contact with for this movement (empty list => no allowed contact).
    for jid, arm in arm_to_male.items():
        male_rb = state.rigid_body_states.get(f"{CANONICAL_JOINT_PREFIX}{jid}_male")
        if male_rb is None:
            continue
        tool = tool_ids.get(arm)
        partners = []
        if movement in ("M1", "M2"):
            # Gripped: the male sits inside its arm's gripper (tool) and rides on
            # the bar tube, so both are expected contacts.
            if tool:
                partners.append(tool)
            partners.append(bar_key)
            if movement == "M2":
                # M2 is the mate: the male seats into the already-built female of
                # the SAME joint_id, so allow that contact too (when it exists).
                female_key = f"{CANONICAL_JOINT_PREFIX}{jid}_female"
                if female_key in env_geom:
                    partners.append(female_key)
                    # The male's screw tip ends up only ~2.5 mm away from the
                    # FEMALE BAR (the already-built bar that the mate female is
                    # part of). On the coarse convex collision meshes PyBullet
                    # reports that tiny gap as a collision, so also whitelist the
                    # male against the female's parent bar. Example: inserting
                    # bar_B9 with male joint_J35-9_male, whose mate
                    # joint_J35-9_female belongs to bar_B35 -> allow the male to
                    # touch bar_B35. M2 only (the male is not that close to the
                    # female bar in any other movement).
                    female_bar_id = env_geom[female_key].get("parent_bar_id")
                    if female_bar_id:
                        partners.append(f"{CANONICAL_BAR_PREFIX}{female_bar_id}")
        elif movement == "M3":
            # Released, tool peeling off the male: only male<->tool can still
            # touch; the bar/female are now static, so compas_fab auto-skips them.
            if tool:
                partners.append(tool)
        # M0/M4 (bar not / no longer held): partners stays empty -> no allow-list.
        male_rb.touch_bodies = sorted(set(partners))

    # (2) Each carried FEMALE joint half is rigidly bonded to the bar while it is
    # gripped, so allow female<->bar contact during M1/M2; clear once released.
    for key in active_keys:
        if not (key.startswith(CANONICAL_JOINT_PREFIX) and key.endswith("_female")):
            continue
        frb = state.rigid_body_states.get(key)
        if frb is not None:
            frb.touch_bodies = [bar_key] if movement in ("M1", "M2") else []

    # (3) The two gripper tools overlap the grasped tube by a few mm on the coarse
    # collision meshes (see note above). Whitelist tool<->bar while the arms are
    # at the joints (M1-M3); clear it once the bar is gone (M0/M4).
    bar_rb = state.rigid_body_states.get(bar_key)
    if bar_rb is not None:
        if movement in ("M1", "M2", "M3"):
            bar_rb.touch_bodies = sorted({t for t in tool_ids.values() if t})
        else:
            bar_rb.touch_bodies = []


def _build_m1(
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
    approach_distance_mm: float,
    bar_arm_side: str = "left",
) -> EndEffectorConstrainedDualArmFreeMovement:
    """Build M1: bar loading position -> approach, gripping the bar (constrained dual-arm).

    ``start_state.robot_configuration`` is left ``None`` on purpose: M1's start
    configuration is computed by its own downstream free-motion planner (seeded
    from M0's end at deploy time), not fixed here. Only the approach EE target is
    known at build time; the IK keyframe solver reaches it via random restarts,
    so it never depends on this (absent) start config.

    Args:
        template_state (RobotCellState): full static-cell template to clone.
        bar_id (str): the bar being assembled.
        env_geom (dict): ``{name: body_info}`` with world frames.
        active_keys (set): canonical names of the grasped bar + its joint halves.
        arm_to_male (dict): ``{joint_id: 'left' | 'right'}``.
        bar_key (str): canonical grasped-bar name (``bar_<id>``).
        tool_ids (dict): ``{"left": tool_id, "right": tool_id}``.
        tool0_left_assembled_mm, tool0_right_assembled_mm (ndarray): 4x4 mm
            flange poses at the assembled keyframe.
        base_frame_world_mm (ndarray): 4x4 mm robot base frame.
        approach_distance_mm (float): approach offset distance
            (``config.LM_APPROACH_DISTANCE``).
        bar_arm_side (str): arm the bar attaches to (default ``"left"``).

    Returns:
        EndEffectorConstrainedDualArmFreeMovement: the M1 movement (start config
        ``None`` -- planner-filled -- gripping the bar; target = approach EE frames).
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    # Start config is planner-computed (see docstring); leave it unset here.
    state.robot_configuration = None
    _set_active_attachments(
        state, active_keys, env_geom, arm_to_male,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        bar_arm_side=bar_arm_side,
    )
    _apply_movement_touch_policy(
        state, "M1", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )
    tool0_left_approach_mm, tool0_right_approach_mm = _compute_approach_targets_mm(
        tool0_left_assembled_mm, tool0_right_assembled_mm, approach_distance_mm,
    )
    return EndEffectorConstrainedDualArmFreeMovement(
        movement_id=f"{bar_id}_M1_CDFM_bar_loading_to_approach",
        tag="Bar loading position -> Approach (gripped bar, fixed relative EE)",
        start_state=state,
        target_ee_frames={
            "left": _mm4_to_frame(tool0_left_approach_mm),
            "right": _mm4_to_frame(tool0_right_approach_mm),
        },
        target_configuration=None,
        notes={
            "constraint": "fixed_relative_ee_transform",
            "approach_offset_mm": float(approach_distance_mm),
            "bar_arm_side": bar_arm_side,
            "start_config_is_none": True,
            "planner_fills": "start_state.robot_configuration",
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
    approach_distance_mm: float,
    bar_arm_side: str = "left",
) -> EndEffectorConstrainedDualArmLinearMovement:
    """Build M2: approach -> assembled (linear mate), still gripping the bar.

    Shared args are as in :func:`_build_m1`.

    Args:
        approach_groups (dict): per-arm approach-keyframe joint config
            (``{side: {"joint_names": [...], "joint_values": [...]}}``) written
            onto the start_state.

    Returns:
        EndEffectorConstrainedDualArmLinearMovement: the M2 movement (start_state
        at the approach config; target = assembled EE frames; mate touch
        whitelisted).
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
    return EndEffectorConstrainedDualArmLinearMovement(
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
            # Key name kept as-is for downstream consumers; the value is the
            # APPROACH distance -- M2 travels back over M1's offset.
            "lm_distance_mm": float(approach_distance_mm),
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
    retreat_distance_mm: float,
) -> IndependentDualArmLinearMovement:
    """Build M3: assembled -> retreated (per-arm linear), bar released.

    Shared args are as in :func:`_build_m1`.

    Args:
        assembled_groups (dict): per-arm assembled-keyframe joint config written
            onto the start_state.

    Returns:
        IndependentDualArmLinearMovement: the M3 movement (start_state at the
        assembled config with the bar/joints detached to their world poses;
        target = per-arm retreated EE frames; only male<->tool whitelisted).
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
            tool0_assembled, joint_body_info["frame_world_mm"], retreat_distance_mm,
        )
        if arm == "left":
            target_left_mm = target_mm
        else:
            target_right_mm = target_mm
        retreat_axes_world[arm] = [float(x) for x in axis_world]

    return IndependentDualArmLinearMovement(
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
            # Key name kept as-is for downstream consumers; the value is the
            # RETREAT distance (config.LM_RETREAT_DISTANCE), not M1/M2's approach.
            "lm_distance_mm": float(retreat_distance_mm),
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
) -> IndependentDualArmFreeMovement:
    """Build M4: retreated -> home (free joint-space motion), bar released.

    Shared args are as in :func:`_build_m1`.

    Returns:
        IndependentDualArmFreeMovement: the M4 movement.
        ``start_state.robot_configuration`` is ``None`` (the planner fills it
        from M3's end); the bar/joints are detached at their world poses;
        target = HOME configuration.
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
    return IndependentDualArmFreeMovement(
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


def _build_m0(
    template_state,
    bar_id: str,
    env_geom: dict,
    active_keys,
    arm_to_male: dict,
    bar_key: str,
    tool_ids: dict,
    base_frame_world_mm,
) -> IndependentDualArmFreeMovement:
    """Build M0: current pose -> M1 start (free joint-space motion, live-planned).

    M0 is the leading movement for live deployment: at runtime the robot's real
    starting configuration is unknown, so this movement carries the arms from
    wherever they are to the configuration M1 begins at. It is deliberately left
    UNPLANNED in the offline export -- the live monitor plans it and fills its
    ``trajectory`` -- so:

    - ``start_state.robot_configuration`` is ``None`` (the live monitor supplies
      the real current configuration).
    - ``target_ee_frames`` is ``None`` (no Cartesian goal, and the two arms move
      independently -- the bar is not yet gripped, so there is no fixed
      relative-flange constraint).
    - ``target_configuration`` is ``None`` here; it is filled in later with M1's
      start configuration once M1 has been planned (forward-propagation done by
      the planner / live monitor, mirroring how M4's start config is filled from
      M3's end).

    The collision context mirrors M4 (the "bar released" classification): the arm
    tools stay attached, the active bar + its joint halves are detached to their
    assembled world pose, and no allowed-touch pairs are whitelisted. The
    assembled world pose is only a placeholder for the bar's true pre-grasp pose;
    the live monitor overrides it when it plans this movement.

    Shared args are as in :func:`_build_m1` (M0 needs no ``rcell``/home values --
    it has no goal configuration at build time).

    Returns:
        IndependentDualArmFreeMovement: the M0 movement (unplanned placeholder;
        start config ``None``, goal config ``None`` until M1 is planned).
    """
    state = template_state.copy()
    _set_robot_base_frame(state, base_frame_world_mm)
    # Live monitor supplies the real current configuration at runtime.
    state.robot_configuration = None
    # Bar not yet grasped: same "released" classification as M4 (bodies detached
    # at their assembled world pose, which is a placeholder for the pre-grasp pose).
    _detach_active_to_assembled_world(state, active_keys, env_geom)
    # No grasp, arms independent -> no allowed-touch whitelist (falls through the
    # M1/M2/M3 branches, same as M4).
    _apply_movement_touch_policy(
        state, "M0", active_keys, env_geom, arm_to_male, bar_key, tool_ids,
    )
    return IndependentDualArmFreeMovement(
        movement_id=f"{bar_id}_M0_free_to_M1_start",
        tag="Current -> M1 start (free, live-planned)",
        start_state=state,
        target_ee_frames=None,
        target_configuration=None,
        notes={
            "unplanned_offline": True,
            "planner_fills": "trajectory",
            "goal_backfilled_from": "M1.start_state.robot_configuration",
            "bar_pose_is_placeholder": True,
        },
    )


# ---------------------------------------------------------------------------
# Movement builder (shared by the action factory and the IK keyframe tool)
# ---------------------------------------------------------------------------


def build_assembly_movements(
    rcell,
    planner,
    bar_id: str,
    base_frame_world_mm,
    tool0_left_assembled_mm,
    tool0_right_assembled_mm,
    approach_groups: dict = None,
    assembled_groups: dict = None,
    retreat_groups: dict = None,
    bar_arm_side: str = "left",
):
    """Build the five assembly movements (M0-M4) for ``bar_id``.

    This is the single place that turns the cached static cell + the two placed
    tool blocks into the M0-M4 ``start_state``/``target_ee_frames`` pairs. It is
    callable **before** IK has been solved: ``approach_groups``/``assembled_groups``
    are optional, so the start configs default to the template seed and the IK
    solver fills them in later. ``build_bar_assembly_action`` (export) passes the
    saved keyframe groups; ``rs_ik_keyframe`` (solver) passes ``None`` and reads the
    M1/M2/M3 ``start_state``s back out to solve against.

    Everything each movement needs is known here without IK:
      - tool0 at the assembled pose = the placed tool block world transforms,
      - the approach EE targets = a pure geometric offset of those (``_build_m1``),
      - the retreat EE targets = the male-joint OCF offset (``_build_m3``),
      - attachments / allowed-touch policy = cell geometry + arm classification.
    Only ``robot_configuration`` is movement-state data that IK supplies.

    Args:
        rcell (RobotCell): the cached static cell.
        planner (PyBulletPlanner): active planner (used by ``ensure_assembly_cell``).
        bar_id (str): id of the bar being assembled.
        base_frame_world_mm (ndarray): 4x4 mm robot base frame. Cosmetic for the
            solver (``solve_dual_arm_ik`` overrides it per attempt); authoritative
            for the export + viewer.
        tool0_left_assembled_mm, tool0_right_assembled_mm (ndarray): 4x4 mm flange
            poses at the assembled keyframe (the placed tool block xforms).
        approach_groups (dict | None): saved approach per-arm config -> M1.target +
            M2.start, or ``None`` (pre-IK) to leave them unset.
        assembled_groups (dict | None): saved assembled per-arm config -> M2.target +
            M3.start, or ``None`` (pre-IK).
        retreat_groups (dict | None): saved retreat per-arm config -> M3.target +
            M4.start, or ``None`` (pre-IK, or a bar solved before retreat was saved).
        bar_arm_side (str): arm the bar + carried females attach to (default "left").

    Returns:
        tuple: ``(movements, env_geom)`` where ``movements`` is
        ``{"M0": .., "M1": .., "M2": .., "M3": .., "M4": ..}`` and ``env_geom`` is
        the cached ``{name: body_info}`` collision-body dict for the active bar's
        bodies. ``M0`` is the unplanned live-deployment lead-in (see
        :func:`_build_m0`).
    """
    from core import config
    from core import ik_collision_setup
    from core import robot_cell

    # Build the full static-cell template state. `prepare_assembly_collision_state`
    # calls `ensure_assembly_cell` (registers the full canonical assembly + env
    # obstacles + arm ToolModels onto the cached cell) and returns a full-key-set
    # state: built bars/joints visible static, the active bar a static obstacle,
    # not-yet-built bars/joints `is_hidden=True`, the two arm tools attached. Every
    # Mi re-classes only the active (grasped) bodies.
    slim_state = robot_cell.default_cell_state()
    _set_robot_base_frame(slim_state, base_frame_world_mm)
    template_state, env_geom = ik_collision_setup.prepare_assembly_collision_state(
        rcell, planner, slim_state, bar_id,
    )
    arm_to_male = _classify_male_joints_per_arm(bar_id)

    # * The grasped (active) bodies = bar_<bar_id> + every joint half mounted on it.
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

    # M0 is the live-deployment lead-in (current pose -> M1 start). Built first so
    # the code reads in movement order; it forks the same template independently,
    # so the build order among M0-M4 does not matter (see module docstring).
    m0 = _build_m0(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        base_frame_world_mm,
    )
    # M1's start config is planner-computed (left None); it needs no HOME values
    # or planning-group names, unlike M4 which targets the fixed home pose.
    m1 = _build_m1(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        config.LM_APPROACH_DISTANCE,
        bar_arm_side=bar_arm_side,
    )
    m2 = _build_m2(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        approach_groups,
        config.LM_APPROACH_DISTANCE,
        bar_arm_side=bar_arm_side,
    )
    m3 = _build_m3(
        template_state, bar_id, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        base_frame_world_mm,
        assembled_groups,
        config.LM_RETREAT_DISTANCE,
    )
    # M4 returns the arms to the fixed dual-arm home pose. Use the real saved
    # home split (HUSKY_DUAL_ARM_HOME_CONF_12) rather than the zero
    # HOME_CONFIG_* placeholders, so the exported action + RSShowIK preview show
    # the actual home the user authored.
    m4 = _build_m4(
        template_state, bar_id, rcell, env_geom, active_keys, arm_to_male,
        bar_key, tool_ids,
        base_frame_world_mm,
        config.HOME_CONF_LEFT_6, config.HOME_CONF_RIGHT_6,
        config.LEFT_GROUP, config.RIGHT_GROUP,
    )

    # Keyframe-config chain: each Mi.target == Mi+1.start == the config between
    # them. _build_m2/_build_m3 already stamp approach -> M2.start and assembled ->
    # M3.start; here we fill the remaining half of every pair so the exported
    # action carries the full chain and matches the headless keyframe solver's
    # write-back (see ``accept_solved_movement``):
    #   approach  -> M1.target
    #   assembled -> M2.target
    #   retreat   -> M3.target + M4.start   (M4 then runs retreat -> home)
    # All are None (unset) for a pre-IK bar whose groups are None.
    approach_cfg = _configuration_from_groups(template_state, approach_groups)
    assembled_cfg = _configuration_from_groups(template_state, assembled_groups)
    retreat_cfg = _configuration_from_groups(template_state, retreat_groups)
    if approach_cfg is not None:
        m1.target_configuration = approach_cfg
    if assembled_cfg is not None:
        m2.target_configuration = assembled_cfg
    if retreat_cfg is not None:
        m3.target_configuration = retreat_cfg
        m4.start_state.robot_configuration = retreat_cfg.copy()

    return {"M0": m0, "M1": m1, "M2": m2, "M3": m3, "M4": m4}, env_geom


# ---------------------------------------------------------------------------
# Top-level factory
# ---------------------------------------------------------------------------


def build_bar_assembly_action(rcell, planner, bar_id: str, bar_oid, allow_missing_ik: bool = False):
    """Build a `BarAssemblyAction` for `bar_id`. Rhino-only call.

    `prepare_assembly_collision_state` reuses the cached static cell (the full
    canonical assembly + env obstacles + arm ToolModels, built by
    RSRebuildRobotCell) and returns a full-key-set template state: built bars
    visible static, the grasped bar a static obstacle, not-yet-built bars
    `is_hidden=True`, tools attached. Each of the five movements clones that
    template and re-classes only the grasped (active-bar) bodies + sets its own
    allowed-touch policy. No canonicalization, no snapshot/restore.

    When ``allow_missing_ik`` is True and the bar has no saved IK keyframe, the
    action is still built for the headless solver: the movement ``target_ee_frames``
    are pure geometry (from the placed tool blocks) so they are complete, but the
    robot base is a placeholder identity frame and the per-arm configs are left
    unset. The headless base sampler fills in a real base + configs later.

    Args:
        rcell (RobotCell): the cached static cell.
        planner (PyBulletPlanner): active planner.
        bar_id (str): id of the bar to assemble.
        bar_oid: Rhino object id of the bar curve (its ``KEY_ASSEMBLY_*``
            user-text supplies the base frame + approach/assembled keyframes).
        allow_missing_ik (bool): when True, build even if the bar has no IK
            keyframe user-text (placeholder base, no saved configs). When False
            (default), a missing keyframe raises.

    Returns:
        BarAssemblyAction: the five-movement (M0-M4) action for ``bar_id``.

    Raises:
        RuntimeError: if the bar is missing IK keyframe user-text (and
            ``allow_missing_ik`` is False) or its two arm tools can't be resolved.
    """
    import rhinoscriptsyntax as rs  # noqa: F401  (kept; surrounding helpers import lazily)
    from core import config
    from core import env_collision
    from core import ik_collision_setup
    from core.rhino_bar_registry import get_bar_seq_map
    from core.rhino_walkable_ground import get_bar_ground_ids

    print(f"core.bar_action.build_bar_assembly_action: building bar '{bar_id}' ...")

    # 1) Read keyframe + base from the bar curve. Without IK we either raise or
    #    (for the headless-solve export) fall back to a placeholder identity base
    #    and no saved configs -- the headless sampler supplies both later.
    bar_keyframe = _read_bar_keyframe(bar_oid)
    if bar_keyframe is None:
        if not allow_missing_ik:
            raise RuntimeError(
                f"Bar '{bar_id}' is missing one of "
                f"'{config.KEY_ASSEMBLY_BASE_FRAME}'/'{config.KEY_ASSEMBLY_IK_APPROACH}'/"
                f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}'. Run RSIKKeyframe first."
            )
        print(
            f"core.bar_action.build_bar_assembly_action: bar '{bar_id}' has no IK "
            "keyframe; building with placeholder base (headless solve will fill it in)."
        )
        base_frame_world_mm = np.eye(4, dtype=float)
        approach_groups = None
        assembled_groups = None
        retreat_groups = None
    else:
        base_frame_world_mm = bar_keyframe["base_frame_world_mm"]
        approach_groups = bar_keyframe["approach"]
        assembled_groups = bar_keyframe["assembled"]
        retreat_groups = bar_keyframe.get("retreat")

    # TODO the tools should not be separately resolved as objects per bar, but rather there should only be two ToolModel attached to the robot's tool0 links in the RobotCell, and in robot_cell_state we simply attach the bar in action to the left arm of the dual arm. (since in consrained dual arm planning, and the insertion dual arm linear motion, the constraints on two arms'EE is enf)
    # 2) Resolve per-arm tools on the bar.
    arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        raise RuntimeError(f"Bar '{bar_id}': {err}")

    # 3) Tool0 world transforms at IK_ASSEMBLED (= placed tool block instance xforms).
    tool0_left_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["left"])
    tool0_right_assembled_mm = env_collision._block_instance_xform_mm(arm_tools["right"])

    # 4) Build M0-M4 from the cell + tool placements + the saved keyframe configs
    #    (approach/assembled are None when the bar has no IK yet).
    movements, _env_geom = build_assembly_movements(
        rcell, planner, bar_id,
        base_frame_world_mm,
        tool0_left_assembled_mm, tool0_right_assembled_mm,
        approach_groups=approach_groups,
        assembled_groups=assembled_groups,
        retreat_groups=retreat_groups,
    )

    # 5) Assembly-sequence metadata for the action wrapper (the movements
    # themselves don't need it).
    seq_map = get_bar_seq_map()
    assembly_seq = [
        bid for bid, _oid_seq in sorted(seq_map.items(), key=lambda kv: kv[1][1])
    ]
    active_index = assembly_seq.index(bar_id) if bar_id in assembly_seq else -1

    return BarAssemblyAction(
        action_id=f"{bar_id}_A0_assemble",
        tag=f"Assemble bar {bar_id} (index {active_index} of {len(assembly_seq)})",
        movements=[
            movements["M0"], movements["M1"], movements["M2"],
            movements["M3"], movements["M4"],
        ],
        active_bar_id=bar_id,
        assembly_seq=assembly_seq,
        # Which WalkableGround surface(s) the headless base sampler may use for
        # this bar (set via RSAssignAndShowWalkableGround; empty if not assigned yet).
        walkable_ground_ids=get_bar_ground_ids(bar_oid),
    )
