"""Support-keyframe persistence, scene freezing, and release validation.

The support (holding) robot's keyframe lives on the HELD bar's curve as
split user-text keys (see ``core.config`` KEY_SUPPORT_*): which robot the
hold was solved for, the robot base frame, the manually picked grasp frame,
and the approach/held arm configurations. The reader here is all-or-nothing
— a bar with SOME of the keys is corrupt state and raises rather than being
quietly patched over.

Also home to the deferred release validation ("can the gripper actually
back out at release time?"), which runs at two checkpoints:

- checkpoint 1, right after a hold is solved (partial: overlapping holds
  that are not solved yet are skipped with a note);
- checkpoint 2, when the hold's LAST stabilizing bar gets its assembly
  keyframes accepted — every overlapping hold is solved by then, so the
  check is complete (and it re-runs at export as the final gate).

The BarHoldingAction / BarHoldingReleaseAction builders (Layer 6 of the
support-IK work) will live here too.
"""

from __future__ import annotations

import json

import numpy as np
import rhinoscriptsyntax as rs

from core import config
from core import env_collision
from core import robot_cell
from core import robot_cell_support
from core import robot_obstacles
# Frame conversion shared with the assembly builder (private on purpose —
# same package, one definition).
from core.bar_action import _mm4_to_frame
from core.hold_schedule import (
    bodies_built_after,
    derive_hold_plan,
    frozen_robot_leave_steps,
    holds_present_at_release,
    robots_holding_at_step,
)
from core.rhino_bar_registry import (
    collect_hold_inputs,
    get_real_bar_seq_map,
    get_supported_until,
)
from rs_data_structure.bar_action import (
    GripperToolMovement,
    SingleArmFreeMovement,
    SingleArmLinearMovement,
)
from rs_data_structure.hold_action import BarHoldingAction, BarHoldingReleaseAction


# ---------------------------------------------------------------------------
# * Assembly-keyframe reading (the dual-arm pose the hold is anchored to)
# ---------------------------------------------------------------------------


def load_assembly_payload(bar_oid) -> dict:
    """Read the split assembly keys into the flat left/right payload shape.

    Args:
        bar_oid: Rhino object id of the bar curve.

    Returns:
        dict: ``{"base_frame_world_mm": np.ndarray(4,4), "joint_values_left",
        "joint_values_right", "joint_names_left", "joint_names_right"}``.

    Raises:
        RuntimeError: when the assembly keyframe is missing — solve it first.
    """
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not assembled_raw:
        bar_id = rs.GetUserText(bar_oid, "bar_id") or str(bar_oid)
        raise RuntimeError(
            f"Bar {bar_id!r} has no solved assembly keyframe "
            f"({config.KEY_ASSEMBLY_BASE_FRAME} / {config.KEY_ASSEMBLY_IK_ASSEMBLED}). "
            "Run RSIKKeyframe's assembly flow on it first."
        )
    assembled = json.loads(assembled_raw)
    return {
        "base_frame_world_mm": np.asarray(json.loads(base_raw), dtype=float),
        "joint_values_left": assembled["left"]["joint_values"],
        "joint_values_right": assembled["right"]["joint_values"],
        "joint_names_left": assembled["left"]["joint_names"],
        "joint_names_right": assembled["right"]["joint_names"],
    }


# ---------------------------------------------------------------------------
# * Support-keyframe user-text IO (split keys, all-or-nothing)
# ---------------------------------------------------------------------------

# Every key that together forms ONE complete support keyframe.
_SUPPORT_KEYS = (
    "KEY_SUPPORT_ROBOT",
    "KEY_SUPPORT_BASE_FRAME",
    "KEY_SUPPORT_GRASP_FRAME",
    "KEY_SUPPORT_IK_APPROACH",
    "KEY_SUPPORT_IK_HELD",
)


def read_bar_support_keyframe(bar_oid):
    """All-or-nothing reader of the split support keys.

    Args:
        bar_oid: Rhino object id of the HELD bar's curve.

    Returns:
        dict | None: ``{"robot_name": str, "base_frame_world_mm":
        np.ndarray(4,4), "grasp_frame_world_mm": np.ndarray(4,4),
        "approach": {"joint_names", "joint_values"}, "held": {...}}``,
        or None when NO support keys are present at all.

    Raises:
        RuntimeError: when only SOME keys are present (corrupt state — the
            hold must be re-solved), naming the missing keys.
    """
    raw = {name: rs.GetUserText(bar_oid, getattr(config, name)) for name in _SUPPORT_KEYS}
    present = [name for name, value in raw.items() if value]
    if not present:
        return None
    missing = [getattr(config, name) for name, value in raw.items() if not value]
    if missing:
        bar_id = rs.GetUserText(bar_oid, "bar_id") or str(bar_oid)
        raise RuntimeError(
            f"Bar {bar_id!r} has a PARTIAL support keyframe (missing {missing}). "
            "Re-run RSIKKeyframe's support flow on it to write a complete one."
        )
    return {
        "robot_name": raw["KEY_SUPPORT_ROBOT"],
        "base_frame_world_mm": np.asarray(json.loads(raw["KEY_SUPPORT_BASE_FRAME"]), dtype=float),
        "grasp_frame_world_mm": np.asarray(json.loads(raw["KEY_SUPPORT_GRASP_FRAME"]), dtype=float),
        "approach": json.loads(raw["KEY_SUPPORT_IK_APPROACH"]),
        "held": json.loads(raw["KEY_SUPPORT_IK_HELD"]),
    }


def has_support_keyframe(bar_oid) -> bool:
    """True when a COMPLETE support keyframe is stored (raises on partial)."""
    return read_bar_support_keyframe(bar_oid) is not None


def write_support_grasp_frame(bar_oid, grasp_frame_world_mm):
    """Persist just the picked grasp frame (written ASAP, before solving)."""
    rs.SetUserText(
        bar_oid,
        config.KEY_SUPPORT_GRASP_FRAME,
        json.dumps(np.asarray(grasp_frame_world_mm, dtype=float).tolist()),
    )


def write_support_base_frame(bar_oid, base_frame_world_mm):
    """Persist just the support base frame (written ASAP, before solving)."""
    rs.SetUserText(
        bar_oid,
        config.KEY_SUPPORT_BASE_FRAME,
        json.dumps(np.asarray(base_frame_world_mm, dtype=float).tolist()),
    )


def write_bar_support_keyframe(
    bar_oid,
    robot_name: str,
    base_frame_world_mm,
    grasp_frame_world_mm,
    approach_cfg: dict,
    held_cfg: dict,
):
    """Write the COMPLETE split support keyframe onto the held bar.

    Args:
        bar_oid: Rhino object id of the held bar's curve.
        robot_name (str): "Alice" or "Belle" — whose URDF solved this.
        base_frame_world_mm: 4x4 base frame, world mm.
        grasp_frame_world_mm: 4x4 picked grasp frame, world mm.
        approach_cfg (dict): ``{"joint_names", "joint_values"}``.
        held_cfg (dict): same shape, at the grasp pose.
    """
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_ROBOT, robot_name)
    write_support_base_frame(bar_oid, base_frame_world_mm)
    write_support_grasp_frame(bar_oid, grasp_frame_world_mm)
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_IK_APPROACH, json.dumps(approach_cfg))
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_IK_HELD, json.dumps(held_cfg))


# ---------------------------------------------------------------------------
# * Geometry helpers
# ---------------------------------------------------------------------------


def approach_tool0_from_grasp(tool0_world_mm) -> np.ndarray:
    """The approach flange pose: the grasp tool0 backed off along its own -Z.

    The hold's linear approach starts here and the release's retreat ends
    here (same line, reversed). Distance = ``config.SUPPORT_LM_DISTANCE_MM``.
    """
    out = np.array(tool0_world_mm, dtype=float, copy=True)
    out[:3, 3] -= float(config.SUPPORT_LM_DISTANCE_MM) * out[:3, 2]
    return out


# ---------------------------------------------------------------------------
# * Scene states (register the env once; hide/show per scene)
# ---------------------------------------------------------------------------


def get_env_union(bar_map=None):
    """Every bar + joint + static body (the register-once superset).

    Registered into a support cell ONCE per session; each scene state then
    only flips per-body visibility (``is_hidden``) -- mirroring how the
    assembly cell handles not-yet-built bars, with no cell re-uploads when
    switching between hold-time and release-time scenes. The names are the
    same as in Cindy's cell (``bar_*`` / ``joint_*``), and the static bodies
    (environment obstacles, floor slabs) are the very objects Cindy's cell
    uses (``env_collision.collect_static_scene_geometry``).

    Args:
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.

    Returns:
        dict: ``{name: payload}`` for ALL bars/joints + every static body.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    union = dict(env_collision.collect_assembly_geometry(bar_map))
    union.update(env_collision.collect_static_scene_geometry())
    return union


def ensure_support_env_registered(sr_cell, planner, env_union):
    """Mirror the env union into a support cell; re-push it if anything changed.

    Args:
        sr_cell (RobotCell): the support robot's cell.
        planner: that robot's planner, or None when no PyBullet push is
            needed (pure export builds).
        env_union (dict): a ``get_env_union`` result.
    """
    deps = robot_cell.import_compas_stack()
    if env_collision.register_env_in_robot_cell(sr_cell, env_union, deps=deps):
        if planner is not None:
            planner.set_robot_cell(sr_cell)
        else:
            # Changed with no planner at hand (export): push it on next use.
            robot_cell_support.mark_cell_needs_push(sr_cell)


def collect_hold_window_geometry(held_bar_id: str, hold_plan: dict, bar_map=None, env_union=None):
    """The built geometry a hold must clear: the RELEASE-time scene.

    A holding pose is not judged against the world at grasp time. It has to
    survive the WHOLE hold window, and the binding moment is its end: by then
    every stabilizing bar (and every joint on them) has been installed around
    the frozen arm. Solving against the grasp-time scene would happily accept a
    pose sitting exactly where a later stabilizing bar must go -- the hold would
    look fine when made and then block the very bars it exists to enable.

    Bars only ACCUMULATE during the window, so this release-time set is a
    superset of every intermediate moment: one scene covers the whole window.
    The held bar itself IS in the scene (it is already built when the support
    robot arrives): the gripper's contact with it is allowed per movement
    instead (:func:`set_gripper_bar_contact`), so the free approach is still
    checked against it.

    Args:
        held_bar_id (str): the bar being held.
        hold_plan (dict): a ``derive_hold_plan`` result (must contain the bar).
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.
        env_union (dict): a ``get_env_union`` result to filter; collected when omitted.

    Returns:
        dict: ``{name: payload}`` for the release-time built bars + joints.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    entry = hold_plan[held_bar_id]
    return env_collision.collect_built_geometry(
        entry["release_after_bar_id"],
        bar_map,
        include_active=True,      # the last stabilizing bar IS built by then
        all_geom=env_union,
    )


def build_support_scene_state(robot_name: str, env_union: dict, visible_geom: dict):
    """A support state with the whole env placed, and only one scene visible.

    Every env body is stamped at its world frame, then every bar / joint NOT
    in ``visible_geom`` is hidden (not built yet at this scene's moment).
    Static bodies (obstacles, floors) are always shown. Robots default to
    parked; the caller freezes the ones actually present.

    Args:
        robot_name (str): whose cell/state ("Alice"/"Belle").
        env_union (dict): a ``get_env_union`` result (must already be
            registered into the cell).
        visible_geom (dict): the bars + joints shown in THIS scene (e.g. from
            ``collect_built_geometry`` at the scene's step).

    Returns:
        RobotCellState: the scene state.
    """
    state = robot_cell_support.default_support_cell_state(robot_name)
    state = env_collision.build_env_state(state, env_union)
    for name, body_info in env_union.items():
        if body_info.get("kind") in env_collision.STATIC_KINDS:
            continue
        if name not in visible_geom:
            state.rigid_body_states[name].is_hidden = True
    return state


def set_gripper_bar_contact(state, held_bar_id: str, allowed: bool) -> None:
    """Allow (or forbid) the support gripper to touch the held bar.

    The gripper closes around the bar, so the contact is real from the linear
    approach onto the grasp until the retreat away from it -- but NOT during
    the free move to the approach start, which must clear the bar.

    Args:
        state (RobotCellState): a support scene state (mutated in place).
        held_bar_id (str): the held bar.
        allowed (bool): True from the linear approach on, False before.
    """
    robot_obstacles.set_tool_contact(
        state, config.SUPPORT_TOOL_NAME, [env_collision.bar_body_name(held_bar_id)], allowed=allowed,
    )


def _held_bar_bodies(env_union: dict, held_bar_id: str) -> list:
    """The held bar, every joint half on it, and the mate female of each male on it.

    These are the bodies the assembly robot's tools are wrapped around (or
    seated against) while she still grips the bar.

    Args:
        env_union (dict): a ``get_env_union`` result.
        held_bar_id (str): the held bar.

    Returns:
        list: sorted rigid-body names.
    """
    out = set()
    for name, body_info in env_union.items():
        if body_info.get("parent_bar_id") != held_bar_id:
            continue
        out.add(name)
        is_male = str(body_info.get("subtype", "")).lower() == "male"
        if body_info.get("kind") == env_collision.KIND_JOINT and is_male:
            joint_id = name[len(env_collision.CANONICAL_JOINT_PREFIX):].rsplit("_", 1)[0]
            mate = env_collision.joint_body_name(joint_id, "female")
            if mate in env_union:
                out.add(mate)
    return sorted(out)


def build_hold_scene_state(robot_name: str, held_bar_id: str, hold_plan: dict, env_union: dict,
                           bar_map: dict, *, skip_unsolved: bool = False) -> tuple:
    """The scene a support robot's hold is solved and exported against.

    One builder for the interactive solve (RSIKKeyframe), the batch re-solve
    and the ``__H`` export, so all three agree:

    - bars + joints: the RELEASE-time set, held bar included
      (:func:`collect_hold_window_geometry`); static bodies always;
    - the assembly robot frozen at her assembled pose for the held bar (she
      still grips it while the support robot approaches);
    - every other support robot holding at the grasp step frozen at its held
      pose; robots not in the scene parked.

    Allowed contacts on top:

    - the assembly robot may touch the held bar, the joints on it and the mate
      females of its males (her tools are wrapped around / seated against them);
    - a frozen robot may touch the bars + joints built AFTER it leaves (the
      assembly robot after the held bar's step, another holder after its own
      release): those never stand next to it, but the release-time scene
      shows them while it stands at its grasp-time pose.

    The support gripper's own contact with the held bar is NOT set here: it
    depends on the movement (:func:`set_gripper_bar_contact`).

    Args:
        robot_name (str): the holding robot ("Alice"/"Belle").
        held_bar_id (str): the held bar.
        hold_plan (dict): a ``derive_hold_plan`` result (must contain the bar).
        env_union (dict): a ``get_env_union`` result (already registered).
        bar_map (dict): a ``get_bar_seq_map`` result.
        skip_unsolved (bool): True = an overlapping hold without a solved
            keyframe is skipped with a note instead of raising.

    Returns:
        tuple: ``(state, skipped)`` -- the scene state and the skipped
        ``(robot_name, held_bar_id)`` notes (``skip_unsolved`` only).

    Raises:
        RuntimeError: missing assembly keyframe, an unsolved overlapping hold
            (strict mode), or a robot-assignment mismatch.
    """
    entry = hold_plan[held_bar_id]
    visible = collect_hold_window_geometry(held_bar_id, hold_plan, bar_map=bar_map, env_union=env_union)
    state = build_support_scene_state(robot_name, env_union, visible)

    # * The assembly robot, still gripping the held bar.
    assembled = load_assembly_payload(bar_map[held_bar_id][0])
    robot_obstacles.configure_robot_obstacle(
        state,
        config.ASSEMBLY_ROBOT_NAME,
        assembled["base_frame_world_mm"],
        list(assembled["joint_values_left"]) + list(assembled["joint_values_right"]),
        list(assembled["joint_names_left"]) + list(assembled["joint_names_right"]),
    )
    # * Other holders present at the grasp step.
    skipped = freeze_holding_robots(
        state, hold_plan, entry["hold_start_seq"],
        exclude_robot=robot_name, skip_unsolved=skip_unsolved, bar_map=bar_map,
    )

    # * Allowed contacts: the assembly robot is wrapped around the held bar.
    assembly_tool = config.OBSTACLE_TOOL_NAMES[config.ASSEMBLY_ROBOT_NAME]
    robot_obstacles.set_tool_contact(state, assembly_tool, _held_bar_bodies(env_union, held_bar_id))

    # * Allowed contacts: bars built after a frozen robot has left.
    body_steps = {
        name: bar_map[body_info["parent_bar_id"]][1]
        for name, body_info in visible.items()
        if body_info.get("parent_bar_id") in bar_map
    }
    skipped_robots = {robot for robot, _bar in skipped}
    leave_steps = frozen_robot_leave_steps(hold_plan, held_bar_id, config.ASSEMBLY_ROBOT_NAME)
    for frozen_robot, leave_step in leave_steps.items():
        if frozen_robot in skipped_robots:
            continue  # not in the scene (its hold is unsolved)
        robot_obstacles.set_tool_contact(
            state, config.OBSTACLE_TOOL_NAMES[frozen_robot], bodies_built_after(body_steps, leave_step),
        )
    return state, skipped


def solve_hold_pair(planner, scene_state, held_bar_id: str, base_frame_world_mm, tool0_grasp_mm, *,
                    check_collision: bool = True, verbose_pairs: bool = False) -> tuple:
    """Solve the held pose, then the approach pose seeded from it.

    The held pose may have the gripper touching the held bar (it closes around
    it); the approach pose may not (the free move to it must clear the bar).
    The approach is seeded from the held configuration, so both sit on the
    same IK branch and the linear approach between them stays short.

    Args:
        planner: the support robot's planner (its cell pushed).
        scene_state (RobotCellState): a :func:`build_hold_scene_state` state.
        held_bar_id (str): the held bar.
        base_frame_world_mm (np.ndarray): 4x4 robot base pose, world mm.
        tool0_grasp_mm (np.ndarray): 4x4 flange pose at the grasp, world mm.
        check_collision (bool): include collision checking in the IK.
        verbose_pairs (bool): print the collision-pair summary of the held solve.

    Returns:
        tuple: ``(held_state, approach_state)``; either is None when its IK failed
        (the approach is not tried when the held pose fails).
    """
    held_template = scene_state.copy()
    set_gripper_bar_contact(held_template, held_bar_id, allowed=True)
    held_state = robot_cell_support.solve_support_ik(
        planner, held_template, base_frame_world_mm, tool0_grasp_mm,
        check_collision=check_collision, verbose_pairs=verbose_pairs,
    )
    if held_state is None:
        return None, None
    approach_seed = approach_seed_state(scene_state, held_bar_id, held_state)
    approach_state = robot_cell_support.solve_support_ik(
        planner, approach_seed, base_frame_world_mm, approach_tool0_from_grasp(tool0_grasp_mm),
        check_collision=check_collision,
    )
    return held_state, approach_state


def approach_seed_state(scene_state, held_bar_id: str, held_state):
    """The scene for the approach solve: no gripper contact, held config as the seed.

    Args:
        scene_state (RobotCellState): a :func:`build_hold_scene_state` state.
        held_bar_id (str): the held bar.
        held_state (RobotCellState): the solved held state (its configuration
            seeds the approach IK).

    Returns:
        RobotCellState: a new state.
    """
    seed = scene_state.copy()
    set_gripper_bar_contact(seed, held_bar_id, allowed=False)
    seed.robot_configuration = held_state.robot_configuration.copy()
    return seed


# ---------------------------------------------------------------------------
# * Scene freezing (which robots stand where in a step's planning scene)
# ---------------------------------------------------------------------------


def freeze_holding_robots(state, hold_plan, at_seq, *, exclude_robot=None,
                          skip_unsolved=False, bar_map=None):
    """Freeze every support robot holding during step ``at_seq`` into ``state``.

    Robots not holding stay PARKED (the default of every base state). Frozen
    poses come from each held bar's stored support keyframe.

    Args:
        state (RobotCellState): the state to stamp (mutated in place).
        hold_plan (dict): a ``derive_hold_plan`` result.
        at_seq (int): the step being planned.
        exclude_robot (str): a robot to leave alone (the acting robot itself).
        skip_unsolved (bool): True = collect unsolved holds as notes instead
            of raising (checkpoint-1 partial mode).
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.

    Returns:
        list: ``(robot_name, held_bar_id)`` tuples that were SKIPPED because
        their hold has no solved keyframe yet (empty unless skip_unsolved).

    Raises:
        RuntimeError: unsolved overlapping hold (when not skipping), or a
            robot-assignment mismatch (sequence edited since solving).
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    skipped = []
    for robot_name, held_bar_id in robots_holding_at_step(hold_plan, at_seq).items():
        if robot_name == exclude_robot:
            continue
        held_oid = bar_map[held_bar_id][0]
        payload = read_bar_support_keyframe(held_oid)
        if payload is None:
            if skip_unsolved:
                skipped.append((robot_name, held_bar_id))
                continue
            raise RuntimeError(
                f"Support robot {robot_name} holds bar {held_bar_id} during step "
                f"{at_seq}, but that hold has NO solved support keyframe. Run "
                f"RSIKKeyframe's support flow on bar {held_bar_id} first."
            )
        if payload["robot_name"] != robot_name:
            raise RuntimeError(
                f"Bar {held_bar_id}'s support keyframe was solved for "
                f"{payload['robot_name']} but the current sequence assigns "
                f"{robot_name}. The sequence was edited since solving — re-run "
                f"RSIKKeyframe's support flow on bar {held_bar_id}."
            )
        robot_obstacles.configure_robot_obstacle(
            state,
            robot_name,
            payload["base_frame_world_mm"],
            payload["held"]["joint_values"],
            payload["held"]["joint_names"],
        )
        # The frozen gripper is physically clamped around its held bar -- allow
        # that one contact, or every IK candidate in this scene is vetoed by a
        # collision no arm configuration can change.
        robot_obstacles.whitelist_frozen_contact(state, robot_name, [held_bar_id])
    return skipped


# ik_viz sub-layer prefix for holding robots drawn as context in the ASSEMBLY
# flow (mirror of support_grasp_pick.DUAL_ARM_CONTEXT_LAYER_KEY, which draws
# frozen Cindy as context in the SUPPORT flow). One bundle per robot, so
# showing/hiding them never touches the assembly robot's own preview.
HOLDER_CONTEXT_LAYER_PREFIX = "AssemblyHolder"


def holder_context_layer_key(robot_name: str) -> str:
    """The ik_viz sub-layer key one holding robot's context preview draws on."""
    return f"{HOLDER_CONTEXT_LAYER_PREFIX} {robot_name}"


def show_frozen_holders_context(hold_plan: dict, at_seq: int, mesh_mode, *,
                                bar_map=None, exclude_robot=None) -> list:
    """Draw every support robot holding during step ``at_seq`` at its held pose.

    The VIZ twin of :func:`freeze_holding_robots`: that one puts the holding
    robots into the collision scene, this one puts the same poses on screen, so
    picking an assembly base frame is done against the scene the IK will
    actually be solved in (a frozen arm can be the reason a base fails, and
    without this it is invisible).

    A hold with no solved keyframe cannot be drawn (there is no pose yet); it is
    reported rather than skipped silently, since the collision scene leaves it
    out too and the user should know a robot is missing from the picture.

    Args:
        hold_plan (dict): a ``derive_hold_plan`` result.
        at_seq (int): the step being planned (the bar about to be assembled).
        mesh_mode: ik_viz mesh mode ("visual" / "collision") to draw with.
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.
        exclude_robot (str): a robot to leave undrawn (the acting robot).

    Returns:
        list: the ik_viz layer keys drawn, for :func:`hide_frozen_holders_context`.
    """
    # Lazy import: ik_viz is Rhino-display machinery, and this module is also
    # imported by the (display-free) export path.
    from core import ik_viz

    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    drawn = []
    for robot_name, held_bar_id in robots_holding_at_step(hold_plan, at_seq).items():
        if robot_name == exclude_robot:
            continue
        payload = read_bar_support_keyframe(bar_map[held_bar_id][0])
        if payload is None:
            print(
                f"core.hold_action_builder: NOTE - {robot_name} holds bar "
                f"{held_bar_id} during this step but that hold is UNSOLVED, so it "
                "is drawn nowhere and collides with nothing. Solve it "
                "(RSIKKeyframe on that bar) to see it in the scene."
            )
            continue
        sr_cell = robot_cell_support.get_or_load_support_cell(robot_name)
        state = robot_cell_support.default_support_cell_state(robot_name)
        state.robot_base_frame = _mm4_to_frame(payload["base_frame_world_mm"])
        held = payload["held"]
        for name, value in zip(held["joint_names"], held["joint_values"]):
            state.robot_configuration[name] = float(value)

        layer_key = holder_context_layer_key(robot_name)
        ik_viz.update_state(
            state, robot_cell=sr_cell, mesh_mode=mesh_mode, layer_key=layer_key,
        )
        drawn.append(layer_key)
        print(
            f"core.hold_action_builder: {robot_name} drawn holding bar "
            f"{held_bar_id} (frozen in this step's collision scene)."
        )
    return drawn


def hide_frozen_holders_context(layer_keys) -> None:
    """Hide the context previews drawn by :func:`show_frozen_holders_context`.

    Args:
        layer_keys (list): the keys that function returned.
    """
    from core import ik_viz

    for layer_key in layer_keys or []:
        ik_viz.set_layer_visible(layer_key, False)


def build_release_scene_state(
    robot_name: str,
    held_bar_id: str,
    hold_plan: dict,
    env_union: dict,
    *,
    partial: bool,
    bar_map=None,
):
    """The RELEASE-time scene for one hold, as a support cell state.

    Release-time scene: every bar with seq <= the release step is built
    (INCLUDING the last stabilizing bar and the held bar itself); the assembly
    robot stays parked far away (she has finished and driven off); other
    support robots still holding at that moment are frozen at their held
    poses (:func:`hold_schedule.holds_present_at_release`: a hold STARTING at
    the release step is present, one released EARLIER in the same step is
    not). The support gripper's contact with the held bar is set per use
    (:func:`set_gripper_bar_contact`).

    Args:
        robot_name (str): the holding robot the scene is for.
        held_bar_id (str): the held bar being released.
        hold_plan (dict): a ``derive_hold_plan`` result (must contain the bar).
        env_union (dict): a ``get_env_union`` result (already registered).
        partial (bool): True = overlapping holds without a solved keyframe
            are skipped with a note instead of raising (checkpoint-1 mode).
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.

    Returns:
        tuple: ``(state, skipped)`` — the scene state and the list of
        skipped ``(robot_name, held_bar_id)`` notes (partial mode only).

    Raises:
        RuntimeError: in full mode, on an unsolved overlapping hold.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()

    release_geom = collect_hold_window_geometry(
        held_bar_id, hold_plan, bar_map=bar_map, env_union=env_union,
    )
    state = build_support_scene_state(robot_name, env_union, release_geom)

    skipped = []
    for other_bar_id in holds_present_at_release(hold_plan, held_bar_id):
        other = hold_plan[other_bar_id]
        other_payload = read_bar_support_keyframe(bar_map[other_bar_id][0])
        if other_payload is None:
            if partial:
                skipped.append((other["robot_name"], other_bar_id))
                continue
            raise RuntimeError(
                f"Release check for bar {held_bar_id}: overlapping hold on bar "
                f"{other_bar_id} ({other['robot_name']}) has no solved support "
                f"keyframe yet. Solve it, then re-run the check."
            )
        robot_obstacles.configure_robot_obstacle(
            state,
            other["robot_name"],
            other_payload["base_frame_world_mm"],
            other_payload["held"]["joint_values"],
            other_payload["held"]["joint_names"],
        )
        # Same rule as freeze_holding_robots: the other robot's gripper is
        # clamped around ITS held bar (present in this release scene), so that
        # one frozen contact must be allowed.
        robot_obstacles.whitelist_frozen_contact(
            state, other["robot_name"], [other_bar_id]
        )
    return state, skipped


# ---------------------------------------------------------------------------
# * Deferred release validation (checkpoints 1 + 2)
# ---------------------------------------------------------------------------


def validate_release_confs(
    planner,
    sr_cell,
    robot_name: str,
    held_bar_id: str,
    hold_plan: dict,
    base_frame_world_mm,
    approach_cfg: dict,
    held_cfg: dict,
    *,
    partial: bool,
    bar_map=None,
):
    """Check the held + approach configurations against the RELEASE-time scene.

    Release-time scene (:func:`build_release_scene_state`): every bar with
    seq <= the release step is built, the held bar included; the assembly
    robot is parked far away; other support robots still holding at that
    moment are frozen at their held poses. The gripper may touch the held bar
    at the held configuration (it is wrapped around it) but not at the
    approach / retreat configuration.

    Args:
        planner: the support robot's own planner (cell already pushed).
        sr_cell (RobotCell): that robot's cell.
        robot_name (str): the holding robot being validated.
        held_bar_id (str): the held bar.
        hold_plan (dict): a ``derive_hold_plan`` result (must contain the bar).
        base_frame_world_mm: the hold's base frame (4x4 mm).
        approach_cfg (dict): ``{"joint_names", "joint_values"}`` retreat target.
        held_cfg (dict): same shape, at the grasp.
        partial (bool): checkpoint-1 mode — overlapping holds without a
            solved keyframe are skipped with a note instead of raising.
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.

    Returns:
        list: skipped ``(robot_name, held_bar_id)`` notes (partial mode only).

    Raises:
        RuntimeError: on a collision at either configuration (message names
            the held bar and says to re-pick the grasp), or — in full mode —
            on an unsolved overlapping hold.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    env_union = get_env_union(bar_map)
    ensure_support_env_registered(sr_cell, planner, env_union)
    state, skipped = build_release_scene_state(
        robot_name, held_bar_id, hold_plan, env_union,
        partial=partial, bar_map=bar_map,
    )

    # Check both configurations at the hold's base.
    problems = []
    for label, cfg, gripper_on_bar in (
        ("held", held_cfg, True),
        ("approach/retreat", approach_cfg, False),
    ):
        check_state = state.copy()
        set_gripper_bar_contact(check_state, held_bar_id, allowed=gripper_on_bar)
        robot_cell_support._apply_base_frame_mm(
            check_state, np.asarray(base_frame_world_mm, dtype=float)
        )
        for name, value in zip(cfg["joint_names"], cfg["joint_values"]):
            check_state.robot_configuration[name] = float(value)
        try:
            planner.check_collision(check_state, options={"full_report": True})
        except Exception as exc:
            problems.append(f"{label} pose collides: {exc}")
    if problems:
        raise RuntimeError(
            f"Release-time check FAILED for held bar {held_bar_id} "
            f"({robot_name}): " + " | ".join(problems) +
            f" — re-pick the grasp for bar {held_bar_id} (RSIKKeyframe support flow)."
        )
    return skipped


def resolve_support_keyframe_noninteractive(
    bar_id: str,
    bar_oid,
    hold_plan: dict,
    bar_map=None,
    env_union=None,
    check_collision: bool = True,
):
    """Re-solve one held bar's support keyframe from its STORED picks.

    Used by the batch command (RSIKKeyframeAll): the manually picked grasp +
    base frames already live on the bar's user-text, so the solve can re-run
    without any UI — ONE attempt at the stored base (no sampling; a failure
    means the user should re-pick interactively). Writes the complete split
    keys on success. Never auto-picks a grasp — grasps are hand-picked.

    Solved against the hold scene (:func:`build_hold_scene_state`, the same
    one the interactive solve and the ``__H`` export use), so the pose must
    clear every stabilizing bar that will be installed before the hold ends.

    Args:
        bar_id (str): the held bar.
        bar_oid: its Rhino object id.
        hold_plan (dict): a ``derive_hold_plan`` result containing the bar.
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.
        env_union (dict): a ``get_env_union`` result; fetched when omitted.
        check_collision (bool): include collision checking in the IK.

    Returns:
        list: checkpoint-1 skipped ``(robot_name, held_bar_id)`` notes.

    Raises:
        RuntimeError: missing picks/assembly keyframe, IK failure at the
            stored base, or a failed release check.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    if env_union is None:
        env_union = get_env_union(bar_map)
    entry = hold_plan[bar_id]
    robot_name = entry["robot_name"]

    grasp_raw = rs.GetUserText(bar_oid, config.KEY_SUPPORT_GRASP_FRAME)
    base_raw = rs.GetUserText(bar_oid, config.KEY_SUPPORT_BASE_FRAME)
    if not grasp_raw or not base_raw:
        raise RuntimeError(
            f"Bar {bar_id!r} has no stored grasp/base picks — run RSIKKeyframe's "
            "support flow interactively first."
        )
    grasp_mm = np.asarray(json.loads(grasp_raw), dtype=float)
    base_mm = np.asarray(json.loads(base_raw), dtype=float)

    _client, planner, sr_cell = robot_cell_support.ensure_support_cell_pushed(robot_name)
    ensure_support_env_registered(sr_cell, planner, env_union)
    # Strict: overlapping earlier holds must be solved. The batch walks holds
    # in hold-start order, so they are — a miss here is a real problem.
    template, _skipped = build_hold_scene_state(
        robot_name, bar_id, hold_plan, env_union, bar_map, skip_unsolved=False,
    )

    tool0_grasp_mm = grasp_mm @ np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float)
    held, approach = solve_hold_pair(
        planner, template, bar_id, base_mm, tool0_grasp_mm, check_collision=check_collision,
    )
    if held is None:
        raise RuntimeError(
            f"Bar {bar_id!r}: held IK failed at the STORED base — re-pick the "
            "grasp/base with RSIKKeyframe's support flow."
        )
    if approach is None:
        raise RuntimeError(
            f"Bar {bar_id!r}: approach IK failed at the STORED base — re-pick "
            "the grasp/base with RSIKKeyframe's support flow."
        )

    held_cfg = robot_cell_support.extract_group_config(held, config.SUPPORT_GROUP, sr_cell)
    approach_cfg = robot_cell_support.extract_group_config(approach, config.SUPPORT_GROUP, sr_cell)
    skipped = validate_release_confs(
        planner, sr_cell, robot_name, bar_id, hold_plan,
        base_mm, approach_cfg, held_cfg, partial=True, bar_map=bar_map,
    )
    write_bar_support_keyframe(bar_oid, robot_name, base_mm, grasp_mm, approach_cfg, held_cfg)
    return skipped


# ---------------------------------------------------------------------------
# * Holding / holding-release action builders
# ---------------------------------------------------------------------------


def _cfg_from_group(cfg: dict):
    """A compas Configuration from a stored ``{joint_names, joint_values}``.

    The support arm is six revolute UR joints, so
    ``Configuration.from_revolute_values`` supplies the joint types.

    Args:
        cfg (dict): ``{"joint_names": [...], "joint_values": [...]}``.

    Returns:
        Configuration: the arm configuration.
    """
    from compas_robots import Configuration

    return Configuration.from_revolute_values(
        [float(v) for v in cfg["joint_values"]], list(cfg["joint_names"])
    )


def _support_state_at(scene_state, base_frame_world_mm, cfg: dict = None):
    """A copy of ``scene_state`` at the hold's base, optionally at a config.

    Args:
        scene_state (RobotCellState): the scene to fork (not mutated).
        base_frame_world_mm: the hold's 4x4 base frame, world mm.
        cfg (dict): ``{"joint_names", "joint_values"}`` to stamp, or None to
            leave the arm config as the planner-filled unknown (set to None).

    Returns:
        RobotCellState: the forked state.
    """
    state = scene_state.copy()
    robot_cell_support._apply_base_frame_mm(
        state, np.asarray(base_frame_world_mm, dtype=float)
    )
    if cfg is None:
        state.robot_configuration = None
    else:
        for name, value in zip(cfg["joint_names"], cfg["joint_values"]):
            state.robot_configuration[name] = float(value)
    return state


def _assembly_seq_and_grounds(bar_oid, bar_map):
    """The shared BarSceneAction metadata: ordered bar ids + walkable grounds.

    The sequence lists the real bars only (fake bars dropped even when the
    caller's map has them), exactly like ``ActionSchedule.json`` and the
    assembly actions.
    """
    from core.rhino_walkable_ground import get_bar_ground_ids

    real_map = get_real_bar_seq_map(bar_map)
    assembly_seq = [
        bid for bid, _oid_seq in sorted(real_map.items(), key=lambda kv: kv[1][1])
    ]
    return assembly_seq, get_bar_ground_ids(bar_oid)


def _read_hold_build_inputs(bar_id: str, bar_oid, hold_plan: dict):
    """Common reads + checks both holding builders need.

    Returns:
        tuple: ``(entry, payload, tool0_grasp_mm, approach_tool0_mm)``.

    Raises:
        RuntimeError: bar not in the hold plan, unsolved hold, or a
            robot-assignment mismatch.
    """
    if bar_id not in hold_plan:
        raise RuntimeError(
            f"Bar {bar_id!r} is not in the hold plan (its supported_until is "
            "empty or every stabilizer assembles earlier) — no holding action."
        )
    entry = hold_plan[bar_id]
    payload = read_bar_support_keyframe(bar_oid)
    if payload is None:
        raise RuntimeError(
            f"Bar {bar_id!r} needs a hold but has no solved support keyframe. "
            "Run RSIKKeyframe's support flow on it first."
        )
    if payload["robot_name"] != entry["robot_name"]:
        raise RuntimeError(
            f"Bar {bar_id}'s support keyframe was solved for "
            f"{payload['robot_name']} but the current sequence assigns "
            f"{entry['robot_name']}. Re-run RSIKKeyframe's support flow on it."
        )
    grasp_to_tool0 = np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float)
    tool0_grasp_mm = np.asarray(payload["grasp_frame_world_mm"], dtype=float) @ grasp_to_tool0
    return entry, payload, tool0_grasp_mm, approach_tool0_from_grasp(tool0_grasp_mm)


def build_bar_holding_action(bar_id: str, bar_oid, hold_plan: dict, bar_map=None, env_union=None):
    """Build one held bar's ``BarHoldingAction`` from its stored keyframe.

    Scene = :func:`build_hold_scene_state` (the RELEASE-time built set with
    the held bar shown, Cindy frozen at her assembled pose, any other holding
    robot frozen at its held pose, the rest parked, plus the frozen robots'
    allowed contacts). This is the same scene the keyframe is solved against,
    so the exported states cannot permit a pose the solve would have
    rejected. The gripper may touch the held bar from the linear approach on
    (H_M2, H_M3), not in the free move to the approach start (H_M0, H_M1).

    Args:
        bar_id (str): the held bar.
        bar_oid: its Rhino object id.
        hold_plan (dict): a ``derive_hold_plan`` result.
        bar_map (dict): a ``get_bar_seq_map`` result; fetched when omitted.
        env_union (dict): a ``get_env_union`` result; fetched when omitted.

    Returns:
        BarHoldingAction: movements H_M0..H_M3.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    if env_union is None:
        env_union = get_env_union(bar_map)
    entry, payload, tool0_grasp_mm, approach_tool0_mm = _read_hold_build_inputs(
        bar_id, bar_oid, hold_plan
    )
    robot_name = payload["robot_name"]
    sr_cell = robot_cell_support.get_or_load_support_cell(robot_name)
    ensure_support_env_registered(sr_cell, None, env_union)

    # Scene the hold must clear: the same one the keyframe is solved against.
    scene, _skipped = build_hold_scene_state(
        robot_name, bar_id, hold_plan, env_union, bar_map, skip_unsolved=False,
    )

    base_mm = payload["base_frame_world_mm"]
    approach_cfg = payload["approach"]
    held_cfg = payload["held"]
    gripper = [config.SUPPORT_TOOL_NAME]

    # The gripper touches the held bar only from the linear approach on.
    def _hold_state(cfg, gripper_on_bar: bool):
        state = _support_state_at(scene, base_mm, cfg=cfg)
        set_gripper_bar_contact(state, bar_id, allowed=gripper_on_bar)
        return state

    movements = [
        # H_M0: free travel from wherever the arm is to the approach start.
        # Left unplanned offline (start config unknown at design time); the
        # goal is the solved approach config.
        SingleArmFreeMovement(
            movement_id=f"{bar_id}_H_M0_free_to_approach",
            tag="Current -> approach start (free, live-planned)",
            start_state=_hold_state(None, gripper_on_bar=False),
            target_ee_frames={"arm": _mm4_to_frame(approach_tool0_mm)},
            target_configuration=_cfg_from_group(approach_cfg),
            notes={
                "unplanned_offline": True,
                "planner_fills": "trajectory",
            },
        ),
        # H_M1: gripper opens to receive the bar.
        GripperToolMovement(
            movement_id=f"{bar_id}_H_M1_gripper_open",
            tag="Gripper opens to receive the bar",
            start_state=_hold_state(approach_cfg, gripper_on_bar=False),
            tool_action="open",
            tool_names=list(gripper),
        ),
        # H_M2: straight-line approach onto the grasp pose.
        SingleArmLinearMovement(
            movement_id=f"{bar_id}_H_M2_LM_to_grasp",
            tag="Approach -> grasp (straight line onto the bar)",
            start_state=_hold_state(approach_cfg, gripper_on_bar=True),
            target_ee_frames={"arm": _mm4_to_frame(tool0_grasp_mm)},
            target_configuration=_cfg_from_group(held_cfg),
            notes={"lm_distance_mm": float(config.SUPPORT_LM_DISTANCE_MM)},
        ),
        # H_M3: gripper closes — the bar is now held (until release).
        GripperToolMovement(
            movement_id=f"{bar_id}_H_M3_gripper_close",
            tag="Gripper closes; the bar is held until its stabilizers are built",
            start_state=_hold_state(held_cfg, gripper_on_bar=True),
            tool_action="close",
            tool_names=list(gripper),
        ),
    ]

    assembly_seq, walkable_ground_ids = _assembly_seq_and_grounds(bar_oid, bar_map)
    return BarHoldingAction(
        action_id=f"{bar_id}_H_hold",
        tag=(
            f"{robot_name} holds bar {bar_id} until after "
            f"{entry['release_after_bar_id']}"
        ),
        movements=movements,
        robot_id=config.ROBOT_IDS[robot_name],
        active_bar_id=bar_id,
        assembly_seq=assembly_seq,
        walkable_ground_ids=walkable_ground_ids,
        gripper_kind="Robotiq",
        supported_until=get_supported_until(bar_oid),
    )


def build_bar_holding_release_action(bar_id: str, bar_oid, hold_plan: dict, bar_map=None, env_union=None):
    """Build one held bar's ``BarHoldingReleaseAction`` from its stored keyframe.

    Scene = the release moment (after the last stabilizing bar's step): see
    ``build_release_scene_state``. Built strictly (``partial=False``) — every
    overlapping hold must be solved, which is guaranteed once all bars up to
    the release step are keyframed. The gripper starts both movements in
    contact with the held bar (it opens, then backs straight off it).

    Args: same as ``build_bar_holding_action``.

    Returns:
        BarHoldingReleaseAction: movements HR_M0..HR_M1.
    """
    if bar_map is None:
        bar_map = get_real_bar_seq_map()
    if env_union is None:
        env_union = get_env_union(bar_map)
    entry, payload, _tool0_grasp_mm, approach_tool0_mm = _read_hold_build_inputs(
        bar_id, bar_oid, hold_plan
    )
    robot_name = payload["robot_name"]
    sr_cell = robot_cell_support.get_or_load_support_cell(robot_name)
    ensure_support_env_registered(sr_cell, None, env_union)

    scene, _skipped = build_release_scene_state(
        robot_name, bar_id, hold_plan, env_union, partial=False, bar_map=bar_map,
    )
    set_gripper_bar_contact(scene, bar_id, allowed=True)

    base_mm = payload["base_frame_world_mm"]
    approach_cfg = payload["approach"]
    held_cfg = payload["held"]
    gripper = [config.SUPPORT_TOOL_NAME]

    movements = [
        # HR_M0: gripper opens, letting go of the (now stable) bar.
        GripperToolMovement(
            movement_id=f"{bar_id}_HR_M0_gripper_open",
            tag="Gripper opens, letting go of the stabilized bar",
            start_state=_support_state_at(scene, base_mm, cfg=held_cfg),
            tool_action="open",
            tool_names=list(gripper),
        ),
        # HR_M1: straight-line retreat back along the approach line.
        SingleArmLinearMovement(
            movement_id=f"{bar_id}_HR_M1_LM_retreat",
            tag="Grasp -> approach (straight-line retreat; robot then drives away)",
            start_state=_support_state_at(scene, base_mm, cfg=held_cfg),
            target_ee_frames={"arm": _mm4_to_frame(approach_tool0_mm)},
            target_configuration=None,
            notes={
                "lm_distance_mm": float(config.SUPPORT_LM_DISTANCE_MM),
                "retreat_target_config": "the approach keyframe (same line reversed)",
                "after": "robot drives far away (parked; no longer in any scene)",
            },
        ),
    ]
    # The retreat's goal config IS the approach keyframe (same line reversed).
    movements[1].target_configuration = _cfg_from_group(approach_cfg)

    assembly_seq, walkable_ground_ids = _assembly_seq_and_grounds(bar_oid, bar_map)
    return BarHoldingReleaseAction(
        action_id=f"{bar_id}_HR_hold_release",
        tag=(
            f"{robot_name} releases bar {bar_id} after "
            f"{entry['release_after_bar_id']} is built"
        ),
        movements=movements,
        robot_id=config.ROBOT_IDS[robot_name],
        active_bar_id=bar_id,
        assembly_seq=assembly_seq,
        walkable_ground_ids=walkable_ground_ids,
        released_after_bar_id=entry["release_after_bar_id"],
    )


# ---------------------------------------------------------------------------
# * The global action schedule (per-bar files + one interleaved manifest)
# ---------------------------------------------------------------------------

# How a schedule "kind" maps to file suffix / action id / class name.
SCHEDULE_KINDS = {
    "jointing": ("__J", "_J_joint", "BarAssemblyJointingAction"),
    "release": ("__R", "_R_release", "BarAssemblyReleaseAction"),
    "holding": ("__H", "_H_hold", "BarHoldingAction"),
    "holding_release": ("__HR", "_HR_hold_release", "BarHoldingReleaseAction"),
}


def build_action_schedule_payload(bar_map=None, written_files=None) -> dict:
    """The ``ActionSchedule.json`` content: global interleaved order + robots.

    Pure metadata (no states) rebuilt from the document every time, so the
    manifest can never drift from the sequence data. Real bars only (fake
    bars are dropped even when the caller's map has them).

    When ``written_files`` is given, an entry whose action file is not in it
    (the export skipped that bar or hold) is left out of ``schedule`` and
    listed under ``not_exported`` instead, so the schedule never points at a
    file that does not exist.

    Args:
        bar_map (dict): a ``get_bar_seq_map`` result (with or without the
            fake bars); fetched when omitted.
        written_files (set): the ``"BarActions/<name>.json"`` paths that
            exist; None keeps every entry.

    Returns:
        dict: ``{schema_version, robots, assembly_seq, holds, schedule,
        not_exported}``.
    """
    from core.hold_schedule import build_action_schedule

    bar_map = get_real_bar_seq_map(bar_map)
    bar_seq, supported = collect_hold_inputs(bar_map)
    hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    assembly_seq = [
        bid for bid, _oid_seq in sorted(bar_map.items(), key=lambda kv: kv[1][1])
    ]
    entries = build_action_schedule(assembly_seq, hold_plan, config.ASSEMBLY_ROBOT_NAME)

    robots = {config.ASSEMBLY_ROBOT_NAME: {"robot_id": config.ROBOT_ID, "role": "assembly"}}
    for name in config.SUPPORT_ROBOT_NAMES:
        robots[name] = {"robot_id": config.SUPPORT_ROBOTS[name]["robot_id"], "role": "support"}

    schedule = []
    not_exported = []
    for entry in entries:
        suffix, id_suffix, type_name = SCHEDULE_KINDS[entry["kind"]]
        record = {
            "index": len(schedule),
            "action_id": f"{entry['bar_id']}{id_suffix}",
            "type": type_name,
            "bar_id": entry["bar_id"],
            "robot": entry["robot_name"],
            "file": f"BarActions/{entry['bar_id']}{suffix}.json",
        }
        # ! Never point at a file that was not written: list it apart.
        if written_files is not None and record["file"] not in written_files:
            not_exported.append(record["file"])
            continue
        schedule.append(record)
    if not_exported:
        print(
            f"core.hold_action_builder: NOTE - {len(not_exported)} scheduled action "
            f"file(s) were not exported and are left out of the schedule: "
            f"{', '.join(not_exported[:8])}" + (" ..." if len(not_exported) > 8 else "")
        )
    return {
        "schema_version": 1,
        "robots": robots,
        "assembly_seq": assembly_seq,
        "holds": [
            {
                "bar_id": held_bar_id,
                "robot": e["robot_name"],
                "release_after_bar_id": e["release_after_bar_id"],
            }
            for held_bar_id, e in sorted(
                hold_plan.items(), key=lambda kv: kv[1]["hold_start_seq"]
            )
        ],
        "schedule": schedule,
        # Action files the schedule would name but the export did not write.
        "not_exported": not_exported,
    }


def validate_releases_after_bar(released_bar_id: str) -> dict:
    """Checkpoint 2: full release validation for every hold ending at this bar.

    Called after ``released_bar_id``'s assembly keyframes are accepted (and
    again at export). Only holds that already carry a solved support
    keyframe are checked; unsolved ones are reported as pending. Never
    raises — the caller decides how loudly to surface failures (the accepted
    assembly keyframe must not be lost to a support-side problem).

    Args:
        released_bar_id (str): the bar whose completion may trigger releases.

    Returns:
        dict: ``{held_bar_id: verdict}`` where verdict is "OK", a failure
        message, or a pending/skip note. Empty when no hold releases here.
    """
    results = {}
    try:
        bar_seq, supported = collect_hold_inputs()
        hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    except RuntimeError as exc:
        return {"<hold plan>": f"derivation failed: {exc}"}
    bar_map = get_real_bar_seq_map()

    for held_bar_id, entry in hold_plan.items():
        if entry["release_after_bar_id"] != released_bar_id:
            continue
        held_oid = bar_map[held_bar_id][0]
        try:
            payload = read_bar_support_keyframe(held_oid)
        except RuntimeError as exc:
            results[held_bar_id] = str(exc)
            continue
        if payload is None:
            results[held_bar_id] = "no support keyframe yet — solve it, then this check re-runs"
            continue
        if payload["robot_name"] != entry["robot_name"]:
            results[held_bar_id] = (
                f"solved for {payload['robot_name']} but the sequence now assigns "
                f"{entry['robot_name']} — re-run the support flow"
            )
            continue
        try:
            _client, planner, sr_cell = robot_cell_support.ensure_support_cell_pushed(
                payload["robot_name"]
            )
            validate_release_confs(
                planner,
                sr_cell,
                payload["robot_name"],
                held_bar_id,
                hold_plan,
                payload["base_frame_world_mm"],
                payload["approach"],
                payload["held"],
                partial=False,
                bar_map=bar_map,
            )
            results[held_bar_id] = "OK"
        except RuntimeError as exc:
            results[held_bar_id] = str(exc)
    return results
