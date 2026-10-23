"""Is a bar's saved IK still right?  -- replay it against today's model.

A saved IK keyframe is only joint angles and a robot base frame (the
``KEY_ASSEMBLY_*`` / ``KEY_SUPPORT_*`` user text on the bar).  Nothing records
what it was solved against, so when a joint, a bar, the walkable ground or
another bar's IK changes, the old keyframe silently stops fitting.  This module
puts the robot back at each saved pose in the CURRENT model and asks the same
questions the solver asked when it accepted the pose -- no re-solve:

Assembly keyframe (the dual-arm robot), per pose -- approach, assembled,
retreat -- at the saved base:

* **reach**: forward kinematics of each arm's flange (tool0) vs that pose's
  target, which is rebuilt from the tools placed today;
* **collision**: one whole-robot check in the very state the solver used for
  that pose (``bar_action.build_split_assembly_movements``: the same attached
  bodies and allowed contacts, the bars built so far, and the support robots
  frozen holding other bars -- so another bar's changed hold shows up here);
* **ground**: the base still standing on a walkable ground, not floating, sunk
  or tilted against it.

Support keyframe (the robot holding the bar): its assignment in today's hold
plan, its grasp still on the bar, both poses reaching their flange targets, no
collision in the hold scene or the release scene (the two scenes the hold was
accepted in), and its base on the ground.

The tolerances are ``config.IK_CHECK_*``.  The small numpy helpers at the top
are Rhino-free and unit-tested; everything below them needs Rhino and a running
PyBullet (RSPBStart) and imports lazily.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# * ---------------------------------------------------------------------------
# * Pure helpers (numpy only)
# * ---------------------------------------------------------------------------


def frame_error(target_mm, actual_mm) -> tuple:
    """``(distance_mm, angle_deg)`` between two 4x4 frames (translation in mm)."""
    target = np.asarray(target_mm, dtype=float)
    actual = np.asarray(actual_mm, dtype=float)
    distance = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    relative = target[:3, :3].T @ actual[:3, :3]
    cos_angle = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return distance, float(np.degrees(np.arccos(cos_angle)))


def point_to_segment_mm(point, start, end) -> float:
    """Distance from *point* to the segment *start*-*end* (same units in and out)."""
    p, a, b = (np.asarray(v, dtype=float) for v in (point, start, end))
    ab = b - a
    length_sq = float(ab @ ab)
    t = 0.0 if length_sq == 0.0 else float(np.clip((p - a) @ ab / length_sq, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def axis_angle_deg(u, v) -> float:
    """Angle between two LINES (0..90 deg): a direction and its reverse count as the same."""
    u, v = np.asarray(u, dtype=float), np.asarray(v, dtype=float)
    cos_angle = abs(float(u @ v)) / (float(np.linalg.norm(u)) * float(np.linalg.norm(v)))
    return float(np.degrees(np.arccos(min(1.0, cos_angle))))


def base_ground_error(base_mm, ground_point, ground_normal) -> tuple:
    """How a base frame sits against the ground point nearest to it.

    Args:
        base_mm: the 4x4 base frame (mm); its origin stands on the ground and
            its +Z is the ground's up normal when it was placed.
        ground_point: the closest ground point to the base origin (mm).
        ground_normal: the ground's up normal there.

    Returns:
        ``(height_mm, beside_mm, tilt_deg)`` -- the origin's height above the
        ground along the normal (negative = below), its distance from the ground
        point across the normal (non-zero = past the ground's edge), and the
        angle between the base's +Z and the normal.
    """
    base = np.asarray(base_mm, dtype=float)
    normal = np.asarray(ground_normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    offset = base[:3, 3] - np.asarray(ground_point, dtype=float)
    height = float(offset @ normal)
    beside = float(np.linalg.norm(offset - height * normal))
    z_axis = base[:3, 2] / np.linalg.norm(base[:3, 2])
    tilt = float(np.degrees(np.arccos(np.clip(float(z_axis @ normal), -1.0, 1.0))))
    return height, beside, tilt


def reach_problem(label: str, target_mm, actual_mm, tol_mm: float, tol_deg: float):
    """A sentence when *actual_mm* misses *target_mm*, else ``None``."""
    distance, angle = frame_error(target_mm, actual_mm)
    if distance <= tol_mm and angle <= tol_deg:
        return None
    return f"{label} {distance:.1f} mm / {angle:.1f} deg from its target"


def ground_problem(label: str, base_mm, ground_point, ground_normal,
                   tol_mm: float, tol_deg: float):
    """A sentence when a base no longer stands on its ground, else ``None``."""
    if ground_point is None:
        return f"{label}: no walkable ground found under it"
    height, beside, tilt = base_ground_error(base_mm, ground_point, ground_normal)
    if beside > tol_mm:
        return f"{label} {beside:.0f} mm past the edge of the walkable ground"
    if abs(height) > tol_mm:
        where = "above" if height > 0 else "below"
        return f"{label} {abs(height):.0f} mm {where} the walkable ground -- did the ground move?"
    if tilt > tol_deg:
        return f"{label} tilted {tilt:.1f} deg against the walkable ground"
    return None


@dataclass
class BarIKReport:
    """What the check found for one bar."""

    bar_id: str
    seq: int = None
    assembly: list = field(default_factory=list)   # problems with the assembly keyframe
    support: list = field(default_factory=list)    # problems with the support keyframe
    notes: list = field(default_factory=list)      # hints that do not flag the bar by themselves

    @property
    def stale(self) -> bool:
        """True when the bar's saved IK must be recalculated."""
        return bool(self.assembly or self.support)


# * ---------------------------------------------------------------------------
# * Rhino + PyBullet part (lazy imports)
# * ---------------------------------------------------------------------------

def _frame_m_to_mm(frame) -> np.ndarray:
    """A compas Frame (metres) as a 4x4 mm matrix."""
    from husky_assembly_tamp.keyframe.ik_keyframe import frame_to_mm4

    return np.asarray(frame_to_mm4(frame), dtype=float)


def _collision_summary(planner, state, cell):
    """``None`` when *state* is clear, else the colliding pairs as one line."""
    from compas_fab.backends import CollisionCheckError
    from husky_assembly_tamp.keyframe.dual_arm_ik import _resolve_collision_pairs

    try:
        planner.check_collision(state, options={"full_report": True})
    except CollisionCheckError as exc:
        pairs = getattr(exc, "collision_pairs", None)
        if pairs:
            summary, _offenders, _n = _resolve_collision_pairs(pairs, cell)
            return summary
        return str(exc) or "collision"
    return None


def ground_soups(grounds_map: dict, soup_cache: dict) -> list:
    """Every walkable ground as a triangle soup (``_bar_ground_soups``, for all grounds)."""
    from core import rhino_walkable_ground as rwg

    soups = []
    for gid, oid in grounds_map.items():
        if gid not in soup_cache:
            try:
                soup_cache[gid] = rwg._walkable_np._mesh_to_soup(rwg.brep_to_compas_mesh(oid))
            except Exception:  # noqa: BLE001 -- an unmeshable ground is skipped, as rwg does
                soup_cache[gid] = None
        if soup_cache[gid] is not None:
            soups.append(soup_cache[gid])
    return soups


def _base_on_ground(label: str, base_mm, soups):
    """The base-vs-ground sentence, or ``None`` (also ``None`` with no ground in the model)."""
    from core import config
    from core import rhino_walkable_ground as rwg

    if not soups:
        return None
    point, normal = rwg._walkable_np.closest_point_on_meshes(soups, np.asarray(base_mm)[:3, 3])
    return ground_problem(
        label, base_mm, point, normal,
        float(config.IK_CHECK_BASE_GROUND_TOL_MM), float(config.IK_CHECK_BASE_TILT_TOL_DEG),
    )


def check_assembly_keyframe(rcell, planner, bar_id: str, keyframe: dict, soups: list) -> list:
    """Problems with one bar's saved dual-arm keyframe; ``[]`` when it still holds.

    Args:
        rcell: the assembly robot cell (the one loaded in *planner*).
        planner: the assembly robot's planner.
        bar_id (str): the bar.
        keyframe (dict): ``bar_action._read_bar_keyframe`` of the bar.
        soups (list): every walkable ground (:func:`ground_soups`).
    """
    from core import bar_action, config, ik_collision_setup
    from core.rhino_helpers import block_instance_xform_mm
    from husky_assembly_tamp.keyframe.dual_arm_ik import apply_base_frame_mm

    arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        return [f"cannot be replayed: {err}"]
    base_mm = np.asarray(keyframe["base_frame_world_mm"], dtype=float)
    jointing, release, _env = bar_action.build_split_assembly_movements(
        rcell, planner, bar_id, base_mm,
        block_instance_xform_mm(arm_tools["left"]), block_instance_xform_mm(arm_tools["right"]),
        approach_groups=keyframe.get("approach"),
        assembled_groups=keyframe.get("assembled"),
        retreat_groups=keyframe.get("retreat"),
    )
    # The movement whose start state the solver used for each pose -- the same
    # role map as RSIKKeyframe (rs_ik_keyframe.py, "M1".."M3").
    poses = (
        ("approach", jointing, bar_action.MV_TRANSFER),
        ("assembled", jointing, bar_action.MV_LM_INSERT),
        ("retreat", release, bar_action.MV_LM_RETREAT),
    )
    tol_mm, tol_deg = float(config.IK_CHECK_TCP_TOL_MM), float(config.IK_CHECK_TCP_TOL_DEG)

    problems = []
    moved = False
    for pose, movements, name in poses:
        if keyframe.get(pose) is None:
            continue  # retreat is optional (bars solved before it was saved)
        movement = movements.get(name)
        if movement is None or movement.target_configuration is None:
            problems.append(f"{pose}: no movement to replay it in")
            continue
        # The solver's check state: this movement's start state at the saved
        # base, with the saved configuration in it (dual_arm_ik.solve_dual_arm_ik).
        state = movement.start_state.copy()
        apply_base_frame_mm(state, base_mm)
        state.robot_configuration = movement.target_configuration.copy()

        hit = _collision_summary(planner, state, rcell)
        if hit:
            problems.append(f"{pose}: collides -- {hit}")
        for side in ("left", "right"):
            target = (movement.target_ee_frames or {}).get(side)
            if target is None:
                continue
            actual = planner.forward_kinematics_to_link(state, config.ASSEMBLY_TOOL0_LINKS[side])
            miss = reach_problem(
                f"{pose}: {side} tool", _frame_m_to_mm(target), _frame_m_to_mm(actual), tol_mm, tol_deg,
            )
            if miss:
                problems.append(miss)
                moved = True
    if moved:
        problems.append("the arms no longer reach the tools: a joint, the bar or a tool moved")
    ground = _base_on_ground("base", base_mm, soups)
    if ground:
        problems.append(ground)
    return problems


def check_support_keyframe(bar_id: str, bar_oid, payload: dict, hold_plan: dict, bar_map: dict,
                           env_union: dict, soups: list) -> list:
    """Problems with one bar's saved support (holding) keyframe; ``[]`` when it still holds.

    Args:
        bar_id (str): the held bar.
        bar_oid: its curve.
        payload (dict): ``hold_action_builder.read_bar_support_keyframe`` of the bar.
        hold_plan (dict): today's ``derive_hold_plan``.
        bar_map (dict): ``get_real_bar_seq_map()``.
        env_union (dict): ``hold_action_builder.get_env_union(bar_map)``.
        soups (list): every walkable ground (:func:`ground_soups`).
    """
    from core import config
    from core import hold_action_builder as hab
    from core import robot_cell_support
    from core.rhino_helpers import curve_endpoints, doc_unit_scale_to_mm

    robot = payload["robot_name"]
    entry = hold_plan.get(bar_id)
    if entry is None:
        return [f"support ({robot}): the bar is no longer held in today's hold plan -- this keyframe is unused"]
    if entry["robot_name"] != robot:
        return [f"support: solved for {robot}, but today's hold plan gives the bar to {entry['robot_name']}"]

    problems = []
    label = f"support ({robot})"

    # * The grasp was picked ON the bar's centre line, Z along the bar.
    to_mm = doc_unit_scale_to_mm()
    start, end = (np.asarray(p, dtype=float) * to_mm for p in curve_endpoints(bar_oid))
    grasp = np.asarray(payload["grasp_frame_world_mm"], dtype=float)
    off_mm = point_to_segment_mm(grasp[:3, 3], start, end)
    off_deg = axis_angle_deg(grasp[:3, 2], end - start)
    if off_mm > float(config.IK_CHECK_GRASP_TOL_MM) or off_deg > float(config.IK_CHECK_GRASP_TOL_DEG):
        problems.append(f"{label}: grasp {off_mm:.1f} mm / {off_deg:.1f} deg off the bar -- the bar moved")

    _client, planner, sr_cell = robot_cell_support.ensure_support_cell_pushed(robot)
    hab.ensure_support_env_registered(sr_cell, planner, env_union)
    base_mm = np.asarray(payload["base_frame_world_mm"], dtype=float)
    tool0_grasp = grasp @ np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float)
    targets = {"held": tool0_grasp, "approach": hab.approach_tool0_from_grasp(tool0_grasp)}
    tol_mm, tol_deg = float(config.IK_CHECK_TCP_TOL_MM), float(config.IK_CHECK_TCP_TOL_DEG)

    # * The two scenes the hold was accepted in: the hold scene (solve) and the
    # * release scene (release check). Gripper-on-bar contact only when held.
    scenes = []
    try:
        scenes.append(("", hab.build_hold_scene_state(
            robot, bar_id, hold_plan, env_union, bar_map, skip_unsolved=True)[0]))
    except RuntimeError as exc:
        problems.append(f"{label}: the hold scene cannot be built -- {exc}")
    try:
        scenes.append((" at release", hab.build_release_scene_state(
            robot, bar_id, hold_plan, env_union, partial=True, bar_map=bar_map)[0]))
    except RuntimeError as exc:
        problems.append(f"{label}: the release scene cannot be built -- {exc}")

    for pose in ("held", "approach"):
        for index, (where, scene) in enumerate(scenes):
            state = hab._support_state_at(scene, base_mm, payload[pose])
            hab.set_gripper_bar_contact(state, bar_id, allowed=(pose == "held"))
            hit = _collision_summary(planner, state, sr_cell)
            if hit:
                problems.append(f"{label}: {pose} pose collides{where} -- {hit}")
            if index == 0:  # one reach check per pose; the scene does not move the arm
                actual = planner.forward_kinematics_to_link(state, config.SUPPORT_TOOL0_LINK)
                miss = reach_problem(
                    f"{label}: {pose} gripper", targets[pose], _frame_m_to_mm(actual), tol_mm, tol_deg,
                )
                if miss:
                    problems.append(miss)
    ground = _base_on_ground(f"{label}: base", base_mm, soups)
    if ground:
        problems.append(ground)
    return problems


def check_all(rcell, planner, progress=print) -> list:
    """Check every bar that carries a saved assembly or support keyframe.

    The caller must have rebuilt the assembly cell from today's model first
    (``robot_cell.rebuild_assembly_cell``): the cached cell does not notice a
    moved joint.

    Returns:
        list[BarIKReport]: one per checked bar, in assembly order.  A bar whose
        check raised is reported as "could not be checked", never passed.
    """
    from core import bar_action, config
    from core import hold_action_builder as hab
    from core.hold_schedule import derive_hold_plan
    from core.rhino_bar_registry import collect_hold_inputs, get_real_bar_seq_map
    from core.rhino_walkable_ground import get_all_walkable_grounds

    bar_map = get_real_bar_seq_map()
    hold_plan = derive_hold_plan(*collect_hold_inputs(bar_map), config.SUPPORT_ROBOT_NAMES)
    soups = ground_soups(get_all_walkable_grounds(), {})
    env_union = None  # collected once, only when some bar has a support keyframe

    reports = []
    for bar_id, (bar_oid, seq) in sorted(bar_map.items(), key=lambda item: item[1][1]):
        keyframe = bar_action._read_bar_keyframe(bar_oid)
        support, support_error = None, None
        try:
            support = hab.read_bar_support_keyframe(bar_oid)
        except RuntimeError as exc:
            support_error = str(exc)
        if keyframe is None and support is None and support_error is None:
            continue

        progress(f"  checking {bar_id} (step {seq}) ...")
        report = BarIKReport(bar_id, seq)
        if keyframe is not None:
            try:
                report.assembly = check_assembly_keyframe(rcell, planner, bar_id, keyframe, soups)
            except Exception as exc:  # noqa: BLE001 -- reported, never passed
                report.assembly = [f"could not be checked: {exc}"]
        if support_error is not None:
            report.support = [f"support: {support_error}"]
        elif support is not None:
            try:
                if env_union is None:
                    env_union = hab.get_env_union(bar_map)
                report.support = check_support_keyframe(
                    bar_id, bar_oid, support, hold_plan, bar_map, env_union, soups,
                )
            except Exception as exc:  # noqa: BLE001 -- reported, never passed
                report.support = [f"support: could not be checked: {exc}"]
        if report.assembly and support is not None and not report.support:
            report.notes.append(
                "its support keyframe passed, but re-check it after re-solving the assembly: "
                "the hold scene stands the assembly robot at this bar's assembled pose"
            )
        reports.append(report)
    return reports


__all__ = [
    "BarIKReport",
    "axis_angle_deg",
    "base_ground_error",
    "check_all",
    "check_assembly_keyframe",
    "check_support_keyframe",
    "frame_error",
    "ground_problem",
    "ground_soups",
    "point_to_segment_mm",
    "reach_problem",
]
