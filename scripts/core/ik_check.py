"""Is a bar's saved IK still right?  -- replay it against today's document.

A saved IK keyframe is only joint angles and a robot base frame (the
``KEY_ASSEMBLY_*`` / ``KEY_SUPPORT_*`` user text on the bar).  Nothing records
what it was solved against, so when a joint, a bar, the walkable ground or
another bar's IK changes, the old keyframe silently stops fitting.  This module
reads the document as a process (``core.process_source``, the same read
RSExportProcess and the viewers do) and asks the questions the solver asked
when it accepted each pose -- no re-solve:

* **reach**: the core's plan check B15 (``plan_check.unreached_targets``):
  forward kinematics of each arm's flange vs that pose's target, which the
  process rebuilds from the tools placed today (``core.ik_status`` reports the
  same thing on RSExportProcess and RSIKKeyframeAll);
* **collision**: every pose of the bar in the process (approach, assembled,
  hold, retreat, home, and the releases it triggers; ``core.plan_steps``) is
  checked on the robots' planning worlds (``process_planning.scene_collisions``,
  what the viewers' CheckCollision runs) -- with the bars built so far and the
  support robots where the schedule has them, so another bar's changed hold
  shows up here;
* **ground**: the saved base still standing on a walkable ground, not floating,
  sunk or tilted against it;
* **support keyframe** (the robot holding the bar): its assignment in today's
  hold plan, and its grasp still on the bar.

The tolerances are ``config.IK_CHECK_*``.  The small numpy helpers at the top
are Rhino-free and unit-tested; everything below them needs Rhino and the core
(it builds the planning worlds on first use) and imports lazily.
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
# * Rhino + core part (lazy imports: the helpers above are tested without Rhino)
# * ---------------------------------------------------------------------------


def _short(bar: str) -> str:
    """"bars/B10" -> "B10"."""
    return bar.split("/", 1)[-1]


def ground_soups(grounds_map: dict, soup_cache: dict) -> list:
    """Every walkable ground as a triangle soup (``_bar_ground_soups``, for all grounds).

    A ground that cannot be meshed is skipped and named on the command line: a base
    standing on it would then be reported as off the ground.
    """
    from core import rhino_walkable_ground as rwg

    soups = []
    for gid, oid in grounds_map.items():
        if gid not in soup_cache:
            try:
                soup_cache[gid] = rwg._walkable_np._mesh_to_soup(rwg.brep_to_compas_mesh(oid))
            except Exception as exc:  # noqa: BLE001 -- one bad ground must not stop the check
                soup_cache[gid] = None
                print(f"ik_check: walkable ground {gid!r} could not be meshed and is skipped ({exc}).")
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


def _is_assembly_action(action_id: str) -> bool:
    """True for a jointing (J) or release (R) action id; False for a hold (H) or its release (HR)."""
    return action_id.rsplit("_", 1)[-1] in ("J", "R")


def reach_problems(process) -> tuple:
    """What the core's plan check B15 says: ``(assembly, support)``, each ``{bar id: sentence}``.

    The assembly robot's poses (J and R actions) land in the first dict, the support
    robots' (H and HR) in the second; the sentence names the worst miss per pose.
    """
    from core.ik_status import stale_from
    from scaffolding_assembly_core.process.plan_check import unreached_targets

    misses = list(unreached_targets(process))
    assembly = stale_from([m for m in misses if _is_assembly_action(m.action)])
    support = stale_from([m for m in misses if not _is_assembly_action(m.action)])
    return assembly, support


def _collision_summary(process, store: dict, scene, log) -> str:
    """The colliding pairs of a scene as one line, or ``""`` when it is clear."""
    from core import process_planning

    pairs = process_planning.scene_collisions(process, store, scene, log=log)
    return "; ".join(f"{a} x {b}" for a, b in pairs)


def check_support_keyframe(process, bar_id: str, bar_oid, payload: dict, soups: list, to_mm: float) -> list:
    """Problems with one bar's saved support (holding) keyframe that need no planning world; ``[]`` when it holds.

    Reach and collision of the support poses come from :func:`reach_problems` and the
    hold / release steps in :func:`check_all`; this part checks the hold plan, the
    grasp and the base.

    Args:
        process: the document as a process.
        bar_id (str): the held bar.
        bar_oid: its curve.
        payload (dict): ``keyframe_store.read_bar_support_keyframe`` of the bar.
        soups (list): every walkable ground (:func:`ground_soups`).
        to_mm (float): document units -> mm.
    """
    from core import config
    from core.rhino_helpers import curve_endpoints

    robot = payload["robot_name"]
    hold = process.actions.get(f"{bar_id}_H")
    if hold is None:
        return [f"support ({robot}): the bar is no longer held in today's hold plan -- this keyframe is unused"]
    planned = _short(hold.robot)
    if planned.lower() != robot.lower():
        return [f"support: solved for {robot}, but today's hold plan gives the bar to {planned}"]

    problems = []
    label = f"support ({robot})"

    # * The grasp was picked ON the bar's centre line, Z along the bar.
    start, end = (np.asarray(p, dtype=float) * to_mm for p in curve_endpoints(bar_oid))
    grasp = np.asarray(payload["grasp_frame_world_mm"], dtype=float)
    off_mm = point_to_segment_mm(grasp[:3, 3], start, end)
    off_deg = axis_angle_deg(grasp[:3, 2], end - start)
    if off_mm > float(config.IK_CHECK_GRASP_TOL_MM) or off_deg > float(config.IK_CHECK_GRASP_TOL_DEG):
        problems.append(f"{label}: grasp {off_mm:.1f} mm / {off_deg:.1f} deg off the bar -- the bar moved")

    ground = _base_on_ground(f"{label}: base", payload["base_frame_world_mm"], soups)
    if ground:
        problems.append(ground)
    return problems


def check_all(process, store: dict, bar_map: dict, progress=print) -> list:
    """Check every bar that carries a saved assembly or support keyframe.

    Args:
        process: the document as a process (``process_source.RhinoProcessSource(...).build().process``),
            read fresh: a process from before a joint moved would miss exactly the change to find.
        store (dict): where the planning worlds are kept between commands (Rhino's ``sc.sticky``).
        bar_map (dict): ``{bar_id: (bar_oid, seq)}`` of the real bars (``get_real_bar_seq_map``).
        progress: where to print the per-bar progress line.

    Returns:
        list[BarIKReport]: one per checked bar, in assembly order.  A bar whose
        check raised is reported as "could not be checked", never passed.
    """
    from core import keyframe_store, plan_steps
    from core.rhino_helpers import doc_unit_scale_to_mm
    from core.rhino_walkable_ground import get_all_walkable_grounds

    reach_assembly, reach_support = reach_problems(process)
    soups = ground_soups(get_all_walkable_grounds(), {})
    to_mm = doc_unit_scale_to_mm()
    reports = {}

    def report_of(bar_id: str) -> BarIKReport:
        """The report of a bar, made on first use (a release step is charged to the held bar)."""
        if bar_id not in reports:
            reports[bar_id] = BarIKReport(bar_id, bar_map[bar_id][1] if bar_id in bar_map else None)
        return reports[bar_id]

    for bar_id, (bar_oid, seq) in sorted(bar_map.items(), key=lambda item: item[1][1]):
        keyframe = keyframe_store.read_assembly_keyframe(bar_oid)
        support, support_error = None, None
        try:
            support = keyframe_store.read_bar_support_keyframe(bar_oid)
        except RuntimeError as exc:
            support_error = str(exc)
        if keyframe is None and support is None and support_error is None:
            continue

        progress(f"  checking {bar_id} (step {seq}) ...")
        report = report_of(bar_id)
        # * Reach: the core compared every saved pose with the targets the tools give today.
        if bar_id in reach_assembly:
            report.assembly.append(reach_assembly[bar_id])
        if bar_id in reach_support:
            report.support.append(reach_support[bar_id])
        # * Collision: every pose of this bar in the process, on the planning worlds.
        try:
            for step in plan_steps.bar_steps(process, f"bars/{bar_id}"):
                hit = _collision_summary(process, store, plan_steps.step_scene(process, step), progress)
                if not hit:
                    continue
                line = f"{step.label}: collides -- {hit}"
                if isinstance(step.pose, tuple):
                    # A release step: the support robot letting go of an earlier bar, so it is that bar's hold.
                    report_of(_short(step.pose[1])).support.append(line)
                elif step.pose == plan_steps.HOLD:
                    report.support.append(line)
                else:
                    report.assembly.append(line)
        except Exception as exc:  # noqa: BLE001 -- reported, never passed
            report.assembly.append(f"could not be checked for collisions: {exc}")
        # * Ground: the assembly robot's saved base.
        if keyframe is not None:
            ground = _base_on_ground("base", keyframe["base_frame_world_mm"], soups)
            if ground:
                report.assembly.append(ground)
        # * The support keyframe: hold plan, grasp, base.
        if support_error is not None:
            report.support.append(f"support: {support_error}")
        elif support is not None:
            try:
                report.support += check_support_keyframe(process, bar_id, bar_oid, support, soups, to_mm)
            except Exception as exc:  # noqa: BLE001 -- reported, never passed
                report.support.append(f"support: could not be checked: {exc}")
        if report.assembly and support is not None and not report.support:
            report.notes.append(
                "its support keyframe passed, but re-check it after re-solving the assembly: "
                "the hold scene stands the assembly robot at this bar's assembled pose"
            )
    return sorted(reports.values(), key=lambda r: (r.seq is None, r.seq))


__all__ = [
    "BarIKReport",
    "axis_angle_deg",
    "base_ground_error",
    "check_all",
    "check_support_keyframe",
    "frame_error",
    "ground_problem",
    "ground_soups",
    "point_to_segment_mm",
    "reach_problem",
    "reach_problems",
]
