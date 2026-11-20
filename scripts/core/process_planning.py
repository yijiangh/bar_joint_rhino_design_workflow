"""
Plan on the shared core: process -> `scene_at` -> `CompasFabMirror` -> the existing solvers (tamp, compas_fab).

The Rhino commands get their movements, planner and cell from here: `keyframe_movements` for Cindy's chain
(RSIKKeyframe, RSIKKeyframeAll), `hold_solve_state` and `hold_release_collisions` for a support robot's hold,
`release_check` for a hold's release, `scene_collisions` for the viewers' collision check, `own_tool_gaps` for
Cindy's wrist 1 coming too close to its own tool, `state_scene` to draw a solver's state. Rhino-free; needs the
core's `[mirrors]` stack (compas_fab, PyBullet).

Everything a scene needs comes from the core (decisions of 2026-10-09): `scene_at` derives the allowed contacts,
`scenes.seeded` fills in the robot being planned for, the mirror parks other robots whose base or joints are unknown
(`mirror.parked`), and `hold.hold_scenes` gives a hold's two scenes.

- ! Keep the mirrors in `store` (Rhino's `sc.sticky`) and build the process with the same geometry cache every time:
  a mirror rebuilds its cell (seconds) only when a geometry or robot object changes.
- ! A mirror belongs to one thread, and only one may be lent at a time (`mirror.lend()`).
"""

from __future__ import annotations

import os
import sys
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Dict, List, Mapping, Optional, Tuple

import session_log
from core import config
from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.hold import hold_scenes
from scaffolding_assembly_core.process.scenes import seeded
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.mirrors.compas import frame_from_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import FLANGE_SUFFIX, RobotModel
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES
from core.plan_steps import releases_after  # noqa: F401  (the release check's callers ask here)

#: `sc.sticky` key of the mirrors, robot id -> CompasFabMirror.
MIRRORS_KEY = "bar_joint:core_mirrors"
#: Rhino's keyframe roles -> (action suffix, movement suffix) of the movement whose target is that keyframe.
KEYFRAME_MOVEMENTS = {"M1": ("J", "J_transfer"), "M2": ("J", "J_insert"), "M3": ("R", "R_retreat")}


def _stdout_is_a_file() -> bool:
    """Whether `sys.stdout` has an OS handle that can be duplicated and reopened (what compas_fab's redirect does)."""
    try:
        fd = os.dup(sys.stdout.fileno())
    except (OSError, ValueError, AttributeError):
        return False
    try:
        os.fstat(fd)
    except OSError:
        return False
    finally:
        os.close(fd)
    return True


@contextmanager
def _no_redirect(to=os.devnull, enabled=True):
    yield


def _allow_pybullet_without_stdout() -> None:
    """Let compas_fab's PyBullet client connect when Rhino's stdout has no OS handle.

    - ! The client hides PyBullet's start-up text by reopening a duplicate of stdout's descriptor. In Rhino on
      Windows descriptor 1 may exist without a handle behind it (it depends on how Rhino was started); the reopen
      then raises `OSError: [WinError 6] The handle is invalid` and no planning cell can be built.
    - ? Without a handle that text goes nowhere anyway, so the redirect is skipped, not replaced.
    """
    if _stdout_is_a_file():
        return
    from compas_fab.backends.pybullet import client  # noqa: PLC0415

    client.redirect_stdout = _no_redirect


def mirror_for(store: dict, robot_id: str, log=print) -> CompasFabMirror:
    """The planning mirror of one robot, kept in `store` between commands; a new one if it was closed."""
    mirrors = store.setdefault(MIRRORS_KEY, {})
    mirror = mirrors.get(robot_id)
    if mirror is None or not mirror.connected:
        _allow_pybullet_without_stdout()
        with session_log.step("create planning world", robot_id):
            mirror = mirrors[robot_id] = CompasFabMirror(robot_id, log=log)
    return mirror


def close_mirrors(store: dict) -> None:
    """Close every mirror kept in `store` (their PyBullet worlds and files)."""
    for mirror in store.pop(MIRRORS_KEY, {}).values():
        mirror.close()


def short(bar: str) -> str:
    """"bars/B10" -> "B10"."""
    return bar.split("/", 1)[1]


def movement(process: Process, bar: str, action: str, name: str):
    """A movement by its role-named id, e.g. `movement(process, "bars/B10", "J", "J_insert")`.

    Raises:
        KeyError: If the process has no such action or movement (the bar was left out of the export).
    """
    p = short(bar)
    found = [m for m in process.actions[f"{p}_{action}"].movements if m.id == f"{p}_{name}"]
    if not found:
        raise KeyError(f"{p}_{name}")
    return found[0]


def sync_seeded(store: dict, scene: Scene, robot_id: str, joints: Optional[Mapping[str, float]] = None,
                base: Optional[Pose] = None, log=print, why: str = "", report_parked: bool = True) -> CompasFabMirror:
    """Seed `robot_id` in the scene (`scenes.seeded`, in place), sync it into its mirror, and log the parked robots.

    Args:
        joints, base: Seeds for what the process leaves open (else 0 and the origin: the solvers set the base they try).
        why: What the scene is for, in the parked-robots log line.
        report_parked: Log the parked robots (once per chain is enough).
    """
    seeded(scene, robot_id, joints, base)
    mirror = mirror_for(store, robot_id, log)
    with session_log.step("sync scene into planning world", robot_id):
        mirror.sync(scene)
    if mirror.parked and report_parked:
        log(f"core planning: {', '.join(mirror.parked)} parked{' for ' + why if why else ''} "
            "(base or joints not solved yet)")
    return mirror


# --- --- --- --- --- CINDY: KEYFRAME CHAIN --- --- --- --- ---

@session_log.timed()
def keyframe_movements(process: Process, store: dict, bar: str, log=print) -> Tuple[CompasFabMirror, Dict[str, object]]:
    """Cindy's keyframe chain of one bar, as tamp's `solve_keyframe_chain` takes it, from the process's scenes.

    Returns:
        tuple: (Cindy's mirror, {"M1": approach, "M2": assembled, "M3": retreat, "M4": home}), each with a
        `start_state` (compas_fab, synced from `scene_at`), `target_ee_frames` {"left", "right"} (metres) and a
        `target_configuration` (home only). Lend the mirror's planner to solve them.

    Raises:
        KeyError: If the bar has no J or R action in the process.
    """
    robot_id = process.actions[f"{short(bar)}_J"].robot
    flanges = {side: f"{robot_id}/{side}_ur_arm_tool0" for side in ("left", "right")}
    out, mirror = {}, None
    for role, (action, name) in KEYFRAME_MOVEMENTS.items():
        m = movement(process, bar, action, name)
        mirror = sync_seeded(store, process.scene_at(m), robot_id, log=log, why=short(bar),
                             report_parked=role == "M1")
        out[role] = SimpleNamespace(start_state=mirror.state.copy(), target_configuration=None,
                                    target_ee_frames={side: frame_from_pose(m.target.link_poses[link])
                                                      for side, link in flanges.items()})
    home = movement(process, bar, "R", "R_home").target.joints.get(robot_id)
    out["M4"] = SimpleNamespace(start_state=out["M3"].start_state, target_ee_frames={},
                                target_configuration=mirror.configuration(home) if home else None)
    return mirror, out


# --- --- --- --- --- SUPPORT ROBOTS: HOLD AND RELEASE --- --- --- --- ---

@session_log.timed()
def hold_solve_state(process: Process, store: dict, bar: str, log=print):
    """A support robot's mirror and template state for solving one hold (`support_ik.solve_support_ik`).

    The hold is solved in the first of its scenes (`hold.hold_scenes`): the start of the movement that closes the
    gripper, where Cindy still holds the bar at its assembled pose.

    Returns:
        tuple: (the support robot's mirror, a compas_fab `RobotCellState` to fork per base).

    Raises:
        KeyError: If the bar has no hold in the process (no grasp frame picked yet).
        ValueError: If the hold has no closing movement or no release (`hold.hold_scenes`).
    """
    action_id = f"{short(bar)}_H"
    robot_id = process.actions[action_id].robot
    solve, _check = hold_scenes(process, action_id)
    mirror = sync_seeded(store, solve, robot_id, log=log, why=f"the hold of {short(bar)}")
    return mirror, mirror.state.copy()


def hold_scene(process: Process, bar: str, which: int = 0) -> Scene:
    """One of a hold's two scenes (`hold.hold_scenes`): 0 to solve it in, 1 to check it in (the release)."""
    return hold_scenes(process, f"{short(bar)}_H")[which]


@session_log.timed()
def hold_release_collisions(process: Process, store: dict, bar: str, base: Pose, held: Mapping[str, float],
                            approach: Optional[Mapping[str, float]] = None, log=print) -> List[str]:
    """Check a hold candidate in the second of its scenes (the start of its release): at its held and approach joints.

    The release is where the hold must not be in the way of what was built while it held.

    Args:
        base: The candidate base; held, approach: its joints at the grasp and at the approach.

    Returns:
        list[str]: "held: <a> x <b>" / "approach: <a> x <b>" per colliding pair; empty if clear.

    Raises:
        KeyError, ValueError: As `hold_solve_state`.
    """
    action_id = f"{short(bar)}_H"
    robot_id = process.actions[action_id].robot
    check = hold_scenes(process, action_id)[1]
    robot = check.robots[robot_id]
    robot.base, robot.base_tracked = base, True
    robot.joints, robot.unmeasured = {**robot.joints, **{n: float(v) for n, v in held.items()}}, frozenset()
    mirror = sync_seeded(store, check, robot_id, log=log, why=f"the release of {short(bar)}")
    found = [f"held: {a} x {b}" for a, b in mirror.collisions()]
    if approach:
        found += [f"approach: {a} x {b}" for a, b in mirror.collisions(dict(approach))]
    return found


@session_log.timed()
def release_check(process: Process, store: dict, bar: str, log=print) -> str:
    """Check a hold's release in its own scene: the support robot at its held and approach joints.

    Returns:
        str: "OK", "not solved" (no base or joints yet), or the colliding pairs.
    """
    p = short(bar)
    if f"{p}_HR" not in process.actions or f"{p}_H" not in process.actions:
        return "no release in the process"
    action = process.actions[f"{p}_HR"]
    opening, retreat = action.movements[0], action.movements[1]
    robot = opening.start.robots.get(action.robot)
    if robot is None or robot.base is None or robot.joints is None:
        return "not solved"
    check = hold_scene(process, f"bars/{p}", 1)
    mirror = sync_seeded(store, check, action.robot, log=log)
    found = [f"held: {a} x {b}" for a, b in mirror.collisions()]
    approach = (retreat.target.joints or {}).get(action.robot) if retreat.target else None
    if approach:
        found += [f"approach: {a} x {b}" for a, b in mirror.collisions(approach)]
    note = f" ({', '.join(mirror.parked)} parked: not solved)" if mirror.parked else ""
    return ("OK" if not found else "collides: " + "; ".join(found)) + note


@session_log.timed()
def scene_collisions(process: Process, store: dict, scene: Scene, log=print) -> List[Tuple[str, str]]:
    """What collides in a scene, checked on the mirror of every robot placed in it (present, base and joints known).

    Args:
        process: The process the scene comes from (its contacts are already in the scene, from `scene_at`).
        store: Where the mirrors are kept (Rhino's `sc.sticky`).
        scene: The world to check; not changed.

    Returns:
        list: Colliding pairs as our ids, each sorted, without repeats; robot links as "robots/<name>/<link>".
    """
    pairs = set()
    for robot_id, robot in scene.robots.items():
        if not robot.enabled or not robot.base_tracked or robot.unmeasured:
            continue
        mirror = sync_seeded(store, scene.copy(), robot_id, log=log)
        pairs |= {tuple(sorted(pair)) for pair in mirror.collisions(full_report=True)}
    return sorted(pairs)


# --- --- --- --- --- CINDY: WRIST 1 AGAINST ITS OWN TOOL --- --- --- --- ---

#: The arm link that the tool on that arm's flange may come too close to. `urdf.TOOL_TOUCHES_ARM_LINKS` lets the
#: tool touch the links after it; wrist 1 gets a contact check only, which the lab found is not enough (19 Nov 2026,
#: B35: 0.36 mm at the design base, 0.09 mm live).
OWN_TOOL_LINK = "wrist_1_link"
#: Cindy's saved poses (`plan_steps` names) whose wrist 1 is checked. The gap depends on her arm joints only, and
#: every other pose of her J and R actions repeats one of these: each movement starts where the one before ended,
#: so the transfer target is the approach, the ungrasp and the retreat start are the assembled pose, and the home
#: start is the retreat. Home itself is the fixed `config.HUSKY_DUAL_ARM_HOME_CONF_12`.
OWN_TOOL_POSES = ("approach", "assembled", "retreat")
#: Rhino's keyframe roles (`KEYFRAME_MOVEMENTS`) -> the pose each one solves.
ROLE_POSES = {"M1": "approach", "M2": "assembled", "M3": "retreat"}


def own_tool_pairs(robot_id: str, model: RobotModel) -> Dict[Tuple[str, str], str]:
    """The pairs `own_tool_gaps` measures: (wrist 1 id, tool id) -> the arm's prefix, per arm with a tool mounted.

    Example: {("robots/cindy/left_ur_arm_wrist_1_link", "tools/cindy/AT3L"): "left_ur_arm", ...}.
    """
    out = {}
    for flange, tool in sorted(model.tools.items()):
        if not flange.endswith(FLANGE_SUFFIX):
            continue
        arm = flange[:-len(FLANGE_SUFFIX)]
        if f"{arm}_{OWN_TOOL_LINK}" in model.links:
            out[(f"{robot_id}/{arm}_{OWN_TOOL_LINK}", tool.id)] = arm
    return out


@session_log.timed()
def own_tool_gaps(store: dict, scene: Scene, robot_id: str, min_gap: Optional[float] = None,
                  log=print) -> Dict[Tuple[str, str], float]:
    """Each arm's wrist 1 that is closer than `min_gap` to the tool on that same arm, with the gap.

    Measured on the robot's mirror, synced to `scene`, with the core's margin check (`search_check` with
    `pair_margins`, `SearchCheck.distances`). Only the wrist 1 / own tool pairs get the margin.

    Args:
        store: Where the mirrors are kept (Rhino's `sc.sticky`).
        scene: The world to measure in, e.g. a `plan_steps.step_scene` or a solver's `state_scene`; not changed.
        robot_id: The robot whose arms are measured, e.g. "robots/cindy".
        min_gap: Metres, as PyBullet measures it (each shape padded by about 1 mm); None: `config.OWN_TOOL_MIN_GAP_M`.

    Returns:
        dict: (wrist 1 id, tool id) -> metres, 0 for contact, sorted, e.g.
        {("robots/cindy/right_ur_arm_wrist_1_link", "tools/cindy/AT3R"): 0.00036}. Empty when every arm keeps the
        gap, or the robot is absent or not placed (base or joints unknown).
    """
    robot = scene.robots.get(robot_id)
    if robot is None or not robot.enabled or not robot.base_tracked or robot.unmeasured:
        return {}
    pairs = own_tool_pairs(robot_id, robot.model)
    if not pairs:
        return {}
    gap = float(config.OWN_TOOL_MIN_GAP_M if min_gap is None else min_gap)
    mirror = sync_seeded(store, scene.copy(), robot_id, log=log, report_parked=False)
    configuration = mirror.state.robot_configuration
    values = dict(zip(configuration.joint_names, configuration.joint_values))
    names = [f"{arm}_{joint}" for arm in sorted(set(pairs.values())) for joint in UR_JOINT_NAMES]
    check = mirror.search_check(names, pair_margins={frozenset(pair): gap for pair in pairs})
    out = {}
    for (a, b), distance in check.distances([values[name] for name in names]).items():
        key = (a, b) if (a, b) in pairs else (b, a)
        if key in pairs:
            out[key] = float(distance)
    return dict(sorted(out.items()))


def keyframe_own_tool_gaps(store: dict, scene: Scene, robot_id: str, solved: Mapping[str, object],
                           log=print) -> List[Tuple[str, Dict[Tuple[str, str], float]]]:
    """`own_tool_gaps` of a solved keyframe chain: (pose name, gaps) for each pose that is too close, in chain order.

    Args:
        store: Where the mirrors are kept.
        scene: A scene the chain's mirror was synced with, e.g. the bar's J_transfer scene; not changed.
        robot_id: The solved robot.
        solved: Keyframe roles -> solved compas_fab states, {"M1": approach, "M2": assembled, "M3": retreat}.
    """
    out = []
    for role, pose in ROLE_POSES.items():
        state = solved.get(role)
        if state is None:
            continue
        gaps = own_tool_gaps(store, state_scene(scene, robot_id, state), robot_id, log=log)
        if gaps:
            out.append((pose, gaps))
    return out


def bar_own_tool_gaps(process: Process, store: dict, bar: str,
                      log=print) -> List[Tuple[str, Dict[Tuple[str, str], float]]]:
    """`own_tool_gaps` of one bar's saved assembly poses in the process: (pose name, gaps) per pose too close.

    Args:
        bar: E.g. "bars/B35". A bar without IK, or without a J action, has none.
    """
    from core import plan_steps

    action = process.actions.get(f"{short(bar)}_J")
    if action is None:
        return []
    out = []
    for step in plan_steps.bar_steps(process, bar):
        if step.pose not in OWN_TOOL_POSES:
            continue
        gaps = own_tool_gaps(store, plan_steps.step_scene(process, step), action.robot, log=log)
        if gaps:
            out.append((step.pose, gaps))
    return out


def own_tool_gap_line(gaps: Mapping[Tuple[str, str], float], min_gap: Optional[float] = None,
                      brief: bool = False) -> str:
    """The one wording of `own_tool_gaps`' result, for every command that prints it.

    Args:
        gaps: (wrist 1 id, tool id) -> metres.
        min_gap: The gap asked for, metres; None: `config.OWN_TOOL_MIN_GAP_M`.
        brief: The short form for a prompt.

    Returns:
        str: "right wrist 1 is 0.36 mm from its tool (needs 1.0 mm)", "left wrist 1 is 0.80 mm and right wrist 1
        is 0.36 mm from its tool (needs 1.0 mm)"; brief: "right wrist 1 0.36 mm". Empty for no gaps.
    """
    if not gaps:
        return ""
    gap_mm = 1000.0 * float(config.OWN_TOOL_MIN_GAP_M if min_gap is None else min_gap)
    # ? "robots/cindy/right_ur_arm_wrist_1_link" -> "right": the arm's prefix up to its first "_".
    parts = [(link.rsplit("/", 1)[-1].split("_", 1)[0], 1000.0 * distance)
             for (link, _tool), distance in sorted(gaps.items())]
    if brief:
        return ", ".join(f"{side} wrist 1 {mm:.2f} mm" for side, mm in parts)
    return " and ".join(f"{side} wrist 1 is {mm:.2f} mm" for side, mm in parts) + \
        f" from its tool (needs {gap_mm:.1f} mm)"


def state_scene(scene: Scene, robot_id: str, state) -> Scene:
    """A solver's state as a scene to draw: a copy of `scene` with `robot_id` and the bodies where `state` has them.

    The solvers work on a mirror's compas_fab states; viewers draw scenes (`core.ik_viz`). The robot takes the
    state's base and joints; a body takes the state's attachment to the robot, or its world frame when the state
    holds it still, and is shown only if the state shows it.

    Args:
        scene: The scene the mirror was synced with (same robot, same bodies); not changed.
        robot_id: The mirror's robot, e.g. "robots/cindy".
        state: A `RobotCellState` of that mirror.
    """
    from scaffolding_assembly_core.mirrors.compas import pose_from_frame
    from scaffolding_assembly_core.scene import Attachment
    from core.process_display import place

    out = scene.copy()
    movable = set(out.robots[robot_id].model.movable_joints)
    configuration = state.robot_configuration
    joints = {name: value for name, value in zip(configuration.joint_names, configuration.joint_values)
              if name in movable}
    for body_id, body_state in state.rigid_body_states.items():
        body = out.bodies.get(body_id)
        if body is None:
            continue
        body.enabled = not body_state.is_hidden
        if body_state.attached_to_link:
            body.placement = Attachment(robot_id, body_state.attached_to_link,
                                        pose_from_frame(body_state.attachment_frame))
        elif isinstance(body.placement, Attachment) and body.placement.parent == robot_id:
            body.placement = pose_from_frame(body_state.frame)
    place(out, robot_id, pose_from_frame(state.robot_base_frame), joints)
    return out
