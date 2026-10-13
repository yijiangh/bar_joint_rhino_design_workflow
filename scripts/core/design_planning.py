"""
Plan on the shared core: design -> `scene_at` -> `CompasFabMirror` -> the existing solvers (tamp, compas_fab).

The Rhino commands get their movements, planner and cell from here: `keyframe_movements` for Cindy's chain
(RSIKKeyframe, RSIKKeyframeAll), `hold_solve_state` and `hold_release_collisions` for a support robot's hold,
`release_check` for a hold's release, `scene_collisions` for the viewers' collision check, `state_scene` to draw a
solver's state. Rhino-free; needs the core's `[mirrors]` stack (compas_fab, PyBullet).

Everything a scene needs comes from the core (decisions of 2026-10-09): `scene_at` derives the allowed part contacts,
`scenes.seeded` fills in the robot being planned for, the mirror parks other robots whose base or joints are unknown
(`mirror.parked`), and `hold.hold_scenes` gives a hold's two scenes.

- ! Keep the mirrors in `store` (Rhino's `sc.sticky`) and build the design with the same geometry cache every time:
  a mirror rebuilds its cell (seconds) only when a geometry or robot object changes.
- ! A mirror belongs to one thread, and only one may be lent at a time (`mirror.lend()`).
"""

from __future__ import annotations

import os
import sys
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Dict, List, Mapping, Optional, Tuple

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.design import Design
from bar_assembly_core.design.hold import hold_scenes
from bar_assembly_core.design.scenes import seeded
from bar_assembly_core.geometry import Pose
from bar_assembly_core.mirrors.compas import frame_from_pose
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from bar_assembly_core.scene import Scene
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
        mirror = mirrors[robot_id] = CompasFabMirror(robot_id, log=log)
    return mirror


def close_mirrors(store: dict) -> None:
    """Close every mirror kept in `store` (their PyBullet worlds and files)."""
    for mirror in store.pop(MIRRORS_KEY, {}).values():
        mirror.close()


def short(bar: str) -> str:
    """"bars/B10" -> "B10"."""
    return bar.split("/", 1)[1]


def movement(design: Design, bar: str, action: str, name: str):
    """A movement by its role-named id, e.g. `movement(design, "bars/B10", "J", "J_insert")`.

    Raises:
        KeyError: If the design has no such action or movement (the bar was left out of the export).
    """
    p = short(bar)
    found = [m for m in design.actions[f"{p}_{action}"].movements if m.id == f"{p}_{name}"]
    if not found:
        raise KeyError(f"{p}_{name}")
    return found[0]


def sync_seeded(store: dict, scene: Scene, robot_id: str, joints: Optional[Mapping[str, float]] = None,
                base: Optional[Pose] = None, log=print, why: str = "", report_parked: bool = True) -> CompasFabMirror:
    """Seed `robot_id` in the scene (`scenes.seeded`, in place), sync it into its mirror, and log the parked robots.

    Args:
        joints, base: Seeds for what the design leaves open (else 0 and the origin: the solvers set the base they try).
        why: What the scene is for, in the parked-robots log line.
        report_parked: Log the parked robots (once per chain is enough).
    """
    seeded(scene, robot_id, joints, base)
    mirror = mirror_for(store, robot_id, log)
    mirror.sync(scene)
    if mirror.parked and report_parked:
        log(f"core planning: {', '.join(mirror.parked)} parked{' for ' + why if why else ''} "
            "(base or joints not solved yet)")
    return mirror


# --- --- --- --- --- CINDY: KEYFRAME CHAIN --- --- --- --- ---

def keyframe_movements(design: Design, store: dict, bar: str, log=print) -> Tuple[CompasFabMirror, Dict[str, object]]:
    """Cindy's keyframe chain of one bar, as tamp's `solve_keyframe_chain` takes it, from the design's scenes.

    Returns:
        tuple: (Cindy's mirror, {"M1": approach, "M2": assembled, "M3": retreat, "M4": home}), each with a
        `start_state` (compas_fab, synced from `scene_at`), `target_ee_frames` {"left", "right"} (metres) and a
        `target_configuration` (home only). Lend the mirror's planner to solve them.

    Raises:
        KeyError: If the bar has no J or R action in the design.
    """
    robot_id = design.actions[f"{short(bar)}_J"].robot
    flanges = {side: f"{robot_id}/{side}_ur_arm_tool0" for side in ("left", "right")}
    out, mirror = {}, None
    for role, (action, name) in KEYFRAME_MOVEMENTS.items():
        m = movement(design, bar, action, name)
        mirror = sync_seeded(store, design.scene_at(m), robot_id, log=log, why=short(bar),
                             report_parked=role == "M1")
        out[role] = SimpleNamespace(start_state=mirror.state.copy(), target_configuration=None,
                                    target_ee_frames={side: frame_from_pose(m.target.links[link])
                                                      for side, link in flanges.items()})
    home = movement(design, bar, "R", "R_home").target.joints.get(robot_id)
    out["M4"] = SimpleNamespace(start_state=out["M3"].start_state, target_ee_frames={},
                                target_configuration=mirror.configuration(home) if home else None)
    return mirror, out


# --- --- --- --- --- SUPPORT ROBOTS: HOLD AND RELEASE --- --- --- --- ---

def hold_solve_state(design: Design, store: dict, bar: str, log=print):
    """A support robot's mirror and template state for solving one hold (`support_ik.solve_support_ik`).

    The hold is solved in the first of its scenes (`hold.hold_scenes`): the start of the movement that closes the
    gripper, where Cindy still holds the bar at its assembled pose.

    Returns:
        tuple: (the support robot's mirror, a compas_fab `RobotCellState` to fork per base).

    Raises:
        KeyError: If the bar has no hold in the design (no grasp frame picked yet).
        ValueError: If the hold has no closing movement or no release (`hold.hold_scenes`).
    """
    action_id = f"{short(bar)}_H"
    robot_id = design.actions[action_id].robot
    solve, _check = hold_scenes(design, action_id)
    mirror = sync_seeded(store, solve, robot_id, log=log, why=f"the hold of {short(bar)}")
    return mirror, mirror.state.copy()


def hold_scene(design: Design, bar: str, which: int = 0) -> Scene:
    """One of a hold's two scenes (`hold.hold_scenes`): 0 to solve it in, 1 to check it in (the release)."""
    return hold_scenes(design, f"{short(bar)}_H")[which]


def hold_release_collisions(design: Design, store: dict, bar: str, base: Pose, held: Mapping[str, float],
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
    robot_id = design.actions[action_id].robot
    check = hold_scenes(design, action_id)[1]
    robot = check.robots[robot_id]
    robot.base, robot.base_tracked = base, True
    robot.joints, robot.unmeasured = {**robot.joints, **{n: float(v) for n, v in held.items()}}, frozenset()
    mirror = sync_seeded(store, check, robot_id, log=log, why=f"the release of {short(bar)}")
    found = [f"held: {a} x {b}" for a, b in mirror.collisions()]
    if approach:
        found += [f"approach: {a} x {b}" for a, b in mirror.collisions(dict(approach))]
    return found


def release_check(design: Design, store: dict, bar: str, log=print) -> str:
    """Check a hold's release in its own scene: the support robot at its held and approach joints.

    Returns:
        str: "OK", "not solved" (no base or joints yet), or the colliding pairs.
    """
    p = short(bar)
    if f"{p}_HR" not in design.actions or f"{p}_H" not in design.actions:
        return "no release in the design"
    action = design.actions[f"{p}_HR"]
    opening, retreat = action.movements[0], action.movements[1]
    robot = opening.start.robots.get(action.robot)
    if robot is None or robot.base is None or robot.joints is None:
        return "not solved"
    check = hold_scene(design, f"bars/{p}", 1)
    mirror = sync_seeded(store, check, action.robot, log=log)
    found = [f"held: {a} x {b}" for a, b in mirror.collisions()]
    approach = (retreat.target.joints or {}).get(action.robot) if retreat.target else None
    if approach:
        found += [f"approach: {a} x {b}" for a, b in mirror.collisions(approach)]
    note = f" ({', '.join(mirror.parked)} parked: not solved)" if mirror.parked else ""
    return ("OK" if not found else "collides: " + "; ".join(found)) + note


def scene_collisions(design: Design, store: dict, scene: Scene, log=print) -> List[Tuple[str, str]]:
    """What collides in a scene, checked on the mirror of every robot placed in it (present, base and joints known).

    Args:
        design: The design the scene comes from (its contacts are already in the scene, from `scene_at`).
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
    from bar_assembly_core.mirrors.compas import pose_from_frame
    from bar_assembly_core.scene import Attachment
    from core.design_display import place

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
