"""
Plan on the shared core: design -> `scene_at` -> `CompasFabMirror` -> the existing solvers (tamp, compas_fab).

The Rhino commands (RSIKKeyframe, RSIKKeyframeAll, engine Core) get their movements, planner and cell from here
instead of the legacy cell: `keyframe_movements` for Cindy's chain, `hold_solve_state` for a support robot's hold,
`release_check` for a hold's release. Rhino-free; needs the core's `[mirrors]` stack (compas_fab, PyBullet).

- ! Keep the mirrors in `store` (Rhino's `sc.sticky`) and build the design with the same geometry cache every time:
  a mirror rebuilds its cell (seconds) only when a geometry or robot object changes.
- ! A mirror belongs to one thread, and only one may be lent at a time (`mirror.lend()`).
- ? Three workarounds for open decisions (core integration README) live here, not in the core: part contacts
  (#1, `allow_part_contacts`), unknown robots parked (#3, `plannable`), the hold solved against its release-time
  bodies (#4, `hold_solve_scene`).
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.design import Design, State
from bar_assembly_core.design.relations import ENGAGED, PENDING, bar_of, mate_status
from bar_assembly_core.mirrors.compas import frame_from_pose
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from bar_assembly_core.scene import Scene

#: `sc.sticky` key of the mirrors, robot id -> CompasFabMirror.
MIRRORS_KEY = "bar_joint:core_mirrors"
#: Rhino's keyframe roles -> (action suffix, movement suffix) of the movement whose target is that keyframe.
KEYFRAME_MOVEMENTS = {"M1": ("J", "J_transfer"), "M2": ("J", "J_insert"), "M3": ("R", "R_retreat")}


def mirror_for(store: dict, robot_id: str, log=print) -> CompasFabMirror:
    """The planning mirror of one robot, kept in `store` between commands; a new one if it was closed."""
    mirrors = store.setdefault(MIRRORS_KEY, {})
    mirror = mirrors.get(robot_id)
    if mirror is None or not mirror.connected:
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


# --- --- --- --- --- WORKAROUNDS FOR OPEN DECISIONS --- --- --- --- ---

def _touch(scene: Scene, a: str, b: str) -> None:
    """Allow a contact both ways, on the bodies that take part (a tool or robot id is only an entry)."""
    for body, other in ((a, b), (b, a)):
        if body in scene.bodies and other not in scene.bodies[body].touches:
            scene.bodies[body].touches = tuple(sorted(scene.bodies[body].touches + (other,)))


def allow_part_contacts(design: Design, scene: Scene, state: State) -> None:
    """Allow the contacts Rhino allows today; the core derives fewer. ? Open decision #1, change request 1.

    - A tool may touch the whole part it is on: the bar and its halves.
    - The halves of a pending or engaged mate may touch each other's bar.
    - A half of a `*/Ground` part may touch every ground body.
    """
    for tool, tool_state in state.tools.items():
        bar = bar_of(design, tool_state.on) if tool_state is not None and tool_state.on else None
        if bar is not None:
            for body in (bar, *design.halves_of(bar)):
                _touch(scene, tool, body)
    for mate in design.mates:
        if mate_status(design, state, mate) in (PENDING, ENGAGED):
            a, b = mate
            for half, other in ((a, b), (b, a)):
                other_bar = bar_of(design, other)
                if half.startswith("joints/") and other_bar is not None:
                    _touch(scene, half, other_bar)
    grounds = [body for body in design.bodies if body.startswith("ground/")]
    for body_id, body in design.bodies.items():
        if body.part.endswith("/Ground"):
            for ground in grounds:
                _touch(scene, body_id, ground)


def plannable(scene: Scene, robot_id: str, joints: Optional[Dict[str, float]] = None) -> List[str]:
    """Make a scene plannable for `robot_id`, in place; returns the other robots parked.

    - The acting robot's unknown joints get `joints` or 0 (a seed: the solver replaces them), an open base the origin
      (the solvers set the base they try).
    - ? Open decision #3: another robot with an unknown base or joints is parked, not left at the origin.
    """
    robot = scene.robots[robot_id]
    robot.enabled = True
    if robot.unmeasured or not robot.base_tracked:
        robot.joints = {**{j: 0.0 for j in robot.model.movable_joints}, **robot.joints, **(joints or {})}
        robot.unmeasured, robot.base_tracked = frozenset(), True
    parked = []
    for other_id, other in scene.robots.items():
        if other_id != robot_id and other.enabled and (other.unmeasured or not other.base_tracked):
            other.enabled = False
            parked.append(other_id)
    return parked


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
    first = movement(design, bar, "J", "J_transfer")
    robot_id = design.actions[f"{short(bar)}_J"].robot
    flanges = {side: f"{robot_id}/{side}_ur_arm_tool0" for side in ("left", "right")}
    mirror = mirror_for(store, robot_id, log)
    out = {}
    for role, (action, name) in KEYFRAME_MOVEMENTS.items():
        m = movement(design, bar, action, name) if role != "M1" else first
        scene = design.scene_at(m)
        allow_part_contacts(design, scene, m.start)
        parked = plannable(scene, robot_id)
        if parked and role == "M1":
            log(f"core planning: {', '.join(parked)} parked for {short(bar)} (base or joints not solved yet)")
        mirror.sync(scene)
        out[role] = SimpleNamespace(start_state=mirror.state.copy(), target_configuration=None,
                                    target_ee_frames={side: frame_from_pose(m.target.links[link])
                                                      for side, link in flanges.items()})
    home = movement(design, bar, "R", "R_home").target.joints.get(robot_id)
    out["M4"] = SimpleNamespace(start_state=out["M3"].start_state, target_ee_frames={},
                                target_configuration=mirror.configuration(home) if home else None)
    return mirror, out


# --- --- --- --- --- SUPPORT ROBOTS: HOLD AND RELEASE --- --- --- --- ---

def hold_solve_scene(design: Design, bar: str) -> Scene:
    """The scene a hold is solved in: its linear approach's start, plus the bars it supports until.

    ? Open decision #4: as Rhino's legacy solve, the bars the hold waits for stand at their design pose (the hold
    must not block them), and the held bar is left out (the gripper wraps around it).
    """
    action = design.actions[f"{short(bar)}_H"]
    m = movement(design, bar, "H", "H_to_grasp")
    scene = design.scene_at(m)
    allow_part_contacts(design, scene, m.start)
    for other in action.supports_until:
        for body in (other, *design.halves_of(other)):
            if body in scene.bodies:
                scene.bodies[body].enabled = True
    for body in (bar, *design.halves_of(bar)):
        scene.bodies[body].enabled = False
    return scene


def hold_solve_state(design: Design, store: dict, bar: str, log=print):
    """A support robot's mirror and template state for solving one hold (`robot_cell_support.solve_support_ik`).

    Returns:
        tuple: (the support robot's mirror, a compas_fab `RobotCellState` to fork per base).

    Raises:
        KeyError: If the bar has no hold in the design (no grasp frame picked yet).
    """
    robot_id = design.actions[f"{short(bar)}_H"].robot
    scene = hold_solve_scene(design, bar)
    parked = plannable(scene, robot_id)
    if parked:
        log(f"core planning: {', '.join(parked)} parked for the hold of {short(bar)} (not solved yet)")
    mirror = mirror_for(store, robot_id, log)
    mirror.sync(scene)
    return mirror, mirror.state.copy()


def release_check(design: Design, store: dict, bar: str, log=print) -> str:
    """Check a hold's release in its own moment: the support robot at its held and approach joints.

    Returns:
        str: "OK", "not solved" (no base or joints yet), or the colliding pairs.
    """
    p = short(bar)
    if f"{p}_HR" not in design.actions:
        return "no release in the design"
    action = design.actions[f"{p}_HR"]
    opening, retreat = action.movements[0], action.movements[1]
    robot = opening.start.robots.get(action.robot)
    if robot is None or robot.base is None or robot.joints is None:
        return "not solved"
    scene = design.scene_at(opening)
    allow_part_contacts(design, scene, opening.start)
    parked = plannable(scene, action.robot)
    mirror = mirror_for(store, action.robot, log)
    mirror.sync(scene)
    found = [f"held: {a} x {b}" for a, b in mirror.collisions()]
    approach = (retreat.target.joints or {}).get(action.robot) if retreat.target else None
    if approach:
        found += [f"approach: {a} x {b}" for a, b in mirror.collisions(approach)]
    note = f" ({', '.join(parked)} parked: not solved)" if parked else ""
    return ("OK" if not found else "collides: " + "; ".join(found)) + note


def releases_after(design: Design, bar: str) -> List[str]:
    """The held bars whose release follows this bar's actions in the schedule (Rhino's checkpoint 2)."""
    order = list(design.schedule)
    after = [i for i, a in enumerate(order) if design.actions[a].bar == bar and design.actions[a].type == "bar_release"]
    out = []
    for action_id in order[after[0] + 1:] if after else []:
        action = design.actions[action_id]
        if action.type != "bar_holding_release":
            break
        out.append(action.bar)
    return out
