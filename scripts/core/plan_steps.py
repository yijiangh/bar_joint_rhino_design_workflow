"""
The steps the plan viewers show, as moments of a schema 2 design: the poses per bar, the whole timeline, and the
trajectories of a design folder's solutions.

Rhino-free (numpy and the core only); `core.design_display` draws the scenes.

Poses of one bar (the viewers' vocabulary, kept from the legacy viewer):

- `approach`: the arms at the approach keyframe, bar gripped: the start of `<bar>_J_insert`;
- `assembled`: the bar inserted, arms still gripping: the start of the movement after the jointing action;
- `hold` (held bars): the support robot grips too: the start of `<bar>_R_ungrasp`, after the hold;
- `retreat`: arms pulled back, bar released: the start of `<bar>_R_home`;
- `home`: the arms home: the end of `<bar>_R_home`;
- `("release", <held bar>)`: a hold whose release follows this bar lets go: the end of `<held>_HR_retreat`.

A bar whose keyframe is unsolved (Cindy's joints unknown at the insert) has only its release steps.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.design import Design, Movement
from bar_assembly_core.geometry import Pose
from bar_assembly_core.scene import Scene

from core.design_display import place, refresh

#: The release pose's tag: a step `("release", "bars/B2")`.
RELEASE = "release"
#: Poses every solved bar has, in order (`hold` comes after `assembled` for held bars).
BASE_POSES = ("approach", "assembled", "retreat", "home")
HOLD = "hold"

PoseName = Union[str, Tuple[str, str]]


def short(bar: str) -> str:
    """"bars/B10" -> "B10"."""
    return bar.split("/", 1)[1]


@dataclass(frozen=True)
class Step:
    """One moment a viewer shows: the start (or end) of one design movement, named by its host bar's pose.

    Attributes:
        bar: The host bar, e.g. "bars/B3".
        pose: A pose name, or `("release", held bar id)`.
        movement: The movement whose start (or end) this moment is.
        at_end: The movement's end: its target joints applied.
    """

    bar: str
    pose: PoseName
    movement: Movement
    at_end: bool = False

    @property
    def label(self) -> str:
        """E.g. "B3 approach", "B3 release B2"."""
        pose = f"{self.pose[0]} {short(self.pose[1])}" if isinstance(self.pose, tuple) else self.pose
        return f"{short(self.bar)} {pose}"


def find_movement(design: Design, action_id: str, movement_id: str) -> Optional[Movement]:
    """A movement by its action and movement ids, or None."""
    action = design.actions.get(action_id)
    if action is None:
        return None
    return next((m for m in action.movements if m.id == movement_id), None)


def movement_after(design: Design, movement: Movement) -> Optional[Movement]:
    """The movement that follows one in the schedule, or None after the last."""
    found = False
    for _action, m in design.movements():
        if found:
            return m
        found = m is movement
    return None


def releases_after(design: Design, bar: str) -> List[str]:
    """The held bars whose release follows this bar's actions in the schedule (in release order)."""
    order = list(design.schedule)
    after = [i for i, a in enumerate(order) if design.actions[a].bar == bar and design.actions[a].type == "bar_release"]
    out = []
    for action_id in order[after[0] + 1:] if after else []:
        action = design.actions[action_id]
        if action.type != "bar_holding_release":
            break
        out.append(action.bar)
    return out


def _known(movement: Optional[Movement], robot: str) -> bool:
    """Whether a robot's joints are known at the start of a movement."""
    state = movement.start.robots.get(robot) if movement is not None else None
    return state is not None and state.joints is not None


def is_solved(design: Design, bar: str) -> bool:
    """Whether the bar's assembly keyframe is in the design: the assembling robot's joints known at the insert."""
    p = short(bar)
    action = design.actions.get(f"{p}_J")
    return action is not None and _known(find_movement(design, f"{p}_J", f"{p}_J_insert"), action.robot)


def bar_steps(design: Design, bar: str) -> List[Step]:
    """The ordered steps of one bar (see the module docstring); empty for a bar the design does not assemble."""
    p = short(bar)
    jointing = design.actions.get(f"{p}_J")
    if jointing is None:
        return []
    steps: List[Step] = []
    if is_solved(design, bar):
        insert = find_movement(design, f"{p}_J", f"{p}_J_insert")
        steps.append(Step(bar, "approach", insert))
        after = movement_after(design, jointing.movements[-1])
        steps.append(Step(bar, "assembled", after) if after is not None else Step(bar, "assembled", insert, True))
        hold = design.actions.get(f"{p}_H")
        ungrasp = find_movement(design, f"{p}_R", f"{p}_R_ungrasp")
        if hold is not None and _known(ungrasp, hold.robot):
            steps.append(Step(bar, HOLD, ungrasp))
        home = find_movement(design, f"{p}_R", f"{p}_R_home")
        if home is not None:
            steps.append(Step(bar, "retreat", home))
            steps.append(Step(bar, "home", home, True))
    for held in releases_after(design, bar):
        h = short(held)
        retreat = find_movement(design, f"{h}_HR", f"{h}_HR_retreat")
        if retreat is not None:
            steps.append(Step(bar, (RELEASE, held), retreat, True))
    return steps


def timeline(design: Design) -> Tuple[List[Step], List[Tuple[str, str]]]:
    """Every step of the whole assembly in schedule order, and the bars that add no assembly step.

    Returns:
        tuple: (steps, skipped) with skipped `[(bar id, reason)]`: bars whose keyframe is unsolved (their release
        steps are kept: the holding robots must not vanish between two steps).
    """
    steps, skipped = [], []
    for action_id in design.schedule:
        action = design.actions[action_id]
        if action.type != "bar_jointing":
            continue
        own = bar_steps(design, action.bar)
        steps += own
        if not is_solved(design, action.bar):
            reason = "no solved IK keyframe"
            if own:
                reason += f" (its release step(s) for {', '.join(short(s.pose[1]) for s in own)} are kept)"
            skipped.append((action.bar, reason))
    return steps, skipped


def step_scene(design: Design, step: Step) -> Scene:
    """The world at one step: the movement's start, or its end with the target joints applied."""
    scene = design.scene_at(step.movement)
    target = step.movement.target
    if step.at_end and target is not None:
        for robot, joints in target.joints.items():
            place(scene, robot, joints=joints, update=False)
        refresh(scene)
    return scene


def base_of(design: Design, bar: str) -> Optional[Pose]:
    """The assembling robot's base for a bar (from the jointing action), or None before it is picked."""
    p = short(bar)
    jointing = design.actions.get(f"{p}_J")
    insert = find_movement(design, f"{p}_J", f"{p}_J_insert")
    state = insert.start.robots.get(jointing.robot) if insert is not None else None
    return state.base if state is not None else None


# --- --- --- --- --- SOLUTIONS --- --- --- --- ---

@dataclass(frozen=True)
class Frame:
    """One trajectory row of a solution: a movement's scene with one robot moved.

    Attributes:
        movement: The movement it belongs to.
        robot: The robot the row moves.
        joints: Its joint values.
        base: The base the planner chose (the design had none), or None to keep the scene's.
        index, count: Its place in the movement's trajectory (0-based) and the trajectory's length.
    """

    movement: Movement
    robot: str
    joints: Dict[str, float]
    base: Optional[Pose]
    index: int
    count: int

    @property
    def label(self) -> str:
        return f"{self.movement.id} {self.index + 1}/{self.count}"


def trajectory_frames(design: Design, solutions: dict, bar: str) -> List[Frame]:
    """Every trajectory row of a bar's actions (jointing, hold, release, hold release), in schedule order.

    Args:
        design: The design the solutions belong to.
        solutions: Action id -> `Solution` (`bar_assembly_core.design.solutions.read_solutions`).
        bar: E.g. "bars/B3".
    """
    frames: List[Frame] = []
    for action, m in design.movements():
        if action.bar != bar or action.id not in solutions:
            continue
        result = solutions[action.id].movements.get(m.id)
        trajectory = result.trajectory if result is not None else None
        if trajectory is None:
            continue
        base = result.bases.get(trajectory.robot)
        rows = trajectory.positions
        frames += [Frame(m, trajectory.robot, dict(zip(trajectory.joint_names, row)), base, i, len(rows))
                   for i, row in enumerate(rows)]
    return frames


def solved_keyframes(design: Design, solutions: dict, bar: str) -> Optional[dict]:
    """The assembly keyframe a bar's solutions imply, as Rhino stores it; None without a solved insert.

    approach = the end of the transfer, assembled = the end of the insert, retreat = the end of the retreat; the base
    is the one the planner chose, else the design's.

    Returns:
        dict | None: `{"base": Pose, "approach": joints, "assembled": joints, "retreat": joints | None}`.
    """
    p = short(bar)
    jointing, release = solutions.get(f"{p}_J"), solutions.get(f"{p}_R")
    if jointing is None:
        return None
    robot = design.actions[f"{p}_J"].robot

    def end(solution, movement_id):
        result = solution.movements.get(movement_id) if solution is not None else None
        return result.end.get(robot) if result is not None and result.status == "solved" else None

    approach, assembled = end(jointing, f"{p}_J_transfer"), end(jointing, f"{p}_J_insert")
    if not assembled:
        return None
    if not approach:
        insert = jointing.movements.get(f"{p}_J_insert")
        approach = insert.start.get(robot) if insert is not None else None
    base = next((r.bases[robot] for s in (jointing, release) if s is not None for r in s.movements.values()
                 if robot in r.bases), None) or base_of(design, bar)
    if not approach or base is None:
        return None
    return {"base": base, "approach": approach, "assembled": assembled, "retreat": end(release, f"{p}_R_retreat")}


def arm_groups(joints: Dict[str, float]) -> Dict[str, dict]:
    """A dual-arm joint map as Rhino's keyframe record: `{"left": {joint_names, joint_values}, "right": {...}}`.

    Each arm's joints in UR order (shoulder pan ... wrist 3); joints outside it follow, sorted.
    """
    from bar_assembly_core.urdf import UR_JOINT_NAMES

    out = {}
    for side in ("left", "right"):
        mine = [name for name in joints if name.startswith(f"{side}_")]
        ordered = [f"{side}_ur_arm_{j}" for j in UR_JOINT_NAMES if f"{side}_ur_arm_{j}" in joints]
        names = ordered + sorted(set(mine) - set(ordered))
        out[side] = {"joint_names": names, "joint_values": [float(joints[n]) for n in names]}
    return out
