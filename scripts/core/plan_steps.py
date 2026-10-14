"""
The steps the plan viewers show, as moments of a schema 2 process: the poses per bar, the whole timeline, and the
trajectories of a process folder's solutions.

Rhino-free (numpy and the core only); `core.process_display` draws the scenes.

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

from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from scaffolding_assembly_core.process import Process, Movement
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.scene import Scene

from core.process_display import place, refresh

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
    """One moment a viewer shows: the start (or end) of one process movement, named by its host bar's pose.

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


def find_movement(process: Process, action_id: str, movement_id: str) -> Optional[Movement]:
    """A movement by its action and movement ids, or None."""
    action = process.actions.get(action_id)
    if action is None:
        return None
    return next((m for m in action.movements if m.id == movement_id), None)


def movement_after(process: Process, movement: Movement) -> Optional[Movement]:
    """The movement that follows one in the schedule, or None after the last."""
    found = False
    for _action, m in process.movements():
        if found:
            return m
        found = m is movement
    return None


def releases_after(process: Process, bar: str) -> List[str]:
    """The held bars whose release follows this bar's actions in the schedule (in release order)."""
    order = list(process.schedule)
    after = [i for i, a in enumerate(order) if process.actions[a].bar == bar and process.actions[a].type == "bar_release"]
    out = []
    for action_id in order[after[0] + 1:] if after else []:
        action = process.actions[action_id]
        if action.type != "bar_holding_release":
            break
        out.append(action.bar)
    return out


def _known(movement: Optional[Movement], robot: str) -> bool:
    """Whether a robot's joints are known at the start of a movement."""
    state = movement.start.robots.get(robot) if movement is not None else None
    return state is not None and state.joints is not None


def is_solved(process: Process, bar: str) -> bool:
    """Whether the bar's assembly keyframe is in the process: the assembling robot's joints known at the insert."""
    p = short(bar)
    action = process.actions.get(f"{p}_J")
    return action is not None and _known(find_movement(process, f"{p}_J", f"{p}_J_insert"), action.robot)


def bar_steps(process: Process, bar: str) -> List[Step]:
    """The ordered steps of one bar (see the module docstring); empty for a bar the process does not assemble."""
    p = short(bar)
    jointing = process.actions.get(f"{p}_J")
    if jointing is None:
        return []
    steps: List[Step] = []
    if is_solved(process, bar):
        insert = find_movement(process, f"{p}_J", f"{p}_J_insert")
        steps.append(Step(bar, "approach", insert))
        after = movement_after(process, jointing.movements[-1])
        steps.append(Step(bar, "assembled", after) if after is not None else Step(bar, "assembled", insert, True))
        hold = process.actions.get(f"{p}_H")
        ungrasp = find_movement(process, f"{p}_R", f"{p}_R_ungrasp")
        if hold is not None and _known(ungrasp, hold.robot):
            steps.append(Step(bar, HOLD, ungrasp))
        home = find_movement(process, f"{p}_R", f"{p}_R_home")
        if home is not None:
            steps.append(Step(bar, "retreat", home))
            steps.append(Step(bar, "home", home, True))
    for held in releases_after(process, bar):
        h = short(held)
        retreat = find_movement(process, f"{h}_HR", f"{h}_HR_retreat")
        if retreat is not None:
            steps.append(Step(bar, (RELEASE, held), retreat, True))
    return steps


def timeline(process: Process) -> Tuple[List[Step], List[Tuple[str, str]]]:
    """Every step of the whole assembly in schedule order, and the bars that add no assembly step.

    Returns:
        tuple: (steps, skipped) with skipped `[(bar id, reason)]`: bars whose keyframe is unsolved (their release
        steps are kept: the holding robots must not vanish between two steps).
    """
    steps, skipped = [], []
    for action_id in process.schedule:
        action = process.actions[action_id]
        if action.type != "bar_jointing":
            continue
        own = bar_steps(process, action.bar)
        steps += own
        if not is_solved(process, action.bar):
            reason = "no solved IK keyframe"
            if own:
                reason += f" (its release step(s) for {', '.join(short(s.pose[1]) for s in own)} are kept)"
            skipped.append((action.bar, reason))
    return steps, skipped


def step_scene(process: Process, step: Step) -> Scene:
    """The world at one step: the movement's start, or its end with the target joints applied."""
    scene = process.scene_at(step.movement)
    target = step.movement.target
    if step.at_end and target is not None:
        for robot, joints in target.joints.items():
            place(scene, robot, joints=joints, update=False)
        refresh(scene)
    return scene


def base_of(process: Process, bar: str) -> Optional[Pose]:
    """The assembling robot's base for a bar (from the jointing action), or None before it is picked."""
    p = short(bar)
    jointing = process.actions.get(f"{p}_J")
    insert = find_movement(process, f"{p}_J", f"{p}_J_insert")
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
        base: The base the planner chose (the process had none), or None to keep the scene's.
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


def trajectory_frames(process: Process, solutions: dict, bar: str) -> List[Frame]:
    """Every trajectory row of a bar's actions (jointing, hold, release, hold release), in schedule order.

    Args:
        process: The process the solutions belong to.
        solutions: Action id -> `Solution` (`scaffolding_assembly_core.process.solutions.read_solutions`).
        bar: E.g. "bars/B3".
    """
    frames: List[Frame] = []
    for action, m in process.movements():
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


def solved_keyframes(process: Process, solutions: dict, bar: str) -> Optional[dict]:
    """The assembly keyframe a bar's solutions imply, as Rhino stores it; None without a solved insert.

    approach = the end of the transfer, assembled = the end of the insert, retreat = the end of the retreat; the base
    is the one the planner chose, else the process's.

    Returns:
        dict | None: `{"base": Pose, "approach": joints, "assembled": joints, "retreat": joints | None}`.
    """
    p = short(bar)
    jointing, release = solutions.get(f"{p}_J"), solutions.get(f"{p}_R")
    if jointing is None:
        return None
    robot = process.actions[f"{p}_J"].robot

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
                 if robot in r.bases), None) or base_of(process, bar)
    if not approach or base is None:
        return None
    return {"base": base, "approach": approach, "assembled": assembled, "retreat": end(release, f"{p}_R_retreat")}


def arm_groups(joints: Dict[str, float]) -> Dict[str, dict]:
    """A dual-arm joint map as Rhino's keyframe record: `{"left": {joint_names, joint_values}, "right": {...}}`.

    Each arm's joints in UR order (shoulder pan ... wrist 3); joints outside it follow, sorted.
    """
    from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

    out = {}
    for side in ("left", "right"):
        mine = [name for name in joints if name.startswith(f"{side}_")]
        ordered = [f"{side}_ur_arm_{j}" for j in UR_JOINT_NAMES if f"{side}_ur_arm_{j}" in joints]
        names = ordered + sorted(set(mine) - set(ordered))
        out[side] = {"joint_names": names, "joint_values": [float(joints[n]) for n in names]}
    return out


def changed_since_planning(process_now: Process, solutions: dict, package_dirs=()) -> List[str]:
    """The solved actions that differ in a process held in memory (e.g. the Rhino document now) from what was planned.

    Compares each action's hash in `process_now` (`process.process_hashes`, nothing written) with the one its solution
    recorded (`solved_against.action`). ? Only action hashes: `process.json` records the producer (commit, command), so
    its hash differs between any two exports.

    Returns:
        list[str]: Action ids, sorted: changed since planning, or no longer in the process.
    """
    from scaffolding_assembly_core.process import process_hashes

    _process_hash, actions = process_hashes(process_now, package_dirs)
    return sorted(action_id for action_id, solution in solutions.items()
                  if actions.get(action_id) != solution.solved_against.action)
