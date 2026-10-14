"""
Build a schema 2 process's actions from Rhino's per-bar keyframe records, with one running state through the schedule.

Rhino-free (numpy and the core only), so pytest covers it. `core.process_source` reads the records from the document.
Procedure per action (`docs/schema2_integration.md`):

- J (Cindy): load, place_bar (the operator puts the bar into the tools), grasp, transfer, insert (one movement that drives `tighten`; the bar is built at its end);
- R (Cindy), form B: ungrasp (drives `loosen`: the untighten folded in, the bar stays built), linear retreat, home;
  then she drives away (absent until her next J);
- H (support robot): free approach, open, linear onto the bar, close (it now holds the built bar too);
- HR: open, linear retreat, leave (free, planned live); then the robot drives away.

`on` is written in targets, never changed between movements (B11): the place_bar target puts Cindy's tools on their
halves, the support's linear approach puts the gripper on its bar, and the retreats (R and HR) clear it (`None`).
Robots arriving and driving away between actions stay changes of the running state.

`poses_in_tcp` gives, per kind of half a tool acts on, the half's pose in the frame of the tool centre point on it
(a body's `pose_in_tcp`, format §4.3).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from scaffolding_assembly_core.process import (Action, Process, Holder, LineSpec, Movement, RobotState, State, Target,
                                      ToolState)
from scaffolding_assembly_core.geometry import Pose, compose, invert

#: Schedule kind -> action type and the suffix of its action id.
KINDS = {"J": ("bar_jointing", "J"), "R": ("bar_release", "R"), "H": ("bar_holding", "H"),
         "HR": ("bar_holding_release", "HR")}

Joints = Dict[str, float]
Vector = Tuple[float, float, float]


@dataclass(frozen=True)
class BarRecord:
    """What Rhino stores for one bar Cindy assembles, in metres; joints are None until IK solved them.

    Attributes:
        bar: Bar id, e.g. "bars/B10".
        robot: The assembly robot's id.
        tools: Flange link id -> its tool id.
        tool0: Flange link id -> the flange's world pose at the assembled keyframe (the placed tool block).
        on: Flange link id -> the half its tool is on (a male or a ground half of this bar).
        insert_direction: Unit world direction the bar moves during the insert.
        retreat_direction: Flange link id -> unit world direction of its retreat.
        base: Cindy's base (URDF root link pose), or None before IK.
        approach, assembled, retreat: Joint values at the keyframes, or None.
        home: Joint values Cindy returns to.
        ground: Ground ids the base may stand on.
    """

    bar: str
    robot: str
    tools: Dict[str, str]
    tool0: Dict[str, Pose]
    on: Dict[str, str]
    insert_direction: Vector
    retreat_direction: Dict[str, Vector]
    base: Optional[Pose] = None
    approach: Optional[Joints] = None
    assembled: Optional[Joints] = None
    retreat: Optional[Joints] = None
    home: Optional[Joints] = None
    ground: Tuple[str, ...] = ()


@dataclass(frozen=True)
class HoldRecord:
    """What Rhino stores for one hold of a built bar by a support robot, in metres.

    Attributes:
        bar: The held bar.
        robot: The support robot's id.
        flange: Its flange link id; `tool` the tool mounted there.
        tool0: The flange's world pose when it holds the bar.
        grasp: The bar's pose in the flange frame.
        base, approach, held: Base and joint values at the approach and the grasp, or None before IK.
        supports_until: Bars that must be built before the hold is released.
    """

    bar: str
    robot: str
    flange: str
    tool: str
    tool0: Pose
    grasp: Pose
    base: Optional[Pose] = None
    approach: Optional[Joints] = None
    held: Optional[Joints] = None
    supports_until: Tuple[str, ...] = ()
    ground: Tuple[str, ...] = ()


@dataclass
class BuildResult:
    """The process with its actions, and what the builder had to leave out (one line each)."""

    process: Process
    problems: List[str] = field(default_factory=list)


def shifted(pose: Pose, direction: Sequence[float], distance: float) -> Pose:
    """A pose moved by `distance` along a world `direction`, its orientation kept."""
    return Pose.from_arrays(np.add(pose.position, np.multiply(direction, distance)), pose.orientation)


def z_axis(pose: Pose) -> Vector:
    """The world direction of a pose's z axis."""
    return tuple(float(v) for v in pose.matrix()[:3, 2])


def grasp_of(tool0: Pose, body: Pose) -> Pose:
    """A body's pose in a flange's frame, both given in world."""
    return compose(invert(tool0), body)


class Builder:
    """One running state threaded through the schedule; each movement starts where the last one ended."""

    def __init__(self, process: Process, insert_distance: float, retreat_distance: float, support_distance: float):
        """Start with every robot and tool absent, and the ground and the obstacles present."""
        self.process = process
        self.insert_distance, self.retreat_distance = insert_distance, retreat_distance
        self.support_distance = support_distance
        grounds = frozenset(body for body in process.bodies if body.startswith(("ground/", "obstacles/")))
        tools = {tool: None for robot in process.robots.values() for tool in robot.tools.values()}
        self.state = State(robots={robot: None for robot in process.robots}, present=grounds, tools=tools)
        self.movements: List[Movement] = []

    def emit(self, id_: str, label: str, target: Optional[Target] = None, **parts) -> None:
        """Add a movement starting at the running state, then advance the state to its end."""
        self.movements.append(Movement(id=id_, label=label, start=self.state, target=target, **parts))
        s = self.state
        if target is None:
            return
        robots = dict(s.robots)
        for robot, joints in target.joints.items():
            robots[robot] = RobotState(robots[robot].base, dict(joints))
        for arm in parts.get("arms", ()):  # * an arm motion without target joints ends unknown
            robot = "/".join(arm.split("/")[:2])
            if robot not in target.joints and robots[robot] is not None:
                robots[robot] = RobotState(robots[robot].base, None)
        tools = {t: (replace(v, grip=target.tools[t]) if t in target.tools else v) for t, v in s.tools.items()}
        for tool, body in target.on.items():
            if tools.get(tool) is not None:
                tools[tool] = replace(tools[tool], on=body)
        attached = s.attached if target.attached is None else target.attached
        built = s.built if target.built is None else target.built
        self.state = replace(s, robots=robots, tools=tools, attached=dict(attached), built=frozenset(built),
                             present=s.present | set(attached) | built)

    def set(self, **changes) -> None:
        """Change the running state between movements: what a target cannot say (`on`, robots arriving)."""
        self.state = replace(self.state, **changes)

    def arrive(self, robot: str, base: Optional[Pose], tools: Dict[str, ToolState]) -> None:
        """A robot drives to `base` (its joints unknown); its tools keep their state, or take `tools` if absent."""
        robots, states = dict(self.state.robots), dict(self.state.tools)
        robots[robot] = RobotState(base, None)
        for tool, default in tools.items():
            states[tool] = states[tool] or default
        self.set(robots=robots, tools=states)

    def without(self, bar: str, links: Sequence[str]) -> Dict[str, Tuple[Holder, ...]]:
        """The attached map with `links` no longer holding `bar`."""
        attached = {k: v for k, v in self.state.attached.items() if k != bar}
        rest = tuple(h for h in self.state.attached.get(bar, ()) if h.to not in links)
        if rest:
            attached[bar] = rest
        return attached

    # * Cindy: jointing (merged insert) and release (form B, no untighten).

    def placing(self, bar: str) -> None:
        """A bar no robot handles: one manual step, the bar is in the structure at its end."""
        self.emit(f"{bar.split('/')[1]}_P_place", "Placed by hand", ends_on="operator",
                  target=Target(built=self.state.built | {bar}))

    def jointing(self, r: BarRecord) -> None:
        """Load, place_bar, grasp, transfer, insert."""
        p, arms = r.bar.split("/")[1], tuple(sorted(r.tool0))
        tools = {r.tools[a]: ToolState("open", None) for a in arms}
        self.arrive(r.robot, r.base, tools)
        bar_pose = self.process.bodies[r.bar].assembly_pose
        approach_links = {a: shifted(r.tool0[a], r.insert_direction, -self.insert_distance) for a in arms}

        def joints(values: Optional[Joints]) -> Dict[str, Joints]:
            return {r.robot: values} if values else {}

        self.emit(f"{p}_J_load", "Free to the loading pose", arms=arms, path="free", controller="position")
        holders = tuple(Holder(a, grasp_of(r.tool0[a], bar_pose)) for a in arms)
        self.emit(f"{p}_J_place_bar", "Operator places the bar", ends_on="operator",
                  target=Target(attached={**self.state.attached, r.bar: holders},
                                on={r.tools[a]: r.on[a] for a in arms}))
        self.emit(f"{p}_J_grasp", "Grasp", ends_on="tools", target=Target(tools={r.tools[a]: "closed" for a in arms}))
        self.emit(f"{p}_J_transfer", "Transfer to the approach", arms=arms, path="free", coupled=True,
                  controller="position", target=Target.of(joints=joints(r.approach), link_poses=approach_links))
        self.emit(f"{p}_J_insert", "Insert", arms=arms, path="linear", coupled=True, controller="compliant",
                  ends_on="tools", drives={r.tools[a]: "tighten" for a in arms},
                  line={a: LineSpec(r.insert_direction, self.insert_distance) for a in arms},
                  target=Target.of(joints=joints(r.assembled), link_poses=dict(r.tool0), built=self.state.built | {r.bar}))

    def release(self, r: BarRecord) -> None:
        """Ungrasp (form B, backing the jointing screw off), careful retreat, home; then the robot drives away."""
        p, arms = r.bar.split("/")[1], tuple(sorted(r.tool0))
        self.emit(f"{p}_R_ungrasp", "Ungrasp (form B)", ends_on="tools",
                  drives={r.tools[a]: "loosen" for a in arms},
                  target=Target(tools={r.tools[a]: "open" for a in arms}, attached=self.without(r.bar, arms)))
        retreat_links = {a: shifted(r.tool0[a], r.retreat_direction[a], self.retreat_distance) for a in arms}
        self.emit(f"{p}_R_retreat", "Careful retreat", arms=arms, path="linear", controller="position",
                  line={a: LineSpec(r.retreat_direction[a], self.retreat_distance) for a in arms},
                  target=Target.of(joints={r.robot: r.retreat} if r.retreat else {}, link_poses=retreat_links,
                                on={r.tools[a]: None for a in arms}))
        self.emit(f"{p}_R_home", "Home", arms=arms, path="free", controller="position",
                  target=Target.of(joints={r.robot: r.home} if r.home else {}))
        # * Cindy drives away to load her next bar: absent until her next jointing arrives. Left standing here, she
        #   would be an obstacle in the support robots' hold releases, where she no longer is.
        robots = dict(self.state.robots)
        robots[r.robot] = None
        self.set(robots=robots)

    # * Support robot: hold (grasps a built bar Cindy still holds) and release.

    def holding(self, h: HoldRecord) -> None:
        """Free approach, open, linear onto the bar, close."""
        p = h.bar.split("/")[1]
        robots, tools = dict(self.state.robots), dict(self.state.tools)
        robots[h.robot] = RobotState(h.base, None)
        tools[h.tool] = ToolState(None, None)  # * arrives; its grip is unknown until it opens
        self.set(robots=robots, tools=tools)
        down = z_axis(h.tool0)
        approach_link = shifted(h.tool0, down, -self.support_distance)
        self.emit(f"{p}_H_approach", "Free to the approach", arms=(h.flange,), path="free", controller="position",
                  target=Target.of(joints={h.robot: h.approach} if h.approach else {}, link_poses={h.flange: approach_link}))
        self.emit(f"{p}_H_open", "Gripper opens", ends_on="tools", target=Target(tools={h.tool: "open"}))
        self.emit(f"{p}_H_to_grasp", "Linear onto the bar", arms=(h.flange,), path="linear", controller="position",
                  line={h.flange: LineSpec(down, self.support_distance)},
                  target=Target.of(joints={h.robot: h.held} if h.held else {}, link_poses={h.flange: h.tool0},
                                on={h.tool: h.bar}))
        attached = dict(self.state.attached)
        attached[h.bar] = attached.get(h.bar, ()) + (Holder(h.flange, h.grasp),)
        self.emit(f"{p}_H_close", "Gripper closes", ends_on="tools",
                  target=Target(tools={h.tool: "closed"}, attached=attached))

    def holding_release(self, h: HoldRecord) -> None:
        """Open, linear retreat, leave; then the robot drives away."""
        p = h.bar.split("/")[1]
        up = tuple(-v for v in z_axis(h.tool0))
        self.emit(f"{p}_HR_open", "Gripper opens", ends_on="tools",
                  target=Target(tools={h.tool: "open"}, attached=self.without(h.bar, (h.flange,))))
        self.emit(f"{p}_HR_retreat", "Linear retreat", arms=(h.flange,), path="linear", controller="position",
                  line={h.flange: LineSpec(up, self.support_distance)},
                  target=Target.of(joints={h.robot: h.approach} if h.approach else {},
                                link_poses={h.flange: shifted(h.tool0, up, self.support_distance)},
                                on={h.tool: None}))
        self.emit(f"{p}_HR_leave", "Free away from the structure", arms=(h.flange,), path="free",
                  controller="position")
        robots, tools = dict(self.state.robots), dict(self.state.tools)
        robots[h.robot], tools[h.tool] = None, None  # * drives away
        self.set(robots=robots, tools=tools)


def build_actions(process: Process, schedule: Sequence[Tuple[str, str]], bars: Dict[str, BarRecord],
                  holds: Dict[str, HoldRecord], *, insert_distance: float = 0.015, retreat_distance: float = 0.05,
                  support_distance: float = 0.1) -> BuildResult:
    """Add the actions and the schedule to a process that holds its robots, tools, bodies and mates.

    Args:
        process: The process without actions.
        schedule: (kind, bar id) in execution order; kind is "J", "R", "H" or "HR".
        bars: Bar id -> its record, for J and R.
        holds: Held bar id -> its record, for H and HR.
        insert_distance, retreat_distance: Cindy's insert and retreat lengths, metres.
        support_distance: The support robot's linear approach and retreat length, metres.

    - ! A bar without a record has no robot action: it gets a `bar_placing` action at its place in the schedule (where
      its jointing would be), one manual step that ends with the bar built. It is absent before and an obstacle
      after, like any other bar, so the bars after it are planned around it. Reported, because a bar that merely
      lacks its tools looks the same.

    Returns:
        BuildResult: The process, and one line per bar placed by hand and per hold left out (no record).
    """
    builder = Builder(process, insert_distance, retreat_distance, support_distance)
    actions: Dict[str, Action] = {}
    order: List[str] = []
    problems: List[str] = []
    for kind, bar in schedule:
        if kind in ("J", "R") and bar not in bars and bar in process.bodies:
            if kind == "J":
                first = len(builder.movements)
                builder.placing(bar)
                action_id = f"{bar.split('/')[1]}_P"
                actions[action_id] = Action(id=action_id, type="bar_placing", robot="", bar=bar,
                                            movements=tuple(builder.movements[first:]),
                                            label=f"{bar.split('/')[1]} placed by hand")
                order.append(action_id)
                problems.append(f"{bar}: no robot action (no keyframe record); taken as placed by hand at its place "
                                f"in the sequence")
            continue  # ? Nothing to release: no robot held it.
        record = bars.get(bar) if kind in ("J", "R") else holds.get(bar)
        if record is None:
            problems.append(f"{bar} {kind}: no {'keyframe' if kind in ('J', 'R') else 'hold'} record; "
                            f"action left out")
            continue
        first = len(builder.movements)
        {"J": builder.jointing, "R": builder.release, "H": builder.holding,
         "HR": builder.holding_release}[kind](record)
        type_, suffix = KINDS[kind]
        action_id = f"{bar.split('/')[1]}_{suffix}"
        supports = record.supports_until if kind == "H" else ()
        actions[action_id] = Action(id=action_id, type=type_, robot=record.robot, bar=bar,
                                    movements=tuple(builder.movements[first:]), ground=tuple(record.ground),
                                    supports_until=tuple(supports), label=_label(kind, bar, record.robot))
        order.append(action_id)
    return BuildResult(replace(process, actions=actions, schedule=tuple(order), folder=None), problems)


def _label(kind: str, bar: str, robot: str) -> str:
    """Display text of an action, e.g. "cindy joints B10"."""
    verb = {"J": "joints", "R": "releases", "H": "holds", "HR": "lets go of"}[kind]
    return f"{robot.split('/')[1]} {verb} {bar.split('/')[1]}"


def poses_in_tcp(registry, seat_of=None) -> Dict[str, Pose]:
    """Per kind of half a robotic tool acts on: the half's pose in the frame of the TCP on it (`pose_in_tcp`).

    A male's tool TCP is the male block's frame (identity); a ground joint's TCP is the block's frame turned by
    `M_tool_from_block` (the tool placement's rule, `rhino_tool_place.tool_attach_frame`), so its pose is the
    inverse. Receivers (females, MoCap halves) get no tool in Rhino's flow and so no pose.

    Args:
        registry: A `core.joint_pair.JointRegistry` (`load_joint_registry()`).
        seat_of: Block name -> `(type, subtype)` (default `joint_name_conventions.split_block_name`).

    Returns:
        dict: Part name ("T20/Male") -> the pose.
    """
    if seat_of is None:
        from core.joint_name_conventions import split_block_name as seat_of
    parts: Dict[str, Pose] = {}
    for name, half in registry.halves.items():
        if getattr(half, "kind", "") == "male":
            parts["/".join(seat_of(name))] = Pose()
    for ground in registry.ground_joints.values():
        tool_from_block = np.asarray(ground.M_tool_from_block, dtype=float).copy()
        tool_from_block[:3, 3] /= 1000.0  # * mm -> m (a pure rotation today: GroundJointDef zeroes it)
        pose = Pose.from_matrix(np.linalg.inv(tool_from_block))
        # * Exact values: a half turn comes out of the matrix with 1e-12 where 0 is meant.
        turn = np.round(pose.orientation, 9) + 0.0
        parts["/".join(seat_of(ground.block_name))] = Pose.from_arrays(np.round(pose.position, 9) + 0.0,
                                                                       turn / np.linalg.norm(turn))
    return parts
