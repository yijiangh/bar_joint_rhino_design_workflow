"""
Draw a design's scenes in Rhino: what `core.ik_viz` draws of a core `Scene`, part by part.

A scene comes from the design (`Design.scene_at`, `scene_after`); `place` moves a robot in it (a solved keyframe, a
trajectory row, a ghost at the origin); `drawn_parts` says what of it is shown, and where. No PyBullet and no
compas: the Grasshopper preview uses this too.

- Parts are the core's (`bar_assembly_core.scene.scene_parts`): bodies, robot links with shapes
  ("robots/cindy/<link>") and mounted tools ("tools/<name>", at their flange). Their `Geometry` objects are shared
  and never change, so `ik_viz` bakes a part again only when its object does.
- ! Build the design with the same geometry cache every time (`design_source.geometry_cache`), or every body's
  geometry is new and is baked again.
"""

from __future__ import annotations

from typing import Dict, Iterable, Mapping, NamedTuple, Optional

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.geometry import Geometry, Pose
from bar_assembly_core.ids import link_id
from bar_assembly_core.kinematics import link_pose
from bar_assembly_core.scene import Attachment, Scene, robot_parts, world_poses


class Drawn(NamedTuple):
    """One part of a scene as `ik_viz` draws it.

    Attributes:
        geometry: Its shapes, in its own frame.
        pose: Its world pose; None: hidden (disabled, or its robot is parked).
        owner: The robot id it belongs to (a link or a mounted tool), or None for a body.
    """

    geometry: Geometry
    pose: Optional[Pose]
    owner: Optional[str]


def refresh(scene: Scene) -> None:
    """Recompute the world pose of every body, after robots or placements changed."""
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))


def place(scene: Scene, robot_id: str, base: Optional[Pose] = None, joints: Optional[Mapping[str, float]] = None,
          *, enabled: bool = True, update: bool = True) -> None:
    """Put a robot into a scene, in place: present, at `base` (if given) with `joints` (if given) known.

    Args:
        scene: The scene to change.
        robot_id: E.g. "robots/cindy".
        base: The robot's base pose; None keeps the scene's.
        joints: Joint values to set; the rest keep their values.
        enabled: False: take the robot out of the scene instead.
        update: Recompute the bodies' world poses (bodies held by the robot move with it).
    """
    robot = scene.robots[robot_id]
    robot.enabled = enabled
    if base is not None:
        robot.base, robot.base_tracked = base, True
    if joints:
        robot.joints = {**robot.joints, **{name: float(value) for name, value in joints.items()}}
        robot.unmeasured = frozenset(robot.unmeasured - set(joints))
    if update:
        refresh(scene)


def parked(scene: Scene, acting: Iterable[str] = ()) -> set:
    """Robots in the scene that are not drawn: absent, base unknown, or (unless acting) joints unknown.

    An acting robot is drawn with its unknown joints at their guess (0), as the planners' drawing did.

    Args:
        scene: The scene.
        acting: Robot ids drawn even with unknown joints.
    """
    acting = set(acting)
    return {robot_id for robot_id, robot in scene.robots.items()
            if not robot.enabled or not robot.base_tracked or (robot.unmeasured and robot_id not in acting)}


def _holder(scene: Scene, body_id: str) -> Optional[str]:
    """The robot a body hangs from through its chain of attachments, or None if it stands in the world."""
    seen = set()
    while body_id in scene.bodies and body_id not in seen:
        seen.add(body_id)
        placement = scene.bodies[body_id].placement
        if not isinstance(placement, Attachment):
            return None
        if placement.parent in scene.robots:
            return placement.parent
        body_id = placement.parent
    return None


def drawn_parts(scene: Scene, acting: Iterable[str] = (), hide_bodies: bool = False) -> Dict[str, Drawn]:
    """Every part of every body and robot in the scene, by id, with where it is drawn (None: hidden).

    Hidden: disabled bodies, bodies without a world pose, bodies held by a parked robot, every part of a parked
    robot (`parked`), and every body if `hide_bodies` (the document's own geometry shows the bars).

    Args:
        scene: The scene; it is not modified.
        acting: Robot ids drawn even with unknown joints.
        hide_bodies: Draw the robots and their tools only.
    """
    away = parked(scene, acting)
    out: Dict[str, Drawn] = {}
    for body_id, body in scene.bodies.items():
        shown = (not hide_bodies and body.enabled and body_id in scene.world_poses
                 and _holder(scene, body_id) not in away)
        out[body_id] = Drawn(body.geometry, scene.world_poses[body_id] if shown else None, None)
    for robot_id, robot in scene.robots.items():
        if robot_id in away:
            model = robot.model
            for link, geometry in model.link_geometry.items():
                out[link_id(robot_id, link)] = Drawn(geometry, None, robot_id)
            for tool in model.tools.values():
                out[tool.id] = Drawn(tool.geometry, None, robot_id)
            continue
        for part_id, geometry, pose in robot_parts(robot):
            out[part_id] = Drawn(geometry, pose, robot_id)
    return out


def robot_scene(design, robot_id: str, joints: Optional[Mapping[str, float]] = None) -> Scene:
    """A scene of one design robot alone at the world origin (every body hidden), even before any action exists.

    Args:
        design: The design.
        robot_id: The robot shown; the others are absent.
        joints: Its joint values (default 0).
    """
    from bar_assembly_core.design.scenes import design_model
    from bar_assembly_core.robot import RobotObject
    from bar_assembly_core.scene import Body

    robots = {rid: RobotObject(rid, design_model(design, rid), Pose(), {}, enabled=rid == robot_id,
                               label=spec.name)
              for rid, spec in design.robots.items()}
    bodies = {bid: Body(bid, spec.geometry, spec.pose, label=spec.label, enabled=False)
              for bid, spec in design.bodies.items()}
    scene = Scene(bodies=bodies, robots=robots)
    place(scene, robot_id, Pose(), joints or {})
    return scene


def ghost_scene(scene: Scene, robot_id: str, joints: Optional[Mapping[str, float]] = None) -> Scene:
    """A copy of a scene with only `robot_id` present, at the world origin, at `joints` (default 0).

    The ghost previews (base picking) slide such a robot's meshes onto a candidate base as a rigid transform.
    """
    ghost = scene.copy()
    for other in ghost.robots:
        ghost.robots[other].enabled = other == robot_id
    robot = ghost.robots[robot_id]
    robot.joints = {name: 0.0 for name in robot.model.movable_joints}
    place(ghost, robot_id, Pose(), joints or {})
    return ghost
