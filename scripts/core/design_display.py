"""
Draw a design's scenes in Rhino: the core's drawing cells (`CompasFabCell(visual=True)`), one per acting robot.

A scene comes from the design (`Design.scene_at`, `scene_after`); `place` moves a robot in it (a solved keyframe, a
trajectory row, a ghost at the origin); `cell_and_state` turns it into the cell and state `core.ik_viz` draws. No
PyBullet: the Grasshopper preview uses this too.

- ! Keep the cells in `store` (Rhino's `sc.sticky`) and build the design with the same geometry cache every time
  (`design_source.geometry_cache`): a cell rebuilds (seconds) only when a geometry or robot object changes.
- Needs the core's `[mirrors]` stack without PyBullet (compas, compas_robots, compas_fab).
"""

from __future__ import annotations

from typing import Mapping, Optional, Tuple

from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path)
from bar_assembly_core.geometry import Pose
from bar_assembly_core.kinematics import link_pose
from bar_assembly_core.scene import Scene, world_poses

#: `sc.sticky` key of the drawing cells, robot id -> CompasFabCell.
CELLS_KEY = "bar_joint:core_display_cells"


def display_cell(store: dict, robot_id: str):
    """The drawing cell of one acting robot, kept in `store` between commands."""
    from bar_assembly_core.mirrors.cell import CompasFabCell

    cells = store.setdefault(CELLS_KEY, {})
    cell = cells.get(robot_id)
    if cell is None:
        cell = cells[robot_id] = CompasFabCell(robot_id, visual=True)
    return cell


def forget_cells(store: dict) -> None:
    """Drop every drawing cell kept in `store` (the next draw reloads the robot models)."""
    store.pop(CELLS_KEY, None)


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


def cell_and_state(store: dict, scene: Scene, robot_id: str, log=print) -> Tuple[object, object]:
    """The drawing cell of `robot_id`, rebuilt only if the scene's models changed, and the scene as its state.

    Robots absent from the scene, or whose base is unknown, are parked far away (`ik_viz` hides them); unknown joints
    are 0.

    Returns:
        tuple: (compas_fab RobotCell, RobotCellState) for `core.ik_viz.update_state`.
    """
    cells = display_cell(store, robot_id)
    why = cells.update(scene)
    if why is not None:
        log(f"core.design_display: rebuilt the drawing cell for {robot_id}; why: {why}")
    return cells.cell, cells.state(scene, strict=False)


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
