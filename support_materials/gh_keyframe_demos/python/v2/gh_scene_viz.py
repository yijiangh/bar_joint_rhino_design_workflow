#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: compas==2.13.0
# r: compas_robots==0.6.0
"""C5: cached SceneObject draw of the dual-arm robot at `ik_state`.

Inputs:
    ik_state    : RobotCellState (from C4)
    vis_options : compas Data object whose `__data__` is forwarded to
                  Scene.add(robot_cell, **kwargs). Provide a default empty
                  one (e.g. compas.data.Data subclass instance) on first
                  wire; the SceneObject is cached against its sha256 so
                  repeated draws are fast.
    single_arm  : bool, default False -- True for the single-arm support cell

Outputs:
    draw                 : list of Rhino guids drawn this tick
    tool0_left_plane     : Rhino.Geometry.Plane
    tool0_right_plane    : Rhino.Geometry.Plane

Adapted from `support_materials/gh_keyframe_demos/python/GH_scene_viz.py`.
The robot_cell is fetched from `core.robot_cell.get_or_load_robot_cell()`
so it is the same instance used by C4 -- this keeps the SceneObject cache
key valid even after env-RB updates.
"""

import os
import sys

import Rhino
import scriptcontext as sc

REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from compas.scene import Scene  # noqa: E402
from compas.scene import register as _scene_register  # noqa: E402
from compas.geometry import Frame, Transformation  # noqa: E402
from compas_rhino.conversions import frame_to_rhino_plane  # noqa: E402
from compas_ghpython import create_id  # noqa: E402

from core import robot_cell as _rc  # noqa: E402


def _ensure_gh_registration():
    """Make sure `compas.scene.Scene` knows how to draw `RobotCell` in
    Grasshopper context.

    `compas_fab.ghpython.scene` exposes a `@plugin(requires=["Rhino"])`
    function `register_scene_objects()` that maps `RobotCell` ->
    `RobotCellObject` for context "Grasshopper". That registration is
    gated by `if compas.RHINO:` -- under Rhino 8 CPython that flag is
    sometimes False, so the gate skips and Scene.add(robot_cell) raises
    `SceneObjectNotRegisteredError`. Force the import and the explicit
    `register(...)` here so we don't depend on plugin auto-discovery.
    """
    try:
        from compas_fab.ghpython.scene import (  # noqa: F401
            RobotCellObject, RigidBodyObject, ReachabilityMapObject,
        )
        from compas_fab.robots import RobotCell, RigidBody, ReachabilityMap
        _scene_register(RobotCell, RobotCellObject, context="Grasshopper")
        _scene_register(RigidBody, RigidBodyObject, context="Grasshopper")
        _scene_register(ReachabilityMap, ReachabilityMapObject, context="Grasshopper")
    except Exception as exc:
        print(f"gh_scene_viz: registration failed ({exc}); Scene.add will likely raise.")


draw = []
tool0_left_plane = None
tool0_right_plane = None

if ik_state is None:
    print("gh_scene_viz: ik_state is None; nothing to draw.")
else:
    _ensure_gh_registration()
    rcell = _rc.get_or_load_robot_cell()

    object_hash = str(id(rcell))
    options_hash = vis_options.sha256(True) if vis_options is not None else "default"
    gh_component = ghenv.Component
    sticky_id = create_id(gh_component, f"{object_hash}_{options_hash}_{bool(single_arm)}")

    scene_object = sc.sticky.get(sticky_id)
    if scene_object is None or getattr(scene_object, "item", None) is not rcell:
        scene = Scene()
        kwargs = vis_options.__data__ if vis_options is not None else {}
        scene_object = scene.add(rcell, **kwargs)
        sc.sticky[sticky_id] = scene_object

    draw = scene_object.draw(ik_state)

    world_from_base = Transformation.from_frame(ik_state.robot_base_frame)
    if not single_arm:
        left_link = "left_ur_arm_tool0"
        right_link = "right_ur_arm_tool0"
        left_fk = rcell.robot_model.forward_kinematics(ik_state.robot_configuration, link_name=left_link)
        right_fk = rcell.robot_model.forward_kinematics(ik_state.robot_configuration, link_name=right_link)
        tool0_left_plane = frame_to_rhino_plane(
            Frame.from_transformation(world_from_base * Transformation.from_frame(left_fk))
        )
        tool0_right_plane = frame_to_rhino_plane(
            Frame.from_transformation(world_from_base * Transformation.from_frame(right_fk))
        )
    else:
        single_link = "ur_arm_tool0"
        single_fk = rcell.robot_model.forward_kinematics(ik_state.robot_configuration, link_name=single_link)
        tool0_left_plane = frame_to_rhino_plane(
            Frame.from_transformation(world_from_base * Transformation.from_frame(single_fk))
        )
