"""Rhino-side scene cache for dual-arm robot visualization.

compas_fab wip_process only registers a Rhino scene object for
`ReachabilityMap`; `RobotCell` is Grasshopper-only. compas_robots *does*
register `RobotModel` for the Rhino context, so we draw
`robot_cell.robot_model` here and compose the `state.robot_base_frame`
ourselves as a post-bake Rhino transform.

Each `show_state()` call:
    1. Deletes any previously drawn Rhino meshes (tracked in sticky).
    2. Sets the compas scene object's `configuration` to the new joint values.
    3. Calls `scene_object.draw_visual()` which re-bakes the robot at the
       new joint configuration, with the base link at the world origin.
    4. Applies the world-from-base transform (translation converted from
       meters to doc units) to the newly drawn meshes so the robot appears
       at the intended mobile-base pose.
    5. Stores the resulting guids for cleanup on the next call.

It's not a true in-place FK update (the GH demo did that via SceneObject
caching), but it is correct and simple, and the full bake is fast enough
for interactive IK keyframing.
"""

from __future__ import annotations

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino

from core.rhino_frame_io import doc_unit_scale_to_mm
from core.robot_cell import default_cell_state, get_or_load_robot_cell, import_compas_stack


_STICKY_SCENE_OBJECT = "bar_joint:ik_viz_scene_object"
_STICKY_DRAWN_IDS = "bar_joint:ik_viz_drawn_ids"
_STICKY_MESH_MODE = "bar_joint:ik_viz_mesh_mode"

MESH_MODE_VISUAL = "visual"
MESH_MODE_COLLISION = "collision"
_VALID_MESH_MODES = (MESH_MODE_VISUAL, MESH_MODE_COLLISION)


def set_mesh_mode(mode: str) -> None:
    """Pin the mesh-display mode used by `show_state` for the rest of the session."""
    if mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh mode must be one of {_VALID_MESH_MODES}, got {mode!r}")
    sc.sticky[_STICKY_MESH_MODE] = mode


def get_mesh_mode() -> str:
    """Return the current mesh-display mode (defaults to 'visual')."""
    return sc.sticky.get(_STICKY_MESH_MODE, MESH_MODE_VISUAL)


def _meters_to_doc_scale() -> float:
    """compas geometry is in meters; convert into the active Rhino doc unit."""
    return 1000.0 / doc_unit_scale_to_mm()


def _flatten_drawn(drawn):
    """Flatten a possibly-nested result of `SceneObject.draw()` into a flat id list."""
    out = []
    if drawn is None:
        return out
    stack = [drawn]
    while stack:
        current = stack.pop()
        if current is None:
            continue
        if isinstance(current, (list, tuple, set)):
            for item in current:
                stack.append(item)
        else:
            out.append(current)
    return out


def _delete_tracked_ids() -> None:
    ids = sc.sticky.get(_STICKY_DRAWN_IDS) or []
    if not ids:
        return
    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        for oid in ids:
            try:
                rs.DeleteObject(oid)
            except Exception:
                pass
    finally:
        sc.doc.Views.RedrawEnabled = was
    sc.sticky[_STICKY_DRAWN_IDS] = []


def _ensure_rhino_registration() -> None:
    """Import `compas_robots.rhino.scene` so its Rhino SceneObject plugin registers.

    The plugin is an `@plugin(category='factories', requires=['Rhino'])` entry
    point inside `compas_robots/rhino/scene/__init__.py`. Importing the module
    is enough to trigger registration if compas's plugin discovery hasn't run
    already.
    """
    try:
        import compas_robots.rhino.scene  # noqa: F401
    except Exception as exc:  # pragma: no cover
        print(f"ik_viz: compas_robots.rhino.scene import failed ({exc}); "
              "RobotModel Rhino SceneObject may not be registered.")


def _get_or_create_scene_object(robot_model):
    """Return a cached compas `SceneObject` for this `robot_model`, creating if absent."""
    cached = sc.sticky.get(_STICKY_SCENE_OBJECT)
    if cached is not None and getattr(cached, "item", None) is robot_model:
        return cached

    _ensure_rhino_registration()
    deps = import_compas_stack()
    scene = deps["Scene"]()
    scene_object = scene.add(robot_model)
    # Scale the URDF meshes from meters to the active Rhino doc unit exactly once.
    scale_factor = _meters_to_doc_scale()
    if abs(scale_factor - 1.0) > 1e-9:
        scene_object.scale(scale_factor)
    sc.sticky[_STICKY_SCENE_OBJECT] = scene_object
    return scene_object


def _world_from_base_rhino_xform(base_frame) -> Rhino.Geometry.Transform:
    """Build a Rhino Transform from a compas Frame (meters) into doc units.

    Rotation is unit-free; translation is scaled from meters to doc units.
    """
    x = np.asarray(base_frame.xaxis, dtype=float)
    y = np.asarray(base_frame.yaxis, dtype=float)
    z = np.asarray(base_frame.zaxis, dtype=float)
    t_m = np.asarray(base_frame.point, dtype=float)
    t_doc = t_m * _meters_to_doc_scale()

    xform = Rhino.Geometry.Transform(1.0)
    xform[0, 0], xform[0, 1], xform[0, 2], xform[0, 3] = x[0], y[0], z[0], t_doc[0]
    xform[1, 0], xform[1, 1], xform[1, 2], xform[1, 3] = x[1], y[1], z[1], t_doc[1]
    xform[2, 0], xform[2, 1], xform[2, 2], xform[2, 3] = x[2], y[2], z[2], t_doc[2]
    xform[3, 0], xform[3, 1], xform[3, 2], xform[3, 3] = 0.0, 0.0, 0.0, 1.0
    return xform


def show_state(state, mesh_mode: str | None = None, *, robot_model=None) -> None:
    """Draw the robot at `state`, replacing any previously drawn meshes.

    `mesh_mode` selects which `BaseRobotModelObject` draw method runs:

    - ``"visual"``  (default) → ``scene_object.draw_visual()`` — the URDF visual meshes.
    - ``"collision"``         → ``scene_object.draw_collision()`` — the URDF collision proxies.

    If `mesh_mode` is None, falls back to the session-wide value set via
    `set_mesh_mode(...)` (default ``"visual"``).

    `robot_model` overrides the dual-arm cell default so the support cell can
    reuse this draw pipeline. The cached scene object is keyed by identity of
    the model, so swapping models recreates the SceneObject correctly.
    """
    if mesh_mode is None:
        mesh_mode = get_mesh_mode()
    if mesh_mode not in _VALID_MESH_MODES:
        raise ValueError(f"mesh_mode must be one of {_VALID_MESH_MODES}, got {mesh_mode!r}")

    if robot_model is None:
        robot_model = get_or_load_robot_cell().robot_model

    _delete_tracked_ids()

    scene_object = _get_or_create_scene_object(robot_model)
    # `update(joint_state)` runs FK and applies the per-link delta transform
    # to the cached compas Rhino meshes (`native_geometry`). Without this,
    # `draw_visual()` would re-bake the meshes at whatever pose they were
    # last left in (initially identity = robot home), regardless of the
    # `configuration` attribute. See `compas_robots.scene.baserobotmodelobject.
    # BaseRobotModelObject.update`.
    scene_object.update(state.robot_configuration)

    was = sc.doc.Views.RedrawEnabled
    try:
        sc.doc.Views.RedrawEnabled = False
        if mesh_mode == MESH_MODE_COLLISION:
            drawn = scene_object.draw_collision()
        else:
            drawn = scene_object.draw_visual()
        guids = _flatten_drawn(drawn)

        # Apply the robot-base-in-world offset to all freshly baked meshes.
        base_xform = _world_from_base_rhino_xform(state.robot_base_frame)
        if guids:
            rs.TransformObjects(guids, base_xform, copy=False)
    finally:
        sc.doc.Views.RedrawEnabled = was

    sc.sticky[_STICKY_DRAWN_IDS] = guids
    sc.doc.Views.Redraw()


def reset_home() -> None:
    """Show the robot at its default cell state (zero configuration, identity base)."""
    show_state(default_cell_state())


def clear_scene() -> None:
    """Delete all IK-preview meshes and drop the cached scene object."""
    _delete_tracked_ids()
    sc.sticky.pop(_STICKY_SCENE_OBJECT, None)
    sc.doc.Views.Redraw()
