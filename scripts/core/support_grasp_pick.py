"""Interactive pickers for the support-robot keyframe flow.

Moved out of the retired ``rs_ik_support_keyframe.py`` so the single
RSIKKeyframe button can reuse them:

- the two-phase GRASP pick on the held bar's centerline (pick the grasp
  center, then a heading point on the plane perpendicular to the bar axis =
  the gripper's rotation about the bar), with a ghost Robotiq gripper
  tracking the cursor;
- the two-phase BASE pick on a WalkableGround brep (origin, then heading),
  with a ghost of the assigned support robot tracking the cursor;
- the ghost bake of the dual-arm robot at its captured assembled pose
  (collision context the user picks the grasp around).

All Rhino-side (rhinoscriptsyntax / RhinoCommon); world outputs are 4x4
numpy matrices in mm.
"""

from __future__ import annotations

import math

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core import config
from core import dynamic_preview
from core import ik_viz
from core import robot_cell
from core import robot_cell_support
from core.rhino_block_import import has_block_definition
from core.rhino_frame_io import doc_unit_scale_to_mm


# ik_viz sub-layer (bundle) keys owned by the support flow. Distinct from the
# assembly preview's keys so showing/hiding one never touches the other.
DUAL_ARM_CONTEXT_LAYER_KEY = "SupportContext"      # Cindy frozen at assembled
SUPPORT_GHOST_LAYER_PREFIX = "SupportGhost"        # per-robot harvest bundles


# ---------------------------------------------------------------------------
# * Rhino <-> numpy helpers (doc units -> mm)
# ---------------------------------------------------------------------------


def rhino_xform_to_np_mm(xform):
    """A Rhino Transform (doc units) as a 4x4 numpy matrix with mm translation.

    Args:
        xform (Rhino.Geometry.Transform): transform in document units.

    Returns:
        np.ndarray: 4x4 matrix, translation scaled to mm.
    """
    scale = doc_unit_scale_to_mm()
    matrix = np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale
    return matrix


def np_mm_to_rhino_xform(matrix: np.ndarray):
    """A 4x4 numpy matrix (mm translation) as a Rhino Transform in doc units.

    Args:
        matrix (np.ndarray): 4x4 matrix, translation in mm.

    Returns:
        Rhino.Geometry.Transform: the same transform in document units.
    """
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    doc_matrix = np.array(matrix, dtype=float, copy=True)
    doc_matrix[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xform[i, j] = float(doc_matrix[i, j])
    return xform


def point_to_mm(point) -> np.ndarray:
    """A Rhino point (doc units) as a length-3 numpy vector in mm."""
    scale = doc_unit_scale_to_mm()
    if hasattr(point, "X"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale
    return np.asarray(point, dtype=float) * scale


def _unit(vector: np.ndarray) -> np.ndarray:
    """Unitized copy of ``vector``; raises on a zero-length input."""
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot unitize a zero-length vector.")
    return np.asarray(vector, dtype=float) / norm


# ---------------------------------------------------------------------------
# * Frame helpers
# ---------------------------------------------------------------------------


def bar_frame_doc_at_param(curve, t, heading_doc=None):
    """Grasp frame on the bar at parameter ``t``, in doc units.

    The origin is ``curve.PointAt(t)``, Z is the unitized tangent at t.
    If ``heading_doc`` is None, X = projection of world +X onto the plane
    perpendicular to Z (or world +Y if degenerate). Otherwise, X is the
    projection of (heading_doc - origin) onto the plane perp to Z.

    Returns:
        tuple: ``(xform, origin_pt, z_vec, x_vec)``.
    """
    pt = curve.PointAt(t)
    z = curve.TangentAt(t)
    z = Rhino.Geometry.Vector3d(z)
    z.Unitize()
    if heading_doc is not None:
        v = Rhino.Geometry.Vector3d(heading_doc.X - pt.X, heading_doc.Y - pt.Y, heading_doc.Z - pt.Z)
    else:
        v = Rhino.Geometry.Vector3d(1.0, 0.0, 0.0)
        if abs(v * z) > 0.99:
            v = Rhino.Geometry.Vector3d(0.0, 1.0, 0.0)
    x_raw = v - (v * z) * z
    if x_raw.Length < 1e-9:
        x_raw = Rhino.Geometry.Vector3d(0.0, 1.0, 0.0)
        x_raw = x_raw - (x_raw * z) * z
        if x_raw.Length < 1e-9:
            x_raw = Rhino.Geometry.Vector3d(1.0, 0.0, 0.0)
            x_raw = x_raw - (x_raw * z) * z
    x_raw.Unitize()
    y = Rhino.Geometry.Vector3d.CrossProduct(z, x_raw)
    xform = Rhino.Geometry.Transform(1.0)
    xform[0, 0], xform[0, 1], xform[0, 2], xform[0, 3] = x_raw.X, y.X, z.X, pt.X
    xform[1, 0], xform[1, 1], xform[1, 2], xform[1, 3] = x_raw.Y, y.Y, z.Y, pt.Y
    xform[2, 0], xform[2, 1], xform[2, 2], xform[2, 3] = x_raw.Z, y.Z, z.Z, pt.Z
    xform[3, 0], xform[3, 1], xform[3, 2], xform[3, 3] = 0.0, 0.0, 0.0, 1.0
    return xform, pt, z, x_raw


def compose_grasp_to_tool0_doc_xform(grasp_xform_doc, gripper_kind: str = "Robotiq"):
    """Doc-unit transform placing the gripper block (origin == tool0) at the
    world tool0 frame, given a doc-unit grasp frame."""
    grasp_mm = rhino_xform_to_np_mm(grasp_xform_doc)
    bar_grasp_to_tool0_mm = np.asarray(config.BAR_GRASP_TO_TOOL0[gripper_kind], dtype=float)
    tool0_mm = grasp_mm @ bar_grasp_to_tool0_mm
    return np_mm_to_rhino_xform(tool0_mm)


def _world_from_base_doc_xform(origin_doc, normal_doc, heading_doc_vec):
    """Doc-unit Rhino Transform with origin=origin_doc, Z=normal,
    X = heading_doc_vec projected onto the plane perp to Z (None if degenerate)."""
    z = Rhino.Geometry.Vector3d(normal_doc)
    z.Unitize()
    v = Rhino.Geometry.Vector3d(heading_doc_vec)
    x_raw = v - (v * z) * z
    if x_raw.Length < 1e-9:
        return None
    x_raw.Unitize()
    y = Rhino.Geometry.Vector3d.CrossProduct(z, x_raw)
    xform = Rhino.Geometry.Transform(1.0)
    xform[0, 0], xform[0, 1], xform[0, 2], xform[0, 3] = x_raw.X, y.X, z.X, origin_doc.X
    xform[1, 0], xform[1, 1], xform[1, 2], xform[1, 3] = x_raw.Y, y.Y, z.Y, origin_doc.Y
    xform[2, 0], xform[2, 1], xform[2, 2], xform[2, 3] = x_raw.Z, y.Z, z.Z, origin_doc.Z
    xform[3, 0], xform[3, 1], xform[3, 2], xform[3, 3] = 0.0, 0.0, 0.0, 1.0
    return xform


def frame_from_origin_normal_heading(origin_mm, normal, heading_mm) -> np.ndarray:
    """4x4 base frame from a ground point + normal + heading point (all mm)."""
    z = _unit(normal)
    heading_vec = heading_mm - origin_mm
    x_raw = heading_vec - np.dot(heading_vec, z) * z
    if float(np.linalg.norm(x_raw)) < 1e-6:
        raise RuntimeError("Heading point is collinear with base normal; pick a different point.")
    x = _unit(x_raw)
    y = np.cross(z, x)
    frame = np.eye(4, dtype=float)
    frame[:3, 0] = x
    frame[:3, 1] = y
    frame[:3, 2] = z
    frame[:3, 3] = origin_mm
    return frame


# ---------------------------------------------------------------------------
# * WalkableGround helpers
# ---------------------------------------------------------------------------


_WALKABLE_FILTER = rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion


def as_brep(object_id):
    """The Brep geometry for ``object_id``, converting from Extrusion if needed."""
    brep = rs.coercebrep(object_id)
    if brep is not None:
        return brep
    rhobj = rs.coercerhinoobject(object_id, True, True)
    geom = getattr(rhobj, "Geometry", None)
    if isinstance(geom, Rhino.Geometry.Extrusion):
        return geom.ToBrep(False)
    raise RuntimeError(f"Object {object_id} is not a Brep, Surface or Extrusion.")


def _breps_in_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
        return []
    accepted = {rs.filter.surface, rs.filter.polysurface, rs.filter.extrusion}
    return [
        oid for oid in rs.ObjectsByLayer(layer_name) or []
        if rs.ObjectType(oid) in accepted
    ]


def pick_walkable_brep(command_label: str = "RSIKKeyframe"):
    """Pick (or auto-pick when unique) a WalkableGround brep. None on cancel."""
    candidates = _breps_in_layer(config.WALKABLE_GROUND_LAYER)
    if not candidates:
        rs.MessageBox(
            f"Layer '{config.WALKABLE_GROUND_LAYER}' has no surface/polysurface/extrusion. "
            "Add one (e.g. a surface patch or a closed box) and try again.",
            0,
            command_label,
        )
        return None
    if len(candidates) == 1:
        return candidates[0]
    oid = rs.GetObject(
        f"Pick a Brep / Extrusion on the '{config.WALKABLE_GROUND_LAYER}' layer",
        filter=_WALKABLE_FILTER,
    )
    if oid is None or rs.ObjectLayer(oid) != config.WALKABLE_GROUND_LAYER:
        rs.MessageBox(
            f"Please pick an object that belongs to layer '{config.WALKABLE_GROUND_LAYER}'.",
            0,
            command_label,
        )
        return None
    return oid


def closest_point_on_brep(brep, point_doc):
    """Closest point + face normal on ``brep`` for a doc-unit point (None, None on failure)."""
    best = None
    for face in brep.Faces:
        ok, u, v = face.ClosestPoint(point_doc)
        if not ok:
            continue
        close_pt = face.PointAt(u, v)
        dist = close_pt.DistanceTo(point_doc)
        if best is None or dist < best[0]:
            normal = face.NormalAt(u, v)
            best = (dist, close_pt, normal)
    if best is None:
        return None, None
    _, pt, normal = best
    return pt, normal


def snap_to_brep(brep, origin_mm):
    """Snap an mm-space point onto ``brep``; returns (origin_mm, normal) or (None, None)."""
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    pt_doc = Rhino.Geometry.Point3d(*(origin_mm * scale_from_mm))
    close_pt, normal = closest_point_on_brep(brep, pt_doc)
    if close_pt is None:
        return None, None
    return point_to_mm(close_pt), np.array([normal.X, normal.Y, normal.Z], dtype=float)


# ---------------------------------------------------------------------------
# * Grasp pose picker (ghost gripper dynamic preview)
# ---------------------------------------------------------------------------


def gripper_block_meshes(command_label: str = "RSIKKeyframe"):
    """The gripper block's meshes (block-local coords), for ghost conduits.

    Args:
        command_label (str): title for the error message box.

    Returns:
        list | None: ``Rhino.Geometry.Mesh`` values, or None when the block
        definition is missing (reported via message box).
    """
    if not has_block_definition(config.ROBOTIQ_GRIPPER_BLOCK):
        rs.MessageBox(
            f"Block definition '{config.ROBOTIQ_GRIPPER_BLOCK}' not found. "
            "Insert/define it before running this script.",
            0,
            command_label,
        )
        return None
    return dynamic_preview.block_definition_meshes(config.ROBOTIQ_GRIPPER_BLOCK)


def _gripper_z_inplane_angle(gripper_kind: str) -> float:
    """The gripper local-Z direction's angle in the grasp frame's XY plane.

    The grasp frame has Z along the bar; its XY plane is perpendicular to
    the bar. The gripper (tool0) local Z expressed in that frame sits at
    some fixed in-plane angle from the frame's X — this returns that angle,
    so the picker can put the RUBBER-BAND LINE on the gripper's Z instead of
    on the abstract frame X.

    Args:
        gripper_kind (str): key into ``config.BAR_GRASP_TO_TOOL0``.

    Returns:
        float: angle in radians, measured about the bar axis.

    Raises:
        RuntimeError: when the gripper's Z is (near) parallel to the bar —
            then "point the line along gripper Z" is meaningless and the
            grasp convention should be checked.
    """
    grasp_from_tool0 = np.asarray(config.BAR_GRASP_TO_TOOL0[gripper_kind], dtype=float)
    z_in_grasp = grasp_from_tool0[:3, 2]
    inplane = math.hypot(float(z_in_grasp[0]), float(z_in_grasp[1]))
    if inplane < 1e-6:
        raise RuntimeError(
            f"BAR_GRASP_TO_TOOL0[{gripper_kind!r}] has the gripper Z parallel "
            "to the bar axis — the pointing-direction pick cannot work. Check "
            "the grasp convention (RSDefineRoboticTool, SupportGripper mode)."
        )
    return math.atan2(float(z_in_grasp[1]), float(z_in_grasp[0]))


def _heading_for_gripper_z(cursor_doc, grasp_center_doc, bar_tangent_doc, phi: float):
    """Turn a picked pointing direction into the grasp frame's heading point.

    The user drags a line from the grasp center; that line should BE the
    gripper's local Z (projected onto the plane perpendicular to the bar).
    The grasp frame's X sits ``phi`` radians away from the gripper Z about
    the bar axis, so the picked direction is rotated by ``-phi`` to get the
    heading the frame builder expects.

    Args:
        cursor_doc (Rhino.Geometry.Point3d): the picked/hovered point.
        grasp_center_doc (Rhino.Geometry.Point3d): the grasp center.
        bar_tangent_doc (Rhino.Geometry.Vector3d): unit bar tangent.
        phi (float): ``_gripper_z_inplane_angle`` result.

    Returns:
        Rhino.Geometry.Point3d | None: the adjusted heading point, or None
        when the cursor sits on the bar axis (no direction).
    """
    d = Rhino.Geometry.Vector3d(
        cursor_doc.X - grasp_center_doc.X,
        cursor_doc.Y - grasp_center_doc.Y,
        cursor_doc.Z - grasp_center_doc.Z,
    )
    if d.Length < 1e-9:
        return None
    d.Rotate(-phi, bar_tangent_doc)
    return grasp_center_doc + d


def pick_grasp_frame_on_bar(bar_curve, gripper_kind: str = "Robotiq",
                            command_label: str = "RSIKKeyframe"):
    """Pick grasp center + pointing direction on the bar, with a ghost gripper
    tracking the cursor.

    Phase A constrains the cursor to the bar centerline (the grasp center);
    phase B constrains it to the plane perpendicular to the bar axis through
    that center, like Rhino's _Rotate. The rubber-band line drawn from the
    grasp center IS the gripper's local Z direction (its pointing axis),
    projected onto that plane.

    Args:
        bar_curve (Rhino.Geometry.Curve): the held bar's centerline.
        gripper_kind (str): key into ``config.BAR_GRASP_TO_TOOL0``.
        command_label (str): title for error message boxes.

    Returns:
        tuple: ``(grasp_world_mm, tool0_world_mm)`` 4x4 matrices, or
        ``(None, None)`` on cancel.
    """
    if gripper_kind not in config.BAR_GRASP_TO_TOOL0:
        rs.MessageBox(
            f"BAR_GRASP_TO_TOOL0['{gripper_kind}'] is empty. "
            "Run RSDefineRoboticTool in SupportGripper mode first.",
            0,
            command_label,
        )
        return None, None
    gripper_meshes = gripper_block_meshes(command_label)
    if gripper_meshes is None:
        return None, None
    try:
        phi = _gripper_z_inplane_angle(gripper_kind)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, command_label)
        return None, None

    with dynamic_preview.mesh_preview(gripper_meshes, alpha=0.5) as conduit:
        # Phase A: grasp center on bar centerline.
        def _xform_phase_a(cursor_doc):
            ok, t = bar_curve.ClosestPoint(cursor_doc)
            if not ok:
                return None
            grasp_xform, _, _, _ = bar_frame_doc_at_param(bar_curve, t, heading_doc=None)
            return compose_grasp_to_tool0_doc_xform(grasp_xform, gripper_kind)

        picker_a = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_a)
        picker_a.SetCommandPrompt("Pick grasp center on bar centerline")
        picker_a.Constrain(bar_curve, False)
        if picker_a.Get() != Rhino.Input.GetResult.Point:
            return None, None
        grasp_center_doc = picker_a.Point()
        ok, t = bar_curve.ClosestPoint(grasp_center_doc)
        if not ok:
            return None, None

        # Phase B: pointing direction (the gripper's rotation about the bar).
        # Constrain the cursor to the plane perpendicular to the bar tangent
        # through grasp_center, so it slides freely on a 2D plane like
        # Rhino's _Rotate command; the rubber-band line from the center to
        # the cursor IS the gripper's local Z. Disable osnaps so nearby
        # geometry (the bar curve, tube preview, robot ghost) doesn't pull
        # the cursor.
        bar_tangent_doc = bar_curve.TangentAt(t)
        bar_tangent_doc.Unitize()
        heading_plane = Rhino.Geometry.Plane(grasp_center_doc, bar_tangent_doc)

        def _xform_phase_b(cursor_doc):
            heading_doc = _heading_for_gripper_z(
                cursor_doc, grasp_center_doc, bar_tangent_doc, phi
            )
            if heading_doc is None:
                return None
            grasp_xform, _, _, _ = bar_frame_doc_at_param(bar_curve, t, heading_doc=heading_doc)
            return compose_grasp_to_tool0_doc_xform(grasp_xform, gripper_kind)

        picker_b = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_b)
        picker_b.SetCommandPrompt(
            "Pick the gripper pointing direction (the line = gripper local Z)"
        )
        picker_b.SetBasePoint(grasp_center_doc, True)
        picker_b.DrawLineFromPoint(grasp_center_doc, True)
        picker_b.Constrain(heading_plane, False)
        picker_b.PermitObjectSnap(False)
        if picker_b.Get() != Rhino.Input.GetResult.Point:
            return None, None
        heading_doc = _heading_for_gripper_z(
            picker_b.Point(), grasp_center_doc, bar_tangent_doc, phi
        )
        if heading_doc is None:
            return None, None

    grasp_xform_doc, _, _, _ = bar_frame_doc_at_param(bar_curve, t, heading_doc=heading_doc)
    grasp_mm = rhino_xform_to_np_mm(grasp_xform_doc)
    tool0_mm = grasp_mm @ np.asarray(config.BAR_GRASP_TO_TOOL0[gripper_kind], dtype=float)
    return grasp_mm, tool0_mm


# ---------------------------------------------------------------------------
# * Ghost previews (dual-arm collision context + support robot for the base pick)
# ---------------------------------------------------------------------------


def show_dual_arm_context(assembled: dict, mesh_mode):
    """Show Cindy frozen at her captured assembled pose (the pick backdrop).

    Drawn through ik_viz's cached bundle system on its OWN sub-layer
    (``DUAL_ARM_CONTEXT_LAYER_KEY``), so later ghost-mesh harvesting and
    support-pose previews never wipe or hide it. The flow's final
    ``ik_viz.clear_scene()`` removes it with everything else.

    Purely visual — collision with the dual-arm robot is handled by its
    frozen ToolModel obstacle in the support cell.

    Args:
        assembled (dict): ``{"base_frame_world_mm", "joint_values_left",
            "joint_values_right", "joint_names_left", "joint_names_right"}``.
        mesh_mode: ik_viz mesh mode to display.
    """
    deps = robot_cell.import_compas_stack()
    Frame = deps["Frame"]
    Configuration = deps["Configuration"]

    da_cell = robot_cell.get_or_load_robot_cell()
    da_state = da_cell.default_cell_state()
    da_state.robot_base_frame = robot_cell._mm_matrix_to_m_frame(
        Frame, np.asarray(assembled["base_frame_world_mm"], dtype=float)
    )

    zero_cfg = da_cell.robot_model.zero_configuration()
    cfg_names = list(zero_cfg.joint_names)
    cfg_types = list(zero_cfg.joint_types)
    cfg_values = list(zero_cfg.joint_values)
    captured = dict(zip(
        list(assembled["joint_names_left"]) + list(assembled["joint_names_right"]),
        list(assembled["joint_values_left"]) + list(assembled["joint_values_right"]),
    ))
    for i, name in enumerate(cfg_names):
        if name in captured:
            cfg_values[i] = float(captured[name])
    da_state.robot_configuration = Configuration(
        joint_values=cfg_values, joint_types=cfg_types, joint_names=cfg_names,
    )

    ik_viz.update_state(
        da_state,
        robot_cell=da_cell,
        mesh_mode=mesh_mode,
        layer_key=DUAL_ARM_CONTEXT_LAYER_KEY,
    )
    # A leftover ASSEMBLY preview from an earlier RSIKKeyframe run shares the
    # cache layer update_state just forced visible — hide it so only the
    # frozen context shows.
    ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
    sc.doc.Views.Redraw()


def support_robot_ghost_meshes(robot_name: str):
    """One support robot's link meshes at zero config + identity base.

    Harvested through ``ik_viz.get_robot_link_meshes_at_state`` (which bakes
    into a per-robot bundle and returns standalone mesh COPIES for a cursor
    ghost). That harvest hides the whole IK cache layer on exit — assuming
    the ghost should be the only robot on screen — but here Cindy's frozen
    context must STAY visible behind the base pick, so the cache layer is
    turned back on and only the harvest bundle's sub-layer is hidden.

    Args:
        robot_name (str): "Alice" or "Belle".

    Returns:
        list: ``Rhino.Geometry.Mesh`` copies (base-link local, doc units).
    """
    cell = robot_cell_support.get_or_load_support_cell(robot_name)
    state = robot_cell_support.default_support_cell_state(robot_name)
    deps = robot_cell.import_compas_stack()
    state.robot_base_frame = deps["Frame"].worldXY()

    ghost_layer_key = f"{SUPPORT_GHOST_LAYER_PREFIX} {robot_name}"
    meshes = ik_viz.get_robot_link_meshes_at_state(state, cell, layer_key=ghost_layer_key)
    # Keep the frozen-Cindy backdrop on screen; hide only the harvest bake
    # (a static robot parked at the world origin otherwise shows through).
    if rs.IsLayer(config.LAYER_IK_CACHE):
        rs.LayerVisible(config.LAYER_IK_CACHE, True)
    ik_viz.set_layer_visible(ghost_layer_key, False)
    ik_viz.set_layer_visible(DUAL_ARM_CONTEXT_LAYER_KEY, True)
    sc.doc.Views.Redraw()
    return meshes


# ---------------------------------------------------------------------------
# * Base-pose picker (ghost support robot dynamic preview)
# ---------------------------------------------------------------------------


def pick_base_frame_on_walkable(brep_id, robot_name: str,
                                command_label: str = "RSIKKeyframe"):
    """Pick base origin + heading on the walkable brep, with the assigned
    support robot's ghost mesh tracking the cursor.

    Args:
        brep_id: the WalkableGround object id.
        robot_name (str): whose ghost meshes to show ("Alice"/"Belle").
        command_label (str): title for error message boxes.

    Returns:
        tuple: ``(base_origin_mm, base_normal, heading_mm, seed_base_frame_mm)``
        or all None on cancel.
    """
    brep = as_brep(brep_id)
    robot_meshes = support_robot_ghost_meshes(robot_name)

    with dynamic_preview.mesh_preview(robot_meshes, alpha=0.4) as conduit:
        # Phase A: base origin on brep.
        def _xform_phase_a(cursor_doc):
            close_pt, normal = closest_point_on_brep(brep, cursor_doc)
            if close_pt is None:
                return None
            world_x = Rhino.Geometry.Vector3d(1.0, 0.0, 0.0)
            return _world_from_base_doc_xform(close_pt, normal, world_x)

        picker_a = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_a)
        picker_a.SetCommandPrompt(
            f"Pick {robot_name}'s base origin on WalkableGround brep"
        )
        picker_a.Constrain(brep, -1, -1, False)
        if picker_a.Get() != Rhino.Input.GetResult.Point:
            return None, None, None, None
        picked_doc = picker_a.Point()
        close_pt, normal = closest_point_on_brep(brep, picked_doc)
        if close_pt is None:
            return None, None, None, None

        # Phase B: heading point.
        def _xform_phase_b(cursor_doc):
            heading_vec = Rhino.Geometry.Vector3d(
                cursor_doc.X - close_pt.X,
                cursor_doc.Y - close_pt.Y,
                cursor_doc.Z - close_pt.Z,
            )
            return _world_from_base_doc_xform(close_pt, normal, heading_vec)

        picker_b = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_b)
        picker_b.SetCommandPrompt("Pick heading point (defines base +X)")
        picker_b.SetBasePoint(close_pt, True)
        if picker_b.Get() != Rhino.Input.GetResult.Point:
            return None, None, None, None
        heading_doc = picker_b.Point()

    base_origin_mm = point_to_mm(close_pt)
    normal_v = np.array([normal.X, normal.Y, normal.Z], dtype=float)
    heading_mm = point_to_mm(heading_doc)
    try:
        seed_base = frame_from_origin_normal_heading(base_origin_mm, normal_v, heading_mm)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, command_label)
        return None, None, None, None
    return base_origin_mm, normal_v, heading_mm, seed_base
