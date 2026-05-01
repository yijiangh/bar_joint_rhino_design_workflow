#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSIKSupportKeyframe - Single-arm support-robot IK keyframe workflow.

Pick the bar to be HELD (which is the just-assembled bar carrying the
`ik_assembly` user-text written by RSIKKeyframe). The script then:

1. Loads the support cell (single-arm Husky-Alice) with the dual-arm
   robot frozen as a `ToolModel` collision obstacle, configured from the
   assembled bar's `ik_assembly` payload.
2. Picks the grasp pose on the supported bar's centerline. A Robotiq
   gripper preview tracks the cursor (centre, then heading) so the user
   can see the gripper rotation about the bar axis in real time.
3. Picks a base origin + heading on the `WalkableGround` brep. A full
   support-robot mesh preview (baked at zero configuration) tracks the
   cursor.
4. Solves single-arm IK with base sampling. Previews the final pose via
   `core.ik_viz.show_state(state, robot_model=support_cell.robot_model)`.
5. Saves an `ik_support` user-text payload on the supported bar, plus a
   capture JSON for headless replay.
"""

from __future__ import annotations

import datetime
import importlib
import json
import math
import os
import random
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import capture_io as _capture_io_module
from core import config as _config_module
from core import dynamic_preview as _dynamic_preview_module
from core import env_collision as _env_collision_module
from core import highlight_env as _highlight_env_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core import robot_cell_support as _robot_cell_support_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import get_bar_seq_map, repair_on_entry
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import set_objects_layer, suspend_redraw


# ---------------------------------------------------------------------------
# Constants / user-text keys
# ---------------------------------------------------------------------------

IK_ASSEMBLY_KEY = "ik_assembly"
PINEAPPLE_ROLE_KEY = "_ik_support_pineapple_role"
SUPPORT_PREVIEW_BLOCK_ROLE = "support_grasp_preview"


# ---------------------------------------------------------------------------
# Module reload
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    global capture_io, config, dynamic_preview, env_collision, highlight_env, ik_viz, robot_cell, robot_cell_support
    config = importlib.reload(_config_module)
    dynamic_preview = importlib.reload(_dynamic_preview_module)
    env_collision = importlib.reload(_env_collision_module)
    highlight_env = importlib.reload(_highlight_env_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    robot_cell_support = importlib.reload(_robot_cell_support_module)
    capture_io = importlib.reload(_capture_io_module)


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Rhino <-> numpy helpers (doc units -> mm) — copied from rs_ik_keyframe.py
# ---------------------------------------------------------------------------


def _rhino_xform_to_np_mm(xform):
    scale = doc_unit_scale_to_mm()
    matrix = np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale
    return matrix


def _np_mm_to_rhino_xform(matrix: np.ndarray):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    doc_matrix = np.array(matrix, dtype=float, copy=True)
    doc_matrix[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xform[i, j] = float(doc_matrix[i, j])
    return xform


def _point_to_mm(point) -> np.ndarray:
    scale = doc_unit_scale_to_mm()
    if hasattr(point, "X"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale
    return np.asarray(point, dtype=float) * scale


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("Cannot unitize a zero-length vector.")
    return np.asarray(vector, dtype=float) / norm


# ---------------------------------------------------------------------------
# Block / layer helpers
# ---------------------------------------------------------------------------


def _has_block_definition(name) -> bool:
    for idef in sc.doc.InstanceDefinitions:
        if idef is not None and not idef.IsDeleted and idef.Name == name:
            return True
    return False


def _require_block_definition(name) -> str:
    if not _has_block_definition(name):
        raise RuntimeError(f"Missing required Rhino block definition '{name}'.")
    return name


def _cleanup_ids(oids):
    if not oids:
        return
    with suspend_redraw():
        for oid in oids:
            try:
                rs.DeleteObject(oid)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Bar-centerline picking
# ---------------------------------------------------------------------------


def _pick_bar_centerline(prompt):
    """Pick a registered bar (centerline OR tube preview) -> bar curve oid."""
    return pick_bar(prompt)


# ---------------------------------------------------------------------------
# Assembled-bar pose loading
# ---------------------------------------------------------------------------


def _load_assembled_ik(assembled_oid):
    raw = rs.GetUserText(assembled_oid, IK_ASSEMBLY_KEY)
    if not raw:
        raise RuntimeError(
            "Picked assembled bar has no 'ik_assembly' user-text. "
            "Run RSIKKeyframe on it first."
        )
    payload = json.loads(raw)
    base_mm = np.asarray(payload["base_frame_world_mm"], dtype=float)
    final = payload["final"]
    return {
        "base_frame_world_mm": base_mm,
        "joint_values_left": final["left"]["joint_values"],
        "joint_values_right": final["right"]["joint_values"],
        "joint_names_left": final["left"]["joint_names"],
        "joint_names_right": final["right"]["joint_names"],
    }


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------


def _bar_frame_doc_at_param(curve, t, heading_doc=None):
    """Return (xform, origin_pt, z_vec, x_vec) in doc units.

    The origin is `curve.PointAt(t)`, Z is the unitized tangent at t.
    If `heading_doc` is None, X = projection of world +X onto the plane
    perpendicular to Z (or world +Y if degenerate). Otherwise, X is the
    projection of (heading_doc - origin) onto the plane perp to Z.
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


def _compose_grasp_to_tool0_doc_xform(grasp_xform_doc, gripper_kind="Robotiq"):
    """Compose the doc-unit transform that places the gripper block (origin
    == tool0) at the world tool0 frame, given a doc-unit grasp frame.
    """
    grasp_mm = _rhino_xform_to_np_mm(grasp_xform_doc)
    bar_grasp_to_tool0_mm = np.asarray(config.BAR_GRASP_TO_TOOL0[gripper_kind], dtype=float)
    tool0_mm = grasp_mm @ bar_grasp_to_tool0_mm
    return _np_mm_to_rhino_xform(tool0_mm)


def _world_from_base_doc_xform(origin_doc, normal_doc, heading_doc_vec):
    """Doc-unit Rhino.Geometry.Transform with origin=origin_doc, Z=normal,
    X = heading_doc_vec projected onto the plane perp to Z.
    """
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


def _frame_from_origin_normal_heading(origin_mm, normal, heading_mm) -> np.ndarray:
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
# WalkableGround picking — copied from rs_ik_keyframe.py
# ---------------------------------------------------------------------------


_WALKABLE_FILTER = rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion


def _as_brep(object_id):
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


def _pick_walkable_brep():
    candidates = _breps_in_layer(config.WALKABLE_GROUND_LAYER)
    if not candidates:
        rs.MessageBox(
            f"Layer '{config.WALKABLE_GROUND_LAYER}' has no surface/polysurface/extrusion. "
            "Add one (e.g. a surface patch or a closed box) and try again.",
            0,
            "RSIKSupportKeyframe",
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
            "RSIKSupportKeyframe",
        )
        return None
    return oid


def _closest_point_on_brep(brep, point_doc):
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


def _snap_to_brep(brep, origin_mm):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    pt_doc = Rhino.Geometry.Point3d(*(origin_mm * scale_from_mm))
    close_pt, normal = _closest_point_on_brep(brep, pt_doc)
    if close_pt is None:
        return None, None
    return _point_to_mm(close_pt), np.array([normal.X, normal.Y, normal.Z], dtype=float)


# ---------------------------------------------------------------------------
# Grasp pose picker (Robotiq gripper dynamic preview)
# ---------------------------------------------------------------------------


def _pick_grasp_frame_on_bar(bar_curve, gripper_kind="Robotiq"):
    """Pick grasp center + heading on the bar, with gripper preview tracking
    the cursor. Returns (grasp_world_mm, tool0_world_mm) or (None, None).
    """
    if not _has_block_definition(config.ROBOTIQ_GRIPPER_BLOCK):
        rs.MessageBox(
            f"Block definition '{config.ROBOTIQ_GRIPPER_BLOCK}' not found. "
            "Insert/define it before running this script.",
            0,
            "RSIKSupportKeyframe",
        )
        return None, None
    if gripper_kind not in config.BAR_GRASP_TO_TOOL0:
        rs.MessageBox(
            f"BAR_GRASP_TO_TOOL0['{gripper_kind}'] is empty. "
            "Run RSExportGraspTool0TF in Gripper mode first.",
            0,
            "RSIKSupportKeyframe",
        )
        return None, None

    gripper_meshes = dynamic_preview.block_definition_meshes(config.ROBOTIQ_GRIPPER_BLOCK)

    with dynamic_preview.mesh_preview(gripper_meshes, alpha=0.5) as conduit:
        # Phase A: grasp center on bar centerline.
        def _xform_phase_a(cursor_doc):
            ok, t = bar_curve.ClosestPoint(cursor_doc)
            if not ok:
                return None
            grasp_xform, _, _, _ = _bar_frame_doc_at_param(bar_curve, t, heading_doc=None)
            return _compose_grasp_to_tool0_doc_xform(grasp_xform, gripper_kind)

        picker_a = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_a)
        picker_a.SetCommandPrompt("Pick grasp center on bar centerline")
        picker_a.Constrain(bar_curve, False)
        if picker_a.Get() != Rhino.Input.GetResult.Point:
            return None, None
        grasp_center_doc = picker_a.Point()
        ok, t = bar_curve.ClosestPoint(grasp_center_doc)
        if not ok:
            return None, None

        # Phase B: heading point (defines gripper rotation about bar axis).
        # Constrain cursor to the plane perpendicular to the bar tangent
        # through grasp_center, so it slides freely on a 2D plane like
        # Rhino's _Rotate command. Disable osnaps so nearby geometry
        # (the bar curve, tube preview, dual-arm bake) doesn't pull the
        # cursor.
        bar_tangent_doc = bar_curve.TangentAt(t)
        bar_tangent_doc.Unitize()
        heading_plane = Rhino.Geometry.Plane(grasp_center_doc, bar_tangent_doc)

        def _xform_phase_b(cursor_doc):
            grasp_xform, _, _, _ = _bar_frame_doc_at_param(bar_curve, t, heading_doc=cursor_doc)
            return _compose_grasp_to_tool0_doc_xform(grasp_xform, gripper_kind)

        picker_b = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_b)
        picker_b.SetCommandPrompt("Pick heading direction (rotation about bar axis)")
        picker_b.SetBasePoint(grasp_center_doc, True)
        picker_b.DrawLineFromPoint(grasp_center_doc, True)
        picker_b.Constrain(heading_plane, False)
        picker_b.PermitObjectSnap(False)
        if picker_b.Get() != Rhino.Input.GetResult.Point:
            return None, None
        heading_doc = picker_b.Point()

    grasp_xform_doc, _, _, _ = _bar_frame_doc_at_param(bar_curve, t, heading_doc=heading_doc)
    grasp_mm = _rhino_xform_to_np_mm(grasp_xform_doc)
    tool0_mm = grasp_mm @ np.asarray(config.BAR_GRASP_TO_TOOL0[gripper_kind], dtype=float)
    return grasp_mm, tool0_mm


def _insert_robotiq_at_tool0(tool0_world_mm):
    _require_block_definition(config.ROBOTIQ_GRIPPER_BLOCK)
    oid = rs.InsertBlock(config.ROBOTIQ_GRIPPER_BLOCK, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert '{config.ROBOTIQ_GRIPPER_BLOCK}' block.")
    rs.TransformObject(oid, _np_mm_to_rhino_xform(tool0_world_mm))
    rs.SetUserText(oid, PINEAPPLE_ROLE_KEY, SUPPORT_PREVIEW_BLOCK_ROLE)
    set_objects_layer(oid, config.SUPPORT_PREVIEW_LAYER)
    return oid


# ---------------------------------------------------------------------------
# Dual-arm bake at captured ik_assembly pose (collision context preview)
# ---------------------------------------------------------------------------


def _bake_dual_arm_at_captured_pose(assembled, mesh_mode):
    """Bake the dual-arm robot meshes at the captured ik_assembly pose into
    Rhino doc objects, then detach the resulting GUIDs from ik_viz's
    tracking sticky so subsequent ik_viz.show_state calls (e.g. for the
    support robot's final pose) don't wipe them. The caller is responsible
    for deleting the returned GUIDs at cleanup.

    The dual-arm robot is loaded as the dual-arm RobotCell's actuated
    `robot_model` here purely for visualization — collision is still
    handled by the ToolModel obstacle on the support cell.
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

    ik_viz.show_state(da_state, mesh_mode=mesh_mode, robot_model=da_cell.robot_model)
    guids = list(sc.sticky.get(ik_viz._STICKY_DRAWN_IDS, []) or [])
    sc.sticky[ik_viz._STICKY_DRAWN_IDS] = []
    sc.sticky.pop(ik_viz._STICKY_SCENE_OBJECT, None)
    return guids


# ---------------------------------------------------------------------------
# Support-robot mesh bake at zero configuration (for base-pick preview)
# ---------------------------------------------------------------------------


def _bake_support_robot_meshes_at_zero():
    """Bake the support-robot URDF visual meshes once at zero config + identity
    base, capture them as `Rhino.Geometry.Mesh` values (in base-link local
    coordinates, doc units), then delete the baked guids.
    """
    cell = robot_cell_support.get_or_load_support_cell()
    state = robot_cell_support.default_support_cell_state()
    deps = robot_cell.import_compas_stack()
    state.robot_base_frame = deps["Frame"].worldXY()

    ik_viz.show_state(state, robot_model=cell.robot_model)
    guids = list(sc.sticky.get(ik_viz._STICKY_DRAWN_IDS, []) or [])
    meshes = []
    for g in guids:
        m = rs.coercemesh(g)
        if m is not None:
            meshes.append(m.DuplicateMesh())
    ik_viz.clear_scene()
    return meshes


# ---------------------------------------------------------------------------
# Base-pose picker (full support-robot dynamic preview)
# ---------------------------------------------------------------------------


def _pick_base_frame_on_walkable(brep_id):
    """Pick base origin + heading on the walkable brep, with the support
    robot's mesh tracking the cursor. Returns
    (base_origin_mm, base_normal, heading_mm, seed_base_frame_mm) or all None.
    """
    brep = _as_brep(brep_id)
    robot_meshes = _bake_support_robot_meshes_at_zero()

    with dynamic_preview.mesh_preview(robot_meshes, alpha=0.4) as conduit:
        # Phase A: base origin on brep.
        def _xform_phase_a(cursor_doc):
            close_pt, normal = _closest_point_on_brep(brep, cursor_doc)
            if close_pt is None:
                return None
            world_x = Rhino.Geometry.Vector3d(1.0, 0.0, 0.0)
            return _world_from_base_doc_xform(close_pt, normal, world_x)

        picker_a = dynamic_preview.TrackingGetPoint(conduit, _xform_phase_a)
        picker_a.SetCommandPrompt("Pick base origin on WalkableGround brep")
        picker_a.Constrain(brep, -1, -1, False)
        if picker_a.Get() != Rhino.Input.GetResult.Point:
            return None, None, None, None
        picked_doc = picker_a.Point()
        close_pt, normal = _closest_point_on_brep(brep, picked_doc)
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

    base_origin_mm = _point_to_mm(close_pt)
    normal_v = np.array([normal.X, normal.Y, normal.Z], dtype=float)
    heading_mm = _point_to_mm(heading_doc)
    try:
        seed_base = _frame_from_origin_normal_heading(base_origin_mm, normal_v, heading_mm)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSIKSupportKeyframe")
        return None, None, None, None
    return base_origin_mm, normal_v, heading_mm, seed_base


# ---------------------------------------------------------------------------
# Interactive prompts
# ---------------------------------------------------------------------------


def _ask_collision_options(env_count: int = 0):
    current_mesh_mode = ik_viz.get_mesh_mode()
    is_collision_default = current_mesh_mode == ik_viz.MESH_MODE_COLLISION

    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(f"Collision options (env={env_count} bodies)")
    opt_self = Rhino.Input.Custom.OptionToggle(True, "No", "Yes")
    opt_env = Rhino.Input.Custom.OptionToggle(True, "No", "Yes")
    opt_mesh = Rhino.Input.Custom.OptionToggle(is_collision_default, "Visual", "Collision")
    go.AddOptionToggle("IncludeSelf", opt_self)
    go.AddOptionToggle("IncludeEnv", opt_env)
    go.AddOptionToggle("MeshMode", opt_mesh)
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Option:
            continue
        if result == Rhino.Input.GetResult.Nothing:
            mesh_mode = ik_viz.MESH_MODE_COLLISION if bool(opt_mesh.CurrentValue) else ik_viz.MESH_MODE_VISUAL
            ik_viz.set_mesh_mode(mesh_mode)
            return bool(opt_self.CurrentValue), bool(opt_env.CurrentValue), mesh_mode
        return None


def _ask_accept(prompt="Accept this support IK keyframe and save it on the bar"):
    answer = rs.GetString(prompt, "Accept", ["Accept", "Reject"])
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("a")


def _ask_save_failure_capture(phase_label: str) -> bool:
    answer = rs.GetString(
        f"{phase_label} IK failed. Save capture for headless debugging?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


def _ask_save_debug_capture() -> bool:
    """Default-No prompt — the capture is a debug artefact, production state
    lives on the bar's `ik_support` user-text."""
    answer = rs.GetString(
        "Save debug capture for headless replay?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


# ---------------------------------------------------------------------------
# IK solving with base sampling (single-arm support cell)
# ---------------------------------------------------------------------------


def _sample_base_offsets(count, radius_mm):
    rng = random.Random()
    for _ in range(count):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        r = rng.uniform(0.0, radius_mm)
        yield np.array([r * math.cos(angle), r * math.sin(angle), 0.0], dtype=float)


def _solve_with_sampling(planner, template_state, seed_base_frame_mm,
                         tool0_world_mm, brep_id, heading_mm,
                         include_self, include_env):
    check_collision = bool(include_self or include_env)
    attempts = [seed_base_frame_mm]
    brep = _as_brep(brep_id)
    for offset in _sample_base_offsets(config.IK_BASE_SAMPLE_MAX_ITER, config.IK_BASE_SAMPLE_RADIUS):
        sample_origin_mm = seed_base_frame_mm[:3, 3] + offset
        snapped_origin, normal = _snap_to_brep(brep, sample_origin_mm)
        if snapped_origin is None:
            continue
        try:
            sample_frame = _frame_from_origin_normal_heading(snapped_origin, normal, heading_mm)
        except RuntimeError:
            continue
        attempts.append(sample_frame)

    total = len(attempts)
    for idx, base_frame in enumerate(attempts):
        label = "seed" if idx == 0 else f"sample {idx}/{total - 1}"
        origin = base_frame[:3, 3]
        print(
            f"RSIKSupportKeyframe: trying base frame ({label}) "
            f"at ({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
        )
        state = robot_cell_support.solve_support_ik(
            planner,
            template_state,
            base_frame,
            tool0_world_mm,
            check_collision=check_collision,
            verbose_pairs=check_collision,
        )
        if state is not None:
            print(
                f"RSIKSupportKeyframe: [OK] IK solution FOUND on attempt {idx + 1}/{total} ({label})."
            )
            return state, base_frame
        print(f"RSIKSupportKeyframe: [x] IK failed on attempt {idx + 1}/{total} ({label}).")
    print(
        f"RSIKSupportKeyframe: [X] IK failed for all {total} attempt(s). "
        f"Consider increasing IK_BASE_SAMPLE_RADIUS / IK_BASE_SAMPLE_MAX_ITER in config.py."
    )
    return None, None


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------


_CAPTURES_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "tests", "captures"))


def _save_capture(
    *,
    held_bar_id,
    assembled_bar_id,
    grasp_world_mm,
    tool0_world_mm,
    initial_state,
    assembled_payload,
    include_self,
    include_env,
    final_state=None,
    rcell=None,
    suffix: str = "",
):
    """Write a v2 support capture JSON.

    Mirrors ``rs_ik_keyframe._save_capture``; the support-cell flavour has
    a single ``support`` IK target (one-arm) and a ``source.assembled``
    block carrying the dual-arm pose this support pose is anchored to.
    """
    try:
        os.makedirs(_CAPTURES_DIR, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = f"{timestamp}_support_{held_bar_id}{suffix}"
        path = os.path.join(_CAPTURES_DIR, f"{stem}.json")

        cell_ref = capture_io.save_robot_cell_if_changed(rcell, _CAPTURES_DIR)

        ik_targets = {
            "support": {
                "tool0_world_mm": np.asarray(tool0_world_mm, dtype=float).tolist(),
            },
        }
        ik_options = {
            "check_collision": bool(include_self or include_env),
            "include_self": bool(include_self),
            "include_env": bool(include_env),
        }
        source = {
            "held_bar_id": held_bar_id,
            "assembled_bar_id": assembled_bar_id,
            "gripper_kind": "Robotiq",
            "grasp_frame_world_mm": np.asarray(grasp_world_mm, dtype=float).tolist(),
            "doc_unit_scale_to_mm": float(doc_unit_scale_to_mm()),
            "assembled": {
                "base_frame_world_mm": np.asarray(
                    assembled_payload["base_frame_world_mm"], dtype=float
                ).tolist(),
                "joint_values_left": list(assembled_payload["joint_values_left"]),
                "joint_values_right": list(assembled_payload["joint_values_right"]),
                "joint_names_left": list(assembled_payload["joint_names_left"]),
                "joint_names_right": list(assembled_payload["joint_names_right"]),
            },
        }
        capture_io.save_capture_v2(
            path,
            captured_at=datetime.datetime.now().isoformat(timespec="seconds"),
            robot_cell_ref=cell_ref,
            initial_state=initial_state,
            ik_targets=ik_targets,
            ik_options=ik_options,
            expected={"support": final_state},
            source=source,
        )
        rel = os.path.relpath(path, os.path.dirname(SCRIPT_DIR))
        print(f"RSIKSupportKeyframe: capture saved -> {rel}")
        return path
    except Exception as exc:
        print(f"RSIKSupportKeyframe: failed to write capture ({exc}); continuing.")
        return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSIKSupportKeyframe")

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.", 0, "RSIKSupportKeyframe"
        )
        return
    _client, planner = robot_cell.get_planner()

    rs.UnselectAllObjects()
    # The bar being supported IS the just-assembled bar (the one whose
    # `ik_assembly` user-text we read for the dual-arm pose). One pick.
    held_oid = _pick_bar_centerline(
        "Pick the bar to be HELD (just-assembled, must carry 'ik_assembly')"
    )
    if held_oid is None:
        return
    assembled_oid = held_oid

    try:
        assembled = _load_assembled_ik(assembled_oid)
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSIKSupportKeyframe")
        return

    rcell = robot_cell_support.get_or_load_support_cell()
    template_state = robot_cell_support.default_support_cell_state()
    template_state = robot_cell_support.configure_dual_arm_obstacle(
        template_state,
        base_frame_world_mm=assembled["base_frame_world_mm"],
        joint_values_left=assembled["joint_values_left"],
        joint_values_right=assembled["joint_values_right"],
        joint_names_left=assembled["joint_names_left"],
        joint_names_right=assembled["joint_names_right"],
    )

    held_bar_id_pre = rs.GetUserText(held_oid, "bar_id") or str(held_oid)
    env_geom = env_collision.collect_built_geometry(held_bar_id_pre, get_bar_seq_map())
    deps = robot_cell.import_compas_stack()
    if env_collision.register_env_in_robot_cell(rcell, env_geom, deps=deps):
        # Force _ensure_support_cell_loaded to re-push the cell on the next
        # solve_support_ik call (env rigid bodies were added/removed).
        robot_cell._STICKY[robot_cell._STICKY_CURRENT_CELL_KIND] = None
    template_state = env_collision.build_env_state(template_state, env_geom)
    print(f"RSIKSupportKeyframe: env collision -- {env_collision.list_env_summary(env_geom)}")

    bar_curve = rs.coercecurve(held_oid)
    if bar_curve is None:
        rs.MessageBox("Could not coerce held bar to a Curve.", 0, "RSIKSupportKeyframe")
        return

    preview_oids = []
    da_oids = []
    env_token = None
    keep_highlight = False
    try:
        try:
            da_oids = _bake_dual_arm_at_captured_pose(assembled, ik_viz.get_mesh_mode())
            sc.doc.Views.Redraw()
        except Exception as exc:
            print(
                f"RSIKSupportKeyframe: failed to bake dual-arm collision preview "
                f"({exc}); proceeding without."
            )
            da_oids = []

        grasp_mm, tool0_mm = _pick_grasp_frame_on_bar(bar_curve, gripper_kind="Robotiq")
        if grasp_mm is None:
            return

        try:
            preview_oids.append(_insert_robotiq_at_tool0(tool0_mm))
        except RuntimeError as exc:
            rs.MessageBox(str(exc), 0, "RSIKSupportKeyframe")
            return
        sc.doc.Views.Redraw()
        if not _ask_accept(
            "Inspect Robotiq gripper preview at the picked grasp pose. "
            "Accept to proceed to base-point selection"
        ):
            print("RSIKSupportKeyframe: cancelled at gripper preview.")
            return

        brep_id = _pick_walkable_brep()
        if brep_id is None:
            return
        base_origin_mm, base_normal, heading_mm, seed_base_frame = _pick_base_frame_on_walkable(brep_id)
        if seed_base_frame is None:
            return

        env_token = highlight_env.highlight_env_for_ik(held_bar_id_pre)

        collision_opts = _ask_collision_options(env_count=len(env_geom))
        if collision_opts is None:
            return
        include_self, include_env, mesh_mode = collision_opts

        held_bar_id = held_bar_id_pre
        assembled_bar_id = rs.GetUserText(assembled_oid, "bar_id") or str(assembled_oid)

        print("RSIKSupportKeyframe: solving support IK...")
        final_state, final_base = _solve_with_sampling(
            planner, template_state, seed_base_frame,
            tool0_mm, brep_id, heading_mm, include_self, include_env,
        )
        if final_state is None:
            if _ask_save_failure_capture("Support"):
                seed_state = template_state.copy()
                robot_cell_support._apply_base_frame_mm(seed_state, seed_base_frame)
                _save_capture(
                    held_bar_id=held_bar_id,
                    assembled_bar_id=assembled_bar_id,
                    grasp_world_mm=grasp_mm,
                    tool0_world_mm=tool0_mm,
                    initial_state=seed_state,
                    assembled_payload=assembled,
                    include_self=include_self,
                    include_env=include_env,
                    final_state=None,
                    rcell=rcell,
                    suffix="_ik_fail",
                )
            rs.MessageBox("Support IK failed (all samples exhausted).", 0, "RSIKSupportKeyframe")
            return

        robot_cell_support.set_cell_state(planner, final_state)
        ik_viz.show_state(final_state, mesh_mode=mesh_mode, robot_model=rcell.robot_model)
        print("RSIKSupportKeyframe: support pose reachable. Previewing...")

        if _ask_accept():
            payload = {
                "robot_id": config.SUPPORT_ROBOT_ID,
                "base_frame_world_mm": np.asarray(final_base, dtype=float).tolist(),
                "final": robot_cell_support.extract_group_config(
                    final_state, config.SUPPORT_GROUP, rcell
                ),
                "grasp_frame_world_mm": np.asarray(grasp_mm, dtype=float).tolist(),
                "tool0_frame_world_mm": np.asarray(tool0_mm, dtype=float).tolist(),
                "gripper_kind": "Robotiq",
                "linked_assembled_bar_id": assembled_bar_id,
                "include_self_collision": bool(include_self),
                "include_env_collision": bool(include_env),
            }
            rs.SetUserText(held_oid, config.IK_SUPPORT_KEY, json.dumps(payload))
            print(f"RSIKSupportKeyframe: saved '{config.IK_SUPPORT_KEY}' on bar {held_bar_id}.")
            # Capture is a debug artefact (production state lives on the bar
            # user-text above). Opt-in, default No, symmetric with failure path.
            if _ask_save_debug_capture():
                seed_state = template_state.copy()
                robot_cell_support._apply_base_frame_mm(seed_state, final_base)
                _save_capture(
                    held_bar_id=held_bar_id,
                    assembled_bar_id=assembled_bar_id,
                    grasp_world_mm=grasp_mm,
                    tool0_world_mm=tool0_mm,
                    initial_state=seed_state,
                    assembled_payload=assembled,
                    include_self=include_self,
                    include_env=include_env,
                    final_state=final_state,
                    rcell=rcell,
                )
            keep_highlight = True
        else:
            print("RSIKSupportKeyframe: rejected; bar user-text unchanged.")

    finally:
        _cleanup_ids(preview_oids)
        _cleanup_ids(da_oids)
        if env_token is not None and not keep_highlight:
            highlight_env.revert_env_highlight(env_token)
        ik_viz.clear_scene()


if __name__ == "__main__":
    main()
