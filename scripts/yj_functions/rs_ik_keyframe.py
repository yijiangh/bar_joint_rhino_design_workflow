#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSIKKeyframe - Dual-arm IK keyframe workflow.

Pick two male joint blocks (left arm, then right arm). Their joints must
sit on the same Ln bar (shared `male_parent_bar`). The script then:

1. Inserts "pineapple" (wrist + tool) block instances at the derived tool0
   frames so the user can eyeball collisions.
2. Prompts for a base point on a Brep in the `WalkableGround` layer and a
   heading point defining the base X-axis.
3. Solves dual-arm IK (left then right group). If unreachable, samples
   base frames in a circle around the pick and re-snaps each to the same
   Brep face.
4. Previews the robot via `core.ik_viz`.
5. Repeats 1-4 for the approach pose, offset along
   `-unit(avg(male_z_L, male_z_R)) * LM_DISTANCE`.
6. On accept, writes `ik_assembly` user-text (JSON payload) on the shared
   Ln bar axis line; pineapple preview and robot meshes are cleared.
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
from core.rhino_bar_registry import get_bar_seq_map, repair_on_entry
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import set_objects_layer, suspend_redraw


# ---------------------------------------------------------------------------
# Constants / user-text keys
# ---------------------------------------------------------------------------

IK_ASSEMBLY_KEY = "ik_assembly"
PINEAPPLE_ROLE_KEY = "_ik_pineapple_role"
PINEAPPLE_LAYER = "IKPineapplePreview"


# ---------------------------------------------------------------------------
# Module reload (matches rs_joint_place.py pattern)
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    global capture_io, config, dynamic_preview, env_collision, highlight_env, ik_viz, robot_cell
    config = importlib.reload(_config_module)
    dynamic_preview = importlib.reload(_dynamic_preview_module)
    env_collision = importlib.reload(_env_collision_module)
    highlight_env = importlib.reload(_highlight_env_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    capture_io = importlib.reload(_capture_io_module)


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Rhino <-> numpy helpers (doc units -> mm)
# ---------------------------------------------------------------------------


def _rhino_xform_to_np_mm(xform):
    scale = doc_unit_scale_to_mm()
    matrix = np.array([[float(xform[i, j]) for j in range(4)] for i in range(4)], dtype=float)
    matrix[:3, 3] *= scale  # translation -> mm
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


def _block_instance_xform_mm(object_id) -> np.ndarray:
    """Return the block instance's world transform as a 4x4 numpy matrix in mm."""
    obj = rs.coercerhinoobject(object_id, True, True)
    if not isinstance(obj, Rhino.DocObjects.InstanceObject):
        raise RuntimeError(f"Object {object_id} is not a block instance.")
    return _rhino_xform_to_np_mm(obj.InstanceXform)


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


def _insert_pineapple(block_name, frame_mm, role):
    oid = rs.InsertBlock(block_name, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(oid, _np_mm_to_rhino_xform(frame_mm))
    rs.SetUserText(oid, PINEAPPLE_ROLE_KEY, role)
    set_objects_layer(oid, PINEAPPLE_LAYER)
    return oid


def _insert_pineapples(tool0_left_mm, tool0_right_mm):
    _require_block_definition(config.LEFT_PINEAPPLE_BLOCK)
    _require_block_definition(config.RIGHT_PINEAPPLE_BLOCK)
    with suspend_redraw():
        left = _insert_pineapple(config.LEFT_PINEAPPLE_BLOCK, tool0_left_mm, "left")
        right = _insert_pineapple(config.RIGHT_PINEAPPLE_BLOCK, tool0_right_mm, "right")
    return [left, right]


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
# Male joint picking
# ---------------------------------------------------------------------------


def _is_male_block(oid) -> bool:
    return rs.GetUserText(oid, "joint_subtype") == "Male"


def _male_filter(rhino_object, _geometry, _component_index):
    return _is_male_block(rhino_object.Id)


def _pick_male_joint(prompt):
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    go.EnablePreSelect(False, False)
    go.SetCustomGeometryFilter(_male_filter)
    if go.Get() != Rhino.Input.GetResult.Object:
        return None
    return go.Object(0).ObjectId


# ---------------------------------------------------------------------------
# WalkableGround picking
# ---------------------------------------------------------------------------


_WALKABLE_FILTER = rs.filter.surface | rs.filter.polysurface | rs.filter.extrusion


def _as_brep(object_id):
    """Return the Brep geometry for `object_id`, converting from Extrusion if needed.

    `rs.coercebrep` returns None for Rhino Extrusion objects (e.g. a closed
    box primitive); we call `.ToBrep()` on the extrusion geometry so the
    downstream closest-point + face-normal code works uniformly.
    """
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
            "RSIKKeyframe",
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
            "RSIKKeyframe",
        )
        return None
    return oid


def _closest_point_on_brep(brep, point_doc):
    """Return (point_doc, normal_world) of the closest surface point to `point_doc`."""
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


def _bake_robot_meshes_at_zero():
    """Bake the dual-arm robot URDF visual meshes once at zero config + identity
    base, capture as Rhino.Geometry.Mesh values (in base-link local coords,
    doc units), then delete the baked guids.
    """
    cell = robot_cell.get_or_load_robot_cell()
    state = robot_cell.default_cell_state()
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


def _pick_base_frame_on_walkable(brep_id):
    """Pick base origin + heading on the walkable brep, with the dual-arm
    robot's mesh tracking the cursor. Returns
    (base_origin_mm, base_normal, heading_mm, seed_base_frame_mm) or all None.
    """
    brep = _as_brep(brep_id)
    robot_meshes = _bake_robot_meshes_at_zero()

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
        rs.MessageBox(str(exc), 0, "RSIKKeyframe")
        return None, None, None, None
    return base_origin_mm, normal_v, heading_mm, seed_base


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
# Interactive prompts
# ---------------------------------------------------------------------------


def _ask_collision_options(env_count: int = 0):
    """Prompt for IK collision flags + the robot mesh display mode.

    Returns ``(include_self, include_env, mesh_mode)`` or ``None`` on cancel.
    The `mesh_mode` toggle picks between ``ik_viz.MESH_MODE_VISUAL`` and
    ``MESH_MODE_COLLISION`` for `ik_viz.show_state`. The previous choice is
    cached in `sc.sticky` (via ik_viz.set_mesh_mode) so subsequent runs in
    the same session start with the user's last selection pre-toggled.
    """
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


def _ask_accept(prompt="Accept this IK keyframe and save it on the bar"):
    answer = rs.GetString(prompt, "Accept", ["Accept", "Reject"])
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("a")


# ---------------------------------------------------------------------------
# IK solving with base sampling
# ---------------------------------------------------------------------------


def _sample_base_offsets(count, radius_mm):
    rng = random.Random()
    for _ in range(count):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        r = rng.uniform(0.0, radius_mm)
        yield np.array([r * math.cos(angle), r * math.sin(angle), 0.0], dtype=float)


def _snap_to_brep(brep, origin_mm):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    pt_doc = Rhino.Geometry.Point3d(*(origin_mm * scale_from_mm))
    close_pt, normal = _closest_point_on_brep(brep, pt_doc)
    if close_pt is None:
        return None, None
    return _point_to_mm(close_pt), np.array([normal.X, normal.Y, normal.Z], dtype=float)


def _solve_with_sampling(planner, template_state, seed_base_frame_mm,
                         tool0_left_mm, tool0_right_mm,
                         brep_id, heading_mm, include_self, include_env):
    """Try seed first; on failure, sample base frames around seed until success."""
    check_collision = bool(include_self or include_env)
    # Environment collision is only meaningful when env geometry is registered in
    # the robot cell; first-pass workflow has none, so include_env alone is a hint.
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
            f"RSIKKeyframe: trying base frame ({label}) "
            f"at ({origin[0]:.1f}, {origin[1]:.1f}, {origin[2]:.1f}) mm ..."
        )
        state = robot_cell.solve_dual_arm_ik(
            planner,
            template_state,
            base_frame,
            tool0_left_mm,
            tool0_right_mm,
            check_collision=check_collision,
            verbose_pairs=check_collision,
        )
        if state is not None:
            print(
                f"RSIKKeyframe: [OK] IK solution FOUND on attempt {idx + 1}/{total} ({label})."
            )
            return state, base_frame
        print(f"RSIKKeyframe: [x] IK failed on attempt {idx + 1}/{total} ({label}).")
    print(
        f"RSIKKeyframe: [X] IK failed for all {total} attempt(s). "
        f"Consider increasing IK_BASE_SAMPLE_RADIUS / IK_BASE_SAMPLE_MAX_ITER in config.py."
    )
    return None, None


# ---------------------------------------------------------------------------
# Target-bar resolution & payload
# ---------------------------------------------------------------------------


def _resolve_target_bar(male_left_oid, male_right_oid):
    ln_left = rs.GetUserText(male_left_oid, "male_parent_bar")
    ln_right = rs.GetUserText(male_right_oid, "male_parent_bar")
    if not ln_left or not ln_right:
        raise RuntimeError("Picked male joint blocks are missing 'male_parent_bar' user-text.")
    if ln_left != ln_right:
        raise RuntimeError(
            f"Picked male joints are on different Ln bars: left={ln_left}, right={ln_right}. "
            "Both must share the same Ln bar to be saved as a single keyframe."
        )
    seq_map = get_bar_seq_map()
    if ln_left not in seq_map:
        raise RuntimeError(f"Ln bar '{ln_left}' is not registered in the bar registry.")
    return ln_left, seq_map[ln_left][0]


def _tool0_from_male(oid, arm_side):
    """Return `(tool0_world_mm, ocf_world_mm)` for a picked male-joint block.

    `arm_side` is "left" or "right" — selects which per-arm transform to use
    from the nested `MALE_JOINT_OCF_TO_TOOL0[joint_type][arm_side]` dict.
    """
    # Dispatch by full block-definition name (e.g. "T20_Male"), which matches
    # the convention used by RSExportGraspTool0TF. Fall back to the composite
    # of `joint_type` + `joint_subtype` user-text for instances whose block
    # name has been renamed.
    key = rs.BlockInstanceName(oid)
    if key not in config.MALE_JOINT_OCF_TO_TOOL0:
        jtype = rs.GetUserText(oid, "joint_type")
        jsubtype = rs.GetUserText(oid, "joint_subtype")
        fallback = f"{jtype}_{jsubtype}" if jtype and jsubtype else None
        if fallback and fallback in config.MALE_JOINT_OCF_TO_TOOL0:
            key = fallback
        else:
            raise RuntimeError(
                f"No MALE_JOINT_OCF_TO_TOOL0 entry for '{key}'. "
                "Run RSExportGraspTool0TF (Joint mode) for this joint type first."
            )
    per_side = config.MALE_JOINT_OCF_TO_TOOL0[key]
    if arm_side not in per_side:
        raise RuntimeError(
            f"MALE_JOINT_OCF_TO_TOOL0['{key}'] has no '{arm_side}' entry. "
            f"Run RSExportGraspTool0TF (Joint mode) for the {arm_side} arm."
        )
    tf = per_side[arm_side]
    ocf = _block_instance_xform_mm(oid)
    return ocf @ tf, ocf


def _translate_frame(frame_mm, offset_mm):
    out = np.array(frame_mm, dtype=float, copy=True)
    out[:3, 3] = out[:3, 3] + np.asarray(offset_mm, dtype=float)
    return out


def _build_assembly_payload(base_frame_mm, final_state, approach_state, rcell):
    return {
        "robot_id": config.ROBOT_ID,
        "base_frame_world_mm": np.asarray(base_frame_mm, dtype=float).tolist(),
        "final": {
            "left": robot_cell.extract_group_config(final_state, config.LEFT_GROUP, rcell),
            "right": robot_cell.extract_group_config(final_state, config.RIGHT_GROUP, rcell),
        },
        "approach": {
            "left": robot_cell.extract_group_config(approach_state, config.LEFT_GROUP, rcell),
            "right": robot_cell.extract_group_config(approach_state, config.RIGHT_GROUP, rcell),
        },
    }


# ---------------------------------------------------------------------------
# Capture-and-replay
# ---------------------------------------------------------------------------


_CAPTURES_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "tests", "captures"))


def _ask_save_failure_capture(phase_label: str) -> bool:
    """Prompt 'IK failed at <phase>; save capture for headless debugging?'

    Default is No because most users will iterate inside Rhino without
    needing the headless replay. Returns True iff the user typed "Yes".
    """
    answer = rs.GetString(
        f"{phase_label} IK failed. Save capture for headless debugging?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


def _ask_save_debug_capture() -> bool:
    """Prompt 'Save debug capture for headless replay?' on accept.

    Production state already lives on the bar user-text; the capture is a
    debug artefact (RobotCell geometry + IK targets + options) needed only
    when a bug is being investigated headless. Default No.
    """
    answer = rs.GetString(
        "Save debug capture for headless replay?",
        "No",
        ["Yes", "No"],
    )
    if answer is None:
        return False
    return str(answer).strip().lower().startswith("y")


def _save_capture(
    *,
    target_bar_id,
    left_oid,
    right_oid,
    ocf_left,
    ocf_right,
    tool0_left_final,
    tool0_right_final,
    tool0_left_approach,
    tool0_right_approach,
    initial_state,
    include_self,
    include_env,
    final_state=None,
    approach_state=None,
    rcell=None,
    suffix: str = "",
) -> str | None:
    """Write a v2 capture JSON into ``tests/captures/``.

    The serialized ``RobotCell`` (which already has env rigid bodies
    registered by this point) goes to ``robot_cells/<sha8>.json`` and is
    referenced from the capture. ``initial_state`` carries the seed base
    frame + env rigid_body_states. ``expected`` holds the accepted final /
    approach states (or None on failure captures).
    """
    try:
        os.makedirs(_CAPTURES_DIR, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = f"{timestamp}_{target_bar_id}{suffix}"
        path = os.path.join(_CAPTURES_DIR, f"{stem}.json")

        cell_ref = capture_io.save_robot_cell_if_changed(rcell, _CAPTURES_DIR)

        ik_targets = {
            "final": {
                "left_tool0_world_mm": np.asarray(tool0_left_final, dtype=float).tolist(),
                "right_tool0_world_mm": np.asarray(tool0_right_final, dtype=float).tolist(),
            },
            "approach": {
                "left_tool0_world_mm": np.asarray(tool0_left_approach, dtype=float).tolist(),
                "right_tool0_world_mm": np.asarray(tool0_right_approach, dtype=float).tolist(),
            },
        }
        ik_options = {
            "check_collision": bool(include_self or include_env),
            "include_self": bool(include_self),
            "include_env": bool(include_env),
        }
        source = {
            "target_bar_id": target_bar_id,
            "left_block_name": rs.BlockInstanceName(left_oid),
            "right_block_name": rs.BlockInstanceName(right_oid),
            "left_arm_side": "left",
            "right_arm_side": "right",
            "left_ocf_world_mm": np.asarray(ocf_left, dtype=float).tolist(),
            "right_ocf_world_mm": np.asarray(ocf_right, dtype=float).tolist(),
            "lm_distance_mm": float(config.LM_DISTANCE),
            "doc_unit_scale_to_mm": float(doc_unit_scale_to_mm()),
        }
        capture_io.save_capture_v2(
            path,
            captured_at=datetime.datetime.now().isoformat(timespec="seconds"),
            robot_cell_ref=cell_ref,
            initial_state=initial_state,
            ik_targets=ik_targets,
            ik_options=ik_options,
            expected={"final": final_state, "approach": approach_state},
            source=source,
        )
        rel = os.path.relpath(path, os.path.dirname(SCRIPT_DIR))
        print(f"RSIKKeyframe: capture saved -> {rel}")
        return path
    except Exception as exc:
        print(f"RSIKKeyframe: failed to write capture ({exc}); continuing.")
        return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSIKKeyframe")

    if not robot_cell.is_pb_running():
        rs.MessageBox(
            "PyBullet is not running. Click RSPBStart first.", 0, "RSIKKeyframe"
        )
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()
    # Use the module-level wrapper so tool_states[AssemblyLeftTool/AssemblyRightTool] get attached_to_group
    # + touch_links wired up. `rcell.default_cell_state()` alone returns a
    # state with tools un-attached, defeating the IK collision check on tools.
    template_state = robot_cell.default_cell_state()

    rs.UnselectAllObjects()
    left_oid = _pick_male_joint("Pick LEFT arm male joint block")
    if left_oid is None:
        return
    right_oid = _pick_male_joint("Pick RIGHT arm male joint block")
    if right_oid is None:
        return

    try:
        target_bar_id, target_bar_oid = _resolve_target_bar(left_oid, right_oid)
        tool0_left_final, ocf_left = _tool0_from_male(left_oid, arm_side="left")
        tool0_right_final, ocf_right = _tool0_from_male(right_oid, arm_side="right")
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSIKKeyframe")
        return
    print(f"RSIKKeyframe: target Ln bar = {target_bar_id}")

    env_geom = env_collision.collect_built_geometry(target_bar_id, get_bar_seq_map())
    robot_cell.ensure_env_registered(rcell, env_geom, planner)
    template_state = env_collision.build_env_state(template_state, env_geom)
    print(f"RSIKKeyframe: env collision -- {env_collision.list_env_summary(env_geom)}")

    pineapple_ids = []
    env_token = None
    keep_highlight = False
    try:
        try:
            pineapple_ids = _insert_pineapples(tool0_left_final, tool0_right_final)
        except RuntimeError as exc:
            rs.MessageBox(str(exc), 0, "RSIKKeyframe")
            return

        sc.doc.Views.Redraw()
        if not _ask_accept(
            "Inspect pineapple (wrist+tool) preview at the FINAL target. "
            "Accept to proceed to base-point selection"
        ):
            print("RSIKKeyframe: cancelled at pineapple preview.")
            return

        brep_id = _pick_walkable_brep()
        if brep_id is None:
            return
        base_origin_mm, base_normal, heading_mm, seed_base_frame = _pick_base_frame_on_walkable(brep_id)
        if seed_base_frame is None:
            return

        env_token = highlight_env.highlight_env_for_ik(target_bar_id)

        collision_opts = _ask_collision_options(env_count=len(env_geom))
        if collision_opts is None:
            return
        include_self, include_env, mesh_mode = collision_opts

        # Approach target: translate tool0 frames along -avg(male z) * LM_DISTANCE
        z_avg_pre = (ocf_left[:3, 2] + ocf_right[:3, 2]) / 2.0
        try:
            approach_dir_pre = -_unit(z_avg_pre)
        except ValueError:
            rs.MessageBox("Male joint z-axes sum to zero; cannot derive approach direction.", 0, "RSIKKeyframe")
            return
        offset_pre = approach_dir_pre * float(config.LM_DISTANCE)
        tool0_left_approach = _translate_frame(tool0_left_final, offset_pre)
        tool0_right_approach = _translate_frame(tool0_right_final, offset_pre)

        print("RSIKKeyframe: solving final-target IK...")
        final_state, final_base = _solve_with_sampling(
            planner, template_state, seed_base_frame,
            tool0_left_final, tool0_right_final,
            brep_id, heading_mm, include_self, include_env,
        )
        if final_state is None:
            if _ask_save_failure_capture("Final-target"):
                seed_state = template_state.copy()
                robot_cell._apply_base_frame_mm(seed_state, seed_base_frame)
                _save_capture(
                    target_bar_id=target_bar_id,
                    left_oid=left_oid,
                    right_oid=right_oid,
                    ocf_left=ocf_left,
                    ocf_right=ocf_right,
                    tool0_left_final=tool0_left_final,
                    tool0_right_final=tool0_right_final,
                    tool0_left_approach=tool0_left_approach,
                    tool0_right_approach=tool0_right_approach,
                    initial_state=seed_state,
                    include_self=include_self,
                    include_env=include_env,
                    final_state=None,
                    approach_state=None,
                    rcell=rcell,
                    suffix="_ik_fail_final",
                )
            rs.MessageBox("IK failed for the final target (all samples exhausted).", 0, "RSIKKeyframe")
            return
        # Sync the PyBullet world to the new state, then update the Rhino viz
        # (mirrors GH_set_cell_state.py + GH_scene_viz.py ordering).
        robot_cell.set_cell_state(planner, final_state)
        ik_viz.show_state(final_state, mesh_mode=mesh_mode)
        print("RSIKKeyframe: final target reachable. Previewing...")

        # Refresh pineapple preview to approach pose
        _cleanup_ids(pineapple_ids)
        pineapple_ids = _insert_pineapples(tool0_left_approach, tool0_right_approach)

        print("RSIKKeyframe: solving approach-target IK...")
        approach_state, approach_base = _solve_with_sampling(
            planner, template_state, final_base,
            tool0_left_approach, tool0_right_approach,
            brep_id, heading_mm, include_self, include_env,
        )
        if approach_state is None:
            if _ask_save_failure_capture("Approach-target"):
                seed_state = template_state.copy()
                robot_cell._apply_base_frame_mm(seed_state, final_base)
                _save_capture(
                    target_bar_id=target_bar_id,
                    left_oid=left_oid,
                    right_oid=right_oid,
                    ocf_left=ocf_left,
                    ocf_right=ocf_right,
                    tool0_left_final=tool0_left_final,
                    tool0_right_final=tool0_right_final,
                    tool0_left_approach=tool0_left_approach,
                    tool0_right_approach=tool0_right_approach,
                    initial_state=seed_state,
                    include_self=include_self,
                    include_env=include_env,
                    final_state=final_state,
                    approach_state=None,
                    rcell=rcell,
                    suffix="_ik_fail_approach",
                )
            rs.MessageBox("IK failed for the approach target (all samples exhausted).", 0, "RSIKKeyframe")
            return
        robot_cell.set_cell_state(planner, approach_state)
        ik_viz.show_state(approach_state, mesh_mode=mesh_mode)
        print("RSIKKeyframe: approach target reachable. Previewing...")

        # Use the same base frame for the stored record (approach_base ~ final_base since we reused).
        # If sampling drifted, prefer final_base (anchors the assembled pose).
        stored_base = final_base

        if _ask_accept():
            payload = _build_assembly_payload(stored_base, final_state, approach_state, rcell)
            rs.SetUserText(target_bar_oid, IK_ASSEMBLY_KEY, json.dumps(payload))
            print(f"RSIKKeyframe: saved '{IK_ASSEMBLY_KEY}' on bar {target_bar_id}.")
            # Capture file is a debug artefact (not production state — production
            # data lives in the bar user-text above). Opt-in, symmetric with the
            # failure path. Default No.
            if _ask_save_debug_capture():
                seed_state = template_state.copy()
                robot_cell._apply_base_frame_mm(seed_state, stored_base)
                _save_capture(
                    target_bar_id=target_bar_id,
                    left_oid=left_oid,
                    right_oid=right_oid,
                    ocf_left=ocf_left,
                    ocf_right=ocf_right,
                    tool0_left_final=tool0_left_final,
                    tool0_right_final=tool0_right_final,
                    tool0_left_approach=tool0_left_approach,
                    tool0_right_approach=tool0_right_approach,
                    initial_state=seed_state,
                    include_self=include_self,
                    include_env=include_env,
                    final_state=final_state,
                    approach_state=approach_state,
                    rcell=rcell,
                )
            keep_highlight = True
        else:
            print("RSIKKeyframe: rejected; bar user-text unchanged.")

    finally:
        _cleanup_ids(pineapple_ids)
        ik_viz.clear_scene()
        if env_token is not None and not keep_highlight:
            highlight_env.revert_env_highlight(env_token)


if __name__ == "__main__":
    main()
