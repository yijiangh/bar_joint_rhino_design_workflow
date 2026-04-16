#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSJointPlace - Place connector blocks on one bar pair.

Pick an existing bar (Le, gets female joint) and a new bar (Ln, gets male
joint). The optimizer solves 4 assembly variants. An interactive loop lets you
cycle through them with Next/Previous and Accept the current one.
"""

import contextlib
import importlib
import json
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import kinematics as _kinematics_module
from core import transforms as _transforms_module


_DEBUG_FRAME_AXIS_LENGTH = 35.0
_DEBUG_FRAME_IDS_KEY = "rs_joint_place_debug_frame_ids"
_FEMALE_BLOCK_NAME = "FemaleLinkBlock"
_MALE_BLOCK_NAME = "MaleLinkBlock"
_FEMALE_INSTANCES_LAYER = "FemaleJointPlacedInstances"
_MALE_INSTANCES_LAYER = "MaleJointPlacedInstances"
_JOINT_OPTIMIZATION_FRAMES_LAYER = "JointOptimizationFrames"
_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]


# ---------------------------------------------------------------------------
# Module reload
# ---------------------------------------------------------------------------

def _reload_runtime_modules():
    global config, invert_transform, make_bar_frame, optimize_joint_placement
    global rotation_about_local_z, optimize_female_to_target
    global screw_hole_alignment_diagnostics, screw_hole_origin_z_error

    config = importlib.reload(_config_module)
    transforms = importlib.reload(_transforms_module)
    kinematics = importlib.reload(_kinematics_module)
    invert_transform = transforms.invert_transform
    make_bar_frame = kinematics.make_bar_frame
    optimize_female_to_target = kinematics.optimize_female_to_target
    optimize_joint_placement = kinematics.optimize_joint_placement
    rotation_about_local_z = transforms.rotation_about_local_z
    screw_hole_alignment_diagnostics = kinematics.screw_hole_alignment_diagnostics
    screw_hole_origin_z_error = kinematics.screw_hole_origin_z_error


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _as_object_id_list(object_ids):
    if object_ids is None:
        return []
    if isinstance(object_ids, (str, bytes)):
        return [object_ids]
    try:
        return [oid for oid in object_ids if oid is not None]
    except TypeError:
        return [object_ids]


def _ensure_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def _set_objects_layer(object_ids, layer_name):
    baked_ids = _as_object_id_list(object_ids)
    _ensure_layer(layer_name)
    for oid in baked_ids:
        if rs.IsObject(oid):
            rs.ObjectLayer(oid, layer_name)
    return baked_ids


def _group_objects(object_ids):
    baked_ids = _as_object_id_list(object_ids)
    if not baked_ids:
        return None
    group_name = rs.AddGroup()
    if not group_name:
        return None
    rs.AddObjectsToGroup(baked_ids, group_name)
    return group_name


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


def _delete_objects(object_ids):
    for oid in _as_object_id_list(object_ids):
        if rs.IsObject(oid):
            rs.DeleteObject(oid)


@contextlib.contextmanager
def _suspend_redraw():
    previous_state = None
    redraw_supported = hasattr(rs, "EnableRedraw")
    try:
        if redraw_supported:
            previous_state = rs.EnableRedraw(False)
        yield
    finally:
        redraw_is_enabled = True
        if redraw_supported:
            redraw_is_enabled = True if previous_state is None else bool(previous_state)
            rs.EnableRedraw(redraw_is_enabled)
        if redraw_is_enabled and hasattr(rs, "Redraw"):
            rs.Redraw()


def _set_object_color(object_ids, color):
    for oid in _as_object_id_list(object_ids):
        if not rs.IsObject(oid):
            continue
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(oid, 1)
        rs.ObjectColor(oid, color)


def _has_block_definition(name):
    for instance_def in sc.doc.InstanceDefinitions:
        if instance_def is not None and not instance_def.IsDeleted and instance_def.Name == name:
            return True
    return False


def _require_block_definition(name):
    if not _has_block_definition(name):
        raise RuntimeError(f"Missing required Rhino block definition '{name}'.")
    return name


def _numpy_to_rhino_transform(matrix):
    xform = Rhino.Geometry.Transform(1.0)
    for row in range(4):
        for col in range(4):
            xform[row, col] = float(matrix[row, col])
    return xform


def _insert_block_instance(block_name, frame, *, layer_name=None, color=None):
    oid = rs.InsertBlock(block_name, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(oid, _numpy_to_rhino_transform(frame))
    if layer_name:
        _set_objects_layer(oid, layer_name)
    if color is not None:
        _set_object_color(oid, color)
    return oid


# ---------------------------------------------------------------------------
# Debug frame baking
# ---------------------------------------------------------------------------

def _clear_debug_frames():
    for oid in sc.sticky.pop(_DEBUG_FRAME_IDS_KEY, []):
        if rs.IsObject(oid):
            rs.DeleteObject(oid)


def _remember_debug_frame_ids(object_ids):
    baked = sc.sticky.get(_DEBUG_FRAME_IDS_KEY, [])
    baked.extend(oid for oid in object_ids if oid is not None)
    sc.sticky[_DEBUG_FRAME_IDS_KEY] = baked


def _bake_frame_axes(frame, label, axis_length=_DEBUG_FRAME_AXIS_LENGTH):
    frame = np.asarray(frame, dtype=float)
    origin = frame[:3, 3]
    axis_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    axis_names = ["x", "y", "z"]
    baked_ids = []
    for axis_index, (axis_name, color) in enumerate(zip(axis_names, axis_colors)):
        axis_vec = np.asarray(frame[:3, axis_index], dtype=float)
        axis_norm = float(np.linalg.norm(axis_vec))
        if axis_norm <= 1e-12:
            continue
        lid = rs.AddLine(origin.tolist(), (origin + axis_length * axis_vec / axis_norm).tolist())
        if lid is None:
            continue
        rs.ObjectColor(lid, color)
        rs.ObjectName(lid, f"{label}_{axis_name.upper()}")
        rs.SetUserText(lid, "frame_label", label)
        rs.SetUserText(lid, "axis_name", axis_name)
        baked_ids.append(lid)
    dot_id = rs.AddTextDot(label, origin.tolist())
    if dot_id is not None:
        rs.ObjectName(dot_id, f"{label}_label")
        rs.SetUserText(dot_id, "frame_label", label)
        baked_ids.append(dot_id)
    _set_objects_layer(baked_ids, _JOINT_OPTIMIZATION_FRAMES_LAYER)
    _group_objects(baked_ids)
    return baked_ids


def _bake_debug_ocf_frames(result, le_id, ln_id, prefix="RSJointPlace"):
    le_start, le_end = _curve_endpoints(le_id)
    ln_start, ln_end = _curve_endpoints(ln_id)
    baked_ids = []
    baked_ids.extend(_bake_frame_axes(make_bar_frame(le_start, le_end), f"{prefix}_Le"))
    baked_ids.extend(_bake_frame_axes(make_bar_frame(ln_start, ln_end), f"{prefix}_Ln"))
    baked_ids.extend(_bake_frame_axes(result["female_frame"], f"{prefix}_Female"))
    baked_ids.extend(_bake_frame_axes(result["male_frame"], f"{prefix}_Male"))
    _remember_debug_frame_ids(baked_ids)


# ---------------------------------------------------------------------------
# Variant enumeration (carried over from t2_joint_placement.py)
# ---------------------------------------------------------------------------

def _apply_local_z_flip(frame, angle_rad):
    return np.asarray(frame, dtype=float) @ rotation_about_local_z(float(angle_rad))


def _build_variant_result(base_result, *, female_flip_rad, male_flip_rad, variant_index):
    female_frame = _apply_local_z_flip(base_result["female_frame"], female_flip_rad)
    male_frame = _apply_local_z_flip(base_result["male_frame"], male_flip_rad)
    female_screw_hole_frame = female_frame @ np.asarray(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM, dtype=float)
    male_screw_hole_frame = male_frame @ invert_transform(np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float))
    diagnostics = screw_hole_alignment_diagnostics(female_screw_hole_frame, male_screw_hole_frame)
    result = dict(base_result)
    result.update({
        "female_frame": female_frame,
        "male_frame": male_frame,
        "female_screw_hole_frame": female_screw_hole_frame,
        "male_screw_hole_frame": male_screw_hole_frame,
        "residual": screw_hole_origin_z_error(
            female_screw_hole_frame, male_screw_hole_frame,
            orientation_weight_mm=float(getattr(config, "BAR_CONTACT_DISTANCE", 1.0)),
        ),
        "origin_error_mm": diagnostics["origin_error_mm"],
        "z_axis_error_rad": diagnostics["z_axis_error_rad"],
        "variant_index": int(variant_index),
        "female_flip_rad": float(female_flip_rad),
        "male_flip_rad": float(male_flip_rad),
    })
    return result


def _solve_variant_after_male_flip(base_result, le_start, le_end):
    male_frame = _apply_local_z_flip(base_result["male_frame"], np.pi)
    male_screw_hole_frame = male_frame @ invert_transform(np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float))
    female_result = optimize_female_to_target(
        le_start, le_end, male_screw_hole_frame, config,
        initial_guess=(base_result["fjp"], base_result["fjr"]),
        return_debug=True,
    )
    result = dict(base_result)
    result.update({
        "fjp": float(female_result["fjp"]),
        "fjr": float(female_result["fjr"]),
        "residual": float(female_result["residual"]),
        "origin_error_mm": float(female_result["origin_error_mm"]),
        "z_axis_error_rad": float(female_result["z_axis_error_rad"]),
        "female_frame": female_result["female_frame"],
        "female_screw_hole_frame": female_result["female_screw_hole_frame"],
        "male_frame": male_frame,
        "male_screw_hole_frame": male_screw_hole_frame,
        "optimizer_debug": female_result.get("optimizer_debug"),
        "variant_solver": "female_reoptimized_after_male_flip",
        "female_flip_rad": 0.0,
        "male_flip_rad": float(np.pi),
    })
    return result


def _enumerate_solution_variants(base_result, le_start, le_end):
    v1 = dict(base_result)
    v1.update({
        "variant_index": 0,
        "variant_label": "1: female=0deg, male=0deg",
        "variant_solver": "full_joint_optimization",
        "female_flip_rad": 0.0,
        "male_flip_rad": 0.0,
    })

    v2 = _build_variant_result(base_result, female_flip_rad=np.pi, male_flip_rad=0.0, variant_index=1)
    v2["variant_label"] = "2: female=180deg, male=0deg"
    v2["variant_solver"] = "full_joint_optimization"

    v3 = _solve_variant_after_male_flip(base_result, le_start, le_end)
    v3["variant_index"] = 2
    v3["variant_label"] = "3: female reopt, male=180deg"

    v4 = _build_variant_result(v3, female_flip_rad=np.pi, male_flip_rad=0.0, variant_index=3)
    v4["variant_label"] = "4: female=180deg, male=180deg"
    v4["variant_solver"] = "female_reoptimized_after_male_flip"
    v4["male_flip_rad"] = float(np.pi)
    return [v1, v2, v3, v4]


# ---------------------------------------------------------------------------
# Interface metrics
# ---------------------------------------------------------------------------

def _interface_metrics_from_result(result):
    if "origin_error_mm" in result and "z_axis_error_rad" in result:
        return float(result["origin_error_mm"]), float(result["z_axis_error_rad"])
    if "female_screw_hole_frame" in result and "male_screw_hole_frame" in result:
        diagnostics = screw_hole_alignment_diagnostics(
            result["female_screw_hole_frame"], result["male_screw_hole_frame"],
        )
        return float(diagnostics["origin_error_mm"]), float(diagnostics["z_axis_error_rad"])
    raise KeyError("Result is missing interface diagnostics.")


# ---------------------------------------------------------------------------
# Interactive cycling loop
# ---------------------------------------------------------------------------

def _show_variant_preview(variant, female_block_name, male_block_name):
    """Show one variant at the real position. Returns list of preview object IDs."""
    color = _PREVIEW_COLORS[variant["variant_index"] % len(_PREVIEW_COLORS)]
    female_id = _insert_block_instance(female_block_name, variant["female_frame"], color=color)
    male_id = _insert_block_instance(male_block_name, variant["male_frame"], color=color)
    return _as_object_id_list([female_id, male_id])


def _interactive_variant_loop(variants, female_block_name, male_block_name):
    """Cycle through variants with Next/Previous/Accept. Returns chosen variant or None."""
    current = 0
    total = len(variants)
    preview_ids = []

    try:
        # Show first variant
        with _suspend_redraw():
            preview_ids = _show_variant_preview(variants[current], female_block_name, male_block_name)
        _print_variant_info(variants[current], current, total)

        while True:
            go = Rhino.Input.Custom.GetOption()
            go.SetCommandPrompt(f"Variant {current + 1} of {total}")
            go.SetCommandPromptDefault("Accept")
            go.AcceptNothing(True)

            next_idx = go.AddOption("Next")
            prev_idx = go.AddOption("Previous")
            accept_idx = go.AddOption("Accept")

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return None

            if result == Rhino.Input.GetResult.Nothing:
                # Enter = accept current
                return variants[current]

            if result == Rhino.Input.GetResult.Option:
                opt_idx = go.Option().Index

                if opt_idx == accept_idx:
                    return variants[current]

                if opt_idx == next_idx:
                    with _suspend_redraw():
                        _delete_objects(preview_ids)
                        current = (current + 1) % total
                        preview_ids = _show_variant_preview(variants[current], female_block_name, male_block_name)
                    _print_variant_info(variants[current], current, total)
                    continue

                if opt_idx == prev_idx:
                    with _suspend_redraw():
                        _delete_objects(preview_ids)
                        current = (current - 1) % total
                        preview_ids = _show_variant_preview(variants[current], female_block_name, male_block_name)
                    _print_variant_info(variants[current], current, total)
                    continue
    finally:
        _delete_objects(preview_ids)


def _print_variant_info(variant, current, total):
    origin_err, z_err = _interface_metrics_from_result(variant)
    print(
        f"RSJointPlace: Showing variant {current + 1}/{total} — "
        f"{variant.get('variant_label', '?')} | "
        f"residual={variant['residual']:.6f}, "
        f"origin err={origin_err:.4f} mm, "
        f"z err={np.degrees(z_err):.4f}deg"
    )


# ---------------------------------------------------------------------------
# Place final blocks
# ---------------------------------------------------------------------------

def _place_joint_blocks(result, le_id, ln_id):
    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_err, z_err = _interface_metrics_from_result(result)

    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)

    joint_data_female = {
        "type": female_block_name,
        "dof": {"fjp": result["fjp"], "fjr": result["fjr"]},
        "bar_id": str(le_id),
        "connected_bar_id": str(ln_id),
        "transform": female_frame.tolist(),
        "residual": result["residual"],
        "origin_error_mm": origin_err,
        "z_axis_error_rad": z_err,
        "variant_index": result.get("variant_index"),
        "female_flip_rad": result.get("female_flip_rad", 0.0),
        "male_flip_rad": result.get("male_flip_rad", 0.0),
        "variant_solver": result.get("variant_solver"),
    }
    joint_data_male = {
        "type": male_block_name,
        "dof": {"mjp": result["mjp"], "mjr": result["mjr"]},
        "bar_id": str(ln_id),
        "connected_bar_id": str(le_id),
        "transform": male_frame.tolist(),
        "residual": result["residual"],
        "origin_error_mm": origin_err,
        "z_axis_error_rad": z_err,
        "variant_index": result.get("variant_index"),
        "female_flip_rad": result.get("female_flip_rad", 0.0),
        "male_flip_rad": result.get("male_flip_rad", 0.0),
        "variant_solver": result.get("variant_solver"),
    }

    with _suspend_redraw():
        female_id = _insert_block_instance(female_block_name, female_frame, layer_name=_FEMALE_INSTANCES_LAYER)
        male_id = _insert_block_instance(male_block_name, male_frame, layer_name=_MALE_INSTANCES_LAYER)
        rs.SetUserText(female_id, "joint_data", json.dumps(joint_data_female, indent=2))
        rs.SetUserText(male_id, "joint_data", json.dumps(joint_data_male, indent=2))
        _bake_debug_ocf_frames(result, le_id, ln_id)

    print(f"RSJointPlace: Joints placed. Residual: {result['residual']:.6f}")
    if result.get("variant_index") is not None:
        print(
            f"  Variant {int(result['variant_index']) + 1}: "
            f"female flip={np.degrees(float(result.get('female_flip_rad', 0.0))):.1f}deg, "
            f"male flip={np.degrees(float(result.get('male_flip_rad', 0.0))):.1f}deg"
        )
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg"
    )
    print(f"  Interface origin err={origin_err:.4f} mm, z-axis err={np.degrees(z_err):.4f}deg")
    return female_id, male_id


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    _reload_runtime_modules()
    _clear_debug_frames()

    rs.UnselectAllObjects()
    le_id = rs.GetObject("Select existing bar (Le) - gets FEMALE joint", rs.filter.curve)
    if le_id is None:
        return
    ln_id = rs.GetObject("Select new bar (Ln) - gets MALE joint", rs.filter.curve)
    if ln_id is None:
        return

    le_start, le_end = _curve_endpoints(le_id)
    ln_start, ln_end = _curve_endpoints(ln_id)

    # Solve
    base_result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config, return_debug=True)
    variants = _enumerate_solution_variants(base_result, le_start, le_end)

    # Validate block definitions before showing the interactive loop
    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)

    # Print all variants summary
    print("RSJointPlace: 4 assembly variants computed:")
    for v in variants:
        origin_err, z_err = _interface_metrics_from_result(v)
        print(
            f"  {v['variant_label']} | residual={v['residual']:.6f}, "
            f"origin err={origin_err:.4f} mm, z err={np.degrees(z_err):.4f}deg"
        )

    # Interactive cycling loop
    chosen = _interactive_variant_loop(variants, female_block_name, male_block_name)
    if chosen is None:
        print("RSJointPlace: Cancelled.")
        return

    _place_joint_blocks(chosen, le_id, ln_id)


if __name__ == "__main__":
    main()
