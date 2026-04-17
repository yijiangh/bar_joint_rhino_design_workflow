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
from core.rhino_helpers import (
    as_object_id_list,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    group_objects,
    point_to_array,
    set_object_color,
    set_objects_layer,
    suspend_redraw,
)
from core.rhino_bar_registry import ensure_bar_id


_DEBUG_FRAME_AXIS_LENGTH = 35.0
_DEBUG_FRAME_IDS_KEY = "rs_joint_place_debug_frame_ids"
_FEMALE_BLOCK_NAME = "T20_Female"
_MALE_BLOCK_NAME = "T20_Male"
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
    global screw_hole_alignment_diagnostics, screw_hole_origin_z_error

    config = importlib.reload(_config_module)
    transforms = importlib.reload(_transforms_module)
    kinematics = importlib.reload(_kinematics_module)
    invert_transform = transforms.invert_transform
    make_bar_frame = kinematics.make_bar_frame
    optimize_joint_placement = kinematics.optimize_joint_placement
    screw_hole_alignment_diagnostics = kinematics.screw_hole_alignment_diagnostics
    screw_hole_origin_z_error = kinematics.screw_hole_origin_z_error


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Generic helpers (point_to_array, curve_endpoints, as_object_id_list,
# ensure_layer, set_objects_layer, group_objects, delete_objects,
# set_object_color, suspend_redraw) are imported from core.rhino_helpers.


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
        set_objects_layer(oid, layer_name)
    if color is not None:
        set_object_color(oid, color)
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
    set_objects_layer(baked_ids, _JOINT_OPTIMIZATION_FRAMES_LAYER)
    group_objects(baked_ids)
    return baked_ids


def _bake_debug_ocf_frames(result, le_id, ln_id, prefix="RSJointPlace"):
    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)
    baked_ids = []
    baked_ids.extend(_bake_frame_axes(make_bar_frame(le_start, le_end), f"{prefix}_Le"))
    baked_ids.extend(_bake_frame_axes(make_bar_frame(ln_start, ln_end), f"{prefix}_Ln"))
    baked_ids.extend(_bake_frame_axes(result["female_frame"], f"{prefix}_Female"))
    baked_ids.extend(_bake_frame_axes(result["male_frame"], f"{prefix}_Male"))
    _remember_debug_frame_ids(baked_ids)


# ---------------------------------------------------------------------------
# Variant enumeration
# ---------------------------------------------------------------------------
# The 4 assembly variants come from reversing the bar endpoint order.
# Swapping start/end of a bar reverses its frame Z-axis, which reorients
# the connector along the bar.  Each combination is a full 4-DOF re-opt.
# ---------------------------------------------------------------------------

def _enumerate_solution_variants(le_start, le_end, ln_start, ln_end):
    """Run 4 full optimizations with every combination of bar-endpoint ordering."""
    combos = [
        ("1: le=fwd, ln=fwd", le_start, le_end, ln_start, ln_end, False, False),
        ("2: le=rev, ln=fwd", le_end, le_start, ln_start, ln_end, True,  False),
        ("3: le=fwd, ln=rev", le_start, le_end, ln_end, ln_start, False, True),
        ("4: le=rev, ln=rev", le_end, le_start, ln_end, ln_start, True,  True),
    ]
    variants = []
    for idx, (label, les, lee, lns, lne, le_rev, ln_rev) in enumerate(combos):
        res = optimize_joint_placement(les, lee, lns, lne, config, return_debug=True)
        diag = screw_hole_alignment_diagnostics(
            res["female_screw_hole_frame"], res["male_screw_hole_frame"],
        )
        res.update({
            "variant_index": idx,
            "variant_label": label,
            "variant_solver": "full_joint_optimization",
            "female_flip_rad": float(np.pi) if le_rev else 0.0,
            "male_flip_rad": float(np.pi) if ln_rev else 0.0,
            "origin_error_mm": diag["origin_error_mm"],
            "z_axis_error_rad": diag["z_axis_error_rad"],
            # Store the endpoint ordering used so _place can reproduce if needed
            "_le_start": les,
            "_le_end": lee,
            "_ln_start": lns,
            "_ln_end": lne,
        })
        variants.append(res)
    return variants


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
    return as_object_id_list([female_id, male_id])


def _interactive_variant_loop(variants, female_block_name, male_block_name):
    """Cycle through variants with Next/Previous/Accept. Returns chosen variant or None."""
    current = 0
    total = len(variants)
    preview_ids = []

    try:
        # Show first variant
        with suspend_redraw():
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
                    with suspend_redraw():
                        delete_objects(preview_ids)
                        current = (current + 1) % total
                        preview_ids = _show_variant_preview(variants[current], female_block_name, male_block_name)
                    _print_variant_info(variants[current], current, total)
                    continue

                if opt_idx == prev_idx:
                    with suspend_redraw():
                        delete_objects(preview_ids)
                        current = (current - 1) % total
                        preview_ids = _show_variant_preview(variants[current], female_block_name, male_block_name)
                    _print_variant_info(variants[current], current, total)
                    continue
    finally:
        delete_objects(preview_ids)


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

def _place_joint_blocks(result, le_id, ln_id, le_bar_id, ln_bar_id):
    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_err, z_err = _interface_metrics_from_result(result)

    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)

    le_num = le_bar_id.lstrip("B")
    ln_num = ln_bar_id.lstrip("B")
    joint_id = f"J{le_num}-{ln_num}"

    female_type, female_subtype = female_block_name.split("_", 1)
    male_type, male_subtype = male_block_name.split("_", 1)

    # Orientation: P if block x-axis points toward bar end, N toward start
    le_start, le_end = curve_endpoints(le_id)
    le_dir = np.array(le_end) - np.array(le_start)
    le_dir /= np.linalg.norm(le_dir)
    ln_start, ln_end = curve_endpoints(ln_id)
    ln_dir = np.array(ln_end) - np.array(ln_start)
    ln_dir /= np.linalg.norm(ln_dir)

    female_ori = "P" if float(np.dot(female_frame[:3, 0], le_dir)) > 0 else "N"
    male_ori = "P" if float(np.dot(male_frame[:3, 0], ln_dir)) > 0 else "N"

    with suspend_redraw():
        female_id = _insert_block_instance(female_block_name, female_frame, layer_name=_FEMALE_INSTANCES_LAYER)
        male_id = _insert_block_instance(male_block_name, male_frame, layer_name=_MALE_INSTANCES_LAYER)
        for obj_id, block_type, block_subtype, parent_bar, conn_bar, pos, rot_rad, ori in [
            (female_id, female_type, female_subtype, le_bar_id, ln_bar_id,
             result["fjp"], result["fjr"], female_ori),
            (male_id, male_type, male_subtype, ln_bar_id, le_bar_id,
             result["mjp"], result["mjr"], male_ori),
        ]:
            rs.SetUserText(obj_id, "joint_id", joint_id)
            rs.SetUserText(obj_id, "joint_type", block_type)
            rs.SetUserText(obj_id, "joint_subtype", block_subtype)
            rs.SetUserText(obj_id, "parent_bar_id", parent_bar)
            rs.SetUserText(obj_id, "connected_bar_id", conn_bar)
            rs.SetUserText(obj_id, "female_parent_bar", le_bar_id)
            rs.SetUserText(obj_id, "male_parent_bar", ln_bar_id)
            rs.SetUserText(obj_id, "position_mm", f"{float(pos):.4f}")
            rs.SetUserText(obj_id, "rotation_deg", f"{float(np.degrees(rot_rad)):.4f}")
            rs.SetUserText(obj_id, "ori", ori)
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

    le_bar_id = ensure_bar_id(le_id)
    ln_bar_id = ensure_bar_id(ln_id)

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)

    # Solve all 4 endpoint-reversal variants
    variants = _enumerate_solution_variants(le_start, le_end, ln_start, ln_end)

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

    _place_joint_blocks(chosen, le_id, ln_id, le_bar_id, ln_bar_id)


if __name__ == "__main__":
    main()
