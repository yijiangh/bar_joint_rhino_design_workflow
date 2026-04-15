"""Rhino entry point for T2 joint placement."""

import contextlib
import importlib
import json
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import kinematics as _kinematics_module
from core import transforms as _transforms_module


_DEBUG_FRAME_AXIS_LENGTH = 35.0
_DEBUG_FRAME_IDS_KEY = "t2_debug_frame_ids"
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
_PREVIEW_GRID_SPACING = 140.0


def _reload_runtime_modules():
    global config, invert_transform, make_bar_frame, optimize_joint_placement, rotation_about_local_z
    global optimize_female_to_target, screw_hole_alignment_diagnostics, screw_hole_origin_z_error

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
        return [object_id for object_id in object_ids if object_id is not None]
    except TypeError:
        return [object_ids]


def _ensure_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def _set_objects_layer(object_ids, layer_name):
    baked_ids = _as_object_id_list(object_ids)
    _ensure_layer(layer_name)
    for object_id in baked_ids:
        if rs.IsObject(object_id):
            rs.ObjectLayer(object_id, layer_name)
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
    for object_id in _as_object_id_list(object_ids):
        if rs.IsObject(object_id):
            rs.DeleteObject(object_id)


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


def _clear_debug_frames():
    for object_id in sc.sticky.pop(_DEBUG_FRAME_IDS_KEY, []):
        if rs.IsObject(object_id):
            rs.DeleteObject(object_id)


def _remember_debug_frame_ids(object_ids):
    baked_ids = sc.sticky.get(_DEBUG_FRAME_IDS_KEY, [])
    baked_ids.extend(object_id for object_id in object_ids if object_id is not None)
    sc.sticky[_DEBUG_FRAME_IDS_KEY] = baked_ids


def _bake_frame_axes(frame, label, axis_length=_DEBUG_FRAME_AXIS_LENGTH):
    frame = np.asarray(frame, dtype=float)
    origin = frame[:3, 3]
    axis_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    axis_names = ["x", "y", "z"]
    baked_ids = []

    for axis_index, (axis_name, color) in enumerate(zip(axis_names, axis_colors)):
        axis_vector = np.asarray(frame[:3, axis_index], dtype=float)
        axis_norm = float(np.linalg.norm(axis_vector))
        if axis_norm <= 1e-12:
            continue
        line_id = rs.AddLine(origin.tolist(), (origin + axis_length * axis_vector / axis_norm).tolist())
        if line_id is None:
            continue
        rs.ObjectColor(line_id, color)
        rs.ObjectName(line_id, f"{label}_{axis_name.upper()}")
        rs.SetUserText(line_id, "frame_label", label)
        rs.SetUserText(line_id, "axis_name", axis_name)
        baked_ids.append(line_id)

    dot_id = rs.AddTextDot(label, origin.tolist())
    if dot_id is not None:
        rs.ObjectName(dot_id, f"{label}_label")
        rs.SetUserText(dot_id, "frame_label", label)
        baked_ids.append(dot_id)
    _set_objects_layer(baked_ids, _JOINT_OPTIMIZATION_FRAMES_LAYER)
    _group_objects(baked_ids)
    return baked_ids


def _bake_debug_ocf_frames(result, inputs):
    le_start, le_end = _curve_endpoints(inputs["le_id"])
    ln_start, ln_end = _curve_endpoints(inputs["ln_id"])
    prefix = inputs.get("debug_prefix", "T2")
    baked_ids = []
    baked_ids.extend(_bake_frame_axes(make_bar_frame(le_start, le_end), f"{prefix}_Le"))
    baked_ids.extend(_bake_frame_axes(make_bar_frame(ln_start, ln_end), f"{prefix}_Ln"))
    baked_ids.extend(_bake_frame_axes(result["female_frame"], f"{prefix}_Female"))
    baked_ids.extend(_bake_frame_axes(result["male_frame"], f"{prefix}_Male"))
    _remember_debug_frame_ids(baked_ids)


def _has_block_definition(name):
    for instance_def in sc.doc.InstanceDefinitions:
        if instance_def is not None and not instance_def.IsDeleted and instance_def.Name == name:
            return True
    return False


def _require_block_definition(name):
    if not _has_block_definition(name):
        raise RuntimeError(f"Missing required Rhino block definition '{name}'.")
    return name


def _set_object_color(object_ids, color):
    for object_id in _as_object_id_list(object_ids):
        if not rs.IsObject(object_id):
            continue
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(object_id, 1)
        rs.ObjectColor(object_id, color)


def _candidate_pick_filter():
    filter_namespace = getattr(rs, "filter", None)
    if filter_namespace is None:
        return 0

    pick_filter = 0
    for filter_name in ("instance", "annotation", "curve", "surface", "polysurface", "mesh"):
        pick_filter |= getattr(filter_namespace, filter_name, 0)
    return pick_filter


def _numpy_to_rhino_transform(matrix):
    import Rhino

    xform = Rhino.Geometry.Transform(1.0)
    for row in range(4):
        for col in range(4):
            xform[row, col] = float(matrix[row, col])
    return xform


def _translated_frame(frame, offset):
    translated = np.array(frame, dtype=float, copy=True)
    translated[:3, 3] = translated[:3, 3] + np.asarray(offset, dtype=float)
    return translated


def _apply_local_z_flip(frame, angle_rad):
    return np.asarray(frame, dtype=float) @ rotation_about_local_z(float(angle_rad))


def _build_variant_result(base_result, *, female_flip_rad, male_flip_rad, variant_index):
    female_frame = _apply_local_z_flip(base_result["female_frame"], female_flip_rad)
    male_frame = _apply_local_z_flip(base_result["male_frame"], male_flip_rad)
    female_screw_hole_frame = female_frame @ np.asarray(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM, dtype=float)
    male_screw_hole_frame = male_frame @ invert_transform(np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float))
    diagnostics = screw_hole_alignment_diagnostics(female_screw_hole_frame, male_screw_hole_frame)
    result = dict(base_result)
    result.update(
        {
            "female_frame": female_frame,
            "male_frame": male_frame,
            "female_screw_hole_frame": female_screw_hole_frame,
            "male_screw_hole_frame": male_screw_hole_frame,
            "residual": screw_hole_origin_z_error(
                female_screw_hole_frame,
                male_screw_hole_frame,
                orientation_weight_mm=float(getattr(config, "BAR_CONTACT_DISTANCE", 1.0)),
            ),
            "origin_error_mm": diagnostics["origin_error_mm"],
            "z_axis_error_rad": diagnostics["z_axis_error_rad"],
            "variant_index": int(variant_index),
            "female_flip_rad": float(female_flip_rad),
            "male_flip_rad": float(male_flip_rad),
        }
    )
    return result


def _solve_variant_after_male_flip(base_result, le_start, le_end):
    male_frame = _apply_local_z_flip(base_result["male_frame"], np.pi)
    male_screw_hole_frame = male_frame @ invert_transform(np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float))
    female_result = optimize_female_to_target(
        le_start,
        le_end,
        male_screw_hole_frame,
        config,
        initial_guess=(base_result["fjp"], base_result["fjr"]),
        return_debug=True,
    )
    result = dict(base_result)
    result.update(
        {
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
        }
    )
    return result


def _enumerate_solution_variants(base_result, le_start, le_end):
    variant_1 = dict(base_result)
    variant_1.update(
        {
            "variant_index": 0,
            "variant_label": "1: female=0deg, male=0deg",
            "variant_solver": "full_joint_optimization",
            "female_flip_rad": 0.0,
            "male_flip_rad": 0.0,
        }
    )

    variant_2 = _build_variant_result(
        base_result,
        female_flip_rad=np.pi,
        male_flip_rad=0.0,
        variant_index=1,
    )
    variant_2["variant_label"] = "2: female=180deg, male=0deg"
    variant_2["variant_solver"] = "full_joint_optimization"

    variant_3 = _solve_variant_after_male_flip(base_result, le_start, le_end)
    variant_3["variant_index"] = 2
    variant_3["variant_label"] = "3: female reopt, male=180deg"

    variant_4 = _build_variant_result(
        variant_3,
        female_flip_rad=np.pi,
        male_flip_rad=0.0,
        variant_index=3,
    )
    variant_4["variant_label"] = "4: female=180deg, male=180deg"
    variant_4["variant_solver"] = "female_reoptimized_after_male_flip"
    variant_4["male_flip_rad"] = float(np.pi)
    return [variant_1, variant_2, variant_3, variant_4]


def _preview_offsets(reference_frame):
    reference_frame = np.asarray(reference_frame, dtype=float)
    x_axis = reference_frame[:3, 0]
    y_axis = reference_frame[:3, 1]
    spacing = float(_PREVIEW_GRID_SPACING)
    factors = [(-0.75, -0.75), (0.75, -0.75), (-0.75, 0.75), (0.75, 0.75)]
    return [spacing * (x_factor * x_axis + y_factor * y_axis) for x_factor, y_factor in factors]


def _insert_block_instance(block_name, frame, *, layer_name=None, color=None):
    object_id = rs.InsertBlock(block_name, [0, 0, 0])
    if object_id is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(object_id, _numpy_to_rhino_transform(frame))
    if layer_name:
        _set_objects_layer(object_id, layer_name)
    if color is not None:
        _set_object_color(object_id, color)
    return object_id


def _choose_solution_variant(base_result, le_start, le_end, prompt_title):
    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)
    variants = _enumerate_solution_variants(base_result, le_start, le_end)
    offsets = _preview_offsets(base_result["female_frame"])
    preview_items = []

    print(f"{prompt_title}: previewing 4 assembly variants")
    for variant in variants:
        print(
            f"  {variant['variant_label']} | residual={variant['residual']:.6f}, "
            f"origin err={variant['origin_error_mm']:.4f} mm, "
            f"z err={np.degrees(variant['z_axis_error_rad']):.4f}deg"
        )

    with _suspend_redraw():
        for variant, offset, color in zip(variants, offsets, _PREVIEW_COLORS):
            preview_female_frame = _translated_frame(variant["female_frame"], offset)
            preview_male_frame = _translated_frame(variant["male_frame"], offset)
            female_id = _insert_block_instance(female_block_name, preview_female_frame, color=color)
            male_id = _insert_block_instance(male_block_name, preview_male_frame, color=color)
            label_position = 0.5 * (preview_female_frame[:3, 3] + preview_male_frame[:3, 3]) + 20.0 * preview_female_frame[:3, 2]
            dot_id = rs.AddTextDot(str(variant["variant_index"] + 1), label_position.tolist())
            if dot_id is not None:
                _set_object_color(dot_id, color)
            preview_items.append(
                {
                    "variant_index": int(variant["variant_index"]),
                    "pick_ids": set(_as_object_id_list([female_id, male_id, dot_id])),
                    "cleanup_ids": _as_object_id_list([female_id, male_id, dot_id]),
                }
            )

    picked_id = None
    try:
        picked_id = rs.GetObject(
            f"{prompt_title}: click one preview pair or number (1-4):",
            _candidate_pick_filter(),
        )
    finally:
        with _suspend_redraw():
            for preview_item in preview_items:
                _delete_objects(preview_item["cleanup_ids"])

    if picked_id is None:
        print(f"{prompt_title}: no variant selected.")
        return None

    for preview_item in preview_items:
        if picked_id in preview_item["pick_ids"]:
            chosen_variant = variants[preview_item["variant_index"]]
            print(f"{prompt_title}: selected {chosen_variant['variant_label']}")
            return chosen_variant

    print(f"{prompt_title}: selection did not match a preview candidate.")
    return None


def get_inputs_t2():
    mode = rs.ListBox(
        ["S1: ONE connection", "S2: TWO connections"],
        "Select connection mode:",
        "T2 - Joint Placement",
    )
    if mode is None:
        return None

    if "ONE" in mode:
        le_id = rs.GetObject("Select existing bar (Le) - gets FEMALE joint", rs.filter.curve)
        if le_id is None:
            return None
        ln_id = rs.GetObject("Select new bar (Ln) - gets MALE joint", rs.filter.curve)
        if ln_id is None:
            return None
        return {"mode": "S1", "le_id": le_id, "ln_id": ln_id}

    ln_id = rs.GetObject("Select new bar (Ln)", rs.filter.curve)
    if ln_id is None:
        return None
    le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
    if le1_id is None:
        return None
    le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
    if le2_id is None:
        return None
    return {"mode": "S2", "ln_id": ln_id, "le1_id": le1_id, "le2_id": le2_id}


def run_s1_t2(inputs):
    le_start, le_end = _curve_endpoints(inputs["le_id"])
    ln_start, ln_end = _curve_endpoints(inputs["ln_id"])
    return optimize_joint_placement(le_start, le_end, ln_start, ln_end, config, return_debug=True)


def _interface_metrics_from_result(result):
    if "origin_error_mm" in result and "z_axis_error_rad" in result:
        return float(result["origin_error_mm"]), float(result["z_axis_error_rad"])

    if "female_screw_hole_frame" in result and "male_screw_hole_frame" in result:
        diagnostics = screw_hole_alignment_diagnostics(
            result["female_screw_hole_frame"],
            result["male_screw_hole_frame"],
        )
        return float(diagnostics["origin_error_mm"]), float(diagnostics["z_axis_error_rad"])

    raise KeyError(
        "Result is missing interface diagnostics and screw-hole frames; "
        "rerun after reloading the updated core.kinematics module."
    )


def _print_optimizer_debug(result):
    optimizer_debug = result.get("optimizer_debug")
    if not optimizer_debug:
        print("  Optimizer debug: unavailable")
        return

    seed_reports = list(optimizer_debug.get("seed_reports", []))
    best_report = dict(optimizer_debug.get("best_seed_report", {}))
    if not best_report and seed_reports:
        best_report = dict(min(seed_reports, key=lambda report: float(report.get("residual", float("inf")))))

    residuals = [float(report["residual"]) for report in seed_reports if "residual" in report]
    if residuals:
        objective_min = min(residuals)
        objective_median = float(np.median(residuals))
        objective_max = max(residuals)
        print(
            "  Optimizer seeds: "
            f"{int(optimizer_debug.get('seed_count', len(seed_reports)))} tried, "
            f"best seed #{int(optimizer_debug.get('best_seed_index', -1))}"
        )
        print(
            "  Seed objective values: "
            f"best={objective_min:.6f}, median={objective_median:.6f}, worst={objective_max:.6f}"
        )
    else:
        print(
            "  Optimizer seeds: "
            f"{int(optimizer_debug.get('seed_count', 0))} tried, "
            f"best seed #{int(optimizer_debug.get('best_seed_index', -1))}"
        )

    if not best_report:
        return

    print(
        "  Best seed convergence: "
        f"nit={int(best_report.get('nit', -1))}, "
        f"nfev={int(best_report.get('nfev', -1))}, "
        f"objective={float(best_report.get('residual', float('nan'))):.6f}, "
        f"proj-grad-inf={float(best_report.get('proj_grad_inf_norm', float('nan'))):.3e}"
    )
    print(
        "  Best seed status: "
        f"success={bool(best_report.get('success', False))}, "
        f"status={int(best_report.get('status', -1))}, "
        f"message={best_report.get('message', 'n/a')}"
    )


def place_joint_blocks(result, inputs):
    """Place Rhino block instances at the optimized female and male link frames."""

    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_error_mm, z_axis_error_rad = _interface_metrics_from_result(result)

    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)

    joint_data_female = {
        "type": female_block_name,
        "dof": {"fjp": result["fjp"], "fjr": result["fjr"]},
        "bar_id": str(inputs["le_id"]),
        "connected_bar_id": str(inputs["ln_id"]),
        "transform": female_frame.tolist(),
        "residual": result["residual"],
        "origin_error_mm": origin_error_mm,
        "z_axis_error_rad": z_axis_error_rad,
        "variant_index": result.get("variant_index"),
        "female_flip_rad": result.get("female_flip_rad", 0.0),
        "male_flip_rad": result.get("male_flip_rad", 0.0),
        "variant_solver": result.get("variant_solver"),
    }
    joint_data_male = {
        "type": male_block_name,
        "dof": {"mjp": result["mjp"], "mjr": result["mjr"]},
        "bar_id": str(inputs["ln_id"]),
        "connected_bar_id": str(inputs["le_id"]),
        "transform": male_frame.tolist(),
        "residual": result["residual"],
        "origin_error_mm": origin_error_mm,
        "z_axis_error_rad": z_axis_error_rad,
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
        _bake_debug_ocf_frames(result, inputs)

    print(f"T2: Joints placed. Residual: {result['residual']:.6f}")
    if result.get("variant_index") is not None:
        print(
            f"  Variant {int(result['variant_index']) + 1}: "
            f"female flip={np.degrees(float(result.get('female_flip_rad', 0.0))):.1f}deg, "
            f"male flip={np.degrees(float(result.get('male_flip_rad', 0.0))):.1f}deg"
        )
    if result.get("variant_solver"):
        print(f"  Variant solver: {result['variant_solver']}")
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg"
    )
    print(
        f"  Interface origin err={origin_error_mm:.4f} mm, "
        f"z-axis err={np.degrees(z_axis_error_rad):.4f}deg"
    )
    print(f"  Inserted blocks: {female_block_name}, {male_block_name}")
    _print_optimizer_debug(result)
    return female_id, male_id


def main(rerun=False):
    _reload_runtime_modules()

    if rerun and "t2_inputs" in sc.sticky:
        inputs = sc.sticky["t2_inputs"]
    else:
        inputs = get_inputs_t2()
        if inputs is None:
            return
        sc.sticky["t2_inputs"] = inputs

    _clear_debug_frames()

    if inputs["mode"] == "S1":
        base_result = run_s1_t2(inputs)
        le_start, le_end = _curve_endpoints(inputs["le_id"])
        chosen_result = _choose_solution_variant(base_result, le_start, le_end, "T2-S1")
        if chosen_result is None:
            return
        place_joint_blocks(chosen_result, {"le_id": inputs["le_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S1"})
        return

    base_result_1 = run_s1_t2({"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
    le1_start, le1_end = _curve_endpoints(inputs["le1_id"])
    chosen_result_1 = _choose_solution_variant(base_result_1, le1_start, le1_end, "T2-S2 pair 1")
    if chosen_result_1 is None:
        return
    place_joint_blocks(chosen_result_1, {"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S2_1"})

    base_result_2 = run_s1_t2({"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})
    le2_start, le2_end = _curve_endpoints(inputs["le2_id"])
    chosen_result_2 = _choose_solution_variant(base_result_2, le2_start, le2_end, "T2-S2 pair 2")
    if chosen_result_2 is None:
        return
    place_joint_blocks(chosen_result_2, {"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S2_2"})


if __name__ == "__main__":
    main(rerun=False)
