"""Rhino entry point for T2 joint placement."""

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


_DEBUG_FRAME_AXIS_LENGTH = 35.0
_DEBUG_FRAME_IDS_KEY = "t2_debug_frame_ids"
_FEMALE_BLOCK_NAME = "FemaleLinkBlock"
_MALE_BLOCK_NAME = "MaleLinkBlock"
_FEMALE_INSTANCES_LAYER = "FemaleJointPlacedInstances"
_MALE_INSTANCES_LAYER = "MaleJointPlacedInstances"
_JOINT_OPTIMIZATION_FRAMES_LAYER = "JointOptimizationFrames"


def _reload_runtime_modules():
    global config, make_bar_frame, optimize_joint_placement, screw_hole_alignment_diagnostics

    config = importlib.reload(_config_module)
    kinematics = importlib.reload(_kinematics_module)
    make_bar_frame = kinematics.make_bar_frame
    optimize_joint_placement = kinematics.optimize_joint_placement
    screw_hole_alignment_diagnostics = kinematics.screw_hole_alignment_diagnostics


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
    return optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)


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


def place_joint_blocks(result, inputs):
    """Place Rhino block instances at the optimized female and male link frames."""

    import Rhino

    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_error_mm, z_axis_error_rad = _interface_metrics_from_result(result)

    def numpy_to_rhino_transform(matrix):
        xform = Rhino.Geometry.Transform(1.0)
        for row in range(4):
            for col in range(4):
                xform[row, col] = float(matrix[row, col])
        return xform

    female_block_name = _require_block_definition(_FEMALE_BLOCK_NAME)
    male_block_name = _require_block_definition(_MALE_BLOCK_NAME)

    female_id = rs.InsertBlock(female_block_name, [0, 0, 0])
    if female_id is None:
        raise RuntimeError(f"Failed to insert Rhino block '{female_block_name}'.")
    rs.TransformObject(female_id, numpy_to_rhino_transform(female_frame))
    _set_objects_layer(female_id, _FEMALE_INSTANCES_LAYER)

    male_id = rs.InsertBlock(male_block_name, [0, 0, 0])
    if male_id is None:
        raise RuntimeError(f"Failed to insert Rhino block '{male_block_name}'.")
    rs.TransformObject(male_id, numpy_to_rhino_transform(male_frame))
    _set_objects_layer(male_id, _MALE_INSTANCES_LAYER)

    joint_data_female = {
        "type": female_block_name,
        "dof": {"fjp": result["fjp"], "fjr": result["fjr"]},
        "bar_id": str(inputs["le_id"]),
        "connected_bar_id": str(inputs["ln_id"]),
        "transform": female_frame.tolist(),
        "residual": result["residual"],
        "origin_error_mm": origin_error_mm,
        "z_axis_error_rad": z_axis_error_rad,
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
    }

    rs.SetUserText(female_id, "joint_data", json.dumps(joint_data_female, indent=2))
    rs.SetUserText(male_id, "joint_data", json.dumps(joint_data_male, indent=2))
    _bake_debug_ocf_frames(result, inputs)

    print(f"T2: Joints placed. Residual: {result['residual']:.6f}")
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg"
    )
    print(
        f"  Interface origin err={origin_error_mm:.4f} mm, "
        f"z-axis err={np.degrees(z_axis_error_rad):.4f}deg"
    )
    print(f"  Inserted blocks: {female_block_name}, {male_block_name}")
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
        result = run_s1_t2(inputs)
        place_joint_blocks(result, {"le_id": inputs["le_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S1"})
        return

    result1 = run_s1_t2({"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
    result2 = run_s1_t2({"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})
    place_joint_blocks(result1, {"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S2_1"})
    place_joint_blocks(result2, {"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"], "debug_prefix": "T2_S2_2"})


if __name__ == "__main__":
    main(rerun=False)
