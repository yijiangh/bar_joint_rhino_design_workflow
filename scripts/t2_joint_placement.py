"""Rhino entry point for T2 joint placement."""

import json
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.kinematics import make_bar_frame, optimize_joint_placement


_LEGACY_PLACEHOLDER_BOX_SIZE = np.array([20.0, 20.0, 20.0], dtype=float)
_PLACEHOLDER_TOL = 1e-6
_FEMALE_PLACEHOLDER_FOOTPRINT = np.array([46.0, 28.0], dtype=float)
_MALE_PLACEHOLDER_FOOTPRINT = np.array([40.0, 24.0], dtype=float)
_DEBUG_FRAME_AXIS_LENGTH = 35.0
_DEBUG_FRAME_IDS_KEY = "t2_debug_frame_ids"


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


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


def _legacy_placeholder_bbox(block_name):
    import Rhino

    block_def = None
    for instance_def in sc.doc.InstanceDefinitions:
        if instance_def is not None and not instance_def.IsDeleted and instance_def.Name == block_name:
            block_def = instance_def
            break
    if block_def is None:
        return None

    bbox = None
    for rhino_obj in block_def.GetObjects():
        geometry = getattr(rhino_obj, "Geometry", None)
        if geometry is None:
            continue
        geom_bbox = geometry.GetBoundingBox(True)
        if not geom_bbox.IsValid:
            continue
        if bbox is None:
            bbox = geom_bbox
        else:
            bbox.Union(geom_bbox)

    if bbox is None:
        return None

    size = np.array([bbox.Max.X - bbox.Min.X, bbox.Max.Y - bbox.Min.Y, bbox.Max.Z - bbox.Min.Z], dtype=float)
    center = np.array(
        [
            0.5 * (bbox.Min.X + bbox.Max.X),
            0.5 * (bbox.Min.Y + bbox.Max.Y),
            0.5 * (bbox.Min.Z + bbox.Max.Z),
        ],
        dtype=float,
    )
    if not np.allclose(size, _LEGACY_PLACEHOLDER_BOX_SIZE, atol=_PLACEHOLDER_TOL):
        return None
    if np.linalg.norm(center) > _PLACEHOLDER_TOL:
        return None
    return {"size": size, "center": center}


def _placeholder_visual_frame(frame, radial_offset, block_name, radial_sign, footprint_xy):
    bbox = _legacy_placeholder_bbox(block_name)
    if bbox is None:
        return np.asarray(frame, dtype=float)

    depth = float(bbox["size"][2])
    radial_offset = float(radial_offset)
    if depth <= _PLACEHOLDER_TOL or radial_offset <= _PLACEHOLDER_TOL:
        return np.asarray(frame, dtype=float)

    corrected = np.array(frame, dtype=float, copy=True)
    footprint_xy = np.asarray(footprint_xy, dtype=float)
    corrected[:3, 0] = (footprint_xy[0] / float(bbox["size"][0])) * np.asarray(frame[:3, 0], dtype=float)
    corrected[:3, 1] = (footprint_xy[1] / float(bbox["size"][1])) * np.asarray(frame[:3, 1], dtype=float)
    z_axis = np.asarray(frame[:3, 2], dtype=float)
    corrected[:3, 2] = (radial_offset / depth) * z_axis
    corrected[:3, 3] = np.asarray(frame[:3, 3], dtype=float) + 0.5 * float(radial_sign) * radial_offset * z_axis
    return corrected


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


def place_joint_blocks(result, inputs):
    """Place placeholder or existing joint blocks at the optimized transforms."""

    import Rhino

    female_frame = result["female_frame"]
    male_frame = result["male_frame"]

    def numpy_to_rhino_transform(matrix):
        xform = Rhino.Geometry.Transform(1.0)
        for row in range(4):
            for col in range(4):
                xform[row, col] = float(matrix[row, col])
        return xform

    def ensure_block_def(name, color):
        if rs.IsBlock(name):
            return
        box_id = rs.AddBox(
            [
                [-10, -10, -10],
                [10, -10, -10],
                [10, 10, -10],
                [-10, 10, -10],
                [-10, -10, 10],
                [10, -10, 10],
                [10, 10, 10],
                [-10, 10, 10],
            ]
        )
        rs.ObjectColor(box_id, color)
        rs.AddBlock([box_id], [0, 0, 0], name, delete_input=True)

    ensure_block_def("FemaleJoint", (255, 100, 100))
    ensure_block_def("MaleJoint", (100, 255, 100))

    female_visual_frame = _placeholder_visual_frame(
        female_frame,
        config.FEMALE_RADIAL_OFFSET,
        "FemaleJoint",
        -1.0,
        _FEMALE_PLACEHOLDER_FOOTPRINT,
    )
    male_visual_frame = _placeholder_visual_frame(
        male_frame,
        config.MALE_RADIAL_OFFSET,
        "MaleJoint",
        1.0,
        _MALE_PLACEHOLDER_FOOTPRINT,
    )

    female_id = rs.InsertBlock("FemaleJoint", [0, 0, 0])
    rs.TransformObject(female_id, numpy_to_rhino_transform(female_visual_frame))

    male_id = rs.InsertBlock("MaleJoint", [0, 0, 0])
    rs.TransformObject(male_id, numpy_to_rhino_transform(male_visual_frame))

    joint_data_female = {
        "type": "FemaleJoint",
        "dof": {"fjp": result["fjp"], "fjr": result["fjr"], "jjr": result["jjr"]},
        "bar_id": str(inputs["le_id"]),
        "connected_bar_id": str(inputs["ln_id"]),
        "transform": female_frame.tolist(),
        "residual": result["residual"],
    }
    joint_data_male = {
        "type": "MaleJoint",
        "dof": {"mjp": result["mjp"], "mjr": result["mjr"]},
        "bar_id": str(inputs["ln_id"]),
        "connected_bar_id": str(inputs["le_id"]),
        "transform": male_frame.tolist(),
        "residual": result["residual"],
    }

    rs.SetUserText(female_id, "joint_data", json.dumps(joint_data_female, indent=2))
    rs.SetUserText(male_id, "joint_data", json.dumps(joint_data_male, indent=2))
    _bake_debug_ocf_frames(result, inputs)

    print(f"T2: Joints placed. Residual: {result['residual']:.6f}")
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg, "
        f"JJR={np.degrees(result['jjr']):.1f}deg"
    )
    return female_id, male_id


def main(rerun=False):
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
