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
from core.kinematics import optimize_joint_placement


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


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

    female_id = rs.InsertBlock("FemaleJoint", [0, 0, 0])
    rs.TransformObject(female_id, numpy_to_rhino_transform(female_frame))

    male_id = rs.InsertBlock("MaleJoint", [0, 0, 0])
    rs.TransformObject(male_id, numpy_to_rhino_transform(male_frame))

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

    if inputs["mode"] == "S1":
        result = run_s1_t2(inputs)
        place_joint_blocks(result, inputs)
        return

    result1 = run_s1_t2({"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
    result2 = run_s1_t2({"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})
    place_joint_blocks(result1, {"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
    place_joint_blocks(result2, {"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})


if __name__ == "__main__":
    main(rerun=False)
