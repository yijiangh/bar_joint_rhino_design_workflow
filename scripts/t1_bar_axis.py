"""Rhino entry point for T1 bar-axis generation."""

import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.config import BAR_CONTACT_DISTANCE, DEFAULT_NEW_BAR_LENGTH
from core.geometry import closest_params_infinite_lines, solve_s2_t1_all


_S2_PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]

_REFERENCE_SEGMENT_PRINT_WIDTH = 0.8


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _bake_reference_point(point, label, color):
    point_xyz = _point_to_array(point)
    point_id = rs.AddPoint(point_xyz.tolist())
    if point_id is None:
        return None
    rs.ObjectName(point_id, label)
    rs.ObjectColor(point_id, color)
    rs.SetUserText(point_id, "reference_label", label)
    return point_id


def _bake_reference_segment(start_point, end_point, label, color):
    start_xyz = _point_to_array(start_point)
    end_xyz = _point_to_array(end_point)
    line_id = rs.AddLine(start_xyz.tolist(), end_xyz.tolist())
    if line_id is None:
        return None
    rs.ObjectName(line_id, label)
    rs.ObjectColor(line_id, color)
    rs.SetUserText(line_id, "reference_label", label)
    if hasattr(rs, "ObjectPrintWidthSource"):
        rs.ObjectPrintWidthSource(line_id, 1)
    if hasattr(rs, "ObjectPrintWidth"):
        rs.ObjectPrintWidth(line_id, _REFERENCE_SEGMENT_PRINT_WIDTH)
    return line_id


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


def _add_centered_line(midpoint: np.ndarray, direction: np.ndarray):
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    half_length = DEFAULT_NEW_BAR_LENGTH / 2.0
    return rs.AddLine(midpoint - half_length * direction, midpoint + half_length * direction)


def get_inputs():
    rs.UnselectAllObjects()
    mode = rs.ListBox(
        ["S1: Join to ONE bar", "S2: Join to TWO bars"],
        "Select connection mode:",
        "T1 - Bar Axis Generation",
    )
    if mode is None:
        return None

    if "ONE" in mode:
        le_id = rs.GetObject("Select existing bar (Le)", rs.filter.curve)
        if le_id is None:
            return None
        ln_id = rs.GetObject("Select new bar (Ln) - will be repositioned", rs.filter.curve)
        if ln_id is None:
            return None
        return {"mode": "S1", "le_id": le_id, "ln_id": ln_id}

    le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
    if le1_id is None:
        return None
    le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
    if le2_id is None:
        return None
    ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on Le1 (Ce1)")
    if ce1 is None:
        return None
    ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on Le2 (Ce2)")
    if ce2 is None:
        return None
    ce1_ref_id = _bake_reference_point(ce1, "T1_S2_Ce1", (230, 80, 80))
    ce2_ref_id = _bake_reference_point(ce2, "T1_S2_Ce2", (80, 80, 230))
    return {
        "mode": "S2",
        "le1_id": le1_id,
        "le2_id": le2_id,
        "ce1": ce1,
        "ce2": ce2,
        "ce1_ref_id": ce1_ref_id,
        "ce2_ref_id": ce2_ref_id,
    }


def run_s1_t1(inputs):
    le_start, le_end = _curve_endpoints(inputs["le_id"])
    ln_start, ln_end = _curve_endpoints(inputs["ln_id"])
    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start

    t_e, t_n = closest_params_infinite_lines(le_start, le_direction, ln_start, ln_direction)
    p_e = le_start + t_e * le_direction
    p_n = ln_start + t_n * ln_direction

    segment = p_e - p_n
    current_distance = np.linalg.norm(segment)
    if current_distance <= 1e-9:
        rs.MessageBox("Error: the selected bars are coincident, so no unique contact normal exists.")
        return None

    translation = (segment / current_distance) * (current_distance - BAR_CONTACT_DISTANCE)
    shortest_segment_start = p_n + translation
    shortest_segment_end = p_e
    # Preserve the selected bar's original length and object attributes by
    # translating it in place instead of recreating a default-length line.
    line_id = rs.MoveObject(inputs["ln_id"], translation.tolist())
    if line_id is None:
        rs.MessageBox("Error: failed to move the selected new bar.")
        return None

    _bake_reference_segment(
        shortest_segment_start,
        shortest_segment_end,
        "T1_S1_shortest_segment",
        (180, 0, 180),
    )
    inputs["ln_id"] = line_id
    rs.SelectObject(line_id)
    print(f"S1-T1: Bar placed at distance {BAR_CONTACT_DISTANCE:.2f} from Le")
    return line_id


def run_s2_t1(inputs):
    le1_start, le1_end = _curve_endpoints(inputs["le1_id"])
    le2_start, le2_end = _curve_endpoints(inputs["le2_id"])
    ce1 = _point_to_array(inputs["ce1"])
    ce2 = _point_to_array(inputs["ce2"])

    n1 = le1_end - le1_start
    n2 = le2_end - le2_start
    try:
        solutions = solve_s2_t1_all(
            n1,
            ce1,
            n2,
            ce2,
            BAR_CONTACT_DISTANCE,
            nn_init_hint=ce2 - ce1,
        )
    except Exception as exc:
        rs.MessageBox(f"Error while solving S2-T1: {exc}")
        return None

    if not solutions:
        rs.MessageBox("No valid S2-T1 solution found for any sign combination.")
        return None

    if len(solutions) == 1:
        solution = solutions[0]
        midpoint = 0.5 * (solution["p1"] + solution["p2"])
        line_id = _add_centered_line(midpoint, solution["nn"])
        _bake_reference_segment(solution["p1"], ce1, "T1_S2_shortest_segment_1", (230, 80, 80))
        _bake_reference_segment(solution["p2"], ce2, "T1_S2_shortest_segment_2", (80, 80, 230))
        rs.SelectObject(line_id)
        print(f"S2-T1: Single solution, bar placed. Residual: {solution['residual']:.2e}")
        return line_id

    half_length = DEFAULT_NEW_BAR_LENGTH / 2.0
    preview_items = []
    for index, solution in enumerate(solutions):
        midpoint = 0.5 * (solution["p1"] + solution["p2"])
        direction = solution["nn"] / np.linalg.norm(solution["nn"])
        pt_a = midpoint - half_length * direction
        pt_b = midpoint + half_length * direction

        line_id = rs.AddLine(pt_a, pt_b)
        color = _S2_PREVIEW_COLORS[index % len(_S2_PREVIEW_COLORS)]
        rs.ObjectColor(line_id, color)
        segment_1_id = _bake_reference_segment(
            solution["p1"],
            ce1,
            f"T1_S2_preview_{index + 1}_shortest_segment_1",
            color,
        )
        segment_2_id = _bake_reference_segment(
            solution["p2"],
            ce2,
            f"T1_S2_preview_{index + 1}_shortest_segment_2",
            color,
        )

        dot_id = rs.AddTextDot(f"{index + 1}", midpoint)
        rs.ObjectColor(dot_id, color)
        preview_items.append((index, line_id, dot_id, segment_1_id, segment_2_id))

    rs.Redraw()
    picked_id = rs.GetObject(
        f"Click one of the {len(solutions)} candidate bars (numbered 1-{len(solutions)}):",
        rs.filter.curve,
    )

    chosen_index = None
    for index, line_id, dot_id, segment_1_id, segment_2_id in preview_items:
        if line_id == picked_id:
            chosen_index = index
        if rs.IsObject(dot_id):
            rs.DeleteObject(dot_id)
        if rs.IsObject(line_id):
            rs.DeleteObject(line_id)
        if rs.IsObject(segment_1_id):
            rs.DeleteObject(segment_1_id)
        if rs.IsObject(segment_2_id):
            rs.DeleteObject(segment_2_id)

    if chosen_index is None:
        print("S2-T1: No solution selected.")
        return None

    solution = solutions[chosen_index]
    midpoint = 0.5 * (solution["p1"] + solution["p2"])
    line_id = _add_centered_line(midpoint, solution["nn"])
    _bake_reference_segment(solution["p1"], ce1, "T1_S2_shortest_segment_1", (230, 80, 80))
    _bake_reference_segment(solution["p2"], ce2, "T1_S2_shortest_segment_2", (80, 80, 230))
    rs.SelectObject(line_id)
    sign_1, sign_2 = solution["signs"]
    print(
        f"S2-T1: Solution {chosen_index + 1} selected. "
        f"Signs=({sign_1:+.0f},{sign_2:+.0f}), "
        f"Residual: {solution['residual']:.2e}"
    )
    return line_id


def main(rerun=False):
    if rerun and "t1_inputs" in sc.sticky:
        inputs = sc.sticky["t1_inputs"]
    else:
        inputs = get_inputs()
        if inputs is None:
            return
        sc.sticky["t1_inputs"] = inputs

    if inputs["mode"] == "S1":
        run_s1_t1(inputs)
    elif inputs["mode"] == "S2":
        run_s2_t1(inputs)


if __name__ == "__main__":
    main(rerun=False)
