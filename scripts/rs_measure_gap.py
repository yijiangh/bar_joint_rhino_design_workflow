#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""Rhino entry point for drawing the closest segment between two finite lines."""

import os
import sys

import numpy as np
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.geometry import closest_params_finite_segments


_CLOSEST_SEGMENT_COLOR = (0, 0, 255)
_ZERO_LENGTH_TOL = 1e-9


def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


def _get_line_segment(prompt, exclude_id=None):
    while True:
        curve_id = rs.GetObject(prompt, rs.filter.curve)
        if curve_id is None:
            return None
        if exclude_id is not None and curve_id == exclude_id:
            rs.MessageBox("Please select a different line segment.")
            continue
        if not rs.IsLine(curve_id):
            rs.MessageBox("Please select a Rhino line object (finite line segment).")
            continue
        return curve_id


def _closest_points_on_segments(line_1_id, line_2_id):
    line_1_start, line_1_end = _curve_endpoints(line_1_id)
    line_2_start, line_2_end = _curve_endpoints(line_2_id)

    t1, t2 = closest_params_finite_segments(
        line_1_start,
        line_1_end,
        line_2_start,
        line_2_end,
    )
    point_1 = line_1_start + t1 * (line_1_end - line_1_start)
    point_2 = line_2_start + t2 * (line_2_end - line_2_start)
    return point_1, point_2


def main():
    rs.UnselectAllObjects()

    line_1_id = _get_line_segment("Select first line segment")
    if line_1_id is None:
        return None

    line_2_id = _get_line_segment("Select second line segment", exclude_id=line_1_id)
    if line_2_id is None:
        return None

    try:
        point_1, point_2 = _closest_points_on_segments(line_1_id, line_2_id)
    except ValueError as exc:
        rs.MessageBox(f"Error: {exc}")
        return None

    distance = float(np.linalg.norm(point_2 - point_1))
    if distance <= _ZERO_LENGTH_TOL:
        point_id = rs.AddPoint(point_1.tolist())
        if point_id is not None:
            rs.ObjectColor(point_id, _CLOSEST_SEGMENT_COLOR)
            rs.ObjectName(point_id, "ClosestLineSegmentPoint")
            rs.SetUserText(point_id, "distance", "0.0")
            rs.SelectObject(point_id)
        print("The selected line segments intersect. The closest segment has zero length.")
        return point_id

    segment_id = rs.AddLine(point_1.tolist(), point_2.tolist())
    if segment_id is None:
        rs.MessageBox("Error: failed to draw the closest segment.")
        return None

    rs.ObjectColor(segment_id, _CLOSEST_SEGMENT_COLOR)
    rs.ObjectName(segment_id, "ClosestLineSegment")
    rs.SetUserText(segment_id, "distance", f"{distance:.12g}")
    rs.SetUserText(segment_id, "source_line_1", str(line_1_id))
    rs.SetUserText(segment_id, "source_line_2", str(line_2_id))
    rs.SelectObject(segment_id)

    print(f"Closest segment created. Distance: {distance:.6f}")
    return segment_id


if __name__ == "__main__":
    main()
