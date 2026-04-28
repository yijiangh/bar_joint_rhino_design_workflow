"""Shared helpers for reading baked Rhino frame groups (axes + text dot label).

A baked frame group (authored by `rs_bake_frame.py`) contains three line
objects named `frame_x_axis`, `frame_y_axis`, `frame_z_axis` that share a
common origin, plus an optional text-dot label. These helpers resolve such
groups from interactive picks and reconstruct them as 4x4 transforms in
millimeters.
"""

from __future__ import annotations

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core.transforms import frame_from_axes, orthonormalize_rotation


AXIS_OBJECT_NAMES = ("frame_x_axis", "frame_y_axis", "frame_z_axis")
_LINE_TOL = 1e-6


def doc_unit_scale_to_mm() -> float:
    return float(Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters))


def _point_to_mm(point, scale_to_mm: float) -> np.ndarray:
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale_to_mm
    return np.asarray(point, dtype=float) * scale_to_mm


def _line_endpoints_mm(object_id, scale_to_mm: float):
    curve = rs.coercecurve(object_id)
    if curve is None:
        raise ValueError("Expected a line object in the baked frame group.")
    start = _point_to_mm(curve.PointAtStart, scale_to_mm)
    end = _point_to_mm(curve.PointAtEnd, scale_to_mm)
    return start, end


def _same_point(point_a: np.ndarray, point_b: np.ndarray, *, tol: float = _LINE_TOL) -> bool:
    return float(np.linalg.norm(point_a - point_b)) <= tol


def group_members_from_selection(object_id):
    groups = rs.ObjectGroups(object_id) or []
    if not groups:
        raise ValueError("Selected object is not part of a baked frame group.")
    if len(groups) > 1:
        raise ValueError(
            "Selected object belongs to multiple groups; please select a single baked frame group."
        )
    group_name = groups[0]
    members = rs.ObjectsByGroup(group_name) or []
    if not members:
        raise ValueError("Selected frame group is empty.")
    return group_name, members


def resolve_frame_group(prompt: str):
    object_id = rs.GetObject(prompt)
    if object_id is None:
        return None
    return group_members_from_selection(object_id)


def frame_label_from_group(group_members) -> str:
    for object_id in group_members:
        if rs.IsTextDot(object_id):
            text = rs.TextDotText(object_id)
            if text:
                return text
        name = rs.ObjectName(object_id)
        if name and name not in AXIS_OBJECT_NAMES:
            return name
    return ""


def _extract_axis_lines(group_members):
    axis_lines = {}
    for object_id in group_members:
        name = rs.ObjectName(object_id)
        if name in AXIS_OBJECT_NAMES:
            axis_lines[name] = object_id
    missing = [name for name in AXIS_OBJECT_NAMES if name not in axis_lines]
    if missing:
        raise ValueError(f"Frame group is missing baked axes: {', '.join(missing)}")
    return axis_lines


def _shared_origin_mm(axis_lines, scale_to_mm: float):
    endpoints = {name: _line_endpoints_mm(oid, scale_to_mm) for name, oid in axis_lines.items()}
    candidates = []
    for start, end in endpoints.values():
        candidates.extend([start, end])
    for candidate in candidates:
        if all(_same_point(candidate, s) or _same_point(candidate, e) for s, e in endpoints.values()):
            axis_vectors = {}
            for name, (start, end) in endpoints.items():
                tip = end if _same_point(candidate, start) else start
                axis_vectors[name] = tip - candidate
            return candidate, axis_vectors
    raise ValueError("Failed to identify a shared origin in the baked frame group.")


def reconstruct_frame(group_members, scale_to_mm: float):
    """Return (4x4 frame matrix in mm, label) from a baked frame group."""
    axis_lines = _extract_axis_lines(group_members)
    origin, axis_vectors = _shared_origin_mm(axis_lines, scale_to_mm)
    frame = frame_from_axes(
        origin,
        axis_vectors["frame_x_axis"],
        axis_vectors["frame_y_axis"],
        axis_vectors["frame_z_axis"],
    )
    frame[:3, :3] = orthonormalize_rotation(frame[:3, :3])
    return frame, frame_label_from_group(group_members)
