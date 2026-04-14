"""Export CAD-backed connector config from baked Rhino frames.

Run in Rhino 8 with:
    _-ScriptEditor _R "C:\\path\\to\\export_cad_config.py"
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.geometry import closest_params_infinite_lines, distance_infinite_lines
from core.transforms import (
    frame_from_axes,
    invert_transform,
    local_transform,
    orthonormalize_rotation,
    project_point_to_line,
    rotation_preserves_local_z,
)


_FRAME_NAMES = (
    "le_bar_link",
    "female_link",
    "female_screw_hole_link",
    "male_screw_hole_link",
    "male_link",
    "ln_bar_link",
)
_AXIS_OBJECT_NAMES = ("frame_x_axis", "frame_y_axis", "frame_z_axis")
_LINE_TOL = 1e-6
_SEGMENT_COLOR = (255, 64, 255)
_LOCAL_Z_QUARTER_TURN = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def _doc_unit_scale_to_mm() -> float:
    return float(Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters))


def _point_to_mm(point, scale_to_mm: float) -> np.ndarray:
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float) * scale_to_mm
    return np.asarray(point, dtype=float) * scale_to_mm


def _line_endpoints_mm(object_id, scale_to_mm: float) -> tuple[np.ndarray, np.ndarray]:
    curve = rs.coercecurve(object_id)
    if curve is None:
        raise ValueError("Expected a line object in the baked frame group.")
    start = _point_to_mm(curve.PointAtStart, scale_to_mm)
    end = _point_to_mm(curve.PointAtEnd, scale_to_mm)
    return start, end


def _same_point(point_a: np.ndarray, point_b: np.ndarray, *, tol: float = _LINE_TOL) -> bool:
    return float(np.linalg.norm(point_a - point_b)) <= tol


def _group_members_from_selection(object_id):
    groups = rs.ObjectGroups(object_id) or []
    if not groups:
        raise ValueError("Selected object is not part of a baked frame group.")
    if len(groups) > 1:
        raise ValueError("Selected object belongs to multiple groups; please select a single baked frame group.")
    group_name = groups[0]
    members = rs.ObjectsByGroup(group_name) or []
    if not members:
        raise ValueError("Selected frame group is empty.")
    return group_name, members


def _resolve_frame_group(prompt: str):
    object_id = rs.GetObject(prompt)
    if object_id is None:
        return None
    return _group_members_from_selection(object_id)


def _frame_label_from_group(group_members) -> str:
    for object_id in group_members:
        if rs.IsTextDot(object_id):
            text = rs.TextDotText(object_id)
            if text:
                return text
        name = rs.ObjectName(object_id)
        if name and name not in _AXIS_OBJECT_NAMES:
            return name
    return ""


def _extract_axis_lines(group_members):
    axis_lines = {}
    for object_id in group_members:
        name = rs.ObjectName(object_id)
        if name in _AXIS_OBJECT_NAMES:
            axis_lines[name] = object_id
    missing = [name for name in _AXIS_OBJECT_NAMES if name not in axis_lines]
    if missing:
        raise ValueError(f"Frame group is missing baked axes: {', '.join(missing)}")
    return axis_lines


def _shared_origin_mm(axis_lines, scale_to_mm: float) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    endpoints = {name: _line_endpoints_mm(object_id, scale_to_mm) for name, object_id in axis_lines.items()}
    candidates = []
    for start, end in endpoints.values():
        candidates.extend([start, end])
    for candidate in candidates:
        if all(_same_point(candidate, start) or _same_point(candidate, end) for start, end in endpoints.values()):
            axis_vectors = {}
            for name, (start, end) in endpoints.items():
                tip = end if _same_point(candidate, start) else start
                axis_vectors[name] = tip - candidate
            return candidate, axis_vectors
    raise ValueError("Failed to identify a shared origin in the baked frame group.")


def _reconstruct_frame(group_members, scale_to_mm: float) -> tuple[np.ndarray, str]:
    axis_lines = _extract_axis_lines(group_members)
    origin, axis_vectors = _shared_origin_mm(axis_lines, scale_to_mm)
    x_axis = axis_vectors["frame_x_axis"]
    y_axis = axis_vectors["frame_y_axis"]
    z_axis = axis_vectors["frame_z_axis"]
    frame = frame_from_axes(origin, x_axis, y_axis, z_axis)
    frame[:3, :3] = orthonormalize_rotation(frame[:3, :3])
    return frame, _frame_label_from_group(group_members)


def _validate_close_to_identity(rotation: np.ndarray, *, tol: float = 1e-6, message: str = "") -> None:
    if float(np.linalg.norm(rotation - np.eye(3, dtype=float))) > tol:
        raise ValueError(message or "Expected an identity rotation.")


def _validate_near_zero(vector: np.ndarray, *, tol: float = 1e-5, message: str = "") -> None:
    if float(np.linalg.norm(vector)) > tol:
        raise ValueError(message or "Expected a near-zero translation.")


def _matrix_literal(matrix: np.ndarray) -> str:
    rows = []
    for row in np.asarray(matrix, dtype=float):
        rows.append("    (" + ", ".join(f"{float(value):.9g}" for value in row) + "),")
    return "(\n" + "\n".join(rows) + "\n)"


def _triplet_literal(values: tuple[float, float, float]) -> str:
    return "(" + ", ".join(f"{float(value):.9g}" for value in values) + ")"


def _write_generated_config(output_path: str, payload: dict) -> None:
    content = f'''"""Auto-generated CAD geometry data.

Generated by `scripts/export_cad_config.py`. Re-run the Rhino exporter after CAD frame updates.
All translations are stored in millimeters.
"""

BAR_CONTACT_DISTANCE = {payload["bar_contact_distance"]:.9g}

LE_BAR_REFERENCE_FRAME = {_matrix_literal(payload["frames"]["le_bar_link"])}

LN_BAR_REFERENCE_FRAME = {_matrix_literal(payload["frames"]["ln_bar_link"])}

FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM = {_matrix_literal(payload["female_fixed_rot_from_bar_transform"])}

FEMALE_MALE_GAP_OFFSET_TRANSFORM = {_matrix_literal(payload["female_male_gap_offset_transform"])}

JJR_ZERO_TRANSFORM = {_matrix_literal(payload["jjr_zero_transform"])}

MALE_SCREW_HOLE_OFFSET_TRANSFORM = {_matrix_literal(payload["male_screw_hole_offset_transform"])}

MALE_FIXED_ROT_TO_BAR_TRANSFORM = {_matrix_literal(payload["male_fixed_rot_to_bar_transform"])}

FEMALE_MESH_FILENAME = "female_joint_mesh_m.obj"
MALE_MESH_FILENAME = "male_joint_mesh_m.obj"
FEMALE_MESH_SCALE = {_triplet_literal((1.0, 1.0, 1.0))}
MALE_MESH_SCALE = {_triplet_literal((1.0, 1.0, 1.0))}

SOURCE_FRAME_NAMES = (
    "le_bar_link",
    "female_link",
    "female_screw_hole_link",
    "male_screw_hole_link",
    "male_link",
    "ln_bar_link",
)
'''
    with open(output_path, "w", encoding="utf-8") as stream:
        stream.write(content)


def _write_snapshot(snapshot_path: str, payload: dict) -> None:
    serializable = {
        "bar_contact_distance": payload["bar_contact_distance"],
        "closest_points": {
            "le_bar_link": np.asarray(payload["closest_segment"][0], dtype=float).tolist(),
            "ln_bar_link": np.asarray(payload["closest_segment"][1], dtype=float).tolist(),
        },
        "selected_frame_labels": payload["selected_frame_labels"],
        "frames": {name: np.asarray(matrix, dtype=float).tolist() for name, matrix in payload["frames"].items()},
        "transforms": {
            "female_fixed_rot_from_bar_transform": np.asarray(payload["female_fixed_rot_from_bar_transform"], dtype=float).tolist(),
            "female_male_gap_offset_transform": np.asarray(payload["female_male_gap_offset_transform"], dtype=float).tolist(),
            "jjr_zero_transform": np.asarray(payload["jjr_zero_transform"], dtype=float).tolist(),
            "male_screw_hole_offset_transform": np.asarray(payload["male_screw_hole_offset_transform"], dtype=float).tolist(),
            "male_fixed_rot_to_bar_transform": np.asarray(payload["male_fixed_rot_to_bar_transform"], dtype=float).tolist(),
        },
    }
    with open(snapshot_path, "w", encoding="utf-8") as stream:
        json.dump(serializable, stream, indent=2)


def _bake_shortest_segment(point_a: np.ndarray, point_b: np.ndarray, scale_from_mm: float) -> None:
    line_id = rs.AddLine((point_a / scale_from_mm).tolist(), (point_b / scale_from_mm).tolist())
    if line_id is None:
        return
    rs.ObjectColor(line_id, _SEGMENT_COLOR)
    rs.ObjectName(line_id, "cad_bar_axis_shortest_segment")


def _compute_payload(frames: dict[str, np.ndarray], selected_labels: dict[str, str]) -> dict:
    le_frame = frames["le_bar_link"]
    female_frame = frames["female_link"]
    female_screw_frame = frames["female_screw_hole_link"]
    male_screw_frame = frames["male_screw_hole_link"]
    male_frame = frames["male_link"]
    ln_frame = frames["ln_bar_link"]

    le_axis = le_frame[:3, 2]
    ln_axis = ln_frame[:3, 2]

    female_origin_on_bar, _ = project_point_to_line(female_frame[:3, 3], le_frame[:3, 3], le_axis)
    male_origin_on_bar, _ = project_point_to_line(male_frame[:3, 3], ln_frame[:3, 3], ln_axis)

    fjr_zero_frame = np.array(le_frame, dtype=float, copy=True)
    fjr_zero_frame[:3, 3] = female_origin_on_bar
    mjr_zero_frame = np.array(ln_frame, dtype=float, copy=True)
    mjr_zero_frame[:3, 3] = male_origin_on_bar

    female_fixed_rot = local_transform(fjr_zero_frame, female_frame)
    _validate_near_zero(
        female_fixed_rot[:3, 3],
        message="female_fixed_rot_from_bar should be a pure rotation with zero translation.",
    )

    female_gap = local_transform(female_frame, female_screw_frame)
    _validate_close_to_identity(
        female_gap[:3, :3],
        message="female_male_gap_offset should have identity rotation.",
    )

    jjr_zero = local_transform(female_screw_frame, male_screw_frame)
    if not rotation_preserves_local_z(jjr_zero[:3, :3]):
        raise ValueError("jjr_joint zero transform must preserve the shared local Z axis.")

    male_offset = local_transform(male_screw_frame, male_frame)
    if not rotation_preserves_local_z(male_offset[:3, :3]):
        raise ValueError("male_screw_hole_offset rotation must preserve the local Z axis.")
    if float(np.linalg.norm(male_offset[:3, :3] - _LOCAL_Z_QUARTER_TURN)) > 1e-5:
        raise ValueError(
            "male_screw_hole_offset should rotate male_screw_hole_link by +90 degrees around local Z before translation."
        )

    male_fixed_rot = local_transform(male_frame, mjr_zero_frame)
    _validate_near_zero(
        male_fixed_rot[:3, 3],
        message="male_fixed_rot_to_bar should be a pure rotation with zero translation.",
    )

    bar_contact_distance = abs(float(distance_infinite_lines(le_frame[:3, 3], le_axis, ln_frame[:3, 3], ln_axis)))
    t_le, t_ln = closest_params_infinite_lines(le_frame[:3, 3], le_axis, ln_frame[:3, 3], ln_axis)
    point_le = le_frame[:3, 3] + float(t_le) * le_axis
    point_ln = ln_frame[:3, 3] + float(t_ln) * ln_axis

    return {
        "bar_contact_distance": bar_contact_distance,
        "closest_segment": (point_le, point_ln),
        "selected_frame_labels": selected_labels,
        "frames": frames,
        "female_fixed_rot_from_bar_transform": female_fixed_rot,
        "female_male_gap_offset_transform": female_gap,
        "jjr_zero_transform": jjr_zero,
        "male_screw_hole_offset_transform": male_offset,
        "male_fixed_rot_to_bar_transform": male_fixed_rot,
    }


def main() -> None:
    scale_to_mm = _doc_unit_scale_to_mm()
    scale_from_mm = 1.0 / scale_to_mm

    selections = {}
    for frame_name in _FRAME_NAMES:
        resolved = _resolve_frame_group(f"Select the baked frame group for {frame_name}")
        if resolved is None:
            return
        selections[frame_name] = resolved

    frames = {}
    labels = {}
    for frame_name, (_, group_members) in selections.items():
        frame, label = _reconstruct_frame(group_members, scale_to_mm)
        frames[frame_name] = frame
        labels[frame_name] = label

    try:
        payload = _compute_payload(frames, labels)
    except ValueError as exc:
        rs.MessageBox(str(exc), 0, "CAD Config Export")
        return

    core_dir = os.path.join(SCRIPT_DIR, "core")
    generated_path = os.path.join(core_dir, "config_generated.py")
    snapshot_path = os.path.join(core_dir, "cad_frames_snapshot.json")
    _write_generated_config(generated_path, payload)
    _write_snapshot(snapshot_path, payload)
    _bake_shortest_segment(payload["closest_segment"][0], payload["closest_segment"][1], scale_from_mm)
    rs.Redraw()

    print(f"CAD config written to {generated_path}")
    print(f"Frame snapshot written to {snapshot_path}")
    print(f"BAR_CONTACT_DISTANCE = {payload['bar_contact_distance']:.6f} mm")


if __name__ == "__main__":
    main()
