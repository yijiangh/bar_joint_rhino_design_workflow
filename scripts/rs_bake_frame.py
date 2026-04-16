#! python 3
# venv: scaffolding_env
"""Bake a right-handed frame in Rhino from three picked points.

Run in Rhino 8 with:
    _-ScriptEditor _R "C:\\path\\to\\bake_frame.py"
"""

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


_DEFAULT_AXIS_LENGTH_MM = 5.0
_DEFAULT_TEXT_DOT_OFFSET_MM = 2.5
_EPSILON = 1e-9


def _unitized(vector):
    unit = Rhino.Geometry.Vector3d(vector)
    if unit.Length <= _EPSILON:
        return None
    unit.Unitize()
    return unit


def _axis_length_in_doc_units():
    axis_length_mm = rs.GetReal(
        "Axis length in mm",
        number=_DEFAULT_AXIS_LENGTH_MM,
        minimum=_EPSILON,
    )
    if axis_length_mm is None:
        return None

    unit_scale = Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Millimeters,
        sc.doc.ModelUnitSystem,
    )
    return axis_length_mm * unit_scale


def _text_dot_offset_in_doc_units():
    text_dot_offset_mm = rs.GetReal(
        "Text dot offset in mm",
        number=_DEFAULT_TEXT_DOT_OFFSET_MM,
        minimum=_EPSILON,
    )
    if text_dot_offset_mm is None:
        return None

    unit_scale = Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Millimeters,
        sc.doc.ModelUnitSystem,
    )
    return text_dot_offset_mm * unit_scale


def _get_frame_name():
    get_string = Rhino.Input.Custom.GetString()
    get_string.SetCommandPrompt("Frame name")
    get_string.AcceptNothing(True)
    get_string.SetDefaultString("")
    result = get_string.Get()
    if result == Rhino.Input.GetResult.Cancel:
        return None
    if result == Rhino.Input.GetResult.Nothing:
        return ""
    return get_string.StringResult() or ""


def _pick_frame_points():
    origin = rs.GetPoint("Pick frame origin")
    if origin is None:
        return None

    x_point = rs.GetPoint("Pick a point on the +X axis", origin)
    if x_point is None:
        return None

    y_point = rs.GetPoint("Pick a point on the +Y side", origin)
    if y_point is None:
        return None

    return origin, x_point, y_point


def _build_frame(origin, x_point, y_point):
    x_axis = _unitized(x_point - origin)
    if x_axis is None:
        rs.MessageBox("The X-axis point must be different from the origin.")
        return None

    y_hint = _unitized(y_point - origin)
    if y_hint is None:
        rs.MessageBox("The Y-axis point must be different from the origin.")
        return None

    z_axis = Rhino.Geometry.Vector3d.CrossProduct(x_axis, y_hint)
    z_axis = _unitized(z_axis)
    if z_axis is None:
        rs.MessageBox("The X and Y picks are collinear. Please pick a valid Y-side point.")
        return None

    y_axis = Rhino.Geometry.Vector3d.CrossProduct(z_axis, x_axis)
    y_axis = _unitized(y_axis)
    if y_axis is None:
        rs.MessageBox("Failed to construct an orthogonal Y axis.")
        return None

    return x_axis, y_axis, z_axis


def _bake_axis(origin, axis, length, name, color):
    end_point = origin + axis * length
    line_id = rs.AddLine(origin, end_point)
    if line_id is None:
        return None

    rs.ObjectName(line_id, name)
    rs.ObjectColor(line_id, color)
    return line_id


def _bake_frame_name(origin, x_axis, text_dot_offset, frame_name):
    if not frame_name:
        return None

    dot_point = origin - x_axis * text_dot_offset
    dot_id = rs.AddTextDot(frame_name, dot_point)
    if dot_id is None:
        return None

    rs.ObjectName(dot_id, frame_name)
    return dot_id


def _group_baked_objects(object_ids):
    if not object_ids:
        return None

    group_name = rs.AddGroup()
    if not group_name:
        return None

    rs.AddObjectsToGroup(object_ids, group_name)
    return group_name


def main():
    axis_length = _axis_length_in_doc_units()
    if axis_length is None:
        return

    frame_name = _get_frame_name()
    if frame_name is None:
        return

    text_dot_offset = None
    if frame_name:
        text_dot_offset = _text_dot_offset_in_doc_units()
        if text_dot_offset is None:
            return

    picked_points = _pick_frame_points()
    if picked_points is None:
        return

    origin, x_point, y_point = picked_points
    frame = _build_frame(origin, x_point, y_point)
    if frame is None:
        return

    x_axis, y_axis, z_axis = frame
    baked_ids = [
        _bake_axis(origin, x_axis, axis_length, "frame_x_axis", (255, 0, 0)),
        _bake_axis(origin, y_axis, axis_length, "frame_y_axis", (0, 255, 0)),
        _bake_axis(origin, z_axis, axis_length, "frame_z_axis", (0, 0, 255)),
        _bake_frame_name(origin, x_axis, text_dot_offset, frame_name),
    ]
    baked_ids = [object_id for object_id in baked_ids if object_id is not None]
    if baked_ids:
        _group_baked_objects(baked_ids)
        rs.SelectObjects(baked_ids)
        rs.Redraw()


if __name__ == "__main__":
    main()
