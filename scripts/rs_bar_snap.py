#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarSnap - Snap a new bar onto an existing bar at contact distance.

Pick an existing bar (Le) and a new bar (Ln). The script translates Ln so that
the shortest distance between Le and Ln equals BAR_CONTACT_DISTANCE.
"""

import contextlib
import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import geometry


_REFERENCE_SEGMENT_PRINT_WIDTH = 0.8
_BAR_AXIS_LAYER = "Bar Axis Lines"
_CONTACT_SEGMENT_LAYER = "Contact Segments"
_TUBE_LAYER = "Tube preview"
_S1_EXISTING_BAR_COLOR = (120, 120, 120)
_DEFAULT_TUBE_COLOR = (205, 150, 60)


# ---------------------------------------------------------------------------
# Helpers (shared with rs_bar_brace.py)
# ---------------------------------------------------------------------------

def _point_to_array(point):
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def _curve_endpoints(curve_id):
    start = _point_to_array(rs.CurveStartPoint(curve_id))
    end = _point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


def _as_object_id_list(object_ids):
    if object_ids is None:
        return []
    if isinstance(object_ids, (str, bytes)):
        return [object_ids]
    try:
        return [oid for oid in object_ids if oid is not None]
    except TypeError:
        return [object_ids]


def _ensure_layer(layer_name):
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def _apply_object_display(object_ids, label, color=None, layer_name=None):
    baked_ids = _as_object_id_list(object_ids)
    multiple = len(baked_ids) > 1
    for i, oid in enumerate(baked_ids):
        if layer_name is not None:
            _ensure_layer(layer_name)
            rs.ObjectLayer(oid, layer_name)
        if color is not None:
            if hasattr(rs, "ObjectColorSource"):
                rs.ObjectColorSource(oid, 1)
            rs.ObjectColor(oid, color)
        obj_label = f"{label}_{i + 1}" if multiple else label
        rs.ObjectName(oid, obj_label)
        rs.SetUserText(oid, "reference_label", label)
    return baked_ids


def _place_axis_line(curve_id, *, color=None, label=None):
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    _ensure_layer(_BAR_AXIS_LAYER)
    rs.ObjectLayer(curve_id, _BAR_AXIS_LAYER)
    if color is not None:
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(curve_id, 1)
        rs.ObjectColor(curve_id, color)
    if label:
        rs.SetUserText(curve_id, "axis_label", label)
    return curve_id


def _bake_reference_segment(start_point, end_point, label, color):
    start_xyz = _point_to_array(start_point)
    end_xyz = _point_to_array(end_point)
    line_id = rs.AddLine(start_xyz.tolist(), end_xyz.tolist())
    if line_id is None:
        return None
    _apply_object_display(line_id, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    if hasattr(rs, "ObjectPrintWidthSource"):
        rs.ObjectPrintWidthSource(line_id, 1)
    if hasattr(rs, "ObjectPrintWidth"):
        rs.ObjectPrintWidth(line_id, _REFERENCE_SEGMENT_PRINT_WIDTH)
    return line_id


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


def _bake_axis_tube(axis_curve_id, label, color=None, layer_name=_TUBE_LAYER):
    if axis_curve_id is None or not rs.IsObject(axis_curve_id):
        return []
    start_xyz, end_xyz = _curve_endpoints(axis_curve_id)
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return []
    _ensure_layer(layer_name)
    baked_ids = []
    try:
        import Rhino
    except ImportError:
        curve_domain = rs.CurveDomain(axis_curve_id)
        if curve_domain is None:
            return []
        tube_ids = rs.AddPipe(
            axis_curve_id,
            [float(curve_domain[0]), float(curve_domain[1])],
            [float(config.BAR_RADIUS), float(config.BAR_RADIUS)],
            0, 1, False,
        )
        baked_ids = _apply_object_display(tube_ids, label, color=color, layer_name=layer_name)
    else:
        axis_direction = axis_vector / axis_length
        base_plane = Rhino.Geometry.Plane(
            Rhino.Geometry.Point3d(*start_xyz.tolist()),
            Rhino.Geometry.Vector3d(*axis_direction.tolist()),
        )
        cylinder = Rhino.Geometry.Cylinder(
            Rhino.Geometry.Circle(base_plane, float(config.BAR_RADIUS)), axis_length,
        )
        brep = cylinder.ToBrep(True, True)
        if brep is None:
            return []
        tube_id = sc.doc.Objects.AddBrep(brep)
        if tube_id is None:
            return []
        baked_ids = _apply_object_display(tube_id, label, color=color, layer_name=layer_name)
    for oid in baked_ids:
        rs.SetUserText(oid, "tube_axis_id", str(axis_curve_id))
        rs.SetUserText(oid, "tube_radius", f"{config.BAR_RADIUS:.6f}")
    return baked_ids


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    _refresh_runtime_modules()
    rs.UnselectAllObjects()

    le_id = rs.GetObject("Select existing bar (Le)", rs.filter.curve)
    if le_id is None:
        return
    ln_id = rs.GetObject("Select new bar (Ln) - will be repositioned", rs.filter.curve)
    if ln_id is None:
        return

    le_start, le_end = _curve_endpoints(le_id)
    ln_start, ln_end = _curve_endpoints(ln_id)
    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start
    target_distance = float(config.BAR_CONTACT_DISTANCE)

    t_e, t_n = geometry.closest_params_infinite_lines(
        le_start, le_direction, ln_start, ln_direction,
    )
    p_e = le_start + t_e * le_direction
    p_n = ln_start + t_n * ln_direction

    segment = p_e - p_n
    current_distance = np.linalg.norm(segment)
    if current_distance <= 1e-9:
        rs.MessageBox("Error: the selected bars are coincident, so no unique contact normal exists.")
        return

    translation = (segment / current_distance) * (current_distance - target_distance)
    shortest_segment_start = p_n + translation
    shortest_segment_end = p_e

    line_id = rs.MoveObject(ln_id, translation.tolist())
    if line_id is None:
        rs.MessageBox("Error: failed to move the selected new bar.")
        return

    _bake_reference_segment(
        shortest_segment_start, shortest_segment_end,
        "RSBarSnap_shortest_segment", (180, 0, 180),
    )
    _place_axis_line(line_id, label="RSBarSnap_Ln")
    _bake_axis_tube(le_id, "RSBarSnap_Le_tube", color=_S1_EXISTING_BAR_COLOR)
    _bake_axis_tube(line_id, "RSBarSnap_bar_tube", color=_DEFAULT_TUBE_COLOR)
    rs.SelectObject(line_id)
    print(f"RSBarSnap: Bar placed at distance {target_distance:.2f} from Le")


if __name__ == "__main__":
    main()
