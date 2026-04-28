"""Shared Rhino helper utilities used by all rs_*.py entry-point scripts.

This module depends on rhinoscriptsyntax and scriptcontext which are only
available inside Rhino 8 ScriptEditor.  Do not import from standalone tests.
"""

import contextlib

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


# ---------------------------------------------------------------------------
# Point / curve utilities
# ---------------------------------------------------------------------------

def point_to_array(point):
    """Convert a Rhino Point3d (or any XYZ-like) to a numpy array."""
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def curve_endpoints(curve_id):
    """Return (start, end) as numpy arrays for a Rhino curve."""
    start = point_to_array(rs.CurveStartPoint(curve_id))
    end = point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


# ---------------------------------------------------------------------------
# Object-ID list normalisation
# ---------------------------------------------------------------------------

def as_object_id_list(object_ids):
    """Normalise *object_ids* (single id, list, or None) to a flat list."""
    if object_ids is None:
        return []
    if isinstance(object_ids, (str, bytes)):
        return [object_ids]
    try:
        return [oid for oid in object_ids if oid is not None]
    except TypeError:
        return [object_ids]


# ---------------------------------------------------------------------------
# Layer helpers
# ---------------------------------------------------------------------------

def ensure_layer(layer_name):
    """Create *layer_name* (possibly a nested ``Parent::Child`` path) if it
    does not exist, and make every layer along the path visible.  Returns
    the full path."""
    parts = layer_name.split("::")
    cur = ""
    for i, name in enumerate(parts):
        cur = name if i == 0 else cur + "::" + name
        if not rs.IsLayer(cur):
            rs.AddLayer(cur)
        if hasattr(rs, "LayerVisible") and not rs.LayerVisible(cur):
            rs.LayerVisible(cur, True)
    return cur


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def apply_object_display(object_ids, label, color=None, layer_name=None):
    """Set name, color, layer, and reference_label user-text on objects."""
    baked_ids = as_object_id_list(object_ids)
    multiple = len(baked_ids) > 1
    for i, oid in enumerate(baked_ids):
        if layer_name is not None:
            ensure_layer(layer_name)
            rs.ObjectLayer(oid, layer_name)
        if color is not None:
            if hasattr(rs, "ObjectColorSource"):
                rs.ObjectColorSource(oid, 1)
            rs.ObjectColor(oid, color)
        obj_label = f"{label}_{i + 1}" if multiple else label
        rs.ObjectName(oid, obj_label)
        rs.SetUserText(oid, "reference_label", label)
    return baked_ids


def set_object_color(object_ids, color):
    """Set by-object color on one or more Rhino objects."""
    for oid in as_object_id_list(object_ids):
        if not rs.IsObject(oid):
            continue
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(oid, 1)
        rs.ObjectColor(oid, color)


def delete_objects(object_ids):
    """Delete one or more Rhino objects (silently skips missing ones)."""
    for oid in as_object_id_list(object_ids):
        if rs.IsObject(oid):
            rs.DeleteObject(oid)


# ---------------------------------------------------------------------------
# Layer + group helpers
# ---------------------------------------------------------------------------

def set_objects_layer(object_ids, layer_name):
    """Move objects to *layer_name*, creating it if needed."""
    baked_ids = as_object_id_list(object_ids)
    ensure_layer(layer_name)
    for oid in baked_ids:
        if rs.IsObject(oid):
            rs.ObjectLayer(oid, layer_name)
    return baked_ids


def group_objects(object_ids):
    """Group objects and return the group name, or None."""
    baked_ids = as_object_id_list(object_ids)
    if not baked_ids:
        return None
    group_name = rs.AddGroup()
    if not group_name:
        return None
    rs.AddObjectsToGroup(baked_ids, group_name)
    return group_name


# ---------------------------------------------------------------------------
# Redraw context manager
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def suspend_redraw():
    """Context manager that disables viewport redraw for performance."""
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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def add_centered_line(midpoint, direction, length_mm):
    """Add a Rhino line of *length_mm* centered at *midpoint* along
    *direction* (unit-normalized internally).  Returns the new line's
    object id.
    """
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    half_length = float(length_mm) / 2.0
    return rs.AddLine(
        midpoint - half_length * direction, midpoint + half_length * direction
    )
