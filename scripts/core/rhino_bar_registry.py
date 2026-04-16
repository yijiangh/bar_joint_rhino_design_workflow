"""Bar identity and tube-preview management for the Rhino scaffolding workflow.

Every scaffolding bar (a Rhino curve) gets:
  - ``bar_type``  user-text key  → ``"scaffolding_bar"``
  - ``bar_id``    user-text key  → ``"B1"``, ``"B2"``, …
  - ``bar_guid``  user-text key  → the Rhino GUID at assignment time
  - ObjectName                   → same as ``bar_id``

Tube previews (display cylinders) live on the ``"Tube preview"`` layer and
cache the axis endpoints so stale tubes can be detected and regenerated.

This module depends on rhinoscriptsyntax / scriptcontext (Rhino 8 only).
"""

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core.rhino_helpers import (
    as_object_id_list,
    apply_object_display,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAR_TYPE_KEY = "bar_type"
BAR_ID_KEY = "bar_id"
BAR_GUID_KEY = "bar_guid"
BAR_TYPE_VALUE = "scaffolding_bar"

TUBE_LAYER = "Tube preview"
TUBE_BAR_ID_KEY = "tube_bar_id"
TUBE_AXIS_GUID_KEY = "tube_axis_id"
TUBE_CACHE_START = "tube_cache_start"
TUBE_CACHE_END = "tube_cache_end"
BAR_RADIUS_KEY = "tube_radius"

_POINT_TOL = 1e-3  # mm tolerance for endpoint cache comparison


# ---------------------------------------------------------------------------
# Bar ID helpers
# ---------------------------------------------------------------------------

def _parse_bar_number(bar_id):
    """Extract integer from a bar_id like 'B7' → 7.  Return None on failure."""
    if not bar_id or not bar_id.upper().startswith("B"):
        return None
    try:
        return int(bar_id[1:])
    except (ValueError, IndexError):
        return None


def next_bar_id():
    """Return the next available bar ID (e.g. ``'B4'``)."""
    max_num = 0
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
            num = _parse_bar_number(rs.GetUserText(oid, BAR_ID_KEY))
            if num is not None and num > max_num:
                max_num = num
    return f"B{max_num + 1}"


def ensure_bar_id(curve_id, bar_type=BAR_TYPE_VALUE):
    """Ensure *curve_id* has a bar ID.  Assigns one if missing or copy-pasted.

    Returns the ``bar_id`` string (e.g. ``'B3'``).
    """
    current_guid = str(rs.coerceguid(curve_id))
    existing_type = rs.GetUserText(curve_id, BAR_TYPE_KEY)
    existing_guid = rs.GetUserText(curve_id, BAR_GUID_KEY)

    if existing_type == bar_type and existing_guid == current_guid:
        # Already registered and GUID matches — just ensure ObjectName is in sync.
        bar_id = rs.GetUserText(curve_id, BAR_ID_KEY)
        if bar_id and rs.ObjectName(curve_id) != bar_id:
            rs.ObjectName(curve_id, bar_id)
        return bar_id

    # New bar or copy-paste detected — assign a fresh ID.
    new_id = next_bar_id()
    rs.SetUserText(curve_id, BAR_TYPE_KEY, bar_type)
    rs.SetUserText(curve_id, BAR_ID_KEY, new_id)
    rs.SetUserText(curve_id, BAR_GUID_KEY, current_guid)
    rs.ObjectName(curve_id, new_id)
    return new_id


def is_bar(curve_id):
    """Return True if *curve_id* is a registered scaffolding bar."""
    return rs.GetUserText(curve_id, BAR_TYPE_KEY) == BAR_TYPE_VALUE


def get_all_bars():
    """Scan the document for all registered bars.  Heals copy-paste artifacts.

    Returns ``{bar_id: curve_guid}`` dict.
    """
    bars = {}
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
            bar_id = ensure_bar_id(oid)  # heals guid mismatch / name drift
            bars[bar_id] = rs.coerceguid(oid)
    return bars


# ---------------------------------------------------------------------------
# Tube-preview helpers
# ---------------------------------------------------------------------------

def _format_point(arr):
    """Serialise a 3-element array to ``'x,y,z'`` with 6 decimal places."""
    return f"{float(arr[0]):.6f},{float(arr[1]):.6f},{float(arr[2]):.6f}"


def _parse_cached_point(s):
    """Parse ``'x,y,z'`` back to a 3-tuple of floats, or None on failure."""
    if not s:
        return None
    try:
        parts = s.split(",")
        if len(parts) != 3:
            return None
        return tuple(float(p) for p in parts)
    except (ValueError, TypeError):
        return None


def _find_existing_tube(curve_id):
    """Find a tube on *TUBE_LAYER* whose ``tube_axis_id`` matches *curve_id*.

    Returns the tube object GUID or None.
    """
    if not rs.IsLayer(TUBE_LAYER):
        return None
    curve_guid_str = str(rs.coerceguid(curve_id))
    for oid in rs.ObjectsByLayer(TUBE_LAYER):
        if rs.GetUserText(oid, TUBE_AXIS_GUID_KEY) == curve_guid_str:
            return oid
    return None


def _tube_geometry_matches(tube_id, curve_id):
    """Return True if the tube's cached endpoints match the current curve."""
    cached_start = _parse_cached_point(rs.GetUserText(tube_id, TUBE_CACHE_START))
    cached_end = _parse_cached_point(rs.GetUserText(tube_id, TUBE_CACHE_END))
    if cached_start is None or cached_end is None:
        return False
    cur_start = point_to_array(rs.CurveStartPoint(curve_id))
    cur_end = point_to_array(rs.CurveEndPoint(curve_id))
    start_ok = np.linalg.norm(np.array(cached_start) - cur_start) < _POINT_TOL
    end_ok = np.linalg.norm(np.array(cached_end) - cur_end) < _POINT_TOL
    return start_ok and end_ok


def _create_tube_brep(start_xyz, end_xyz, bar_radius):
    """Build a capped cylinder Brep between two endpoints.  Returns GUID or None."""
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return None
    axis_direction = axis_vector / axis_length
    base_plane = Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(*start_xyz.tolist()),
        Rhino.Geometry.Vector3d(*axis_direction.tolist()),
    )
    cylinder = Rhino.Geometry.Cylinder(
        Rhino.Geometry.Circle(base_plane, float(bar_radius)),
        axis_length,
    )
    brep = cylinder.ToBrep(True, True)
    if brep is None:
        return None
    tube_id = sc.doc.Objects.AddBrep(brep)
    return tube_id


def ensure_bar_preview(curve_id, bar_radius, color=None, bar_id=None):
    """Make sure a tube preview exists and matches the current curve geometry.

    Creates or regenerates the tube as needed.  Returns list of tube GUIDs.
    """
    # Check for an existing tube
    existing = _find_existing_tube(curve_id)
    if existing is not None:
        if _tube_geometry_matches(existing, curve_id):
            return [existing]
        # Stale — delete and recreate
        delete_objects([existing])

    start_xyz, end_xyz = curve_endpoints(curve_id)
    tube_id = _create_tube_brep(start_xyz, end_xyz, bar_radius)
    if tube_id is None:
        return []

    # Resolve bar_id
    if bar_id is None:
        bar_id = rs.GetUserText(curve_id, BAR_ID_KEY) or "?"

    # Display setup
    label = f"{bar_id}_tube"
    baked_ids = apply_object_display(tube_id, label, color=color, layer_name=TUBE_LAYER)

    # User text metadata
    curve_guid_str = str(rs.coerceguid(curve_id))
    for oid in baked_ids:
        rs.SetUserText(oid, TUBE_AXIS_GUID_KEY, curve_guid_str)
        rs.SetUserText(oid, TUBE_BAR_ID_KEY, bar_id)
        rs.SetUserText(oid, BAR_RADIUS_KEY, f"{bar_radius:.6f}")
        rs.SetUserText(oid, TUBE_CACHE_START, _format_point(start_xyz))
        rs.SetUserText(oid, TUBE_CACHE_END, _format_point(end_xyz))
    return baked_ids


def update_all_previews(bar_radius, color=None):
    """Ensure every registered bar has an up-to-date tube preview.

    Returns the number of bars processed.
    """
    bars = get_all_bars()
    for bar_id, guid in bars.items():
        ensure_bar_preview(guid, bar_radius, color=color, bar_id=bar_id)
    return len(bars)
