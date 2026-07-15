"""Draw the robot base frame(s) as baked Rhino objects (axis triad + footprint).

The IK viewer (RSShowBarActionPlan) and the loader (RSLoadSolvedBarAction) show where the
mobile base stands for a solved bar. This module bakes, per bar, a small marker on
``config.LAYER_BASE_FRAME_PREVIEW``:

- the base origin point,
- an X/Y/Z axis triad (X = base heading, red; Y green; Z blue),
- a ground rectangle = the mobile-base footprint (``config.BASE_FOOTPRINT_*``),
  lying in the base XY plane.

Every object is tagged with the bar id (user-text ``base_frame_bar_id``) so a
click on a marker can activate that bar in the viewer's all-bars map. The whole
preview layer is cleared on exit. Rhino-only (imports ``rhinoscriptsyntax``).
"""

from __future__ import annotations

import numpy as np
import rhinoscriptsyntax as rs

from core import config
from core.rhino_frame_io import doc_unit_scale_to_mm


# User-text key stamped on every marker object so a pick resolves to its bar.
BASE_FRAME_BAR_ID_KEY = "base_frame_bar_id"

_AXIS_COLORS = {
    "x": (220, 40, 40),    # heading -> red
    "y": (40, 180, 40),    # green
    "z": (40, 90, 220),    # blue
}
_FOOTPRINT_COLOR = (150, 150, 150)  # grey rectangle


def _ensure_preview_layer() -> str:
    """Ensure the base-frame preview layer exists and return its name."""
    layer = config.LAYER_BASE_FRAME_PREVIEW
    if not rs.IsLayer(layer):
        rs.AddLayer(layer)
    return layer


def clear_base_frames() -> None:
    """Delete every baked base-frame marker (all objects on the preview layer)."""
    layer = config.LAYER_BASE_FRAME_PREVIEW
    if not rs.IsLayer(layer):
        return
    oids = rs.ObjectsByLayer(layer) or []
    if oids:
        rs.DeleteObjects(oids)


def _pt(origin_doc, vec, dist):
    """Return a Rhino Point3d at ``origin_doc + vec * dist`` (all doc units)."""
    return rs.CreatePoint(
        origin_doc[0] + vec[0] * dist,
        origin_doc[1] + vec[1] * dist,
        origin_doc[2] + vec[2] * dist,
    )


def draw_base_frame(bar_id: str, base_frame_mm) -> list:
    """Bake one bar's base-frame marker (triad + footprint) and return its oids.

    Args:
        bar_id (str): the bar this base frame belongs to (tagged on each object).
        base_frame_mm (np.ndarray): 4x4 mm base frame (X=heading, Z=up).

    Returns:
        list: the created Rhino object ids (grouped + tagged with ``bar_id``).
    """
    layer = _ensure_preview_layer()
    m = np.asarray(base_frame_mm, dtype=float)
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()

    origin = (m[:3, 3] * scale_from_mm)
    x_axis, y_axis, z_axis = m[:3, 0], m[:3, 1], m[:3, 2]
    axis_len = config.BASE_FRAME_AXIS_LEN_MM * scale_from_mm
    half_l = 0.5 * config.BASE_FOOTPRINT_LENGTH_MM * scale_from_mm
    half_w = 0.5 * config.BASE_FOOTPRINT_WIDTH_MM * scale_from_mm

    origin_pt = rs.CreatePoint(origin[0], origin[1], origin[2])
    created = []

    # Axis triad: three colored lines from the origin.
    for key, axis in (("x", x_axis), ("y", y_axis), ("z", z_axis)):
        line = rs.AddLine(origin_pt, _pt(origin, axis, axis_len))
        rs.ObjectColor(line, _AXIS_COLORS[key])
        created.append(line)

    # Ground footprint rectangle in the base XY plane (corners ordered CCW).
    corners = [
        _pt(origin, x_axis * a + y_axis * b, 1.0)
        for a, b in ((half_l, half_w), (half_l, -half_w),
                     (-half_l, -half_w), (-half_l, half_w))
    ]
    rect = rs.AddPolyline(corners + [corners[0]])
    if rect:
        rs.ObjectColor(rect, _FOOTPRINT_COLOR)
        created.append(rect)

    # Origin dot.
    dot = rs.AddPoint(origin_pt)
    if dot:
        created.append(dot)

    # Layer + bar-id tag on every object, then group them so a pick highlights all.
    for oid in created:
        rs.ObjectLayer(oid, layer)
        rs.SetUserText(oid, BASE_FRAME_BAR_ID_KEY, bar_id)
    if created:
        grp = rs.AddGroup()
        rs.AddObjectsToGroup(created, grp)
    return created


def draw_base_frames(frames_by_bar: dict) -> None:
    """Clear then bake base-frame markers for many bars.

    Args:
        frames_by_bar (dict): ``{bar_id: base_frame_mm (4x4)}``.

    Returns:
        None.
    """
    clear_base_frames()
    for bar_id, base_mm in frames_by_bar.items():
        try:
            draw_base_frame(bar_id, base_mm)
        except Exception as exc:  # one bad frame must not abort the rest
            print(f"core.base_frame_viz.draw_base_frames: bar '{bar_id}' skipped ({exc}).")


def bar_id_of_picked(oid) -> str:
    """Return the bar id tagged on a picked base-frame marker object, or ''."""
    return rs.GetUserText(oid, BASE_FRAME_BAR_ID_KEY) or ""
