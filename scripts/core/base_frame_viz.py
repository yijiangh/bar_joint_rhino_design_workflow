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

The objects are added through RhinoCommon with their attributes prepared up front
and the redraw suspended, so many markers cost one redraw (see
``rhino_helpers.preview_attributes`` for why the ``rhinoscriptsyntax`` route was slow).
"""

from __future__ import annotations

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
import session_log
import System

from core import config
from core.rhino_helpers import (
    doc_unit_scale_to_mm,
    group_objects,
    objects_on_layers,
    preview_attributes,
    suspend_redraw,
)


# User-text key stamped on every marker object so a pick resolves to its bar.
BASE_FRAME_BAR_ID_KEY = "base_frame_bar_id"

_AXIS_COLORS = {
    "x": (220, 40, 40),    # heading -> red
    "y": (40, 180, 40),    # green
    "z": (40, 90, 220),    # blue
}
_FOOTPRINT_COLOR = (150, 150, 150)  # grey rectangle


def clear_base_frames() -> None:
    """Delete every baked base-frame marker (all objects on the preview layer)."""
    oids = objects_on_layers(config.LAYER_BASE_FRAME_PREVIEW)
    if oids:
        rs.DeleteObjects(oids)


def _pt(origin_doc, vec, dist):
    """Return a Rhino Point3d at ``origin_doc + vec * dist`` (all doc units)."""
    return Rhino.Geometry.Point3d(
        float(origin_doc[0] + vec[0] * dist),
        float(origin_doc[1] + vec[1] * dist),
        float(origin_doc[2] + vec[2] * dist),
    )


def _vec(xyz):
    """A unit-ish numpy 3-vector as a Rhino ``Vector3d``."""
    return Rhino.Geometry.Vector3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def draw_base_frame(bar_id: str, base_frame_mm, circle_radius_mm=None) -> list:
    """Bake one bar's base-frame marker (triad + footprint) and return its oids.

    Args:
        bar_id (str): the bar this base frame belongs to (tagged on each object).
        base_frame_mm (np.ndarray): 4x4 mm base frame (X=heading, Z=up).
        circle_radius_mm (float | None): when set, also bake a reach circle of this
            radius (mm) centered at the base origin in the base XY plane. ``None``
            (the default) draws no circle, so existing callers are unaffected.

    Returns:
        list: the created Rhino object ids (grouped + tagged with ``bar_id``).
    """
    layer = config.LAYER_BASE_FRAME_PREVIEW
    tag = {BASE_FRAME_BAR_ID_KEY: bar_id}
    m = np.asarray(base_frame_mm, dtype=float)
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()

    origin = (m[:3, 3] * scale_from_mm)
    x_axis, y_axis, z_axis = m[:3, 0], m[:3, 1], m[:3, 2]
    axis_len = config.BASE_FRAME_AXIS_LEN_MM * scale_from_mm
    half_l = 0.5 * config.BASE_FOOTPRINT_LENGTH_MM * scale_from_mm
    half_w = 0.5 * config.BASE_FOOTPRINT_WIDTH_MM * scale_from_mm

    origin_pt = Rhino.Geometry.Point3d(float(origin[0]), float(origin[1]), float(origin[2]))
    created = []

    def _keep(oid):
        """Collect an added object's id (Rhino returns an empty guid when it refuses)."""
        if oid is not None and oid != System.Guid.Empty:
            created.append(oid)

    # Axis triad: three colored lines from the origin.
    for key, axis in (("x", x_axis), ("y", y_axis), ("z", z_axis)):
        _keep(sc.doc.Objects.AddLine(Rhino.Geometry.Line(origin_pt, _pt(origin, axis, axis_len)),
                                     preview_attributes(layer, _AXIS_COLORS[key], tag)))

    # Optional reach circle in the base XY plane (base +Z is the plane normal).
    if circle_radius_mm:
        plane = Rhino.Geometry.Plane(origin_pt, _vec(x_axis), _vec(y_axis))
        circle = Rhino.Geometry.Circle(plane, float(circle_radius_mm) * scale_from_mm)
        _keep(sc.doc.Objects.AddCircle(circle, preview_attributes(layer, config.REACH_CLEAR_COLOR, tag)))

    # Ground footprint rectangle in the base XY plane (corners ordered CCW).
    footprint_attrs = preview_attributes(layer, _FOOTPRINT_COLOR, tag)
    corners = [
        _pt(origin, x_axis * a + y_axis * b, 1.0)
        for a, b in ((half_l, half_w), (half_l, -half_w),
                     (-half_l, -half_w), (-half_l, half_w))
    ]
    rectangle = Rhino.Geometry.Polyline()
    for corner in corners + [corners[0]]:
        rectangle.Add(corner)
    _keep(sc.doc.Objects.AddPolyline(rectangle, footprint_attrs))

    # Origin dot.
    _keep(sc.doc.Objects.AddPoint(origin_pt, footprint_attrs))

    group_objects(created)
    return created


def draw_base_frames(frames_by_bar: dict, circle_radius_mm=None) -> None:
    """Clear then bake base-frame markers for many bars, with one redraw at the end.

    Args:
        frames_by_bar (dict): ``{bar_id: base_frame_mm (4x4)}``.
        circle_radius_mm (float | None): forwarded to :func:`draw_base_frame`; when
            set, each base also gets a reach circle of this radius (mm).

    Returns:
        None.
    """
    with session_log.step("draw base frames"), suspend_redraw():
        clear_base_frames()
        for bar_id, base_mm in frames_by_bar.items():
            try:
                draw_base_frame(bar_id, base_mm, circle_radius_mm=circle_radius_mm)
            except Exception as exc:  # one bad frame must not abort the rest
                session_log.caught(exc)
                print(f"core.base_frame_viz.draw_base_frames: bar '{bar_id}' skipped ({exc}).")


def bar_id_of_picked(oid) -> str:
    """Return the bar id tagged on a picked base-frame marker object, or ''."""
    return rs.GetUserText(oid, BASE_FRAME_BAR_ID_KEY) or ""
