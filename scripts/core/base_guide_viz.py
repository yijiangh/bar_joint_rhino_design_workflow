"""Bake the base-placement guide lines into the Rhino document.

Rhino half of :mod:`core.base_guide_geom` (which owns the pure-numpy geometry).
RSIKKeyframe draws, for the picked bar, five lines on the walkable ground showing
where the mobile base may stand -- see ``base_guide_geom`` for what the lines
mean. (RSIKKeyframeAll draws no guides: with thirty bars they are clutter, and
drawing them was the slow part of that command.)

Structured exactly like :mod:`core.base_frame_viz`: everything lands on one
dedicated preview layer (``config.LAYER_BASE_GUIDE_PREVIEW``), every object is
tagged with its bar id via user-text so a pick resolves back to the bar, each
bar's objects are grouped so one click selects the whole guide, and the whole
layer is cleared wholesale on exit.

The guides are real document objects, not display-conduit drawings, on purpose:
object snaps only see document objects, and the point of the guides is to snap
the base pick onto them. The baking is fast because every object is added
through RhinoCommon with its attributes prepared up front and the viewport
redraw suspended -- one redraw for the whole set, instead of one per call as the
``rhinoscriptsyntax`` route does.

The guide layer is deliberately SEPARATE from ``LAYER_BASE_FRAME_PREVIEW``:
``base_frame_viz.draw_base_frames`` clears its own layer before drawing, so
sharing one layer would wipe the guides every time a base marker is re-baked.
Neither layer is in ``config.MANAGED_LAYERS`` -- the managed-layer enforcer would
evict their contents as strays.
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


# User-text key stamped on every guide object so a pick resolves to its bar.
GUIDE_BAR_ID_KEY = "base_guide_bar_id"

# Hues chosen against the preview-color registry documented in
# rs_define_joint_half.py: blue = selected bar / reach circle, amber = ground
# preview, green = walkable ground, orange = orphan link, red = base heading,
# magenta = ghost tool. Teal + yellow were free.
_PROJECTED_COLOR = (0, 160, 160)     # offset 0: the joint line projected on the ground
_OFFSET_COLOR = (120, 200, 200)      # the 375 / 500 / 625 mm offset lines
_EXTENSION_COLOR = (255, 255, 0)     # the midpoint line the base origin sits on


def clear_base_guides() -> None:
    """Delete every baked guide line (all objects on the guide preview layer)."""
    oids = objects_on_layers(config.LAYER_BASE_GUIDE_PREVIEW)
    if oids:
        rs.DeleteObjects(oids)


def _pt(point_mm, scale_from_mm):
    """Return a Rhino ``Point3d`` for a mm point, converted to document units."""
    p = np.asarray(point_mm, dtype=float) * scale_from_mm
    return Rhino.Geometry.Point3d(float(p[0]), float(p[1]), float(p[2]))


def _add_line_or_point(start, end, attrs):
    """Add the line ``start -> end``; a zero-length one (a post seen from above) becomes a point.

    Rhino refuses a zero-length line, and the two joint centres of a vertical bar
    project onto one ground point, which used to fail the whole guide with
    "Unable to add line to document".

    Returns:
        System.Guid | None: the new object's id, or None when Rhino refused it.
    """
    if start.DistanceTo(end) <= sc.doc.ModelAbsoluteTolerance:
        oid = sc.doc.Objects.AddPoint(start, attrs)
    else:
        oid = sc.doc.Objects.AddLine(Rhino.Geometry.Line(start, end), attrs)
    return None if oid == System.Guid.Empty else oid


def draw_base_guide(bar_id: str, guides: dict) -> list:
    """Bake one bar's guide lines and return the created object ids.

    Args:
        bar_id (str): the bar these guides belong to (tagged on every object).
        guides (dict): the result of ``core.base_guide_geom.build_base_guides``.

    Returns:
        list: the created Rhino object ids (grouped + tagged with ``bar_id``).
    """
    layer = config.LAYER_BASE_GUIDE_PREVIEW
    tag = {GUIDE_BAR_ID_KEY: bar_id}
    projected_attrs = preview_attributes(layer, _PROJECTED_COLOR, tag)
    offset_attrs = preview_attributes(layer, _OFFSET_COLOR, tag)
    extension_attrs = preview_attributes(layer, _EXTENSION_COLOR, tag)
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    created = []

    for distance, (end_a, end_b) in zip(guides["offsets"], guides["lines"]):
        start, end = _pt(end_a, scale_from_mm), _pt(end_b, scale_from_mm)
        line = _add_line_or_point(start, end, projected_attrs if distance == 0.0 else offset_attrs)
        if line is None:
            continue
        created.append(line)
        if distance == 0.0:
            continue
        # Label each offset so the user can read the standoff straight off the
        # canvas before typing it at the prompt.
        middle = Rhino.Geometry.Point3d((start.X + end.X) / 2.0, (start.Y + end.Y) / 2.0, (start.Z + end.Z) / 2.0)
        dot = sc.doc.Objects.AddTextDot(f"{distance:.0f}", middle, offset_attrs)
        if dot != System.Guid.Empty:
            created.append(dot)

    # The extension line: joins the midpoints, and carries the placed base origin.
    midpoints = [_pt(m, scale_from_mm) for m in guides["extension"]]
    if len(midpoints) >= 2:
        polyline = Rhino.Geometry.Polyline()
        for point in midpoints:
            polyline.Add(point)
        polyline.DeleteShortSegments(sc.doc.ModelAbsoluteTolerance)
        extension = None
        if polyline.Count >= 2:
            extension = sc.doc.Objects.AddPolyline(polyline, extension_attrs)
        elif polyline.Count == 1:
            extension = sc.doc.Objects.AddPoint(polyline[0], extension_attrs)
        if extension is not None and extension != System.Guid.Empty:
            created.append(extension)

    group_objects(created)
    return created


def draw_base_guides(guides_by_bar: dict) -> None:
    """Clear then bake guide lines for many bars, with one redraw at the end.

    Args:
        guides_by_bar (dict): ``{bar_id: guides}``, each value a
            ``core.base_guide_geom.build_base_guides`` result (``None`` values
            are skipped, so callers can pass bars whose ground projection failed).

    Returns:
        None.
    """
    with session_log.step("draw base guides"), suspend_redraw():
        clear_base_guides()
        for bar_id, guides in guides_by_bar.items():
            if not guides:
                continue
            try:
                draw_base_guide(bar_id, guides)
            except Exception as exc:  # one bad guide must not abort the rest
                session_log.caught(exc)
                print(f"core.base_guide_viz.draw_base_guides: bar '{bar_id}' skipped ({exc}).")


def bar_id_of_picked(oid) -> str:
    """Return the bar id tagged on a picked guide object, or ''."""
    return rs.GetUserText(oid, GUIDE_BAR_ID_KEY) or ""
