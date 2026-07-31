"""Bake the base-placement guide lines into the Rhino document.

Rhino half of :mod:`core.base_guide_geom` (which owns the pure-numpy geometry).
Both IK keyframe commands draw, per selected bar, five lines on the walkable
ground showing where the mobile base may stand -- see ``base_guide_geom`` for
what the lines mean.

Structured exactly like :mod:`core.base_frame_viz`: everything lands on one
dedicated preview layer (``config.LAYER_BASE_GUIDE_PREVIEW``), every object is
tagged with its bar id via user-text so a pick resolves back to the bar, each
bar's objects are grouped so one click selects the whole guide, and the whole
layer is cleared wholesale on exit.

The guide layer is deliberately SEPARATE from ``LAYER_BASE_FRAME_PREVIEW``:
``base_frame_viz.draw_base_frames`` clears its own layer before drawing, so
sharing one layer would wipe the guides every time a base marker is re-baked
(which the multi-bar Flip loop does on every flip). Neither layer is in
``config.MANAGED_LAYERS`` -- the managed-layer enforcer would evict their
contents as strays.
"""

from __future__ import annotations

import numpy as np
import rhinoscriptsyntax as rs

from core import config
from core.rhino_frame_io import doc_unit_scale_to_mm


# User-text key stamped on every guide object so a pick resolves to its bar.
GUIDE_BAR_ID_KEY = "base_guide_bar_id"

# Hues chosen against the preview-color registry documented in
# rs_define_joint_half.py: blue = selected bar / reach circle, amber = ground
# preview, green = walkable ground, orange = orphan link, red = base heading,
# magenta = ghost tool. Teal + yellow were free.
_PROJECTED_COLOR = (0, 160, 160)     # offset 0: the joint line projected on the ground
_OFFSET_COLOR = (120, 200, 200)      # the 375 / 500 / 625 mm offset lines
_EXTENSION_COLOR = (255, 255, 0)     # the midpoint line the base origin sits on


def _ensure_guide_layer() -> str:
    """Ensure the base-guide preview layer exists and return its name."""
    layer = config.LAYER_BASE_GUIDE_PREVIEW
    if not rs.IsLayer(layer):
        rs.AddLayer(layer)
    return layer


def clear_base_guides() -> None:
    """Delete every baked guide line (all objects on the guide preview layer)."""
    layer = config.LAYER_BASE_GUIDE_PREVIEW
    if not rs.IsLayer(layer):
        return
    oids = rs.ObjectsByLayer(layer) or []
    if oids:
        rs.DeleteObjects(oids)


def _pt(point_mm, scale_from_mm):
    """Return a Rhino ``Point3d`` for a mm point, converted to document units."""
    p = np.asarray(point_mm, dtype=float) * scale_from_mm
    return rs.CreatePoint(p[0], p[1], p[2])


def draw_base_guide(bar_id: str, guides: dict) -> list:
    """Bake one bar's guide lines and return the created object ids.

    Args:
        bar_id (str): the bar these guides belong to (tagged on every object).
        guides (dict): the result of ``core.base_guide_geom.build_base_guides``.

    Returns:
        list: the created Rhino object ids (grouped + tagged with ``bar_id``).
    """
    layer = _ensure_guide_layer()
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    created = []

    for distance, (end_a, end_b) in zip(guides["offsets"], guides["lines"]):
        line = rs.AddLine(_pt(end_a, scale_from_mm), _pt(end_b, scale_from_mm))
        if not line:
            continue
        rs.ObjectColor(line, _PROJECTED_COLOR if distance == 0.0 else _OFFSET_COLOR)
        created.append(line)
        if distance == 0.0:
            continue
        # Label each offset so the user can read the standoff straight off the
        # canvas before typing it at the prompt.
        dot = rs.AddTextDot(f"{distance:.0f}", _pt(0.5 * (np.asarray(end_a, dtype=float)
                                                          + np.asarray(end_b, dtype=float)),
                                                  scale_from_mm))
        if dot:
            rs.ObjectColor(dot, _OFFSET_COLOR)
            created.append(dot)

    # The extension line: joins the midpoints, and carries the placed base origin.
    midpoints = [_pt(m, scale_from_mm) for m in guides["extension"]]
    if len(midpoints) >= 2:
        extension = rs.AddPolyline(midpoints)
        if extension:
            rs.ObjectColor(extension, _EXTENSION_COLOR)
            created.append(extension)

    for oid in created:
        rs.ObjectLayer(oid, layer)
        rs.SetUserText(oid, GUIDE_BAR_ID_KEY, bar_id)
    if created:
        grp = rs.AddGroup()
        rs.AddObjectsToGroup(created, grp)
    return created


def draw_base_guides(guides_by_bar: dict) -> None:
    """Clear then bake guide lines for many bars.

    Args:
        guides_by_bar (dict): ``{bar_id: guides}``, each value a
            ``core.base_guide_geom.build_base_guides`` result (``None`` values
            are skipped, so callers can pass bars whose ground projection failed).

    Returns:
        None.
    """
    clear_base_guides()
    for bar_id, guides in guides_by_bar.items():
        if not guides:
            continue
        try:
            draw_base_guide(bar_id, guides)
        except Exception as exc:  # one bad guide must not abort the rest
            print(f"core.base_guide_viz.draw_base_guides: bar '{bar_id}' skipped ({exc}).")


def bar_id_of_picked(oid) -> str:
    """Return the bar id tagged on a picked guide object, or ''."""
    return rs.GetUserText(oid, GUIDE_BAR_ID_KEY) or ""