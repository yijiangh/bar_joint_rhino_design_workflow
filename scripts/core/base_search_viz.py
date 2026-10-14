"""Draw what the base search of RSIKKeyframe did (`core.base_search`), step by step. DEBUG AID.

Shows a `base_search.Trace` while the search fills it:

- a SHORT ARROW at every ground cell in the search circle, along its cheapest allowed heading and shaded by the
  closed-form arm cost there: this dense grid is computed first (no planner) and orders the cells the search visits;
- a LONGER ARROW at every base whose full cost (arm extension + clearance) was computed, along the base's heading.
  The planner was asked about these, so each also carries a green circle;
- all arrows share one scale, pale to dark purple from the dearest to the cheapest;
- a CIRCLE at every base the robot was collision-checked at with its arms at home: green = free, red = collides;
- a LINE per descent, from where it started through every base that was cheaper than all before it;
- a big dot at every base the search proposed (the IK chain was then solved there).

Drawn by a display conduit, so nothing is baked into the document. Switched by ``config.DEBUG_BASE_SEARCH_TRACE``.

To remove: delete this file, that constant, and the lines naming ``base_search_viz`` in ``rs_ik_keyframe.py``.
"""

from __future__ import annotations

import Rhino
import scriptcontext as sc
import session_log
import System
from System.Drawing import Color

from core import config
from core.rhino_helpers import doc_unit_scale_to_mm

# Last conduit switched on, so `hide` (and the next search) finds it even after a module reload.
_STICKY_CONDUIT = "bar_joint:base_search_trace_conduit"

_COLOR_DEAR = (225, 215, 245)
_COLOR_CHEAP = (90, 30, 170)
_COLOR_FREE = (30, 160, 60)
_COLOR_COLLIDES = (220, 30, 30)
_COLOR_PATH = (40, 40, 40)
# Sizes on the ground, in mm; the marks float `_LIFT_MM` above it so the surface does not swallow them.
_ARROW_MM = 40.0
_CIRCLE_MM = 12.0
_CELL_ARROW_MM = 25.0
_LIFT_MM = 5.0
# How many shades the grid's arrows are drawn in.
_SHADES = 8


class _TraceConduit(Rhino.Display.DisplayConduit):
    """Draws a trace; the geometry is built again whenever the search has added to it."""

    def __init__(self, trace):
        super().__init__()
        self._trace = trace
        self._seen = None
        self._arrows, self._circles, self._lines, self._dots = [], [], [], []
        self._cells = []
        self._box = Rhino.Geometry.BoundingBox.Empty

    def _build(self):
        trace = self._trace
        seen = (len(trace.costs), len(trace.checks), sum(len(path) for path in trace.paths), len(trace.proposed),
                trace.cells is not None)
        if seen == self._seen:
            return
        self._seen = seen
        scale = 1.0 / doc_unit_scale_to_mm()

        def at(frame):
            lifted = (frame[:3, 3] + _LIFT_MM * frame[:3, 2]) * scale
            return Rhino.Geometry.Point3d(float(lifted[0]), float(lifted[1]), float(lifted[2]))

        def vector(xyz):
            return Rhino.Geometry.Vector3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))

        box = Rhino.Geometry.BoundingBox.Empty
        values = [cost for _, cost in trace.costs] + ([] if trace.cells is None else list(trace.cells[1]))
        low, high = (min(values), max(values)) if values else (0.0, 0.0)

        def shade(cost):
            cheap = 1.0 - (cost - low) / max(high - low, 1e-12)
            return Color.FromArgb(*[int(d + (c - d) * cheap) for d, c in zip(_COLOR_DEAR, _COLOR_CHEAP)])

        # The grid is many arrows: a few shades, each drawn in one call.
        by_shade = {}
        for frame, cost in zip(*(trace.cells or ((), ()))):
            cheap = 1.0 - (float(cost) - low) / max(high - low, 1e-12)
            start = at(frame)
            by_shade.setdefault(round(cheap * _SHADES), []).append(
                Rhino.Geometry.Line(start, start + vector(frame[:3, 0]) * (_CELL_ARROW_MM * scale)))
            box.Union(start)
        self._cells = [(System.Array[Rhino.Geometry.Line](lines), shade(low + (1.0 - step / _SHADES) * (high - low)))
                       for step, lines in by_shade.items()]
        self._arrows = []
        for frame, cost in trace.costs:
            start = at(frame)
            self._arrows.append((Rhino.Geometry.Line(start, start + vector(frame[:3, 0]) * (_ARROW_MM * scale)),
                                 shade(cost)))
            box.Union(start)
        self._circles = []
        for frame, collides in trace.checks:
            plane = Rhino.Geometry.Plane(at(frame), vector(frame[:3, 2]))
            self._circles.append((Rhino.Geometry.Circle(plane, _CIRCLE_MM * scale),
                                  Color.FromArgb(*(_COLOR_COLLIDES if collides else _COLOR_FREE))))
            box.Union(plane.Origin)
        self._lines = [(at(a), at(b)) for path in trace.paths for a, b in zip(path, path[1:])]
        self._dots = [at(frame) for frame in trace.proposed]
        self._box = box

    def CalculateBoundingBox(self, e):
        if self._box.IsValid:
            e.IncludeBoundingBox(self._box)

    def PostDrawObjects(self, e):
        try:
            self._build()
            for lines, color in self._cells:
                e.Display.DrawArrows(lines, color)
            for circle, color in self._circles:
                e.Display.DrawCircle(circle, color, 2)
            for start, end in self._lines:
                e.Display.DrawLine(start, end, Color.FromArgb(*_COLOR_PATH), 2)
            for line, color in self._arrows:
                e.Display.DrawArrow(line, color)
            for point in self._dots:
                e.Display.DrawPoint(point, Rhino.Display.PointStyle.RoundControlPoint, 10,
                                    Color.FromArgb(*_COLOR_CHEAP))
        except Exception as exc:  # noqa: BLE001 -- runs on every redraw: fail once, then stay out of the way
            self.Enabled = False
            session_log.caught(exc)
            print(f"RSIKKeyframe (debug): base search trace drawing failed and was switched off "
                  f"({type(exc).__name__}: {exc}).")


def hide():
    """Switch the trace off (the end of the command, or a new search); never raises."""
    try:
        conduit = sc.sticky.pop(_STICKY_CONDUIT, None)
        if conduit is not None:
            conduit.Enabled = False
            sc.doc.Views.Redraw()
    except Exception as exc:  # noqa: BLE001 -- a debug view must never break the command
        session_log.caught(exc)


def show(trace):
    """Draw `trace` (a `base_search.Trace`) from now on, as the search fills it; never raises."""
    hide()
    try:
        if getattr(config, "DEBUG_BASE_SEARCH_TRACE", False):
            conduit = _TraceConduit(trace)
            conduit.Enabled = True
            sc.sticky[_STICKY_CONDUIT] = conduit
    except Exception as exc:  # noqa: BLE001 -- a debug view must never break the command
        session_log.caught(exc)
        print(f"RSIKKeyframe (debug): base search trace unavailable ({type(exc).__name__}: {exc}).")
