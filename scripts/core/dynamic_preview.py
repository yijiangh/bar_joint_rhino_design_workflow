"""Half-transparent cursor-following mesh preview for interactive picking.

Pattern (like Rhino Orient3Pt): construct a MeshPreviewConduit with a list
of Rhino meshes baked at IDENTITY, enable it for the duration of one or
more rs.GetPoint / Rhino.Input.Custom.GetPoint calls, and update its
transform each tick via OnDynamicDraw on a GetPoint subclass.

Depends on Rhino + scriptcontext + System.Drawing; only importable inside
Rhino 8 ScriptEditor (not from standalone tests).
"""

from __future__ import annotations

import contextlib
import math

import Rhino
import scriptcontext as sc
from System.Drawing import Color as _SDColor


def _color_from_rgb(r, g, b):
    return _SDColor.FromArgb(int(r), int(g), int(b))


def _force_redraw():
    """Invalidate the viewport AND pump Rhino's message queue so the redraw
    actually paints now.

    ``sc.doc.Views.Redraw()`` only marks the views dirty; the repaint happens
    when Rhino's message loop next runs. That is fine for interactive callbacks
    (GetPoint pumps every tick), but a conduit driven from a long, synchronous
    loop -- e.g. the IK base-frame sampling in ``rs_ik_keyframe`` -- never yields
    to the message loop, so nothing paints until the loop returns. ``RhinoApp.Wait``
    pumps the queued messages, letting the invalidated views repaint mid-loop.
    """
    sc.doc.Views.Redraw()
    Rhino.RhinoApp.Wait()


# ---------------------------------------------------------------------------
# Reach-circle outline (radial-clamp deformation vs obstacles / ground edge)
# ---------------------------------------------------------------------------

# Shared reach-outline colors: clear (no contact) vs touching an obstacle or the
# walkable-ground edge.
_REACH_COLOR_CLEAR = _color_from_rgb(100, 100, 220)
_REACH_COLOR_TOUCH = _color_from_rgb(255, 100, 100)


def _ray_segment_t(cu, cv, dx, dy, ax, ay, bx, by):
    """Parametric distance ``t`` along the ray (origin (cu,cv), unit dir (dx,dy))
    to its crossing with segment A(ax,ay)->B(bx,by), or ``None`` if it misses.

    Pure 2D float math (no RhinoCommon) so it is cheap enough to run per spoke on
    every mouse-move.
    """
    ex = bx - ax
    ey = by - ay
    det = ex * dy - dx * ey
    if -1e-12 < det < 1e-12:
        return None  # parallel
    aox = ax - cu
    aoy = ay - cv
    t = (-aox * ey + ex * aoy) / det   # distance along the ray
    s = (dx * aoy - aox * dy) / det    # position along the segment [0,1]
    if t < 0.0 or s < 0.0 or s > 1.0:
        return None
    return t


def _ray_polygons_min_dist(cu, cv, dx, dy, maxlen, polygons):
    """Nearest crossing distance (<= ``maxlen``) of the ray with any polygon edge.

    Returns ``(dist, hit)``. Each polygon is a list of (u, v) points in plane
    coords; edges wrap (last -> first).
    """
    best = maxlen
    hit = False
    for poly in polygons:
        m = len(poly)
        if m < 2:
            continue
        for i in range(m):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % m]
            t = _ray_segment_t(cu, cv, dx, dy, ax, ay, bx, by)
            if t is not None and 0.0 < t < best:
                best = t
                hit = True
    return best, hit


def compute_reach_outline(center_doc, normal_doc, radius_doc, clip_curves, tol, segments=90):
    """Return ``(outline_curve, overlaps)`` for the base reach circle, clamped
    inward where it would enter an obstacle footprint or leave the walkable edge.

    Radial-clamp method done in the ground plane's 2D coordinates: cast
    ``segments`` spokes from the center; each spoke length starts at ``radius_doc``
    and is shortened to the nearest crossing with an obstacle footprint polygon or
    the walkable boundary polygon (all precomputed 2D polygons in ``clip_curves``).
    Every intersection is pure float math -- no per-frame RhinoCommon curve
    intersection -- so it stays smooth when recomputed on every mouse-move.

    Args:
        center_doc (Rhino.Geometry.Point3d): circle center (base origin), doc units.
        normal_doc (Rhino.Geometry.Vector3d): plane normal, used only when
            ``clip_curves`` carries no precomputed plane (plain-circle fast path).
        radius_doc (float): full reach radius, doc units.
        clip_curves (dict | None): ``{"plane": Plane, "boundary": [(u,v),...] | None,
            "obstacles": [[(u,v),...], ...]}`` precomputed by the caller in ONE
            fixed plane. Empty -> plain circle.
        tol (float): absolute tolerance, used for the overlap epsilon.
        segments (int): number of spokes.

    Returns:
        (Rhino.Geometry.Curve | None, bool): outline + whether any spoke clamped.
    """
    if radius_doc is None or float(radius_doc) <= 0.0:
        return None, False
    radius_doc = float(radius_doc)

    clip = clip_curves or {}
    plane = clip.get("plane")
    boundary = clip.get("boundary")
    obstacles = clip.get("obstacles") or []
    if plane is None:
        if normal_doc is None:
            return None, False
        plane = Rhino.Geometry.Plane(center_doc, normal_doc)

    # Fast path: nothing to clip against -> the exact circle.
    if not boundary and not obstacles:
        return Rhino.Geometry.Circle(plane, radius_doc).ToNurbsCurve(), False

    polygons = []
    if boundary:
        polygons.append(boundary)
    polygons.extend(obstacles)

    ok, center_plane = plane.RemapToPlaneSpace(center_doc)
    if not ok:
        return Rhino.Geometry.Circle(plane, radius_doc).ToNurbsCurve(), False
    cu = center_plane.X
    cv = center_plane.Y

    eps = max(float(tol), radius_doc * 1e-4)
    n = max(8, int(segments))
    overlaps = False
    poly = Rhino.Geometry.Polyline()
    first_pt = None
    for i in range(n):
        ang = (2.0 * math.pi * i) / n
        dx = math.cos(ang)
        dy = math.sin(ang)
        dist, hit = _ray_polygons_min_dist(cu, cv, dx, dy, radius_doc, polygons)
        r = dist if hit else radius_doc
        if r < radius_doc - eps:
            overlaps = True
        pt = plane.PointAt(cu + dx * r, cv + dy * r)
        if first_pt is None:
            first_pt = pt
        poly.Add(pt)
    if first_pt is not None:
        poly.Add(first_pt)  # close the loop
    return poly.ToNurbsCurve(), overlaps


def _draw_reach(e, curve, overlaps, color_clear=None, color_touch=None):
    """Draw the reach outline in the touch color when it overlaps, else clear."""
    if curve is None:
        return
    color = (color_touch or _REACH_COLOR_TOUCH) if overlaps else (color_clear or _REACH_COLOR_CLEAR)
    e.Display.DrawCurve(curve, color, 2)


class MeshPreviewConduit(Rhino.Display.DisplayConduit):
    """DisplayConduit drawing a fixed list of meshes under a mutable world
    transform with a translucent material.

    `alpha` is opacity: 1.0 = opaque, 0.0 = invisible. Maps to Rhino's
    DisplayMaterial.Transparency = 1 - alpha.
    """

    def __init__(self, meshes_at_identity, color=None, alpha=0.5,
                 extra_meshes=None, extra_color=None, extra_alpha=None):
        super().__init__()
        self._meshes = list(meshes_at_identity)
        self._xform = Rhino.Geometry.Transform.Identity
        if color is None:
            color = _color_from_rgb(180, 180, 220)
        self._material = Rhino.Display.DisplayMaterial(color)
        self._material.Transparency = max(0.0, min(1.0, 1.0 - float(alpha)))
        # Optional SECOND mesh set drawn under the same model transform but with
        # its own material -- used for the arm reach-volume spheres, which must
        # follow the ghost robot but be far more transparent than it. None ->
        # nothing extra drawn, so other callers are unaffected.
        self._extra_meshes = list(extra_meshes) if extra_meshes else []
        self._extra_material = None
        if self._extra_meshes:
            if extra_color is None:
                extra_color = _color_from_rgb(120, 200, 255)
            self._extra_material = Rhino.Display.DisplayMaterial(extra_color)
            # Faint enough to read as a volume you can see the robot through, but
            # not so faint it disappears against a light viewport. Tunable.
            extra_alpha = 0.20 if extra_alpha is None else extra_alpha
            self._extra_material.Transparency = max(0.0, min(1.0, 1.0 - float(extra_alpha)))
        # Optional reach-circle outline drawn on top of the ghost meshes (used by
        # the RSIKKeyframe base pick). None -> nothing drawn, so other callers of
        # MeshPreviewConduit are unaffected.
        self._reach_curve = None
        self._reach_overlaps = False

    def set_reach_outline(self, curve, overlaps):
        """Set the reach outline to draw. No redraw here -- the paired
        ``update_xform`` call on the same tick triggers the single redraw."""
        self._reach_curve = curve
        self._reach_overlaps = bool(overlaps)

    def update_xform(self, xform):
        self._xform = xform if xform is not None else Rhino.Geometry.Transform.Identity
        sc.doc.Views.Redraw()

    def CalculateBoundingBox(self, e):
        for m in list(self._meshes) + list(self._extra_meshes):
            bb = m.GetBoundingBox(self._xform)
            if bb.IsValid:
                e.IncludeBoundingBox(bb)
        if self._reach_curve is not None:
            bb = self._reach_curve.GetBoundingBox(False)
            if bb.IsValid:
                e.IncludeBoundingBox(bb)

    def PostDrawObjects(self, e):
        e.Display.PushModelTransform(self._xform)
        try:
            for m in self._meshes:
                e.Display.DrawMeshShaded(m, self._material)
            # Reach volumes last so they blend OVER the robot they surround.
            for m in self._extra_meshes:
                e.Display.DrawMeshShaded(m, self._extra_material)
        finally:
            e.Display.PopModelTransform()
        # Reach outline is in doc/world coords -> draw AFTER popping the model
        # transform so it does not follow the ghost-robot xform.
        _draw_reach(e, self._reach_curve, self._reach_overlaps)


@contextlib.contextmanager
def mesh_preview(meshes_at_identity, *, color=None, alpha=0.5,
                 extra_meshes=None, extra_color=None, extra_alpha=None):
    """Enable a MeshPreviewConduit for the duration of the with-block.

    ``extra_meshes`` / ``extra_color`` / ``extra_alpha`` are forwarded to
    :class:`MeshPreviewConduit` -- a second mesh set moving with the same
    transform but drawn with its own (typically far more transparent) material.
    """
    conduit = MeshPreviewConduit(
        meshes_at_identity, color=color, alpha=alpha,
        extra_meshes=extra_meshes, extra_color=extra_color, extra_alpha=extra_alpha,
    )
    conduit.Enabled = True
    try:
        yield conduit
    finally:
        conduit.Enabled = False
        sc.doc.Views.Redraw()


class TrackingGetPoint(Rhino.Input.Custom.GetPoint):
    """GetPoint subclass that recomputes a candidate world transform from
    the current cursor and pushes it into the supplied conduit on every
    OnDynamicDraw tick.

    `frame_from_cursor(cursor_point) -> Rhino.Geometry.Transform | None`.
    Returning None means: don't update the conduit (keep last good xform).
    """

    def __init__(self, conduit, frame_from_cursor):
        super().__init__()
        self._conduit = conduit
        self._frame_from_cursor = frame_from_cursor

    def OnDynamicDraw(self, e):
        try:
            xform = self._frame_from_cursor(e.CurrentPoint)
        except Exception as exc:
            print(f"TrackingGetPoint: frame_from_cursor raised {exc}; keeping last xform.")
            xform = None
        if xform is not None:
            self._conduit.update_xform(xform)
        super().OnDynamicDraw(e)


class IKSampleVizConduit(Rhino.Display.DisplayConduit):
    """Conduit for visualizing IK base-frame sampling.

    Renders, in doc units:

    * Seed point + X-axis arrow at the user-picked base frame (bright color).
    * Sampling circle in the seed's tangent plane (light blue).
    * Translucent ghost robot meshes at the current attempt's base frame
      (set via ``set_ghost_xform``).
    * Past attempt markers (point + X-axis arrow) in red for failed samples,
      green for the succeeded sample; seed attempt is omitted because the
      seed marker already covers it.

    All state is mutated by callers; ``sc.doc.Views.Redraw()`` is invoked on
    setter calls so the display tracks the IK loop in real time.
    """

    def __init__(self, robot_meshes, *, alpha=0.35):
        super().__init__()
        self._robot_meshes = list(robot_meshes)
        self._robot_xform = None
        self._robot_material = Rhino.Display.DisplayMaterial(_color_from_rgb(180, 180, 220))
        self._robot_material.Transparency = max(0.0, min(1.0, 1.0 - float(alpha)))

        self._seed_origin_doc = None
        self._seed_x_axis_doc = None
        self._seed_arrow_len_doc = 1.0
        self._circle_doc = None
        # Deformed (obstacle-clipped) reach outline; when set it is drawn instead
        # of the plain _circle_doc.
        self._reach_curve = None
        self._reach_overlaps = False
        # tried = [(Point3d origin_doc, Vector3d x_axis_doc, bool success), ...]
        self._tried = []

        self._color_seed = _color_from_rgb(80, 200, 255)
        self._color_circle = _color_from_rgb(100, 100, 220)
        self._color_failed = _color_from_rgb(255, 100, 100)
        self._color_success = _color_from_rgb(100, 220, 100)

    def set_seed_and_circle(
        self,
        origin_doc,
        x_axis_doc,
        normal_doc,
        radius_doc,
        arrow_len_doc,
        clip_curves=None,
        tol=None,
    ):
        self._seed_origin_doc = origin_doc
        self._seed_x_axis_doc = x_axis_doc
        self._seed_arrow_len_doc = float(arrow_len_doc)
        self._circle_doc = None
        self._reach_curve = None
        self._reach_overlaps = False
        if radius_doc and float(radius_doc) > 0 and normal_doc is not None:
            if clip_curves is not None:
                # Deformed outline clipped against obstacles / walkable edge.
                self._reach_curve, self._reach_overlaps = compute_reach_outline(
                    origin_doc,
                    normal_doc,
                    float(radius_doc),
                    clip_curves,
                    tol if tol is not None else sc.doc.ModelAbsoluteTolerance,
                )
            else:
                plane = Rhino.Geometry.Plane(origin_doc, normal_doc)
                self._circle_doc = Rhino.Geometry.Circle(plane, float(radius_doc))
        # Pump the message queue: this conduit is driven from the blocking IK
        # base-sampling loop, so a plain Redraw() would not paint until the loop
        # ends (see _force_redraw).
        _force_redraw()

    def set_ghost_xform(self, xform):
        self._robot_xform = xform
        _force_redraw()

    def add_tried(self, origin_doc, x_axis_doc, success):
        self._tried.append((origin_doc, x_axis_doc, bool(success)))
        _force_redraw()

    def CalculateBoundingBox(self, e):
        if self._robot_xform is not None:
            for m in self._robot_meshes:
                bb = m.GetBoundingBox(self._robot_xform)
                if bb.IsValid:
                    e.IncludeBoundingBox(bb)
        if self._circle_doc is not None:
            e.IncludeBoundingBox(self._circle_doc.BoundingBox)
        if self._reach_curve is not None:
            bb = self._reach_curve.GetBoundingBox(False)
            if bb.IsValid:
                e.IncludeBoundingBox(bb)
        if self._seed_origin_doc is not None:
            e.IncludeBoundingBox(Rhino.Geometry.BoundingBox(self._seed_origin_doc, self._seed_origin_doc))
        for origin, _x, _s in self._tried:
            e.IncludeBoundingBox(Rhino.Geometry.BoundingBox(origin, origin))

    def _draw_arrow(self, e, origin, axis, length, color):
        if axis is None or length <= 0.0:
            return
        tip = Rhino.Geometry.Point3d(
            origin.X + axis.X * length,
            origin.Y + axis.Y * length,
            origin.Z + axis.Z * length,
        )
        e.Display.DrawArrow(Rhino.Geometry.Line(origin, tip), color)

    def PostDrawObjects(self, e):
        if self._robot_xform is not None and self._robot_meshes:
            e.Display.PushModelTransform(self._robot_xform)
            try:
                for m in self._robot_meshes:
                    e.Display.DrawMeshShaded(m, self._robot_material)
            finally:
                e.Display.PopModelTransform()

        if self._reach_curve is not None:
            _draw_reach(e, self._reach_curve, self._reach_overlaps, self._color_circle)
        elif self._circle_doc is not None:
            e.Display.DrawCircle(self._circle_doc, self._color_circle, 2)

        sample_arrow_len = self._seed_arrow_len_doc * 0.6
        for origin, x_axis, success in self._tried:
            color = self._color_success if success else self._color_failed
            e.Display.DrawPoint(origin, Rhino.Display.PointStyle.RoundSimple, 6, color)
            self._draw_arrow(e, origin, x_axis, sample_arrow_len, color)

        if self._seed_origin_doc is not None:
            e.Display.DrawPoint(
                self._seed_origin_doc,
                Rhino.Display.PointStyle.RoundControlPoint,
                10,
                self._color_seed,
            )
            self._draw_arrow(
                e,
                self._seed_origin_doc,
                self._seed_x_axis_doc,
                self._seed_arrow_len_doc,
                self._color_seed,
            )


@contextlib.contextmanager
def ik_sample_viz(robot_meshes, *, alpha=0.35):
    """Enable an IKSampleVizConduit for the duration of the with-block."""
    conduit = IKSampleVizConduit(robot_meshes, alpha=alpha)
    conduit.Enabled = True
    try:
        yield conduit
    finally:
        conduit.Enabled = False
        sc.doc.Views.Redraw()


class ReachCircleConduit(Rhino.Display.DisplayConduit):
    """Draws only the (optionally obstacle-clipped) reach outline -- no meshes.

    Used for the RSIKKeyframe saved-base reuse preview, where the robot is
    already shown via ``ik_viz`` and only the reach circle needs a conduit.
    """

    def __init__(self):
        super().__init__()
        self._reach_curve = None
        self._reach_overlaps = False

    def set_reach_outline(self, curve, overlaps):
        self._reach_curve = curve
        self._reach_overlaps = bool(overlaps)
        _force_redraw()

    def CalculateBoundingBox(self, e):
        if self._reach_curve is not None:
            bb = self._reach_curve.GetBoundingBox(False)
            if bb.IsValid:
                e.IncludeBoundingBox(bb)

    def PostDrawObjects(self, e):
        _draw_reach(e, self._reach_curve, self._reach_overlaps)


@contextlib.contextmanager
def reach_circle_viz():
    """Enable a ReachCircleConduit for the duration of the with-block."""
    conduit = ReachCircleConduit()
    conduit.Enabled = True
    try:
        yield conduit
    finally:
        conduit.Enabled = False
        sc.doc.Views.Redraw()


class MarkerConduit(Rhino.Display.DisplayConduit):
    """Labelled point + line markers in doc units -- "here is what will move".

    Callers replace the whole marker set at once with :meth:`set_markers`;
    there is no incremental add, because the markers track a state that is
    recomputed wholesale on every repick anyway.

    A marker is ``(Point3d, label, (r, g, b))`` -- a dot with the label beside
    it.  A line is ``(Point3d, Point3d, (r, g, b))``.
    """

    def __init__(self):
        super().__init__()
        self._markers = []
        self._lines = []

    def set_markers(self, markers=None, lines=None):
        self._markers = list(markers or [])
        self._lines = list(lines or [])
        _force_redraw()

    def CalculateBoundingBox(self, e):
        for point, _label, _rgb in self._markers:
            e.IncludeBoundingBox(Rhino.Geometry.BoundingBox(point, point))
        for start, end, _rgb in self._lines:
            e.IncludeBoundingBox(Rhino.Geometry.BoundingBox(start, end))

    def PostDrawObjects(self, e):
        for start, end, rgb in self._lines:
            e.Display.DrawLine(start, end, _color_from_rgb(*rgb), 2)
        for point, label, rgb in self._markers:
            color = _color_from_rgb(*rgb)
            e.Display.DrawPoint(
                point, Rhino.Display.PointStyle.RoundControlPoint, 9, color
            )
            if label:
                e.Display.DrawDot(point, label, color, _color_from_rgb(255, 255, 255))


# Last MarkerConduit handed out, so a conduit stranded by a hard script abort
# (which skips the with-block's finally) gets switched off the next time a
# command opens one.  Esc, an early return and an exception are all already
# handled by that finally; this only covers the case Python never gets to run.
_ACTIVE_MARKER_CONDUIT = None


@contextlib.contextmanager
def marker_viz():
    """Enable a MarkerConduit for the duration of the with-block."""
    global _ACTIVE_MARKER_CONDUIT
    if _ACTIVE_MARKER_CONDUIT is not None:
        _ACTIVE_MARKER_CONDUIT.Enabled = False
    conduit = MarkerConduit()
    conduit.Enabled = True
    _ACTIVE_MARKER_CONDUIT = conduit
    try:
        yield conduit
    finally:
        conduit.Enabled = False
        _ACTIVE_MARKER_CONDUIT = None
        sc.doc.Views.Redraw()


def block_definition_meshes(block_name):
    """Return a list of Rhino meshes (at the block's local frame) for all
    mesh-bearing geometry in the named block definition.

    Curves and other non-mesh items are skipped. Breps and Extrusions are
    meshed with default MeshingParameters. Used to feed mesh_preview()
    with a Robotiq-gripper or tool block geometry without baking an
    instance into the doc.
    """
    idef = sc.doc.InstanceDefinitions.Find(block_name, True)
    if idef is None:
        raise RuntimeError(f"Block definition '{block_name}' not found in document.")
    meshes = []
    mp = Rhino.Geometry.MeshingParameters.Default
    for obj in idef.GetObjects():
        geom = obj.Geometry
        if isinstance(geom, Rhino.Geometry.Mesh):
            meshes.append(geom.DuplicateMesh())
        elif isinstance(geom, Rhino.Geometry.Brep):
            for m in Rhino.Geometry.Mesh.CreateFromBrep(geom, mp) or []:
                meshes.append(m)
        elif isinstance(geom, Rhino.Geometry.Extrusion):
            brep = geom.ToBrep(False)
            if brep is not None:
                for m in Rhino.Geometry.Mesh.CreateFromBrep(brep, mp) or []:
                    meshes.append(m)
    return meshes
