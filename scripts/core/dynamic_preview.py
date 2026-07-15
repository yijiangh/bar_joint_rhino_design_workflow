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


class MeshPreviewConduit(Rhino.Display.DisplayConduit):
    """DisplayConduit drawing a fixed list of meshes under a mutable world
    transform with a translucent material.

    `alpha` is opacity: 1.0 = opaque, 0.0 = invisible. Maps to Rhino's
    DisplayMaterial.Transparency = 1 - alpha.
    """

    def __init__(self, meshes_at_identity, color=None, alpha=0.5):
        super().__init__()
        self._meshes = list(meshes_at_identity)
        self._xform = Rhino.Geometry.Transform.Identity
        if color is None:
            color = _color_from_rgb(180, 180, 220)
        self._material = Rhino.Display.DisplayMaterial(color)
        self._material.Transparency = max(0.0, min(1.0, 1.0 - float(alpha)))

    def update_xform(self, xform):
        self._xform = xform if xform is not None else Rhino.Geometry.Transform.Identity
        sc.doc.Views.Redraw()

    def CalculateBoundingBox(self, e):
        for m in self._meshes:
            bb = m.GetBoundingBox(self._xform)
            if bb.IsValid:
                e.IncludeBoundingBox(bb)

    def PostDrawObjects(self, e):
        e.Display.PushModelTransform(self._xform)
        try:
            for m in self._meshes:
                e.Display.DrawMeshShaded(m, self._material)
        finally:
            e.Display.PopModelTransform()


@contextlib.contextmanager
def mesh_preview(meshes_at_identity, *, color=None, alpha=0.5):
    """Enable a MeshPreviewConduit for the duration of the with-block."""
    conduit = MeshPreviewConduit(meshes_at_identity, color=color, alpha=alpha)
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
    ):
        self._seed_origin_doc = origin_doc
        self._seed_x_axis_doc = x_axis_doc
        self._seed_arrow_len_doc = float(arrow_len_doc)
        if radius_doc and float(radius_doc) > 0 and normal_doc is not None:
            plane = Rhino.Geometry.Plane(origin_doc, normal_doc)
            self._circle_doc = Rhino.Geometry.Circle(plane, float(radius_doc))
        else:
            self._circle_doc = None
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

        if self._circle_doc is not None:
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
