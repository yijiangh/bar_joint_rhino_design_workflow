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


def block_definition_meshes(block_name):
    """Return a list of Rhino meshes (at the block's local frame) for all
    mesh-bearing geometry in the named block definition.

    Curves and other non-mesh items are skipped. Breps and Extrusions are
    meshed with default MeshingParameters. Used to feed mesh_preview()
    with a Robotiq-gripper or pineapple block geometry without baking an
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
