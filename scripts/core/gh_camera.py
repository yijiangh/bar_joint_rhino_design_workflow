"""Logic behind the ``RSGHCameraControl`` Grasshopper component.

Drives the Rhino viewport camera from Grasshopper geometry, so a camera point
swept along a curve (with the same slider that drives
:mod:`core.gh_seq_preview`) produces a moving shot of the assembly sequence.

Before this, the repo had no camera control at all -- the only camera-adjacent
call anywhere was ``rs.ZoomSelected()`` in ``rs_select_bar.py``.

Units
-----
``camera`` / ``target`` are in **document units**, passed straight through.  The
repo's millimetre convention applies to robot and IK data (base frames, joint
payloads), not to viewport geometry: the viewport already speaks the document's
units, so converting here would move the camera to the wrong place.

The one API trap
----------------
``ViewportInfo.SetCameraLocations`` takes ``(target, location)`` -- **target
first**.  Reversed, the camera ends up inside the model looking out.  It also
resets ``CameraUp``, so the up vector has to be applied *after* it.
"""

from __future__ import annotations

import Rhino
import scriptcontext as sc

from core import gh_bridge


# Below this, the up vector is treated as parallel to the view direction.  Rhino
# silently ignores a degenerate up rather than raising, so the component checks
# it and says so instead of leaving the user with an unexplained roll.
_PARALLEL_TOL = 1e-6


def _as_point(value):
    """Coerce a GH point-ish input to ``Rhino.Geometry.Point3d``, or None."""
    if value is None:
        return None
    if isinstance(value, Rhino.Geometry.Point3d):
        return value
    if isinstance(value, Rhino.Geometry.Point):
        return value.Location
    try:
        return Rhino.Geometry.Point3d(float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return None


def _as_vector(value):
    """Coerce a GH vector-ish input to ``Rhino.Geometry.Vector3d``, or None."""
    if value is None:
        return None
    if isinstance(value, Rhino.Geometry.Vector3d):
        return value
    if isinstance(value, Rhino.Geometry.Point3d):
        return Rhino.Geometry.Vector3d(value)
    try:
        return Rhino.Geometry.Vector3d(float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return None


def _find_view(viewport):
    """Return ``(view, note)`` for the named viewport, falling back to the active one."""
    if not viewport:
        return sc.doc.Views.ActiveView, None
    view = sc.doc.Views.Find(str(viewport), False)
    if view is None:
        names = [v.ActiveViewport.Name for v in sc.doc.Views]
        return sc.doc.Views.ActiveView, (
            f"no viewport named {viewport!r} (have: {', '.join(names)}); used the active view"
        )
    return view, None


def run(camera=None, target=None, lens=50.0, up=None, active=False, viewport=None):
    """Point a Rhino viewport's camera at *target* from *camera*.

    Args:
        camera (Point3d): camera position, document units.
        target (Point3d): what the camera looks at, document units.
        lens (float): 35mm-equivalent focal length; 50 is the Rhino default.
        up (Vector3d): camera up; defaults to world Z.
        active (bool): False makes the whole thing a no-op, so the component can
            sit on the canvas without hijacking the viewport.
        viewport (str | None): a named viewport; ``None`` uses the active view.

    Returns:
        dict: ``{"ok": bool, "info": str}``.
    """
    if not active:
        return {"ok": False, "info": "inactive"}

    cam_pt = _as_point(camera)
    tgt_pt = _as_point(target)
    if cam_pt is None or tgt_pt is None:
        return {"ok": False, "info": "camera and target are both required (Point3d)"}
    if cam_pt.DistanceTo(tgt_pt) < Rhino.RhinoMath.ZeroTolerance:
        return {"ok": False, "info": "camera == target; nothing to look at"}

    up_vec = _as_vector(up) or Rhino.Geometry.Vector3d(0.0, 0.0, 1.0)
    try:
        lens_mm = float(lens) if lens else 50.0
    except (TypeError, ValueError):
        lens_mm = 50.0
    if lens_mm <= 0.0:
        return {"ok": False, "info": f"lens must be positive, got {lens!r}"}

    notes = []

    with gh_bridge.rhino_doc():
        view, view_note = _find_view(viewport)
        if view_note:
            notes.append(view_note)
        if view is None:
            return {"ok": False, "info": "no Rhino view to drive"}
        vp = view.ActiveViewport

        # A parallel (Top / Front / ...) viewport has no meaningful camera lens;
        # switch it once rather than silently doing nothing.
        if not vp.IsPerspectiveProjection:
            vp.ChangeToPerspectiveProjection(True, lens_mm)
            notes.append("switched viewport to perspective")

        # (target, location) -- target first.  Reversed, the camera lands inside
        # the model looking outward.
        vp.SetCameraLocations(tgt_pt, cam_pt)

        # Only after SetCameraLocations, which resets CameraUp.
        direction = Rhino.Geometry.Vector3d(tgt_pt - cam_pt)
        direction.Unitize()
        test_up = Rhino.Geometry.Vector3d(up_vec)
        if not test_up.Unitize():
            notes.append("up vector is zero-length; kept Rhino's default up")
        elif Rhino.Geometry.Vector3d.CrossProduct(direction, test_up).Length < _PARALLEL_TOL:
            # Rhino ignores a parallel up without complaining, which reads as
            # "my up input does nothing" -- so say it out loud.
            notes.append("up is parallel to the view direction; ignored by Rhino")
        else:
            vp.CameraUp = up_vec

        vp.Camera35mmLensLength = lens_mm
        view.Redraw()

    info = (
        f"{vp.Name}: camera {cam_pt} -> target {tgt_pt}, {lens_mm:g}mm"
    )
    if notes:
        info += " | " + " | ".join(notes)
    return {"ok": True, "info": info}
