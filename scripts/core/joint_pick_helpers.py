"""Shared Rhino-side pick + frame helpers for the joint half/mate commands.

Extracted from the legacy `rs_define_joint_pair.py` so both
`rs_define_joint_half.py` and `rs_define_joint_mate.py` can import them
without duplicating code.

These helpers depend on Rhino runtime modules (`Rhino`, `rhinoscriptsyntax`,
`scriptcontext`) and so cannot be imported from a non-Rhino Python process.
"""

from __future__ import annotations

import contextlib

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core.rhino_helpers import point_to_array


# ---------------------------------------------------------------------------
# Doc-unit scaling
# ---------------------------------------------------------------------------


def doc_unit_scale_to_mm() -> float:
    return float(
        Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters)
    )


def frame_to_mm(matrix: np.ndarray, scale_to_mm: float) -> np.ndarray:
    out = np.array(matrix, dtype=float, copy=True)
    out[:3, 3] *= scale_to_mm
    return out


def vec_to_mm(vector: np.ndarray, scale_to_mm: float) -> np.ndarray:
    return np.asarray(vector, dtype=float) * scale_to_mm


# ---------------------------------------------------------------------------
# Hide / show helpers
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def temporarily_hidden(object_ids):
    """Hide given object ids on entry; restore visibility on exit."""

    hidden: list = []
    try:
        for oid in object_ids:
            if oid is not None and rs.IsObject(oid):
                if rs.HideObject(oid):
                    hidden.append(oid)
        yield
    finally:
        for oid in hidden:
            if rs.IsObject(oid):
                rs.ShowObject(oid)


# ---------------------------------------------------------------------------
# Pickers
# ---------------------------------------------------------------------------


def pick_block_instance(prompt: str, dialog_title: str = "RSDefineJoint"):
    return rs.GetObject(prompt, filter=4096, preselect=False, select=False)


def pick_line(prompt: str, dialog_title: str = "RSDefineJoint"):
    object_id = rs.GetObject(prompt, filter=4, preselect=False, select=False)
    if object_id is None:
        return None
    curve = rs.coercecurve(object_id)
    if curve is None or not curve.IsLinear():
        rs.MessageBox(
            "Selected curve must be a single straight line segment.", 0, dialog_title
        )
        return None
    return object_id


def pick_point(prompt: str):
    return rs.GetObject(prompt, filter=1, preselect=False, select=False)


def pick_meshes(prompt: str):
    """Pick one or more mesh objects (or curves we'll skip).  Returns a list of guids or None."""
    return rs.GetObjects(prompt, filter=32, preselect=False, select=False, minimum_count=1)


# ---------------------------------------------------------------------------
# Block instance -> 4x4 numpy frame
# ---------------------------------------------------------------------------


def block_instance_frame(block_instance_id) -> tuple[np.ndarray, str]:
    rh_obj = sc.doc.Objects.FindId(block_instance_id)
    if rh_obj is None or not isinstance(rh_obj, Rhino.DocObjects.InstanceObject):
        raise ValueError("Selected object is not a block instance.")
    xform = rh_obj.InstanceXform
    matrix = np.array(
        [[xform[r, c] for c in range(4)] for r in range(4)],
        dtype=float,
    )
    instance_def = rh_obj.InstanceDefinition
    block_name = instance_def.Name if instance_def is not None else ""
    return matrix, block_name


def line_endpoints(line_id) -> tuple[np.ndarray, np.ndarray]:
    curve = rs.coercecurve(line_id)
    return point_to_array(curve.PointAtStart), point_to_array(curve.PointAtEnd)


def point_xyz(point_id) -> np.ndarray:
    point_obj = sc.doc.Objects.FindId(point_id)
    if point_obj is None:
        raise ValueError("Failed to resolve selected point.")
    return point_to_array(point_obj.Geometry.Location)


# ---------------------------------------------------------------------------
# Frame computation (block-X-anchored bar/screw frames; world-rotation invariant)
# ---------------------------------------------------------------------------


def x_anchored_to_block_factory(block_frame_mm: np.ndarray):
    """Return a callable f(z_axis) -> unit X axis perpendicular to z_axis,
    anchored to the block's local X (or Y if X is parallel to Z).

    This makes the resulting bar/screw frames invariant under any rigid
    rotation of the whole rig in world space (block + bar + screw rotated
    together). Without this, the X axis would be derived via
    `orthogonal_to(z)`, which snaps to a world reference axis and is
    therefore NOT equivariant under world rotation.
    """
    from core.transforms import orthogonal_to, unit

    block_x_world = unit(block_frame_mm[:3, 0])
    block_y_world = unit(block_frame_mm[:3, 1])

    def _x_for(z_axis: np.ndarray) -> np.ndarray:
        for ref in (block_x_world, block_y_world):
            x_proj = ref - float(np.dot(ref, z_axis)) * z_axis
            n = float(np.linalg.norm(x_proj))
            if n > 1e-9:
                return x_proj / n
        return orthogonal_to(z_axis)

    return _x_for


def compute_M_block_from_bar(
    block_frame_mm: np.ndarray,
    bar_start_mm: np.ndarray,
    bar_end_mm: np.ndarray,
) -> np.ndarray:
    from core.transforms import frame_from_axes, invert_transform, unit

    x_anchored = x_anchored_to_block_factory(block_frame_mm)
    bar_z = unit(np.asarray(bar_end_mm - bar_start_mm, dtype=float))
    bar_x = x_anchored(bar_z)
    bar_y = unit(np.cross(bar_z, bar_x))
    bar_frame = frame_from_axes(bar_start_mm, bar_x, bar_y, bar_z)
    return invert_transform(bar_frame) @ block_frame_mm


def compute_M_screw_from_block(
    block_frame_mm: np.ndarray,
    screw_origin_mm: np.ndarray,
    screw_dir_world: np.ndarray,
) -> np.ndarray:
    from core.transforms import frame_from_axes, invert_transform, unit

    x_anchored = x_anchored_to_block_factory(block_frame_mm)
    z_axis = unit(np.asarray(screw_dir_world, dtype=float))
    x_axis = x_anchored(z_axis)
    y_axis = unit(np.cross(z_axis, x_axis))
    screw_frame = frame_from_axes(screw_origin_mm, x_axis, y_axis, z_axis)
    return invert_transform(block_frame_mm) @ screw_frame
