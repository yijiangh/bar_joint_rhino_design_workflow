"""Marker spheres on a MoCap joint, moved between the frames they are read in.

A MoCap block (``T20_MoCap``) carries OptiTrack marker spheres.  Their centres
are recorded once, when the block is defined, in the BLOCK's own frame
(``JointHalfDef.marker_points_mm``: ``{Motive label: (x, y, z)}``, mm), so the
numbers describe the part, not wherever it sat when it was picked.  A placed
instance then predicts each sphere's world position as ``block_world @ point``,
and RSExportPrefab re-expresses those in the bar's own frame.

Pure numpy, no Rhino: the pick step lives in RSDefineJointHalf.
"""

from __future__ import annotations

import numpy as np

#: Default Motive labels, in pick order: M1, M2, ...
DEFAULT_LABEL_PREFIX = "M"


def bounding_box_centre(corners) -> np.ndarray:
    """Centre of a sphere from its bounding-box corners (any number, >= 2)."""
    pts = np.asarray(corners, dtype=float).reshape(-1, 3)
    return (pts.min(axis=0) + pts.max(axis=0)) / 2.0


def to_block_local_mm(block_frame_mm, point_mm) -> tuple:
    """A world point (mm) in the block's own frame (mm), as a plain tuple."""
    frame = np.asarray(block_frame_mm, dtype=float)
    local = np.linalg.inv(frame) @ np.append(np.asarray(point_mm, dtype=float), 1.0)
    return tuple(float(c) for c in local[:3])


def to_world_mm(block_world_mm, points: dict) -> dict:
    """``{label: block-local point}`` -> ``{label: world point}`` for a placed block."""
    frame = np.asarray(block_world_mm, dtype=float)
    return {
        label: tuple(float(c) for c in (frame @ np.append(np.asarray(p, dtype=float), 1.0))[:3])
        for label, p in points.items()
    }


def frame_from_axes(origin, x_axis, z_axis) -> np.ndarray:
    """4x4 frame with the given origin, +X and +Z (Y = Z x X, right-handed)."""
    z = np.asarray(z_axis, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_axis, dtype=float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    frame = np.eye(4)
    frame[:3, 0] = x
    frame[:3, 1] = np.cross(z, x)
    frame[:3, 2] = z
    frame[:3, 3] = np.asarray(origin, dtype=float)
    return frame


def in_frame_mm(points_world: dict, frame_mm) -> dict:
    """``{label: world point}`` expressed in *frame_mm*, rounded to 0.01 mm."""
    inv = np.linalg.inv(np.asarray(frame_mm, dtype=float))
    return {
        label: [round(float(c), 2) for c in (inv @ np.append(np.asarray(p, dtype=float), 1.0))[:3]]
        for label, p in points_world.items()
    }


def next_default_label(used) -> str:
    """The first ``M<n>`` (n = 1, 2, ...) not in *used*."""
    n = 1
    while f"{DEFAULT_LABEL_PREFIX}{n}" in used:
        n += 1
    return f"{DEFAULT_LABEL_PREFIX}{n}"


__all__ = [
    "bounding_box_centre",
    "to_block_local_mm",
    "to_world_mm",
    "frame_from_axes",
    "in_frame_mm",
    "next_default_label",
]
