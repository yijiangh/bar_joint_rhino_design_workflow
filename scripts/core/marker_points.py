"""Marker spheres on a MoCap joint, moved between the frames they are read in.

A MoCap block (``T20_MoCap``) carries OptiTrack marker spheres.  Their centres
are recorded once, when the block is defined, in the BLOCK's own frame
(``JointHalfDef.marker_points_mm``: ``{Motive label: (x, y, z)}``, mm), so the
numbers describe the part, not wherever it sat when it was picked.  A placed
instance then predicts each sphere as ``block_world @ point``, and
RSExportPrefab re-expresses those in the bar's own frame
(``joint_pair.canonical_bar_frame_from_line``).

Pure numpy, no Rhino; the frame maths is ``core.transforms``.  The pick step
lives in RSDefineJointHalf.
"""

from __future__ import annotations

import numpy as np

from core.transforms import invert_transform, local_transform, transform_point

#: Default Motive labels, in pick order: M1, M2, ...
DEFAULT_LABEL_PREFIX = "M"


def bounding_box_centre(corners) -> np.ndarray:
    """Centre of a sphere from its bounding-box corners (any number, >= 2)."""
    pts = np.asarray(corners, dtype=float).reshape(-1, 3)
    return (pts.min(axis=0) + pts.max(axis=0)) / 2.0


def to_block_local_mm(block_frame_mm, point_mm) -> tuple:
    """A world point (mm) in the block's own frame (mm), as a plain tuple."""
    local = transform_point(invert_transform(block_frame_mm), point_mm)
    return tuple(float(c) for c in local)


def markers_in_frame_mm(block_world_mm, points: dict, frame_mm) -> dict:
    """A placed block's markers expressed in *frame_mm*, rounded to 0.01 mm.

    ``points`` is ``{label: block-local point}``; *block_world_mm* the placed
    block's world transform; *frame_mm* the frame to report in (the bar frame).
    """
    block_in_frame = local_transform(frame_mm, block_world_mm)
    return {
        label: [round(float(c), 2) for c in transform_point(block_in_frame, p)]
        for label, p in points.items()
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
    "markers_in_frame_mm",
    "next_default_label",
]
