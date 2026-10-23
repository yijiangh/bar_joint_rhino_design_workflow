"""MoCap marker layouts for tests, built the way RSJointPlace places joints.

Shared by ``test_mocap_layout_check`` and ``test_mocap_similarity``: the real
``T20_MoCap`` registry points placed on a synthetic 1800 mm bar along world X,
expressed in the bar's own frame and divided by 1000 (both modules work in
metres).  Pure numpy.
"""

from __future__ import annotations

import numpy as np

from core.joint_pair import canonical_bar_frame_from_line, load_joint_registry
from core.marker_points import markers_in_frame_mm
from core.single_sided_placement import fk_single_block_frame

T20_MOCAP = load_joint_registry().halves["T20_MoCap"]
BAR_LENGTH_MM = 1800.0
BAR = (np.array([0.0, 0.0, 0.0]), np.array([BAR_LENGTH_MM, 0.0, 0.0]))


def layout(joints) -> np.ndarray:
    """``(2n, 3)`` marker centres in METRES, bar frame, for ``[(along_mm, turn_deg), ...]``.

    Marker order: joint by joint as given, M1 then M2 -- the order the export
    uses when the joints are already sorted along the bar.
    """
    start, end = BAR
    bar_frame = canonical_bar_frame_from_line(start, end)
    points = []
    for along_mm, turn_deg in joints:
        world = fk_single_block_frame(start, end, along_mm, np.radians(turn_deg), T20_MOCAP)
        in_bar = markers_in_frame_mm(world, T20_MOCAP.marker_points_mm, bar_frame)
        points.append(in_bar["M1"])
        points.append(in_bar["M2"])
    return np.asarray(points, dtype=float) / 1000.0


def rigid_motion(seed: int):
    """A random rotation and translation, as ``(R, t)``."""
    rng = np.random.default_rng(seed)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rot, rng.normal(scale=1.0, size=3)
