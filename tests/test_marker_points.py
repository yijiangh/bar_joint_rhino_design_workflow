"""``core.marker_points`` -- marker spheres between block, world and bar frames.

The round trip that matters: a sphere picked in the document is stored in the
block's own frame, and the prefab export re-expresses it in the bar frame of
wherever the block is placed.  Pure numpy -- no Rhino.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import marker_points as mp
from core.joint_pair import canonical_bar_frame_from_line


def _rot_z(deg):
    a = np.radians(deg)
    frame = np.eye(4)
    frame[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return frame


def _block_frame():
    frame = _rot_z(30.0)
    frame[:3, 3] = (100.0, -50.0, 20.0)
    return frame


def test_bounding_box_centre_of_a_sphere():
    corners = [(1, 2, 3), (5, 2, 3), (1, 6, 3), (5, 6, 7), (1, 2, 7)]
    assert np.allclose(mp.bounding_box_centre(corners), (3, 4, 5))


def test_block_local_is_a_plain_float_tuple():
    local = mp.to_block_local_mm(_block_frame(), (130.0, -10.0, 45.0))
    assert isinstance(local, tuple) and all(type(c) is float for c in local)


def test_picked_point_comes_back_in_the_world_frame():
    """Stored block-local, reported in the world frame (identity) = the pick."""
    block = _block_frame()
    picked = (130.0, -10.0, 45.0)
    local = mp.to_block_local_mm(block, picked)
    out = mp.markers_in_frame_mm(block, {"M1": local}, np.eye(4))
    assert np.allclose(out["M1"], picked, atol=0.01)


def test_markers_in_the_bar_frame():
    """Bar along world +X from x=10: 100 mm along the bar is bar-frame z = 100."""
    bar = canonical_bar_frame_from_line((10.0, 0.0, 0.0), (1010.0, 0.0, 0.0))
    out = mp.markers_in_frame_mm(np.eye(4), {"M1": (110.0, 0.0, 0.0)}, bar)
    assert out["M1"] == pytest.approx([0.0, 0.0, 100.0], abs=0.01)


def test_marker_distance_from_the_bar_axis_is_kept():
    """Moving the block moves its markers rigidly."""
    local = mp.to_block_local_mm(_block_frame(), (130.0, -10.0, 45.0))
    placed = _rot_z(-90.0)
    out = mp.markers_in_frame_mm(placed, {"M1": local}, np.eye(4))
    assert np.linalg.norm(out["M1"]) == pytest.approx(np.linalg.norm(local), abs=0.01)


def test_default_labels_skip_used_ones():
    assert mp.next_default_label(set()) == "M1"
    assert mp.next_default_label({"M1", "M2", "M4"}) == "M3"
