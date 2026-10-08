"""``core.marker_points`` -- marker spheres between block, world and bar frames.

The round trip that matters: a sphere picked in the document is stored in the
block's own frame, a placed instance predicts it back in the world, and the
prefab export re-expresses it in the bar frame.  Pure numpy -- no Rhino.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import marker_points as mp


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


def test_block_local_round_trip():
    block = _block_frame()
    picked = (130.0, -10.0, 45.0)
    local = mp.to_block_local_mm(block, picked)
    assert all(type(c) is float for c in local)
    assert np.allclose(mp.to_world_mm(block, {"M1": local})["M1"], picked)


def test_a_moved_instance_carries_its_markers():
    """Defined at one pose, predicted at another: the offset from the block is kept."""
    local = mp.to_block_local_mm(_block_frame(), (130.0, -10.0, 45.0))
    moved = _rot_z(-90.0)
    moved[:3, 3] = (0.0, 0.0, 0.0)
    world = np.asarray(mp.to_world_mm(moved, {"M1": local})["M1"])
    assert np.linalg.norm(world) == pytest.approx(np.linalg.norm(local))


def test_bar_frame_coordinates():
    """Origin at the bar start, z along the bar, x as given."""
    frame = mp.frame_from_axes((10.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))
    out = mp.in_frame_mm({"M1": (110.0, 5.0, 2.0)}, frame)
    # bar z = world +X: 100 mm along the bar; bar x = world +Y: 5; bar y = z x x = world +Z: 2
    assert out == {"M1": [5.0, 2.0, 100.0]}


def test_frame_from_axes_is_orthonormal_even_from_a_loose_x():
    frame = mp.frame_from_axes((0, 0, 0), (1.0, 0.2, 0.3), (0.0, 0.0, 2.0))
    assert np.allclose(frame[:3, :3].T @ frame[:3, :3], np.eye(3))
    assert np.allclose(frame[:3, 2], (0, 0, 1))


def test_default_labels_skip_used_ones():
    assert mp.next_default_label(set()) == "M1"
    assert mp.next_default_label({"M1", "M2", "M4"}) == "M3"
