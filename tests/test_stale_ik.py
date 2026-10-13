"""Stale IK: a solved base that stands out of reach of its bar (the bar was moved or copied after the solve)."""

import sys
from types import SimpleNamespace

import numpy as np

from core import keyframe_store
from core.keyframe_store import STALE_BASE_DISTANCE_MM, base_to_bar_distance_mm


def _base(x, y, yaw=0.0, z=0.0):
    frame = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    frame[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    frame[:3, 3] = [x, y, z]
    return frame


def test_distance_is_to_the_nearest_point_of_the_segment():
    base = _base(0.0, 0.0)
    assert np.isclose(base_to_bar_distance_mm(base, [800, -500, 1000], [800, 500, 1000]), 800.0)  # beside the middle
    assert np.isclose(base_to_bar_distance_mm(base, [300, 400, 0], [300, 2000, 0]), 500.0)  # nearest is an end


def test_distance_ignores_height_and_the_base_heading():
    start, end = [800, -500, 2500], [800, 500, 2500]
    assert np.isclose(base_to_bar_distance_mm(_base(0, 0, yaw=1.2), start, end), 800.0)
    assert np.isclose(base_to_bar_distance_mm(_base(0, 0, z=300.0), start, end), 800.0)


def test_vertical_bar_is_a_point_in_the_ground_plane():
    assert np.isclose(base_to_bar_distance_mm(_base(100, 0), [700, 0, 0], [700, 0, 2000]), 600.0)


def test_threshold_separates_solved_bars_from_copied_modules():
    # Measured on 261008_2ModulesDemo: solved bases stand 0.4-0.9 m from their bar's midpoint, the stale ones of
    # the copied module 5-11 m.
    assert 900.0 < STALE_BASE_DISTANCE_MM < 4000.0


def test_stale_ik_reason(monkeypatch):
    bars = {"near": ([800, -500, 1000], [800, 500, 1000]), "far": ([6000, -500, 1000], [6000, 500, 1000])}
    monkeypatch.setattr(keyframe_store, "read_assembly_keyframe",
                        lambda oid: None if oid == "unsolved" else {"base_frame_world_mm": _base(0, 0)})
    # core.rhino_helpers needs Rhino: stand in for the two functions the check reads the bar with.
    monkeypatch.setitem(sys.modules, "core.rhino_helpers", SimpleNamespace(
        doc_unit_scale_to_mm=lambda: 1.0,
        curve_endpoints=lambda oid: tuple(np.array(p, float) for p in bars[oid])))
    assert keyframe_store.stale_ik_reason("unsolved") is None
    assert keyframe_store.stale_ik_reason("near") is None
    assert "6.0 m from the bar" in keyframe_store.stale_ik_reason("far")
