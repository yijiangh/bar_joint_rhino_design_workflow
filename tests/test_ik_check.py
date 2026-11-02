"""``core.ik_check`` -- the Rhino-free helpers behind RSIKMaintenance > CheckIK.

The replay itself needs Rhino and PyBullet (checked by hand, see the toolbar
doc); these pin the geometry each verdict rests on.  Millimetres throughout.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import ik_check


def frame(origin=(0.0, 0.0, 0.0), rot=None) -> np.ndarray:
    out = np.eye(4)
    if rot is not None:
        out[:3, :3] = rot
    out[:3, 3] = origin
    return out


def rot_z(deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ---------------------------------------------------------------------------
# frame_error / reach_problem
# ---------------------------------------------------------------------------


def test_frame_error_reads_distance_and_angle():
    distance, angle = ik_check.frame_error(frame(), frame((3.0, 4.0, 0.0), rot_z(30.0)))
    assert distance == pytest.approx(5.0)
    assert angle == pytest.approx(30.0)


def test_identical_frames_have_no_error():
    target = frame((100.0, -20.0, 7.0), rot_x(12.0))
    assert ik_check.frame_error(target, target.copy()) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_reach_problem_only_past_either_tolerance():
    target = frame()
    assert ik_check.reach_problem("left tool", target, frame((0.5, 0.0, 0.0)), 1.0, 0.5) is None
    assert "2.0 mm" in ik_check.reach_problem("left tool", target, frame((2.0, 0.0, 0.0)), 1.0, 0.5)
    assert "1.0 deg" in ik_check.reach_problem("left tool", target, frame(rot=rot_z(1.0)), 1.0, 0.5)


# ---------------------------------------------------------------------------
# point_to_segment_mm / axis_angle_deg (the support grasp on its bar)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("point, expected", [
    ((500.0, 0.0, 0.0), 0.0),        # on the bar
    ((500.0, 30.0, 40.0), 50.0),     # beside the middle
    ((-30.0, 40.0, 0.0), 50.0),      # before the start: distance to the start
    ((1030.0, 0.0, 40.0), 50.0),     # past the end: distance to the end
])
def test_point_to_segment(point, expected):
    assert ik_check.point_to_segment_mm(point, (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)) == pytest.approx(expected)


def test_axis_angle_ignores_direction():
    assert ik_check.axis_angle_deg((1.0, 0.0, 0.0), (-2.0, 0.0, 0.0)) == pytest.approx(0.0)
    assert ik_check.axis_angle_deg((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)) == pytest.approx(45.0)
    assert ik_check.axis_angle_deg((1.0, 0.0, 0.0), (0.0, 0.0, 3.0)) == pytest.approx(90.0)


# ---------------------------------------------------------------------------
# base_ground_error / ground_problem
# ---------------------------------------------------------------------------

UP = np.array([0.0, 0.0, 1.0])


def test_base_on_the_ground_is_fine():
    base = frame((100.0, 200.0, 0.0))
    assert ik_check.base_ground_error(base, (100.0, 200.0, 0.0), UP) == pytest.approx((0.0, 0.0, 0.0))
    assert ik_check.ground_problem("base", base, (100.0, 200.0, 0.0), UP, 10.0, 3.0) is None


def test_ground_raised_under_the_base_reads_as_below():
    base = frame((0.0, 0.0, 0.0))
    height, beside, tilt = ik_check.base_ground_error(base, (0.0, 0.0, 50.0), UP)
    assert (height, beside, tilt) == pytest.approx((-50.0, 0.0, 0.0))
    assert "50 mm below" in ik_check.ground_problem("base", base, (0.0, 0.0, 50.0), UP, 10.0, 3.0)


def test_ground_lowered_reads_as_above():
    base = frame((0.0, 0.0, 0.0))
    assert "40 mm above" in ik_check.ground_problem("base", base, (0.0, 0.0, -40.0), UP, 10.0, 3.0)


def test_base_past_the_ground_edge():
    # The nearest ground point is the edge, 80 mm to the side at the same height.
    base = frame((80.0, 0.0, 0.0))
    assert "past the edge" in ik_check.ground_problem("base", base, (0.0, 0.0, 0.0), UP, 10.0, 3.0)


def test_base_tilted_against_a_sloped_ground():
    base = frame((0.0, 0.0, 0.0))
    sloped = rot_x(5.0) @ UP
    _height, _beside, tilt = ik_check.base_ground_error(base, (0.0, 0.0, 0.0), sloped)
    assert tilt == pytest.approx(5.0)
    assert "tilted 5.0 deg" in ik_check.ground_problem("base", base, (0.0, 0.0, 0.0), sloped, 10.0, 3.0)


def test_no_ground_found_is_a_problem():
    assert "no walkable ground" in ik_check.ground_problem("base", frame(), None, None, 10.0, 3.0)


# ---------------------------------------------------------------------------
# BarIKReport
# ---------------------------------------------------------------------------


def test_a_report_is_stale_only_with_problems():
    report = ik_check.BarIKReport("B4", 4)
    assert not report.stale
    report.notes.append("a hint only")
    assert not report.stale
    report.support.append("support (Alice): grasp 35.0 mm / 0.0 deg off the bar")
    assert report.stale
