"""``core.mocap_layout_check`` -- can a bar's marker layout give one 6-DoF pose?

Layouts are built from the real ``T20_MoCap`` registry points placed on a
synthetic 1800 mm bar the way RSJointPlace places them, then divided by 1000
(the module works in metres) -- see ``tests/mocap_layouts.py``.  Pure numpy.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from core import mocap_layout_check as lc
from mocap_layouts import layout, rigid_motion

TOL = lc.DEFAULT_TOL_M

#: How exactly a half-turn's angle can be read back, in degrees.  The angle comes
#: from arccos of a value next to -1, where rounding noise of 1e-13 in the matrix
#: already moves it by about 1e-6 deg, so a 1e-6 tolerance fails on round-off.
ANGLE_ABS_DEG = 1e-4


# ---------------------------------------------------------------------------
# fit_rigid
# ---------------------------------------------------------------------------


def test_fit_rigid_recovers_a_known_motion():
    pts = layout([(300, 0), (900, 90), (1500, 180)])
    rot, trans = rigid_motion(1)
    moved = pts @ rot.T + trans
    rot_fit, trans_fit = lc.fit_rigid(pts, moved)
    assert np.allclose(rot_fit, rot, atol=1e-9)
    assert np.allclose(trans_fit, trans, atol=1e-9)
    assert np.max(lc.rigid_residuals_m(rot_fit, trans_fit, pts, moved)) < 1e-9


def test_fit_rigid_never_returns_a_mirror():
    pts = layout([(300, 0), (900, 90), (1500, 180)])
    mirrored = pts * np.array([1.0, 1.0, -1.0])
    rot, trans = lc.fit_rigid(pts, mirrored)
    assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-9)
    # A rotation cannot reproduce a mirror image of a 3-D layout: large residual.
    assert np.max(lc.rigid_residuals_m(rot, trans, pts, mirrored)) > 0.01


def test_rotation_angle_axis_of_a_half_turn():
    rot = np.diag([1.0, -1.0, -1.0])   # 180 deg about x
    angle, axis = lc.rotation_angle_axis(rot)
    assert angle == pytest.approx(180.0)
    assert abs(axis[0]) == pytest.approx(1.0) and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9


# ---------------------------------------------------------------------------
# check_layout: the refusals
# ---------------------------------------------------------------------------


def test_one_joint_is_refused():
    report = lc.check_layout(layout([(300, 0)]), TOL)
    assert not report.ok
    assert "fewer than 3" in report.reason()


def test_collinear_markers_are_refused():
    pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 1.2]])
    report = lc.check_layout(pts, TOL)
    assert report.collinear and not report.ok
    assert "one line" in report.reason()


def test_two_joints_same_facing_are_ambiguous():
    report = lc.check_layout(layout([(300, 0), (1100, 0)]), TOL)
    assert not report.ok
    assert len(report.symmetries) >= 3
    for sym in report.symmetries:
        assert sym.max_residual_m <= 1e-6
        assert sym.angle_deg == pytest.approx(180.0, abs=ANGLE_ABS_DEG)


def test_two_joints_are_never_enough_even_turned():
    """Two joints 90 deg apart: the end-for-end half-turn still maps the set onto itself."""
    report = lc.check_layout(layout([(300, 0), (1100, 90)]), TOL)
    assert not report.ok
    # Symmetries are sorted by residual: the exact end-for-end half-turn comes
    # first (joint 1's markers 0, 1 land on joint 2's 2, 3 and back).  Near
    # misses within the tolerance may follow it; they do not matter here.
    sym = report.symmetries[0]
    assert set(sym.permutation[:2]) == {2, 3} and set(sym.permutation[2:]) == {0, 1}
    assert sym.angle_deg == pytest.approx(180.0, abs=ANGLE_ABS_DEG)
    assert sym.max_residual_m <= 1e-6
    assert "half-turn" in report.reason()


def test_three_joints_same_facing_are_ambiguous_by_one_pair_swap():
    report = lc.check_layout(layout([(200, 0), (700, 0), (1500, 0)]), TOL)
    assert not report.ok
    assert len(report.symmetries) == 1
    assert report.symmetries[0].permutation == (1, 0, 3, 2, 5, 4)


def test_three_joints_mirror_symmetric_about_the_middle_are_refused_and_a_shift_fixes_it():
    assert not lc.check_layout(layout([(300, 0), (900, 180), (1500, 0)]), TOL).ok
    assert lc.check_layout(layout([(300, 0), (920, 180), (1500, 0)]), TOL).ok


# ---------------------------------------------------------------------------
# check_layout: the accepted ones
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joints", [
    [(300, 0), (900, 90), (1500, 0)],      # equal spacing, one turned
    [(300, 0), (700, 90), (1500, 0)],      # unequal spacing, one turned
    [(300, 0), (700, 90), (1500, 180)],
    [(74.9, 90), (430.0, 180), (1135.3, 20), (1670.0, 180)],   # B20 of the real demo, roughly
])
def test_three_or_more_joints_with_mixed_facing_pass(joints):
    report = lc.check_layout(layout(joints), TOL)
    assert report.ok, report.reason()
    assert report.symmetries == ()
    assert report.nearest_miss_m is None or report.nearest_miss_m > TOL


def test_long_bar_signatures_agree_but_the_layout_is_still_ok():
    """Guard against a signature-only rule: a pair's two markers have the same distance
    pattern within the tolerance (every other marker is far away), yet the bar is fine."""
    pts = layout([(100, 0), (900, 90), (1700, 0)])
    dist = np.linalg.norm(pts[:, None] - pts[None], axis=2)
    sig = np.sort(dist, axis=1)
    assert np.max(np.abs(sig[0] - sig[1])) < TOL          # M1 vs M2 of the first joint
    assert lc.check_layout(pts, TOL).ok


# ---------------------------------------------------------------------------
# match: design against a measured set
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("noise_m", [0.0, 0.0005, 0.001])
def test_match_finds_exactly_the_inverse_shuffle(noise_m):
    design = layout([(300, 0), (700, 90), (1500, 180)])
    rot, trans = rigid_motion(7)
    rng = np.random.default_rng(3)
    shuffle = rng.permutation(len(design))
    measured = (design @ rot.T + trans)[shuffle] + rng.normal(scale=noise_m / 2, size=design.shape)
    found = lc.match(design, measured, TOL)
    assert len(found) == 1
    # measured[i] is design[shuffle[i]]
    assert found[0].permutation == tuple(int(s) for s in shuffle)
    assert found[0].max_residual_m <= TOL


def test_match_rejects_a_mirror_image():
    design = layout([(300, 0), (700, 90), (1500, 180)])
    assert lc.match(design, design * np.array([1.0, 1.0, -1.0]), TOL) == []


def test_match_rejects_a_different_bar():
    design = layout([(300, 0), (700, 90), (1500, 180)])
    other = layout([(300, 0), (800, 90), (1500, 180)])   # middle joint 100 mm away
    assert lc.match(design, other, TOL) == []


# ---------------------------------------------------------------------------
# require_unambiguous, and speed
# ---------------------------------------------------------------------------


def test_require_unambiguous_names_the_bar_and_adds_the_hint():
    with pytest.raises(lc.MoCapLayoutError, match="bar B20") as info:
        lc.require_unambiguous(layout([(300, 0), (1100, 0)]), TOL, "bar B20", hint="Add a joint.")
    assert "Add a joint." in str(info.value)
    assert lc.require_unambiguous(layout([(300, 0), (700, 90), (1500, 0)]), TOL, "bar B21").ok


def test_twelve_markers_check_quickly():
    pts = layout([(100, 0), (400, 0), (700, 0), (1000, 0), (1300, 0), (1600, 0)])   # worst case: all same facing
    t0 = time.perf_counter()
    report = lc.check_layout(pts, TOL)
    assert time.perf_counter() - t0 < 1.0
    assert not report.ok
