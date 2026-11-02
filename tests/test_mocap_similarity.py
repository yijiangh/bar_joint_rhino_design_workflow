"""``core.mocap_similarity`` -- which bars could Motive mix up?

Layouts come from ``tests/mocap_layouts.py``: the real ``T20_MoCap`` markers on a
synthetic 1800 mm bar, in the bar's own frame, metres.  Pure numpy.
"""

from __future__ import annotations

import numpy as np
import pytest

from core import mocap_similarity as ms
from core.mocap_layout_check import DEFAULT_TOL_M
from mocap_layouts import BAR_LENGTH_MM, layout, rigid_motion

#: The RSCheckMoCapLayout default (config.MOCAP_SIMILAR_TOL_MM), in metres.
TOL = 0.005

#: A layout the export accepts (three joints, mixed facing).
GOOD = [(300, 0), (700, 90), (1500, 180)]


def end_for_end(points_m) -> np.ndarray:
    """The same bar turned end for end, in its own frame: a half-turn about x."""
    return points_m * np.array([1.0, -1.0, -1.0]) + np.array([0.0, 0.0, BAR_LENGTH_MM / 1000.0])


def turned_about_bar(points_m, deg: float) -> np.ndarray:
    """The same bar turned *deg* about its own axis (z)."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return points_m @ rot.T


# ---------------------------------------------------------------------------
# compare_bars: what counts as similar
# ---------------------------------------------------------------------------


def test_identical_placements_are_similar():
    pairs, unchecked = ms.compare_bars({"B1": layout(GOOD), "B2": layout(GOOD)}, TOL)
    assert unchecked == []
    assert len(pairs) == 1
    pair = pairs[0]
    assert (pair.bar_a, pair.bar_b, pair.n_markers) == ("B1", "B2", 6)
    assert pair.relation == "same placement"
    assert pair.max_residual_m < 1e-9


def test_the_same_placement_end_for_end_is_similar():
    pairs, _ = ms.compare_bars({"B1": layout(GOOD), "B2": end_for_end(layout(GOOD))}, TOL)
    assert [p.relation for p in pairs] == ["end for end"]


def test_the_same_placement_turned_about_the_bar_is_similar():
    pairs, _ = ms.compare_bars({"B1": layout(GOOD), "B2": turned_about_bar(layout(GOOD), 90.0)}, TOL)
    assert [p.relation for p in pairs] == ["turned 90 deg about the bar"]


def test_any_rigid_motion_is_still_similar():
    rot, trans = rigid_motion(5)
    moved = layout(GOOD) @ rot.T + trans
    pairs, _ = ms.compare_bars({"B1": layout(GOOD), "B2": moved}, TOL)
    assert len(pairs) == 1
    assert pairs[0].max_residual_m < 1e-9


def test_a_joint_moved_5_mm_is_similar_at_5_mm_but_not_at_2_mm():
    a = layout(GOOD)
    b = layout([(305, 0), (700, 90), (1500, 180)])
    assert len(ms.compare_bars({"B1": a, "B2": b}, TOL)[0]) == 1
    assert ms.compare_bars({"B1": a, "B2": b}, DEFAULT_TOL_M)[0] == []


def test_a_joint_moved_50_mm_is_not_similar():
    a = layout(GOOD)
    b = layout([(300, 0), (750, 90), (1500, 180)])
    assert ms.compare_bars({"B1": a, "B2": b}, TOL)[0] == []


def test_a_mirror_image_is_not_similar():
    a = layout(GOOD)
    pairs, _ = ms.compare_bars({"B1": a, "B2": a * np.array([1.0, 1.0, -1.0])}, TOL)
    assert pairs == []


def test_different_marker_counts_are_never_compared():
    a = layout(GOOD)
    b = layout(GOOD + [(1700, 90)])
    assert ms.compare_bars({"B1": a, "B2": b}, TOL)[0] == []


def test_pairs_come_in_bar_order():
    pairs, _ = ms.compare_bars({"B10": layout(GOOD), "B2": layout(GOOD)}, TOL)
    assert (pairs[0].bar_a, pairs[0].bar_b) == ("B2", "B10")
    assert pairs[0].describe() == "B10 = B2 same placement, within 0.0 mm"


# ---------------------------------------------------------------------------
# similar_groups
# ---------------------------------------------------------------------------


def _pair(a, b):
    return ms.SimilarPair(a, b, 6, 0.0, "same placement")


def test_chains_join_into_one_group_and_groups_are_ordered():
    groups = ms.similar_groups([_pair("B5", "B6"), _pair("B2", "B3"), _pair("B1", "B3")])
    assert groups == [["B1", "B2", "B3"], ["B5", "B6"]]


def test_no_pairs_no_groups():
    assert ms.similar_groups([]) == []


def test_three_identical_bars_form_one_group():
    layouts = {bar: layout(GOOD) for bar in ("B1", "B2", "B3")}
    pairs, _ = ms.compare_bars(layouts, TOL)
    assert len(pairs) == 3
    assert ms.similar_groups(pairs) == [["B1", "B2", "B3"]]


# ---------------------------------------------------------------------------
# refused_bars: the export's own per-bar check
# ---------------------------------------------------------------------------


def test_refused_bars_flags_what_the_export_would_refuse():
    refused = ms.refused_bars(
        {"B1": layout(GOOD), "B2": layout([(300, 0), (1100, 90)])}, DEFAULT_TOL_M
    )
    assert list(refused) == ["B2"]
    assert "half-turn" in refused["B2"]


@pytest.mark.parametrize("joints", [[(300, 0)], [(300, 0), (1100, 0)]])
def test_one_or_two_joints_are_always_refused(joints):
    assert "B1" in ms.refused_bars({"B1": layout(joints)}, DEFAULT_TOL_M)
