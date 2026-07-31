"""Tests for `core.base_guide_geom` -- base-placement guides, heading, arm sides.

These cover the three ways the OLD base heading went wrong and put the mobile
base on an arbitrary (often opposite) side of the bar:

* ground-joint anchors were never collected at all;
* two anchors ~180 deg apart in `jr` cancelled, and the floating-point residual
  was returned as if it were a direction;
* a near-vertical anchor axis contributed a noise azimuth once flattened.

Pure numpy -- no Rhino -- per the scope note in ``tests/conftest.py``. The Rhino
half (`core.rhino_walkable_ground.resolve_bar_heading`) only gathers the inputs
these functions consume, and is verified manually via the toolbar workflow.
"""

import os
import sys

import numpy as np
import pytest


TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(TESTS_DIR, "..", "scripts")
for _path in (TESTS_DIR, SCRIPTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from core import base_guide_geom as bgg  # noqa: E402
from core import config  # noqa: E402


TOL = 1e-9
UP = np.array([0.0, 0.0, 1.0])

# The heading / arm-side maths is pure numpy and runs anywhere. `build_base_guides`
# additionally needs `closest_point_on_meshes`, which lives in the tamp submodule
# alongside a top-level `compas` import -- present in the Rhino `scaffolding_env`
# (every script header pins `# r: compas==2.13.0`) but not in a bare interpreter.
# Skip rather than fail there, so the suite stays honest about what it covered.
try:  # noqa: SIM105
    import compas  # noqa: F401
    _HAS_COMPAS = True
except ImportError:
    _HAS_COMPAS = False

needs_compas = pytest.mark.skipif(
    not _HAS_COMPAS,
    reason="ground projection needs compas (install it, or run in scaffolding_env)",
)


# ---------------------------------------------------------------------------
# Fixtures: synthetic walkable grounds as triangle soups
# ---------------------------------------------------------------------------


def _flat_soup(size=20000.0, z=0.0):
    """One big flat square in the z=`z` plane, as two triangles."""
    half = size / 2.0
    vertices = np.array([
        [-half, -half, z],
        [half, -half, z],
        [half, half, z],
        [-half, half, z],
    ], dtype=float)
    triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)
    return (vertices, triangles)


def _half_soup(size=20000.0, z=0.0, x_min=0.0):
    """A flat square covering only ``x >= x_min`` -- ground that runs out."""
    half = size / 2.0
    vertices = np.array([
        [x_min, -half, z],
        [half, -half, z],
        [half, half, z],
        [x_min, half, z],
    ], dtype=float)
    triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=int)
    return (vertices, triangles)


# ---------------------------------------------------------------------------
# resolve_heading_from_axes
# ---------------------------------------------------------------------------


def test_axes_that_agree_are_averaged():
    """Two anchors pointing the same way -> that direction, not ambiguous."""
    axes = [np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
    verdict = bgg.resolve_heading_from_axes(axes, UP)

    assert verdict["ambiguous"] is False
    assert verdict["mean_resultant"] == pytest.approx(1.0)
    np.testing.assert_allclose(verdict["heading"], [1.0, 0.0, 0.0], atol=1e-12)


def test_slightly_splayed_axes_still_average():
    """A real bar's two anchors rarely agree exactly; 30 deg apart is fine."""
    a = np.array([np.cos(np.radians(15.0)), np.sin(np.radians(15.0)), 0.0])
    b = np.array([np.cos(np.radians(-15.0)), np.sin(np.radians(-15.0)), 0.0])
    verdict = bgg.resolve_heading_from_axes([a, b], UP)

    assert verdict["ambiguous"] is False
    np.testing.assert_allclose(verdict["heading"], [1.0, 0.0, 0.0], atol=1e-12)


def test_antiparallel_axes_are_reported_ambiguous():
    """REGRESSION (the reported 2-male-bar bug).

    A male half's +Z is -bar_X spun about the bar by its `jr`, so two anchors
    180 deg apart in `jr` cancel. The old code summed them and returned the
    floating-point residual whenever it cleared 1e-9 -- an essentially random
    azimuth, which is why SOME two-male bars landed on the opposite side. The
    resolver must refuse to answer instead.
    """
    axes = [np.array([1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])]
    verdict = bgg.resolve_heading_from_axes(axes, UP)

    assert verdict["ambiguous"] is True
    assert verdict["heading"] is None
    assert verdict["mean_resultant"] == pytest.approx(0.0, abs=1e-12)
    assert "cancel" in verdict["detail"]


def test_nearly_antiparallel_axes_are_also_ambiguous():
    """The failure is not knife-edge: 170 deg apart is just as meaningless."""
    a = np.array([1.0, 0.0, 0.0])
    b = np.array([np.cos(np.radians(170.0)), np.sin(np.radians(170.0)), 0.0])
    verdict = bgg.resolve_heading_from_axes([a, b], UP)

    assert verdict["ambiguous"] is True
    assert verdict["mean_resultant"] < config.INSERTION_DIR_CANCEL_TOL


def test_near_vertical_axis_is_discarded_not_averaged():
    """A `jr` that tips an axis near-vertical must not steer the heading.

    Here one anchor points along +X and the other almost straight down. The
    down-pointing one has a tiny horizontal component whose direction is
    numerical noise; dropping it leaves a clean +X heading.
    """
    horizontal = np.array([1.0, 0.0, 0.0])
    near_vertical = np.array([0.02, -0.03, -0.999])
    near_vertical = near_vertical / np.linalg.norm(near_vertical)
    verdict = bgg.resolve_heading_from_axes([horizontal, near_vertical], UP)

    assert verdict["n_vertical"] == 1
    assert verdict["ambiguous"] is False
    np.testing.assert_allclose(verdict["heading"], [1.0, 0.0, 0.0], atol=1e-12)


def test_all_axes_vertical_is_ambiguous():
    """Nothing usable left -> ambiguous, so the caller falls back."""
    axes = [np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])]
    verdict = bgg.resolve_heading_from_axes(axes, UP)

    assert verdict["ambiguous"] is True
    assert verdict["n_vertical"] == 2
    assert verdict["heading"] is None


def test_no_axes_at_all_is_ambiguous():
    """A bar whose anchors could not be read (the old ground-joint case)."""
    verdict = bgg.resolve_heading_from_axes([], UP)

    assert verdict["ambiguous"] is True
    assert verdict["heading"] is None


def test_heading_is_flattened_onto_a_sloped_ground():
    """The heading always lies IN the ground plane, whatever the slope."""
    normal = np.array([0.0, -1.0, 1.0]) / np.sqrt(2.0)  # ground tilted about X
    verdict = bgg.resolve_heading_from_axes([np.array([0.0, 1.0, 0.0])], normal)

    assert verdict["ambiguous"] is False
    assert float(np.dot(verdict["heading"], normal)) == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# perpendicular_candidates + pick_best_candidate (the degenerate fallback)
# ---------------------------------------------------------------------------


def test_perpendicular_candidates_are_square_to_the_bar_and_flat():
    bar_axis = np.array([0.0, 1.0, 0.0])  # bar along world Y
    plus, minus = bgg.perpendicular_candidates(bar_axis, UP)

    np.testing.assert_allclose(plus, -minus, atol=1e-12)
    for candidate in (plus, minus):
        assert float(np.dot(candidate, bar_axis)) == pytest.approx(0.0, abs=1e-12)
        assert float(np.dot(candidate, UP)) == pytest.approx(0.0, abs=1e-12)
        assert float(np.linalg.norm(candidate)) == pytest.approx(1.0)


def test_upright_bar_has_no_distinguishable_sides():
    """A bar standing straight up: neither side is 'the' side."""
    assert bgg.perpendicular_candidates(UP, UP) is None


def test_open_side_wins_and_flips_when_the_obstruction_moves():
    """REGRESSION: the ambiguous case must be decided, and decided by geometry.

    Score +X generously and -X poorly -> +X wins; swap the scores and the answer
    must swap too. That is what makes the fallback a decision rather than the
    coin-flip the old noise vector amounted to.
    """
    candidates = bgg.perpendicular_candidates(np.array([0.0, 1.0, 0.0]), UP)
    plus = candidates[0]

    open_on_plus = bgg.pick_best_candidate(
        candidates, lambda h: 1800.0 if np.allclose(h, plus) else 310.0
    )[0]
    open_on_minus = bgg.pick_best_candidate(
        candidates, lambda h: 310.0 if np.allclose(h, plus) else 1800.0
    )[0]

    np.testing.assert_allclose(open_on_plus, plus, atol=1e-12)
    np.testing.assert_allclose(open_on_minus, -plus, atol=1e-12)


def test_a_side_off_the_ground_is_rejected_outright():
    """`None` means "the base cannot stand there", not "score of zero"."""
    candidates = bgg.perpendicular_candidates(np.array([0.0, 1.0, 0.0]), UP)
    plus = candidates[0]

    # +X is off the ground even though -X is barely clear: -X must still win.
    chosen, _scored = bgg.pick_best_candidate(
        candidates, lambda h: None if np.allclose(h, plus) else 5.0
    )
    np.testing.assert_allclose(chosen, -plus, atol=1e-12)


def test_no_viable_side_returns_none():
    candidates = bgg.perpendicular_candidates(np.array([0.0, 1.0, 0.0]), UP)
    chosen, _scored = bgg.pick_best_candidate(candidates, lambda _h: None)
    assert chosen is None


# ---------------------------------------------------------------------------
# arm_side_for_joint -- which end of the bar takes the LEFT tool
# ---------------------------------------------------------------------------


def test_arm_sides_match_the_worked_example():
    """Bar along world Y, robot on the +X side facing it (heading -X).

        [tool R]
           |
          bar        [robot]
           |
        [tool L]

    left = up x heading = -Y, so the -Y end takes the left tool.
    """
    heading = np.array([-1.0, 0.0, 0.0])
    bar_center = np.array([0.0, 0.0, 0.0])

    np.testing.assert_allclose(bgg.robot_left_dir(heading, UP), [0.0, -1.0, 0.0],
                               atol=1e-12)
    assert bgg.arm_side_for_joint([0.0, -500.0, 0.0], bar_center, heading, UP) == "left"
    assert bgg.arm_side_for_joint([0.0, 500.0, 0.0], bar_center, heading, UP) == "right"


def test_flipping_the_base_swaps_both_tool_sides():
    """The whole reason the sides are derived from the heading, not the bar."""
    bar_center = np.array([0.0, 0.0, 0.0])
    near = np.array([0.0, -500.0, 0.0])
    far = np.array([0.0, 500.0, 0.0])
    heading = np.array([-1.0, 0.0, 0.0])

    before = (bgg.arm_side_for_joint(near, bar_center, heading, UP),
              bgg.arm_side_for_joint(far, bar_center, heading, UP))
    after = (bgg.arm_side_for_joint(near, bar_center, -heading, UP),
             bgg.arm_side_for_joint(far, bar_center, -heading, UP))

    assert before == ("left", "right")
    assert after == ("right", "left")


def test_joint_on_the_fore_aft_axis_has_no_side():
    """Robot facing the bar end-on: neither arm has a distinct end to take."""
    heading = np.array([-1.0, 0.0, 0.0])
    bar_center = np.array([0.0, 0.0, 0.0])
    # Joint offset lies along the heading, so its projection onto `left` is 0.
    assert bgg.arm_side_for_joint([500.0, 0.0, 0.0], bar_center, heading, UP) is None


# ---------------------------------------------------------------------------
# build_base_guides -- the five lines
# ---------------------------------------------------------------------------


def _guides_on_flat_ground(offsets=(375.0, 500.0, 625.0)):
    """A bar along world Y, 1000 mm up, with the robot approaching from -X."""
    soups = [_flat_soup()]
    joint_a = np.array([0.0, -500.0, 1000.0])
    joint_b = np.array([0.0, 500.0, 1000.0])
    heading = np.array([1.0, 0.0, 0.0])
    return bgg.build_base_guides(soups, joint_a, joint_b, heading,
                                 offsets_mm=offsets), heading


@needs_compas
def test_guides_have_four_lines_plus_the_extension_line():
    guides, _heading = _guides_on_flat_ground()

    assert len(guides["lines"]) == 4          # projection + 3 offsets
    assert len(guides["extension"]) == 4      # one midpoint per line
    assert guides["offsets"] == [0.0, 375.0, 500.0, 625.0]


@needs_compas
def test_projection_drops_the_joints_onto_the_ground():
    """Only the PROJECTION is drawn -- the airborne joint line never is."""
    guides, _heading = _guides_on_flat_ground()
    start, end = guides["lines"][0]

    assert start[2] == pytest.approx(0.0, abs=TOL)   # was 1000 mm up
    assert end[2] == pytest.approx(0.0, abs=TOL)
    assert start[1] == pytest.approx(-500.0)         # ...but stays over the joints
    assert end[1] == pytest.approx(500.0)


@needs_compas
def test_each_offset_line_steps_against_the_heading():
    """The base stands BEHIND, so the guides march opposite the assembly dir."""
    guides, heading = _guides_on_flat_ground()
    base_start, _base_end = guides["lines"][0]

    for line, distance in zip(guides["lines"][1:], guides["offsets"][1:]):
        offset_vec = np.asarray(line[0]) - np.asarray(base_start)
        assert float(np.linalg.norm(offset_vec)) == pytest.approx(distance)
        # ...and in the -heading direction, not +heading.
        assert float(np.dot(offset_vec, heading)) == pytest.approx(-distance)


@needs_compas
def test_extension_line_is_parallel_to_the_heading():
    """The extension line is where the placed base origin sits."""
    guides, heading = _guides_on_flat_ground()
    span = np.asarray(guides["extension"][-1]) - np.asarray(guides["extension"][0])
    direction = span / np.linalg.norm(span)

    np.testing.assert_allclose(direction, -heading, atol=1e-12)


@needs_compas
def test_extension_points_are_midpoints_of_their_lines():
    guides, _heading = _guides_on_flat_ground()
    for (start, end), midpoint in zip(guides["lines"], guides["extension"]):
        np.testing.assert_allclose(midpoint, 0.5 * (np.asarray(start) + np.asarray(end)),
                                   atol=1e-12)


@needs_compas
def test_default_offsets_come_from_config():
    guides, _heading = _guides_on_flat_ground(offsets=None)
    assert guides["offsets"] == [0.0] + list(config.BASE_GUIDE_OFFSETS_MM)


def test_the_middle_guide_is_the_multibar_standoff():
    """The default auto-placed base must land ON a drawn guide line."""
    assert float(config.IK_BASE_STANDOFF_MULTIBAR_MM) in tuple(
        float(offset) for offset in config.BASE_GUIDE_OFFSETS_MM
    )


@needs_compas
def test_guides_follow_a_ground_that_steps_up():
    """Endpoints are re-snapped, so the guide sits ON a non-flat ground."""
    soups = [_flat_soup(), _flat_soup(size=6000.0, z=300.0)]
    joint_a = np.array([0.0, -500.0, 1000.0])
    joint_b = np.array([0.0, 500.0, 1000.0])
    guides = bgg.build_base_guides(soups, joint_a, joint_b,
                                   np.array([1.0, 0.0, 0.0]))

    # The raised slab is nearer the airborne joints, so the projection lands on it.
    assert guides["lines"][0][0][2] == pytest.approx(300.0, abs=TOL)


@needs_compas
def test_no_ground_yields_no_guides():
    """A bar with no assigned/meshable ground is skipped, not crashed on."""
    assert bgg.build_base_guides([], np.zeros(3), np.ones(3),
                                 np.array([1.0, 0.0, 0.0])) is None
