import numpy as np
import pytest

from core.geometry import (
    are_lines_parallel,
    closest_params_finite_segments,
    closest_params_infinite_lines,
    distance_infinite_lines,
)


TOL = 1e-10


def _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_i, t_j, title=""):
    extent = max(abs(t_i), abs(t_j), 5.0) * 1.5
    viz.plot_line(x_i - extent * n_i, x_i + extent * n_i, color="grey", linewidth=3, label="Line A")
    viz.plot_line(x_j - extent * n_j, x_j + extent * n_j, color="orange", linewidth=3, label="Line B")
    p_a = x_i + t_i * n_i
    p_b = x_j + t_j * n_j
    viz.plot_point(p_a, color="red", size=60, label=f"Closest on A (t={t_i:.2f})")
    viz.plot_point(p_b, color="blue", size=60, label=f"Closest on B (t={t_j:.2f})")
    viz.plot_segment(p_a, p_b, color="magenta", label=f"Dist={np.linalg.norm(p_a - p_b):.3f}")
    viz.plot_point(x_i, color="grey", size=20, label="Anchor A")
    viz.plot_point(x_j, color="orange", size=20, label="Anchor B")
    viz.set_title(title)


def _viz_finite_segments(viz, p1_start, p1_end, p2_start, p2_end, t1, t2, title=""):
    d1 = p1_end - p1_start
    d2 = p2_end - p2_start
    viz.plot_line(p1_start, p1_end, color="grey", linewidth=4, label="Segment 1")
    viz.plot_line(p2_start, p2_end, color="orange", linewidth=4, label="Segment 2")
    cp1 = p1_start + t1 * d1
    cp2 = p2_start + t2 * d2
    viz.plot_point(cp1, color="red", size=60, label=f"Closest on S1 (t={t1:.2f})")
    viz.plot_point(cp2, color="blue", size=60, label=f"Closest on S2 (t={t2:.2f})")
    viz.plot_segment(cp1, cp2, color="magenta", label=f"Dist={np.linalg.norm(cp1 - cp2):.3f}")
    for point in (p1_start, p1_end):
        viz.plot_point(point, color="grey", size=15)
    for point in (p2_start, p2_end):
        viz.plot_point(point, color="orange", size=15)
    viz.set_title(title)


class TestClosestParamsInfiniteLines:
    def test_perpendicular_at_origins(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([0.0, 0.0, 5.0])
        n_j = np.array([0.0, 1.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a) < TOL
        assert abs(t_b) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_perpendicular_at_origins")

    def test_skew_lines_offset(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([1.0, 2.0, 3.0])
        n_j = np.array([0.0, 1.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a - 1.0) < TOL
        assert abs(t_b + 2.0) < TOL
        p_a = x_i + t_a * n_i
        p_b = x_j + t_b * n_j
        np.testing.assert_allclose(p_a, [1.0, 0.0, 0.0], atol=TOL)
        np.testing.assert_allclose(p_b, [1.0, 0.0, 3.0], atol=TOL)
        assert abs(np.linalg.norm(p_a - p_b) - 3.0) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_skew_lines_offset")

    def test_45_degree_lines(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([5.0, 3.0, 4.0])
        n_j = np.array([1.0, 1.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a - 2.0) < TOL
        assert abs(t_b + 3.0) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_45_degree_lines")

    def test_symmetric_config(self, viz):
        distance = 20.0
        x_i = np.array([0.0, 0.0, -distance / 2.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([0.0, 0.0, distance / 2.0])
        n_j = np.array([0.0, 1.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a) < TOL
        assert abs(t_b) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_symmetric_config")

    def test_parallel_raises(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([0.0, 1.0, 0.0])
        n_j = np.array([2.0, 0.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a) < TOL
        assert abs(t_b) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_parallel_raises")

    def test_parallel_shifted_along_axis(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([2.0, 0.0, 0.0])
        x_j = np.array([6.0, 3.0, 0.0])
        n_j = np.array([4.0, 0.0, 0.0])
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(t_a - 3.0) < TOL
        assert abs(t_b) < TOL
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_parallel_shifted_along_axis")


class TestDistanceInfiniteLines:
    def test_perpendicular_z_separated(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([0.0, 0.0, 5.0])
        n_j = np.array([0.0, 1.0, 0.0])
        distance = distance_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(abs(distance) - 5.0) < TOL
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_perpendicular_z_separated")

    def test_skew_offset(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([1.0, 0.0, 0.0])
        x_j = np.array([1.0, 2.0, 3.0])
        n_j = np.array([0.0, 1.0, 0.0])
        distance = distance_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(distance + 3.0) < TOL
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_skew_offset")

    def test_non_unit_directions(self, viz):
        x_i = np.array([0.0, 0.0, 0.0])
        n_i = np.array([5.0, 0.0, 0.0])
        x_j = np.array([0.0, 0.0, 7.0])
        n_j = np.array([0.0, 3.0, 0.0])
        distance = distance_infinite_lines(x_i, n_i, x_j, n_j)
        assert abs(abs(distance) - 7.0) < TOL
        t_a, t_b = closest_params_infinite_lines(x_i, n_i, x_j, n_j)
        _viz_infinite_lines(viz, x_i, n_i, x_j, n_j, t_a, t_b, title="test_non_unit_directions")

    def test_parallel_raises(self):
        distance = distance_infinite_lines(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
        )
        assert abs(abs(distance) - 1.0) < TOL

    def test_parallel_coincident_lines(self):
        distance = distance_infinite_lines(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([5.0, 0.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
        )
        assert abs(distance) < TOL


class TestClosestParamsFiniteSegments:
    def test_interior_solution(self, viz):
        p1_start = np.array([0.0, 0.0, 0.0])
        p1_end = np.array([4.0, 0.0, 0.0])
        p2_start = np.array([2.0, 0.0, 3.0])
        p2_end = np.array([2.0, 4.0, 3.0])
        t1, t2 = closest_params_finite_segments(p1_start, p1_end, p2_start, p2_end)
        assert abs(t1 - 0.5) < TOL
        assert abs(t2 - 0.0) < TOL
        _viz_finite_segments(viz, p1_start, p1_end, p2_start, p2_end, t1, t2, title="test_interior_solution")

    def test_boundary_clipping(self, viz):
        p1_start = np.array([0.0, 0.0, 0.0])
        p1_end = np.array([1.0, 0.0, 0.0])
        p2_start = np.array([5.0, 0.0, 2.0])
        p2_end = np.array([5.0, 4.0, 2.0])
        t1, t2 = closest_params_finite_segments(p1_start, p1_end, p2_start, p2_end)
        assert abs(t1 - 1.0) < TOL
        assert abs(t2 - 0.0) < TOL
        p1 = p1_start + t1 * (p1_end - p1_start)
        p2 = p2_start + t2 * (p2_end - p2_start)
        np.testing.assert_allclose(p2, [5.0, 0.0, 2.0], atol=TOL)
        assert abs(np.linalg.norm(p1 - p2) - np.sqrt(20.0)) < 1e-6
        _viz_finite_segments(viz, p1_start, p1_end, p2_start, p2_end, t1, t2, title="test_boundary_clipping")

    def test_coincides_with_infinite_when_inside(self, viz):
        x1 = np.array([0.0, 0.0, 0.0])
        n1 = np.array([1.0, 0.0, 0.0])
        x2 = np.array([0.0, 0.0, 5.0])
        n2 = np.array([0.0, 1.0, 0.0])
        t_a_inf, t_b_inf = closest_params_infinite_lines(x1, n1, x2, n2)
        assert abs(t_a_inf) < TOL
        assert abs(t_b_inf) < TOL

        t1, t2 = closest_params_finite_segments(x1 - 10 * n1, x1 + 10 * n1, x2 - 10 * n2, x2 + 10 * n2)
        assert abs(t1 - 0.5) < TOL
        assert abs(t2 - 0.5) < TOL
        _viz_finite_segments(
            viz,
            x1 - 10 * n1,
            x1 + 10 * n1,
            x2 - 10 * n2,
            x2 + 10 * n2,
            t1,
            t2,
            title="test_coincides_with_infinite_when_inside",
        )

    def test_degenerate_segment_raises(self):
        with pytest.raises(ValueError):
            closest_params_finite_segments(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.0]),
                np.array([1.0, 0.0, 0.0]),
                np.array([2.0, 0.0, 0.0]),
            )


class TestAreLinesParallel:
    def test_parallel(self):
        assert are_lines_parallel(np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]))
        assert are_lines_parallel(np.array([1.0, 0.0, 0.0]), np.array([-3.0, 0.0, 0.0]))

    def test_not_parallel(self):
        assert not are_lines_parallel(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))
        assert not are_lines_parallel(np.array([1.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0]))

    def test_near_parallel(self):
        assert are_lines_parallel(np.array([1.0, 0.0, 0.0]), np.array([1.0, 1e-8, 0.0]))
