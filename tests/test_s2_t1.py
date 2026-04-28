import numpy as np

from core.geometry import (
    closest_params_infinite_lines,
    distance_infinite_lines,
    solve_s2_t1,
    solve_s2_t1_all,
    solve_s2_t1_report,
)


# Representative connector contact distance (mm) used purely as a numeric
# fixture for the S2-T1 solver tests.  The real per-pair contact distances
# are stored in ``scripts/core/joint_pairs.json``.
BAR_CONTACT_DISTANCE = 36.0

TOL = 1e-4


def _viz_s2_result(viz, n1, ce1, n2, ce2, nn, p1, p2, distance, title=""):
    viz.plot_line(ce1 - 60.0 * n1, ce1 + 60.0 * n1, color="grey", linewidth=4, label="Le1")
    viz.plot_line(ce2 - 60.0 * n2, ce2 + 60.0 * n2, color="dimgrey", linewidth=4, label="Le2")
    viz.plot_point(ce1, color="red", size=60, label="Ce1")
    viz.plot_point(ce2, color="blue", size=60, label="Ce2")
    mid = 0.5 * (p1 + p2)
    viz.plot_line(mid - 80.0 * nn, mid + 80.0 * nn, color="orange", linewidth=4, label="Ln (result)")
    viz.plot_point(p1, color="salmon", size=40, label="P1 (on Ln)")
    viz.plot_point(p2, color="cornflowerblue", size=40, label="P2 (on Ln)")
    viz.plot_segment(ce1, p1, color="red", label=f"d1={np.linalg.norm(p1 - ce1):.2f}")
    viz.plot_segment(ce2, p2, color="blue", label=f"d2={np.linalg.norm(p2 - ce2):.2f}")
    viz.set_title(f"{title} | D={distance:.2f}")


def _viz_s2_all_solutions(viz, n1, ce1, n2, ce2, solutions, distance, title=""):
    viz.plot_line(ce1 - 60.0 * n1, ce1 + 60.0 * n1, color="grey", linewidth=4, label="Le1")
    viz.plot_line(ce2 - 60.0 * n2, ce2 + 60.0 * n2, color="dimgrey", linewidth=4, label="Le2")
    viz.plot_point(ce1, color="red", size=60, label="Ce1")
    viz.plot_point(ce2, color="blue", size=60, label="Ce2")
    sol_colors = ("orange", "cyan", "lime", "magenta")
    for index, solution in enumerate(solutions):
        mid = 0.5 * (solution["p1"] + solution["p2"])
        nn = solution["nn"]
        color = sol_colors[index % len(sol_colors)]
        theta1, theta2 = solution["angles"]
        viz.plot_line(
            mid - 80.0 * nn,
            mid + 80.0 * nn,
            color=color,
            linewidth=3,
            label=f"Sol {index + 1} ang=({theta1:.2f},{theta2:.2f}) res={solution['residual']:.1e}",
        )
        viz.plot_segment(ce1, solution["p1"], color=color, linestyle=":")
        viz.plot_segment(ce2, solution["p2"], color=color, linestyle=":")
    viz.set_title(f"{title} | D={distance:.2f}")


class TestS2T1:
    def _verify_s2_result(self, nn, p1, p2, n1, ce1, n2, ce2, distance):
        diff = p2 - p1
        if np.linalg.norm(diff) > 1e-10:
            diff_unit = diff / np.linalg.norm(diff)
            assert abs(abs(np.dot(diff_unit, nn)) - 1.0) < TOL

        d1 = abs(distance_infinite_lines(p1, nn, ce1, n1))
        d2 = abs(distance_infinite_lines(p1, nn, ce2, n2))
        assert abs(d1 - distance) < TOL
        assert abs(d2 - distance) < TOL

        t_a, _ = closest_params_infinite_lines(ce1, n1, p1, nn)
        closest_on_le1 = ce1 + t_a * n1
        assert np.linalg.norm(closest_on_le1 - ce1) < TOL

        t_a, _ = closest_params_infinite_lines(ce2, n2, p1, nn)
        closest_on_le2 = ce2 + t_a * n2
        assert np.linalg.norm(closest_on_le2 - ce2) < TOL

    def test_perpendicular_existing_bars(self, viz):
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        nn, p1, p2 = solve_s2_t1(n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        self._verify_s2_result(nn, p1, p2, n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        _viz_s2_result(viz, n1, ce1, n2, ce2, nn, p1, p2, BAR_CONTACT_DISTANCE, title="test_perpendicular_existing_bars")

    def test_parallel_existing_bars(self, viz):
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([10.0, 0.0, 0.0])
        n2 = np.array([1.0, 0.0, 0.0])
        ce2 = np.array([20.0, 5.0, 0.0])
        nn, p1, p2 = solve_s2_t1(n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        self._verify_s2_result(nn, p1, p2, n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        assert abs(np.dot(nn, n1 / np.linalg.norm(n1))) < 0.99
        _viz_s2_result(viz, n1, ce1, n2, ce2, nn, p1, p2, BAR_CONTACT_DISTANCE, title="test_parallel_existing_bars")

    def test_3d_skew_bars(self, viz):
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([15.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)
        ce2 = np.array([0.0, 10.0, 10.0])
        nn, p1, p2 = solve_s2_t1(n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        self._verify_s2_result(nn, p1, p2, n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        _viz_s2_result(viz, n1, ce1, n2, ce2, nn, p1, p2, BAR_CONTACT_DISTANCE, title="test_3d_skew_bars")

    def test_output_direction_is_unit(self):
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        nn, _, _ = solve_s2_t1(n1, ce1, n2, ce2, BAR_CONTACT_DISTANCE)
        assert abs(np.linalg.norm(nn) - 1.0) < 1e-10


class TestS2T1AllSolutions:
    def _verify_s2_result(self, nn, p1, p2, n1, ce1, n2, ce2, distance):
        d1 = abs(distance_infinite_lines(p1, nn, ce1, n1))
        d2 = abs(distance_infinite_lines(p1, nn, ce2, n2))
        assert abs(d1 - distance) < 1e-3
        assert abs(d2 - distance) < 1e-3

        t_a, _ = closest_params_infinite_lines(ce1, n1, p1, nn)
        closest_on_le1 = ce1 + t_a * n1
        assert np.linalg.norm(closest_on_le1 - ce1) < 1e-3

        t_a, _ = closest_params_infinite_lines(ce2, n2, p1, nn)
        closest_on_le2 = ce2 + t_a * n2
        assert np.linalg.norm(closest_on_le2 - ce2) < 1e-3

    def test_returns_list(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        solutions = solve_s2_t1_all(n1, ce1, n2, ce2, distance)
        assert isinstance(solutions, list)
        assert len(solutions) >= 1

    def test_each_solution_valid(self, viz):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        solutions = solve_s2_t1_all(n1, ce1, n2, ce2, distance)
        for solution in solutions:
            assert "nn" in solution and "p1" in solution and "p2" in solution
            assert "residual" in solution and "angles" in solution
            assert len(solution["angles"]) == 2
            assert all(np.isfinite(angle) for angle in solution["angles"])
            assert abs(np.linalg.norm(solution["nn"]) - 1.0) < 1e-8
            self._verify_s2_result(solution["nn"], solution["p1"], solution["p2"], n1, ce1, n2, ce2, distance)
        _viz_s2_all_solutions(viz, n1, ce1, n2, ce2, solutions, distance, title="test_each_solution_valid")

    def test_sorted_by_residual(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        solutions = solve_s2_t1_all(n1, ce1, n2, ce2, distance)
        residuals = [solution["residual"] for solution in solutions]
        assert residuals == sorted(residuals)

    def test_solutions_are_distinct(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])
        solutions = solve_s2_t1_all(n1, ce1, n2, ce2, distance)
        if len(solutions) >= 2:
            mid0 = 0.5 * (solutions[0]["p1"] + solutions[0]["p2"])
            mid1 = 0.5 * (solutions[1]["p1"] + solutions[1]["p2"])
            dir_diff = 1.0 - abs(np.dot(solutions[0]["nn"], solutions[1]["nn"]))
            pos_diff = np.linalg.norm(mid0 - mid1)
            assert dir_diff > 1e-3 or pos_diff > 1e-3

    def test_perpendicular_case_has_four_geometric_branches(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])

        solutions = solve_s2_t1_all(n1, ce1, n2, ce2, distance)

        assert len(solutions) == 4
        midpoints = [0.5 * (solution["p1"] + solution["p2"]) for solution in solutions]
        assert sum(mid[2] > 1e-3 for mid in midpoints) == 2
        assert sum(mid[2] < -1e-3 for mid in midpoints) == 2
        angle_keys = {
            (round(solution["angles"][0], 6), round(solution["angles"][1], 6))
            for solution in solutions
        }
        assert len(angle_keys) == 4

    def test_report_includes_sign_family_diagnostics(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([1.0, 0.0, 0.0])
        ce1 = np.array([5.0, 0.0, 0.0])
        n2 = np.array([0.0, 1.0, 0.0])
        ce2 = np.array([0.0, 8.0, 0.0])

        report = solve_s2_t1_report(n1, ce1, n2, ce2, distance)

        assert report["solution_count"] == 4
        assert report["optimizer_settings"]["max_nfev"] == 10000
        assert report["found_sign_families"] == ["(+,+)", "(-,+)", "(+,-)", "(-,-)"]
        assert report["missing_sign_families"] == []
        assert report["missing_family_attempts"] == []
        for solution in report["solutions"]:
            assert solution["sign_family"] in report["found_sign_families"]
            diagnostics = solution["optimization_diagnostics"]
            optimizer_result = solution["optimizer_result"]
            assert diagnostics["contact_1_objective_component"] >= 0.0
            assert diagnostics["contact_2_objective_component"] >= 0.0
            assert diagnostics["contact_1_orthogonality_error_deg"] < 1e-4
            assert diagnostics["contact_2_orthogonality_error_deg"] < 1e-4
            assert optimizer_result["success"] is True
            assert optimizer_result["termination_reason"] in {"ftol", "gtol", "xtol"}

    def test_report_marks_missing_sign_families_for_two_solution_case(self):
        distance = BAR_CONTACT_DISTANCE
        n1 = np.array([-1.3073986337985843e-12, -9.200817885357537e-10, -150.5276928808155])
        ce1 = np.array([-196.0606314462008, 116.00000000029755, -8.435536257997462])
        n2 = np.array([199.99999999999983, 5.684341886080802e-14, -6.963318810448982e-13])
        ce2 = np.array([-282.61053931669187, 69.3067692961276, -2.9364342132458403e-10])

        report = solve_s2_t1_report(n1, ce1, n2, ce2, distance, nn_init_hint=ce2 - ce1)

        assert report["solution_count"] == 2
        assert report["found_sign_families"] == ["(+,+)", "(+,-)"]
        assert report["missing_sign_families"] == ["(-,+)", "(-,-)"]
        assert len(report["missing_family_attempts"]) == 2
        attempt_targets = [attempt["target_sign_family"] for attempt in report["missing_family_attempts"]]
        assert attempt_targets == report["missing_sign_families"]
        for solution in report["solutions"]:
            diagnostics = solution["optimization_diagnostics"]
            assert solution["sign_family"] in report["found_sign_families"]
            assert diagnostics["contact_1_orthogonality_error_deg"] < 1e-4
            assert diagnostics["contact_2_orthogonality_error_deg"] < 1e-4
        for attempt in report["missing_family_attempts"]:
            diagnostics = attempt["optimization_diagnostics"]
            assert attempt["objective"] >= 0.0
            assert attempt["converged_sign_family"] is not None
            assert attempt["termination_reason"] in {"ftol", "gtol", "xtol", "max_nfev", "other", "failure", "status_0"}
            assert attempt["nfev"] <= report["optimizer_settings"]["max_nfev"]
            assert max(
                diagnostics["contact_1_orthogonality_error_deg"],
                diagnostics["contact_2_orthogonality_error_deg"],
            ) > 1.0
