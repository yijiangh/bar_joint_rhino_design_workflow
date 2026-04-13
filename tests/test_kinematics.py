import numpy as np

from core import config
from core.kinematics import (
    fk_female,
    fk_male,
    frame_distance,
    optimize_joint_placement,
    perpendicular_to,
)


TOL_POSITION = 1e-4
TOL_ORIENTATION = 1e-4
TOL_RESIDUAL = 1e-3


def _viz_fk_result(viz, bar_start, bar_end, frame, bar_color="grey", frame_label="OCF", title=""):
    viz.plot_line(bar_start, bar_end, color=bar_color, linewidth=5, label=f"Bar ({bar_color})")
    viz.plot_frame(frame, scale=15, label=frame_label)
    bar_dir = bar_end - bar_start
    bar_unit = bar_dir / np.linalg.norm(bar_dir)
    viz.plot_line(
        bar_start - 20.0 * bar_unit,
        bar_end + 20.0 * bar_unit,
        color=bar_color,
        linewidth=0.5,
        linestyle="--",
    )
    viz.set_title(title)


def _viz_optimization_result(viz, le_start, le_end, ln_start, ln_end, result, title=""):
    viz.plot_line(le_start, le_end, color="grey", linewidth=5, label="Le (existing)")
    viz.plot_line(ln_start, ln_end, color="orange", linewidth=5, label="Ln (new)")
    viz.plot_frame(result["female_frame"], scale=12, label="Female OCF")
    viz.plot_frame(result["male_frame"], scale=12, label="Male OCF")
    female_origin = result["female_frame"][:3, 3]
    male_origin = result["male_frame"][:3, 3]
    viz.plot_segment(female_origin, male_origin, color="purple", label=f"Residual={result['residual']:.2e}")
    viz.plot_text(
        female_origin + np.array([0.0, 5.0, 0.0]),
        f"FJP={result['fjp']:.1f} FJR={np.degrees(result['fjr']):.0f}deg",
        fontsize=8,
    )
    viz.plot_text(
        male_origin + np.array([0.0, 5.0, 0.0]),
        f"MJP={result['mjp']:.1f} MJR={np.degrees(result['mjr']):.0f}deg",
        fontsize=8,
    )
    viz.set_title(title)


class TestPerpendicularTo:
    def test_result_is_unit(self):
        for z_axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 1, 1], [0.1, 0.2, 0.95]):
            vector = perpendicular_to(np.array(z_axis, dtype=float))
            assert abs(np.linalg.norm(vector) - 1.0) < 1e-12

    def test_result_is_perpendicular(self):
        for z_axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 1, 1]):
            z_axis = np.array(z_axis, dtype=float)
            vector = perpendicular_to(z_axis)
            assert abs(np.dot(vector, z_axis / np.linalg.norm(z_axis))) < 1e-12

    def test_deterministic(self):
        z_axis = np.array([0.3, 0.5, 0.7])
        np.testing.assert_array_equal(perpendicular_to(z_axis), perpendicular_to(z_axis))


class TestFrameDistance:
    def test_identical_frames(self):
        frame = np.eye(4)
        assert frame_distance(frame, frame) < 1e-15

    def test_translation_only(self):
        frame_a = np.eye(4)
        frame_b = np.eye(4)
        frame_b[:3, 3] = [3.0, 4.0, 0.0]
        assert abs(frame_distance(frame_a, frame_b) - 25.0) < 1e-12

    def test_rotation_only(self):
        frame_a = np.eye(4)
        frame_b = np.eye(4)
        frame_b[:3, 0] = [0.0, 1.0, 0.0]
        frame_b[:3, 1] = [-1.0, 0.0, 0.0]
        frame_b[:3, 2] = [0.0, 0.0, 1.0]
        assert abs(frame_distance(frame_a, frame_b) - 4.0) < 1e-12


class TestFKSanity:
    def test_fk_female_origin_on_correct_side(self, viz):
        le_start = np.array([-100.0, 0.0, 0.0])
        le_end = np.array([100.0, 0.0, 0.0])
        frame = fk_female(le_start, le_end, 50.0, 0.0, 0.0, config)
        assert abs(frame[0, 3] - (-50.0)) < TOL_POSITION
        bar_axis_point = le_start + (50.0 / np.linalg.norm(le_end - le_start)) * (le_end - le_start)
        radial_distance = np.linalg.norm(frame[:3, 3] - bar_axis_point)
        assert abs(radial_distance - config.FEMALE_RADIAL_OFFSET) < TOL_POSITION
        _viz_fk_result(
            viz,
            le_start,
            le_end,
            frame,
            bar_color="grey",
            frame_label="Female OCF",
            title="test_fk_female_origin_on_correct_side",
        )

    def test_fk_male_origin_on_correct_side(self, viz):
        ln_start = np.array([0.0, -100.0, 20.0])
        ln_end = np.array([0.0, 100.0, 20.0])
        frame = fk_male(ln_start, ln_end, 100.0, 0.0, config)
        bar_axis_point = ln_start + (100.0 / np.linalg.norm(ln_end - ln_start)) * (ln_end - ln_start)
        radial_distance = np.linalg.norm(frame[:3, 3] - bar_axis_point)
        assert abs(radial_distance - config.MALE_RADIAL_OFFSET) < TOL_POSITION
        _viz_fk_result(
            viz,
            ln_start,
            ln_end,
            frame,
            bar_color="orange",
            frame_label="Male OCF",
            title="test_fk_male_origin_on_correct_side",
        )

    def test_fk_female_x_axis_along_bar(self, viz):
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        frame = fk_female(le_start, le_end, 100.0, 0.0, 0.0, config)
        bar_unit = np.array([1.0, 0.0, 0.0])
        assert abs(abs(np.dot(frame[:3, 0], bar_unit)) - 1.0) < TOL_ORIENTATION
        _viz_fk_result(
            viz,
            le_start,
            le_end,
            frame,
            bar_color="grey",
            frame_label="Female OCF",
            title="test_fk_female_x_axis_along_bar",
        )

    def test_fjr_rotates_radial_direction(self, viz):
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        frame_0 = fk_female(le_start, le_end, 100.0, 0.0, 0.0, config)
        frame_pi = fk_female(le_start, le_end, 100.0, np.pi, 0.0, config)
        bar_point = np.array([100.0, 0.0, 0.0])
        offset_0 = frame_0[:3, 3] - bar_point
        offset_pi = frame_pi[:3, 3] - bar_point
        cosine = np.dot(offset_0, offset_pi) / (np.linalg.norm(offset_0) * np.linalg.norm(offset_pi))
        assert cosine < -0.99
        viz.plot_line(le_start, le_end, color="grey", linewidth=5, label="Le")
        viz.plot_frame(frame_0, scale=15, label="FJR=0")
        viz.plot_frame(frame_pi, scale=15, label="FJR=pi")
        viz.plot_point(bar_point, color="black", size=30, label="Bar contact pt")
        viz.set_title("test_fjr_rotates_radial_direction")


class TestOptimizeJointPlacement:
    def test_perpendicular_bars_at_contact_distance(self, viz):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([-200.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        ln_start = np.array([0.0, -200.0, distance])
        ln_end = np.array([0.0, 200.0, distance])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["residual"] < TOL_RESIDUAL
        np.testing.assert_allclose(result["female_frame"][:3, 3], result["male_frame"][:3, 3], atol=TOL_POSITION)
        _viz_optimization_result(
            viz,
            le_start,
            le_end,
            ln_start,
            ln_end,
            result,
            title="test_perpendicular_bars_at_contact_distance",
        )

    def test_angled_bars(self, viz):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([-200.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        ln_dir = np.array([0.5, 0.0, np.sqrt(3.0) / 2.0])
        common_normal = np.cross(np.array([1.0, 0.0, 0.0]), ln_dir)
        common_normal = common_normal / np.linalg.norm(common_normal)
        ln_mid = distance * common_normal
        ln_start = ln_mid - 200.0 * ln_dir
        ln_end = ln_mid + 200.0 * ln_dir

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["residual"] < TOL_RESIDUAL
        _viz_optimization_result(viz, le_start, le_end, ln_start, ln_end, result, title="test_angled_bars")

    def test_skew_bars_3d(self, viz):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([-200.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        ln_dir = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)
        common_normal = np.cross(np.array([1.0, 0.0, 0.0]), ln_dir)
        common_normal = common_normal / np.linalg.norm(common_normal)
        closest_point_on_le = np.array([30.0, 0.0, 0.0])
        closest_point_on_ln = closest_point_on_le - distance * common_normal
        ln_start = closest_point_on_ln - 200.0 * ln_dir
        ln_end = closest_point_on_ln + 200.0 * ln_dir

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["residual"] < TOL_RESIDUAL
        _viz_optimization_result(viz, le_start, le_end, ln_start, ln_end, result, title="test_skew_bars_3d")

    def test_dof_values_within_bounds(self, viz):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([-200.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        ln_start = np.array([0.0, -200.0, distance])
        ln_end = np.array([0.0, 200.0, distance])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert config.FJP_RANGE[0] <= result["fjp"] <= config.FJP_RANGE[1]
        assert config.FJR_RANGE[0] <= result["fjr"] <= config.FJR_RANGE[1]
        assert config.MJP_RANGE[0] <= result["mjp"] <= config.MJP_RANGE[1]
        assert config.MJR_RANGE[0] <= result["mjr"] <= config.MJR_RANGE[1]
        assert config.JJR_RANGE[0] <= result["jjr"] <= config.JJR_RANGE[1]
        _viz_optimization_result(viz, le_start, le_end, ln_start, ln_end, result, title="test_dof_values_within_bounds")

    def test_fjp_mjp_near_contact_point(self, viz):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([-200.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        ln_start = np.array([0.0, -200.0, distance])
        ln_end = np.array([0.0, 200.0, distance])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert abs(result["fjp"] - 200.0) < 5.0
        assert abs(result["mjp"] - 200.0) < 5.0
        _viz_optimization_result(viz, le_start, le_end, ln_start, ln_end, result, title="test_fjp_mjp_near_contact_point")
