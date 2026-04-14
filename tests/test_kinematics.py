import numpy as np

from core import config
from core.kinematics import (
    fk_female,
    fk_female_side,
    fk_male,
    fk_male_side,
    frame_distance,
    optimize_joint_placement,
    perpendicular_to,
)


TOL_POSITION = 1e-4
TOL_ORIENTATION = 1e-4
TOL_RESIDUAL = 1e-4


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


class TestCADTransforms:
    def test_fixed_rotations_are_inverses(self):
        product = config.FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM[:3, :3] @ config.MALE_FIXED_ROT_TO_BAR_TRANSFORM[:3, :3]
        np.testing.assert_allclose(product, np.eye(3), atol=TOL_ORIENTATION)

    def test_gap_transform_is_translation_only(self):
        np.testing.assert_allclose(
            config.FEMALE_MALE_GAP_OFFSET_TRANSFORM[:3, :3],
            np.eye(3),
            atol=TOL_ORIENTATION,
        )

    def test_male_screw_hole_offset_is_quarter_turn_about_local_z(self):
        expected = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        np.testing.assert_allclose(
            config.MALE_SCREW_HOLE_OFFSET_TRANSFORM[:3, :3],
            expected,
            atol=TOL_ORIENTATION,
        )


class TestFKSanity:
    def test_fk_female_origin_lies_on_bar_axis(self):
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([0.0, 0.0, 200.0])
        frame = fk_female(le_start, le_end, 50.0, 0.0, 0.0, config)
        np.testing.assert_allclose(frame[:3, 3], np.array([0.0, 0.0, 50.0]), atol=TOL_POSITION)

    def test_fk_male_origin_lies_on_bar_axis(self):
        ln_start = np.array([0.0, -config.BAR_CONTACT_DISTANCE, 0.0])
        ln_end = np.array([0.0, -config.BAR_CONTACT_DISTANCE, 200.0])
        frame = fk_male(ln_start, ln_end, -125.0, 0.0, config)
        np.testing.assert_allclose(
            frame[:3, 3],
            np.array([0.0, -config.BAR_CONTACT_DISTANCE, 125.0]),
            atol=TOL_POSITION,
        )

    def test_fk_female_x_axis_along_bar(self):
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([200.0, 0.0, 0.0])
        frame = fk_female(le_start, le_end, 100.0, 0.0, 0.0, config)
        bar_unit = np.array([1.0, 0.0, 0.0])
        assert abs(abs(np.dot(frame[:3, 0], bar_unit)) - 1.0) < TOL_ORIENTATION

    def test_fk_male_x_axis_along_bar(self):
        ln_start = np.array([0.0, 0.0, 0.0])
        ln_end = np.array([0.0, 200.0, 0.0])
        frame = fk_male(ln_start, ln_end, 50.0, 0.0, config)
        bar_unit = np.array([0.0, 1.0, 0.0])
        assert abs(abs(np.dot(frame[:3, 0], bar_unit)) - 1.0) < TOL_ORIENTATION

    def test_fjr_rotates_female_contact_direction(self):
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([0.0, 0.0, 200.0])
        state_0 = fk_female_side(le_start, le_end, 100.0, 0.0, config)
        state_pi = fk_female_side(le_start, le_end, 100.0, np.pi, config)
        offset_0 = state_0["female_screw_hole_frame"][:3, 3] - state_0["female_frame"][:3, 3]
        offset_pi = state_pi["female_screw_hole_frame"][:3, 3] - state_pi["female_frame"][:3, 3]
        np.testing.assert_allclose(np.linalg.norm(offset_0), config.BAR_CONTACT_DISTANCE, atol=TOL_POSITION)
        cosine = np.dot(offset_0, offset_pi) / (np.linalg.norm(offset_0) * np.linalg.norm(offset_pi))
        assert cosine < -0.99

    def test_male_side_bar_axis_matches_mjr_frame_z(self):
        ln_start = np.array([0.0, 0.0, 0.0])
        ln_end = np.array([150.0, 150.0, 0.0])
        state = fk_male_side(ln_start, ln_end, 75.0, 0.0, config)
        bar_unit = (ln_end - ln_start) / np.linalg.norm(ln_end - ln_start)
        assert abs(abs(np.dot(state["mjr_frame"][:3, 2], bar_unit)) - 1.0) < TOL_ORIENTATION


class TestOptimizeJointPlacement:
    def test_parallel_bars_at_contact_distance(self):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([0.0, 0.0, 400.0])
        ln_start = np.array([0.0, -distance, 0.0])
        ln_end = np.array([0.0, -distance, 400.0])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["residual"] < TOL_RESIDUAL
        np.testing.assert_allclose(
            result["predicted_male_screw_hole_frame"],
            result["male_screw_hole_frame"],
            atol=1e-3,
        )

    def test_perpendicular_bars_at_contact_distance(self):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([0.0, 0.0, 400.0])
        ln_start = np.array([-200.0, -distance, 200.0])
        ln_end = np.array([200.0, -distance, 200.0])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["residual"] < TOL_RESIDUAL
        np.testing.assert_allclose(
            result["predicted_male_screw_hole_frame"],
            result["male_screw_hole_frame"],
            atol=1e-3,
        )

    def test_dof_values_within_bounds(self):
        distance = config.BAR_CONTACT_DISTANCE
        le_start = np.array([0.0, 0.0, 0.0])
        le_end = np.array([0.0, 0.0, 400.0])
        ln_start = np.array([0.0, -distance, 0.0])
        ln_end = np.array([0.0, -distance, 400.0])

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert config.FJP_RANGE[0] <= result["fjp"] <= config.FJP_RANGE[1]
        assert config.FJR_RANGE[0] <= result["fjr"] <= config.FJR_RANGE[1]
        assert config.MJP_RANGE[0] <= result["mjp"] <= config.MJP_RANGE[1]
        assert config.MJR_RANGE[0] <= result["mjr"] <= config.MJR_RANGE[1]
        assert config.JJR_RANGE[0] <= result["jjr"] <= config.JJR_RANGE[1]
