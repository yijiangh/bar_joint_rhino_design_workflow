import numpy as np

from core import config
from core.kinematics import (
    fk_female,
    fk_female_side,
    fk_female_side_from_lines,
    fk_male,
    fk_male_side,
    fk_male_side_from_lines,
    frame_distance,
    optimize_joint_placement,
    perpendicular_to,
    screw_hole_alignment_diagnostics,
    screw_hole_origin_z_error,
)
from core.transforms import rotation_about_local_z, translation_transform


TOL_POSITION = 1e-4
TOL_ORIENTATION = 1e-4


def _frame(origin=(0.0, 0.0, 0.0), rotation=None) -> np.ndarray:
    frame = np.eye(4, dtype=float)
    frame[:3, 3] = np.asarray(origin, dtype=float)
    if rotation is not None:
        frame[:3, :3] = np.asarray(rotation, dtype=float)
    return frame


def _line_from_bar_frame(bar_frame: np.ndarray, length_mm: float = 400.0) -> tuple[np.ndarray, np.ndarray]:
    start = np.asarray(bar_frame[:3, 3], dtype=float)
    end = start + float(length_mm) * np.asarray(bar_frame[:3, 2], dtype=float)
    return start, end


def _assembled_bar_frames(
    *,
    fjp: float,
    fjr: float,
    jjr: float,
    mjr: float,
    mjp: float,
) -> tuple[np.ndarray, np.ndarray]:
    le_bar_frame = np.eye(4, dtype=float)
    female_side = fk_female_side(le_bar_frame, fjp, fjr, config)
    male_screw_hole_frame = (
        female_side["female_screw_hole_frame"]
        @ np.asarray(config.JJR_ZERO_TRANSFORM, dtype=float)
        @ rotation_about_local_z(float(jjr))
    )
    male_frame = male_screw_hole_frame @ np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float)
    mjr_frame = male_frame @ np.asarray(config.MALE_FIXED_ROT_TO_BAR_TRANSFORM, dtype=float)
    mjp_frame = mjr_frame @ rotation_about_local_z(float(mjr))
    ln_bar_frame = mjp_frame @ translation_transform((0.0, 0.0, float(mjp)))
    return le_bar_frame, ln_bar_frame


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


class TestScrewHoleOriginZError:
    def test_identical_frames_have_zero_error(self):
        frame = np.eye(4)
        assert screw_hole_origin_z_error(frame, frame) < 1e-15

    def test_twist_about_local_z_is_ignored(self):
        frame_a = np.eye(4)
        frame_b = np.eye(4)
        frame_b[:3, :3] = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        assert screw_hole_origin_z_error(frame_a, frame_b) < 1e-15

    def test_origin_and_z_diagnostics_report_expected_values(self):
        frame_a = np.eye(4)
        frame_b = np.eye(4)
        frame_b[:3, 3] = np.array([3.0, 4.0, 0.0], dtype=float)
        frame_b[:3, :3] = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        )
        diagnostics = screw_hole_alignment_diagnostics(frame_a, frame_b)
        assert abs(diagnostics["origin_error_mm"] - 5.0) < 1e-12
        assert abs(diagnostics["z_axis_error_rad"] - 0.5 * np.pi) < 1e-12


class TestCADTransforms:
    def test_female_fixed_rotation_has_zero_translation(self):
        np.testing.assert_allclose(config.FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM[:3, 3], np.zeros(3), atol=TOL_POSITION)

    def test_gap_transform_is_translation_only(self):
        np.testing.assert_allclose(
            config.FEMALE_MALE_GAP_OFFSET_TRANSFORM[:3, :3],
            np.eye(3),
            atol=TOL_ORIENTATION,
        )

    def test_jjr_zero_transform_keeps_origin_and_z(self):
        np.testing.assert_allclose(config.JJR_ZERO_TRANSFORM[:3, 3], np.zeros(3), atol=TOL_POSITION)
        np.testing.assert_allclose(
            config.JJR_ZERO_TRANSFORM[:3, 2],
            np.array([0.0, 0.0, 1.0]),
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

    def test_male_fixed_rotation_has_zero_translation(self):
        np.testing.assert_allclose(config.MALE_FIXED_ROT_TO_BAR_TRANSFORM[:3, 3], np.zeros(3), atol=TOL_POSITION)


class TestFKSanity:
    def test_fk_female_origin_lies_on_bar_axis(self):
        frame = fk_female_side(np.eye(4), 50.0, 0.0, config)["female_frame"]
        np.testing.assert_allclose(frame[:3, 3], np.array([0.0, 0.0, 50.0]), atol=TOL_POSITION)

    def test_fk_male_origin_lies_on_bar_axis(self):
        ln_bar_frame = _frame((0.0, -config.BAR_CONTACT_DISTANCE, 0.0))
        frame = fk_male_side(ln_bar_frame, -125.0, 0.0, config)["male_frame"]
        np.testing.assert_allclose(
            frame[:3, 3],
            np.array([0.0, -config.BAR_CONTACT_DISTANCE, 125.0]),
            atol=TOL_POSITION,
        )

    def test_line_wrapper_matches_bar_frame_on_female_side(self):
        start = np.array([15.0, -25.0, 40.0], dtype=float)
        direction = np.asarray(config.LE_BAR_REFERENCE_FRAME[:3, 2], dtype=float)
        end = start + 200.0 * direction
        le_bar_frame = np.array(config.LE_BAR_REFERENCE_FRAME, dtype=float, copy=True)
        le_bar_frame[:3, 3] = start
        state_from_frame = fk_female_side(le_bar_frame, 80.0, 0.3, config)
        state_from_lines = fk_female_side_from_lines(start, end, 80.0, 0.3, config)
        np.testing.assert_allclose(
            state_from_frame["female_screw_hole_frame"],
            state_from_lines["female_screw_hole_frame"],
            atol=TOL_POSITION,
        )

    def test_line_wrapper_matches_bar_frame_on_male_side(self):
        start = np.array([-40.0, 30.0, 10.0], dtype=float)
        direction = np.asarray(config.LN_BAR_REFERENCE_FRAME[:3, 2], dtype=float)
        end = start + 200.0 * direction
        ln_bar_frame = np.array(config.LN_BAR_REFERENCE_FRAME, dtype=float, copy=True)
        ln_bar_frame[:3, 3] = start
        state_from_frame = fk_male_side(ln_bar_frame, -60.0, 0.4, config)
        state_from_lines = fk_male_side_from_lines(start, end, -60.0, 0.4, config)
        np.testing.assert_allclose(
            state_from_frame["male_screw_hole_frame"],
            state_from_lines["male_screw_hole_frame"],
            atol=TOL_POSITION,
        )

    def test_fk_female_wrapper_returns_same_frame_as_side_helper(self):
        le_start = np.array([0.0, 0.0, 0.0], dtype=float)
        le_end = np.array([0.0, 0.0, 200.0], dtype=float)
        frame = fk_female(le_start, le_end, 40.0, 0.2, 0.0, config)
        expected = fk_female_side_from_lines(le_start, le_end, 40.0, 0.2, config)["female_frame"]
        np.testing.assert_allclose(frame, expected, atol=TOL_POSITION)

    def test_fk_male_wrapper_returns_same_frame_as_side_helper(self):
        ln_start = np.array([0.0, -config.BAR_CONTACT_DISTANCE, 0.0], dtype=float)
        ln_end = np.array([0.0, -config.BAR_CONTACT_DISTANCE, 200.0], dtype=float)
        frame = fk_male(ln_start, ln_end, -55.0, 0.1, config)
        expected = fk_male_side_from_lines(ln_start, ln_end, -55.0, 0.1, config)["male_frame"]
        np.testing.assert_allclose(frame, expected, atol=TOL_POSITION)

    def test_fjr_rotates_female_gap_vector(self):
        state_0 = fk_female_side(np.eye(4), 100.0, 0.0, config)
        state_pi = fk_female_side(np.eye(4), 100.0, np.pi, config)
        offset_0 = state_0["female_screw_hole_frame"][:3, 3] - state_0["female_frame"][:3, 3]
        offset_pi = state_pi["female_screw_hole_frame"][:3, 3] - state_pi["female_frame"][:3, 3]
        gap_length = float(np.linalg.norm(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM[:3, 3]))
        np.testing.assert_allclose(np.linalg.norm(offset_0), gap_length, atol=TOL_POSITION)
        cosine = np.dot(offset_0, offset_pi) / (np.linalg.norm(offset_0) * np.linalg.norm(offset_pi))
        assert cosine < -0.99

    def test_male_side_bar_axis_matches_mjr_frame_z(self):
        ln_start = np.array([0.0, 0.0, 0.0], dtype=float)
        ln_end = np.array([150.0, 150.0, 0.0], dtype=float)
        state = fk_male_side_from_lines(ln_start, ln_end, 75.0, 0.0, config)
        bar_unit = (ln_end - ln_start) / np.linalg.norm(ln_end - ln_start)
        assert abs(abs(np.dot(state["mjr_frame"][:3, 2], bar_unit)) - 1.0) < TOL_ORIENTATION


class TestOptimizeJointPlacement:
    def test_optimizer_recovers_a_consistent_assembled_case(self):
        le_bar_frame, ln_bar_frame = _assembled_bar_frames(
            fjp=85.0,
            fjr=0.45,
            jjr=0.30,
            mjr=-0.70,
            mjp=120.0,
        )
        le_start, le_end = _line_from_bar_frame(le_bar_frame)
        ln_start, ln_end = _line_from_bar_frame(ln_bar_frame)

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["origin_error_mm"] < TOL_POSITION
        assert result["z_axis_error_rad"] < TOL_ORIENTATION
        assert "jjr" not in result

    def test_optimizer_handles_a_second_assembled_case(self):
        le_bar_frame, ln_bar_frame = _assembled_bar_frames(
            fjp=-40.0,
            fjr=-0.80,
            jjr=1.10,
            mjr=0.55,
            mjp=-65.0,
        )
        le_start, le_end = _line_from_bar_frame(le_bar_frame)
        ln_start, ln_end = _line_from_bar_frame(ln_bar_frame)

        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert result["origin_error_mm"] < TOL_POSITION
        assert result["z_axis_error_rad"] < TOL_ORIENTATION

    def test_dof_values_within_bounds(self):
        le_bar_frame, ln_bar_frame = _assembled_bar_frames(
            fjp=20.0,
            fjr=0.10,
            jjr=-0.45,
            mjr=0.25,
            mjp=35.0,
        )
        le_start, le_end = _line_from_bar_frame(le_bar_frame)
        ln_start, ln_end = _line_from_bar_frame(ln_bar_frame)
        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config)
        assert config.FJP_RANGE[0] <= result["fjp"] <= config.FJP_RANGE[1]
        assert config.FJR_RANGE[0] <= result["fjr"] <= config.FJR_RANGE[1]
        assert config.MJP_RANGE[0] <= result["mjp"] <= config.MJP_RANGE[1]
        assert config.MJR_RANGE[0] <= result["mjr"] <= config.MJR_RANGE[1]

    def test_optimizer_can_return_debug_reports(self):
        le_bar_frame, ln_bar_frame = _assembled_bar_frames(
            fjp=10.0,
            fjr=0.20,
            jjr=0.35,
            mjr=-0.30,
            mjp=45.0,
        )
        le_start, le_end = _line_from_bar_frame(le_bar_frame)
        ln_start, ln_end = _line_from_bar_frame(ln_bar_frame)
        result = optimize_joint_placement(le_start, le_end, ln_start, ln_end, config, return_debug=True)
        debug = result["optimizer_debug"]
        assert debug["seed_count"] >= 4
        assert 0 <= debug["best_seed_index"] < debug["seed_count"]
        assert isinstance(debug["best_seed_report"]["message"], str)
        assert len(debug["seed_reports"]) == debug["seed_count"]
