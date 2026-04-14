"""Forward kinematics and optimization for CAD-backed joint placement."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import minimize

from core.geometry import closest_params_infinite_lines
from core.transforms import (
    align_vectors,
    as_vector,
    frame_from_axes,
    invert_transform,
    orthogonal_to,
    rotation_about_local_z,
    rotation_matrix,
    transport_reference_frame,
    translation_transform,
    unit,
)


def _as_vector(value: Iterable[float]) -> np.ndarray:
    return as_vector(value)


def _unit(vector: Iterable[float], *, tol: float = 1e-12) -> np.ndarray:
    return unit(vector, tol=tol)


def perpendicular_to(z_axis: Iterable[float]) -> np.ndarray:
    """Return a stable unit vector perpendicular to the provided axis."""

    return orthogonal_to(z_axis)


def _rotation_matrix(axis: Iterable[float], angle: float) -> np.ndarray:
    return rotation_matrix(axis, angle)


def _make_frame(origin: np.ndarray, x_axis: np.ndarray, y_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    return frame_from_axes(origin, x_axis, y_axis, z_axis)


def make_bar_frame(line_start: Iterable[float], line_end: Iterable[float]) -> np.ndarray:
    """Return a simple midpoint bar frame for debug visualization."""

    line_start = _as_vector(line_start)
    line_end = _as_vector(line_end)
    z_axis = _unit(line_end - line_start)
    x_axis = perpendicular_to(z_axis)
    y_axis = _unit(np.cross(z_axis, x_axis))
    origin = 0.5 * (line_start + line_end)
    return _make_frame(origin, x_axis, y_axis, z_axis)


def _make_runtime_bar_frame(line_start: Iterable[float], line_end: Iterable[float], reference_frame: np.ndarray) -> np.ndarray:
    start = _as_vector(line_start)
    end = _as_vector(line_end)
    direction = _unit(end - start)
    rotation = transport_reference_frame(reference_frame, direction)
    frame = np.eye(4, dtype=float)
    frame[:3, :3] = rotation
    frame[:3, 3] = start
    return frame


def translate_along_axis(frame: np.ndarray, axis: Iterable[float], distance: float) -> np.ndarray:
    """Return a copy of the frame translated along the given axis."""

    translated = np.array(frame, dtype=float, copy=True)
    translated[:3, 3] = translated[:3, 3] + float(distance) * _unit(axis)
    return translated


def rotate_around_axis(frame: np.ndarray, axis: Iterable[float], angle: float) -> np.ndarray:
    """Return a copy of the frame rotated around the given axis."""

    rotated = np.array(frame, dtype=float, copy=True)
    rotation = _rotation_matrix(axis, angle)
    rotated[:3, :3] = rotation @ rotated[:3, :3]
    return rotated


def fk_female(Le_start, Le_end, fjp, fjr, jjr, config) -> np.ndarray:
    """Forward kinematics for the female link frame.

    The `jjr` argument is accepted for backwards compatibility but is not used on
    the female side of the chain.
    """

    del jjr
    return fk_female_side(Le_start, Le_end, fjp, fjr, config)["female_frame"]


def fk_male(Ln_start, Ln_end, mjp, mjr, config) -> np.ndarray:
    """Forward kinematics for the male link frame."""

    return fk_male_side(Ln_start, Ln_end, mjp, mjr, config)["male_frame"]


def fk_female_side(Le_start, Le_end, fjp, fjr, config) -> dict[str, np.ndarray]:
    le_bar_frame = _make_runtime_bar_frame(Le_start, Le_end, config.LE_BAR_REFERENCE_FRAME)
    fjp_frame = le_bar_frame @ translation_transform((0.0, 0.0, float(fjp)))
    fjr_frame = fjp_frame @ rotation_about_local_z(float(fjr))
    female_frame = fjr_frame @ np.asarray(config.FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM, dtype=float)
    female_screw_hole_frame = female_frame @ np.asarray(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM, dtype=float)
    return {
        "le_bar_frame": le_bar_frame,
        "fjp_frame": fjp_frame,
        "fjr_frame": fjr_frame,
        "female_frame": female_frame,
        "female_screw_hole_frame": female_screw_hole_frame,
    }


def fk_male_side(Ln_start, Ln_end, mjp, mjr, config) -> dict[str, np.ndarray]:
    ln_bar_frame = _make_runtime_bar_frame(Ln_start, Ln_end, config.LN_BAR_REFERENCE_FRAME)
    mjp_frame = ln_bar_frame @ translation_transform((0.0, 0.0, -float(mjp)))
    mjr_frame = mjp_frame @ rotation_about_local_z(-float(mjr))
    male_frame = mjr_frame @ invert_transform(np.asarray(config.MALE_FIXED_ROT_TO_BAR_TRANSFORM, dtype=float))
    male_screw_hole_frame = male_frame @ invert_transform(np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float))
    return {
        "ln_bar_frame": ln_bar_frame,
        "mjp_frame": mjp_frame,
        "mjr_frame": mjr_frame,
        "male_frame": male_frame,
        "male_screw_hole_frame": male_screw_hole_frame,
    }


def predict_male_from_female(female_screw_hole_frame: np.ndarray, jjr: float, config) -> dict[str, np.ndarray]:
    predicted_male_screw_hole_frame = (
        np.asarray(female_screw_hole_frame, dtype=float)
        @ np.asarray(config.JJR_ZERO_TRANSFORM, dtype=float)
        @ rotation_about_local_z(float(jjr))
    )
    predicted_male_frame = predicted_male_screw_hole_frame @ np.asarray(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM, dtype=float)
    predicted_mjr_frame = predicted_male_frame @ np.asarray(config.MALE_FIXED_ROT_TO_BAR_TRANSFORM, dtype=float)
    return {
        "predicted_male_screw_hole_frame": predicted_male_screw_hole_frame,
        "predicted_male_frame": predicted_male_frame,
        "predicted_mjr_frame": predicted_mjr_frame,
    }


def frame_distance(frame_a: np.ndarray, frame_b: np.ndarray) -> float:
    """Return a combined position-plus-orientation mismatch between frames."""

    frame_a = np.asarray(frame_a, dtype=float)
    frame_b = np.asarray(frame_b, dtype=float)
    pos_err = float(np.sum((frame_a[:3, 3] - frame_b[:3, 3]) ** 2))
    orient_err = 0.0
    for axis_index in range(3):
        orient_err += float(np.sum((frame_a[:3, axis_index] - frame_b[:3, axis_index]) ** 2))
    return pos_err + orient_err


def optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config):
    """Optimize the five joint DOFs that best align the male and female screw-hole frames."""

    le_start = _as_vector(Le_start)
    le_end = _as_vector(Le_end)
    ln_start = _as_vector(Ln_start)
    ln_end = _as_vector(Ln_end)

    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start
    le_length = float(np.linalg.norm(le_direction))
    ln_length = float(np.linalg.norm(ln_direction))
    if le_length <= 1e-12 or ln_length <= 1e-12:
        raise ValueError("Bars must have non-zero length.")

    le_unit = le_direction / le_length
    ln_unit = ln_direction / ln_length

    def objective(params: np.ndarray) -> float:
        fjp, fjr, mjp, mjr, jjr = params
        female_side = fk_female_side(le_start, le_end, fjp, fjr, config)
        male_side = fk_male_side(ln_start, ln_end, mjp, mjr, config)
        predicted = predict_male_from_female(female_side["female_screw_hole_frame"], jjr, config)
        return frame_distance(predicted["predicted_male_screw_hole_frame"], male_side["male_screw_hole_frame"])

    t_e, t_n = closest_params_infinite_lines(le_start, le_unit, ln_start, ln_unit)
    fjp0 = float(t_e)
    mjp0 = -float(t_n)
    bounds = [config.FJP_RANGE, config.FJR_RANGE, config.MJP_RANGE, config.MJR_RANGE, config.JJR_RANGE]

    seeds = [
        np.array([fjp0, 0.0, mjp0, 0.0, 0.0], dtype=float),
        np.array([fjp0, math.pi / 2.0, mjp0, 0.0, 0.0], dtype=float),
        np.array([fjp0, 0.0, mjp0, math.pi / 2.0, 0.0], dtype=float),
        np.array([fjp0, math.pi, mjp0, math.pi, math.pi / 2.0], dtype=float),
    ]

    rng = np.random.default_rng(0)
    for _ in range(int(getattr(config, "OPTIMIZER_RANDOM_RESTARTS", 12))):
        seeds.append(
            np.array(
                [
                    fjp0 + rng.uniform(-config.OPTIMIZER_TRANSLATION_PERTURBATION, config.OPTIMIZER_TRANSLATION_PERTURBATION),
                    rng.uniform(*config.FJR_RANGE),
                    mjp0 + rng.uniform(-config.OPTIMIZER_TRANSLATION_PERTURBATION, config.OPTIMIZER_TRANSLATION_PERTURBATION),
                    rng.uniform(*config.MJR_RANGE),
                    rng.uniform(*config.JJR_RANGE),
                ],
                dtype=float,
            )
        )

    best_result = None
    for seed in seeds:
        result = minimize(objective, seed, method="L-BFGS-B", bounds=bounds)
        if best_result is None or float(result.fun) < float(best_result.fun):
            best_result = result

    assert best_result is not None
    fjp, fjr, mjp, mjr, jjr = [float(value) for value in best_result.x]
    female_side = fk_female_side(le_start, le_end, fjp, fjr, config)
    male_side = fk_male_side(ln_start, ln_end, mjp, mjr, config)
    predicted = predict_male_from_female(female_side["female_screw_hole_frame"], jjr, config)
    return {
        "fjp": fjp,
        "fjr": fjr,
        "mjp": mjp,
        "mjr": mjr,
        "jjr": jjr,
        "residual": float(best_result.fun),
        "female_frame": female_side["female_frame"],
        "female_screw_hole_frame": female_side["female_screw_hole_frame"],
        "male_screw_hole_frame": male_side["male_screw_hole_frame"],
        "male_frame": male_side["male_frame"],
        "predicted_male_frame": predicted["predicted_male_frame"],
        "predicted_male_screw_hole_frame": predicted["predicted_male_screw_hole_frame"],
    }
