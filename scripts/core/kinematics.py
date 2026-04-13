"""Forward kinematics and optimization for T20-5 joint placement."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import minimize

from core.geometry import closest_params_infinite_lines


def _as_vector(value: Iterable[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Expected a 3D vector.")
    return vector


def _unit(vector: Iterable[float], *, tol: float = 1e-12) -> np.ndarray:
    vector = _as_vector(vector)
    norm = np.linalg.norm(vector)
    if norm <= tol:
        raise ValueError("Cannot normalize a near-zero vector.")
    return vector / norm


def perpendicular_to(z_axis: Iterable[float]) -> np.ndarray:
    """Return a stable unit vector perpendicular to the provided axis."""

    z_axis = _unit(z_axis)
    if abs(float(np.dot(z_axis, np.array([0.0, 0.0, 1.0])))) < 0.95:
        x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
    else:
        x_axis = np.cross(np.array([1.0, 0.0, 0.0]), z_axis)
    return _unit(x_axis)


def _rotation_matrix(axis: Iterable[float], angle: float) -> np.ndarray:
    axis = _unit(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def _make_frame(origin: np.ndarray, x_axis: np.ndarray, y_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    frame = np.eye(4, dtype=float)
    frame[:3, 0] = _unit(x_axis)
    frame[:3, 1] = _unit(y_axis)
    frame[:3, 2] = _unit(z_axis)
    frame[:3, 3] = origin
    return frame


def make_bar_frame(line_start: Iterable[float], line_end: Iterable[float]) -> np.ndarray:
    """Return a 4x4 frame at the midpoint of a bar."""

    line_start = _as_vector(line_start)
    line_end = _as_vector(line_end)
    z_axis = _unit(line_end - line_start)
    x_axis = perpendicular_to(z_axis)
    y_axis = _unit(np.cross(z_axis, x_axis))
    origin = 0.5 * (line_start + line_end)
    return _make_frame(origin, x_axis, y_axis, z_axis)


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


def _rotated_radial_basis(bar_unit: np.ndarray, angle: float) -> tuple[np.ndarray, np.ndarray]:
    x_ref = perpendicular_to(bar_unit)
    y_ref = _unit(np.cross(bar_unit, x_ref))
    rotation = _rotation_matrix(bar_unit, angle)
    radial = _unit(rotation @ x_ref)
    tangent = _unit(rotation @ y_ref)
    return radial, tangent


def fk_female(Le_start, Le_end, fjp, fjr, jjr, config) -> np.ndarray:
    """Forward kinematics for the female joint chain."""

    le_start = _as_vector(Le_start)
    le_end = _as_vector(Le_end)
    bar_unit = _unit(le_end - le_start)
    pos_on_bar = le_start + float(fjp) * bar_unit

    radial_dir, _ = _rotated_radial_basis(bar_unit, float(fjr))
    origin = (
        pos_on_bar
        + float(config.FEMALE_AXIAL_OFFSET) * bar_unit
        + float(config.FEMALE_RADIAL_OFFSET) * radial_dir
    )

    x_axis = bar_unit
    y_axis = _unit(np.cross(radial_dir, bar_unit))
    z_axis = radial_dir

    rotation = _rotation_matrix(z_axis, float(jjr))
    x_axis = _unit(rotation @ x_axis)
    y_axis = _unit(rotation @ y_axis)
    return _make_frame(origin, x_axis, y_axis, z_axis)


def fk_male(Ln_start, Ln_end, mjp, mjr, config) -> np.ndarray:
    """Forward kinematics for the male joint chain."""

    ln_start = _as_vector(Ln_start)
    ln_end = _as_vector(Ln_end)
    bar_unit = _unit(ln_end - ln_start)
    pos_on_bar = ln_start + float(mjp) * bar_unit

    radial_dir, _ = _rotated_radial_basis(bar_unit, float(mjr))
    origin = (
        pos_on_bar
        + float(config.MALE_AXIAL_OFFSET) * bar_unit
        - float(config.MALE_RADIAL_OFFSET) * radial_dir
    )

    x_axis = bar_unit
    y_axis = _unit(np.cross(radial_dir, bar_unit))
    z_axis = radial_dir
    return _make_frame(origin, x_axis, y_axis, z_axis)


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
    """Optimize the five joint DOFs that best align the male and female OCFs."""

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

    def objective(params: np.ndarray) -> float:
        fjp, fjr, mjp, mjr, jjr = params
        female_frame = fk_female(le_start, le_end, fjp, fjr, jjr, config)
        male_frame = fk_male(ln_start, ln_end, mjp, mjr, config)
        return frame_distance(female_frame, male_frame)

    t_e, t_n = closest_params_infinite_lines(le_start, le_direction, ln_start, ln_direction)
    fjp0 = float(t_e * le_length)
    mjp0 = float(t_n * ln_length)
    bounds = [config.FJP_RANGE, config.FJR_RANGE, config.MJP_RANGE, config.MJR_RANGE, config.JJR_RANGE]

    seeds = [
        np.array([fjp0, 0.0, mjp0, 0.0, 0.0], dtype=float),
        np.array([fjp0, 0.0, mjp0, math.pi, 0.0], dtype=float),
        np.array([fjp0, math.pi, mjp0, 0.0, 0.0], dtype=float),
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
    female_frame = fk_female(le_start, le_end, fjp, fjr, jjr, config)
    male_frame = fk_male(ln_start, ln_end, mjp, mjr, config)
    return {
        "fjp": fjp,
        "fjr": fjr,
        "mjp": mjp,
        "mjr": mjr,
        "jjr": jjr,
        "residual": float(best_result.fun),
        "female_frame": female_frame,
        "male_frame": male_frame,
    }
