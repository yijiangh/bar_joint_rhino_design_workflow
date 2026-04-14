"""Forward kinematics and optimization for CAD-backed joint placement."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import minimize

from core.geometry import closest_params_infinite_lines
from core.transforms import as_vector, frame_from_axes, invert_transform, orthogonal_to, rotation_about_local_z, rotation_matrix, transport_reference_frame, translation_transform, unit


def _as_vector(value: Iterable[float]) -> np.ndarray:
    return as_vector(value)


def _unit(vector: Iterable[float], *, tol: float = 1e-12) -> np.ndarray:
    return unit(vector, tol=tol)


def _as_frame(value: Iterable[Iterable[float]]) -> np.ndarray:
    frame = np.asarray(value, dtype=float)
    if frame.shape != (4, 4):
        raise ValueError("Expected a 4x4 frame.")
    return frame


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
    return fk_female_side_from_lines(Le_start, Le_end, fjp, fjr, config)["female_frame"]


def fk_male(Ln_start, Ln_end, mjp, mjr, config) -> np.ndarray:
    """Forward kinematics for the male link frame."""

    return fk_male_side_from_lines(Ln_start, Ln_end, mjp, mjr, config)["male_frame"]


def fk_female_side(le_bar_frame, fjp, fjr, config) -> dict[str, np.ndarray]:
    le_bar_frame = _as_frame(le_bar_frame)
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


def fk_female_side_from_lines(Le_start, Le_end, fjp, fjr, config) -> dict[str, np.ndarray]:
    le_bar_frame = _make_runtime_bar_frame(Le_start, Le_end, config.LE_BAR_REFERENCE_FRAME)
    return fk_female_side(le_bar_frame, fjp, fjr, config)


def fk_male_side(ln_bar_frame, mjp, mjr, config) -> dict[str, np.ndarray]:
    ln_bar_frame = _as_frame(ln_bar_frame)
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


def fk_male_side_from_lines(Ln_start, Ln_end, mjp, mjr, config) -> dict[str, np.ndarray]:
    ln_bar_frame = _make_runtime_bar_frame(Ln_start, Ln_end, config.LN_BAR_REFERENCE_FRAME)
    return fk_male_side(ln_bar_frame, mjp, mjr, config)


def frame_distance(frame_a: np.ndarray, frame_b: np.ndarray) -> float:
    """Return a combined position-plus-orientation mismatch between frames."""

    frame_a = np.asarray(frame_a, dtype=float)
    frame_b = np.asarray(frame_b, dtype=float)
    pos_err = float(np.sum((frame_a[:3, 3] - frame_b[:3, 3]) ** 2))
    orient_err = 0.0
    for axis_index in range(3):
        orient_err += float(np.sum((frame_a[:3, axis_index] - frame_b[:3, axis_index]) ** 2))
    return pos_err + orient_err


def screw_hole_origin_z_error(female_screw_hole_frame: np.ndarray, male_screw_hole_frame: np.ndarray, *, orientation_weight_mm: float = 1.0) -> float:
    """Return an interface error that constrains only origin coincidence and local-Z alignment."""

    female_screw_hole_frame = np.asarray(female_screw_hole_frame, dtype=float)
    male_screw_hole_frame = np.asarray(male_screw_hole_frame, dtype=float)
    origin_delta = female_screw_hole_frame[:3, 3] - male_screw_hole_frame[:3, 3]
    z_delta = female_screw_hole_frame[:3, 2] - male_screw_hole_frame[:3, 2]
    return float(np.dot(origin_delta, origin_delta) + (orientation_weight_mm ** 2) * np.dot(z_delta, z_delta))


def screw_hole_alignment_diagnostics(female_screw_hole_frame: np.ndarray, male_screw_hole_frame: np.ndarray) -> dict[str, float]:
    female_screw_hole_frame = np.asarray(female_screw_hole_frame, dtype=float)
    male_screw_hole_frame = np.asarray(male_screw_hole_frame, dtype=float)
    origin_error_mm = float(np.linalg.norm(female_screw_hole_frame[:3, 3] - male_screw_hole_frame[:3, 3]))
    dot_product = float(np.clip(np.dot(female_screw_hole_frame[:3, 2], male_screw_hole_frame[:3, 2]), -1.0, 1.0))
    z_axis_error_rad = float(math.acos(dot_product))
    return {"origin_error_mm": origin_error_mm, "z_axis_error_rad": z_axis_error_rad}


def optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config, *, return_debug: bool = False):
    """Optimize the four bar-side joint DOFs that best align the screw-hole origins and local Z axes."""

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
    le_bar_frame = _make_runtime_bar_frame(le_start, le_end, config.LE_BAR_REFERENCE_FRAME)
    ln_bar_frame = _make_runtime_bar_frame(ln_start, ln_end, config.LN_BAR_REFERENCE_FRAME)
    orientation_weight_mm = float(getattr(config, "BAR_CONTACT_DISTANCE", 1.0))

    def objective(params: np.ndarray) -> float:
        fjp, fjr, mjp, mjr = params
        female_side = fk_female_side(le_bar_frame, fjp, fjr, config)
        male_side = fk_male_side(ln_bar_frame, mjp, mjr, config)
        return screw_hole_origin_z_error(
            female_side["female_screw_hole_frame"],
            male_side["male_screw_hole_frame"],
            orientation_weight_mm=orientation_weight_mm,
        )

    t_e, t_n = closest_params_infinite_lines(le_start, le_unit, ln_start, ln_unit)
    fjp0 = float(t_e)
    mjp0 = -float(t_n)
    bounds = [config.FJP_RANGE, config.FJR_RANGE, config.MJP_RANGE, config.MJR_RANGE]

    seeds = [
        np.array([fjp0, 0.0, mjp0, 0.0], dtype=float),
        np.array([fjp0, math.pi / 2.0, mjp0, 0.0], dtype=float),
        np.array([fjp0, 0.0, mjp0, math.pi / 2.0], dtype=float),
        np.array([fjp0, math.pi, mjp0, math.pi], dtype=float),
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
                ],
                dtype=float,
            )
        )

    best_result = None
    best_seed_index = -1
    seed_reports: list[dict[str, object]] = []
    for seed_index, seed in enumerate(seeds):
        result = minimize(objective, seed, method="L-BFGS-B", bounds=bounds)
        if return_debug:
            seed_reports.append(
                {
                    "seed_index": int(seed_index),
                    "initial_guess": tuple(float(value) for value in seed),
                    "solution": tuple(float(value) for value in result.x),
                    "success": bool(result.success),
                    "status": int(result.status),
                    "message": str(result.message),
                    "residual": float(result.fun),
                    "nit": int(getattr(result, "nit", -1)),
                    "nfev": int(getattr(result, "nfev", -1)),
                }
            )
        if best_result is None or float(result.fun) < float(best_result.fun):
            best_result = result
            best_seed_index = seed_index

    assert best_result is not None
    fjp, fjr, mjp, mjr = [float(value) for value in best_result.x]
    female_side = fk_female_side(le_bar_frame, fjp, fjr, config)
    male_side = fk_male_side(ln_bar_frame, mjp, mjr, config)
    diagnostics = screw_hole_alignment_diagnostics(
        female_side["female_screw_hole_frame"],
        male_side["male_screw_hole_frame"],
    )
    output = {
        "fjp": fjp,
        "fjr": fjr,
        "mjp": mjp,
        "mjr": mjr,
        "residual": float(best_result.fun),
        "origin_error_mm": diagnostics["origin_error_mm"],
        "z_axis_error_rad": diagnostics["z_axis_error_rad"],
        "female_frame": female_side["female_frame"],
        "female_screw_hole_frame": female_side["female_screw_hole_frame"],
        "male_screw_hole_frame": male_side["male_screw_hole_frame"],
        "male_frame": male_side["male_frame"],
    }
    if return_debug:
        output["optimizer_debug"] = {
            "seed_count": len(seeds),
            "best_seed_index": int(best_seed_index),
            "best_seed_report": seed_reports[best_seed_index],
            "seed_reports": seed_reports,
        }
    return output
