"""Geometry helpers for line-line distance and S2 bar-axis solving."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize


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


def _orthogonal_to(vector: Iterable[float], *, tol: float = 1e-12) -> np.ndarray:
    """Return a stable unit vector orthogonal to the input direction."""

    vector = _unit(vector, tol=tol)
    if abs(float(np.dot(vector, np.array([0.0, 0.0, 1.0])))) < 0.95:
        candidate = np.cross(vector, np.array([0.0, 0.0, 1.0]))
    else:
        candidate = np.cross(vector, np.array([1.0, 0.0, 0.0]))
    return _unit(candidate, tol=tol)


def _parallel_distance_normal(
    x_i: np.ndarray,
    n_i: np.ndarray,
    x_j: np.ndarray,
    n_j: np.ndarray,
    *,
    tol: float,
) -> np.ndarray:
    """Return the appendix-A normal used for parallel or near-parallel bars."""

    delta = x_i - x_j
    normal = np.cross(np.cross(delta, n_i), n_j)
    normal_norm = np.linalg.norm(normal)
    if normal_norm <= tol:
        # Appendix A suggests replacing the degenerate normal with any vector
        # orthogonal to the bar direction.
        normal = _orthogonal_to(n_i, tol=tol)
    return _unit(normal, tol=tol)


def closest_params_infinite_lines(
    x_i: Iterable[float],
    n_i: Iterable[float],
    x_j: Iterable[float],
    n_j: Iterable[float],
    *,
    tol: float = 1e-10,
) -> Tuple[float, float]:
    """Return closest-point parameters on two infinite lines.

    For parallel lines the closest-point pair is not unique. In that case this
    returns a canonical solution with ``t_j = 0`` and ``t_i`` chosen so that the
    point on line ``i`` is the orthogonal projection of ``x_j`` onto line ``i``.
    """

    x_i = _as_vector(x_i)
    n_i = _as_vector(n_i)
    x_j = _as_vector(x_j)
    n_j = _as_vector(n_j)

    matrix = np.array(
        [
            [float(np.dot(n_i, n_i)), -float(np.dot(n_i, n_j))],
            [float(np.dot(n_j, n_i)), -float(np.dot(n_j, n_j))],
        ],
        dtype=float,
    )
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) <= tol:
        n_i_unit = _unit(n_i, tol=tol)
        t_i = float(np.dot(x_j - x_i, n_i_unit) / np.linalg.norm(n_i))
        return t_i, 0.0

    delta = x_j - x_i
    rhs = np.array([float(np.dot(delta, n_i)), float(np.dot(delta, n_j))], dtype=float)
    solution = np.linalg.solve(matrix, rhs)
    return float(solution[0]), float(solution[1])


def distance_infinite_lines(
    x_i: Iterable[float],
    n_i: Iterable[float],
    x_j: Iterable[float],
    n_j: Iterable[float],
    *,
    tol: float = 1e-10,
) -> float:
    """Return the signed distance between two infinite lines.

    For parallel or near-parallel lines, use the Appendix A construction from
    the referenced paper. With ``dx_i = dx_j = 0``, Eq. (A.3) reduces to the
    signed distance term used here.
    """

    x_i = _as_vector(x_i)
    x_j = _as_vector(x_j)
    n_i = _unit(n_i, tol=tol)
    n_j = _unit(n_j, tol=tol)
    normal = np.cross(n_i, n_j)
    normal_norm = np.linalg.norm(normal)
    if normal_norm <= tol:
        normal = _parallel_distance_normal(x_i, n_i, x_j, n_j, tol=tol)
        return float(np.dot(normal, x_i - x_j))
    normal /= normal_norm
    return float(np.dot(x_i - x_j, normal))


def closest_params_finite_segments(
    p1_start: Iterable[float],
    p1_end: Iterable[float],
    p2_start: Iterable[float],
    p2_end: Iterable[float],
    *,
    tol: float = 1e-8,
) -> Tuple[float, float]:
    """Return closest-point parameters in [0, 1] for two finite line segments."""

    p1_start = _as_vector(p1_start)
    p1_end = _as_vector(p1_end)
    p2_start = _as_vector(p2_start)
    p2_end = _as_vector(p2_end)

    d1 = p1_end - p1_start
    d2 = p2_end - p2_start
    if np.linalg.norm(d1) <= tol or np.linalg.norm(d2) <= tol:
        raise ValueError("Degenerate line segment.")

    def point_on_segment(start: np.ndarray, end: np.ndarray, t: float) -> np.ndarray:
        return (1.0 - t) * start + t * end

    def project_parameter(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
        direction = end - start
        denom = float(np.dot(direction, direction))
        if denom <= tol:
            raise ValueError("Degenerate line segment.")
        t_value = -float(np.dot(start - point, direction)) / denom
        return float(np.clip(t_value, 0.0, 1.0))

    matrix = np.array(
        [
            [float(np.dot(d1, d1)), -float(np.dot(d1, d2))],
            [float(np.dot(d2, d1)), -float(np.dot(d2, d2))],
        ],
        dtype=float,
    )
    determinant = float(np.linalg.det(matrix))
    rhs = np.array(
        [float(np.dot(p2_start - p1_start, d1)), float(np.dot(p2_start - p1_start, d2))],
        dtype=float,
    )

    if abs(determinant) > tol:
        params = np.linalg.solve(matrix, rhs)
        if 0.0 <= params[0] <= 1.0 and 0.0 <= params[1] <= 1.0:
            return float(params[0]), float(params[1])

    candidates = [(0.0, None), (1.0, None), (None, 0.0), (None, 1.0)]
    best_pair = None
    best_distance = math.inf
    for fixed_t1, fixed_t2 in candidates:
        if fixed_t1 is None:
            point = point_on_segment(p2_start, p2_end, fixed_t2)
            t1 = project_parameter(point, p1_start, p1_end)
            pair = (t1, fixed_t2)
        else:
            point = point_on_segment(p1_start, p1_end, fixed_t1)
            t2 = project_parameter(point, p2_start, p2_end)
            pair = (fixed_t1, t2)

        point1 = point_on_segment(p1_start, p1_end, pair[0])
        point2 = point_on_segment(p2_start, p2_end, pair[1])
        distance = float(np.linalg.norm(point1 - point2))
        if distance < best_distance:
            best_distance = distance
            best_pair = pair

    assert best_pair is not None
    return float(best_pair[0]), float(best_pair[1])


def are_lines_parallel(
    n_i: Iterable[float],
    n_j: Iterable[float],
    *,
    tol: float = 1e-6,
) -> bool:
    """Return True when the directions are parallel within tolerance."""

    n_i = _as_vector(n_i)
    n_j = _as_vector(n_j)
    norm_i = np.linalg.norm(n_i)
    norm_j = np.linalg.norm(n_j)
    if norm_i <= tol or norm_j <= tol:
        raise ValueError("Line direction cannot be near zero.")
    cross_norm = np.linalg.norm(np.cross(n_i, n_j))
    return float(cross_norm / (norm_i * norm_j)) < tol


def _angles_from_direction(direction: Iterable[float]) -> Tuple[float, float]:
    unit = _unit(direction)
    theta = float(math.acos(np.clip(unit[2], -1.0, 1.0)))
    phi = float(math.atan2(unit[1], unit[0]))
    return theta, phi


def _direction_from_angles(theta: float, phi: float) -> np.ndarray:
    return np.array(
        [
            math.sin(theta) * math.cos(phi),
            math.sin(theta) * math.sin(phi),
            math.cos(theta),
        ],
        dtype=float,
    )


def _candidate_initial_directions(
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    hint: Optional[Sequence[float]],
) -> list[np.ndarray]:
    candidates: list[np.ndarray] = []

    def add(vector: Iterable[float]) -> None:
        try:
            unit = _unit(vector)
        except ValueError:
            return
        for existing in candidates:
            if np.linalg.norm(existing - unit) < 1e-6 or np.linalg.norm(existing + unit) < 1e-6:
                return
        candidates.append(unit)

    if hint is not None:
        add(hint)

    delta = ce2 - ce1
    add(delta)
    add(-delta)
    add(np.cross(n1, n2))
    add(-np.cross(n1, n2))

    world_axes = np.eye(3, dtype=float)
    for axis in world_axes:
        add(axis)
        add(-axis)
        add(np.cross(n1, axis))
        add(np.cross(n2, axis))

    add(n1 + n2)
    add(n1 - n2)
    add(np.cross(delta, n1))
    add(np.cross(delta, n2))

    if not candidates:
        raise ValueError("Unable to construct an initial direction guess.")
    return candidates


def _solve_s2_t1_objective(
    params: np.ndarray,
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    distance: float,
    sign_1: float,
    sign_2: float,
    *,
    tol: float,
) -> float:
    nn = _direction_from_angles(float(params[0]), float(params[1]))
    cn1 = np.cross(nn, n1)
    cn2 = np.cross(nn, n2)
    cn1_norm = np.linalg.norm(cn1)
    cn2_norm = np.linalg.norm(cn2)
    if cn1_norm <= tol or cn2_norm <= tol:
        return 1e12
    cn1 /= cn1_norm
    cn2 /= cn2_norm

    p1 = ce1 + sign_1 * distance * cn1
    p2 = ce2 + sign_2 * distance * cn2
    diff = p2 - p1
    cross_val = np.cross(diff, nn)
    residual = float(np.dot(cross_val, cross_val))

    penalty = 0.0
    t1, _ = closest_params_infinite_lines(ce1, n1, p1, nn, tol=tol)
    t2, _ = closest_params_infinite_lines(ce2, n2, p1, nn, tol=tol)
    penalty += float(t1 * t1 + t2 * t2)
    return residual + 10.0 * penalty


def _build_s2_t1_solution(
    params: np.ndarray,
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    distance: float,
    sign_1: float,
    sign_2: float,
    *,
    tol: float,
    residual: float,
) -> dict:
    nn = _unit(_direction_from_angles(float(params[0]), float(params[1])))
    cn1 = _unit(np.cross(nn, n1), tol=tol)
    cn2 = _unit(np.cross(nn, n2), tol=tol)
    p1 = ce1 + sign_1 * distance * cn1
    p2 = ce2 + sign_2 * distance * cn2

    diff = p2 - p1
    if np.linalg.norm(diff) > tol:
        line_direction = _unit(diff)
        nn = line_direction if np.dot(line_direction, nn) >= 0.0 else -line_direction

    return {
        "nn": nn,
        "p1": p1,
        "p2": p2,
        "residual": float(residual),
        "signs": (float(sign_1), float(sign_2)),
    }


def _s2_t1_same_geometry(
    solution_a: dict,
    solution_b: dict,
    *,
    pos_tol: float,
    dir_tol: float,
) -> bool:
    if abs(abs(float(np.dot(solution_a["nn"], solution_b["nn"]))) - 1.0) > dir_tol:
        return False
    if float(np.linalg.norm(solution_a["p1"] - solution_b["p1"])) > pos_tol:
        return False
    if float(np.linalg.norm(solution_a["p2"] - solution_b["p2"])) > pos_tol:
        return False
    return True


def _append_unique_s2_t1_solution(
    solutions: list[dict],
    candidate: dict,
    *,
    pos_tol: float,
    dir_tol: float,
) -> None:
    for index, existing in enumerate(solutions):
        if _s2_t1_same_geometry(existing, candidate, pos_tol=pos_tol, dir_tol=dir_tol):
            if candidate["residual"] < existing["residual"]:
                solutions[index] = candidate
            return
    solutions.append(candidate)


def solve_s2_t1_all(
    n1: Iterable[float],
    x_ce1: Iterable[float],
    n2: Iterable[float],
    x_ce2: Iterable[float],
    distance: float,
    nn_init_hint: Optional[Sequence[float]] = None,
    *,
    tol: float = 1e-6,
    residual_threshold: float = 1e-4,
) -> list[dict]:
    """Solve S2-T1 for all 4 sign combinations and return converged solutions."""

    n1 = _unit(n1)
    n2 = _unit(n2)
    ce1 = _as_vector(x_ce1)
    ce2 = _as_vector(x_ce2)
    distance = float(distance)
    if distance <= 0.0:
        raise ValueError("Contact distance must be positive.")

    initial_directions = _candidate_initial_directions(n1, ce1, n2, ce2, nn_init_hint)
    sign_pairs = ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))
    solutions = []
    pos_tol = max(1e-3, 10.0 * tol * max(1.0, distance))
    dir_tol = 1e-6
    for sign_1, sign_2 in sign_pairs:
        sign_solutions = []
        for init_direction in initial_directions:
            theta0, phi0 = _angles_from_direction(init_direction)
            def objective(params: np.ndarray, sign_1=sign_1, sign_2=sign_2) -> float:
                return _solve_s2_t1_objective(
                    params,
                    n1,
                    ce1,
                    n2,
                    ce2,
                    distance,
                    sign_1,
                    sign_2,
                    tol=tol,
                )
            result = minimize(
                objective,
                np.array([theta0, phi0], dtype=float),
                method="Nelder-Mead",
                options={"xatol": 1e-12, "fatol": 1e-12, "maxiter": 10000},
            )
            solution = _build_s2_t1_solution(
                np.asarray(result.x, dtype=float),
                n1,
                ce1,
                n2,
                ce2,
                distance,
                sign_1,
                sign_2,
                tol=tol,
                residual=float(result.fun),
            )
            if solution["residual"] < residual_threshold:
                _append_unique_s2_t1_solution(
                    sign_solutions,
                    solution,
                    pos_tol=pos_tol,
                    dir_tol=dir_tol,
                )

        for solution in sign_solutions:
            _append_unique_s2_t1_solution(
                solutions,
                solution,
                pos_tol=pos_tol,
                dir_tol=dir_tol,
            )

    solutions.sort(key=lambda item: item["residual"])
    return solutions


def solve_s2_t1(
    n1: Iterable[float],
    x_ce1: Iterable[float],
    n2: Iterable[float],
    x_ce2: Iterable[float],
    distance: float,
    nn_init_hint: Optional[Sequence[float]] = None,
    *,
    tol: float = 1e-6,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Original API: return the single best S2-T1 solution."""

    solutions = solve_s2_t1_all(
        n1,
        x_ce1,
        n2,
        x_ce2,
        distance,
        nn_init_hint=nn_init_hint,
        tol=tol,
    )
    if not solutions:
        raise RuntimeError("Failed to solve the S2 bar-axis problem.")
    best = solutions[0]
    return best["nn"], best["p1"], best["p2"]
