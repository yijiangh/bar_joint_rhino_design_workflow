"""Geometry helpers for line-line distance and S2 bar-axis solving."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares


_S2_T1_LSQ_XTOL = None
_S2_T1_LSQ_FTOL = None
_S2_T1_LSQ_GTOL = 1e-12
_S2_T1_LSQ_MAX_NFEV = 10000


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


def _wrap_angle(angle: float) -> float:
    return float(math.atan2(math.sin(angle), math.cos(angle)))


def _circle_basis(normal: Iterable[float], *, tol: float = 1e-12) -> Tuple[np.ndarray, np.ndarray]:
    normal = _unit(normal, tol=tol)
    u = _orthogonal_to(normal, tol=tol)
    v = _unit(np.cross(normal, u), tol=tol)
    return u, v


def _angle_on_circle(
    reference: Iterable[float],
    basis_u: np.ndarray,
    basis_v: np.ndarray,
    *,
    tol: float = 1e-12,
) -> float:
    reference = _as_vector(reference)
    proj_u = float(np.dot(reference, basis_u))
    proj_v = float(np.dot(reference, basis_v))
    if proj_u * proj_u + proj_v * proj_v <= tol * tol:
        return 0.0
    return _wrap_angle(math.atan2(proj_v, proj_u))


def _point_on_circle(
    center: np.ndarray,
    basis_u: np.ndarray,
    basis_v: np.ndarray,
    angle: float,
    radius: float,
) -> np.ndarray:
    return center + float(radius) * (
        math.cos(float(angle)) * basis_u + math.sin(float(angle)) * basis_v
    )


_SIGN_FAMILY_ORDER = (
    (+1, +1),
    (-1, +1),
    (+1, -1),
    (-1, -1),
)


def _sign_family_label(sign_1: int, sign_2: int) -> str:
    sign_1_label = "+" if int(sign_1) >= 0 else "-"
    sign_2_label = "+" if int(sign_2) >= 0 else "-"
    return f"({sign_1_label},{sign_2_label})"


def _parse_sign_family(label: str) -> Tuple[int, int]:
    cleaned = str(label).strip()
    if len(cleaned) != 5 or cleaned[0] != "(" or cleaned[2] != "," or cleaned[4] != ")":
        raise ValueError(f"Invalid sign family label: {label!r}")
    return (+1 if cleaned[1] == "+" else -1, +1 if cleaned[3] == "+" else -1)


def _angle_between_degrees(
    vector_a: Iterable[float],
    vector_b: Iterable[float],
    *,
    tol: float = 1e-12,
) -> float:
    vector_a = _as_vector(vector_a)
    vector_b = _as_vector(vector_b)
    norm_a = float(np.linalg.norm(vector_a))
    norm_b = float(np.linalg.norm(vector_b))
    if norm_a <= tol or norm_b <= tol:
        return float("nan")
    cosine = float(np.dot(vector_a, vector_b)) / (norm_a * norm_b)
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def _optimizer_settings_dict() -> dict:
    return {
        "xtol": None if _S2_T1_LSQ_XTOL is None else float(_S2_T1_LSQ_XTOL),
        "ftol": None if _S2_T1_LSQ_FTOL is None else float(_S2_T1_LSQ_FTOL),
        "gtol": float(_S2_T1_LSQ_GTOL),
        "max_nfev": int(_S2_T1_LSQ_MAX_NFEV),
    }


def _termination_reason_label(message: str, status: int) -> str:
    lowered = str(message).lower()
    if "gtol" in lowered:
        return "gtol"
    if "ftol" in lowered:
        return "ftol"
    if "xtol" in lowered:
        return "xtol"
    if "maximum number of function evaluations" in lowered:
        return "max_nfev"
    if int(status) == 0:
        return "status_0"
    if int(status) < 0:
        return "failure"
    return "other"


def _solution_sign_components(
    solution: dict,
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
) -> Tuple[int, int]:
    radial_1 = solution["p1"] - ce1
    radial_2 = solution["p2"] - ce2
    ref_1 = np.cross(n1, solution["nn"])
    ref_2 = np.cross(n2, solution["nn"])
    sign_1 = +1 if float(np.dot(radial_1, ref_1)) >= 0.0 else -1
    sign_2 = +1 if float(np.dot(radial_2, ref_2)) >= 0.0 else -1
    return sign_1, sign_2


def _solution_optimization_diagnostics(
    solution: dict,
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    distance: float,
) -> dict:
    radial_1 = solution["p1"] - ce1
    radial_2 = solution["p2"] - ce2
    nn = solution["nn"]
    scale = max(abs(distance), 1.0)
    residual_1 = float(np.dot(radial_1, nn)) / scale
    residual_2 = float(np.dot(radial_2, nn)) / scale
    angle_1 = _angle_between_degrees(radial_1, nn)
    angle_2 = _angle_between_degrees(radial_2, nn)
    return {
        "contact_1_axial_offset": float(np.dot(radial_1, nn)),
        "contact_2_axial_offset": float(np.dot(radial_2, nn)),
        "contact_1_normalized_residual": residual_1,
        "contact_2_normalized_residual": residual_2,
        "contact_1_objective_component": residual_1 * residual_1,
        "contact_2_objective_component": residual_2 * residual_2,
        "contact_1_to_bar_angle_deg": angle_1,
        "contact_2_to_bar_angle_deg": angle_2,
        "contact_1_orthogonality_error_deg": abs(90.0 - angle_1),
        "contact_2_orthogonality_error_deg": abs(90.0 - angle_2),
    }


def _decorate_s2_t1_solution(
    solution: dict,
    n1: np.ndarray,
    ce1: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    distance: float,
) -> dict:
    sign_1, sign_2 = _solution_sign_components(solution, n1, ce1, n2, ce2)
    solution["sign_family"] = _sign_family_label(sign_1, sign_2)
    solution["optimization_diagnostics"] = _solution_optimization_diagnostics(
        solution,
        n1,
        ce1,
        n2,
        ce2,
        distance,
    )
    return solution


def _target_family_seed_angles(anchor_solution: dict, target_sign_family: str) -> Tuple[float, float]:
    anchor_sign_1, anchor_sign_2 = _parse_sign_family(anchor_solution["sign_family"])
    target_sign_1, target_sign_2 = _parse_sign_family(target_sign_family)
    theta_1, theta_2 = anchor_solution["angles"]
    if anchor_sign_1 != target_sign_1:
        theta_1 += math.pi
    if anchor_sign_2 != target_sign_2:
        theta_2 += math.pi
    return theta_1, theta_2


def _candidate_initial_circle_angles(
    n1: np.ndarray,
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    n2: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    hint: Optional[Sequence[float]],
    *,
    tol: float,
) -> Tuple[float, float]:
    reference = ce2 - ce1
    if hint is not None:
        try:
            hint_vector = _as_vector(hint)
        except ValueError:
            hint_vector = None
        else:
            if np.linalg.norm(hint_vector) > tol:
                reference = hint_vector

    if np.linalg.norm(reference) <= tol:
        reference = basis1_u + basis2_u

    ref1 = reference - float(np.dot(reference, n1)) * n1
    ref2 = reference - float(np.dot(reference, n2)) * n2
    theta1 = _angle_on_circle(ref1, basis1_u, basis1_v, tol=tol)
    theta2 = _angle_on_circle(ref2, basis2_u, basis2_v, tol=tol)
    return theta1, theta2


def _solve_s2_t1_residuals(
    params: np.ndarray,
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
) -> np.ndarray:
    theta1 = float(params[0])
    theta2 = float(params[1])
    p1 = _point_on_circle(ce1, basis1_u, basis1_v, theta1, distance)
    p2 = _point_on_circle(ce2, basis2_u, basis2_v, theta2, distance)
    diff = p2 - p1
    diff_norm = float(np.linalg.norm(diff))
    if diff_norm <= tol:
        return np.array([1e6, 1e6], dtype=float)

    nn = diff / diff_norm
    radial_1 = p1 - ce1
    radial_2 = p2 - ce2
    scale = max(abs(distance), 1.0)
    return np.array(
        [
            float(np.dot(radial_1, nn)) / scale,
            float(np.dot(radial_2, nn)) / scale,
        ],
        dtype=float,
    )


def _solve_s2_t1_objective(
    params: np.ndarray,
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
) -> float:
    residuals = _solve_s2_t1_residuals(
        params,
        ce1,
        basis1_u,
        basis1_v,
        ce2,
        basis2_u,
        basis2_v,
        distance,
        tol=tol,
    )
    return float(np.dot(residuals, residuals))


def _build_s2_t1_solution(
    params: np.ndarray,
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
    residual: float,
) -> Optional[dict]:
    theta1 = _wrap_angle(float(params[0]))
    theta2 = _wrap_angle(float(params[1]))
    p1 = _point_on_circle(ce1, basis1_u, basis1_v, theta1, distance)
    p2 = _point_on_circle(ce2, basis2_u, basis2_v, theta2, distance)

    diff = p2 - p1
    diff_norm = float(np.linalg.norm(diff))
    if diff_norm <= tol:
        return None

    nn = diff / diff_norm
    return {
        "nn": nn,
        "p1": p1,
        "p2": p2,
        "residual": float(residual),
        "angles": (theta1, theta2),
    }


def _optimize_s2_t1_from_seed(
    seed_angles: Sequence[float],
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
) -> Optional[dict]:
    attempt = _run_s2_t1_attempt_from_seed(
        seed_angles,
        ce1,
        basis1_u,
        basis1_v,
        ce2,
        basis2_u,
        basis2_v,
        distance,
        tol=tol,
    )
    return attempt["solution"]


def _run_s2_t1_attempt_from_seed(
    seed_angles: Sequence[float],
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
    target_sign_family: Optional[str] = None,
) -> dict:
    def residuals(params: np.ndarray) -> np.ndarray:
        return _solve_s2_t1_residuals(
            params,
            ce1,
            basis1_u,
            basis1_v,
            ce2,
            basis2_u,
            basis2_v,
            distance,
            tol=tol,
        )

    result = least_squares(
        residuals,
        np.asarray(seed_angles, dtype=float),
        xtol=_S2_T1_LSQ_XTOL,
        ftol=_S2_T1_LSQ_FTOL,
        gtol=_S2_T1_LSQ_GTOL,
        max_nfev=_S2_T1_LSQ_MAX_NFEV,
    )
    objective_value = float(np.dot(result.fun, result.fun))
    optimizer_result = {
        "nfev": int(result.nfev),
        "status": int(result.status),
        "success": bool(result.success),
        "termination_reason": _termination_reason_label(result.message, result.status),
        "message": str(result.message),
    }
    attempt = {
        "target_sign_family": target_sign_family,
        "seed_angles": (
            _wrap_angle(float(seed_angles[0])),
            _wrap_angle(float(seed_angles[1])),
        ),
        "final_angles": (
            _wrap_angle(float(result.x[0])),
            _wrap_angle(float(result.x[1])),
        ),
        "objective": objective_value,
        "objective_components": [
            float(result.fun[0] * result.fun[0]),
            float(result.fun[1] * result.fun[1]),
        ],
        "normalized_residuals": [float(result.fun[0]), float(result.fun[1])],
        "least_squares_cost": float(result.cost),
        **optimizer_result,
    }
    solution = _build_s2_t1_solution(
        np.asarray(result.x, dtype=float),
        ce1,
        basis1_u,
        basis1_v,
        ce2,
        basis2_u,
        basis2_v,
        distance,
        tol=tol,
        residual=objective_value,
    )
    if solution is not None:
        _decorate_s2_t1_solution(
            solution,
            _unit(np.cross(basis1_u, basis1_v), tol=tol),
            ce1,
            _unit(np.cross(basis2_u, basis2_v), tol=tol),
            ce2,
            distance,
        )
        solution["optimizer_result"] = dict(optimizer_result)
        attempt["converged_sign_family"] = solution["sign_family"]
        attempt["optimization_diagnostics"] = solution["optimization_diagnostics"]
    else:
        attempt["converged_sign_family"] = None
        attempt["optimization_diagnostics"] = None
    attempt["solution"] = solution
    return attempt


def _collect_s2_t1_seed_family(
    anchor_angles: Sequence[float],
    ce1: np.ndarray,
    basis1_u: np.ndarray,
    basis1_v: np.ndarray,
    ce2: np.ndarray,
    basis2_u: np.ndarray,
    basis2_v: np.ndarray,
    distance: float,
    *,
    tol: float,
    residual_threshold: float,
) -> list[dict]:
    seeds = [
        (anchor_angles[0], anchor_angles[1]),
        (anchor_angles[0] + math.pi, anchor_angles[1]),
        (anchor_angles[0], anchor_angles[1] + math.pi),
        (anchor_angles[0] + math.pi, anchor_angles[1] + math.pi),
    ]
    solutions = []
    pos_tol = max(1e-3, 10.0 * tol * max(1.0, distance))
    dir_tol = 1e-6
    for seed_angles in seeds:
        solution = _optimize_s2_t1_from_seed(
            seed_angles,
            ce1,
            basis1_u,
            basis1_v,
            ce2,
            basis2_u,
            basis2_v,
            distance,
            tol=tol,
        )
        if solution is None or solution["residual"] >= residual_threshold:
            continue
        _append_unique_s2_t1_solution(
            solutions,
            solution,
            pos_tol=pos_tol,
            dir_tol=dir_tol,
        )
    return solutions


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


def solve_s2_t1_report(
    n1: Iterable[float],
    x_ce1: Iterable[float],
    n2: Iterable[float],
    x_ce2: Iterable[float],
    distance: float,
    nn_init_hint: Optional[Sequence[float]] = None,
    *,
    tol: float = 1e-6,
    residual_threshold: float = 1e-4,
) -> dict:
    """Solve S2-T1 and return solutions plus diagnostic metadata."""

    n1 = _unit(n1)
    n2 = _unit(n2)
    ce1 = _as_vector(x_ce1)
    ce2 = _as_vector(x_ce2)
    distance = float(distance)
    if distance <= 0.0:
        raise ValueError("Contact distance must be positive.")

    basis1_u, basis1_v = _circle_basis(n1, tol=tol)
    basis2_u, basis2_v = _circle_basis(n2, tol=tol)
    initial_angles = _candidate_initial_circle_angles(
        n1,
        ce1,
        basis1_u,
        basis1_v,
        n2,
        ce2,
        basis2_u,
        basis2_v,
        nn_init_hint,
        tol=tol,
    )
    solutions = []
    pos_tol = max(1e-3, 10.0 * tol * max(1.0, distance))
    dir_tol = 1e-6
    initial_family = _collect_s2_t1_seed_family(
        initial_angles,
        ce1,
        basis1_u,
        basis1_v,
        ce2,
        basis2_u,
        basis2_v,
        distance,
        tol=tol,
        residual_threshold=residual_threshold,
    )
    for solution in initial_family:
        _append_unique_s2_t1_solution(
            solutions,
            solution,
            pos_tol=pos_tol,
            dir_tol=dir_tol,
        )

    for anchor_solution in list(initial_family):
        branch_family = _collect_s2_t1_seed_family(
            anchor_solution["angles"],
            ce1,
            basis1_u,
            basis1_v,
            ce2,
            basis2_u,
            basis2_v,
            distance,
            tol=tol,
            residual_threshold=residual_threshold,
        )
        for solution in branch_family:
            _append_unique_s2_t1_solution(
                solutions,
                solution,
                pos_tol=pos_tol,
                dir_tol=dir_tol,
            )

    for solution in solutions:
        _decorate_s2_t1_solution(solution, n1, ce1, n2, ce2, distance)

    solutions.sort(key=lambda item: item["residual"])
    found_sign_families = []
    for sign_pair in _SIGN_FAMILY_ORDER:
        label = _sign_family_label(*sign_pair)
        if any(solution.get("sign_family") == label for solution in solutions):
            found_sign_families.append(label)
    missing_sign_families = [
        _sign_family_label(*sign_pair)
        for sign_pair in _SIGN_FAMILY_ORDER
        if _sign_family_label(*sign_pair) not in found_sign_families
    ]
    missing_family_attempts = []
    if solutions:
        anchor_solution = solutions[0]
        for missing_family in missing_sign_families:
            missing_family_attempts.append(
                _run_s2_t1_attempt_from_seed(
                    _target_family_seed_angles(anchor_solution, missing_family),
                    ce1,
                    basis1_u,
                    basis1_v,
                    ce2,
                    basis2_u,
                    basis2_v,
                    distance,
                    tol=tol,
                    target_sign_family=missing_family,
                )
            )
    return {
        "initial_angles": tuple(float(angle) for angle in initial_angles),
        "optimizer_settings": _optimizer_settings_dict(),
        "solution_count": len(solutions),
        "found_sign_families": found_sign_families,
        "missing_sign_families": missing_sign_families,
        "missing_family_attempts": missing_family_attempts,
        "solutions": solutions,
    }


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
    """Solve S2-T1 via circle-angle parameterization and return converged solutions."""

    report = solve_s2_t1_report(
        n1,
        x_ce1,
        n2,
        x_ce2,
        distance,
        nn_init_hint=nn_init_hint,
        tol=tol,
        residual_threshold=residual_threshold,
    )
    return report["solutions"]


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
