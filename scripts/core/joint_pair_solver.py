"""Joint-pair optimizer: align female + male screw frames by adjusting bar-side DOFs.

Operates on the :class:`JointPairDef` representation (see
:mod:`core.joint_pair`).  Solves a 4-DOF problem (two prismatic
positions ``fjp``/``mjp`` and two revolute angles ``fjr``/``mjr``) so
that the female and male screw frames coincide in origin and Z-axis.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import minimize

from core.geometry import closest_params_infinite_lines
from core.joint_pair import (
    JointPairDef,
    canonical_bar_frame_from_line,
    fk_half_from_bar_frame,
)
from core.transforms import as_vector


def screw_alignment_error(
    female_screw_frame: np.ndarray,
    male_screw_frame: np.ndarray,
    *,
    orientation_weight_mm: float = 1.0,
) -> float:
    """Squared origin error + weighted squared local-Z error."""

    f = np.asarray(female_screw_frame, dtype=float)
    m = np.asarray(male_screw_frame, dtype=float)
    origin_delta = f[:3, 3] - m[:3, 3]
    z_delta = f[:3, 2] - m[:3, 2]
    return float(
        np.dot(origin_delta, origin_delta)
        + (orientation_weight_mm ** 2) * np.dot(z_delta, z_delta)
    )


def screw_alignment_diagnostics(
    female_screw_frame: np.ndarray, male_screw_frame: np.ndarray
) -> dict[str, float]:
    f = np.asarray(female_screw_frame, dtype=float)
    m = np.asarray(male_screw_frame, dtype=float)
    origin_error_mm = float(np.linalg.norm(f[:3, 3] - m[:3, 3]))
    dot = float(np.clip(np.dot(f[:3, 2], m[:3, 2]), -1.0, 1.0))
    z_axis_error_rad = float(math.acos(dot))
    return {"origin_error_mm": origin_error_mm, "z_axis_error_rad": z_axis_error_rad}


def _seed_grid(fjp0: float, mjp0: float, jr_range: tuple[float, float]) -> list[np.ndarray]:
    # Dense product over (fjr, mjr) on a 6-spoke wheel covering [-pi, pi).
    # The wheel resolution (60 deg) is enough to put every local basin within
    # one spoke of a seed, while keeping the total seed count modest.
    angles = [k * math.pi / 3.0 for k in range(-3, 3)]
    return [
        np.array([fjp0, fjr, mjp0, mjr], dtype=float)
        for fjr in angles
        for mjr in angles
    ]


def _random_seeds(
    fjp0: float, mjp0: float, jp_range, jr_range, n: int, perturb_mm: float
) -> list[np.ndarray]:
    rng = np.random.default_rng(0)
    seeds: list[np.ndarray] = []
    for _ in range(n):
        seeds.append(
            np.array(
                [
                    fjp0 + rng.uniform(-perturb_mm, perturb_mm),
                    rng.uniform(*jr_range),
                    mjp0 + rng.uniform(-perturb_mm, perturb_mm),
                    rng.uniform(*jr_range),
                ],
                dtype=float,
            )
        )
    return seeds


def optimize_pair_placement(
    le_start: Iterable[float],
    le_end: Iterable[float],
    ln_start: Iterable[float],
    ln_end: Iterable[float],
    pair: JointPairDef,
    *,
    random_restarts: int = 12,
    translation_perturbation_mm: float = 50.0,
    fjp_range: tuple[float, float] | None = None,
    mjp_range: tuple[float, float] | None = None,
    jp_margin: float = 100.0,
    return_debug: bool = False,
) -> dict:
    """Solve for `(fjp, fjr, mjp, mjr)` that mate the two halves' screw frames.

    The translation bounds default to the actual bar lengths (with a small
    ``jp_margin`` on each side) rather than ``pair.jp_range``, because the
    natural domain of the bar-axis offset depends on the *current* bar, not
    on a global per-pair default.  Pass ``fjp_range`` / ``mjp_range`` to
    override.
    """

    le_s = as_vector(le_start)
    le_e = as_vector(le_end)
    ln_s = as_vector(ln_start)
    ln_e = as_vector(ln_end)

    le_dir = le_e - le_s
    ln_dir = ln_e - ln_s
    le_len = float(np.linalg.norm(le_dir))
    ln_len = float(np.linalg.norm(ln_dir))
    if le_len <= 1e-12 or ln_len <= 1e-12:
        raise ValueError("Bars must have non-zero length.")

    le_unit = le_dir / le_len
    ln_unit = ln_dir / ln_len
    le_bar_frame = canonical_bar_frame_from_line(le_s, le_e)
    ln_bar_frame = canonical_bar_frame_from_line(ln_s, ln_e)

    orientation_weight_mm = float(pair.contact_distance_mm) or 1.0

    def objective(params: np.ndarray) -> float:
        fjp, fjr, mjp, mjr = params
        fside = fk_half_from_bar_frame(le_bar_frame, fjp, fjr, pair.female)
        mside = fk_half_from_bar_frame(ln_bar_frame, mjp, mjr, pair.male)
        return screw_alignment_error(
            fside["screw_frame"],
            mside["screw_frame"],
            orientation_weight_mm=orientation_weight_mm,
        )

    t_e, t_n = closest_params_infinite_lines(le_s, le_unit, ln_s, ln_unit)
    fjp0 = float(t_e)
    mjp0 = float(t_n)

    if fjp_range is None:
        fjp_range = (-jp_margin, le_len + jp_margin)
    if mjp_range is None:
        mjp_range = (-jp_margin, ln_len + jp_margin)
    bounds = [fjp_range, pair.jr_range, mjp_range, pair.jr_range]

    seeds = _seed_grid(fjp0, mjp0, pair.jr_range)
    seeds.extend(
        _random_seeds(
            fjp0,
            mjp0,
            pair.jp_range,
            pair.jr_range,
            random_restarts,
            translation_perturbation_mm,
        )
    )

    best = None
    best_idx = -1
    reports: list[dict] = []
    for idx, seed in enumerate(seeds):
        result = minimize(objective, seed, method="L-BFGS-B", bounds=bounds)
        if return_debug:
            reports.append(
                {
                    "seed_index": idx,
                    "seed": tuple(float(v) for v in seed),
                    "x": tuple(float(v) for v in result.x),
                    "fun": float(result.fun),
                    "success": bool(result.success),
                    "nit": int(getattr(result, "nit", -1)),
                }
            )
        if best is None or float(result.fun) < float(best.fun):
            best = result
            best_idx = idx
    assert best is not None

    fjp, fjr, mjp, mjr = (float(v) for v in best.x)
    fside = fk_half_from_bar_frame(le_bar_frame, fjp, fjr, pair.female)
    mside = fk_half_from_bar_frame(ln_bar_frame, mjp, mjr, pair.male)
    diag = screw_alignment_diagnostics(fside["screw_frame"], mside["screw_frame"])

    output = {
        "fjp": fjp,
        "fjr": fjr,
        "mjp": mjp,
        "mjr": mjr,
        "residual": float(best.fun),
        "origin_error_mm": diag["origin_error_mm"],
        "z_axis_error_rad": diag["z_axis_error_rad"],
        "female_frame": fside["block_frame"],
        "male_frame": mside["block_frame"],
        "female_screw_frame": fside["screw_frame"],
        "male_screw_frame": mside["screw_frame"],
    }
    if return_debug:
        output["debug"] = {
            "seed_count": len(seeds),
            "best_seed_index": int(best_idx),
            "seed_reports": reports,
        }
    return output
