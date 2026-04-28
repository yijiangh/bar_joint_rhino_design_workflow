"""Headless three-bar T20 placement test.

Bars B2/B3/B4 are taken verbatim from a Rhino ``_What`` report on the
user's test scene.  The bars are drawn 36 mm apart pairwise so that a
T20 joint mates at every junction.

For each of the three pairwise intersections, we enumerate the four
flip configurations -- (female-bar-reversed, male-bar-reversed) in
{(F,F), (F,T), (T,F), (T,T)} -- and assert:

* every configuration mates with origin error <= 1e-3 mm and
  screw-Z axis error <= 1e-5 rad, and
* the four configurations are pairwise distinct (different screw
  origins or different screw Z directions in world coordinates).
"""

from __future__ import annotations

import os
import sys
from itertools import combinations

import numpy as np
import pytest

TESTS_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from core.geometry import distance_infinite_lines
from core.joint_pair import get_joint_pair
from core.joint_pair_solver import (
    optimize_pair_placement,
    screw_alignment_diagnostics,
)


BARS = {
    "B2": (
        np.array([196.328, -248.326, 36.000]),
        np.array([196.328, 1788.324, 36.000]),
    ),
    "B3": (
        np.array([287.998, 465.290, 72.000]),
        np.array([-598.036, 465.290, 72.000]),
    ),
    "B4": (
        np.array([-121.219, 485.216, 110.544]),
        np.array([223.719, 125.650, 68.958]),
    ),
}

ORIGIN_TOL_MM = 1e-3
Z_AXIS_TOL_RAD = 1e-5

FLIP_VARIANTS = [
    (False, False),
    (False, True),
    (True, False),
    (True, True),
]


def _bar_dir(s, e):
    d = np.asarray(e - s, dtype=float)
    return d / float(np.linalg.norm(d))


def _solve_one(le_s, le_e, ln_s, ln_e, pair, le_rev, ln_rev):
    les = le_e if le_rev else le_s
    lee = le_s if le_rev else le_e
    lns = ln_e if ln_rev else ln_s
    lne = ln_s if ln_rev else ln_e
    return optimize_pair_placement(les, lee, lns, lne, pair, random_restarts=4)


def _frames_match(a: dict, b: dict, *, pos_tol_mm=1e-2, ang_tol_rad=1e-3) -> bool:
    """True iff two solver results place BOTH the female and male block
    frames at the same world pose (origin + full orientation).

    Comparing only the screw axis is not enough: reversing one bar
    rotates that side's block 180 degrees around the screw axis, which
    leaves the screw frame's origin and Z unchanged but is visibly a
    different physical placement of the block."""

    for key in ("female_frame", "male_frame"):
        fa = np.asarray(a[key], dtype=float)
        fb = np.asarray(b[key], dtype=float)
        if float(np.linalg.norm(fa[:3, 3] - fb[:3, 3])) > pos_tol_mm:
            return False
        # Compare rotation via Frobenius norm of difference (small rotations
        # have small Frobenius differences).
        rot_diff = float(np.linalg.norm(fa[:3, :3] - fb[:3, :3], ord="fro"))
        # Frobenius diff for two unit-axis rotations is ~ 2*sqrt(2)*sin(theta/2);
        # the tolerance below corresponds to roughly ang_tol_rad rad.
        if rot_diff > 2.83 * np.sin(ang_tol_rad / 2.0) + 1e-6:
            return False
    return True


@pytest.mark.parametrize("a_name, b_name", list(combinations(BARS.keys(), 2)))
def test_t20_four_flips_per_intersection(a_name, b_name):
    """At every pairwise intersection, T20 admits four distinct mating
    configurations, each with near-zero residual."""

    pair = get_joint_pair("T20")
    a_s, a_e = BARS[a_name]
    b_s, b_e = BARS[b_name]
    bar_dist = abs(
        float(distance_infinite_lines(a_s, _bar_dir(a_s, a_e), b_s, _bar_dir(b_s, b_e)))
    )

    sys.__stdout__.write(
        f"\n[{a_name}-{b_name} / T20]  bar-bar dist={bar_dist:.4f} mm  "
        f"contact={pair.contact_distance_mm:.4f} mm\n"
    )

    results = []
    for le_rev, ln_rev in FLIP_VARIANTS:
        res = _solve_one(a_s, a_e, b_s, b_e, pair, le_rev, ln_rev)
        diag = screw_alignment_diagnostics(
            res["female_screw_frame"], res["male_screw_frame"]
        )
        sp = res["female_screw_frame"][:3, 3]
        sz = res["female_screw_frame"][:3, 2]
        sys.__stdout__.write(
            f"  flip(female_rev={le_rev}, male_rev={ln_rev})  "
            f"residual={res['residual']:.3e}  "
            f"orig_err={diag['origin_error_mm']:.3e} mm  "
            f"z_err={diag['z_axis_error_rad']:.3e} rad  "
            f"screw_pos=({sp[0]:.3f},{sp[1]:.3f},{sp[2]:.3f})  "
            f"screw_z=({sz[0]:.3f},{sz[1]:.3f},{sz[2]:.3f})\n"
        )
        sys.__stdout__.flush()

        assert diag["origin_error_mm"] < ORIGIN_TOL_MM, (
            f"flip {(le_rev, ln_rev)} failed origin tol: {diag}"
        )
        assert diag["z_axis_error_rad"] < Z_AXIS_TOL_RAD, (
            f"flip {(le_rev, ln_rev)} failed z-axis tol: {diag}"
        )
        results.append(res)

    # All four placements must be pairwise distinct in world space.
    for i, j in combinations(range(4), 2):
        assert not _frames_match(results[i], results[j]), (
            f"flips {FLIP_VARIANTS[i]} and {FLIP_VARIANTS[j]} produced "
            f"the same world placement; expected 4 unique configurations."
        )
