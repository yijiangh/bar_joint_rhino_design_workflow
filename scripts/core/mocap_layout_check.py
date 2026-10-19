"""Can a bar's MoCap marker layout give ONE 6-DoF pose?  -- the layout check.

The teleop monitor reads the markers of a bar from Motive as loose points (the
marker ids shuffle, so they are not trusted) and has to work out which measured
point is which designed marker before it can fit the bar's pose.  That only has
a unique answer when no OTHER rigid motion maps the designed layout onto itself.
This module decides that, and the export refuses a bar that fails.

What "ambiguous" means here, precisely::

    there is a NON-IDENTITY permutation of the markers that a PROPER rigid
    transform (a rotation with det = +1, plus a translation -- no mirror)
    reproduces with a maximum residual <= tol over every marker.

Why not just compare pairwise-distance patterns ("signatures")?  Two reasons,
both checked on a real export:

* on a long bar the two markers of one 48 mm pair have almost the same
  distances to every other marker (those sit a metre away), so their
  signatures agree within a fraction of a millimetre -- a signature rule would
  refuse perfectly fine bars;
* signatures cannot tell a mirror image from a rotation, but a mirror is not a
  pose the physical bar can take, so it must NOT count as an ambiguity.

So signatures are used only to shrink the search; the decision is always the
full-set residual of a proper rigid fit.

Two exact ambiguities of the ``T20_MoCap`` plate (two markers, 48.3 mm apart,
25.5 mm off the bar axis) are worth knowing when a bar is refused:

* **two joints are never enough**, whatever their spacing or relative turn
  (a half-turn about the line bisecting their plate normals maps the set onto
  itself end for end);
* **joints that all face the same way are never enough** (a half-turn about the
  plate midline swaps M1 and M2 in every joint).

Rule of thumb: three or more MoCap joints per tracked bar, not all facing the
same way, not mirror-symmetric about the bar's middle.

Pure numpy, no Rhino.  Units are METRES, as in ``MoCapMarkers.json``; the Rhino
caller divides by 1000.  :func:`match` is also the reference answer for the
monitor's own matcher (same tolerance, same det = +1 rule).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Default tolerance in metres (2 mm): about twice the Motive per-marker noise.
DEFAULT_TOL_M = 0.002

#: Safety cap on how many complete marker assignments the search may visit
#: before giving up; real layouts need a few dozen.
MAX_ASSIGNMENTS = 200_000


class MoCapLayoutError(RuntimeError):
    """A bar's marker layout cannot give a unique 6-DoF pose."""


# * ---------------------------------------------------------------------------
# * Rigid fit
# * ---------------------------------------------------------------------------


def fit_rigid(source, target) -> tuple:
    """Best proper rigid transform ``(R, t)`` with ``target ~= R @ source + t``.

    Least squares over matching rows (Kabsch, via SVD).  The determinant of
    ``R`` is forced to +1: when the best orthogonal map would be a mirror, the
    last singular direction is flipped so a rotation is returned instead.
    Needs at least three points.

    Args:
        source: ``(n, 3)`` points.
        target: ``(n, 3)`` points, row ``i`` matching ``source[i]``.

    Returns:
        ``(R, t)`` -- a ``(3, 3)`` rotation and a ``(3,)`` translation.
    """
    src = np.asarray(source, dtype=float).reshape(-1, 3)
    dst = np.asarray(target, dtype=float).reshape(-1, 3)
    if len(src) != len(dst) or len(src) < 3:
        raise ValueError(f"fit_rigid needs >= 3 matching points, got {len(src)} and {len(dst)}")
    src_centre, dst_centre = src.mean(axis=0), dst.mean(axis=0)
    cross_cov = (src - src_centre).T @ (dst - dst_centre)
    u, _singular, vt = np.linalg.svd(cross_cov)
    # Sign of det(V U^T): -1 means the best orthogonal map is a mirror.
    sign = np.sign(np.linalg.det(vt.T @ u.T))
    if sign == 0:
        sign = 1.0
    rotation = vt.T @ np.diag([1.0, 1.0, sign]) @ u.T
    translation = dst_centre - rotation @ src_centre
    return rotation, translation


def rigid_residuals_m(rotation, translation, source, target) -> np.ndarray:
    """Distance, per point, between ``R @ source + t`` and ``target``."""
    moved = np.asarray(source, dtype=float) @ np.asarray(rotation).T + np.asarray(translation)
    return np.linalg.norm(moved - np.asarray(target, dtype=float), axis=1)


def rotation_angle_axis(rotation) -> tuple:
    """``(angle_deg, unit_axis)`` of a rotation matrix.

    The axis is read from the skew part of ``R``; for a half-turn that part is
    zero, so the axis is taken from ``R + I`` instead (its columns are all
    parallel to the axis then).
    """
    rot = np.asarray(rotation, dtype=float)
    cos_angle = np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)
    angle_deg = float(np.degrees(np.arccos(cos_angle)))
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    if np.linalg.norm(axis) < 1e-9:
        if angle_deg < 1e-6:
            return 0.0, (0.0, 0.0, 1.0)   # identity: any axis
        columns = rot + np.eye(3)
        axis = columns[:, int(np.argmax(np.linalg.norm(columns, axis=0)))]
    axis = axis / np.linalg.norm(axis)
    return angle_deg, tuple(float(c) for c in axis)


# * ---------------------------------------------------------------------------
# * Matching: which measured marker is which designed one
# * ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Symmetry:
    """One rigid motion that maps the designed markers onto the measured ones.

    ``permutation[i] = j`` means measured marker ``i`` is designed marker ``j``.
    In a self-check (design against itself) a non-identity entry is an
    ambiguity: the bar could be in two poses that look the same to Motive.
    """

    permutation: tuple
    angle_deg: float
    axis: tuple
    max_residual_m: float

    def describe(self) -> str:
        """``"half-turn (180.0 deg) about (0.00, 1.00, 0.00), residual 0.00 mm"``."""
        kind = "half-turn" if abs(self.angle_deg - 180.0) < 1.0 else "rotation"
        axis = ", ".join(f"{c:.2f}" for c in self.axis)
        return (
            f"{kind} ({self.angle_deg:.1f} deg) about ({axis}), "
            f"residual {self.max_residual_m * 1000:.2f} mm"
        )


def _pairwise_distances(points) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)


def _candidate_assignments(dist_design, dist_measured, tol_m: float):
    """Every assignment ``measured[i] -> design[perm[i]]`` that distances allow.

    A rigid motion that moves every point by at most ``tol`` changes a
    pairwise distance by at most ``2 * tol``, so that is the pruning bound:

    1. a measured marker may only take a designed marker whose sorted row of
       distances agrees with its own, entry by entry, within ``2 * tol``;
    2. while building the assignment, every already-placed pair must have the
       same distance in both sets within ``2 * tol``.

    Measured markers with the fewest candidates are placed first so wrong
    branches die early.  Yields permutations as tuples.
    """
    n = len(dist_measured)
    bound = 2.0 * tol_m
    sig_design = np.sort(dist_design, axis=1)
    sig_measured = np.sort(dist_measured, axis=1)
    candidates = [
        [j for j in range(n) if np.max(np.abs(sig_measured[i] - sig_design[j])) <= bound]
        for i in range(n)
    ]
    order = sorted(range(n), key=lambda i: len(candidates[i]))
    perm = [-1] * n
    used = [False] * n
    visited = [0]

    def place(k: int):
        if k == n:
            visited[0] += 1
            if visited[0] > MAX_ASSIGNMENTS:
                raise MoCapLayoutError(
                    f"more than {MAX_ASSIGNMENTS} candidate marker assignments; "
                    "the layout is far too regular to check (or tol is far too large)."
                )
            yield tuple(perm)
            return
        i = order[k]
        for j in candidates[i]:
            if used[j]:
                continue
            consistent = all(
                abs(dist_measured[i, order[kk]] - dist_design[j, perm[order[kk]]]) <= bound
                for kk in range(k)
            )
            if not consistent:
                continue
            perm[i], used[j] = j, True
            yield from place(k + 1)
            perm[i], used[j] = -1, False

    yield from place(0)


def _match_full(design_m, measured_m, tol_m: float, skip_identity: bool) -> tuple:
    """``(symmetries, nearest_miss_m)``: the assignments that fit, and the best one that did not."""
    design = np.asarray(design_m, dtype=float).reshape(-1, 3)
    measured = np.asarray(measured_m, dtype=float).reshape(-1, 3)
    if len(design) != len(measured):
        raise ValueError(f"match needs the same marker count, got {len(design)} and {len(measured)}")
    if len(design) < 3:
        return [], None
    identity = tuple(range(len(design)))
    dist_design = _pairwise_distances(design)
    dist_measured = _pairwise_distances(measured)
    found = []
    nearest_miss = None
    for perm in _candidate_assignments(dist_design, dist_measured, tol_m):
        if skip_identity and perm == identity:
            continue
        ordered_design = design[list(perm)]
        rotation, translation = fit_rigid(ordered_design, measured)
        worst = float(np.max(rigid_residuals_m(rotation, translation, ordered_design, measured)))
        if worst <= tol_m:
            angle, axis = rotation_angle_axis(rotation)
            found.append(Symmetry(perm, angle, axis, worst))
        elif nearest_miss is None or worst < nearest_miss:
            nearest_miss = worst
    found.sort(key=lambda s: s.max_residual_m)
    return found, nearest_miss


def match(design_m, measured_m, tol_m: float = DEFAULT_TOL_M) -> list:
    """Every way the designed markers can sit on the measured ones.

    Both sets must have the same count.  Ids and order of the measured points
    carry no meaning; only geometry is used.  One result = a unique match; none
    = the sets differ by more than ``tol`` (or are mirror images); several = the
    layout is ambiguous.

    Args:
        design_m: ``(n, 3)`` designed marker centres, metres (bar frame).
        measured_m: ``(n, 3)`` measured centres, metres (any frame).
        tol_m: largest per-marker residual a match may have.
    """
    found, _miss = _match_full(design_m, measured_m, tol_m, skip_identity=False)
    return found


# * ---------------------------------------------------------------------------
# * The layout check itself
# * ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LayoutReport:
    """What :func:`check_layout` found for one bar."""

    n_markers: int
    tol_m: float
    collinear: bool
    symmetries: tuple           # non-identity self-matches within tol: the ambiguities
    nearest_miss_m: float       # smallest residual of a rejected candidate; None when none

    @property
    def ok(self) -> bool:
        return self.n_markers >= 3 and not self.collinear and not self.symmetries

    def reason(self) -> str:
        """One plain sentence on why the layout is not ok; ``""`` when it is."""
        if self.n_markers < 3:
            return (
                f"only {self.n_markers} marker(s): fewer than 3 cannot fix a 6-DoF pose "
                "(a pair leaves the roll about its own axis free)"
            )
        if self.collinear:
            return "all markers lie on one line: the roll about that line is free"
        if self.symmetries:
            return (
                f"{len(self.symmetries)} other rigid motion(s) map the {self.n_markers} markers "
                f"onto themselves within {self.tol_m * 1000:.1f} mm; the first is a "
                f"{self.symmetries[0].describe()}"
            )
        return ""


def _collinear(points, tol_m: float) -> bool:
    """True when every point is within ``tol`` of the best-fit line."""
    pts = np.asarray(points, dtype=float)
    centred = pts - pts.mean(axis=0)
    _u, _s, vt = np.linalg.svd(centred, full_matrices=False)
    direction = vt[0]
    off_line = centred - np.outer(centred @ direction, direction)
    return bool(np.max(np.linalg.norm(off_line, axis=1)) <= tol_m)


def check_layout(points_m, tol_m: float = DEFAULT_TOL_M) -> LayoutReport:
    """Report whether a marker layout gives a unique 6-DoF pose.

    Args:
        points_m: ``(n, 3)`` designed marker centres of ONE bar, metres.
        tol_m: largest residual a self-match may have to count as an ambiguity.
    """
    pts = np.asarray(points_m, dtype=float).reshape(-1, 3)
    n = len(pts)
    if n < 3:
        return LayoutReport(n, tol_m, collinear=n > 0, symmetries=(), nearest_miss_m=None)
    collinear = _collinear(pts, tol_m)
    symmetries, nearest_miss = _match_full(pts, pts, tol_m, skip_identity=True)
    return LayoutReport(n, tol_m, collinear, tuple(symmetries), nearest_miss)


def require_unambiguous(points_m, tol_m: float, name: str, hint: str = "") -> LayoutReport:
    """:func:`check_layout`, raising :class:`MoCapLayoutError` for a bad layout.

    Args:
        points_m: ``(n, 3)`` designed marker centres of one bar, metres.
        tol_m: the ambiguity tolerance.
        name: what to call the bar in the message (``"bar B20"``).
        hint: an optional sentence telling the user how to fix it.
    """
    report = check_layout(points_m, tol_m)
    if not report.ok:
        message = f"{name}: MoCap marker layout is ambiguous -- {report.reason()}."
        if hint:
            message += f" {hint}"
        raise MoCapLayoutError(message)
    return report


__all__ = [
    "DEFAULT_TOL_M",
    "LayoutReport",
    "MoCapLayoutError",
    "Symmetry",
    "check_layout",
    "fit_rigid",
    "match",
    "require_unambiguous",
    "rigid_residuals_m",
    "rotation_angle_axis",
]
