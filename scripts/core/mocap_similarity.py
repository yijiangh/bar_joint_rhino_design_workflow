"""Which bars could Motive mix up?  -- the bar-against-bar MoCap check.

Motive tells tracked bars apart only by the SHAPE their marker spheres form.
Two bars whose spheres form the same shape -- the same MoCap joint placement, or
the same placement seen from the other end of the bar -- look identical to it,
so it cannot say which bar is which.  ``core.mocap_layout_check`` already
decides whether ONE bar's layout fits itself in two poses (the export refuses
those); this module asks the same question between two DIFFERENT bars.

What "similar" means here, precisely::

    some PROPER rigid transform (a rotation with det = +1, plus a translation --
    no mirror) lays every marker of bar A onto a marker of bar B with a maximum
    residual <= tol.

That is :func:`core.mocap_layout_check.match` with bar B as the "measured" set,
so the two checks share one rule.  A mirror image does not count: a bar cannot
be turned into its mirror.  Bars with different marker counts are never
compared -- with every marker in view, Motive does not confuse them.

Layouts are given in each bar's OWN frame (origin at the bar start, z along the
bar), so the fit also says HOW the bars relate: the same placement, the same
placement end for end, or turned about the bar.

Pure numpy, no Rhino.  Units are METRES, as in ``core.mocap_layout_check``; the
Rhino caller (RSCheckMoCapLayout) divides by 1000.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core import joint_name_conventions as jnc
from core.mocap_layout_check import (
    MoCapLayoutError,
    check_layout,
    fit_rigid,
    match,
    rotation_angle_axis,
)

#: Below this rotation (degrees) two bars count as the "same placement".
SAME_PLACEMENT_MAX_DEG = 5.0

#: Above this rotation (degrees), about an axis across the bar, they count as
#: "end for end".
END_FOR_END_MIN_DEG = 175.0


@dataclass(frozen=True)
class SimilarPair:
    """Two bars whose marker layouts match within the tolerance."""

    bar_a: str
    bar_b: str
    n_markers: int
    max_residual_m: float
    relation: str               # how B sits relative to A, in plain words

    def describe(self) -> str:
        """``"B22 = B20 end for end, within 0.4 mm"``."""
        return (
            f"{self.bar_b} = {self.bar_a} {self.relation}, "
            f"within {self.max_residual_m * 1000:.1f} mm"
        )


def _relation(rotation, translation) -> str:
    """How the fit moves bar A's frame onto bar B's, in plain words.

    Both layouts are in their own bar frame (z along the bar), so the rotation
    axis says it all: none -> same placement; a half-turn about an axis ACROSS
    the bar -> end for end; about the bar itself -> turned about the bar.
    """
    angle_deg, axis = rotation_angle_axis(rotation)
    along_bar = abs(axis[2])
    if angle_deg < SAME_PLACEMENT_MAX_DEG:
        shift_mm = float(np.linalg.norm(translation)) * 1000.0
        if shift_mm < 5.0:
            return "same placement"
        return f"same placement, {shift_mm:.0f} mm further along"
    if angle_deg > END_FOR_END_MIN_DEG and along_bar < 0.2:
        return "end for end"
    if along_bar > 0.98:
        return f"turned {angle_deg:.0f} deg about the bar"
    return f"rotated {angle_deg:.0f} deg"


def _distances_can_match(points_a, points_b, tol_m: float) -> bool:
    """Quick reject: can ANY assignment keep every pairwise distance within ``2 * tol``?

    A rigid motion that moves each marker by at most ``tol`` changes each
    distance by at most ``2 * tol``, and if some pairing of two distance lists
    stays within a bound, so do the two lists sorted.  So sorted lists that
    differ by more prove no match exists, without the assignment search.
    """
    def sorted_distances(points):
        pts = np.asarray(points, dtype=float)
        dist = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
        return np.sort(dist[np.triu_indices(len(pts), k=1)])

    return bool(np.max(np.abs(sorted_distances(points_a) - sorted_distances(points_b))) <= 2.0 * tol_m)


def compare_bars(layouts_m: dict, tol_m: float) -> tuple:
    """Every pair of bars whose layouts match within *tol_m*.

    Args:
        layouts_m: ``{bar_id: (n, 3) marker centres}``, metres, each in its own
            bar frame.  Order of the markers carries no meaning.
        tol_m: largest per-marker residual a match may have.

    Returns:
        ``(pairs, unchecked)`` -- ``pairs`` a list of :class:`SimilarPair`
        (``bar_a`` before ``bar_b`` in bar order); ``unchecked`` a list of
        ``(bar_a, bar_b, reason)`` for pairs whose layout was too regular to
        search (``MoCapLayoutError``), which the caller should report.
    """
    bar_ids = sorted(layouts_m, key=jnc.bar_sort_key)
    pairs, unchecked = [], []
    for i, bar_a in enumerate(bar_ids):
        points_a = np.asarray(layouts_m[bar_a], dtype=float).reshape(-1, 3)
        for bar_b in bar_ids[i + 1:]:
            points_b = np.asarray(layouts_m[bar_b], dtype=float).reshape(-1, 3)
            if len(points_a) != len(points_b) or len(points_a) < 3:
                continue
            if not _distances_can_match(points_a, points_b, tol_m):
                continue
            try:
                found = match(points_a, points_b, tol_m)
            except MoCapLayoutError as exc:
                unchecked.append((bar_a, bar_b, str(exc)))
                continue
            if not found:
                continue
            best = found[0]   # match() sorts by residual
            rotation, translation = fit_rigid(points_a[list(best.permutation)], points_b)
            pairs.append(SimilarPair(
                bar_a, bar_b, len(points_a), best.max_residual_m,
                _relation(rotation, translation),
            ))
    return pairs, unchecked


def similar_groups(pairs) -> list:
    """Bars joined by similar pairs, as groups: A~B and B~C put A, B, C together.

    Returns a list of bar-id lists, each in bar order, groups ordered by their
    first bar.
    """
    parent: dict = {}

    def root(bar_id):
        parent.setdefault(bar_id, bar_id)
        while parent[bar_id] != bar_id:
            parent[bar_id] = parent[parent[bar_id]]
            bar_id = parent[bar_id]
        return bar_id

    for pair in pairs:
        parent[root(pair.bar_a)] = root(pair.bar_b)
    groups: dict = {}
    for bar_id in list(parent):
        groups.setdefault(root(bar_id), []).append(bar_id)
    ordered = [sorted(members, key=jnc.bar_sort_key) for members in groups.values()]
    return sorted(ordered, key=lambda members: jnc.bar_sort_key(members[0]))


def refused_bars(layouts_m: dict, tol_m: float) -> dict:
    """``{bar_id: reason}`` for every bar the export would refuse.

    The per-bar check of ``core.mocap_layout_check`` (fewer than 3 markers, all
    on one line, or a layout that fits itself in two poses), run here so the
    problem shows up before the export stops on it.  A layout too regular to
    search stops the export too, so it is reported the same way.
    """
    out = {}
    for bar_id in sorted(layouts_m, key=jnc.bar_sort_key):
        try:
            report = check_layout(layouts_m[bar_id], tol_m)
        except MoCapLayoutError as exc:
            out[bar_id] = str(exc)
            continue
        if not report.ok:
            out[bar_id] = report.reason()
    return out


__all__ = [
    "SimilarPair",
    "compare_bars",
    "refused_bars",
    "similar_groups",
]
