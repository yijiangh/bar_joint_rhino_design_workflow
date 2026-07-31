"""Base-placement guide lines: where the mobile base may stand for one bar.

Pure numpy -- no ``rhinoscriptsyntax`` / ``Rhino`` -- so ``pytest`` can cover it
headlessly (same split as ``core.geometry`` / ``core.transforms``; the Rhino half
that bakes these lines into the document is :mod:`core.base_guide_viz`).

The guide is five lines on the walkable ground, built from a bar's two
assembly-joint centers and the resolved base heading:

1. the line joining the two joint centers, PROJECTED onto the walkable ground
   (offset 0 -- only the projection is drawn, never the airborne original);
2. three copies of it offset AGAINST the assembly direction by each entry of
   ``config.BASE_GUIDE_OFFSETS_MM`` (375 / 500 / 625 mm by default);
3. the "extension line" joining the midpoints of those four lines.

The extension line is the one that matters: ``keyframe.walkable_ground.
derive_seed_base`` places the base by stepping the standoff distance against the
heading from the joint-center ground projection, so the placed base origin lies
ON this line, at the offset line matching the standoff. Seeing the two together
is what lets the user judge a standoff before committing to it.

Every endpoint is re-snapped onto the ground with ``closest_point_on_meshes`` so
the guide follows a stepped or sloped WalkableGround instead of floating over it.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from core import config


_EPS = 1e-9


def _walkable_np():
    """The shared ground-projection math, imported lazily.

    ``husky_assembly_tamp.keyframe.walkable_ground`` pulls in ``compas`` at
    import time, which is not installed in a bare test environment. Only the
    guide-line builder needs it, so it is imported inside the functions that use
    it -- keeping the heading / arm-side maths in this module importable (and
    testable) anywhere. Same reasoning as the lazy Rhino imports documented in
    ``docs/Su_note.md`` section 12.
    """
    from husky_assembly_tamp.keyframe import walkable_ground  # noqa: PLC0415

    return walkable_ground


def _unit(vector) -> Optional[np.ndarray]:
    """Return ``vector`` normalized, or ``None`` when it is (near) zero-length."""
    v = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(v))
    if norm <= _EPS:
        return None
    return v / norm


def _snap(soups, point_mm):
    """Closest point on the ground soups, or the point itself if none is found."""
    snapped, _normal = _walkable_np().closest_point_on_meshes(soups, point_mm)
    return np.asarray(point_mm if snapped is None else snapped, dtype=float)


def resolve_heading_from_axes(axes, ground_normal,
                              min_horizontal=None, cancel_tol=None) -> dict:
    """Average a bar's anchor insertion axes into one ground-plane heading.

    This is the pure-math half of
    ``core.rhino_walkable_ground.resolve_bar_heading`` (which supplies the axes
    from the Rhino document). Two guards make it disagree with a naive average,
    and both are there because the naive version put bases on arbitrary sides:

    1. **Flatten each axis onto the ground BEFORE averaging**, and drop the ones
       whose horizontal part is shorter than ``min_horizontal``. A near-vertical
       insertion axis carries no azimuth at all, so averaging it in only injects
       noise. (The old code flattened once, AFTER the sum was already spoiled.)
    2. **Detect cancellation.** A male half's local +Z is ``-bar_X`` spun about
       the bar by its ``jr``, i.e. horizontal and perpendicular to the bar -- so
       a bar whose two anchors are ~180 deg apart in ``jr`` sums to (almost)
       zero. The old code returned that residual whenever its length cleared
       1e-9, which is pure floating-point noise. Here the *mean resultant length*
       ``|sum of unit axes| / count`` measures the agreement: 1.0 = all axes
       point the same way, 0.0 = they cancel exactly. Below ``cancel_tol`` the
       answer is reported as ambiguous and no heading is returned, so the caller
       can fall back to something deterministic.

    Args:
        axes (Sequence): the anchor insertion axes (unit world vectors).
        ground_normal (np.ndarray): the ground normal at the base.
        min_horizontal (float | None): drop axes flatter than this once
            projected. ``None`` -> ``config.INSERTION_DIR_MIN_HORIZONTAL``.
        cancel_tol (float | None): mean-resultant-length threshold below which
            the axes are treated as cancelling. ``None`` ->
            ``config.INSERTION_DIR_CANCEL_TOL``.

    Returns:
        dict: ``{"heading": unit np.ndarray | None, "ambiguous": bool,
        "n_vertical": int, "mean_resultant": float, "detail": str}``.
        ``heading`` is ``None`` exactly when ``ambiguous`` is True.
    """
    if min_horizontal is None:
        min_horizontal = config.INSERTION_DIR_MIN_HORIZONTAL
    if cancel_tol is None:
        cancel_tol = config.INSERTION_DIR_CANCEL_TOL

    normal = _unit(ground_normal)
    if normal is None:
        return {"heading": None, "ambiguous": True, "n_vertical": 0,
                "mean_resultant": 0.0, "detail": "ground normal is degenerate"}

    flat = []
    n_vertical = 0
    for axis in axes:
        a = np.asarray(axis, dtype=float)
        horizontal = a - float(np.dot(a, normal)) * normal
        length = float(np.linalg.norm(horizontal))
        if length < float(min_horizontal):
            n_vertical += 1
            continue
        flat.append(horizontal / length)

    if not flat:
        return {"heading": None, "ambiguous": True, "n_vertical": n_vertical,
                "mean_resultant": 0.0,
                "detail": "no anchor axis has a usable horizontal component"}

    resultant = np.sum(flat, axis=0)
    mean_resultant = float(np.linalg.norm(resultant)) / len(flat)
    if mean_resultant < float(cancel_tol):
        return {"heading": None, "ambiguous": True, "n_vertical": n_vertical,
                "mean_resultant": mean_resultant,
                "detail": (f"anchor axes cancel (mean resultant length "
                           f"{mean_resultant:.3f} < {float(cancel_tol):.2f})")}

    return {"heading": resultant / float(np.linalg.norm(resultant)),
            "ambiguous": False, "n_vertical": n_vertical,
            "mean_resultant": mean_resultant,
            "detail": f"mean resultant length {mean_resultant:.3f}"}


def perpendicular_candidates(bar_axis, ground_normal):
    """The two ground-plane headings perpendicular to a bar: ``(+n, -n)``.

    When the anchor axes cancel there is no "behind the bar", but there are still
    exactly two sensible sides to stand on -- either side of the bar, square to
    it. Returns ``None`` when the bar is perpendicular to the ground (standing
    upright), where the two sides are not distinguishable.
    """
    perp = _unit(np.cross(np.asarray(ground_normal, dtype=float),
                          np.asarray(bar_axis, dtype=float)))
    if perp is None:
        return None
    return (perp, -perp)


def pick_best_candidate(candidates, score_fn):
    """Return the highest-scoring candidate heading, or ``None`` if none scores.

    ``score_fn(heading) -> float | None`` returns ``None`` for a side the base
    cannot occupy (off the walkable ground) and otherwise a "how open is it"
    score; the caller supplies it because that measurement needs the Rhino
    document. Ties keep the first candidate, so the result is deterministic.

    Returns:
        tuple: ``(heading | None, [(index, score), ...])`` -- the scores are
        returned too so the caller can report why it chose.
    """
    scored = [(i, score_fn(h)) for i, h in enumerate(candidates)]
    viable = [(i, s) for i, s in scored if s is not None]
    if not viable:
        return None, scored
    best_index = max(viable, key=lambda pair: pair[1])[0]
    return candidates[best_index], scored


def robot_left_dir(heading_mm, ground_normal):
    """Which world direction points out of the mobile base's LEFT side.

    The base frame is +X = heading (forward, facing the bar) and +Z = the ground
    normal (up), so "left" is ``up x forward``.

    Worked example -- bar running along world Y, robot standing to its +X side
    and facing it::

        heading = (-1, 0, 0)        # forward: robot -> bar
        normal  = ( 0, 0, 1)        # up
        left    = normal x heading = (0, -1, 0)     # world -Y

    Args:
        heading_mm (np.ndarray): the resolved base heading (base +X).
        ground_normal (np.ndarray): the ground normal at the base (base +Z).

    Returns:
        np.ndarray | None: the unit left direction, or ``None`` if degenerate
        (heading parallel to the normal).
    """
    return _unit(np.cross(np.asarray(ground_normal, dtype=float),
                          np.asarray(heading_mm, dtype=float)))


def arm_side_for_joint(joint_center_mm, bar_center_mm, heading_mm, ground_normal):
    """Which arm's tool belongs on a joint, given where the robot stands.

    The robot faces the bar along ``heading_mm``, so the bar end lying on the
    robot's LEFT is the end its LEFT arm reaches, and must carry the left tool.
    Move the base to the other side of the bar and the heading negates, so
    :func:`robot_left_dir` negates and BOTH ends swap tools -- which is why the
    side is derived from the heading and never from the bar's own geometry.
    (``rhino_tool_place.enforce_bar_tool_sides`` used to refuse to decide this
    for exactly that reason: the approach was not knowable there.)

    Continuing the :func:`robot_left_dir` example, with the bar's two joints
    500 mm either side of its center along world Y and ``left = (0, -1, 0)``::

        joint at (0, -500, 0):  offset . left = +500 > 0  ->  "left"   (tool L)
        joint at (0, +500, 0):  offset . left = -500 < 0  ->  "right"  (tool R)

    Args:
        joint_center_mm (np.ndarray): the joint block origin (mm) -- where the
            robot actually grabs.
        bar_center_mm (np.ndarray): the midpoint of the bar's two joint centers.
        heading_mm (np.ndarray): the resolved base heading (base +X).
        ground_normal (np.ndarray): the ground normal at the base (base +Z).

    Returns:
        str | None: ``"left"`` / ``"right"``, or ``None`` when the joint sits on
        the base's fore-aft axis, where neither side is meaningful.
    """
    left = robot_left_dir(heading_mm, ground_normal)
    if left is None:
        return None
    offset = np.asarray(joint_center_mm, dtype=float) - np.asarray(bar_center_mm, dtype=float)
    projection = float(np.dot(offset, left))
    if abs(projection) <= _EPS:
        return None
    return "left" if projection > 0.0 else "right"


def build_base_guides(soups: Sequence, joint_a_mm, joint_b_mm, heading_mm,
                      offsets_mm=None) -> Optional[dict]:
    """Build the base-placement guide lines for one bar.

    Args:
        soups (Sequence): the bar's assigned WalkableGround triangle soups, as
            produced by ``core.rhino_walkable_ground._bar_ground_soups``.
        joint_a_mm (np.ndarray): world center (mm) of the bar's first assembly
            joint (the block instance origin -- where the robot grabs).
        joint_b_mm (np.ndarray): world center (mm) of the second assembly joint.
        heading_mm (np.ndarray): the ALREADY-RESOLVED unit base heading from
            ``core.rhino_walkable_ground.resolve_bar_heading`` -- NOT a raw
            insertion axis. The base faces along it and stands against it, so the
            guides are drawn on exactly the side the base will take (including
            after a Flip).
        offsets_mm (Sequence[float] | None): standoff distances to draw, in mm.
            ``None`` -> ``config.BASE_GUIDE_OFFSETS_MM``.

    Returns:
        dict | None: ``{"lines": [(a_mm, b_mm), ...], "offsets": [0.0, ...],
        "extension": [midpoint_mm, ...], "heading": unit_dir}`` -- ``lines[i]``
        is the guide at ``offsets[i]``, with ``offsets[0] == 0.0`` for the plain
        ground projection. ``None`` when the joint centers cannot be projected
        onto the ground (no / empty soups).
    """
    if offsets_mm is None:
        offsets_mm = config.BASE_GUIDE_OFFSETS_MM

    walkable = _walkable_np()
    a_ground, normal_a = walkable.closest_point_on_meshes(soups, joint_a_mm)
    b_ground, normal_b = walkable.closest_point_on_meshes(soups, joint_b_mm)
    if a_ground is None or b_ground is None:
        return None
    a_ground = np.asarray(a_ground, dtype=float)
    b_ground = np.asarray(b_ground, dtype=float)

    # One representative normal for the pair: the two projections may land on
    # different triangles of a stepped ground, so average and re-normalize.
    normal = _unit(np.asarray(normal_a, dtype=float) + np.asarray(normal_b, dtype=float))
    if normal is None:
        normal = np.asarray(normal_a, dtype=float)

    # Re-flatten the heading onto THIS pair's ground plane. `heading_mm` is
    # already a ground-plane direction, but the plane it was resolved against
    # (the standoff point's) need not be the same triangle as here.
    heading = walkable._ground_heading_direction(heading_mm, normal)

    # offset 0 = the plain ground projection of the joint-center line; the rest
    # step AGAINST the heading (the base stands behind, facing the assembly).
    distances = [0.0] + [float(d) for d in offsets_mm]
    lines = []
    midpoints = []
    for distance in distances:
        shift = -heading * distance
        end_a = _snap(soups, a_ground + shift)
        end_b = _snap(soups, b_ground + shift)
        lines.append((end_a, end_b))
        midpoints.append(0.5 * (end_a + end_b))

    return {
        "lines": lines,
        "offsets": distances,
        "extension": midpoints,
        "heading": np.asarray(heading, dtype=float),
    }