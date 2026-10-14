"""Where on the ground a robot base can stand at all, from the arms' reach and the flange targets' full poses.

The math of the base search (`core.base_search`) and of the debug overlay (`core.base_region_viz`), which only draws
it. Rhino-free (numpy, scipy and the core).

A flange target fixes more than the flange: every point on the arm's last joint axis is rigid with the flange, so it
is known in the world too. `wrist_reach` picks the one nearest the joint before (on a UR the two axes cross there,
about one wrist length behind the flange) and finds how far it can get from the arm's first moving joint (the pivot).
A base is possible only if every pivot is within that distance of its wrist point. At one ground point this is an
interval of base headings per target, in closed form; `feasible_yaws` intersects them.

- ! One-sided, like `core.reach_check`: outside the region there is no IK solution; inside it joint limits, the
  arm's inner dead zone and collisions can still fail.
- ? Tighter than the flange distance of `core.reach_check` because it uses the target's orientation: the sphere's
  centre moves back along the tool axis and its radius shrinks.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from scaffolding_assembly_core.urdf import load_urdf

#: Added to the stretched-out distance: the search for it may stop a little short.
MARGIN_M = 0.02
_MOVING = ("revolute", "continuous")
#: (resolved URDF path, tip link) -> (pivot in the root link's frame, wrist point in the tip link's frame, reach), m.
_CACHE: Dict[Tuple[Path, str], Tuple[np.ndarray, np.ndarray, float]] = {}

#: One target of one arm: (pivot in the root link's frame, wrist point in the world, reach), all in metres.
Constraint = Tuple[np.ndarray, np.ndarray, float]


def _origin(joint) -> np.ndarray:
    return np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float)


def _axis(joint) -> np.ndarray:
    axis = np.asarray(joint.axis if joint.axis is not None else (1.0, 0.0, 0.0), dtype=float)
    return axis / np.linalg.norm(axis)


def _turn(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    matrix = np.eye(4)
    matrix[:3, :3] = np.eye(3) + s * k + (1.0 - c) * (k @ k)
    return matrix


def wrist_reach(urdf_file, tip_link: str) -> Tuple[np.ndarray, np.ndarray, float]:
    """An arm's pivot, the point a flange target fixes behind the flange, and how far that point gets from the pivot.

    Args:
        urdf_file: The robot's URDF.
        tip_link: The arm's flange link, e.g. "left_ur_arm_tool0".

    Returns:
        tuple: (pivot (3,) in the root link's frame, wrist point (3,) in the tip link's frame, reach), in metres.
        The reach includes `MARGIN_M`.

    Raises:
        KeyError: If the URDF has no such link.
        ValueError: If fewer than three joints between the root and the tip rotate, or one of them slides.
    """
    key = (Path(urdf_file).resolve(), tip_link)
    if key in _CACHE:
        return _CACHE[key]
    model = load_urdf(key[0])
    by_child = {joint.child: joint for joint in model.robot.joints}
    if tip_link not in model.link_map:
        raise KeyError(f"{key[0].name} has no link {tip_link!r}")
    chain, link = [], tip_link
    while link in by_child:
        chain.append(by_child[link])
        link = by_child[link].parent
    chain.reverse()
    if any(joint.type == "prismatic" for joint in chain):
        raise ValueError(f"{tip_link}: a sliding joint on the way, no fixed reach")
    moving = [i for i, joint in enumerate(chain) if joint.type in _MOVING]
    if len(moving) < 3:
        raise ValueError(f"{tip_link}: fewer than three rotating joints between the root and the tip")
    first, before, last = moving[0], moving[-2], moving[-1]

    to_pivot = np.eye(4)
    for joint in chain[:first + 1]:
        to_pivot = to_pivot @ _origin(joint)

    # The last axis seen from the link the joint before it turns: a line through `start` along `along`.
    to_last = np.eye(4)
    for joint in chain[before + 1:last + 1]:
        to_last = to_last @ _origin(joint)
    last_axis, before_axis = _axis(chain[last]), _axis(chain[before])
    start, along = to_last[:3, 3], to_last[:3, :3] @ last_axis
    # Where on it the wrist point sits: nearest the axis of the joint before (0 when the two are parallel).
    cos = float(along @ before_axis)
    slide = 0.0 if 1.0 - cos * cos < 1e-9 else (cos * float(before_axis @ start) - float(along @ start)) / (1.0 - cos * cos)
    wrist_before = start + slide * along

    # Turning the last joint does not move a point on its axis, so in the last link it is simply `slide` along it.
    to_tip = np.eye(4)
    for joint in chain[last + 1:]:
        to_tip = to_tip @ _origin(joint)
    wrist_tip = (np.linalg.inv(to_tip) @ np.append(slide * last_axis, 1.0))[:3]

    # The first joint turns about the pivot and the last about the wrist point: only the joints between count.
    rest = [(_origin(j), _axis(j) if j.type in _MOVING else None) for j in chain[first + 1:before + 1]]
    count = sum(1 for _, axis in rest if axis is not None)

    def negative_distance(values: Sequence[float]) -> float:
        matrix, values = np.eye(4), iter(values)
        for joint_origin, axis in rest:
            matrix = matrix @ joint_origin
            if axis is not None:
                matrix = matrix @ _turn(axis, next(values))
        return -float(np.linalg.norm(matrix[:3, :3] @ wrist_before + matrix[:3, 3]))

    rng = np.random.default_rng(0)
    starts = sorted(rng.uniform(-np.pi, np.pi, size=(400, count)), key=negative_distance)[:8]
    best = max(-negative_distance([0.0] * count),
               *(-minimize(negative_distance, start, method="Powell").fun for start in starts))
    _CACHE[key] = (to_pivot[:3, 3].copy(), wrist_tip, best + MARGIN_M)
    return _CACHE[key]


def constraints(reach: Dict[str, Tuple[np.ndarray, np.ndarray, float]],
                targets_m: Dict[str, Dict[str, np.ndarray]]) -> list:
    """One `Constraint` per flange target, its wrist point placed in the world.

    Args:
        reach: Per arm side, `wrist_reach` of its flange link.
        targets_m: Per keyframe name, per arm side, the flange target as a 4x4 world transform in metres.
    """
    out = []
    for sides in targets_m.values():
        for side, target in sides.items():
            if side not in reach:
                continue
            pivot, wrist_tip, limit = reach[side]
            target = np.asarray(target, dtype=float)
            out.append((pivot, target[:3, :3] @ wrist_tip + target[:3, 3], limit))
    return out


def window(limits: Sequence[Constraint]) -> Tuple[np.ndarray, np.ndarray]:
    """The world XY box no base origin can lie outside: (lower (2,), upper (2,)) in mm."""
    centres = np.array([wrist[:2] for _, wrist, _ in limits])
    pad = max(limit + float(np.hypot(pivot[0], pivot[1])) for pivot, _, limit in limits)
    return (centres.min(axis=0) - pad) * 1000.0, (centres.max(axis=0) + pad) * 1000.0


def ground_grid(soups, lower_mm, upper_mm, step_mm: float) -> Tuple[np.ndarray, np.ndarray]:
    """A square grid of points on the ground inside a world XY box, with the ground's upward normal at each.

    Args:
        soups: The ground as triangle soups, each (vertices (V, 3) in mm, triangles (T, 3) of vertex rows).
        lower_mm, upper_mm: The box's XY corners in mm.
        step_mm: The grid spacing.

    Returns:
        tuple: (points (N, 3) in mm, unit normals (N, 3)). Where grounds overlap in plan, the highest one counts.
    """
    xs = np.arange(lower_mm[0], upper_mm[0] + step_mm, step_mm)
    ys = np.arange(lower_mm[1], upper_mm[1] + step_mm, step_mm)
    px, py = (grid.ravel() for grid in np.meshgrid(xs, ys))
    height = np.full(px.shape, -np.inf)
    normals = np.zeros((px.size, 3))
    for vertices, triangles in soups:
        vertices = np.asarray(vertices, dtype=float)
        for a, b, c in vertices[np.asarray(triangles, dtype=int)]:
            if (max(a[0], b[0], c[0]) < lower_mm[0] or min(a[0], b[0], c[0]) > upper_mm[0]
                    or max(a[1], b[1], c[1]) < lower_mm[1] or min(a[1], b[1], c[1]) > upper_mm[1]):
                continue
            area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
            if abs(area) < 1e-9:  # seen edge-on from above: nothing stands on it
                continue
            u = ((px - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (py - a[1])) / area
            v = ((b[0] - a[0]) * (py - a[1]) - (px - a[0]) * (b[1] - a[1])) / area
            z = a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2])
            hit = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1.0 + 1e-9) & (z > height)
            if not hit.any():
                continue
            normal = np.cross(b - a, c - a)
            normal = normal / np.linalg.norm(normal) * (1.0 if normal[2] >= 0.0 else -1.0)
            height[hit], normals[hit] = z[hit], normal
    on = np.isfinite(height)
    return np.column_stack([px[on], py[on], height[on]]), normals[on]


def heading_axes(normals) -> Tuple[np.ndarray, np.ndarray]:
    """Heading 0 and heading 90 degrees on the ground, per unit normal (N, 3): two (N, 3) arrays of unit vectors."""
    up = np.asarray(normals, dtype=float)
    flat = np.array([1.0, 0.0, 0.0]) - up[:, :1] * up
    steep = np.linalg.norm(flat, axis=1) < 1e-6
    flat[steep] = np.array([0.0, 1.0, 0.0]) - up[steep, 1:2] * up[steep]
    ahead = flat / np.linalg.norm(flat, axis=1, keepdims=True)
    return ahead, np.cross(up, ahead)


def base_frames(points_mm, normals, yaws) -> np.ndarray:
    """The base frames at ground points with one heading each: (N, 4, 4), translation in mm."""
    up = np.asarray(normals, dtype=float)
    yaws = np.asarray(yaws, dtype=float)[:, None]
    ahead, left = heading_axes(up)
    frames = np.tile(np.eye(4), (up.shape[0], 1, 1))
    frames[:, :3, 0] = np.cos(yaws) * ahead + np.sin(yaws) * left
    frames[:, :3, 1] = np.cos(yaws) * left - np.sin(yaws) * ahead
    frames[:, :3, 2] = up
    frames[:, :3, 3] = np.asarray(points_mm, dtype=float)
    return frames


def widest_run(ok) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The longest unbroken run of possible headings per ground point; the headings wrap around.

    Several targets can leave two separate runs (each rules out one arc of headings, and two such arcs need not
    overlap), so the middle of all possible headings may itself be impossible: the middle of the longest run is not.

    Args:
        ok: (N, K) of bool from `feasible_yaws`, its K headings evenly spaced over the full turn.

    Returns:
        tuple: (length (N,) in headings, middle (N,) as a possibly fractional column of `ok`, runs (N,) the number
        of separate runs). A point where every heading works has length K, one run, and a middle that means nothing.
    """
    ok = np.asarray(ok, dtype=bool)
    count = ok.shape[1]
    twice = np.concatenate([ok, ok], axis=1)
    column = np.arange(2 * count)
    last_gap = np.maximum.accumulate(np.where(twice, -1, column), axis=1)
    ending_here = np.minimum(column - last_gap, count)
    end = ending_here.argmax(axis=1)
    length = ending_here[np.arange(ok.shape[0]), end]
    middle = (end - (length - 1) / 2.0) % count
    runs = (ok & ~np.roll(ok, 1, axis=1)).sum(axis=1)
    return length, middle, np.where(length == count, 1, runs)


def extension_cost(points_mm, normals, limits: Sequence[Constraint], yaws, prefer: float) -> np.ndarray:
    """How far the arms are from a comfortable, partly extended pose, per base origin and heading.

    An arm's extension is its pivot-to-wrist-point distance over its reach. Per target the cost is 0 at the
    preferred extension and rises with the square of the deviation to 1 both fully stretched and folded onto the
    pivot; the result is the mean over the targets. This is the cost a base optimiser would descend on.

    - ! Not masked: headings out of reach get a cost too (above 0 per stretched arm). Mask with `feasible_yaws`.

    Args:
        points_mm, normals, limits, yaws: As for `feasible_yaws`.
        prefer: The preferred extension, between 0 and 1.

    Returns:
        np.ndarray: (N, K) of float.
    """
    origins = np.asarray(points_mm, dtype=float) / 1000.0
    up = np.asarray(normals, dtype=float)
    yaws = np.asarray(yaws, dtype=float)
    ahead, left = heading_axes(up)

    total = np.zeros((origins.shape[0], yaws.size))
    for pivot, wrist, limit in limits:
        # Same geometry as `feasible_yaws`, solved for the distance itself.
        offset = np.asarray(wrist, dtype=float) - origins - pivot[2] * up
        x, y, above = (offset * ahead).sum(axis=1), (offset * left).sum(axis=1), (offset * up).sum(axis=1)
        far, radius = np.hypot(x, y), float(np.hypot(pivot[0], pivot[1]))
        angle = yaws[None, :] + np.arctan2(pivot[1], pivot[0]) - np.arctan2(y, x)[:, None]
        squared = (above * above + far * far + radius * radius)[:, None] - (2.0 * radius * far)[:, None] * np.cos(angle)
        extension = np.sqrt(np.maximum(squared, 0.0)) / limit
        off = np.where(extension > prefer, (extension - prefer) / (1.0 - prefer), (prefer - extension) / prefer)
        total += off * off
    return total / max(len(limits), 1)


def feasible_yaws(points_mm, normals, limits: Sequence[Constraint], yaws) -> np.ndarray:
    """Which base headings keep every wrist point within reach, for base origins on the ground.

    The base stands with its Z along the ground normal; heading 0 points along world X laid flat on the ground
    (world Y where X is the normal), and headings turn about the normal.

    Args:
        points_mm: Base origins (N, 3) in mm.
        normals: The ground's unit normal at each (N, 3).
        limits: The `Constraint`s, from `constraints`.
        yaws: The headings to test (K,), in radians.

    Returns:
        np.ndarray: (N, K) of bool.
    """
    origins = np.asarray(points_mm, dtype=float) / 1000.0
    up = np.asarray(normals, dtype=float)
    yaws = np.asarray(yaws, dtype=float)
    ahead, left = heading_axes(up)

    ok = np.ones((origins.shape[0], yaws.size), dtype=bool)
    for pivot, wrist, limit in limits:
        # The pivot circles the base's Z at radius `radius`; the wrist point sits `far` out and `above` its plane.
        offset = np.asarray(wrist, dtype=float) - origins - pivot[2] * up
        x, y, above = (offset * ahead).sum(axis=1), (offset * left).sum(axis=1), (offset * up).sum(axis=1)
        far, radius = np.hypot(x, y), float(np.hypot(pivot[0], pivot[1]))
        slack = limit * limit - above * above - far * far - radius * radius
        product = 2.0 * radius * far
        with np.errstate(divide="ignore", invalid="ignore"):
            # distance^2 = above^2 + far^2 + radius^2 - product * cos(angle between pivot and wrist point)
            least_cos = np.where(product > 1e-12, -slack / product, np.where(slack >= 0.0, -np.inf, np.inf))
        angle = yaws[None, :] + np.arctan2(pivot[1], pivot[0]) - np.arctan2(y, x)[:, None]
        ok &= np.cos(angle) >= least_cos[:, None]
    return ok
