"""Reject a robot base whose arms cannot reach their flange targets by distance alone, before any IK runs.

An arm's flange is never farther from the arm's first moving joint than the arm stretched out. `arm_reach` finds
that joint's place on the robot (the pivot) and the stretched-out distance from the URDF; `out_of_reach` compares it
with the targets at one base frame. Rhino-free (numpy, scipy and the core).

- ! One-sided: a target inside the distance may still have no IK solution (orientation, joint limits, collisions).
  A target outside it has none, so the base can be skipped without asking the solver.
- ? The distance is found numerically (joint limits ignored, so it can only be too generous) and `MARGIN_M` is added.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize

from core import core_lib  # noqa: F401  (puts scaffolding_assembly_core on sys.path)
from scaffolding_assembly_core.urdf import load_urdf

#: Added to the stretched-out distance: the search for it may stop a little short.
MARGIN_M = 0.02
_MOVING = ("revolute", "continuous")
#: (resolved URDF path, tip link) -> (pivot xyz in the root link's frame, reach), both in metres.
_CACHE: Dict[Tuple[Path, str], Tuple[np.ndarray, float]] = {}


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def arm_reach(urdf_file, tip_link: str) -> Tuple[np.ndarray, float]:
    """Where an arm's first moving joint sits on the robot, and how far from it the tip link's frame can get.

    Args:
        urdf_file: The robot's URDF.
        tip_link: The arm's flange link, e.g. "left_ur_arm_tool0".

    Returns:
        tuple: (pivot (3,) in the root link's frame, reach), in metres. The reach includes `MARGIN_M`.

    Raises:
        KeyError: If the URDF has no such link.
        ValueError: If no joint between the root and the tip rotates, or one of them slides.
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
    first = next((i for i, joint in enumerate(chain) if joint.type in _MOVING), None)
    if first is None:
        raise ValueError(f"{tip_link}: no rotating joint between the root and the tip")

    def origin(joint):
        return np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float)

    to_pivot = np.eye(4)
    for joint in chain[:first + 1]:
        to_pivot = to_pivot @ origin(joint)
    # The first joint turns about the pivot, so it does not change the distance: only the joints after it count.
    rest = [(origin(j), None if j.type not in _MOVING else np.asarray(j.axis if j.axis is not None else (1, 0, 0),
                                                                     dtype=float)) for j in chain[first + 1:]]
    moving = sum(1 for _, axis in rest if axis is not None)

    def negative_distance(values: Sequence[float]) -> float:
        matrix, values = np.eye(4), iter(values)
        for joint_origin, axis in rest:
            matrix = matrix @ joint_origin
            if axis is not None:
                turn = np.eye(4)
                turn[:3, :3] = _rotation(axis / np.linalg.norm(axis), next(values))
                matrix = matrix @ turn
        return -float(np.linalg.norm(matrix[:3, 3]))

    best = -negative_distance([0.0] * moving)
    if moving:
        rng = np.random.default_rng(0)
        starts = sorted(rng.uniform(-np.pi, np.pi, size=(400, moving)), key=negative_distance)[:8]
        best = max(best, *(-minimize(negative_distance, start, method="Powell").fun for start in starts))
    _CACHE[key] = (to_pivot[:3, 3].copy(), best + MARGIN_M)
    return _CACHE[key]


def out_of_reach(base_frame_mm: np.ndarray, reach: Mapping[str, Tuple[np.ndarray, float]],
                 targets_m: Mapping[str, Mapping[str, Sequence[float]]]) -> List[str]:
    """The flange targets an arm cannot reach from one base frame, by distance alone.

    Args:
        base_frame_mm: 4x4 world frame of the robot's root link, translation in mm.
        reach: Per arm side, `arm_reach` of its flange link.
        targets_m: Per keyframe name, per arm side, the flange target's world position in metres.

    Returns:
        list[str]: One line per unreachable target ("M1 left arm: target 1.42 m from the shoulder, reach 1.00 m");
        empty when every target is within reach.
    """
    base = np.asarray(base_frame_mm, dtype=float)
    out = []
    for name, sides in targets_m.items():
        for side, target in sides.items():
            if side not in reach:
                continue
            pivot, limit = reach[side]
            pivot_world = base[:3, :3] @ pivot + base[:3, 3] / 1000.0
            distance = float(np.linalg.norm(np.asarray(target, dtype=float) - pivot_world))
            if distance > limit:
                out.append(f"{name} {side} arm: target {distance:.2f} m from the shoulder, reach {limit:.2f} m")
    return out
