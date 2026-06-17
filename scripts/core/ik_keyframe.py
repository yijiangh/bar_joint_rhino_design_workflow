"""Shared IK-keyframe solve logic (Rhino-free).

This is the one place that implements "load a movement's ``start_state``, solve
IK for its ``target_ee_frames``", chained across M1 -> M2 -> M3. Both the Rhino
front-end (``rs_ik_keyframe``) and the headless test
(``tests/headless_ik_keyframe.py``) call ``solve_keyframe_chain`` so the solve is
defined exactly once. It lives in its own module (not ``rs_ik_keyframe.py``)
because that script imports Rhino at the top, which a plain-python test can't.

The movements come from ``core.bar_action.build_assembly_movements`` (Rhino) or
from a loaded ``BarAssemblyAction`` JSON (headless). Either way each movement
already carries the per-keyframe collision context (attachments + allowed-touch
policy) on its ``start_state`` and the EE goal on its ``target_ee_frames`` -- the
only thing IK supplies is ``robot_configuration``.
"""

from __future__ import annotations

import numpy as np

from core import robot_cell


def frame_to_mm4(frame) -> np.ndarray:
    """Convert a compas ``Frame`` (meters) to a 4x4 numpy transform in mm.

    The inverse of ``core.bar_action._mm4_to_frame``. ``solve_dual_arm_ik`` wants
    its tool0 targets in mm (repo convention), but ``Movement.target_ee_frames``
    stores compas ``Frame``s in meters, so every target is run through here.

    Args:
        frame (compas.geometry.Frame): pose in meters.

    Returns:
        np.ndarray: 4x4 homogeneous transform with mm translation; columns 0-2
        are the frame's x/y/z axes.
    """
    matrix = np.eye(4, dtype=float)
    matrix[:3, 0] = np.asarray(frame.xaxis, dtype=float)
    matrix[:3, 1] = np.asarray(frame.yaxis, dtype=float)
    matrix[:3, 2] = np.asarray(frame.zaxis, dtype=float)
    matrix[:3, 3] = np.asarray(frame.point, dtype=float) * 1000.0  # m -> mm
    return matrix


def solve_keyframe_chain(
    planner,
    ordered_movements,
    base_frame_mm,
    *,
    check_collision: bool = True,
    verbose_pairs: bool = False,
):
    """Solve IK for a sequence of movements, chaining configs forward.

    For each ``(role, movement)`` in order:

    1. Copy the movement's ``start_state`` (which carries the per-keyframe
       attachments + allowed-touch policy).
    2. Seed its ``robot_configuration`` with the *previous* movement's solved
       config -- this realizes the chain (M2 starts where M1 ended, M3 where M2
       ended). The first movement keeps its own start config (HOME for M1).
    3. Solve dual-arm IK for the movement's ``target_ee_frames`` at
       ``base_frame_mm``.

    All movements share ``base_frame_mm`` (one robot base for the whole bar);
    ``solve_dual_arm_ik`` overrides each state's stored base frame with it.

    Args:
        planner (PyBulletPlanner): active planner with the dual-arm cell loaded.
        ordered_movements (list): ``[(role, movement), ...]`` in solve order,
            e.g. ``[("M1", m1), ("M2", m2), ("M3", m3)]``. Each ``movement`` needs
            a ``start_state`` and a ``target_ee_frames`` dict with ``"left"`` /
            ``"right"`` compas ``Frame``s.
        base_frame_mm (np.ndarray): 4x4 mm robot base frame shared by all solves.
        check_collision (bool): pass-through to ``solve_dual_arm_ik``.
        verbose_pairs (bool): pass-through; prints the collision-pair summary.

    Returns:
        dict | None: ``{role: solved_state}`` once every movement solves, or
        ``None`` at the first failure (so a base-sampling caller can try another
        base).
    """
    results = {}
    prev_config = None
    for role, movement in ordered_movements:
        state = movement.start_state.copy()
        # Chain: each movement begins at the previous movement's solved config.
        # That warm-start is a good seed, so those solves descend deterministically
        # (max_restart_iter=1). The FIRST movement has no warm-start, so it uses the
        # cold default (random restarts) to find the collision-free basin.
        if prev_config is not None:
            state.robot_configuration = prev_config.copy()
            max_restart_iter = 1
        else:
            max_restart_iter = None  # -> config.IK_MAX_RESTART_ITER (cold restarts)

        targets = movement.target_ee_frames or {}
        left_frame = targets.get("left")
        right_frame = targets.get("right")
        if left_frame is None or right_frame is None:
            print(
                f"core.ik_keyframe.solve_keyframe_chain: {role} has no "
                "left/right target_ee_frames; aborting chain."
            )
            return None

        print(f"core.ik_keyframe.solve_keyframe_chain: solving {role} ...")
        solved = robot_cell.solve_dual_arm_ik(
            planner,
            state,
            base_frame_mm,
            frame_to_mm4(left_frame),
            frame_to_mm4(right_frame),
            check_collision=check_collision,
            max_restart_iter=max_restart_iter,
            verbose_pairs=verbose_pairs,
        )
        if solved is None:
            print(f"core.ik_keyframe.solve_keyframe_chain: {role} IK failed.")
            return None

        results[role] = solved
        prev_config = solved.robot_configuration
    return results
