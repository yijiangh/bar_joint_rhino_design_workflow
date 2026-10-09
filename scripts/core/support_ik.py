"""IK for a support robot's single arm, on any compas_fab planner: in Rhino, a mirror's lent planner.

The planner's cell and the template state come from the core (`design_planning.hold_solve_state`); nothing here
builds a cell. The arm group is `config.SUPPORT_GROUP`. Frames come in as 4x4 world matrices in mm.

- Needs compas_fab and husky_assembly_tamp (frame conversion, collision-pair naming).
"""

from __future__ import annotations

import numpy as np

from core import config


def _defaults(max_results, max_descend_iterations, tolerance_position, tolerance_orientation):
    """The IK options, `config` values where not given."""
    return (config.IK_MAX_RESULTS if max_results is None else max_results,
            config.IK_MAX_DESCEND_ITERATIONS if max_descend_iterations is None else max_descend_iterations,
            config.IK_TOLERANCE_POSITION if tolerance_position is None else tolerance_position,
            config.IK_TOLERANCE_ORIENTATION if tolerance_orientation is None else tolerance_orientation)


def apply_base_frame_mm(state, base_frame_world_mm) -> None:
    """Set a state's robot base frame from a 4x4 world matrix in mm, in place."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import apply_base_frame_mm as _apply
    _apply(state, np.asarray(base_frame_world_mm, dtype=float))


def _target(tool0_world_mm, tolerance_position, tolerance_orientation):
    """A compas_fab frame target for the flange, metres."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import import_compas_stack, mm_matrix_to_m_frame
    deps = import_compas_stack()
    return deps["FrameTarget"](mm_matrix_to_m_frame(deps["Frame"], np.asarray(tool0_world_mm, dtype=float)),
                               deps["TargetMode"].ROBOT, tolerance_position=tolerance_position,
                               tolerance_orientation=tolerance_orientation)


def solve_support_ik(planner, template_state, base_frame_world_mm, tool0_world_mm, *, check_collision: bool = True,
                     max_results: int = None, max_descend_iterations: int = None, tolerance_position: float = None,
                     tolerance_orientation: float = None, verbose_pairs: bool = False):
    """Solve the support arm's IK at one base for one flange target.

    Args:
        planner: The support robot's planner, its cell and state set (a lent mirror).
        template_state: The scene state to fork; not changed.
        base_frame_world_mm: 4x4 robot base pose, world mm.
        tool0_world_mm: 4x4 flange target, world mm.
        check_collision: Check collisions during the descent.
        max_results, max_descend_iterations, tolerance_position, tolerance_orientation: IK options; `config` defaults.
        verbose_pairs: Print the collision-pair summary of the result.

    Returns:
        RobotCellState | None: The solved state, or None when IK failed.
    """
    max_results, iterations, tol_p, tol_o = _defaults(max_results, max_descend_iterations, tolerance_position,
                                                      tolerance_orientation)
    state = template_state.copy()
    apply_base_frame_mm(state, base_frame_world_mm)
    options = {"max_results": max_results, "check_collision": check_collision,
               "max_descend_iterations": iterations, "verbose": False}
    try:
        cfg = planner.inverse_kinematics(_target(tool0_world_mm, tol_p, tol_o), state, group=config.SUPPORT_GROUP,
                                         options=options)
    except Exception as exc:  # noqa: BLE001 -- compas_fab raises several kinds; all mean no solution
        print(f"IK failed for group '{config.SUPPORT_GROUP}': {exc}")
        return None
    if cfg is None:
        print(f"IK returned no solution for group '{config.SUPPORT_GROUP}'.")
        return None
    state.robot_configuration.merge(cfg)
    if verbose_pairs:
        from husky_assembly_tamp.keyframe.dual_arm_ik import summarize_check_collision
        print(summarize_check_collision(planner, state))
    return state


def extract_group_config(state, group: str, robot_cell) -> dict:
    """`{"joint_names": [...], "joint_values": [...]}` of one SRDF group in a state."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import extract_group_config as _extract
    return _extract(state, group, robot_cell)


def enumerate_support_ik_candidates(planner, template_state, base_frame_world_mm, tool0_world_mm, *,
                                    max_results: int = None, max_descend_iterations: int = None,
                                    tolerance_position: float = None, tolerance_orientation: float = None) -> dict:
    """Every support-arm IK solution that reaches the target, each with its collisions: why a solve failed.

    The same gradient solver as `solve_support_ik`, with collision checks off; each solution is then checked with a
    full report. No candidates: the target is out of reach from this base. Candidates that all collide: reachable but
    blocked, and the offenders say by what (named by the cell's keys: design ids on a mirror).

    Returns:
        dict: `{"n_reachable": int, "candidates": [{"index", "state", "in_collision", "summary", "offenders",
        "num_offending_pairs"}]}`, collision-free first, then fewest offending pairs.
    """
    from compas_fab.backends import CollisionCheckError
    from husky_assembly_tamp.keyframe.dual_arm_ik import _resolve_collision_pairs

    max_results, iterations, tol_p, tol_o = _defaults(max_results, max_descend_iterations, tolerance_position,
                                                      tolerance_orientation)
    state = template_state.copy()
    apply_base_frame_mm(state, base_frame_world_mm)
    # ! Collision off on purpose: the colliding poses are what we want to show.
    options = {"max_results": max_results, "check_collision": False, "max_descend_iterations": iterations,
               "verbose": False}
    configurations = []
    try:
        for cfg in planner.iter_inverse_kinematics(_target(tool0_world_mm, tol_p, tol_o), state,
                                                   group=config.SUPPORT_GROUP, options=options):
            configurations.append(cfg)
    except Exception as exc:  # noqa: BLE001 -- nothing reachable, or a backend failure: the empty list says so
        print(f"core.support_ik.enumerate_support_ik_candidates: {type(exc).__name__}: {exc}")

    cell = planner.client.robot_cell
    candidates = []
    for index, cfg in enumerate(configurations):
        trial = state.copy()
        trial.robot_configuration.merge(cfg)
        in_collision, summary, offenders, num_pairs = False, "clear (no collision)", [], 0
        try:
            planner.check_collision(trial, options={"full_report": True, "verbose": False})
        except CollisionCheckError as exc:
            in_collision = True
            pairs = list(getattr(exc, "collision_pairs", []) or [])
            summary, offenders, num_pairs = _resolve_collision_pairs(pairs, cell)
            if not pairs:
                summary = str(exc).splitlines()[0] if str(exc) else "collision"
        except Exception as exc:  # noqa: BLE001 -- a non-collision failure (state mismatch): still show it
            in_collision = True
            summary = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
        candidates.append({"index": index, "state": trial, "in_collision": in_collision, "summary": summary,
                           "offenders": offenders, "num_offending_pairs": num_pairs})
    candidates.sort(key=lambda c: (c["in_collision"], c["num_offending_pairs"]))
    return {"n_reachable": len(configurations), "candidates": candidates}
