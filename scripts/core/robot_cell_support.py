"""Single-arm Husky (support) robot cells, one per support robot.

Each support robot (Alice, Belle) has its OWN calibrated URDF/SRDF and its
OWN persistent PyBullet session (see ``core.robot_cell``). A support cell
contains: that robot's model + the SupportGripper tool on its arm, plus the
OTHER robots (Cindy and the other support robot) frozen as articulated
ToolModel obstacles (see ``core.robot_obstacles``).

No cell swapping: each session's planner permanently owns its one cell. The
cell is built + pushed lazily on the first support-flow use via
``ensure_support_cell_pushed``.
"""

from __future__ import annotations

import os

import numpy as np

from core import config
from core import robot_obstacles
from core.robot_cell import (  # noqa: F401  re-exported for callers
    import_compas_stack,
    stop_pb_client,
    get_session,
    is_pb_running,
    _STICKY,
    _STICKY_PB_SESSIONS,
    _mm_matrix_to_m_frame,
    _frame_to_m_matrix,
)


# One cached cell per support robot name.
_STICKY_SUPPORT_CELL_PREFIX = "bar_joint:support_cell:"


# ---------------------------------------------------------------------------
# Support cell loading (per robot)
# ---------------------------------------------------------------------------


def _other_robot_names(robot_name: str) -> list:
    """Every roster robot EXCEPT the given one (they become frozen obstacles).

    Args:
        robot_name (str): the support robot that owns the cell.

    Returns:
        list: e.g. ["Cindy", "Belle"] for Alice's cell.
    """
    others = [config.ASSEMBLY_ROBOT_NAME]
    others += [n for n in config.SUPPORT_ROBOT_NAMES if n != robot_name]
    return others


def _attach_support_tool_models(cell, robot_name: str):
    """Register the SupportGripper + the frozen-robot obstacles into a cell.

    Args:
        cell (RobotCell): the support cell (mutated in place).
        robot_name (str): the support robot that owns the cell.

    Raises:
        RuntimeError: if the hand-modeled Robotiq collision OBJ is missing —
            the gripper's collisions are load-bearing for every support IK
            solve, so a missing mesh is an error, not a warning.
    """
    deps = import_compas_stack()
    Frame = deps["Frame"]
    Mesh = deps["Mesh"]
    ToolModel = deps["ToolModel"]

    sg_name = config.SUPPORT_TOOL_NAME
    mesh_path = config.ROBOTIQ_GRIPPER_TOOL_MESH
    existing = cell.tool_models.get(sg_name)
    already_loaded = existing is not None and getattr(existing, "_loaded_from", None) == mesh_path
    if not already_loaded:
        if not os.path.isfile(mesh_path):
            raise RuntimeError(
                f"Robotiq gripper collision mesh missing: {mesh_path!r}. The "
                "SupportGripper tool cannot be attached without it — export the "
                "hand-modeled mesh to that path first."
            )
        mesh = Mesh.from_obj(mesh_path)
        tool = ToolModel(mesh, Frame.worldXY(), name=sg_name)
        try:
            tool._loaded_from = mesh_path
        except AttributeError:
            pass
        cell.tool_models[sg_name] = tool

    # Cindy + the other support robot, frozen as articulated obstacles.
    robot_obstacles.attach_obstacle_robot_tools(cell, _other_robot_names(robot_name))


def get_or_load_support_cell(robot_name: str):
    """One support robot's cached ``RobotCell``, loading URDF/SRDF on first call.

    Args:
        robot_name (str): "Alice" or "Belle".

    Returns:
        RobotCell: that robot's cell (model + gripper + frozen obstacles).

    Raises:
        RuntimeError: on an unknown support robot name.
    """
    if robot_name not in config.SUPPORT_ROBOTS:
        raise RuntimeError(
            f"Unknown support robot {robot_name!r}. "
            f"Known support robots: {sorted(config.SUPPORT_ROBOTS)}."
        )

    key = _STICKY_SUPPORT_CELL_PREFIX + robot_name
    cached = _STICKY.get(key)
    if cached is not None:
        _attach_support_tool_models(cached, robot_name)
        return cached

    deps = import_compas_stack()
    deps["compas"].PRECISION = "12f"

    entry = config.SUPPORT_ROBOTS[robot_name]
    # The RobotModel is loaded (and cached) by robot_obstacles so the SAME
    # load serves both this actuated cell and any frozen-obstacle use.
    robot_model = robot_obstacles.get_or_load_robot_model(robot_name)

    LocalPackageMeshLoader = deps["LocalPackageMeshLoader"]
    main_loader = LocalPackageMeshLoader(config.HUSKY_PKG_PATH, config.SUPPORT_URDF_PKG_NAME)
    srdf_path = main_loader.build_path(
        os.path.dirname(entry["srdf_rel_path"]),
        os.path.basename(entry["srdf_rel_path"]),
    )
    semantics = deps["RobotSemantics"].from_srdf_file(srdf_path, robot_model)

    cell = deps["RobotCell"](robot_model, semantics)
    _attach_support_tool_models(cell, robot_name)

    _STICKY[key] = cell
    return cell


def ensure_support_cell_pushed(robot_name: str):
    """Get a support robot's session, pushing its cell into the planner once.

    The first call for a robot pays the one-time cell build (URDF + gripper +
    frozen obstacles) and the ``planner.set_robot_cell`` upload; later calls
    just return the cached session.

    Args:
        robot_name (str): "Alice" or "Belle".

    Returns:
        tuple: ``(client, planner, cell)`` for that robot.
    """
    from core.robot_cell import get_session as _get_session

    client, planner = _get_session(robot_name)
    cell = get_or_load_support_cell(robot_name)
    session = _STICKY[_STICKY_PB_SESSIONS][robot_name]
    if not session.get("cell_loaded"):
        print(
            f"core.robot_cell_support: first use of {robot_name} — building + "
            "pushing her cell into PyBullet (one-time load)..."
        )
        planner.set_robot_cell(cell)
        session["cell_loaded"] = True
    return client, planner, cell


# ---------------------------------------------------------------------------
# Cell states
# ---------------------------------------------------------------------------


def _configure_support_tool_states(state, robot_name: str):
    """Attach the gripper to the arm group; park every frozen robot obstacle.

    Args:
        state (RobotCellState): the state to prepare (mutated in place).
        robot_name (str): the support robot that owns the cell.
    """
    sg_name = config.SUPPORT_TOOL_NAME
    if sg_name in state.tool_states:
        ts = state.tool_states[sg_name]
        ts.attached_to_group = config.SUPPORT_GROUP
        ts.touch_links = list(config.SUPPORT_TOOL_TOUCH_LINKS)
    # ! Frozen robots default to PARKED (far away, collisions on). Scenes that
    # ! want one of them present call configure_robot_obstacle on top of this.
    for other in _other_robot_names(robot_name):
        robot_obstacles.park_robot_obstacle(state, other)


def default_support_cell_state(robot_name: str):
    """A fresh default state for one support robot's cell.

    Args:
        robot_name (str): "Alice" or "Belle".

    Returns:
        RobotCellState: gripper attached, all other robots parked far away.
    """
    cell = get_or_load_support_cell(robot_name)
    state = cell.default_cell_state()
    _configure_support_tool_states(state, robot_name)
    return state


# ---------------------------------------------------------------------------
# Cell state push
# ---------------------------------------------------------------------------


def set_cell_state(planner, robot_cell_state):
    """Push a support state into PyBullet on the GIVEN planner.

    The caller is responsible for passing the right support robot's planner
    (``ensure_support_cell_pushed``); there is no cell swapping anymore. The
    robot base pose rides in the state itself (compas_fab pushes it via
    ``client._set_base_frame``).
    """
    planner.set_robot_cell_state(robot_cell_state)


# ---------------------------------------------------------------------------
# IK
# ---------------------------------------------------------------------------


def _apply_base_frame_mm(state, base_frame_world_mm: np.ndarray):
    """Set a state's robot base frame from a 4x4 world matrix in mm.

    Args:
        state (RobotCellState): the state to modify (mutated in place).
        base_frame_world_mm (np.ndarray): 4x4 world base pose, mm.
    """
    deps = import_compas_stack()
    state.robot_base_frame = _mm_matrix_to_m_frame(deps["Frame"], base_frame_world_mm)


def solve_support_ik(
    planner,
    template_state,
    base_frame_world_mm: np.ndarray,
    tool0_world_mm: np.ndarray,
    *,
    check_collision: bool = True,
    max_results: int = None,
    max_descend_iterations: int = None,
    tolerance_position: float = None,
    tolerance_orientation: float = None,
    verbose_pairs: bool = False,
):
    """Solve IK for the single support-arm group. Returns mutated state or None.

    The caller must pass the RIGHT support robot's planner (whose cell was
    pushed by ``ensure_support_cell_pushed``) and a ``template_state`` from
    that same robot's ``default_support_cell_state`` — the solve runs against
    whatever cell the planner owns.

    Args:
        planner (PyBulletPlanner): the support robot's own planner.
        template_state (RobotCellState): state to fork (not mutated).
        base_frame_world_mm (np.ndarray): 4x4 robot base pose, world mm.
        tool0_world_mm (np.ndarray): 4x4 arm flange target, world mm.
        check_collision (bool): include collision checking in the IK descent.
        max_results (int): candidate descents per call (default config).
        max_descend_iterations (int): per-descent iteration cap (default config).
        tolerance_position (float): position tolerance (default config).
        tolerance_orientation (float): orientation tolerance (default config).
        verbose_pairs (bool): print the collision-pair summary of the result.

    Returns:
        RobotCellState or None: the solved state, or None when IK failed.
    """
    deps = import_compas_stack()
    Frame = deps["Frame"]
    FrameTarget = deps["FrameTarget"]
    TargetMode = deps["TargetMode"]

    max_results = max_results if max_results is not None else config.IK_MAX_RESULTS
    max_descend_iterations = (
        max_descend_iterations
        if max_descend_iterations is not None
        else config.IK_MAX_DESCEND_ITERATIONS
    )
    tolerance_position = (
        tolerance_position if tolerance_position is not None else config.IK_TOLERANCE_POSITION
    )
    tolerance_orientation = (
        tolerance_orientation
        if tolerance_orientation is not None
        else config.IK_TOLERANCE_ORIENTATION
    )

    state = template_state.copy()
    _apply_base_frame_mm(state, base_frame_world_mm)

    target = FrameTarget(
        _mm_matrix_to_m_frame(Frame, tool0_world_mm),
        TargetMode.ROBOT,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
    )
    options = {
        "max_results": max_results,
        "check_collision": check_collision,
        "max_descend_iterations": max_descend_iterations,
        "verbose": False,
    }

    try:
        cfg = planner.inverse_kinematics(target, state, group=config.SUPPORT_GROUP, options=options)
    except Exception as exc:
        print(f"IK failed for group '{config.SUPPORT_GROUP}': {exc}")
        return None
    if cfg is None:
        print(f"IK returned no solution for group '{config.SUPPORT_GROUP}'.")
        return None
    state.robot_configuration.merge(cfg)

    if verbose_pairs:
        # The pair-count summary moved to the tamp solvers with the dual-arm IK.
        from husky_assembly_tamp.keyframe.dual_arm_ik import summarize_check_collision
        print(summarize_check_collision(planner, state))

    return state


def extract_group_config(state, group: str, robot_cell) -> dict:
    """Return ``{'joint_names': [...], 'joint_values': [...]}`` for a group."""
    from husky_assembly_tamp.keyframe.dual_arm_ik import extract_group_config as _extract
    return _extract(state, group, robot_cell)


def enumerate_support_ik_candidates(
    planner,
    template_state,
    base_frame_world_mm: np.ndarray,
    tool0_world_mm: np.ndarray,
    *,
    max_results: int = None,
    max_descend_iterations: int = None,
    tolerance_position: float = None,
    tolerance_orientation: float = None,
):
    """Enumerate every REACHABLE support-arm IK solution, collision status included.

    Debug companion to :func:`solve_support_ik`, and the single-arm answer to
    the dual-arm ``enumerate_ssik_candidate_pairs``: when the normal solve
    reports "no collision-free solution", this re-runs the SAME gradient
    solver with collision checking turned OFF, so every pose that physically
    reaches the target comes back — including the ones that collide. Each is
    then collision-checked with a FULL report, so a caller can step the user
    through them with the offending bodies highlighted.

    Reading the result:
      - no candidates at all -> the target is UNREACHABLE from this base
        (a kinematics/base-placement problem, not a collision one);
      - candidates that all collide -> reachable but blocked; the offenders
        say exactly what is in the way.

    Note the support robot has no analytical (ssik) solver — ssik artifacts
    are built per dual-arm arm ("left"/"right") — so these candidates are
    gradient-descent solutions and the set is not guaranteed exhaustive.

    Args:
        planner (PyBulletPlanner): the support robot's planner (its cell loaded).
        template_state (RobotCellState): the scene state to fork (not mutated).
        base_frame_world_mm (np.ndarray): 4x4 robot base pose, world mm.
        tool0_world_mm (np.ndarray): 4x4 arm flange target, world mm.
        max_results (int): cap on solutions to enumerate (default config).
        max_descend_iterations (int): per-descent iteration cap (default config).
        tolerance_position (float): position tolerance (default config).
        tolerance_orientation (float): orientation tolerance (default config).

    Returns:
        dict: ``{"n_reachable": int, "candidates": [...]}``. Each candidate is
        ``{"index", "state", "in_collision", "summary", "offenders",
        "num_offending_pairs"}`` — the same shape the dual-arm enumerator
        returns, sorted collision-free first then fewest offending pairs.
    """
    from compas_fab.backends import CollisionCheckError
    # One definition of the pair -> (kind, name) resolution, shared with the
    # dual-arm enumerator so both inspectors highlight the same way.
    from husky_assembly_tamp.keyframe.dual_arm_ik import _resolve_collision_pairs

    deps = import_compas_stack()
    Frame = deps["Frame"]
    FrameTarget = deps["FrameTarget"]
    TargetMode = deps["TargetMode"]

    max_results = max_results if max_results is not None else config.IK_MAX_RESULTS
    max_descend_iterations = (
        max_descend_iterations
        if max_descend_iterations is not None
        else config.IK_MAX_DESCEND_ITERATIONS
    )
    tolerance_position = (
        tolerance_position if tolerance_position is not None else config.IK_TOLERANCE_POSITION
    )
    tolerance_orientation = (
        tolerance_orientation
        if tolerance_orientation is not None
        else config.IK_TOLERANCE_ORIENTATION
    )

    state = template_state.copy()
    _apply_base_frame_mm(state, base_frame_world_mm)
    target = FrameTarget(
        _mm_matrix_to_m_frame(Frame, tool0_world_mm),
        TargetMode.ROBOT,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
    )
    options = {
        "max_results": max_results,
        # ! Collision OFF on purpose: we WANT the colliding poses back so the
        # user can see what blocks them. Each is checked separately below.
        "check_collision": False,
        "max_descend_iterations": max_descend_iterations,
        "verbose": False,
    }

    configurations = []
    try:
        for cfg in planner.iter_inverse_kinematics(
            target, state, group=config.SUPPORT_GROUP, options=options
        ):
            configurations.append(cfg)
    except Exception as exc:
        # InverseKinematicsError (nothing reachable) or a backend failure: an
        # empty candidate list is itself the diagnosis, so report and continue.
        print(f"core.robot_cell_support.enumerate_support_ik_candidates: {type(exc).__name__}: {exc}")

    rcell = planner.client.robot_cell
    candidates = []
    for index, cfg in enumerate(configurations):
        trial = state.copy()
        trial.robot_configuration.merge(cfg)
        in_collision = False
        summary = "clear (no collision)"
        offenders = []
        num_pairs = 0
        try:
            planner.check_collision(trial, options={"full_report": True, "verbose": False})
        except CollisionCheckError as exc:
            in_collision = True
            pairs = list(getattr(exc, "collision_pairs", []) or [])
            summary, offenders, num_pairs = _resolve_collision_pairs(pairs, rcell)
            if not pairs:
                summary = str(exc).splitlines()[0] if str(exc) else "collision"
        except Exception as exc:
            # A non-collision failure (e.g. a cell/state mismatch): still show it.
            in_collision = True
            summary = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
        candidates.append({
            "index": index,
            "state": trial,
            "in_collision": in_collision,
            "summary": summary,
            "offenders": offenders,
            "num_offending_pairs": num_pairs,
        })

    # Collision-free first, then fewest offending pairs -- so near-misses lead.
    candidates.sort(key=lambda c: (c["in_collision"], c["num_offending_pairs"]))
    return {"n_reachable": len(configurations), "candidates": candidates}
