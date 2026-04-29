"""Single-arm Husky (support) robot cell: loads the Alice URDF/SRDF and
attaches the dual-arm Husky as a ToolModel obstacle.

Mirrors :mod:`core.robot_cell` but only one arm is actuated — the dual-arm
robot is frozen at a captured pose via ``tool_states['DualArm'].configuration``.
PyBullet client + planner sticky keys are SHARED with `core.robot_cell` so
only one PB client lives per Rhino session; cell switching swaps which
:class:`RobotCell` is currently loaded into the planner.
"""

from __future__ import annotations

import os

import numpy as np

from core import config
from core import robot_cell as _rc

# Reuse the single PB client + planner sticky cache + the cross-module
# `_STICKY_CURRENT_CELL_KIND` flag (the dual-arm side updates it too — see
# `core.robot_cell.set_cell_state`).
from core.robot_cell import (  # noqa: F401  re-exported for callers
    import_compas_stack,
    start_pb_client,
    stop_pb_client,
    get_planner,
    is_pb_running,
    _STICKY,
    _STICKY_PB_CLIENT,
    _STICKY_PB_PLANNER,
    _STICKY_ROBOT_CELL,
    _STICKY_CURRENT_CELL_KIND,
    _mm_matrix_to_m_frame,
    _frame_to_m_matrix,
    _pose_from_frame,
)


_STICKY_SUPPORT_CELL = "bar_joint:support_cell"
_STICKY_SUPPORT_DUAL_ARM_MODEL = "bar_joint:support_cell_dual_arm_model"


# ---------------------------------------------------------------------------
# Dual-arm RobotModel (used as a ToolModel obstacle on the support cell)
# ---------------------------------------------------------------------------


def _get_or_load_dual_arm_robot_model():
    cached = _STICKY.get(_STICKY_SUPPORT_DUAL_ARM_MODEL)
    if cached is not None:
        return cached

    deps = import_compas_stack()
    LocalPackageMeshLoader = deps["LocalPackageMeshLoader"]
    pkg_path = config.HUSKY_PKG_PATH
    if not os.path.isdir(pkg_path):
        raise RuntimeError(
            f"Husky URDF package directory not found: {pkg_path}. "
            "Clone the husky_urdf submodule under asset/."
        )

    main_loader = LocalPackageMeshLoader(pkg_path, config.HUSKY_URDF_PKG_NAME)
    husky_loader = LocalPackageMeshLoader(pkg_path, "husky_description")
    dualarm_loader = LocalPackageMeshLoader(pkg_path, "husky_ur_description")
    ur_loader = LocalPackageMeshLoader(pkg_path, "ur_description")

    urdf_stream = main_loader.load_urdf(config.HUSKY_URDF_FILENAME)
    model = deps["RobotModel"].from_urdf_string(urdf_stream.read())
    model.load_geometry(main_loader, husky_loader, dualarm_loader, ur_loader)

    _STICKY[_STICKY_SUPPORT_DUAL_ARM_MODEL] = model
    return model


# ---------------------------------------------------------------------------
# Support cell loading
# ---------------------------------------------------------------------------


def _attach_support_tool_models(cell):
    deps = import_compas_stack()
    Frame = deps["Frame"]
    Mesh = deps["Mesh"]
    ToolModel = deps["ToolModel"]

    sg_name = config.SUPPORT_TOOL_NAME
    mesh_path = config.ROBOTIQ_GRIPPER_TOOL_MESH
    existing = cell.tool_models.get(sg_name)
    already_loaded = existing is not None and getattr(existing, "_loaded_from", None) == mesh_path
    if not already_loaded:
        if os.path.isfile(mesh_path):
            mesh = Mesh.from_obj(mesh_path)
            tool = ToolModel(mesh, Frame.worldXY(), name=sg_name)
            try:
                tool._loaded_from = mesh_path
            except AttributeError:
                pass
            cell.tool_models[sg_name] = tool
        else:
            print(
                f"core.robot_cell_support: Robotiq mesh '{mesh_path}' missing; "
                f"SupportGripper tool not attached. Export it first."
            )

    da_name = config.DUAL_ARM_OBSTACLE_TOOL_NAME
    if da_name not in cell.tool_models:
        da_model = _get_or_load_dual_arm_robot_model()
        cell.tool_models[da_name] = ToolModel.from_robot_model(da_model.copy(), Frame.worldXY())


def get_or_load_support_cell():
    cached = _STICKY.get(_STICKY_SUPPORT_CELL)
    if cached is not None:
        _attach_support_tool_models(cached)
        return cached

    deps = import_compas_stack()
    deps["compas"].PRECISION = "12f"

    pkg_path = config.HUSKY_PKG_PATH
    if not os.path.isdir(pkg_path):
        raise RuntimeError(
            f"Husky URDF package directory not found: {pkg_path}. "
            "Clone the husky_urdf submodule under asset/."
        )

    LocalPackageMeshLoader = deps["LocalPackageMeshLoader"]
    main_loader = LocalPackageMeshLoader(pkg_path, config.SUPPORT_URDF_PKG_NAME)
    husky_loader = LocalPackageMeshLoader(pkg_path, "husky_description")
    ur_loader = LocalPackageMeshLoader(pkg_path, "ur_description")

    urdf_stream = main_loader.load_urdf(config.SUPPORT_URDF_FILENAME)
    robot_model = deps["RobotModel"].from_urdf_string(urdf_stream.read())
    robot_model.load_geometry(main_loader, husky_loader, ur_loader)

    srdf_path = main_loader.build_path(
        os.path.dirname(config.SUPPORT_SRDF_REL_PATH),
        os.path.basename(config.SUPPORT_SRDF_REL_PATH),
    )
    semantics = deps["RobotSemantics"].from_srdf_file(srdf_path, robot_model)

    cell = deps["RobotCell"](robot_model, semantics)
    _attach_support_tool_models(cell)

    _STICKY[_STICKY_SUPPORT_CELL] = cell
    return cell


def _configure_support_tool_states(state):
    sg_name = config.SUPPORT_TOOL_NAME
    if sg_name in state.tool_states:
        ts = state.tool_states[sg_name]
        ts.attached_to_group = config.SUPPORT_GROUP
        ts.touch_links = list(config.SUPPORT_TOOL_TOUCH_LINKS)
    # DualArm tool deliberately stays unattached (static obstacle).


def default_support_cell_state():
    cell = get_or_load_support_cell()
    state = cell.default_cell_state()
    _configure_support_tool_states(state)
    return state


# ---------------------------------------------------------------------------
# Dual-arm-as-tool obstacle configuration
# ---------------------------------------------------------------------------


def configure_dual_arm_obstacle(
    state,
    base_frame_world_mm: np.ndarray,
    joint_values_left,
    joint_values_right,
    joint_names_left=None,
    joint_names_right=None,
):
    """Pin the DualArm tool's world pose + internal joint configuration.

    The DualArm tool is the dual-arm robot reused as a ``ToolModel`` (one
    actuated robot per :class:`RobotCell`). We freeze it at the captured
    pose by setting ``tool_states[DualArm].frame`` (world placement of the
    husky base in mm) and ``tool_states[DualArm].configuration`` (left +
    right arm joint values).

    Joint names are optional: if not provided, the order is taken from
    the ToolModel's own configurable joints (which preserves URDF
    declaration order).
    """
    deps = import_compas_stack()
    Frame = deps["Frame"]

    cell = get_or_load_support_cell()
    da_name = config.DUAL_ARM_OBSTACLE_TOOL_NAME
    da_tool = cell.tool_models[da_name]

    state.tool_states[da_name].frame = _mm_matrix_to_m_frame(Frame, base_frame_world_mm)

    zero_cfg = da_tool.zero_configuration()
    cfg_joint_names = list(zero_cfg.joint_names)
    cfg_joint_types = list(zero_cfg.joint_types)
    cfg_values = list(zero_cfg.joint_values)

    if joint_names_left and joint_names_right:
        captured = dict(zip(list(joint_names_left) + list(joint_names_right),
                            list(joint_values_left) + list(joint_values_right)))
        for i, name in enumerate(cfg_joint_names):
            if name in captured:
                cfg_values[i] = float(captured[name])
    else:
        # Trust order: left arm first, right arm second, matching URDF declaration.
        flat = list(joint_values_left) + list(joint_values_right)
        if len(flat) > len(cfg_values):
            raise RuntimeError(
                f"DualArm tool has {len(cfg_values)} configurable joints but received "
                f"{len(flat)} values (left+right)."
            )
        for i, v in enumerate(flat):
            cfg_values[i] = float(v)

    state.tool_states[da_name].configuration = deps["Configuration"](
        joint_values=cfg_values,
        joint_types=cfg_joint_types,
        joint_names=cfg_joint_names,
    )
    return state


# ---------------------------------------------------------------------------
# Cell state push (with cell-kind switching)
# ---------------------------------------------------------------------------


def set_cell_state(planner, robot_cell_state):
    """Push state to PyBullet, swapping the planner's robot cell to the support
    cell first if a different cell kind is currently loaded.
    """
    deps = import_compas_stack()
    current_kind = _STICKY.get(_STICKY_CURRENT_CELL_KIND)
    if current_kind != "support":
        planner.set_robot_cell(get_or_load_support_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "support"

    planner.set_robot_cell_state(robot_cell_state)
    base_frame = robot_cell_state.robot_base_frame
    deps["pp"].set_pose(planner.client.robot_puid, _pose_from_frame(base_frame))


# ---------------------------------------------------------------------------
# IK
# ---------------------------------------------------------------------------


def _apply_base_frame_mm(state, base_frame_world_mm: np.ndarray):
    deps = import_compas_stack()
    state.robot_base_frame = _mm_matrix_to_m_frame(deps["Frame"], base_frame_world_mm)


def _ensure_support_cell_loaded(planner):
    """Swap the planner's robot cell to the support cell if a different
    kind (e.g. dual-arm from a prior RSIKKeyframe run, or the default
    loaded by RSPBStart) is currently active.

    Without this, ``planner.inverse_kinematics(state=support_state)`` raises
    "The tools in the cell state do not match the tools in the robot cell"
    because the planner's tool_models (AssemblyLeftTool/AssemblyRightTool)
    don't match the state's tool_states (SupportGripper/DualArm).
    """
    if _STICKY.get(_STICKY_CURRENT_CELL_KIND) != "support":
        planner.set_robot_cell(get_or_load_support_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "support"


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
    """Solve IK for the single support-arm group. Returns mutated state or None."""
    _ensure_support_cell_loaded(planner)
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
        from core import env_collision
        print(env_collision.summarize_check_collision(planner, state))

    return state


def extract_group_config(state, group: str, robot_cell) -> dict:
    """Return ``{'joint_names': [...], 'joint_values': [...]}`` for a group."""
    return _rc.extract_group_config(state, group, robot_cell)
