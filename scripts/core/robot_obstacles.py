"""Frozen-robot obstacles: any robot, reused as a static articulated tool.

The compas_fab pybullet backend plans for ONE robot per client. Whenever
another robot is physically present in a scene (a support robot holding a
bar, or the assembly robot seen from a support robot's session), it is
modeled as an UNATTACHED articulated ``ToolModel`` built from that robot's
own ``RobotModel``. Its world pose and full arm configuration are frozen per
step on the ``RobotCellState``; collision checking always includes it.

This generalizes the "DualArm ToolModel obstacle" pattern that used to live
in ``core.robot_cell_support`` to every robot in the roster.

Cost discipline: loading a URDF and converting a RobotModel into a ToolModel
are both expensive, so each happens ONCE per robot per Rhino session (sticky
caches). Per-step work only touches ``state.tool_states[...]`` (frame +
configuration) — never a fresh load or conversion.

A robot that is NOT in a scene is PARKED far away (``park_robot_obstacle``),
never hidden: collisions stay on but can never trigger. This matches the
physical protocol — a released support robot drives away from the structure.
"""

from __future__ import annotations

import os

import numpy as np

from core import config
# Session/sticky plumbing + unit converters come from robot_cell. That module
# lazily imports THIS one inside two functions (base state + cell rebuild), so
# importing robot_cell at our top level is safe (no import cycle at load time).
from core.robot_cell import (
    _STICKY,
    _mm_matrix_to_m_frame,
    get_or_load_robot_cell,
    import_compas_stack,
)


# One cache slot per robot name.
_STICKY_ROBOT_MODEL_PREFIX = "bar_joint:robot_model:"
_STICKY_OBSTACLE_TOOL_PREFIX = "bar_joint:obstacle_tool_model:"


def _support_mesh_loaders(deps):
    """Mesh loaders shared by both single-arm support URDFs.

    Args:
        deps (dict): the lazily imported compas stack.

    Returns:
        tuple: ``(main_loader, extra_loaders)`` for ``RobotModel.load_geometry``.
    """
    LocalPackageMeshLoader = deps["LocalPackageMeshLoader"]
    pkg_path = config.HUSKY_PKG_PATH
    if not os.path.isdir(pkg_path):
        raise RuntimeError(
            f"Husky URDF package directory not found: {pkg_path}. "
            "Clone the husky_urdf submodule under asset/."
        )
    main_loader = LocalPackageMeshLoader(pkg_path, config.SUPPORT_URDF_PKG_NAME)
    husky_loader = LocalPackageMeshLoader(pkg_path, "husky_description")
    ur_loader = LocalPackageMeshLoader(pkg_path, "ur_description")
    return main_loader, (husky_loader, ur_loader)


def get_or_load_robot_model(robot_name: str):
    """One robot's ``RobotModel`` (geometry loaded), cached per Rhino session.

    Cindy's model is taken from the already-cached dual-arm ``RobotCell`` (no
    second URDF load); each support robot's model is loaded from its OWN
    calibrated URDF the first time it is asked for.

    Args:
        robot_name (str): "Cindy", "Alice", or "Belle".

    Returns:
        RobotModel: the cached model. Shared object — copy before mutating.

    Raises:
        RuntimeError: on an unknown robot name or a missing URDF package.
    """
    if robot_name == config.ASSEMBLY_ROBOT_NAME:
        # The dual-arm cell loader already caches Cindy's model; reuse it.
        return get_or_load_robot_cell().robot_model

    if robot_name not in config.SUPPORT_ROBOTS:
        raise RuntimeError(
            f"Unknown robot name {robot_name!r}. Known robots: "
            f"[{config.ASSEMBLY_ROBOT_NAME!r}] + {sorted(config.SUPPORT_ROBOTS)}."
        )

    key = _STICKY_ROBOT_MODEL_PREFIX + robot_name
    cached = _STICKY.get(key)
    if cached is not None:
        return cached

    deps = import_compas_stack()
    entry = config.SUPPORT_ROBOTS[robot_name]
    main_loader, extra_loaders = _support_mesh_loaders(deps)
    print(f"core.robot_obstacles: loading {robot_name} URDF ({entry['urdf_filename']!r})...")
    urdf_stream = main_loader.load_urdf(entry["urdf_filename"])
    model = deps["RobotModel"].from_urdf_string(urdf_stream.read())
    model.load_geometry(main_loader, *extra_loaders)

    _STICKY[key] = model
    return model


def _end_effector_tool_meshes(robot_name: str) -> dict:
    """The meshes to weld onto a robot's flange link(s), in tool0 coordinates.

    A robot's end-effector tools live as separate ToolModels in its OWN cell,
    so a plain URDF-to-ToolModel conversion produces a bare-armed obstacle —
    invisible in previews AND absent from collision, right where the other
    robot works. This returns the geometry to weld on so the frozen copy is
    the whole machine.

    Args:
        robot_name (str): "Cindy", "Alice", or "Belle".

    Returns:
        dict: ``{flange_link_name: [Mesh, ...]}`` (meters, tool0 frame).

    Raises:
        RuntimeError: when a required collision mesh is missing — the frozen
            robot's tools are load-bearing for collision, so a missing mesh
            is an error rather than a silently bare arm.
    """
    deps = import_compas_stack()
    Mesh = deps["Mesh"]

    if robot_name == config.ASSEMBLY_ROBOT_NAME:
        # The active arm-tool pair (RSSwapRoboticTool decides which), one per
        # flange. Their collision OBJs are authored in mm.
        from core.robotic_tool import get_active_pair

        out = {}
        pair = get_active_pair()
        for side, link_name in config.ASSEMBLY_TOOL0_LINKS.items():
            tdef = pair[side]
            col_path = tdef.collision_path()
            if not col_path or not os.path.isfile(col_path):
                raise RuntimeError(
                    f"Collision OBJ missing for tool {tdef.name!r}: {col_path!r}. "
                    f"It is needed to freeze {robot_name} as an obstacle. Re-run "
                    f"RSDefineRoboticTool (AssemblyTool mode) for {tdef.name!r}."
                )
            mesh = Mesh.from_obj(col_path)
            mesh.scale(0.001)  # OBJ exported in mm -> meters
            out[link_name] = [mesh]
        return out

    # Support robots: the Robotiq gripper, authored in meters.
    mesh_path = config.ROBOTIQ_GRIPPER_TOOL_MESH
    if not os.path.isfile(mesh_path):
        raise RuntimeError(
            f"Robotiq gripper collision mesh missing: {mesh_path!r}. It is needed "
            f"to freeze {robot_name} as an obstacle (its gripper must collide)."
        )
    return {config.SUPPORT_TOOL0_LINK: [Mesh.from_obj(mesh_path)]}


def _weld_end_effector_tools(model, robot_name: str) -> int:
    """Weld a robot's end-effector tool meshes onto its model's flange link(s).

    Each tool becomes a new link joined to its flange by a FIXED joint at the
    identity origin (the tool meshes are already expressed in tool0 coords),
    so it follows the arm as the frozen configuration is applied.

    Args:
        model (RobotModel): the COPY to mutate (never the cached original).
        robot_name (str): whose tools to weld.

    Returns:
        int: how many tool links were welded.

    Raises:
        RuntimeError: if a flange link named in the config is not in the model.
    """
    from compas_robots.model import Joint

    welded = 0
    for link_name, meshes in _end_effector_tool_meshes(robot_name).items():
        parent = model.get_link_by_name(link_name)
        if parent is None:
            raise RuntimeError(
                f"{robot_name}'s model has no link {link_name!r} to weld its tool "
                "onto. Check the *_TOOL0_LINK names in core.config against the URDF."
            )
        child = model.add_link(
            f"{link_name}_obstacle_tool",
            visual_meshes=list(meshes),
            collision_meshes=list(meshes),
        )
        model.add_joint(
            f"{link_name}_obstacle_tool_joint",
            Joint.FIXED,
            parent,
            child,
        )
        welded += 1
    return welded


def get_or_load_obstacle_tool_model(robot_name: str):
    """One robot's frozen-obstacle ``ToolModel``, converted once and cached.

    The model is the robot PLUS its end-effector tools welded on (see
    :func:`_weld_end_effector_tools`), so the frozen copy collides and draws
    as the complete machine. ``ToolModel.from_robot_model`` is the expensive
    conversion — it runs once per robot per Rhino session; every cell that
    needs this robot as an obstacle registers the SAME ToolModel object.

    Args:
        robot_name (str): "Cindy", "Alice", or "Belle".

    Returns:
        ToolModel: named ``config.OBSTACLE_TOOL_NAMES[robot_name]``.
    """
    key = _STICKY_OBSTACLE_TOOL_PREFIX + robot_name
    cached = _STICKY.get(key)
    if cached is not None:
        return cached

    deps = import_compas_stack()
    Frame = deps["Frame"]
    ToolModel = deps["ToolModel"]
    print(f"core.robot_obstacles: converting {robot_name} RobotModel -> obstacle ToolModel...")
    # Copy first: the weld mutates the model, and the cached original is the
    # actuated robot's own model in its own cell.
    model = get_or_load_robot_model(robot_name).copy()
    n_welded = _weld_end_effector_tools(model, robot_name)
    tool = ToolModel.from_robot_model(model, Frame.worldXY())
    tool.name = config.OBSTACLE_TOOL_NAMES[robot_name]
    print(
        f"core.robot_obstacles: {robot_name} obstacle carries {n_welded} welded "
        "end-effector tool(s)."
    )

    _STICKY[key] = tool
    return tool


def attach_obstacle_robot_tools(cell, present_robot_names: list):
    """Register frozen-robot obstacle ToolModels into a cell (once per cell).

    Args:
        cell (RobotCell): the cell to register into (mutated in place).
        present_robot_names (list): robot names that may appear in this
            cell's scenes, e.g. ["Alice", "Belle"] for Cindy's cell.
    """
    for robot_name in present_robot_names:
        tool_name = config.OBSTACLE_TOOL_NAMES[robot_name]
        if tool_name not in cell.tool_models:
            cell.tool_models[tool_name] = get_or_load_obstacle_tool_model(robot_name)


def configure_robot_obstacle(
    state,
    robot_name: str,
    base_frame_world_mm: np.ndarray,
    joint_values,
    joint_names=None,
):
    """Freeze one robot obstacle at a base pose + arm configuration.

    Works for both a 6-joint support robot and the 12-joint dual-arm robot.
    The values overwrite the tool's zero configuration; joints not covered by
    ``joint_values`` stay at zero.

    Args:
        state (RobotCellState): the state to stamp (mutated in place).
        robot_name (str): which robot obstacle to configure.
        base_frame_world_mm (np.ndarray): 4x4 world base pose, mm.
        joint_values (list): flat joint values. For Cindy pass left arm's 6
            then right arm's 6 (URDF declaration order) — or give
            ``joint_names`` to match by name instead of order.
        joint_names (list): optional names matching ``joint_values``; when
            given, values are merged by NAME into the zero configuration.

    Returns:
        RobotCellState: the same ``state``, for chaining.

    Raises:
        RuntimeError: if more values than configurable joints are given, or
            the state has no tool_state for this robot (cell not registered).
    """
    deps = import_compas_stack()
    Frame = deps["Frame"]

    tool_name = config.OBSTACLE_TOOL_NAMES[robot_name]
    if tool_name not in state.tool_states:
        raise RuntimeError(
            f"State has no tool_state {tool_name!r} — the cell this state came "
            f"from was built without {robot_name}'s obstacle tool. Register it "
            "with attach_obstacle_robot_tools at cell build."
        )
    tool = get_or_load_obstacle_tool_model(robot_name)

    state.tool_states[tool_name].frame = _mm_matrix_to_m_frame(
        Frame, np.asarray(base_frame_world_mm, dtype=float)
    )

    zero_cfg = tool.zero_configuration()
    cfg_joint_names = list(zero_cfg.joint_names)
    cfg_joint_types = list(zero_cfg.joint_types)
    cfg_values = list(zero_cfg.joint_values)

    if joint_names:
        captured = dict(zip(list(joint_names), [float(v) for v in joint_values]))
        for i, name in enumerate(cfg_joint_names):
            if name in captured:
                cfg_values[i] = captured[name]
    else:
        flat = [float(v) for v in joint_values]
        if len(flat) > len(cfg_values):
            raise RuntimeError(
                f"{robot_name} obstacle tool has {len(cfg_values)} configurable "
                f"joints but received {len(flat)} values."
            )
        for i, v in enumerate(flat):
            cfg_values[i] = v

    state.tool_states[tool_name].configuration = deps["Configuration"](
        joint_values=cfg_values,
        joint_types=cfg_joint_types,
        joint_names=cfg_joint_names,
    )
    return state


def park_robot_obstacle(state, robot_name: str):
    """Move one robot obstacle to the faraway parking spot (zero arm config).

    Used for a robot that is NOT in the scene: a support robot before its
    first deployment or after its release, and Cindy in release-time scenes.
    Parking — never hiding — keeps collision checking on while making it
    geometrically impossible to trigger.

    Args:
        state (RobotCellState): the state to stamp (mutated in place).
        robot_name (str): which robot obstacle to park.

    Returns:
        RobotCellState: the same ``state``, for chaining.
    """
    parked = np.asarray(config.ROBOT_PARKED_BASE_FRAME_MM, dtype=float)
    return configure_robot_obstacle(state, robot_name, parked, joint_values=[])
