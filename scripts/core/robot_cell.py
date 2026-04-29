"""Dual-arm Husky robot cell loading, PyBullet lifecycle, and IK helpers.

Adapted from the Grasshopper prototype scripts under
`support_materials/gh_keyframe_demos/python/`:

- `GH_create_robot_model.py` + `GH_create_robot_cell.py` → `get_or_load_robot_cell`
- `GH_init_pb.py` → `start_pb_client`
- `GH_disconnect_pb.py` → `stop_pb_client`
- `GH_set_cell_state.py` → `set_cell_state`
- `GH_ik_dual_arm_keyframe.py` → `solve_dual_arm_ik`

compas_fab consumes SI units (meters, radians). Public 4x4 transforms here
are mm (matching `core.config`). Unit conversion happens at the compas
boundary inside this module.
"""

from __future__ import annotations

import os
import sys

import numpy as np

from core import config


# When imported inside Rhino (ScriptEditor), `scriptcontext.sticky` is the
# canonical per-session cache. Outside Rhino (pure-Python replays / pytest),
# `scriptcontext` doesn't exist; fall back to a module-level dict so the rest
# of this module's API works unchanged. Tests can still introspect and reset
# the cache via `_STICKY` directly.
try:
    import scriptcontext as _sc  # type: ignore
    _STICKY = _sc.sticky
except ImportError:
    _STICKY: dict = {}


_STICKY_ROBOT_CELL = "bar_joint:robot_cell"
_STICKY_PB_CLIENT = "bar_joint:pb_client"
_STICKY_PB_PLANNER = "bar_joint:pb_planner"
_STICKY_CURRENT_CELL_KIND = "bar_joint:current_cell_kind"  # "dual_arm" | "support"


# ---------------------------------------------------------------------------
# compas_fab from the in-repo submodule (`external/compas_fab`, branch
# `wip_process`). We (a) inject its `src/` folder onto the front of
# sys.path, and (b) purge any previously-imported `compas_fab*` modules
# from sys.modules so a stale copy cached from a sibling repo in the same
# Rhino runtime session (e.g. `husky-assembly-teleop/external/compas_fab`)
# gets replaced by ours.
#
# This avoids the Rhino ScriptEditor `# r:` pip-cache gotcha for
# branch-tracking installs — see `tasks/yh_lesson.md`.
# ---------------------------------------------------------------------------

_COMPAS_FAB_SRC = os.path.normpath(
    os.path.join(config.REPO_ROOT, "external", "compas_fab", "src")
)


def _ensure_submodule_compas_fab_loaded(verbose: bool = True):
    """Verify `import compas_fab` resolves to our in-repo submodule copy.

    Strategy: prepend the submodule `src/` to `sys.path`, then import
    compas_fab once if it's not already cached. We DO NOT purge stale
    entries from `sys.modules` — that path created hard-to-debug
    class-identity bugs (e.g. `super(PyBulletBase, self)` failing because
    `compas` / `compas_robots` plugin registries held references to the
    previous PyBulletBase class object). To force a fresh re-import of
    compas_fab — for example after switching the submodule SHA — use
    ScriptEditor → Tools → Reload Python 3 (CPython) Engine. The engine
    reload clears the interpreter state cleanly; mid-run re-imports of
    one library while siblings still cache its old classes do not.

    If compas_fab is already loaded from somewhere else (e.g. a leftover
    editable install pointing at a sibling repo), this raises with
    actionable guidance.
    """
    if not os.path.isdir(_COMPAS_FAB_SRC):
        raise RuntimeError(
            f"compas_fab submodule missing at {_COMPAS_FAB_SRC}. "
            "Run `git submodule update --init --recursive`."
        )

    if _COMPAS_FAB_SRC not in sys.path:
        sys.path.insert(0, _COMPAS_FAB_SRC)

    if "compas_fab" not in sys.modules:
        import compas_fab  # noqa: F401  — first import resolves via sys.path

    cached = sys.modules["compas_fab"]
    loaded_from = getattr(cached, "__file__", "") or ""
    target = os.path.normcase(_COMPAS_FAB_SRC)
    if not os.path.normcase(loaded_from).startswith(target):
        raise RuntimeError(
            f"compas_fab is loaded from '{loaded_from}', NOT from the in-repo submodule "
            f"at '{_COMPAS_FAB_SRC}'.\n"
            "Reset the Python 3 (CPython) engine (ScriptEditor -> Tools -> Reload Python 3 Engine) "
            "and click again. If the problem persists, you have a leftover editable install of "
            "compas_fab in the base Rhino interpreter; see tasks/yh_lesson.md for cleanup steps."
        )
    if verbose:
        print(f"compas_fab loaded from: {loaded_from}")


_ensure_submodule_compas_fab_loaded(verbose=False)


# ---------------------------------------------------------------------------
# Deferred imports
# ---------------------------------------------------------------------------
# compas, compas_fab, compas_robots, and pybullet are heavy optional
# dependencies. We import them lazily so that importing this module does not
# pay that cost on every Rhino script run.


def import_compas_stack():
    """Lazy-import the compas / compas_fab / pybullet dependency stack.

    Returned dict contains the concrete classes/modules used by this module
    and by `core.ik_viz`. Reuse across the project to avoid scattered top-
    level imports (which Rhino's ScriptEditor pays the cost for on first run).
    """
    import compas  # noqa: F401 - consumer below
    from compas.geometry import Frame, Transformation  # noqa: F401
    from compas.scene import Scene  # noqa: F401
    from compas.datastructures import Mesh  # noqa: F401
    from compas_robots import Configuration, RobotModel, ToolModel  # noqa: F401
    from compas_robots.resources import LocalPackageMeshLoader  # noqa: F401
    from compas_fab.robots import RobotCell, RobotSemantics  # noqa: F401
    from compas_fab.backends import PyBulletClient, PyBulletPlanner  # noqa: F401
    from compas_fab.robots import FrameTarget, TargetMode  # noqa: F401
    import pybullet_planning as pp  # noqa: F401

    return {
        "compas": compas,
        "Frame": Frame,
        "Transformation": Transformation,
        "Scene": Scene,
        "Mesh": Mesh,
        "Configuration": Configuration,
        "RobotModel": RobotModel,
        "ToolModel": ToolModel,
        "LocalPackageMeshLoader": LocalPackageMeshLoader,
        "RobotCell": RobotCell,
        "RobotSemantics": RobotSemantics,
        "PyBulletClient": PyBulletClient,
        "PyBulletPlanner": PyBulletPlanner,
        "FrameTarget": FrameTarget,
        "TargetMode": TargetMode,
        "pp": pp,
    }


# Backwards-compatible private alias (kept for internal callers below)
_import_compas_stack = import_compas_stack


# ---------------------------------------------------------------------------
# Unit helpers
# ---------------------------------------------------------------------------


def _mm_matrix_to_m_frame(Frame, matrix_mm: np.ndarray):
    """Convert a 4x4 matrix with mm translation to a compas Frame in meters."""
    matrix = np.asarray(matrix_mm, dtype=float)
    origin = matrix[:3, 3] / 1000.0
    x_axis = matrix[:3, 0]
    y_axis = matrix[:3, 1]
    return Frame(list(map(float, origin)), list(map(float, x_axis)), list(map(float, y_axis)))


def _frame_to_m_matrix(frame) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, 0] = np.asarray(frame.xaxis, dtype=float)
    matrix[:3, 1] = np.asarray(frame.yaxis, dtype=float)
    matrix[:3, 2] = np.asarray(frame.zaxis, dtype=float)
    matrix[:3, 3] = np.asarray(frame.point, dtype=float)
    return matrix


def _pose_from_frame(frame):
    return (list(frame.point), list(frame.quaternion.xyzw))


# ---------------------------------------------------------------------------
# Robot cell loading
# ---------------------------------------------------------------------------


_TOOL_TARGETS = (
    # (tool_name, group_name, touch_links, mesh_path_attr)
    # touch_links cover the arm links the pineapple OBJ INHERENTLY overlaps
    # when attached at tool0. The proxy mesh extends ~157 mm backward from
    # tool0 along -Z (wrist visualization plus tool), which on a UR5e
    # reaches across wrist_3 + wrist_2 + into wrist_1's volume. All three
    # must be allowed; otherwise IK rejects valid poses on the wrist_1
    # overlap that's a property of the proxy, not a real collision.
    # forearm_link is left OUT — the proxy does not reach forearm in
    # practice; if it ever does, that IS worth flagging as a real bad pose.
    (
        "LEFT_TOOL_NAME", "LEFT_GROUP",
        [
            "left_ur_arm_wrist_1_link",
            "left_ur_arm_wrist_2_link",
            "left_ur_arm_wrist_3_link",
        ],
        "LEFT_PINEAPPLE_TOOL_MESH",
    ),
    (
        "RIGHT_TOOL_NAME", "RIGHT_GROUP",
        [
            "right_ur_arm_wrist_1_link",
            "right_ur_arm_wrist_2_link",
            "right_ur_arm_wrist_3_link",
        ],
        "RIGHT_PINEAPPLE_TOOL_MESH",
    ),
)


def _attach_tool_models(robot_cell, deps):
    """Load pineapple OBJs as `ToolModel`s on `robot_cell.tool_models`.

    Idempotent: if the same OBJ has already been attached under the same
    tool_name, this is a no-op (per `_loaded_from` marker stored on the
    ToolModel). Missing OBJs are skipped with a one-line note pointing at
    `RSExportPineappleOBJ`.

    The attached tool's `frame` is `Frame.worldXY()` because the OBJ was
    exported from a Rhino block whose local origin coincides with tool0
    (block instances are placed at tool0 world frames in `rs_ik_keyframe`).
    """
    Frame = deps["Frame"]
    Mesh = deps["Mesh"]
    ToolModel = deps["ToolModel"]
    for tool_attr, _group_attr, _touch_links, mesh_attr in _TOOL_TARGETS:
        tool_name = getattr(config, tool_attr)
        mesh_path = getattr(config, mesh_attr)
        existing = robot_cell.tool_models.get(tool_name)
        if existing is not None and getattr(existing, "_loaded_from", None) == mesh_path:
            continue  # already attached from this exact OBJ
        if not os.path.isfile(mesh_path):
            if existing is None:
                print(
                    f"core.robot_cell: tool mesh '{mesh_path}' missing; "
                    f"skipping tool '{tool_name}'. Run RSExportPineappleOBJ "
                    f"so IK collision can see the wrist + tool geometry."
                )
            continue
        mesh = Mesh.from_obj(mesh_path)
        tool = ToolModel(mesh, Frame.worldXY(), name=tool_name)
        try:
            tool._loaded_from = mesh_path  # cheap cache key for the next call
        except AttributeError:
            pass
        robot_cell.tool_models[tool_name] = tool
        print(f"core.robot_cell: attached tool '{tool_name}' from {mesh_path}")


def _configure_tool_states(state):
    """Wire `tool_states[AL/AR]` to their planning groups + touch links.

    Mirrors `GH_create_robot_cell.py`. Run on every fresh state because
    `RobotCell.default_cell_state()` produces a state with default
    (unattached) tool_states each time.
    """
    for tool_attr, group_attr, touch_links, _mesh_attr in _TOOL_TARGETS:
        tool_name = getattr(config, tool_attr)
        if tool_name not in state.tool_states:
            continue  # tool wasn't attached (mesh missing) — IK still works, just no tool collision
        ts = state.tool_states[tool_name]
        ts.attached_to_group = getattr(config, group_attr)
        ts.touch_links = list(touch_links)


def get_or_load_robot_cell():
    """Return a cached `RobotCell`, loading URDF/SRDF + geometry on first call.

    Tool models are (re-)attached on every call so that exporting the
    pineapple OBJs after the cell was first loaded picks up automatically
    on the next call. The PyBullet planner snapshot (`set_robot_cell`)
    happens once at `start_pb_client`; if you change tool meshes mid-
    session, run `RSPBStop` + `RSPBStart` (or Reload Python 3 Engine) to
    push the new geometry into the planner's collision world.
    """
    cached = _STICKY.get(_STICKY_ROBOT_CELL)
    if cached is not None:
        deps = _import_compas_stack()
        _attach_tool_models(cached, deps)
        return cached

    deps = _import_compas_stack()
    deps["compas"].PRECISION = "12f"

    pkg_path = config.HUSKY_PKG_PATH
    if not os.path.isdir(pkg_path):
        raise RuntimeError(
            f"Husky URDF package directory not found: {pkg_path}. "
            "Clone the husky_urdf submodule under asset/."
        )

    LocalPackageMeshLoader = deps["LocalPackageMeshLoader"]
    main_loader = LocalPackageMeshLoader(pkg_path, config.HUSKY_URDF_PKG_NAME)
    husky_loader = LocalPackageMeshLoader(pkg_path, "husky_description")
    dualarm_husky_loader = LocalPackageMeshLoader(pkg_path, "husky_ur_description")
    ur_loader = LocalPackageMeshLoader(pkg_path, "ur_description")

    urdf_stream = main_loader.load_urdf(config.HUSKY_URDF_FILENAME)
    robot_model = deps["RobotModel"].from_urdf_string(urdf_stream.read())
    robot_model.load_geometry(main_loader, husky_loader, dualarm_husky_loader, ur_loader)

    srdf_path = main_loader.build_path(
        os.path.dirname(config.HUSKY_SRDF_REL_PATH),
        os.path.basename(config.HUSKY_SRDF_REL_PATH),
    )
    robot_semantics = deps["RobotSemantics"].from_srdf_file(srdf_path, robot_model)

    robot_cell = deps["RobotCell"](robot_model, robot_semantics)
    _attach_tool_models(robot_cell, deps)

    _STICKY[_STICKY_ROBOT_CELL] = robot_cell
    return robot_cell


def default_cell_state():
    """Return a fresh default `RobotCellState` with tool attachments wired in.

    Callers should use this rather than `robot_cell.default_cell_state()`
    directly so the per-state `tool_states[AL/AR].attached_to_group` and
    `touch_links` get configured in lockstep with the tool models.
    """
    robot_cell = get_or_load_robot_cell()
    state = robot_cell.default_cell_state()
    _configure_tool_states(state)
    return state


# ---------------------------------------------------------------------------
# PyBullet lifecycle
# ---------------------------------------------------------------------------


def start_pb_client(use_gui: bool = False, verbose: bool = True):
    """Start PyBullet, load the robot cell into the planner, cache in sticky.

    Note: `verbose=True` by default. Setting it to False triggers
    `compas_fab.backends.pybullet.utils.redirect_stdout`, which calls
    `os.dup(fd)` on stdout — that raises inside Rhino's ScriptEditor
    (Rhino's stdout is not a normal OS file descriptor). Keep verbose on
    unless you are running outside Rhino.
    """
    if _STICKY.get(_STICKY_PB_CLIENT) is not None:
        raise RuntimeError(
            "PyBullet client already running. Call stop_pb_client() first."
        )

    # Defensive: re-run the submodule purge-and-reload every RSPBStart, so
    # edits to this file or a stale sys.modules entry from a sibling repo
    # cannot cause a silent shadowing. Idempotent when our copy is already
    # the resolved one.
    _ensure_submodule_compas_fab_loaded(verbose=True)

    deps = _import_compas_stack()
    client = deps["PyBulletClient"](
        connection_type="gui" if use_gui else "direct",
        verbose=verbose,
    )
    client.__enter__()
    planner = deps["PyBulletPlanner"](client)

    robot_cell = get_or_load_robot_cell()
    planner.set_robot_cell(robot_cell)

    _STICKY[_STICKY_PB_CLIENT] = client
    _STICKY[_STICKY_PB_PLANNER] = planner
    _STICKY[_STICKY_CURRENT_CELL_KIND] = "dual_arm"
    return client, planner


def stop_pb_client():
    """Disconnect PyBullet; always clear sticky entries, even on disconnect failure.

    Robust against the case where PyBullet is already disconnected (user
    closed the GUI window, or a prior stop partially succeeded): we swallow
    the disconnect error and still clear our sticky cache, so the next
    RSPBStart sees a clean slate.
    """
    try:
        deps = _import_compas_stack()
        deps["pp"].disconnect()
    except Exception as exc:
        print(f"stop_pb_client: disconnect raised ({exc}); clearing sticky anyway.")
    finally:
        _STICKY.pop(_STICKY_PB_CLIENT, None)
        _STICKY.pop(_STICKY_PB_PLANNER, None)


def get_planner():
    """Return the cached `(client, planner)` tuple or raise if not started."""
    if not is_pb_running():
        raise RuntimeError("PyBullet client not started. Run RSPBStart first.")
    client = _STICKY.get(_STICKY_PB_CLIENT)
    planner = _STICKY.get(_STICKY_PB_PLANNER)
    return client, planner


def _pybullet_connected() -> bool:
    """Whether PyBullet itself reports an active connection."""
    try:
        import pybullet_planning as pp
        return bool(pp.is_connected())
    except Exception:
        return False


def is_pb_running() -> bool:
    """Authoritative "is the PB client usable right now?" check.

    Cross-references our sticky cache against PyBullet's own connection
    state and self-heals on mismatch:

    - sticky populated, PB connected  → True
    - sticky populated, PB disconnected (user closed GUI)  → clean sticky, return False
    - sticky empty                    → False (regardless of PB state)
    """
    sticky_has = _STICKY.get(_STICKY_PB_CLIENT) is not None
    if not sticky_has:
        return False
    if _pybullet_connected():
        return True
    # Sticky thinks we're running but PyBullet is not — stale state; clean it.
    _STICKY.pop(_STICKY_PB_CLIENT, None)
    _STICKY.pop(_STICKY_PB_PLANNER, None)
    print("is_pb_running: stale sticky detected (PyBullet no longer connected); cleared cache.")
    return False


# ---------------------------------------------------------------------------
# Cell state / IK
# ---------------------------------------------------------------------------


def set_cell_state(planner, robot_cell_state):
    """Push a robot cell state into PyBullet (matches GH_set_cell_state.py).

    If a different cell kind is currently loaded into the planner (e.g. the
    support cell from a prior `RSIKSupportKeyframe` run), swap back to the
    dual-arm cell first so the IK / collision check operates against the
    right RobotCell.
    """
    deps = _import_compas_stack()
    if _STICKY.get(_STICKY_CURRENT_CELL_KIND) != "dual_arm":
        planner.set_robot_cell(get_or_load_robot_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "dual_arm"
    planner.set_robot_cell_state(robot_cell_state)
    base_frame = robot_cell_state.robot_base_frame
    deps["pp"].set_pose(planner.client.robot_puid, _pose_from_frame(base_frame))


def _apply_base_frame_mm(state, base_frame_world_mm: np.ndarray):
    deps = _import_compas_stack()
    state.robot_base_frame = _mm_matrix_to_m_frame(deps["Frame"], base_frame_world_mm)


def solve_dual_arm_ik(
    planner,
    template_state,
    base_frame_world_mm: np.ndarray,
    tool0_left_world_mm: np.ndarray,
    tool0_right_world_mm: np.ndarray,
    *,
    check_collision: bool = True,
    max_results: int = None,
    max_descend_iterations: int = None,
    tolerance_position: float = None,
    tolerance_orientation: float = None,
):
    """Solve IK for the left and right groups in order.

    All 4x4 inputs are mm (consistent with `core.config`). Converted to
    meters for compas_fab internally.

    Returns the mutated copy of `template_state` on success, or None on
    any-group failure. The caller can call `set_cell_state(planner, state)`
    to view the result.
    """
    deps = _import_compas_stack()
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

    left_target = FrameTarget(
        _mm_matrix_to_m_frame(Frame, tool0_left_world_mm),
        TargetMode.ROBOT,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
    )
    right_target = FrameTarget(
        _mm_matrix_to_m_frame(Frame, tool0_right_world_mm),
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

    for target, group in (
        (left_target, config.LEFT_GROUP),
        (right_target, config.RIGHT_GROUP),
    ):
        try:
            cfg = planner.inverse_kinematics(target, state, group=group, options=options)
        except Exception as exc:
            print(f"IK failed for group '{group}': {exc}")
            return None
        if cfg is None:
            print(f"IK returned no solution for group '{group}'.")
            return None
        state.robot_configuration.merge(cfg)

    return state


def extract_group_config(state, group: str, robot_cell) -> dict:
    """Return `{'joint_names': [...], 'joint_values': [...]}` for a group."""
    group_joint_names = list(robot_cell.get_configurable_joint_names(group))
    joint_values = [
        float(state.robot_configuration[name]) for name in group_joint_names
    ]
    return {"joint_names": group_joint_names, "joint_values": joint_values}
