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
import time

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
_STICKY_ENV_GEOM = "bar_joint:env_geom"
# Static-cell snapshot: the full canonical assembly (bars + joints + obstacles)
# + tool models, built once by `rebuild_assembly_cell` (RSRebuildRobotCell) and
# reused by every per-command `ensure_assembly_cell`. Plus a cheap fingerprint
# for the staleness warning. Tied to the PB session -> cleared on stop_pb_client.
_STICKY_ASSEMBLY_SNAPSHOT = "bar_joint:assembly_cell_snapshot"
_STICKY_ASSEMBLY_FINGERPRINT = "bar_joint:assembly_cell_fingerprint"
# Fingerprint the user explicitly chose to "Proceed" with at a staleness prompt,
# so we don't re-prompt on every command until the geometry changes again.
_STICKY_ASSEMBLY_STALE_ACK = "bar_joint:assembly_cell_stale_ack"


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
    from compas_fab.robots import RigidBody, RigidBodyState  # noqa: F401
    from compas_fab.robots import ToolState  # noqa: F401
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
        "RigidBody": RigidBody,
        "RigidBodyState": RigidBodyState,
        "ToolState": ToolState,
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


def get_or_load_robot_cell():
    """Return a cached `RobotCell`, loading URDF/SRDF + geometry on first call.

    Per-tool geometry and the assembly bodies are NOT attached here. They are
    registered by `rebuild_assembly_cell` (RSRebuildRobotCell): arm tools as
    `ToolModel`s and bars/joints/obstacles as canonical rigid bodies.
    """
    cached = _STICKY.get(_STICKY_ROBOT_CELL)
    if cached is not None:
        print("core.robot_cell.get_or_load_robot_cell: returning cached RobotCell.")
        return cached

    print("core.robot_cell.get_or_load_robot_cell: cold load (URDF + SRDF + geometry).")
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

    print(f"core.robot_cell: LocalPackageMeshLoader.load_urdf({config.HUSKY_URDF_FILENAME!r})")
    urdf_stream = main_loader.load_urdf(config.HUSKY_URDF_FILENAME)
    robot_model = deps["RobotModel"].from_urdf_string(urdf_stream.read())
    print("core.robot_cell: RobotModel.load_geometry(<4 mesh loaders>)")
    robot_model.load_geometry(main_loader, husky_loader, dualarm_husky_loader, ur_loader)

    srdf_path = main_loader.build_path(
        os.path.dirname(config.HUSKY_SRDF_REL_PATH),
        os.path.basename(config.HUSKY_SRDF_REL_PATH),
    )
    robot_semantics = deps["RobotSemantics"].from_srdf_file(srdf_path, robot_model)

    robot_cell = deps["RobotCell"](robot_model, robot_semantics)

    _STICKY[_STICKY_ROBOT_CELL] = robot_cell
    return robot_cell


def default_cell_state():
    """Return a fresh default `RobotCellState` for the cached robot cell."""
    robot_cell = get_or_load_robot_cell()
    return robot_cell.default_cell_state()


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
    # cannot cause a silent shadowing. Does nothing when our copy is
    # already the resolved one.
    _ensure_submodule_compas_fab_loaded(verbose=True)

    deps = _import_compas_stack()
    client = deps["PyBulletClient"](
        connection_type="gui" if use_gui else "direct",
        verbose=verbose,
    )
    print("core.robot_cell.start_pb_client: PyBulletClient.__enter__()")
    client.__enter__()
    planner = deps["PyBulletPlanner"](client)

    robot_cell = get_or_load_robot_cell()
    print("core.robot_cell.start_pb_client: planner.set_robot_cell(<dual-arm>)")
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
        print("core.robot_cell.stop_pb_client: pp.disconnect()")
        deps["pp"].disconnect()
    except Exception as exc:
        print(f"stop_pb_client: disconnect raised ({exc}); clearing sticky anyway.")
    finally:
        _STICKY.pop(_STICKY_PB_CLIENT, None)
        _STICKY.pop(_STICKY_PB_PLANNER, None)
        _STICKY.pop(_STICKY_ENV_GEOM, None)
        # Static-cell snapshot is tied to this PB session's registered bodies.
        _STICKY.pop(_STICKY_ASSEMBLY_SNAPSHOT, None)
        _STICKY.pop(_STICKY_ASSEMBLY_FINGERPRINT, None)
        _STICKY.pop(_STICKY_ASSEMBLY_STALE_ACK, None)
        # Bar/joint mesh + RigidBody caches built by core.env_collision are
        # tied to the PB client lifecycle -- clear them so the next RSPBStart
        # rebuilds against possibly-edited bar curves / re-exported joint OBJs.
        _STICKY.pop("bar_joint:env_bar_tube_mesh_cache", None)
        _STICKY.pop("bar_joint:env_joint_def_mesh_cache", None)
        _STICKY.pop("bar_joint:env_joint_rb_cache", None)
        _STICKY.pop("bar_joint:env_bar_rb_cache", None)
        _STICKY.pop("bar_joint:env_joint_obj_path_map", None)


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
        print("core.robot_cell.set_cell_state: cell-kind swap -> planner.set_robot_cell(<dual-arm>)")
        planner.set_robot_cell(get_or_load_robot_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "dual_arm"
    print("core.robot_cell.set_cell_state: planner.set_robot_cell_state(state) + pp.set_pose(robot_base)")
    planner.set_robot_cell_state(robot_cell_state)
    base_frame = robot_cell_state.robot_base_frame
    deps["pp"].set_pose(planner.client.robot_puid, _pose_from_frame(base_frame))


def _apply_base_frame_mm(state, base_frame_world_mm: np.ndarray):
    deps = _import_compas_stack()
    state.robot_base_frame = _mm_matrix_to_m_frame(deps["Frame"], base_frame_world_mm)


def _ensure_dual_arm_cell_loaded(planner):
    """Swap the planner's robot cell to the dual-arm cell if a different
    kind (e.g. the support cell from a prior RSIKSupportKeyframe run) is
    currently active. Symmetric counterpart to
    `core.robot_cell_support._ensure_support_cell_loaded`.
    """
    if _STICKY.get(_STICKY_CURRENT_CELL_KIND) != "dual_arm":
        planner.set_robot_cell(get_or_load_robot_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "dual_arm"


def ensure_env_registered(robot_cell, env_geom, planner):
    """Mirror ``env_geom`` into ``robot_cell.rigid_body_models`` and re-push
    the cell to the planner if anything changed.

    ``env_geom`` comes from ``core.env_collision.collect_built_geometry``.
    If the same geometry was registered last call, this does nothing (it
    does not call ``planner.set_robot_cell``).
    """
    from core import env_collision

    t_top = time.perf_counter()
    deps = _import_compas_stack()
    t_deps = time.perf_counter()
    changed = env_collision.register_env_in_robot_cell(robot_cell, env_geom, deps=deps)
    t_reg = time.perf_counter()
    if changed:
        print(
            f"core.robot_cell.ensure_env_registered: env changed "
            f"({len(env_geom)} bodies) -> planner.set_robot_cell(<dual-arm>)"
        )
        planner.set_robot_cell(robot_cell)
    else:
        print("core.robot_cell.ensure_env_registered: env unchanged; skipping planner.set_robot_cell")
    t_push = time.perf_counter()
    _STICKY[_STICKY_ENV_GEOM] = env_geom
    print(
        f"core.robot_cell.ensure_env_registered: timing "
        f"deps={ (t_deps-t_top)*1000:.1f}ms "
        f"register={ (t_reg-t_deps)*1000:.1f}ms "
        f"set_robot_cell={ (t_push-t_reg)*1000:.1f}ms "
        f"total={ (t_push-t_top)*1000:.1f}ms"
    )


def _set_random_dual_arm_config(planner, state):
    """Warm-start both arms from a fresh random configuration (for IK restarts).

    Mutates ``state.robot_configuration`` in place with random joint values for the
    left and right arm groups, using the planner's robot-cell sampler (the same one
    compas_fab's IK uses for its internal random restarts).

    Args:
        planner (PyBulletPlanner): active planner (its ``client.robot_cell`` samples).
        state (RobotCellState): state whose configuration is randomized in place.

    Returns:
        None.
    """
    rcell = planner.client.robot_cell
    for group in (config.LEFT_GROUP, config.RIGHT_GROUP):
        state.robot_configuration.merge(rcell.random_configuration(group))


def solve_dual_arm_ik(
    planner,
    template_state,
    base_frame_world_mm: np.ndarray,
    tool0_left_world_mm: np.ndarray,
    tool0_right_world_mm: np.ndarray,
    *,
    check_collision: bool = True,
    max_restart_iter: int = None,
    max_descend_iterations: int = None,
    tolerance_position: float = None,
    tolerance_orientation: float = None,
    verbose_pairs: bool = False,
):
    """Solve IK for the left then right group, warm-started from ``template_state``.

    Each attempt descends DETERMINISTICALLY (the underlying pybullet solver is run
    with ``max_results=1``) from a warm-start configuration. The outer loop here
    owns the random restarts: attempt 0 uses the caller's seed (``template_state``'s
    configuration), and every subsequent attempt resamples a random dual-arm config
    to descend from (see ``_set_random_dual_arm_config``).

    This split lets a chained caller warm-start ``M2``/``M3`` from the previous
    keyframe with ``max_restart_iter=1`` (only the given seed, no randomization),
    while a cold solve with no good seed (``M1``) uses the full restart budget --
    random restarts are the only reliable way into a narrow collision-free basin
    (pybullet's per-group internal restarts do not couple the two arms and fail to
    find tight dual-arm poses).

    Args:
        planner (PyBulletPlanner): active planner with the dual-arm cell loaded.
        template_state (RobotCellState): seed state; its ``robot_configuration`` is
            the warm-start for attempt 0.
        base_frame_world_mm (np.ndarray): 4x4 mm robot base frame (overrides the
            state's stored base).
        tool0_left_world_mm, tool0_right_world_mm (np.ndarray): 4x4 mm tool0 targets.
        check_collision (bool): collision-check each candidate.
        max_restart_iter (int): number of warm-start attempts; ``None`` ->
            ``config.IK_MAX_RESTART_ITER`` (cold default). Pass ``1`` to use only the
            given seed with no random restarts.
        max_descend_iterations, tolerance_position, tolerance_orientation: solver
            tuning; ``None`` -> the ``config.IK_*`` defaults.
        verbose_pairs (bool): print the collision-pair summary on success.

    Returns:
        RobotCellState | None: the solved state copy on success, or ``None`` if every
        restart fails.
    """
    _ensure_dual_arm_cell_loaded(planner)
    deps = _import_compas_stack()
    Frame = deps["Frame"]
    FrameTarget = deps["FrameTarget"]
    TargetMode = deps["TargetMode"]

    max_restart_iter = (
        config.IK_MAX_RESTART_ITER if max_restart_iter is None else max(1, int(max_restart_iter))
    )
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

    seed_state = template_state.copy()
    _apply_base_frame_mm(seed_state, base_frame_world_mm)

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
    targets = ((left_target, config.LEFT_GROUP), (right_target, config.RIGHT_GROUP))

    # max_results=1 -> deterministic descent from the warm-start; the outer loop
    # below supplies the random restarts.
    options = {
        "max_results": 1,
        "check_collision": check_collision,
        "max_descend_iterations": max_descend_iterations,
        "verbose": False,
    }

    print(
        f"core.robot_cell.solve_dual_arm_ik: check_collision={check_collision}, "
        f"max_restart_iter={max_restart_iter}"
    )
    last_failure = None
    for attempt in range(max_restart_iter):
        trial = seed_state.copy()
        if attempt > 0:
            # No good seed (or the given one failed): warm-start from a random config.
            _set_random_dual_arm_config(planner, trial)

        solved = True
        for target, group in targets:
            try:
                cfg = planner.inverse_kinematics(target, trial, group=group, options=options)
            except Exception as exc:  # reachability or collision (max_results=1 raises)
                last_failure = f"group '{group}': {exc}"
                solved = False
                break
            if cfg is None:
                last_failure = f"group '{group}': no solution"
                solved = False
                break
            trial.robot_configuration.merge(cfg)

        if solved:
            if attempt > 0:
                print(
                    f"core.robot_cell.solve_dual_arm_ik: solved on random restart "
                    f"{attempt}/{max_restart_iter - 1}."
                )
            if verbose_pairs:
                from core import env_collision
                print(env_collision.summarize_check_collision(planner, trial))
            return trial

    print(
        f"core.robot_cell.solve_dual_arm_ik: no solution after {max_restart_iter} "
        f"attempt(s) (last failure: {last_failure})."
    )
    return None


def extract_group_config(state, group: str, robot_cell) -> dict:
    """Return `{'joint_names': [...], 'joint_values': [...]}` for a group."""
    group_joint_names = list(robot_cell.get_configurable_joint_names(group))
    joint_values = [
        float(state.robot_configuration[name]) for name in group_joint_names
    ]
    return {"joint_names": group_joint_names, "joint_values": joint_values}


# ---------------------------------------------------------------------------
# Static assembly cell: ToolModels + canonical bar/joint/obstacle registry
# ---------------------------------------------------------------------------
#
# compas_fab model: `rigid_body_models` / `tool_models` are a STATIC geometry
# registry; per-step facts live in the RobotCellState. The assembly cell is
# (re)built explicitly by `rebuild_assembly_cell` (the RSRebuildRobotCell
# button); every per-command entry calls `ensure_assembly_cell`, which reuses
# the cached snapshot (building once on first use, warning if the document
# geometry changed since the last rebuild).

_ARM_TOOL_WRIST_TOUCH_LINKS = {
    "left": ["left_ur_arm_wrist_2_link", "left_ur_arm_wrist_3_link"],
    "right": ["right_ur_arm_wrist_2_link", "right_ur_arm_wrist_3_link"],
    # TODO to be safe, we start with only allow tool to touoch wrist 3 link
    # but indeed the assembly v3 tool is very close to be in collision with wrist 2, only 2 mm gap
    # "left": ["left_ur_arm_wrist_3_link"],
    # "right": ["right_ur_arm_wrist_3_link"],
}


def _arm_side_from_tool_name(tool_name):
    """Classify an arm side from a tool name's L/R suffix.

    Rhino-free duplicate of ``ik_collision_setup._arm_side_from_tool_name`` so
    this module stays headless-importable (that module imports rhinoscriptsyntax
    at top).

    Args:
        tool_name (str): e.g. ``"AT3L"`` / ``"AT3R"``.

    Returns:
        str | None: ``"left"`` for an 'L' suffix, ``"right"`` for 'R', else
        ``None``.
    """
    if not tool_name:
        return None
    last = str(tool_name).strip()[-1].upper()
    if last == "L":
        return "left"
    if last == "R":
        return "right"
    return None


def arm_tool_ids():
    """Map each arm side to its tool id from the registry.

    The tool id is the registry name (e.g. ``AT3L``/``AT3R``); side is the
    L/R name suffix. Headless-safe (reads ``robotic_tools.json`` only).

    Returns:
        dict: ``{"left": tool_id | None, "right": tool_id | None}``.
    """
    from core.robotic_tool import load_robotic_tools

    out = {"left": None, "right": None}
    for name in load_robotic_tools():
        side = _arm_side_from_tool_name(name)
        if side in out and out[side] is None:
            out[side] = name
    return out


def build_arm_tool_models():
    """Build the two arm-tool ``ToolModel``s straight from ``robotic_tools.json``.

    Picks the registry tool whose name ends in 'L' and the one ending in 'R'
    (exactly one of each is required), loads each tool's collision OBJ as the
    tool geometry, and sets ``tool_model.frame`` from the registry's
    ``M_tcp_from_block`` (which, despite its name, holds ``tool0_from_tcp`` --
    the TCP expressed in the flange/block frame; see `core.robotic_tool`).

    Headless-safe (no Rhino).

    Returns:
        dict: ``{tool_id: ToolModel}`` keyed by the registry tool name
        (e.g. ``AT3L`` / ``AT3R``).

    Raises:
        RuntimeError: if there isn't exactly one left + one right tool, or a
            tool's collision OBJ is missing.
    """
    from core.robotic_tool import load_robotic_tools

    deps = _import_compas_stack()
    Mesh = deps["Mesh"]
    ToolModel = deps["ToolModel"]
    Frame = deps["Frame"]
    Transformation = deps["Transformation"]

    tools = load_robotic_tools()
    by_side = {"left": [], "right": []}
    for name, tdef in tools.items():
        side = _arm_side_from_tool_name(name)
        if side in by_side:
            by_side[side].append(tdef)

    out = {}
    for side in ("left", "right"):
        defs = by_side[side]
        if len(defs) != 1:
            raise RuntimeError(
                f"build_arm_tool_models: expected exactly one {side}-arm tool "
                f"(name ending {side[0].upper()!r}); found {[d.name for d in defs]}. "
                "Define tools with RSDefineRoboticTool."
            )
        tdef = defs[0]
        col_path = tdef.collision_path()
        if not col_path or not os.path.isfile(col_path):
            raise RuntimeError(
                f"build_arm_tool_models: collision OBJ missing for tool "
                f"{tdef.name!r}: {col_path!r}."
            )
        mesh = Mesh.from_obj(col_path)
        mesh.scale(0.001)  # OBJ exported in mm -> meters
        # M_tcp_from_block == tool0_from_tcp (block baked at the flange). Use as-is.
        tcp_frame = _mm_matrix_to_m_frame(Frame, np.asarray(tdef.M_tcp_from_block, dtype=float))
        tool_model = ToolModel(None, tcp_frame, None, tdef.name)
        tool_model.add_link(
            "attached_tool_link", visual_meshes=[mesh], collision_meshes=[mesh]
        )
        tool_model._rebuild_tree()
        tool_model._create(tool_model.root, Transformation())
        print(
            f"core.robot_cell.build_arm_tool_models: built ToolModel {tdef.name!r} "
            f"({mesh.number_of_vertices()}v) for {side} arm"
        )
        out[tdef.name] = tool_model
    return out


def base_assembly_cell_state():
    """Default cell state with the arm ToolModels attached to their arm groups.

    Must be called after the tools are registered (i.e. after
    ``ensure_assembly_cell``). Every assembly state derives from this so the
    ``tool_states`` key-set matches ``tool_models`` (assert_cell_state_match).

    Returns:
        RobotCellState: the cell's default state with ``AT3L``/``AT3R``
        ``tool_states`` attached to the LEFT/RIGHT arm groups (wrist touch-links,
        identity attachment frame).
    """
    deps = _import_compas_stack()
    Frame = deps["Frame"]
    ToolState = deps["ToolState"]
    robot_cell = get_or_load_robot_cell()
    state = robot_cell.default_cell_state()

    arm_group = {"left": config.LEFT_GROUP, "right": config.RIGHT_GROUP}
    for tid in robot_cell.tool_models:
        side = _arm_side_from_tool_name(tid)
        if side is None:
            continue
        ts = state.tool_states.get(tid)
        if ts is None:
            ts = ToolState(frame=None)
            state.tool_states[tid] = ts
        ts.attached_to_group = arm_group[side]
        ts.touch_links = list(_ARM_TOOL_WRIST_TOUCH_LINKS[side])
        ts.attachment_frame = Frame.worldXY()
        ts.frame = None
        ts.is_hidden = False
    return state


def _live_assembly_fingerprint():
    """Cheap Rhino-side signal of assembly geometry identity (no RB/mesh build).

    Counts bars / joint instances / environment meshes and sums bar endpoint
    coordinates, so add/remove (counts) and move/resize (coord sum) both change
    it. Used only for the staleness warning. Rhino-only -> lazy imports.

    Returns:
        tuple: ``(n_bars, n_joint_instances, n_env_meshes, rounded_endpoint_sum)``.
    """
    import rhinoscriptsyntax as rs
    from core.rhino_bar_registry import get_bar_seq_map

    seq_map = get_bar_seq_map()
    coord_sum = 0.0
    for _bid, (oid, _seq) in seq_map.items():
        try:
            s = rs.CurveStartPoint(oid)
            e = rs.CurveEndPoint(oid)
            coord_sum += float(s.X + s.Y + s.Z + e.X + e.Y + e.Z)
        except Exception:
            pass
    n_joints = 0
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if rs.IsLayer(layer):
            n_joints += len(rs.ObjectsByLayer(layer) or [])
    n_env = (
        len(rs.ObjectsByLayer(config.LAYER_ENVIRONMENT) or [])
        if rs.IsLayer(config.LAYER_ENVIRONMENT)
        else 0
    )
    return (len(seq_map), n_joints, n_env, round(coord_sum, 3))


def rebuild_assembly_cell(robot_cell, planner):
    """Manual rebuild of the static assembly cell (the RSRebuildRobotCell button).

    Collects every bar + joint (canonical names) + environment obstacle, builds
    + registers the arm ToolModels, pushes one ``set_robot_cell``, and caches the
    full snapshot (bodies + world poses) + a cheap fingerprint in sticky.

    Args:
        robot_cell (RobotCell): the cached cell to populate (mutated in place).
        planner (PyBulletPlanner): active planner; receives one ``set_robot_cell``.

    Returns:
        dict: ``collision_bodies`` (``{name: body_info}``) for the full assembly
        + environment obstacles.
    """
    from core import env_collision
    from core.rhino_bar_registry import get_bar_seq_map

    seq_map = get_bar_seq_map()
    collision_bodies = dict(env_collision.collect_assembly_geometry(seq_map))
    collision_bodies.update(env_collision.collect_environment_geometry())

    # Tool models (geometry-only, per-arm).
    for tid, tm in build_arm_tool_models().items():
        robot_cell.tool_models[tid] = tm

    # Canonical bar/joint/obstacle rigid-body registry: replace the managed set.
    managed_prefixes = (
        env_collision.CANONICAL_BAR_PREFIX,
        env_collision.CANONICAL_JOINT_PREFIX,
        env_collision.OBSTACLE_PREFIX,
    )
    desired = {name: bi["rigid_body"] for name, bi in collision_bodies.items()}
    existing_managed = {
        n for n in list(robot_cell.rigid_body_models.keys())
        if n.startswith(managed_prefixes)
    }
    for stale in existing_managed - set(desired):
        robot_cell.rigid_body_models.pop(stale, None)
    for name, rb in desired.items():
        robot_cell.rigid_body_models[name] = rb

    print(
        f"core.robot_cell.rebuild_assembly_cell: {len(desired)} rigid bodies + "
        f"{len(robot_cell.tool_models)} tools -> planner.set_robot_cell(<dual-arm>)"
    )
    planner.set_robot_cell(robot_cell)

    _STICKY[_STICKY_ASSEMBLY_SNAPSHOT] = {
        "collision_bodies": collision_bodies,
        "tool_ids": sorted(robot_cell.tool_models.keys()),
    }
    try:
        _STICKY[_STICKY_ASSEMBLY_FINGERPRINT] = _live_assembly_fingerprint()
    except Exception as exc:
        print(f"core.robot_cell.rebuild_assembly_cell: fingerprint skipped ({exc}).")
        _STICKY[_STICKY_ASSEMBLY_FINGERPRINT] = None
    # Cell is fresh again -> clear any prior "proceed with stale cell" ack.
    _STICKY.pop(_STICKY_ASSEMBLY_STALE_ACK, None)
    return collision_bodies


def prompt_if_cell_stale(robot_cell, planner) -> bool:
    """Command-line prompt when the assembly geometry changed since the last
    ``RSRebuildRobotCell``. Call once at a command's entry point.

    On a stale cell, asks (on the Rhino command line, no popup) to **Rebuild**
    now / **Proceed** with the old cell / **Abort**. "Proceed" is remembered for
    this fingerprint so later commands don't re-prompt until geometry changes
    again.

    Args:
        robot_cell (RobotCell): the cached cell.
        planner (PyBulletPlanner): active planner (used if the user picks Rebuild).

    Returns:
        bool: ``True`` to proceed (in-sync, rebuilt, or user chose Proceed),
        ``False`` if the user aborted. Always ``True`` headless / when no cell is
        cached yet (``ensure_assembly_cell`` will build it fresh).
    """
    if _STICKY.get(_STICKY_ASSEMBLY_SNAPSHOT) is None:
        return True  # nothing cached yet; it will be built fresh on first use
    try:
        live_fp = _live_assembly_fingerprint()
    except Exception:
        return True  # can't read the document (headless) -> can't prompt
    if live_fp == _STICKY.get(_STICKY_ASSEMBLY_FINGERPRINT):
        return True  # in sync
    if live_fp == _STICKY.get(_STICKY_ASSEMBLY_STALE_ACK):
        return True  # user already chose to proceed with this exact stale state

    try:
        import rhinoscriptsyntax as rs
    except ImportError:
        print(
            "core.robot_cell.prompt_if_cell_stale: geometry changed since last "
            "RSRebuildRobotCell (proceeding with OLD cell; no Rhino to prompt)."
        )
        return True

    choice = rs.GetString(
        "Assembly geometry changed since the last RSRebuildRobotCell",
        "Rebuild",
        ["Rebuild", "Proceed", "Abort"],
    )
    if choice is None:
        print("Aborted -- run RSRebuildRobotCell, then re-run the command.")
        return False
    c = choice.strip().lower()
    if c.startswith("r"):
        rebuild_assembly_cell(robot_cell, planner)
        print("core.robot_cell.prompt_if_cell_stale: rebuilt the collision cell.")
        return True
    if c.startswith("p"):
        _STICKY[_STICKY_ASSEMBLY_STALE_ACK] = live_fp
        print("Proceeding with the OLD collision cell -- geometry edits are NOT reflected.")
        return True
    print("Aborted -- run RSRebuildRobotCell, then re-run the command.")
    return False


def ensure_assembly_cell(robot_cell, planner):
    """Return the cached assembly ``collision_bodies``; build once if absent.

    Per-command entry point. Does NOT re-scan geometry when a snapshot exists --
    it only compares a cheap fingerprint and prints a staleness warning if the
    document changed since the last ``RSRebuildRobotCell`` (warn only; never
    auto-rebuilds).

    Args:
        robot_cell (RobotCell): the cached cell.
        planner (PyBulletPlanner): active planner (used only if a build is needed).

    Returns:
        dict: the cached ``collision_bodies`` (``{name: body_info}``); built
        fresh via ``rebuild_assembly_cell`` if no snapshot exists yet.
    """
    snapshot = _STICKY.get(_STICKY_ASSEMBLY_SNAPSHOT)
    if snapshot is None:
        print(
            "core.robot_cell.ensure_assembly_cell: no cached cell; building once "
            "(run RSRebuildRobotCell after geometry edits to refresh)."
        )
        return rebuild_assembly_cell(robot_cell, planner)
    try:
        live_fp = _live_assembly_fingerprint()
        stale = live_fp != _STICKY.get(_STICKY_ASSEMBLY_FINGERPRINT)
        acked = live_fp == _STICKY.get(_STICKY_ASSEMBLY_STALE_ACK)
        # `prompt_if_cell_stale` (command entry) handles the interactive case;
        # only warn here if it wasn't run / acknowledged (e.g. direct API use).
        if stale and not acked:
            print(
                "core.robot_cell.ensure_assembly_cell: WARNING -- assembly geometry "
                "changed since the last RSRebuildRobotCell. Collision results use the "
                "OLD cell until you click RSRebuildRobotCell."
            )
    except Exception as exc:
        print(f"core.robot_cell.ensure_assembly_cell: staleness check skipped ({exc}).")
    return snapshot["collision_bodies"]
