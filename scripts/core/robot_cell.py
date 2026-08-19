"""Dual-arm Husky robot cell loading, PyBullet lifecycle, and IK helpers.

Adapted from the Grasshopper prototype scripts under
`support_materials/gh_keyframe_demos/python/`:

- `GH_create_robot_model.py` + `GH_create_robot_cell.py` → `get_or_load_robot_cell`
- `GH_init_pb.py` → `start_pb_sessions`
- `GH_disconnect_pb.py` → `stop_pb_client`
- `GH_set_cell_state.py` → `set_cell_state`

Multi-robot model: ONE persistent PyBullet session per robot (Cindy the
dual-arm assembly robot + the Alice/Belle support robots), all cached in
sticky under `_STICKY_PB_SESSIONS`. Each planner permanently owns its one
robot's cell; the OTHER robots appear in a cell only as frozen articulated
ToolModel obstacles (see `core.robot_obstacles`). Never use the
pybullet_planning `pp.*` helpers here — they bind a module-global client id
and silently talk to the wrong session in a multi-client process; use raw
`pybullet.X(..., physicsClientId=client.client_id)` instead.

The dual-arm IK solvers themselves (`solve_dual_arm_ik` and friends) moved to
`husky_assembly_tamp.keyframe.dual_arm_ik` so the offline planner and Rhino
share one implementation; this module keeps the Rhino session concerns (sticky
cache, PyBullet lifecycle, assembly-cell building).

compas_fab consumes SI units (meters, radians). Public 4x4 transforms here
are mm (matching `core.config`). Unit conversion happens at the compas
boundary inside this module.
"""

from __future__ import annotations

import hashlib
import os
import sys
import time

import numpy as np

from core import config
# Tool registry: the L/R naming rule + the active candidate pair. Rhino-free,
# so importing at the top keeps this module headless-importable.
from core.robotic_tool import (
    arm_side_from_tool_name,
    get_active_pair,
    get_active_pair_names,
)


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
# One persistent PyBullet session PER ROBOT, all living in one sticky dict:
#   {robot_name: {"client": PyBulletClient, "planner": PyBulletPlanner,
#                 "cell_loaded": bool}}
# Cindy (assembly) gets her dual-arm cell pushed at start; the support robots'
# cells are built + pushed lazily on their first use ("cell_loaded" tracks
# that). Each planner permanently owns ONE robot's cell -- there is no cell
# swapping anymore.
_STICKY_PB_SESSIONS = "bar_joint:pb_sessions"
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
# Shared helpers from the tamp keyframe package
# ---------------------------------------------------------------------------
# The lazy compas-stack loader, the mm<->m unit converters, the IK solvers, and
# the ssik sidecar client all live in `husky_assembly_tamp.keyframe` now (one
# definition, shared with the offline planner). `core.config` already put the
# submodule on sys.path. These imports are cheap: neither module pulls the
# compas / pybullet stack at import time.
from husky_assembly_tamp.keyframe import ssik_client as _tamp_ssik_client  # noqa: E402
from husky_assembly_tamp.keyframe.dual_arm_ik import (  # noqa: E402
    apply_base_frame_mm,
    frame_to_m_matrix,
    import_compas_stack,
    mm_matrix_to_m_frame,
)

# Backwards-compatible private aliases (Rhino call sites use these names).
_import_compas_stack = import_compas_stack
_mm_matrix_to_m_frame = mm_matrix_to_m_frame
_frame_to_m_matrix = frame_to_m_matrix
_apply_base_frame_mm = apply_base_frame_mm


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
        # print("core.robot_cell.get_or_load_robot_cell: returning cached RobotCell.")
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


def start_pb_sessions(use_gui: bool = False, verbose: bool = True):
    """Start one PyBullet client per robot and cache all sessions in sticky.

    Three persistent sessions come up: Cindy (assembly, gui or direct per
    ``use_gui``) plus one DIRECT session per support robot (PyBullet allows
    only one GUI window per process, so Alice/Belle are always headless).
    Cindy's dual-arm cell is pushed into her planner right away; the support
    cells are built + pushed lazily on their first support-flow use (see
    ``core.robot_cell_support.ensure_support_cell_pushed``) so startup does
    not pay for URDFs that may never be needed.

    Args:
        use_gui (bool): Open the GUI window for Cindy's session only.
        verbose (bool): Keep True inside Rhino. Setting it to False triggers
            `compas_fab.backends.pybullet.utils.redirect_stdout`, which calls
            `os.dup(fd)` on stdout — that raises inside Rhino's ScriptEditor
            (Rhino's stdout is not a normal OS file descriptor).

    Returns:
        tuple: Cindy's ``(client, planner)`` (the assembly session).
    """
    if _STICKY.get(_STICKY_PB_SESSIONS):
        raise RuntimeError(
            "PyBullet sessions already running (or partially running). "
            "Call stop_pb_client() / RSPBStop first."
        )

    # Defensive: re-run the submodule check every RSPBStart, so a stale
    # sys.modules entry from a sibling repo cannot cause a silent shadowing.
    _ensure_submodule_compas_fab_loaded(verbose=True)

    deps = _import_compas_stack()
    sessions = {}

    # ---- Cindy (assembly robot) first, so the one allowed GUI is hers.
    cindy = config.ASSEMBLY_ROBOT_NAME
    client = deps["PyBulletClient"](
        connection_type="gui" if use_gui else "direct",
        verbose=verbose,
    )
    print(f"core.robot_cell.start_pb_sessions: {cindy} PyBulletClient.__enter__()")
    client.__enter__()
    planner = deps["PyBulletPlanner"](client)
    robot_cell = get_or_load_robot_cell()
    print(f"core.robot_cell.start_pb_sessions: planner.set_robot_cell(<dual-arm>) for {cindy}")
    planner.set_robot_cell(robot_cell)
    sessions[cindy] = {"client": client, "planner": planner, "cell_loaded": True}

    # ---- Support robots: always DIRECT, cells lazy.
    for name in config.SUPPORT_ROBOT_NAMES:
        sr_client = deps["PyBulletClient"](connection_type="direct", verbose=verbose)
        print(f"core.robot_cell.start_pb_sessions: {name} PyBulletClient.__enter__() (direct)")
        sr_client.__enter__()
        sr_planner = deps["PyBulletPlanner"](sr_client)
        sessions[name] = {"client": sr_client, "planner": sr_planner, "cell_loaded": False}

    _STICKY[_STICKY_PB_SESSIONS] = sessions
    ids = {name: s["client"].client_id for name, s in sessions.items()}
    print(f"core.robot_cell.start_pb_sessions: client ids {ids}")
    return sessions[cindy]["client"], sessions[cindy]["planner"]


def stop_pb_client():
    """Disconnect ALL PyBullet sessions; always clear sticky, even on failure.

    Robust against sessions that already died (user closed the GUI window, or
    a prior stop partially succeeded): each disconnect error is swallowed and
    the sticky cache is cleared regardless, so the next RSPBStart sees a
    clean slate.
    """
    # Shut the ssik sidecar (if the "ssik" backend spawned one) down first, so a
    # stray Python 3.11 process never outlives the PyBullet sessions.
    try:
        _tamp_ssik_client.shutdown()
    except Exception as exc:
        print(f"stop_pb_client: ssik sidecar shutdown raised ({exc}); continuing.")

    try:
        sessions = _STICKY.get(_STICKY_PB_SESSIONS) or {}
        # Per-client disconnects (NOT the global pp.disconnect(), which only
        # knows one module-global client id and would miss the other two).
        for name, session in sessions.items():
            try:
                print(f"core.robot_cell.stop_pb_client: {name} client.disconnect()")
                # verbose=True on purpose: verbose=False routes through
                # compas_fab's redirect_stdout, whose os.dup(fd) raises inside
                # Rhino BEFORE pybullet.disconnect runs — leaking the connection.
                session["client"].disconnect(verbose=True)
            except Exception as exc:
                print(f"stop_pb_client: {name} disconnect raised ({exc}); continuing.")
    finally:
        _STICKY.pop(_STICKY_PB_SESSIONS, None)
        _STICKY.pop(_STICKY_ENV_GEOM, None)
        # Per-robot caches tied to the cells that were loaded into those
        # sessions: the support cells and the frozen-robot obstacle ToolModels
        # (prefix-keyed, see core.robot_obstacles / core.robot_cell_support).
        # Dropped here so RSPBStop + RSPBStart genuinely rebuilds them -- a
        # stale obstacle model would otherwise outlive an edit to how it is
        # built and quietly keep the old geometry.
        for key in [
            k for k in list(_STICKY.keys())
            if isinstance(k, str) and k.startswith((
                "bar_joint:obstacle_tool_model:",
                "bar_joint:support_cell:",
            ))
        ]:
            _STICKY.pop(key, None)
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


def get_session(robot_name: str = None):
    """Return one robot's cached `(client, planner)` pair, or raise.

    Args:
        robot_name (str): "Cindy", "Alice", or "Belle". None means Cindy.

    Raises:
        RuntimeError: if sessions were never started, the name is unknown,
            or that robot's session died (e.g. the GUI window was closed).
    """
    name = robot_name or config.ASSEMBLY_ROBOT_NAME
    sessions = _STICKY.get(_STICKY_PB_SESSIONS)
    if not sessions:
        raise RuntimeError("PyBullet sessions not started. Run RSPBStart first.")
    if name not in config.ROBOT_IDS:
        raise RuntimeError(
            f"Unknown robot name {name!r}. Known robots: {sorted(config.ROBOT_IDS)}."
        )
    session = sessions.get(name)
    if session is None or not _pybullet_connected(session["client"]):
        raise RuntimeError(
            f"PyBullet session for {name} is not alive (GUI closed or crashed). "
            "Run RSPBStop, then RSPBStart to restart all sessions."
        )
    return session["client"], session["planner"]


def get_planner():
    """Cindy's cached `(client, planner)` tuple, or raise if not started.

    Kept under its old name so the many assembly-side call sites keep working;
    support-robot code uses ``get_session(robot_name)`` instead.
    """
    if not is_pb_running():
        raise RuntimeError("PyBullet client not started. Run RSPBStart first.")
    return get_session(config.ASSEMBLY_ROBOT_NAME)


def _pybullet_connected(client) -> bool:
    """Whether ONE client's connection is alive, scoped by its physics client id.

    Deliberately raw ``pybullet.getConnectionInfo`` and not
    ``pybullet_planning.is_connected`` — the pp helpers read a module-global
    client id and answer for the wrong session in a multi-client process.

    Args:
        client: a compas_fab PyBulletClient.
    """
    try:
        import pybullet
        info = pybullet.getConnectionInfo(physicsClientId=client.client_id)
        return bool(info.get("isConnected"))
    except Exception:
        return False


def is_pb_running(robot_name: str = None) -> bool:
    """Authoritative "is this robot's PB session usable right now?" check.

    ``robot_name=None`` asks about Cindy (the assembly session), matching the
    many existing call sites. Cross-references the sticky session registry
    against PyBullet's own per-client connection state and self-heals:

    - session cached + connected  → True
    - session cached + disconnected (user closed GUI) → drop the dead session,
      return False. If EVERY session is dead the whole registry is cleared so
      the next RSPBStart starts clean.
    - no sessions cached          → False
    """
    name = robot_name or config.ASSEMBLY_ROBOT_NAME
    sessions = _STICKY.get(_STICKY_PB_SESSIONS)
    if not sessions or name not in sessions:
        return False
    if _pybullet_connected(sessions[name]["client"]):
        return True
    # Sticky thinks this session runs but PyBullet says no — stale; drop it.
    sessions.pop(name, None)
    print(f"is_pb_running: stale session for {name} (PyBullet no longer connected); dropped it.")
    if not any(_pybullet_connected(s["client"]) for s in sessions.values()):
        _STICKY.pop(_STICKY_PB_SESSIONS, None)
        print("is_pb_running: no live sessions remain; cleared the session registry.")
    return False


# ---------------------------------------------------------------------------
# Cell state / IK
# ---------------------------------------------------------------------------


def set_cell_state(planner, robot_cell_state):
    """Push a robot cell state into PyBullet.

    Each planner permanently owns one robot's cell now (no cell-kind
    swapping), so this is a plain state push. The robot base pose is part of
    the state: compas_fab's ``set_robot_cell_state`` pushes it per-client via
    ``client._set_base_frame`` — the old extra ``pp.set_pose`` call here was
    redundant AND unsafe with multiple clients (pp helpers read a
    module-global client id).
    """
    print("core.robot_cell.set_cell_state: planner.set_robot_cell_state(state)")
    planner.set_robot_cell_state(robot_cell_state)


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


# ---------------------------------------------------------------------------
# Dual-arm IK solvers -> moved to husky_assembly_tamp.keyframe.dual_arm_ik
# ---------------------------------------------------------------------------
# solve_dual_arm_ik / solve_dual_arm_ik_ssik / enumerate_ssik_candidate_pairs /
# extract_group_config and their helpers live in the tamp submodule now, so the
# offline planner and Rhino share ONE implementation. This module keeps only the
# Rhino session concerns: the sticky cache, the PyBullet lifecycle, and the
# static assembly-cell building below. Callers import the solvers directly:
#   from husky_assembly_tamp.keyframe import dual_arm_ik


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


def arm_tool_ids():
    """Map each arm side to the ACTIVE pair's tool id.

    The tool id is the registry name (e.g. ``AT3L``/``AT3R``); the active
    pair comes from the registry's ``active`` entry. Headless-safe (reads
    ``robotic_tools.json`` only).

    Returns:
        dict: ``{"left": tool_id, "right": tool_id}``.

    Raises:
        RuntimeError: if the active pair cannot be resolved (empty registry,
            missing/invalid ``active`` entry) -- run RSSwapRoboticTool.
    """
    return get_active_pair_names()


def build_arm_tool_models():
    """Build the two arm-tool ``ToolModel``s straight from ``robotic_tools.json``.

    Uses the registry's ACTIVE candidate pair (the ``active`` entry, set by
    RSSwapRoboticTool), loads each tool's collision OBJ as the tool geometry,
    and sets ``tool_model.frame`` from the registry's ``M_tcp_from_block``
    (which, despite its name, holds ``tool0_from_tcp`` -- the TCP expressed
    in the flange/block frame; see `core.robotic_tool`).

    Headless-safe (no Rhino).

    Returns:
        dict: ``{tool_id: ToolModel}`` keyed by the registry tool name
        (e.g. ``AT3L`` / ``AT3R``).

    Raises:
        RuntimeError: if the active pair cannot be resolved, or a tool's
            collision OBJ is missing.
    """
    deps = _import_compas_stack()
    Mesh = deps["Mesh"]
    ToolModel = deps["ToolModel"]
    Frame = deps["Frame"]
    Transformation = deps["Transformation"]

    pair = get_active_pair()

    out = {}
    for side in ("left", "right"):
        tdef = pair[side]
        col_path = tdef.collision_path()
        if not col_path or not os.path.isfile(col_path):
            raise RuntimeError(
                f"build_arm_tool_models: collision OBJ missing for tool "
                f"{tdef.name!r}: {col_path!r}. Re-run RSDefineRoboticTool "
                f"(AssemblyTool mode) for {tdef.name!r} to export it."
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


def ensure_arm_tool_models(robot_cell):
    """Register the active pair's arm ToolModels on *robot_cell* if absent.

    PyBullet-free subset of what ``rebuild_assembly_cell`` does for tools:
    viewers (the Grasshopper preview, its warm-up) call this so the arms render
    WITH their assembly tools without requiring RSIKKeyframe /
    RSRebuildRobotCell to have run first.  Idempotent: does nothing when a
    non-obstacle tool is already registered.

    Returns:
        bool: True when the tools were (re)built and added, False when they
        were already present.

    Raises:
        RuntimeError: from ``build_arm_tool_models`` when the registry's active
            pair cannot be resolved or a collision OBJ export is missing.
    """
    existing = getattr(robot_cell, "tool_models", None) or {}
    obstacle_names = set(config.OBSTACLE_TOOL_NAMES.values())
    if any(tid not in obstacle_names for tid in existing):
        return False
    for tid, tm in build_arm_tool_models().items():
        robot_cell.tool_models[tid] = tm
    return True


def base_assembly_cell_state():
    """Default cell state with the arm ToolModels attached to their arm groups.

    Must be called after the tools are registered (i.e. after
    ``ensure_assembly_cell``). Every assembly state derives from this so the
    ``tool_states`` key-set matches ``tool_models`` (assert_cell_state_match).

    Returns:
        RobotCellState: the cell's default state with the active pair's
        ``tool_states`` attached to the LEFT/RIGHT arm groups (wrist touch-links,
        identity attachment frame).
    """
    deps = _import_compas_stack()
    Frame = deps["Frame"]
    ToolState = deps["ToolState"]
    robot_cell = get_or_load_robot_cell()
    state = robot_cell.default_cell_state()

    arm_group = {"left": config.LEFT_GROUP, "right": config.RIGHT_GROUP}
    obstacle_names = set(config.OBSTACLE_TOOL_NAMES.values())
    for tid in robot_cell.tool_models:
        if tid in obstacle_names:
            continue  # frozen-robot obstacles are handled below, not arm tools
        side = arm_side_from_tool_name(tid)
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

    # ! Support robots default to PARKED (far away, collisions on). Any state
    # ! that wants a support robot IN the scene must call
    # ! robot_obstacles.configure_robot_obstacle on top of this base state.
    # Local import: robot_obstacles imports from this module (circular at top).
    from core import robot_obstacles
    for robot_name, tool_name in config.OBSTACLE_TOOL_NAMES.items():
        if tool_name in robot_cell.tool_models:
            robot_obstacles.park_robot_obstacle(state, robot_name)
    return state


def _live_assembly_fingerprint():
    """Cheap Rhino-side signal of assembly geometry identity (no RB/mesh build).

    Counts bars / joint instances / environment meshes and sums bar endpoint
    coordinates, so add/remove (counts) and move/resize (coord sum) both change
    it. The fingerprint also includes the ACTIVE tool-pair names. If
    RSSwapRoboticTool changes that pair without rebuilding the collision cell,
    the next IK command detects the mismatch and prompts the user to Rebuild,
    Proceed with the old cell, or Abort. If the cell was rebuilt during the
    swap, its fingerprint is already current and no prompt appears. Used only
    for the staleness (outdate) warning. Rhino-only -> lazy imports.

    It also carries a NAMES hash: an md5 over the sorted bar ids plus every
    joint block's (joint_id, subtype, parent_bar_id) user text -- the exact
    inputs the canonical collision-body names are built from. Renaming or
    renumbering bars/joints (RSReorderBarID, relink, hand edits) changes no
    count and no coordinate, so without this hash the cached cell would keep
    serving phantom body names after a rename with no warning at all.

    Returns:
        tuple: ``(n_bars, n_joint_instances, n_env_meshes,
        rounded_endpoint_sum, (left_tool, right_tool), names_md5)``.
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
    # The naming inputs of `env_collision.collect_assembly_geometry`: bar ids
    # plus each joint block's id / subtype-or-type / parent bar. Plain user-text
    # reads, no meshes -- cheap enough for the every-command staleness probe.
    # The FAKE mark rides along because a fake bar (and its joint halves) is
    # dropped from the collision scene, so toggling it changes the scene while
    # changing no count and no coordinate.
    from core.rhino_bar_registry import get_fake_bar_ids

    fake_bar_ids = get_fake_bar_ids(seq_map)
    name_parts = sorted(
        f"{bid}{':fake' if bid in fake_bar_ids else ''}" for bid in seq_map
    )
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if rs.IsLayer(layer):
            joint_oids = rs.ObjectsByLayer(layer) or []
            n_joints += len(joint_oids)
            for joint_oid in joint_oids:
                jid = rs.GetUserText(joint_oid, "joint_id") or ""
                subtype = (
                    rs.GetUserText(joint_oid, "joint_subtype")
                    or rs.GetUserText(joint_oid, "joint_type")
                    or ""
                )
                parent = rs.GetUserText(joint_oid, "parent_bar_id") or ""
                name_parts.append(f"{jid}:{subtype}:{parent}")
    n_env = (
        len(rs.ObjectsByLayer(config.LAYER_ENVIRONMENT) or [])
        if rs.IsLayer(config.LAYER_ENVIRONMENT)
        else 0
    )
    # Active tool identity: a tool swap changes the cell geometry just like a
    # bar edit does, so it must change the fingerprint too. The staleness probe
    # runs at every command entry, so an unresolvable pair only gets a
    # placeholder here -- the loud error surfaces in build_arm_tool_models /
    # arm_tool_ids, which every real consumer hits.
    try:
        names = get_active_pair_names()
        active_sig = (names["left"], names["right"])
    except Exception:
        active_sig = ("<unresolved>", "<unresolved>")
    # Deterministic digest of the sorted naming inputs (see docstring). Sorted so
    # document order / layer scan order cannot flip the fingerprint.
    names_md5 = hashlib.md5("|".join(sorted(name_parts)).encode("utf-8")).hexdigest()
    return (len(seq_map), n_joints, n_env, round(coord_sum, 3), active_sig, names_md5)


def rebuild_assembly_cell(robot_cell, planner):
    """Manual rebuild of the static assembly cell (the RSRebuildRobotCell button).

    Collects every bar + joint (canonical names) + environment obstacle, builds
    + registers the arm ToolModels, then calls ``planner.set_robot_cell`` once 
    to load the fully-populated cell into PyBullet in a single call (that
    call rebuilds the whole PyBullet scene, so we register everything first and
    load once rather than re-loading per body), and caches the full snapshot
    (bodies + world poses) + a cheap geometry fingerprint in sticky.

    The fingerprint (bar/joint/env counts + a sum of bar endpoints, from
    ``_live_assembly_fingerprint``) is a lightweight signal of the document's
    geometry. Later commands recompute it and compare against this stored value
    to detect that the cached cell is stale (geometry edited since the rebuild)
    without paying for a full rebuild -- see ``prompt_if_cell_stale`` /
    ``ensure_assembly_cell``.

    Args:
        robot_cell (RobotCell): the cached cell to populate (mutated in place).
        planner (PyBulletPlanner): active planner; its ``set_robot_cell`` is
            called exactly once, after all rigid bodies + tools are registered.

    Returns:
        dict: ``collision_bodies`` -- ``{name: body_info}`` for the full assembly
        + environment obstacles. Each ``body_info`` carries the world pose +
        metadata the RobotCell itself does NOT store: ``frame_world_mm`` (4x4 mm
        world pose), ``kind`` (``"bar"`` / ``"joint"`` / ``"environment"``),
        ``parent_bar_id`` (bars/joints only), and ``rigid_body`` (the RigidBody
        geometry -- the only field mirrored into ``robot_cell.rigid_body_models``).
        ``core.ik_collision_setup.build_full_assembly_state`` consumes this to
        stamp one ``RigidBodyState`` (world frame + ``is_hidden``) per body into
        each per-step ``RobotCellState`` -- placement info that cannot be
        recovered from the cell alone.
    """
    from core import env_collision
    from core.rhino_bar_registry import get_bar_seq_map

    seq_map = get_bar_seq_map()
    collision_bodies = dict(env_collision.collect_assembly_geometry(seq_map))
    collision_bodies.update(env_collision.collect_environment_geometry())

    # Tool models (geometry-only, per-arm). Rebuild the ACTIVE pair's tools, then
    # evict any tool that is no longer in the active pair. Without the eviction a
    # tool swap (which changes the tool NAMES, e.g. AT3_E1L -> AT4_E3_L) leaves the
    # OLD ToolModels attached to the arms alongside the new ones, so PyBullet
    # collides against BOTH and the preview draws BOTH. Mirrors the rigid-body
    # eviction below. The frozen-robot obstacle tools are NOT part of the
    # active pair -- exempt them from eviction.
    new_tools = build_arm_tool_models()
    obstacle_names = set(config.OBSTACLE_TOOL_NAMES.values())
    stale_tools = set(robot_cell.tool_models.keys()) - set(new_tools) - obstacle_names
    for stale_tool in stale_tools:
        robot_cell.tool_models.pop(stale_tool, None)
    for tid, tm in new_tools.items():
        robot_cell.tool_models[tid] = tm
    if stale_tools:
        print(
            f"core.robot_cell.rebuild_assembly_cell: evicted stale tool(s) "
            f"{sorted(stale_tools)} (no longer in the active pair)."
        )

    # ! Register the two support robots as frozen articulated obstacles in
    # ! Cindy's cell, so a robot holding a bar shows up (and collides) in every
    # ! later assembly step's planning scene. Their per-step pose/configuration
    # ! is set on the STATE (configure_robot_obstacle / park_robot_obstacle);
    # ! here we only register the geometry once.
    # Local import: robot_obstacles imports from this module (circular at top).
    from core import robot_obstacles
    robot_obstacles.attach_obstacle_robot_tools(robot_cell, list(config.SUPPORT_ROBOT_NAMES))

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
        dict: the cached ``collision_bodies`` -- ``{name: body_info}`` with a
        world pose + ``kind`` + ``parent_bar_id`` + ``rigid_body`` per body (see
        ``rebuild_assembly_cell``). Fed to
        ``ik_collision_setup.build_full_assembly_state`` to build each per-step
        ``RobotCellState``. Built fresh via ``rebuild_assembly_cell`` if no
        snapshot exists yet.
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
