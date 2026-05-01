"""Probe whether two `compas_fab.PyBulletClient` instances stay isolated
in the same Python process, before we commit to the "two persistent
planners" refactor that would replace the current cell-swap design.

WHY this exists
---------------
Today RSIKKeyframe and RSIKSupportKeyframe share ONE PyBulletClient +
ONE PyBulletPlanner. Switching cells calls `planner.set_robot_cell(...)`
which re-loads URDFs + rebuilds collision geometry — slow. The
proposed refactor: two persistent planners, one per cell, with no
swap. This script verifies the prerequisite: that two clients
isolate cleanly and stay stable headless for long runs.

Findings already established (see `tasks/cc_lessons.md`):

* `compas_fab.PyBulletClient` IS per-client clean — every `pybullet.*`
  call passes `physicsClientId=self.client_id`. `client.robot_puid`,
  `client.tools_puids`, `client.disabled_collisions` are instance state.
  `PyBulletPlanner` only holds `self._client`.
  `client._set_base_frame` (client.py:702-704) already scopes the base
  push correctly, so `planner.set_robot_cell_state(state)` is sufficient
  to push BOTH joint state AND base frame on the right client.
* `pybullet_planning` has a module-global `CLIENT` footgun. The
  following helpers all bind to that global:
    - `pp.set_pose(body, pose)`            — no `client=` kwarg
    - `pp.get_pose(body)`                  — no `client=` kwarg
    - `pp.disconnect()`                    — no `client=` kwarg
    - `pp.is_connected()`                  — no `client=` kwarg
    - `pp.link_from_name(body, name)`      — no `client=` kwarg
    - `pp.get_link_pose(body, link)`       — no `client=` kwarg
  Only `pp.clone_body(..., client=...)` accepts an explicit client.
* Footgun callsites in our code (NOT touched by this script — see
  REFACTOR NOTES at the bottom for the future fix):
    - `scripts/core/robot_cell.py:469`        — `pp.set_pose(...)` (also redundant — compas_fab already pushes base via `_set_base_frame`)
    - `scripts/core/robot_cell_support.py:247`— same
    - `scripts/core/robot_cell.py:423`        — `pp.is_connected()`
    - `scripts/core/robot_cell.py:396`        — `pp.disconnect()`
    - `scripts/rs_show_ik.py:230-231`         — `pp.link_from_name`, `pp.get_link_pose`
* PyBullet allows ONE GUI window per process. Two `connection_type="gui"`
  in the same process is forbidden.

This script sidesteps every footgun by using ONLY raw `pybullet.*`
calls (with `physicsClientId=client.client_id`) and `planner.set_robot_cell_state(state)`.
We never invoke `core.robot_cell.set_cell_state` or any pp helper.

USAGE
-----

Run under Rhino's bundled Python 3.9 (the site-envs we bootstrap contain
native modules — cryptography in particular — built for py3.9; loading
them under py3.10+ raises the PyO3 once-per-process init error). On
Windows that is:

    ~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py [args]

    Args:
        [--gui {a,b,none}]
        [--capture <tests/captures/*_support_*.json>]
        [--stability-iterations N] [--stop-on-error]
        [--seed S] [--verbose]

Two passing recipes (verified 2026-04-30, mean 0.05ms / p95 0.06ms,
0 failures over 5000 iterations):

    ~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --gui none
    ~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --stability-iterations 5000 --stop-on-error

A passing run of both is the green light to proceed with the refactor.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import statistics
import sys
import time


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (SCRIPTS_DIR, TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)


from _rhino_env_bootstrap import bootstrap_rhino_site_envs  # noqa: E402

bootstrap_rhino_site_envs()


import numpy as np  # noqa: E402  (deferred until site-envs are wired)
import pybullet_planning as pp  # noqa: E402

from contextlib import contextmanager  # noqa: E402


@contextmanager
def pp_active_client_BROKEN(client):
    """The naive (and BROKEN) attempt: just call `pp.set_client`. This does
    NOT actually route most pp.* helpers to the chosen client.

    Reason: `pybullet_planning` submodules like
    `pybullet_planning.interfaces.env_manager.pose_transformation` do
    `from pybullet_planning.utils import CLIENT, ...` — that creates a
    SEPARATE binding `pose_transformation.CLIENT` at import time. The
    official `pp.set_client(...)` mutates `shared_const.CLIENT` but not
    the per-submodule rebinds. So `pp.set_pose`, `pp.get_pose`, etc.
    keep reading their stale 0.

    Kept here purely for the test that PROVES this brokenness.
    """
    prev = pp.get_client()
    pp.set_client(client.client_id)
    try:
        yield
    finally:
        pp.set_client(prev)


def _enumerate_pp_client_modules():
    """Walk the pybullet_planning package and return every submodule that
    holds its own `CLIENT` attribute (= `from ...utils import CLIENT`
    rebinds). Cached at module load time.
    """
    import importlib
    import pkgutil
    found = []
    visited = set()
    queue = [pp]
    while queue:
        mod = queue.pop()
        if mod.__name__ in visited:
            continue
        visited.add(mod.__name__)
        if hasattr(mod, "__path__"):
            for _, sub_name, _ in pkgutil.iter_modules(mod.__path__, prefix=mod.__name__ + "."):
                try:
                    submod = importlib.import_module(sub_name)
                except Exception:
                    continue
                queue.append(submod)
        if hasattr(mod, "CLIENT") and isinstance(mod.CLIENT, int):
            found.append(mod)
    return found


_PP_CLIENT_MODULES = _enumerate_pp_client_modules()


@contextmanager
def pp_active_client(client):
    """Route pp.* helpers to ``client.client_id`` for the duration of the
    block. Patches `CLIENT` in EVERY pybullet_planning submodule that
    holds its own rebinding (because plain `pp.set_client` only updates
    `shared_const.CLIENT`, and most pp helpers read a stale local copy).
    Restores all bindings on exit.

    Trade-off: this is the only way to make pp helpers behave in a
    multi-client process today, but it depends on the brittle assumption
    that every pp submodule worth scoping has been imported by the time
    `_enumerate_pp_client_modules()` runs. Lazy imports inside individual
    helpers can still slip past it. For production, prefer raw
    `pybullet.*(physicsClientId=client.client_id)` calls (per-client
    scoping is enforced at the call site, not via global state).
    """
    target = client.client_id
    saved = [(m, m.CLIENT) for m in _PP_CLIENT_MODULES]
    pp.set_client(target)
    for m in _PP_CLIENT_MODULES:
        try:
            m.CLIENT = target
        except Exception:
            pass
    try:
        yield
    finally:
        for m, prev in saved:
            try:
                m.CLIENT = prev
            except Exception:
                pass
        # Restore pp.set_client global to whatever shared_const had originally.
        if saved:
            pp.set_client(saved[0][1])


# ---------------------------------------------------------------------------
# Small utilities (copied from `tests/debug_ik_support_collisions.py` —
# the sibling debug scripts are intentionally self-contained).
# ---------------------------------------------------------------------------


def _decode_or_none(b):
    if b is None:
        return None
    if isinstance(b, bytes):
        return b.decode("utf-8", errors="replace")
    return str(b)


def _body_label(p, body_id, client_id):
    info = p.getBodyInfo(body_id, physicsClientId=client_id)
    if not info:
        return f"body[{body_id}]"
    base_link_name = _decode_or_none(info[0]) or ""
    body_name = _decode_or_none(info[1]) or ""
    return body_name or base_link_name or f"body[{body_id}]"


def _link_label(p, body_id, link_index, client_id):
    if link_index == -1:
        info = p.getBodyInfo(body_id, physicsClientId=client_id)
        return _decode_or_none(info[0]) if info else "<base>"
    try:
        ji = p.getJointInfo(body_id, link_index, physicsClientId=client_id)
        return _decode_or_none(ji[12]) or f"link[{link_index}]"
    except Exception:
        return f"link[{link_index}]"


# ---------------------------------------------------------------------------
# Inline mirror of `core/robot_cell_support.py::_attach_support_tool_models`
# but for the ASSEMBLY cell — wraps the support RobotModel as a static
# articulated `ToolModel` obstacle on the dual-arm cell. Test-local; no
# edits to `core/`.
# ---------------------------------------------------------------------------


_INLINE_SUPPORT_OBSTACLE_NAME = "SupportRobotObstacle"  # distinct from the SupportGripper tool name in cell B


def _attach_support_as_obstacle_inline(stack, dual_arm_cell, support_robot_model):
    Frame = stack["Frame"]
    ToolModel = stack["ToolModel"]
    name = _INLINE_SUPPORT_OBSTACLE_NAME
    if name in dual_arm_cell.tool_models:
        return name
    dual_arm_cell.tool_models[name] = ToolModel.from_robot_model(
        support_robot_model.copy(), Frame.worldXY()
    )
    return name


# ---------------------------------------------------------------------------
# Client builders
# ---------------------------------------------------------------------------


def _connect_client(stack, connection_type: str, label: str, verbose: bool):
    PyBulletClient = stack["PyBulletClient"]
    PyBulletPlanner = stack["PyBulletPlanner"]
    client = PyBulletClient(connection_type=connection_type, verbose=True)
    client.__enter__()
    planner = PyBulletPlanner(client)
    if verbose:
        print(f"  client_{label}: connection={connection_type!r} "
              f"physicsClientId={client.client_id}")
    return client, planner


def _make_client_a(stack, connection_type: str, capture: dict | None, verbose: bool):
    """Client A: dual-arm cell (Cindy actuated + AssemblyLeft/Right tools)
    with the support robot wrapped as a static obstacle ToolModel.
    """
    import core.robot_cell as rc
    import core.robot_cell_support as rcs

    client, planner = _connect_client(stack, connection_type, "A", verbose)
    cell = rc.get_or_load_robot_cell()
    # Reuse the support cell's robot_model (Alice) as the static obstacle on
    # the dual-arm cell. `get_or_load_support_cell` is sticky-cached so this
    # is just a lookup if it's been loaded before.
    support_cell = rcs.get_or_load_support_cell()
    obstacle_name = _attach_support_as_obstacle_inline(stack, cell, support_cell.robot_model)

    state = rc.default_cell_state()
    if capture is not None and "assembled" in capture:
        # Push captured dual-arm joint values + base.
        ass = capture["assembled"]
        names = list(ass["joint_names_left"]) + list(ass["joint_names_right"])
        values = list(ass["joint_values_left"]) + list(ass["joint_values_right"])
        for n, v in zip(names, values):
            state.robot_configuration[n] = float(v)
        base_mm = np.asarray(ass["base_frame_world_mm"], dtype=float)
        state.robot_base_frame = rc._mm_matrix_to_m_frame(stack["Frame"], base_mm)
    # Default tool_states for the inline obstacle (frame=worldXY, zero config) is fine.
    planner.set_robot_cell(cell)
    return client, planner, cell, state, obstacle_name


def _make_client_b(stack, connection_type: str, capture: dict | None, verbose: bool):
    """Client B: support cell (Alice actuated + SupportGripper) with the
    dual-arm wrapped as a static obstacle (existing `DualArm` ToolModel).
    """
    import core.robot_cell as rc
    import core.robot_cell_support as rcs

    client, planner = _connect_client(stack, connection_type, "B", verbose)
    cell = rcs.get_or_load_support_cell()
    state = rcs.default_support_cell_state()

    if capture is not None and "assembled" in capture:
        ass = capture["assembled"]
        state = rcs.configure_dual_arm_obstacle(
            state,
            base_frame_world_mm=np.asarray(ass["base_frame_world_mm"], dtype=float),
            joint_values_left=ass["joint_values_left"],
            joint_values_right=ass["joint_values_right"],
            joint_names_left=ass["joint_names_left"],
            joint_names_right=ass["joint_names_right"],
        )
        if "base_frame_world_mm" in capture:
            sup_base_mm = np.asarray(capture["base_frame_world_mm"], dtype=float)
            state.robot_base_frame = rc._mm_matrix_to_m_frame(stack["Frame"], sup_base_mm)
        if "final" in capture:
            for n, v in zip(capture["final"]["joint_names"], capture["final"]["joint_values"]):
                state.robot_configuration[n] = float(v)

    planner.set_robot_cell(cell)
    import core.config as _cfg
    return client, planner, cell, state, _cfg.DUAL_ARM_OBSTACLE_TOOL_NAME


# ---------------------------------------------------------------------------
# Belle (second support robot) cell loader. Test-local — no edits to core/.
# Belle's URDF lives in the same package as Alice's; the only diff is the
# URDF/SRDF filenames and the calibrated joint origins. Group + ACM
# definitions are identical between Alice and Belle.
# ---------------------------------------------------------------------------


_BELLE_URDF_FILENAME = "husky_ur5_e_no_base_joint_Belle_Calibrated.urdf"
_BELLE_SRDF_REL_PATH = os.path.join("config", "belle.srdf")
_INLINE_BELLE_OBSTACLE_NAME = "BelleObstacle"  # for any future cell-A wiring


def _load_belle_cell_inline(stack):
    """Load the Belle support cell — mirrors `core.robot_cell_support.get_or_load_support_cell`
    but with Belle's URDF + SRDF. Attaches DualArm-as-tool the same way the
    production support cell does.
    """
    import core.config as _cfg
    import core.robot_cell as rc
    import core.robot_cell_support as rcs

    pkg_path = _cfg.HUSKY_PKG_PATH
    LocalPackageMeshLoader = stack["LocalPackageMeshLoader"]
    main_loader = LocalPackageMeshLoader(pkg_path, _cfg.SUPPORT_URDF_PKG_NAME)
    husky_loader = LocalPackageMeshLoader(pkg_path, "husky_description")
    ur_loader = LocalPackageMeshLoader(pkg_path, "ur_description")

    urdf_stream = main_loader.load_urdf(_BELLE_URDF_FILENAME)
    robot_model = stack["RobotModel"].from_urdf_string(urdf_stream.read())
    robot_model.load_geometry(main_loader, husky_loader, ur_loader)

    srdf_path = main_loader.build_path(
        os.path.dirname(_BELLE_SRDF_REL_PATH),
        os.path.basename(_BELLE_SRDF_REL_PATH),
    )
    semantics = stack["RobotSemantics"].from_srdf_file(srdf_path, robot_model)
    cell = stack["RobotCell"](robot_model, semantics)

    # Same SupportGripper + DualArm tool wiring Alice gets in production.
    rcs._attach_support_tool_models(cell)
    return cell


def _make_client_c(stack, connection_type: str, capture: dict | None, verbose: bool):
    """Client C: Belle support cell (Belle actuated + SupportGripper + DualArm-as-tool obstacle)."""
    import core.config as _cfg
    import core.robot_cell as rc
    import core.robot_cell_support as rcs

    client, planner = _connect_client(stack, connection_type, "C", verbose)
    cell = _load_belle_cell_inline(stack)
    # Use the same default-state wiring (SG attached_to_group + touch_links).
    state = cell.default_cell_state()
    rcs._configure_support_tool_states(state)

    if capture is not None and "assembled" in capture:
        ass = capture["assembled"]
        # Configure DualArm obstacle on Belle the same way as on Alice.
        Frame = stack["Frame"]
        Configuration = stack["Configuration"]
        da_name = _cfg.DUAL_ARM_OBSTACLE_TOOL_NAME
        da_tool = cell.tool_models[da_name]
        state.tool_states[da_name].frame = rc._mm_matrix_to_m_frame(
            Frame, np.asarray(ass["base_frame_world_mm"], dtype=float)
        )
        zero = da_tool.zero_configuration()
        cfg_names = list(zero.joint_names)
        cfg_types = list(zero.joint_types)
        cfg_values = list(zero.joint_values)
        captured = dict(zip(
            list(ass["joint_names_left"]) + list(ass["joint_names_right"]),
            list(ass["joint_values_left"]) + list(ass["joint_values_right"]),
        ))
        for i, name in enumerate(cfg_names):
            if name in captured:
                cfg_values[i] = float(captured[name])
        state.tool_states[da_name].configuration = Configuration(
            joint_values=cfg_values, joint_types=cfg_types, joint_names=cfg_names,
        )

    planner.set_robot_cell(cell)
    return client, planner, cell, state, _cfg.DUAL_ARM_OBSTACLE_TOOL_NAME


# ---------------------------------------------------------------------------
# Footgun-bypass: push state via compas_fab only — no pp helpers.
# ---------------------------------------------------------------------------


def _push_state_safely(planner, state):
    """Push joint state + base frame to PyBullet via compas_fab's per-
    client-scoped path. Avoids:

      * `pp.set_pose(body, pose)`           uses module-global CLIENT
      * `core.robot_cell.set_cell_state`     calls pp.set_pose redundantly
      * any `set_client(...)` global mutation

    `planner.set_robot_cell_state(state)` already invokes
    `client._set_base_frame(...)` (client.py:702) which uses
    `physicsClientId=self.client_id` correctly.
    """
    planner.set_robot_cell_state(state)


# ---------------------------------------------------------------------------
# Read / mutate via raw pybullet (always scoped with physicsClientId=).
# ---------------------------------------------------------------------------


def _get_movable_joint_ids(p, body_id, client_id):
    """Return joint indices whose type != FIXED, for getJointStates."""
    n = p.getNumJoints(body_id, physicsClientId=client_id)
    movable = []
    for j in range(n):
        ji = p.getJointInfo(body_id, j, physicsClientId=client_id)
        # ji[2] is jointType; pybullet.JOINT_FIXED == 4
        if ji[2] != p.JOINT_FIXED:
            movable.append(j)
    return movable


def _movable_for(p, client_id, body_id, cache):
    key = (client_id, body_id)
    if key not in cache:
        cache[key] = _get_movable_joint_ids(p, body_id, client_id)
    return cache[key]


def _read_body_pose(p, client_id, body_id, movable_ids_cache: dict):
    """Returns (base_xyz, base_quat, joint_positions_tuple) for ANY body."""
    point, quat = p.getBasePositionAndOrientation(body_id, physicsClientId=client_id)
    movable = _movable_for(p, client_id, body_id, movable_ids_cache)
    if not movable:
        return tuple(point), tuple(quat), tuple()
    states = p.getJointStates(body_id, movable, physicsClientId=client_id)
    positions = tuple(s[0] for s in states)
    return tuple(point), tuple(quat), positions


def _read_robot_pose(p, client, movable_ids_cache: dict):
    return _read_body_pose(p, client.client_id, client.robot_puid, movable_ids_cache)


def _mutate_base_and_joint(p, client, dx, joint_offset_idx, dq_val, movable_ids_cache):
    """Translate base by dx (m) and bump movable joint #joint_offset_idx by dq_val rad.
    All scoped per-client.
    """
    body_id = client.robot_puid
    cid = client.client_id
    point, quat = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)
    new_point = (point[0] + dx[0], point[1] + dx[1], point[2] + dx[2])
    p.resetBasePositionAndOrientation(body_id, new_point, quat, physicsClientId=cid)
    movable = _movable_for(p, cid, body_id, movable_ids_cache)
    if movable:
        idx = movable[joint_offset_idx % len(movable)]
        cur = p.getJointState(body_id, idx, physicsClientId=cid)[0]
        p.resetJointState(body_id, idx, cur + dq_val, physicsClientId=cid)


def _sync_mirror_raw(p, gui_client, mirror_tool_name, src_client, movable_ids_cache):
    """Copy `src_client`'s actuated robot pose + joint configuration into
    `gui_client`'s mirror-tool body via raw pybullet calls. The mirror tool
    is a `ToolModel.from_robot_model(...)` that compas_fab loads as its own
    pybullet body via `loadURDF` — `gui_client.tools_puids[name]` is its
    body_id. We poke that body's base + joints directly so the GUI can show
    where the OTHER (DIRECT) client's robot really is.

    Returns the mirror's pose AFTER the sync, for printing.
    """
    src_pose = _read_robot_pose(p, src_client, movable_ids_cache)
    point, quat, src_values = src_pose

    mirror_body = gui_client.tools_puids[mirror_tool_name]
    gui_cid = gui_client.client_id
    p.resetBasePositionAndOrientation(mirror_body, point, quat, physicsClientId=gui_cid)
    mirror_movable = _movable_for(p, gui_cid, mirror_body, movable_ids_cache)
    n_to_apply = min(len(src_values), len(mirror_movable))
    for k in range(n_to_apply):
        p.resetJointState(mirror_body, mirror_movable[k], src_values[k], physicsClientId=gui_cid)
    return _read_body_pose(p, gui_cid, mirror_body, movable_ids_cache)


# ---------------------------------------------------------------------------
# Assertions / pretty-print
# ---------------------------------------------------------------------------


def _fmt_pose(label, pose):
    base_xyz, base_quat, joints = pose
    j_preview = ", ".join(f"{j:+.4f}" for j in joints[:3]) + (", ..." if len(joints) > 3 else "")
    return (f"{label} base=({base_xyz[0]:+.4f},{base_xyz[1]:+.4f},{base_xyz[2]:+.4f}) "
            f"q=({base_quat[0]:+.3f},{base_quat[1]:+.3f},{base_quat[2]:+.3f},{base_quat[3]:+.3f}) "
            f"j=[{j_preview}] (n={len(joints)})")


def _allclose(a, b, atol):
    if not a and not b:
        return True
    return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=atol, rtol=0.0)


def _assert_unchanged(label, before, after, atol_pos=1e-9, atol_q=1e-9, atol_j=1e-9):
    base_ok = _allclose(before[0], after[0], atol_pos)
    quat_ok = _allclose(before[1], after[1], atol_q)
    joints_ok = _allclose(before[2], after[2], atol_j)
    ok = base_ok and quat_ok and joints_ok
    status = "[OK]" if ok else "[FAIL]"
    print(f"  {status} {label}: base={base_ok} quat={quat_ok} joints={joints_ok}")
    if not ok:
        print(f"     before: {_fmt_pose('     ', before)}")
        print(f"     after:  {_fmt_pose('     ', after)}")
    return ok


def _assert_changed(label, before, after, atol_pos=1e-9):
    moved = not _allclose(before[0], after[0], atol_pos)
    status = "[OK]" if moved else "[FAIL]"
    print(f"  {status} {label}: changed={moved}")
    return moved


def _assert_joints_changed(label, before, after, atol=1e-9):
    j_before, j_after = before[2], after[2]
    if not j_before and not j_after:
        moved = False
    else:
        moved = not _allclose(j_before, j_after, atol)
    status = "[OK]" if moved else "[FAIL]"
    print(f"  {status} {label}: joints changed={moved}")
    return moved


# ---------------------------------------------------------------------------
# Stability soak loop (DIRECT only)
# ---------------------------------------------------------------------------


def _run_stability_loop(p, clients, n_iter, stop_on_error, rng, movable_ids_cache):
    """Round-robin mutate + read across `clients` (a list of (label, client))."""
    timings = []
    failures = []
    progress_every = max(1, n_iter // 10)
    n_clients = len(clients)
    for i in range(1, n_iter + 1):
        t0 = time.perf_counter()
        try:
            label, client = clients[(i - 1) % n_clients]
            _mutate_base_and_joint(
                p, client,
                dx=(rng.uniform(-0.001, 0.001), rng.uniform(-0.001, 0.001), 0.0),
                joint_offset_idx=rng.randrange(0, 6),
                dq_val=rng.uniform(-0.01, 0.01),
                movable_ids_cache=movable_ids_cache,
            )
            for _label, c in clients:
                _read_robot_pose(p, c, movable_ids_cache)
            for _label, c in clients:
                info = p.getConnectionInfo(physicsClientId=c.client_id)
                if not info.get("isConnected", 0):
                    raise RuntimeError(f"client {_label} disconnected mid-loop")
        except Exception as exc:
            failures.append((i, type(exc).__name__, str(exc)))
            print(f"  [FAIL] iter {i}: {type(exc).__name__}: {exc}")
            if stop_on_error:
                break
            continue
        timings.append((time.perf_counter() - t0) * 1000.0)
        if i % progress_every == 0:
            print(f"  iter {i}/{n_iter} "
                  f"(last={timings[-1]:.2f}ms, mean so far={statistics.mean(timings):.2f}ms, "
                  f"failures={len(failures)})")
    return timings, failures


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def _step_with_prompt(prompt: str, gui_active: bool):
    if gui_active:
        try:
            input(prompt)
        except EOFError:
            pass


def _disconnect_both(clients):
    # Reverse construction order; each call goes through PyBulletBase.disconnect
    # which is per-client-scoped (client.py:145).
    for label, c in reversed(clients):
        try:
            c.disconnect()
            print(f"  disconnected client_{label} (was id={c.client_id})")
        except Exception as exc:
            print(f"  client_{label}.disconnect() raised: {type(exc).__name__}: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Probe two-client PyBullet isolation + stability."
    )
    parser.add_argument(
        "--gui", choices=["a", "b", "none", "both"], default="a",
        help="Which client gets a GUI window (default: a). 'both' is rejected — "
             "PyBullet allows only ONE GUI per process.",
    )
    parser.add_argument(
        "--capture", default=None,
        help="Optional support-IK capture JSON to seed both initial states.",
    )
    parser.add_argument(
        "--stability-iterations", type=int, default=0,
        help="If >0, run a soak loop in DIRECT mode for this many iterations.",
    )
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--three-way", action="store_true",
        help="Stress-test with THREE clients: Cindy (assembly) + Alice (support) + Belle "
             "(second support). Default off keeps the 2-client behaviour.",
    )
    args = parser.parse_args()

    if args.gui == "both":
        parser.error(
            "PyBullet allows only ONE GUI per process. Pass --gui a, --gui b, "
            "or --gui none."
        )

    if args.stability_iterations > 0 and args.gui != "none":
        print(f"[!] --stability-iterations={args.stability_iterations} forces --gui=none "
              f"(was {args.gui!r}); both clients will be DIRECT.")
        args.gui = "none"

    capture = None
    if args.capture:
        if not os.path.isfile(args.capture):
            print(f"[X] --capture path not found: {args.capture}")
            return 1
        with open(args.capture, "r", encoding="utf-8") as stream:
            capture = json.load(stream)
        print(f"--capture loaded: {os.path.basename(args.capture)}")

    # Defensive: clear any leftover module-global state from a prior shared-client run.
    import core.robot_cell as rc
    if rc.is_pb_running():
        print("[!] is_pb_running() True — calling stop_pb_client() before two-client setup.")
        rc.stop_pb_client()

    stack = rc.import_compas_stack()
    import pybullet as p

    conn_a = "gui" if args.gui == "a" else "direct"
    conn_b = "gui" if args.gui == "b" else "direct"
    n_label = "three" if args.three_way else "two"
    print(f"\n=== Connecting {n_label} clients (A={conn_a}, B={conn_b}"
          + (", C=direct" if args.three_way else "") + ") ===")
    client_a, planner_a, _cell_a, state_a, obstacle_a_name = _make_client_a(
        stack, conn_a, capture, args.verbose
    )
    client_b, planner_b, _cell_b, state_b, obstacle_b_name = _make_client_b(
        stack, conn_b, capture, args.verbose
    )
    print(f"  Client A: dual-arm cell + obstacle tool '{obstacle_a_name}' (Alice as static)")
    print(f"  Client B: Alice support cell + obstacle tool '{obstacle_b_name}' (Cindy as static)")

    client_c = planner_c = state_c = None
    if args.three_way:
        # Belle is always DIRECT (one GUI per process — A or B already may have it).
        client_c, planner_c, _cell_c, state_c, _obstacle_c_name = _make_client_c(
            stack, "direct", capture, args.verbose
        )
        print(f"  Client C: Belle support cell + obstacle tool '{_obstacle_c_name}' (Cindy as static)")

    # Mirror wiring: in the GUI client, that client's obstacle tool body
    # gets visually slaved to the OTHER client's actuated robot state via
    # `_sync_mirror_raw`. So with --gui a, the SupportRobotObstacle tool on
    # client A becomes the visible mirror of client B's Alice; with --gui b,
    # the DualArm tool on client B becomes the visible mirror of client A's
    # Cindy. With --gui none, no mirror sync happens.
    if args.gui == "a":
        gui_planner = planner_a; gui_client_obj = client_a
        mirror_tool = obstacle_a_name; src_client = client_b
    elif args.gui == "b":
        gui_planner = planner_b; gui_client_obj = client_b
        mirror_tool = obstacle_b_name; src_client = client_a
    else:
        gui_planner = None; gui_client_obj = None; mirror_tool = None; src_client = None

    clients = [("A", client_a), ("B", client_b)]
    if args.three_way:
        clients.append(("C", client_c))
    movable_ids_cache: dict = {}

    overall_pass = True
    try:
        print("\n=== Pushing initial state on each client ===")
        _push_state_safely(planner_a, state_a)
        _push_state_safely(planner_b, state_b)
        if args.three_way:
            _push_state_safely(planner_c, state_c)

        a0 = _read_robot_pose(p, client_a, movable_ids_cache)
        b0 = _read_robot_pose(p, client_b, movable_ids_cache)
        print(_fmt_pose("  A0:", a0))
        print(_fmt_pose("  B0:", b0))
        c0 = None
        if args.three_way:
            c0 = _read_robot_pose(p, client_c, movable_ids_cache)
            print(_fmt_pose("  C0:", c0))

        gui_active = args.gui in ("a", "b")
        gui_client_id = client_a.client_id if args.gui == "a" else (
            client_b.client_id if args.gui == "b" else None
        )

        if gui_active:
            print(f"\nGUI={args.gui}: the actuated robot in this window is "
                  f"{'Cindy' if args.gui == 'a' else 'Alice'}; the visible MIRROR "
                  f"is the '{mirror_tool}' tool body, slaved to the OTHER (DIRECT) "
                  f"client's actuated robot via raw pybullet. The mirror should "
                  f"move ONLY when we sync it explicitly after a mutation on the "
                  f"OTHER client. If it ever moves after a same-client mutation, "
                  f"isolation has FAILED.")
            # Initial sync so the mirror starts at the source's current pose.
            _sync_mirror_raw(p, gui_client_obj, mirror_tool, src_client, movable_ids_cache)

        def _maybe_sync_after(label_src):
            """Sync the GUI's mirror only if its source is the just-mutated client."""
            if not gui_active:
                return
            mutated_was_src = (
                (args.gui == "a" and label_src == "B") or
                (args.gui == "b" and label_src == "A")
            )
            verb = "syncing (mirror should MOVE)" if mutated_was_src else "syncing (mirror should STAY put)"
            print(f"  GUI mirror {verb}")
            _sync_mirror_raw(p, gui_client_obj, mirror_tool, src_client, movable_ids_cache)

        # ----- Isolation phase 1: mutate A, B unchanged -----
        print("\n=== Isolation phase 1: mutate A; expect B unchanged ===")
        _mutate_base_and_joint(p, client_a, dx=(0.10, 0.0, 0.0),
                               joint_offset_idx=0, dq_val=0.20,
                               movable_ids_cache=movable_ids_cache)
        a1 = _read_robot_pose(p, client_a, movable_ids_cache)
        b1 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_changed("A vs A0", a0, a1)
        overall_pass &= _assert_unchanged("B vs B0", b0, b1)
        _maybe_sync_after("A")
        _step_with_prompt("press Enter for next mutation... ", gui_active)

        # ----- Isolation phase 2: mutate B, A unchanged -----
        print("\n=== Isolation phase 2: mutate B; expect A unchanged ===")
        _mutate_base_and_joint(p, client_b, dx=(0.0, 0.15, 0.0),
                               joint_offset_idx=1, dq_val=-0.30,
                               movable_ids_cache=movable_ids_cache)
        a2 = _read_robot_pose(p, client_a, movable_ids_cache)
        b2 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_unchanged("A vs A1", a1, a2)
        overall_pass &= _assert_changed("B vs B1", b1, b2)
        _maybe_sync_after("B")
        _step_with_prompt("press Enter for next mutation... ", gui_active)

        # ----- Cross-mutation: alternate one more on each -----
        print("\n=== Cross-mutation: alternate one more A then B ===")
        _mutate_base_and_joint(p, client_a, dx=(-0.05, 0.0, 0.02),
                               joint_offset_idx=2, dq_val=0.10,
                               movable_ids_cache=movable_ids_cache)
        a3 = _read_robot_pose(p, client_a, movable_ids_cache)
        b3 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_changed("A vs A2", a2, a3)
        overall_pass &= _assert_unchanged("B vs B2", b2, b3)
        _maybe_sync_after("A")
        _step_with_prompt("press Enter for next mutation... ", gui_active)

        _mutate_base_and_joint(p, client_b, dx=(0.0, -0.05, 0.0),
                               joint_offset_idx=3, dq_val=0.15,
                               movable_ids_cache=movable_ids_cache)
        a4 = _read_robot_pose(p, client_a, movable_ids_cache)
        b4 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_unchanged("A vs A3", a3, a4)
        overall_pass &= _assert_changed("B vs B3", b3, b4)
        _maybe_sync_after("B")

        # ----- pp.* helper scoping investigation -----
        print("\n=== pp.* helper scoping ===")
        print(f"  Found {len(_PP_CLIENT_MODULES)} pybullet_planning submodule(s) "
              "with their own CLIENT rebinding.")

        movable_a = _movable_for(p, client_a.client_id, client_a.robot_puid, movable_ids_cache)
        movable_b = _movable_for(p, client_b.client_id, client_b.robot_puid, movable_ids_cache)

        # ----- Negative test: pp.set_client alone does NOT route pp.set_pose.
        # We expect this to FAIL (B does NOT change). The "[OK]" labels below
        # are inverted: a passing run prints "[OK] pp.set_client BROKEN as
        # expected" — meaning the brokenness is reproduced, not fixed.
        print("\n--- pp Phase 0 (negative): pp.set_client alone is BROKEN for pp.set_pose ---")
        a_before_neg = _read_robot_pose(p, client_a, movable_ids_cache)
        b_before_neg = _read_robot_pose(p, client_b, movable_ids_cache)
        # Use the BROKEN context manager (only patches shared_const.CLIENT).
        with pp_active_client_BROKEN(client_b):
            pp.set_pose(client_b.robot_puid, ((1.0, 1.0, 0.05), (0.0, 0.0, 0.0, 1.0)))
        b_after_neg = _read_robot_pose(p, client_b, movable_ids_cache)
        b_unchanged = _allclose(b_before_neg[0], b_after_neg[0], atol=1e-9)
        if b_unchanged:
            print("  [OK] (proof) pp.set_client + pp.set_pose did NOT route to client B "
                  "(brokenness reproduced; bug = `from utils import CLIENT` rebinding).")
        else:
            print("  [!!] pp.set_client + pp.set_pose DID route correctly. The rebinding bug "
                  "may have been fixed upstream — re-evaluate the helper.")
        # Restore B's pose to its pre-negative-test position so the
        # downstream phase deltas remain meaningful.
        p.resetBasePositionAndOrientation(
            client_b.robot_puid, b_before_neg[0], b_before_neg[1],
            physicsClientId=client_b.client_id,
        )
        for k, v in zip(movable_b, b_before_neg[2]):
            p.resetJointState(client_b.robot_puid, k, v, physicsClientId=client_b.client_id)
        # Same for A (the BROKEN call probably moved A's body 0 to the
        # target — undo it).
        p.resetBasePositionAndOrientation(
            client_a.robot_puid, a_before_neg[0], a_before_neg[1],
            physicsClientId=client_a.client_id,
        )
        for k, v in zip(movable_a, a_before_neg[2]):
            p.resetJointState(client_a.robot_puid, k, v, physicsClientId=client_a.client_id)

        # ----- Positive tests: with the patched-all-submodules pp_active_client,
        # pp helpers DO scope correctly. These should pass. -----
        print("\n=== pp.* helpers with patched pp_active_client (patches every pp submodule's CLIENT) ===")

        # ----- pp Phase 1: pp.set_pose on A; expect B unchanged -----
        print("\n--- pp Phase 1: pp.set_pose on A; expect B unchanged ---")
        target_pose_a = ((1.5, 0.5, 0.1), (0.0, 0.0, 0.0, 1.0))
        with pp_active_client(client_a):
            pp.set_pose(client_a.robot_puid, target_pose_a)
        a_pp1 = _read_robot_pose(p, client_a, movable_ids_cache)
        b_pp1 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_changed("A vs A4 (pp.set_pose)", a4, a_pp1)
        overall_pass &= _assert_unchanged("B vs B4 (pp.set_pose on A)", b4, b_pp1)
        _maybe_sync_after("A")

        # ----- pp Phase 2: pp.set_pose on B; expect A unchanged -----
        print("\n--- pp Phase 2: pp.set_pose on B; expect A unchanged ---")
        target_pose_b = ((-0.7, 1.2, 0.05), (0.0, 0.0, 0.0, 1.0))
        with pp_active_client(client_b):
            pp.set_pose(client_b.robot_puid, target_pose_b)
        a_pp2 = _read_robot_pose(p, client_a, movable_ids_cache)
        b_pp2 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_unchanged("A vs A_pp1 (pp.set_pose on B)", a_pp1, a_pp2)
        overall_pass &= _assert_changed("B vs B_pp1 (pp.set_pose on B)", b_pp1, b_pp2)
        _maybe_sync_after("B")

        # ----- pp Phase 3: pp.set_joint_positions on A; expect B's joints unchanged -----
        print("\n--- pp Phase 3: pp.set_joint_positions on A; expect B's joints unchanged ---")
        new_vals_a = [0.5] * len(movable_a)
        with pp_active_client(client_a):
            pp.set_joint_positions(client_a.robot_puid, movable_a, new_vals_a)
        a_pp3 = _read_robot_pose(p, client_a, movable_ids_cache)
        b_pp3 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_joints_changed("A joints vs A_pp2 (pp.set_joint_positions on A)", a_pp2, a_pp3)
        overall_pass &= _assert_unchanged("B vs B_pp2 (joints+base unchanged)", b_pp2, b_pp3)
        _maybe_sync_after("A")

        # ----- pp Phase 4: pp.set_joint_positions on B; expect A's joints unchanged -----
        print("\n--- pp Phase 4: pp.set_joint_positions on B; expect A's joints unchanged ---")
        new_vals_b = [-0.3] * len(movable_b)
        with pp_active_client(client_b):
            pp.set_joint_positions(client_b.robot_puid, movable_b, new_vals_b)
        a_pp4 = _read_robot_pose(p, client_a, movable_ids_cache)
        b_pp4 = _read_robot_pose(p, client_b, movable_ids_cache)
        overall_pass &= _assert_unchanged("A vs A_pp3 (joints+base unchanged on A)", a_pp3, a_pp4)
        overall_pass &= _assert_joints_changed("B joints vs B_pp3 (pp.set_joint_positions on B)", b_pp3, b_pp4)
        _maybe_sync_after("B")

        # ----- pp Phase 5: pp.get_link_pose returns the right client's link pose -----
        print("\n--- pp Phase 5: pp.get_link_pose returns the right client's link pose ---")
        link_a = movable_a[0]
        link_b = movable_b[0]
        with pp_active_client(client_a):
            pose_link_a = pp.get_link_pose(client_a.robot_puid, link_a)
        with pp_active_client(client_b):
            pose_link_b = pp.get_link_pose(client_b.robot_puid, link_b)
        # The two clients are at deliberately different base poses, so the
        # link poses should differ. Use a relaxed tolerance for floating-
        # point comparison.
        distinct = not np.allclose(pose_link_a[0], pose_link_b[0], atol=1e-4)
        status = "[OK]" if distinct else "[FAIL]"
        print(f"  {status} pp.get_link_pose returns distinct poses per client")
        print(f"    A pose[0]: {tuple(f'{v:+.4f}' for v in pose_link_a[0])}")
        print(f"    B pose[0]: {tuple(f'{v:+.4f}' for v in pose_link_b[0])}")
        overall_pass &= distinct

        # Also verify cross-read: get_link_pose under A returns A's value,
        # under B returns B's. Re-read under flipped contexts and compare.
        with pp_active_client(client_a):
            pose_link_a_again = pp.get_link_pose(client_a.robot_puid, link_a)
        match_a = np.allclose(pose_link_a[0], pose_link_a_again[0], atol=1e-9)
        status = "[OK]" if match_a else "[FAIL]"
        print(f"  {status} pp.get_link_pose under A is deterministic (re-read matches)")
        overall_pass &= match_a

        # ----- Three-way isolation phases (Cindy + Alice + Belle) -----
        if args.three_way:
            print("\n=== Three-way isolation: mutate one of {A, B, C}; expect the other two unchanged ===")

            # Phase 3W-1: mutate A; expect B and C unchanged.
            print("\n--- Three-way phase 1: mutate A; expect B and C unchanged ---")
            a_3w0 = _read_robot_pose(p, client_a, movable_ids_cache)
            b_3w0 = _read_robot_pose(p, client_b, movable_ids_cache)
            c_3w0 = _read_robot_pose(p, client_c, movable_ids_cache)
            _mutate_base_and_joint(p, client_a, dx=(0.07, -0.05, 0.0),
                                   joint_offset_idx=4, dq_val=0.18,
                                   movable_ids_cache=movable_ids_cache)
            a_3w1 = _read_robot_pose(p, client_a, movable_ids_cache)
            b_3w1 = _read_robot_pose(p, client_b, movable_ids_cache)
            c_3w1 = _read_robot_pose(p, client_c, movable_ids_cache)
            overall_pass &= _assert_changed("A vs A_3w0", a_3w0, a_3w1)
            overall_pass &= _assert_unchanged("B vs B_3w0 (mutate A)", b_3w0, b_3w1)
            overall_pass &= _assert_unchanged("C vs C_3w0 (mutate A)", c_3w0, c_3w1)

            # Phase 3W-2: mutate B; expect A and C unchanged.
            print("\n--- Three-way phase 2: mutate B; expect A and C unchanged ---")
            _mutate_base_and_joint(p, client_b, dx=(0.0, 0.08, 0.0),
                                   joint_offset_idx=2, dq_val=-0.22,
                                   movable_ids_cache=movable_ids_cache)
            a_3w2 = _read_robot_pose(p, client_a, movable_ids_cache)
            b_3w2 = _read_robot_pose(p, client_b, movable_ids_cache)
            c_3w2 = _read_robot_pose(p, client_c, movable_ids_cache)
            overall_pass &= _assert_unchanged("A vs A_3w1 (mutate B)", a_3w1, a_3w2)
            overall_pass &= _assert_changed("B vs B_3w1", b_3w1, b_3w2)
            overall_pass &= _assert_unchanged("C vs C_3w1 (mutate B)", c_3w1, c_3w2)

            # Phase 3W-3: mutate C; expect A and B unchanged.
            print("\n--- Three-way phase 3: mutate C; expect A and B unchanged ---")
            _mutate_base_and_joint(p, client_c, dx=(-0.06, 0.04, 0.01),
                                   joint_offset_idx=3, dq_val=0.27,
                                   movable_ids_cache=movable_ids_cache)
            a_3w3 = _read_robot_pose(p, client_a, movable_ids_cache)
            b_3w3 = _read_robot_pose(p, client_b, movable_ids_cache)
            c_3w3 = _read_robot_pose(p, client_c, movable_ids_cache)
            overall_pass &= _assert_unchanged("A vs A_3w2 (mutate C)", a_3w2, a_3w3)
            overall_pass &= _assert_unchanged("B vs B_3w2 (mutate C)", b_3w2, b_3w3)
            overall_pass &= _assert_changed("C vs C_3w2", c_3w2, c_3w3)

            # Phase 3W-4: pp.set_pose with patched pp_active_client on each client; verify others unchanged.
            print("\n--- Three-way phase 4: pp.set_pose on each, scoped via pp_active_client ---")
            movable_c = _movable_for(p, client_c.client_id, client_c.robot_puid, movable_ids_cache)
            for label, cli, target in (
                ("A", client_a, ((2.0, 0.4, 0.05), (0, 0, 0, 1))),
                ("B", client_b, ((0.5, 1.7, 0.05), (0, 0, 0, 1))),
                ("C", client_c, ((-1.2, -0.6, 0.05), (0, 0, 0, 1))),
            ):
                before = {nm: _read_robot_pose(p, c, movable_ids_cache)
                          for nm, c in clients}
                with pp_active_client(cli):
                    pp.set_pose(cli.robot_puid, target)
                after = {nm: _read_robot_pose(p, c, movable_ids_cache)
                         for nm, c in clients}
                overall_pass &= _assert_changed(f"{label} vs prev (pp.set_pose on {label})",
                                                before[label], after[label])
                for other_nm, _ in clients:
                    if other_nm == label:
                        continue
                    overall_pass &= _assert_unchanged(
                        f"{other_nm} unchanged when pp.set_pose targets {label}",
                        before[other_nm], after[other_nm],
                    )

        if args.stability_iterations > 0:
            print(f"\n=== Stability soak: {args.stability_iterations} iterations "
                  f"({len(clients)} clients, all DIRECT) ===")
            rng = random.Random(args.seed)
            timings, failures = _run_stability_loop(
                p, clients, args.stability_iterations,
                args.stop_on_error, rng, movable_ids_cache,
            )
            n = len(timings)
            mean_ms = statistics.mean(timings) if timings else float("nan")
            p95_ms = (sorted(timings)[int(0.95 * n) - 1]
                      if n >= 20 else (max(timings) if timings else float("nan")))
            print(f"\nStability summary: iterations={args.stability_iterations} "
                  f"completed={n} failures={len(failures)} "
                  f"mean={mean_ms:.2f}ms p95={p95_ms:.2f}ms")
            if failures:
                overall_pass = False
                print("First 5 failures:")
                for f in failures[:5]:
                    print(f"  iter={f[0]} {f[1]}: {f[2]}")

        verdict = "ISOLATION OK" if overall_pass else "ISOLATION FAILED"
        print(f"\n=== Verdict: {verdict} ===")

        if gui_active and gui_client_id is not None:
            print("\nPyBullet GUI is open. Inspect, then close the window to exit.")
            try:
                while p.isConnected(physicsClientId=gui_client_id):
                    time.sleep(0.2)
            except KeyboardInterrupt:
                pass

    finally:
        print("\n=== Disconnecting ===")
        _disconnect_both(clients)

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())


# ============================================================================
# === REFACTOR NOTES ===
# When the dual-client design is adopted in production, fix the global-CLIENT
# footguns. Preferred approach: replace pp helpers with raw `pybullet.*` +
# `physicsClientId=client.client_id`. Alternative: bracket each pp call with
# `pp.set_client(client.client_id)` immediately before. Sites:
#
#   scripts/core/robot_cell.py:469
#     deps["pp"].set_pose(planner.client.robot_puid, _pose_from_frame(base_frame))
#     -> Either DELETE (compas_fab's set_robot_cell_state -> _set_base_frame
#        already pushes the base correctly per-client; this call is redundant)
#        OR replace with raw `pybullet.resetBasePositionAndOrientation(
#                               planner.client.robot_puid, point, quat,
#                               physicsClientId=planner.client.client_id)`.
#
#   scripts/core/robot_cell_support.py:247  -> same fix, identical reasoning.
#
#   scripts/core/robot_cell.py:423  pp.is_connected()
#     -> for each tracked client, use
#        `pybullet.getConnectionInfo(physicsClientId=client.client_id)["isConnected"]`.
#
#   scripts/core/robot_cell.py:396  pp.disconnect()
#     -> `client.disconnect()` (compas_fab.PyBulletBase.disconnect, client.py:145)
#        is already per-client. Replace.
#
#   scripts/rs_show_ik.py:230-231  pp.link_from_name + pp.get_link_pose
#     -> bracket with `pp.set_client(client.client_id)` (cheap, low-risk) OR
#        rewrite using raw pybullet.getLinkState + index-by-name walk.
#
# Once those are scoped, drop `_STICKY_CURRENT_CELL_KIND` and the
# `_ensure_<kind>_cell_loaded(planner)` helpers — they become dead code
# under the two-persistent-planners model.
# ============================================================================
