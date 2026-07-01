"""Headless planner + slider replay for a single BarAssemblyAction movement.

Mirrors the planning + replay workflow of
``husky-assembly-teleop/scripts/headless_live_monitor_test.py`` but stays
platform-independent and has no dependency on ``husky_assembly_teleop`` or ros2.

Goal: debug how Rhino-generated ``BarAssemblyAction`` JSONs load into
compas_fab + pybullet, run planning, and confirm collision + ACM setup
are correct end-to-end.

CLI:
    python tests/headless_bar_action_planner.py [<data_root>]
        --bar-action B6.json
        --movement {M1,M2,M3,M4}
        [--gui]
        [--max-time 60]
        [--no-replay]
        [--cell <RobotCell.json>]

Default ``<data_root>`` matches ``tests/debug_load_bar_action.py``.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from typing import Optional

import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
CFAB_SRC = os.path.join(REPO_ROOT, "external", "compas_fab", "src")
TAMP_SRC = os.path.join(REPO_ROOT, "external", "husky_assembly_tamp")
RSDS_SRC = os.path.join(REPO_ROOT, "external", "rs_data_structure")

for _p in (SCRIPTS_DIR, TESTS_DIR, CFAB_SRC, TAMP_SRC, RSDS_SRC):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

# scripts/core package (HOME_CONFIG_*, etc.).
from core import config as _config  # noqa: E402

# Movement dataclasses. Imported here (after ``core`` is cached above) rather
# than inside functions: they are cheap and carry no import-order risk of
# their own now that ``core`` has already loaded.
from rs_data_structure.bar_action import (  # noqa: E402
    RoboticDualArmConstrainedMovement,
    RoboticFreeMovement,
    RoboticLinearMovement,
)

# PyBullet + compas_fab backends and the planner API. Heavier than the imports
# above, but kept at module top for clarity. The CLI arg/path validation in
# main() still runs first, so a bad invocation just pays the import cost.
import pybullet  # noqa: E402
import pybullet_planning as pp  # noqa: E402
from compas.data import json_load  # noqa: E402
from compas_fab.backends import (  # noqa: E402
    CollisionCheckError,
    PyBulletClient,
    PyBulletPlanner,
)
from husky_assembly_tamp.motion_planner.api import (  # noqa: E402
    _ARM_SUFFIXES,
    plan_constrained_dual_arm,
    plan_constrained_dual_arm_linear,
    plan_dual_arm_linear_independent,
    plan_free_dual_arm,
)


DEFAULT_DATA_ROOT = (
    r"C:\Users\yijiangh\Insync\yijiang94817@gmail.com"
    r"\Google Drive - Shared with me\2025-03 Husky Assembly\data_design_study"
)
# DEFAULT_PROBLEM = "2026-05-14_foc_demo_reduced"
# DEFAULT_BAR_ACTION = "B226.json"
DEFAULT_PROBLEM = "2026-05-16_double_kissing_jig_demo"
DEFAULT_BAR_ACTION = "B6.json"

LEFT_GROUP = "base_left_arm_manipulator"
RIGHT_GROUP = "base_right_arm_manipulator"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


_ROLE_RE = re.compile(r"_M([0-9])_")


def match_role(mv) -> Optional[str]:
    """Return 'M1' / 'M2' / 'M3' / 'M4' for a Movement, or None.

    Primary signal: substring '_M<n>_' in ``movement_id``
    (e.g. 'B6_M1_CDFM_home_to_approach'). Fallback: Python type — only used
    when the id is missing or doesn't match the pattern.
    """
    mid = getattr(mv, "movement_id", None) or ""
    m = _ROLE_RE.search(mid)
    if m:
        return f"M{m.group(1)}"
    # TODO should also add a FreeMovement at the beginning to bring whatever robot current state to the start of M1, which should be computed after M1 is planned
    if isinstance(mv, RoboticDualArmConstrainedMovement):
        return "M1"
    if isinstance(mv, RoboticFreeMovement):
        return "M4"
    if isinstance(mv, RoboticLinearMovement):
        # TODO we will need to split RoboticConstrainedDualArmLinearMovement and RoboticLinearMovement
        return None  # ambiguous between M2 and M3 without the id
    return None


def fill_missing_config(state, rcell, home_left, home_right) -> None:
    """If state.robot_configuration is None, fill with HOME_CONFIG. Test-only."""
    if state is None or state.robot_configuration is not None:
        return
    cfg = rcell.zero_full_configuration()
    left_names = list(rcell.get_configurable_joint_names(LEFT_GROUP))
    right_names = list(rcell.get_configurable_joint_names(RIGHT_GROUP))
    for n, v in zip(left_names, home_left):
        cfg[n] = float(v)
    for n, v in zip(right_names, home_right):
        cfg[n] = float(v)
    state.robot_configuration = cfg


def start_planner(rcell, *, use_gui: bool):
    """Boot a PyBullet client + planner with ``rcell`` loaded."""
    # enable_debug_gui=True exposes the right-side parameter panel that
    # addUserDebugParameter sliders live in (cfab defaults this to False).
    client = PyBulletClient(
        connection_type="gui" if use_gui else "direct",
        verbose=True,
        enable_debug_gui=use_gui,
    )
    client.__enter__()
    pp.set_client(client.client_id)
    pp.CLIENTS[client.client_id] = True if use_gui else None
    planner = PyBulletPlanner(client)
    t0 = time.time()
    with pp.LockRenderer(False):
        planner.set_robot_cell(rcell)
    print(f"[pb] set_robot_cell: {time.time() - t0:.2f}s")
    return client, planner


def color_rigid_body(planner, rb_name, rgba):
    """Re-tint every pybullet sub-body of ``rb_name`` to ``rgba``.

    Visual-only — does not affect collision. Color persists across
    ``set_robot_cell_state`` calls (cfab only updates pose, not visual props),
    so calling once after the cell is loaded is enough to keep the active bar
    visible through the whole replay.
    """
    cid = planner.client.client_id
    for body_id in planner.client.rigid_bodies_puids[rb_name]:
        pybullet.changeVisualShape(
            body_id, -1, rgbaColor=list(rgba),
            physicsClientId=cid,
        )


def check_collision(planner, state, *, label: str = "") -> bool:
    """Run a full-report collision check on ``state``. Returns True if clean."""
    tag = f" [{label}]" if label else ""
    try:
        planner.check_collision(state, options={"full_report": True, "verbose": False})
        print(f"[OK]{tag} no collisions reported.")
        return True
    except CollisionCheckError as exc:
        pairs = list(getattr(exc, "collision_pairs", []) or [])
        print(f"[!!]{tag} {len(pairs)} colliding pair(s):")
        for line in str(exc).splitlines():
            print(f"     {line}")
        return False


def replay_with_slider(planner, template_state, path, joint_names_12):
    """Block on a pybullet debug-parameter slider that scrubs the path."""
    cid = planner.client.client_id
    n = len(path)
    if n == 0:
        print("[replay] empty path, skipping slider.")
        return
    print(f"[replay] {n} waypoint(s); Ctrl-C to exit slider.")

    slider = pybullet.addUserDebugParameter(
        f"Replay t (0..{n - 1})", 0.0, float(n - 1), 0.0,
        physicsClientId=cid,
    )

    states = []
    for q in path:
        s = template_state.copy()
        for name, val in zip(joint_names_12, q):
            s.robot_configuration[name] = float(val)
        states.append(s)

    try:
        while pp.has_gui():
            t = pybullet.readUserDebugParameter(slider, physicsClientId=cid)
            idx = int(round(t))
            idx = max(0, min(n - 1, idx))
            with pp.LockRenderer(False):
                planner.set_robot_cell_state(states[idx])
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("[replay] interrupted.")


def replay_segments(planner, segments, joint_names_12):
    """Slider that scrubs a chained sequence of movements end-to-end.

    ``segments`` is a list of ``(role, template_state, path)``. Each segment
    carries its own ``template_state`` (the movement's start_state, with that
    phase's attachments / allowed-contact set), so the held bar renders
    correctly: attached during M1/M2, released during M3/M4.
    """
    cid = planner.client.client_id

    # Flatten to (state_for_this_waypoint, q), remembering role boundaries.
    built = []
    labels = []
    for role, template_state, path in segments:
        for q in path:
            s = template_state.copy()
            for name, val in zip(joint_names_12, q):
                s.robot_configuration[name] = float(val)
            built.append(s)
            labels.append(role)

    n = len(built)
    if n == 0:
        print("[replay] empty path, skipping slider.")
        return
    print(f"[replay] {n} waypoint(s) across {len(segments)} movement(s); "
          "Ctrl-C to exit slider.")

    slider = pybullet.addUserDebugParameter(
        f"Replay t (0..{n - 1})", 0.0, float(n - 1), 0.0,
        physicsClientId=cid,
    )

    last_role = None
    try:
        while pp.has_gui():
            t = pybullet.readUserDebugParameter(slider, physicsClientId=cid)
            idx = int(round(t))
            idx = max(0, min(n - 1, idx))
            if labels[idx] != last_role:
                print(f"[replay] -> {labels[idx]}")
                last_role = labels[idx]
            with pp.LockRenderer(False):
                planner.set_robot_cell_state(built[idx])
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("[replay] interrupted.")


def select_movement(action, role: str):
    """Return the Movement for ``role`` ('M1'..'M4'), or None.

    Primary signal is ``match_role`` (the ``_M<n>_`` id tag). Falls back to
    type + order for the M2/M3 ambiguity when ids are untagged.
    """
    for mv in action.movements:
        if match_role(mv) == role:
            return mv
    linear = [mv for mv in action.movements
              if isinstance(mv, RoboticLinearMovement)]
    if role == "M2" and linear:
        return linear[0]
    if role == "M3" and len(linear) > 1:
        return linear[1]
    return None


def print_movement_roster(roles, movements) -> None:
    """Print the resolved movements in planning order before the run starts.

    A quick pre-flight roster: which movement fills each role, its type, and
    whether it already carries a start/target configuration ("yes" means the
    field is populated, e.g. a start propagated from the previous movement).
    Purely informational -- plans nothing and mutates nothing.
    """
    print("\n[roster] movements in planning order:")
    print(f"  {'role':<5} {'type':<36} {'movement_id':<26} {'start':<5} {'target':<6}")
    for role, mv in zip(roles, movements):
        if mv is None:
            print(f"  {role:<5} {'<none>':<36} {'-':<26} {'-':<5} {'-':<6}")
            continue
        start_state = getattr(mv, "start_state", None)
        has_start = "yes" if (start_state is not None
                              and getattr(start_state, "robot_configuration", None) is not None) else "no"
        has_target = "yes" if getattr(mv, "target_configuration", None) is not None else "no"
        mv_id = str(getattr(mv, "movement_id", "<?>") or "<?>")
        print(f"  {role:<5} {type(mv).__name__:<36} {mv_id:<26} {has_start:<5} {has_target:<6}")


def apply_conf12(state, rcell, joint_names_12, q) -> None:
    """Write a 12-vec ``q`` into ``state.robot_configuration`` (the arm joints).

    Builds a zero full configuration first if the state has none, so chained
    movements start exactly where the previous one ended.
    """
    if state.robot_configuration is None:
        state.robot_configuration = rcell.zero_full_configuration()
    for name, val in zip(joint_names_12, q):
        state.robot_configuration[name] = float(val)


def vec12_from_conf(conf, joint_names_12):
    """Extract a 12-vec (left||right arm joints) from a compas Configuration."""
    return np.asarray([float(conf[n]) for n in joint_names_12], dtype=float)


def print_movement_roster(movements, tag="roster"):
    """Which movements (in planning order) have a start_conf and a trajectory.

    Headless adaptation of ``husky_monitor._print_movement_roster``.
    """
    print(f"[{tag}] movement roster:")
    for i, m in enumerate(movements):
        if m is None:
            print(f"  [{i}] <missing>")
            continue
        has_conf = (m.start_state is not None
                    and getattr(m.start_state, "robot_configuration", None) is not None)
        has_traj = getattr(m, "trajectory", None) is not None
        print(f"  [{i}] {m.movement_id!r}")
        print(f"     - start state: has robot_conf = {has_conf}")
        print(f"     - has trajectory = {has_traj}")


def accept_trajectory(mv, path, *, role, index, movements, rcell,
                      joint_names_12, source="Plan"):
    """Post-step after a movement is planned: chain the configs forward.

    Headless adaptation of ``husky_monitor._accept_trajectory`` (trimmed of
    ROS logging, visualizer wiring, disk save, CDFM validation and the
    synthetic-M0 backfill). ``path`` is a list of 12-vec waypoints ordered by
    ``joint_names_12``.

    Behaviour, role-based:
      - M1/M4 own their start: mirror ``path[0]`` into ``mv.start_state``.
      - M2/M3 must already carry a propagated start config; reject the
        trajectory if it is missing or disagrees with ``path[0]``.
      - M1/M2/M3 forward-propagate ``path[-1]`` into the *next* movement's
        ``start_state.robot_configuration`` (M0/M4 terminate the chain).
      - Backward continuity check vs the previous movement's ``path[-1]``.

    Returns True when accepted, False on a chain break (caller should stop).
    """
    mv.trajectory = path
    if path:
        start_vec = np.asarray(path[0], dtype=float)

        if role in ("M2", "M3") and mv.start_state is not None:
            existing = mv.start_state.robot_configuration
            if existing is None:
                print(f"[{source}] {mv.movement_id!r} has no propagated start_conf; "
                      "rejecting trajectory.")
                mv.trajectory = None
                return False
            diff = float(np.abs(start_vec - vec12_from_conf(existing, joint_names_12)).max())
            if diff > 1e-3:
                print(f"[{source}] start of {mv.movement_id!r} differs from propagated "
                      f"start_conf by max {diff:.4f} rad/m; rejecting trajectory.")
                mv.trajectory = None
                return False
        elif mv.start_state is not None:
            # M1/M4 own their generated start_conf.
            apply_conf12(mv.start_state, rcell, joint_names_12, start_vec)

        # Forward-chain propagation.
        if role in ("M0", "M4"):
            pass
        elif index + 1 < len(movements):
            next_mv = movements[index + 1]
            if next_mv is not None and next_mv.start_state is not None:
                existing = next_mv.start_state.robot_configuration
                new_end = np.asarray(path[-1], dtype=float)
                if existing is None:
                    apply_conf12(next_mv.start_state, rcell, joint_names_12, new_end)
                    print(f"[{source}] propagated {mv.movement_id!r}.traj[-1] -> "
                          f"{next_mv.movement_id!r}.start_state.robot_configuration "
                          "(was None).")
                else:
                    diff = float(np.abs(
                        new_end - vec12_from_conf(existing, joint_names_12)
                    ).max())
                    if diff > 1e-3:
                        print(f"[{source}] end of {mv.movement_id!r} differs from existing "
                              f"{next_mv.movement_id!r}.start by max {diff:.4f} rad/m; "
                              "overwriting (chain rule).")
                    apply_conf12(next_mv.start_state, rcell, joint_names_12, new_end)

        # Backward continuity check.
        if index > 0:
            prev_mv = movements[index - 1]
            prev_path = getattr(prev_mv, "trajectory", None)
            if prev_path:
                diff = float(np.abs(
                    np.asarray(prev_path[-1], dtype=float) - start_vec
                ).max())
                if diff > 1e-3:
                    print(f"[{source}] start of {mv.movement_id!r} differs from "
                          f"{prev_mv.movement_id!r}.trajectory[-1] by max {diff:.4f} rad/m.")
                else:
                    print(f"[{source}] start agrees with {prev_mv.movement_id!r}."
                          f"trajectory[-1] (max diff {diff:.6f}).")

    print_movement_roster(movements, tag=source)
    return True


def _path_from_jt(jt, joint_names_12):
    """Convert a JointTrajectory to a list of 12-vecs ordered by names."""
    if jt is None:
        return None
    return [
        [float(p.joint_values[p.joint_names.index(n)]) for n in joint_names_12]
        for p in jt.points
    ]


def plan_movement(planner, state, role, selected, *, active_bar_id,
                  active_bar_rb_name, joint_names_12, max_time,
                  derive_start=True):
    """Dispatch one movement to its planner API. Returns ``(path, info)``.

    ``path`` is a list of 12-vec waypoints, or None on failure (with a
    ``failure_reason`` in ``info``).

    For M1, ``derive_start`` (default) asks the planner to compute a feasible,
    grasp-consistent start instead of trusting the (placeholder) start config
    in the cell state — see ``api.plan_constrained_dual_arm(derive_start=...)``.
    """
    goal_conf = selected.target_configuration
    goal_ee_frames = selected.target_ee_frames or None

    if role == "M1" and isinstance(selected, RoboticDualArmConstrainedMovement):
        if not active_bar_id:
            return None, {"failure_reason": "M1 needs active_bar_id on the action."}
        if not goal_ee_frames:
            return None, {"failure_reason": "M1 needs target_ee_frames on the movement."}
        print(f"[plan] plan_constrained_dual_arm "
              f"(active_bar_id={active_bar_id}, derive_start={derive_start})")
        return plan_constrained_dual_arm(
            planner, state,
            active_bar_id=active_bar_rb_name,
            goal_ee_frames=goal_ee_frames,
            max_time=max_time,
            derive_start=derive_start,
        )
    if role == "M2" and isinstance(selected, RoboticLinearMovement):
        if not active_bar_id:
            return None, {"failure_reason": "M2 needs active_bar_id on the action."}
        print(f"[plan] plan_constrained_dual_arm_linear (active_bar_id={active_bar_id})")
        jt = plan_constrained_dual_arm_linear(
            planner, state,
            active_bar_id=active_bar_rb_name,
            goal_conf=goal_conf,
            goal_ee_frames=goal_ee_frames,
        )
        path = _path_from_jt(jt, joint_names_12)
        return path, {"failure_reason": None if jt is not None else "linear-ik failed"}
    if role == "M3" and isinstance(selected, RoboticLinearMovement):
        print("[plan] plan_dual_arm_linear_independent")
        jt = plan_dual_arm_linear_independent(
            planner, state,
            goal_conf=goal_conf,
            goal_ee_frames=goal_ee_frames,
        )
        path = _path_from_jt(jt, joint_names_12)
        return path, {"failure_reason": None if jt is not None else "linear-ik failed"}
    if role == "M4" and isinstance(selected, RoboticFreeMovement):
        # M4 returns to a fixed dual-arm home. Override the (placeholder) target
        # from the action with the known-good home config (left 6 then right 6,
        # matching joint_names_12).
        goal_conf = list(_config.HUSKY_DUAL_ARM_HOME_CONF_12)
        print("[plan] plan_free_dual_arm (goal = HUSKY_DUAL_ARM_HOME_CONF_12)")
        return plan_free_dual_arm(planner, state, goal_conf, max_time=max_time)

    return None, {
        "failure_reason": (
            f"role {role!r} does not match movement type {type(selected).__name__}"
        )
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "data_root", nargs="?", default=DEFAULT_DATA_ROOT,
        help="Parent folder containing per-problem subfolders.",
    )
    parser.add_argument(
        "--problem", default=DEFAULT_PROBLEM,
        help="Subfolder under <data_root> with BarActions/ + RobotCell.json.",
    )
    parser.add_argument(
        "--bar-action", default=DEFAULT_BAR_ACTION,
        help="BarAction filename inside <data_root>/<problem>/BarActions/.",
    )
    parser.add_argument(
        "--movement", required=True, choices=("M1", "M2", "M3", "M4", "all"),
        help="Which movement role to plan. 'all' chains M1->M2->M3->M4.",
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--max-time", type=float, default=60.0)
    parser.add_argument("--no-replay", action="store_true")
    parser.add_argument(
        "--no-derive-start", dest="derive_start", action="store_false",
        help="Trust M1's start config from the cell state instead of deriving "
             "a feasible grasp-consistent start (default: derive).",
    )
    parser.set_defaults(derive_start=True)
    parser.add_argument(
        "--cell", default=None,
        help="Path to RobotCell.json (default: <data_root>/RobotCell.json).",
    )
    args = parser.parse_args()

    data_root = os.path.abspath(args.data_root)
    if not os.path.isdir(data_root):
        print(f"[X] missing data_root: {data_root}")
        return 1

    problem_dir = os.path.join(data_root, args.problem)
    if not os.path.isdir(problem_dir):
        print(f"[X] missing problem dir: {problem_dir}")
        return 1

    action_path = os.path.join(problem_dir, "BarActions", args.bar_action)
    if not os.path.isfile(action_path):
        print(f"[X] missing BarAction file: {action_path}")
        return 1

    cell_path = args.cell or os.path.join(problem_dir, "RobotCell.json")
    if not os.path.isfile(cell_path):
        print(f"[X] missing RobotCell.json: {cell_path}")
        return 1

    print(f"[load] RobotCell    <- {cell_path}")
    rcell = json_load(cell_path)
    print(f"  robot model   : {getattr(rcell.robot_model, 'name', '<?>')}")
    print(f"  tool models   : {sorted(rcell.tool_models.keys())}")
    print(f"  rigid bodies  : {len(rcell.rigid_body_models)}")

    print(f"[load] BarAction    <- {action_path}")
    action = json_load(action_path)
    active_bar_id = getattr(action, "active_bar_id", "") or ""
    print(f"  action_id     : {action.action_id}")
    print(f"  active_bar_id : {active_bar_id}")
    print(f"  movements     : {len(action.movements)}")

    # The cell stores bars as rigid bodies named ``bar_<bar_id>``. The
    # planner API needs that rigid-body name, not the bare bar id.
    def _resolve_bar_rb_name(cell, bar_id: str) -> Optional[str]:
        for candidate in (bar_id, f"bar_{bar_id}"):
            if candidate in cell.rigid_body_models:
                return candidate
        return None

    active_bar_rb_name = _resolve_bar_rb_name(rcell, active_bar_id) if active_bar_id else None
    if active_bar_id and active_bar_rb_name is None:
        print(f"[X] active_bar_id {active_bar_id!r} not found in cell.rigid_body_models")
        return 1
    if active_bar_rb_name and active_bar_rb_name != active_bar_id:
        print(f"  bar rigid-body name: {active_bar_rb_name}")

    roles = ["M1", "M2", "M3", "M4"] if args.movement == "all" else [args.movement]

    print(f"\n[pb] starting PyBullet ({'GUI' if args.gui else 'DIRECT'})")
    _client, planner = start_planner(rcell, use_gui=args.gui)

    try:
        # 12-vec joint names — read from the cell, shared across movements.
        left_names = [n for n in rcell.get_configurable_joint_names(LEFT_GROUP)
                      if any(n.endswith(s) for s in _ARM_SUFFIXES)]
        right_names = [n for n in rcell.get_configurable_joint_names(RIGHT_GROUP)
                       if any(n.endswith(s) for s in _ARM_SUFFIXES)]
        joint_names_12 = left_names + right_names

        # Resolve the movements once, in planning order, and mutate their
        # start_states in place — accept_trajectory propagates each planned
        # endpoint into the next movement's start_state (the key chaining step).
        movements = [select_movement(action, r) for r in roles]

        segments = []          # (role, state, path) for each solved movement
        results = []           # (role, ok, detail) for the final roll-up

        # Pre-flight: show what will be planned, in order, before we start.
        print_movement_roster(roles, movements)

        for index, role in enumerate(roles):
            selected = movements[index]
            if selected is None:
                msg = f"no movement matches role {role!r} in {args.bar_action}"
                print(f"[X] {msg}")
                results.append((role, False, msg))
                break  # chain can't continue past a missing movement

            print(f"\n[pick] {role} -> {type(selected).__name__} {selected.movement_id}")

            state = selected.start_state
            if state is None:
                msg = f"{selected.movement_id}: start_state is None."
                print(f"[X] {msg}")
                results.append((role, False, msg))
                break

            # M1 derives its own start; M2/M3/M4 should already carry a start
            # config propagated from the previous movement's accept_trajectory.
            # Home is only a fallback when nothing has set one (e.g. a single
            # non-M1 movement run in isolation, or M1 before it derives).
            if state.robot_configuration is None:
                fill_missing_config(
                    state, rcell, _config.HOME_CONFIG_LEFT, _config.HOME_CONFIG_RIGHT,
                )

            # Apply start_state to the live scene.
            t0 = time.time()
            with pp.LockRenderer(False):
                planner.set_robot_cell_state(state)
            print(f"[pb] set_robot_cell_state: {time.time() - t0:.2f}s")

            # Tint the active bar vivid blue so it's easy to track during replay.
            if args.gui and active_bar_rb_name is not None:
                color_rigid_body(planner, active_bar_rb_name, rgba=(0.1, 0.4, 1.0, 1.0))

            # Primary ACM check.
            check_collision(planner, state, label=selected.movement_id)

            path, info = plan_movement(
                planner, state, role, selected,
                active_bar_id=active_bar_id,
                active_bar_rb_name=active_bar_rb_name,
                joint_names_12=joint_names_12,
                max_time=args.max_time,
                derive_start=args.derive_start,
            )

            if path is None:
                reason = (info or {}).get("failure_reason", "<unknown>")
                print(f"[plan] {role} FAILED: {reason}")
                results.append((role, False, reason))
                break  # can't chain the next movement without this one's end config

            print(f"[plan] {role} OK: {len(path)} waypoint(s)")
            for k, v in (info or {}).items():
                if k in ("profile", "smooth_profile", "path_poses", "derived_start_conf"):
                    continue
                print(f"  {k}: {v}")

            # Chain bookkeeping: write traj[0] into this movement's start_state
            # (M1/M4), validate it (M2/M3), and propagate traj[-1] into the next
            # movement's start_state. Mirrors husky_monitor._accept_trajectory.
            accepted = accept_trajectory(
                selected, path,
                role=role, index=index, movements=movements,
                rcell=rcell, joint_names_12=joint_names_12, source="Plan",
            )
            if not accepted:
                results.append((role, False, "chain rejected (start mismatch)"))
                break

            segments.append((role, selected.start_state, path))
            results.append((role, True, f"{len(path)} waypoint(s)"))

        # Roll-up summary (most useful in batch mode).
        if len(roles) > 1:
            print("\n[summary]")
            for role, ok, detail in results:
                mark = "OK  " if ok else "FAIL"
                print(f"  [{mark}] {role}: {detail}")

        all_ok = bool(segments) and all(ok for _, ok, _ in results)

        # Replay.
        if not segments:
            return 2
        if args.no_replay or not args.gui:
            print("[replay] skipped (use --gui without --no-replay to enable).")
            return 0 if all_ok else 2
        if len(segments) == 1:
            _role, state, path = segments[0]
            replay_with_slider(planner, state, path, joint_names_12)
        else:
            replay_segments(planner, segments, joint_names_12)
        return 0 if all_ok else 2
    finally:
        try:
            pp.disconnect()
        except Exception as exc:
            print(f"[pb] disconnect raised ({exc}); continuing.")


if __name__ == "__main__":
    raise SystemExit(main())
