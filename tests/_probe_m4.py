"""Diagnose M4 (free birrt to home): endpoint-collision vs timeout vs iterations.

Reproduces plan_free_dual_arm's setup on the half-solved M4 (start = M3's
propagated end, goal = HUSKY_DUAL_ARM_HOME_CONF_12) and reports:
  1. collision verdict for start and goal under birrt's own predicate,
  2. full-report collision on each endpoint (to see any pairs),
  3. whether check_initial_end passes,
  4. birrt result + wall-clock at a few (max_iterations, max_time) budgets,
so we can tell a genuine no-path from an iteration-starved search.

Run:
    C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe tests/_probe_m4.py
"""
import argparse
import os
import sys
import time

_TESTS = os.path.dirname(os.path.abspath(__file__))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs

bootstrap_rhino_site_envs(verbose=False)

import numpy as np
import pybullet_planning as pp
from compas.data import json_load

import headless_bar_action_planner as H
from husky_assembly_tamp.motion_planner.api import (
    _arm_joint_names,
    _build_cfab_collision_fn,
    _conf12_from_state,
    _conf12_from_target,
    _state_with_conf12,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--max-time", type=float, default=30.0,
                        help="birrt wall-clock budget (seconds). Timeout is the ONLY "
                             "stop control; max_iterations is set absurdly high.")
    args = parser.parse_args()

    data_root = os.path.abspath(H.DEFAULT_DATA_ROOT)
    problem_dir = os.path.join(data_root, H.DEFAULT_PROBLEM)
    cell_path = os.path.join(problem_dir, "RobotCell.json")
    solved_path = H.solved_action_path(os.path.join(problem_dir, "BarActions", H.DEFAULT_BAR_ACTION))
    if not os.path.isfile(solved_path):
        print(f"[X] no half-solved file: {solved_path}")
        return 1

    print(f"[load] cell   <- {cell_path}")
    print(f"[load] action <- {solved_path}  (half-solved)")
    rcell = json_load(cell_path)
    action = json_load(solved_path)

    _client, planner = H.start_planner(rcell, use_gui=False)

    m4 = H.select_movement(action, "M4")
    state = m4.start_state
    if state is None or state.robot_configuration is None:
        print("[X] M4 has no propagated start conf in the half-solved file.")
        return 2

    left, right = _arm_joint_names(rcell)
    jn12 = left + right
    robot_puid = planner.client.robot_puid
    arm_joints = pp.joints_from_names(robot_puid, jn12)

    start_conf = _conf12_from_state(state, jn12)
    goal_conf = _conf12_from_target(list(H._config.HUSKY_DUAL_ARM_HOME_CONF_12), jn12)
    print(f"\n[endpoints] max|start-goal| joint delta = {np.abs(start_conf - goal_conf).max():.3f} rad")

    planner.set_robot_cell_state(state)
    collision_fn = _build_cfab_collision_fn(planner, state, jn12)

    # (1) birrt predicate verdicts.
    print("\n[predicate] birrt collision_fn (full CC1-CC5, True == colliding):")
    print(f"    collision_fn(start) = {collision_fn(start_conf)}")
    planner.set_robot_cell_state(state)
    print(f"    collision_fn(goal)  = {collision_fn(goal_conf)}")

    # (2) full-report on each endpoint (list any colliding pairs).
    print("\n[full report] START (M3 end):")
    planner.set_robot_cell_state(state)
    H.check_collision(planner, _state_with_conf12(state, start_conf, jn12), label="M4.start")
    print("[full report] GOAL (home HUSKY_DUAL_ARM_HOME_CONF_12):")
    H.check_collision(planner, _state_with_conf12(state, goal_conf, jn12), label="M4.goal")

    # (3) check_initial_end (what plan_free_dual_arm gates on).
    with pp.WorldSaver():
        pp.set_joint_positions(robot_puid, arm_joints, start_conf)
        gate = pp.check_initial_end(start_conf, goal_conf, collision_fn, diagnosis=False)
    print(f"\n[gate] check_initial_end passed = {gate}")

    # (4) birrt at increasing budgets.
    resolutions = np.ones(12) * 0.05
    sample_fn = pp.get_sample_fn(robot_puid, arm_joints)
    distance_fn = pp.get_distance_fn(robot_puid, arm_joints)
    extend_fn = pp.get_extend_fn(robot_puid, arm_joints, resolutions=resolutions)

    # Profiled birrt run. Timeout is the ONLY stop control: max_iterations is set
    # absurdly high so it never trips before max_time. We wrap the four core
    # callbacks to break down where wall-clock actually goes.
    prof_calls = {}
    prof_time = {}

    def _wrap(name, fn):
        def w(*a, **k):
            t = time.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                prof_calls[name] = prof_calls.get(name, 0) + 1
                prof_time[name] = prof_time.get(name, 0.0) + (time.perf_counter() - t)
        return w

    c_collision = _wrap("collision_fn", collision_fn)
    c_sample = _wrap("sample_fn", sample_fn)
    c_distance = _wrap("distance_fn", distance_fn)
    c_extend = _wrap("extend_fn", extend_fn)

    MAX_ITER = 10_000_000  # effectively unbounded -> only max_time stops the search
    print(f"\n[birrt] profiled run: max_time={args.max_time}s  max_iterations={MAX_ITER} "
          "(timeout-governed)")
    with pp.WorldSaver():
        pp.set_joint_positions(robot_puid, arm_joints, start_conf)
        t0 = time.perf_counter()
        path = pp.solve_motion_plan(
            start_conf, goal_conf, c_distance, c_sample, c_extend, c_collision,
            algorithm="birrt", max_time=float(args.max_time), max_iterations=MAX_ITER,
            smooth=0, coarse_waypoints=False,
        )
        wall = time.perf_counter() - t0
    status = f"OK ({len(path)} waypoints)" if path else "None (no path found)"
    print(f"[birrt] result: {status}   wall={wall:.2f}s")

    print("\n=================== PLANNING TIME BREAKDOWN ===================")
    accounted = 0.0
    for name in ("collision_fn", "extend_fn", "sample_fn", "distance_fn"):
        c = prof_calls.get(name, 0)
        tt = prof_time.get(name, 0.0)
        accounted += tt
        per = (tt / c * 1e3) if c else 0.0
        print(f"  {name:14s}: {tt:8.2f}s  ({100 * tt / wall:5.1f}%)  calls={c:<9} {per:.3f} ms/call")
    other = max(0.0, wall - accounted)
    print(f"  {'other (birrt)':14s}: {other:8.2f}s  ({100 * other / wall:5.1f}%)  "
          "nearest-neighbor / tree bookkeeping")
    print(f"  {'TOTAL':14s}: {wall:8.2f}s")
    cps = prof_calls.get("collision_fn", 0) / wall if wall else 0.0
    print(f"\n  collision checks/sec = {cps:.1f}")
    print("===============================================================")

    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        try:
            pp.disconnect()
        except Exception:
            pass
    raise SystemExit(rc)
