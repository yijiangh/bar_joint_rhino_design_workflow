"""Diagnostic: does M2 (linear mate) fail on collision (CC1/CC2) or on IK reach?

Loads the half-solved BarAction (so M2's start = M1's propagated end), then runs
plan_constrained_dual_arm_linear four ways, toggling the self-collision (CC1) and
robot-vs-tool (CC2) checks that are the only ones active during M2 IK
(skip_env_collisions=True already disables CC3/4/5).

We monkeypatch planner.inverse_kinematics to inject _skip_cc1/_skip_cc2 into the
options the IK loop passes -- no change to api.py.

    both on   (baseline)  -> reproduces the real run
    both off              -> pure IK reachability (no self/tool collision)
    CC1 only  (self on)   -> is self-collision the blocker?
    CC2 only  (tool on)   -> is robot-vs-tool the blocker?

Run:
    C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe tests/_probe_m2_cc.py
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs

bootstrap_rhino_site_envs(verbose=False)

from compas.data import json_load

import headless_bar_action_planner as H
from husky_assembly_tamp.motion_planner.api import plan_constrained_dual_arm_linear


def run_variant(planner, state, active_bar_rb, goal_conf, goal_ee_frames, *, skip_cc1, skip_cc2):
    """Run M2 linear planning with CC1/CC2 forced on/off. Returns the JointTrajectory or None."""
    real_ik = planner.inverse_kinematics

    def patched_ik(target, s, group, options=None):
        opts = dict(options or {})
        opts["_skip_cc1"] = skip_cc1
        opts["_skip_cc2"] = skip_cc2
        return real_ik(target, s, group, opts)

    planner.inverse_kinematics = patched_ik
    try:
        return plan_constrained_dual_arm_linear(
            planner, state,
            active_bar_id=active_bar_rb,
            goal_conf=goal_conf,
            goal_ee_frames=goal_ee_frames,
        )
    finally:
        planner.inverse_kinematics = real_ik


def main() -> int:
    data_root = os.path.abspath(H.DEFAULT_DATA_ROOT)
    problem_dir = os.path.join(data_root, H.DEFAULT_PROBLEM)
    cell_path = os.path.join(problem_dir, "RobotCell.json")
    clean_path = os.path.join(problem_dir, "BarActions", H.DEFAULT_BAR_ACTION)
    solved_path = H.solved_action_path(clean_path)

    if not os.path.isfile(solved_path):
        print(f"[X] no half-solved file: {solved_path}\n    run once with --load clean --movement all first.")
        return 1

    print(f"[load] cell   <- {cell_path}")
    print(f"[load] action <- {solved_path}  (half-solved)")
    rcell = json_load(cell_path)
    action = json_load(solved_path)

    active_bar_id = getattr(action, "active_bar_id", "") or ""
    active_bar_rb = active_bar_id if active_bar_id in rcell.rigid_body_models else f"bar_{active_bar_id}"

    _client, planner = H.start_planner(rcell, use_gui=False)

    selected = H.select_movement(action, "M2")
    if selected is None:
        print("[X] no M2 movement.")
        return 2
    state = selected.start_state
    if state is None or state.robot_configuration is None:
        print("[X] M2 has no propagated start conf in the half-solved file.")
        return 2

    goal_conf = selected.target_configuration
    goal_ee_frames = selected.target_ee_frames or None
    print(f"[probe] M2 goal: {'ee_frames' if goal_ee_frames else 'conf'}   active_bar={active_bar_rb!r}")

    variants = [
        ("both ON  (baseline)", False, False),
        ("both OFF (no CC1/CC2)", True, True),
        ("CC1 only (self ON, tool OFF)", False, True),
        ("CC2 only (self OFF, tool ON)", True, False),
    ]
    results = []
    for label, skip1, skip2 in variants:
        print(f"\n--- variant: {label} ---")
        jt = run_variant(
            planner, state.copy(), active_bar_rb, goal_conf, goal_ee_frames,
            skip_cc1=skip1, skip_cc2=skip2,
        )
        ok = jt is not None
        n = len(jt.points) if ok else 0
        print(f"    result: {'OK' if ok else 'FAIL'}  waypoints={n}")
        results.append((label, ok, n))

    print("\n=================== M2 CC TOGGLE SUMMARY ===================")
    for label, ok, n in results:
        print(f"  [{'OK  ' if ok else 'FAIL'}] {label:32s} waypoints={n}")
    print("============================================================")
    return 0


if __name__ == "__main__":
    import pybullet_planning as pp
    try:
        rc = main()
    finally:
        try:
            pp.disconnect()
        except Exception:
            pass
    raise SystemExit(rc)
