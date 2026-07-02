"""Diagnose M2 (linear mate) waypoint-1 LEFT-IK failure on the half-solved action.

Reconstructs M2's interpolated tool0 frames exactly as
``plan_constrained_dual_arm_linear`` does (start = M2's propagated start = M1's
planned end), then attacks waypoint 1's LEFT target with several IK variants to
split the cause:

  * repro    : max_results=20, check_collision on, env (CC3/4/5) skipped
  * no-collision : same, but collision off        -> if this solves, the
                   blocker is CC1/CC2 (self / robot-vs-tool), not reachability
  * hard-refute  : max_results=1 + verbose         -> re-raises the collision
                   error if a converged solution merely collided
  * reach probe  : collision off, max_results=60   -> is the frame reachable AT
                   ALL from any restart?

Also prints the Cartesian step size (waypoint 0 -> 1) and, on success, the
joint delta from the warm seed (small => warm start stayed on-branch).

Run:
    C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe tests/_probe_m2_waypoints.py
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from _rhino_env_bootstrap import bootstrap_rhino_site_envs

bootstrap_rhino_site_envs(verbose=False)

import numpy as np
import pybullet_planning as pp
from compas.data import json_load
from compas.geometry import Frame, Transformation

# Import the headless module first: its module-level sys.path setup adds
# external/compas_fab/src (and the tamp package), so compas_fab imports below
# resolve.
import headless_bar_action_planner as H  # noqa: E402
from compas_fab.backends import CollisionCheckError, InverseKinematicsError  # noqa: E402
from compas_fab.backends.pybullet.exceptions import PlanningGroupNotSupported  # noqa: E402
from compas_fab.robots import FrameTarget, TargetMode  # noqa: E402
from compas_fab.backends.pybullet.backend_features.pybullet_plan_cartesian_motion import (  # noqa: E402
    FrameInterpolator,
)
from husky_assembly_tamp.motion_planner.api import (  # noqa: E402
    LEFT_GROUP, TOOL_LINK_LEFT, TOOL_LINK_RIGHT,
    _arm_joint_names, _bar_body_id, _conf12_from_state, _fk_link_frame,
)


def _left_target(frame):
    return FrameTarget(frame, target_mode=TargetMode.ROBOT,
                       tolerance_position=0.001, tolerance_orientation=0.01)


def _try_ik(planner, state, frame, jn12, left_names, *, opts, label):
    """Solve LEFT IK for `frame` from `state`; report outcome + delta from seed."""
    seed = np.asarray([float(state.robot_configuration[n]) for n in jn12])
    try:
        conf = planner.inverse_kinematics(_left_target(frame), state.copy(), LEFT_GROUP, opts)
    except (InverseKinematicsError, CollisionCheckError, PlanningGroupNotSupported) as e:
        print(f"    {label:16s}: FAIL ({type(e).__name__}: {getattr(e,'message',e)})")
        return
    sol = np.asarray([float(conf[n]) for n in jn12])
    dleft = float(np.abs(sol[:6] - seed[:6]).max())
    print(f"    {label:16s}: OK   left-arm |d| from seed = {dleft:.3f} rad")


def main() -> int:
    data_root = os.path.abspath(H.DEFAULT_DATA_ROOT)
    pd = os.path.join(data_root, H.DEFAULT_PROBLEM)
    solved = H.solved_action_path(os.path.join(pd, "BarActions", H.DEFAULT_BAR_ACTION))
    if not os.path.isfile(solved):
        print(f"[X] no half-solved file: {solved}")
        return 1
    rcell = json_load(os.path.join(pd, "RobotCell.json"))
    action = json_load(solved)
    active = getattr(action, "active_bar_id", "") or ""
    bar_rb = active if active in rcell.rigid_body_models else f"bar_{active}"

    _c, planner = H.start_planner(rcell, use_gui=False)
    m2 = H.select_movement(action, "M2")
    state = m2.start_state
    if state is None or state.robot_configuration is None:
        print("[X] M2 has no propagated start conf in the half-solved file.")
        return 2
    goal_ee = m2.target_ee_frames or None
    if not goal_ee:
        print("[X] M2 has no target_ee_frames.")
        return 2

    left_names, right_names = _arm_joint_names(rcell)
    jn12 = left_names + right_names

    # --- reconstruct M2's interpolated frames (mirror plan_constrained_dual_arm_linear) ---
    planner.set_robot_cell_state(state)
    start_left = _fk_link_frame(planner, state, TOOL_LINK_LEFT)
    start_right = _fk_link_frame(planner, state, TOOL_LINK_RIGHT)
    planner.set_robot_cell_state(state)
    bar_pos, bar_quat = pp.get_pose(_bar_body_id(planner, bar_rb))
    start_bar = Frame.from_quaternion([bar_quat[3], bar_quat[0], bar_quat[1], bar_quat[2]], point=list(bar_pos))
    inv_bar = Transformation.from_frame(start_bar).inverse()
    bar_from_L = Frame.from_transformation(inv_bar * Transformation.from_frame(start_left))
    bar_from_R = Frame.from_transformation(inv_bar * Transformation.from_frame(start_right))
    goal_bar = Frame.from_transformation(
        Transformation.from_frame(goal_ee["left"]) * Transformation.from_frame(bar_from_L).inverse()
    )
    interp = FrameInterpolator(start_bar, goal_bar, {"max_step_distance": 0.005, "max_step_angle": 0.05})
    N = max(2, interp.regular_interpolation_steps + 1)
    left_frames, right_frames = [], []
    for i in range(N):
        t = i / (N - 1)
        bt = Transformation.from_frame(interp.get_interpolated_frame(t))
        left_frames.append(Frame.from_transformation(bt * Transformation.from_frame(bar_from_L)))
        right_frames.append(Frame.from_transformation(bt * Transformation.from_frame(bar_from_R)))

    step = float(np.linalg.norm(np.asarray(left_frames[1].point) - np.asarray(left_frames[0].point)))
    fk0 = float(np.linalg.norm(np.asarray(left_frames[0].point) - np.asarray(start_left.point)))
    print(f"\n[frames] N={N}  waypoint0->1 LEFT step = {step*1000:.2f} mm")
    print(f"[frames] left_frames[0] vs FK(start) left  = {fk0*1000:.3f} mm  (should be ~0)")
    print(f"[frames] bar start->goal = {np.linalg.norm(np.asarray(goal_bar.point)-np.asarray(start_bar.point))*1000:.1f} mm")

    base = {"max_descend_iterations": 200, "return_full_configuration": True, "verbose": False}
    env_skip = {"_skip_cc3": True, "_skip_cc4": True, "_skip_cc5": True}

    print("\n[waypoint 1 LEFT IK] seeded from M2 start (warm):")
    _try_ik(planner, state, left_frames[1], jn12, left_names,
            opts={**base, "max_results": 20, "check_collision": True, **env_skip}, label="repro")
    _try_ik(planner, state, left_frames[1], jn12, left_names,
            opts={**base, "max_results": 20, "check_collision": False}, label="no-collision")
    _try_ik(planner, state, left_frames[1], jn12, left_names,
            opts={**base, "max_results": 60, "check_collision": False}, label="reach x60")
    _try_ik(planner, state, left_frames[1], jn12, left_names,
            opts={**base, "max_results": 1, "check_collision": True, "verbose": True, **env_skip},
            label="hard-refute")
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
