"""Interactive visualizer test for ``build_cfab_pose_collision_fn``.

Runs four scenarios on the M1 start state of a real BarAction:

  1. **Clean** — sample the bar at its own (start) pose; expect False.
  2. **Bar vs bar** — sample at another built bar's world pose; expect True.
  3. **Bar vs unrelated joint** — sample at a joint NOT in the bar's
     touch_bodies list; expect True.
  4. **Bar inside robot body** — sample at a pose under the husky base
     that visibly intersects the robot; expect False, because the pose-space
     check intentionally skips CC.3 (robot-link vs rigid-body).

For each scenario:
  - draws the sampled bar pose with ``pp.draw_pose`` so you can visually
    compare with where pybullet places the bar mesh after the closure's
    state override (they must coincide),
  - applies the state override to the live planner (same math as the
    closure) so the bar visibly snaps to the sampled pose,
  - calls the closure to get True/False and prints PASS / FAIL.

Caveat: joints in this cell attach to robot tool0 links, NOT to the bar.
When the bar is moved to a sampled pose, joints stay anchored to the
robot. Their cfab collisions therefore reflect the joints' current
tool0-anchored positions, not their hypothetical positions had they
followed the bar. That's fine for the pose-space pre-reject layer.

CLI:
    python tests/viz_pose_collision_fn.py
        [<data_root>]
        --problem 2026-05-14_foc_demo_reduced
        --bar-action B226.json
        --gui                       # required — this is a viz test
        [--scenario {1,2,3,4,all}]  # default all
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from typing import List, Optional


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
CFAB_SRC = os.path.join(REPO_ROOT, "external", "compas_fab", "src")
TAMP_SRC = os.path.join(REPO_ROOT, "external", "husky_assembly_tamp")
RSDS_SRC = os.path.join(REPO_ROOT, "external", "rs_data_structure")

for _p in (SCRIPTS_DIR, TESTS_DIR, CFAB_SRC, TAMP_SRC, RSDS_SRC):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)


DEFAULT_DATA_ROOT = (
    r"C:\Users\yijiangh\Insync\yijiang94817@gmail.com"
    r"\Google Drive - Shared with me\2025-03 Husky Assembly\data_design_study"
)
DEFAULT_PROBLEM = "2026-05-14_foc_demo_reduced"
DEFAULT_BAR_ACTION = "B226.json"

LEFT_GROUP = "base_left_arm_manipulator"
RIGHT_GROUP = "base_right_arm_manipulator"

_ROLE_RE = re.compile(r"_M([0-9])_")


def match_role(mv) -> Optional[str]:
    mid = getattr(mv, "movement_id", None) or ""
    m = _ROLE_RE.search(mid)
    return f"M{m.group(1)}" if m else None


def resolve_bar_rb_name(cell, bar_id: str) -> Optional[str]:
    for cand in (bar_id, f"bar_{bar_id}"):
        if cand in cell.rigid_body_models:
            return cand
    return None


def start_planner(rcell, *, use_gui: bool):
    from compas_fab.backends import PyBulletClient, PyBulletPlanner
    import pybullet_planning as pp

    client = PyBulletClient(connection_type="gui" if use_gui else "direct", verbose=True)
    client.__enter__()
    pp.set_client(client.client_id)
    pp.CLIENTS[client.client_id] = True if use_gui else None
    planner = PyBulletPlanner(client)
    t0 = time.time()
    with pp.LockRenderer(False):
        planner.set_robot_cell(rcell)
    print(f"[pb] set_robot_cell: {time.time() - t0:.2f}s")
    return client, planner


def fill_missing_config(state, rcell, home_left, home_right) -> None:
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


def pp_pose_from_frame(frame):
    return (list(frame.point), list(frame.quaternion.xyzw))


def color_rigid_body(planner, rb_name, rgba):
    """Re-tint every pybullet sub-body of ``rb_name`` to ``rgba``.

    Visual-only — does not affect collision. The change persists across
    ``set_robot_cell_state`` calls (cfab only updates pose, not visual props).
    """
    import pybullet
    cid = planner.client.client_id
    for body_id in planner.client.rigid_bodies_puids[rb_name]:
        pybullet.changeVisualShape(
            body_id, -1, rgbaColor=list(rgba),
            physicsClientId=cid,
        )


def apply_bar_pose_to_planner(planner, start_state, active_bar_id, world_from_bar_sample):
    """Same attach_frame override the closure does, but applied to the live
    planner so the GUI shows the bar at the sampled pose."""
    import pybullet_planning as pp
    from compas.geometry import Frame

    attached_link_name = start_state.rigid_body_states[active_bar_id].attached_to_link
    robot_puid = planner.client.robot_puid
    attached_link_id = planner.client.robot_link_puids[attached_link_name]
    link_pose_world = pp.get_link_pose(robot_puid, attached_link_id)
    inv_link = pp.invert(link_pose_world)
    attach_pose = pp.multiply(inv_link, world_from_bar_sample)
    attach_frame = Frame.from_quaternion(
        [attach_pose[1][3], attach_pose[1][0], attach_pose[1][1], attach_pose[1][2]],
        point=list(attach_pose[0]),
    )
    s = start_state.copy()
    s.rigid_body_states[active_bar_id].attachment_frame = attach_frame
    planner.set_robot_cell_state(s)


def collision_pairs_for(planner, start_state, active_bar_id, world_from_bar_sample) -> List[str]:
    """Run the same check the closure does but with full_report=True; return
    a list of human-readable pair descriptions for the printout."""
    from compas_fab.backends import CollisionCheckError
    import pybullet_planning as pp
    from compas.geometry import Frame

    attached_link_name = start_state.rigid_body_states[active_bar_id].attached_to_link
    robot_puid = planner.client.robot_puid
    attached_link_id = planner.client.robot_link_puids[attached_link_name]
    link_pose_world = pp.get_link_pose(robot_puid, attached_link_id)
    inv_link = pp.invert(link_pose_world)
    attach_pose = pp.multiply(inv_link, world_from_bar_sample)
    attach_frame = Frame.from_quaternion(
        [attach_pose[1][3], attach_pose[1][0], attach_pose[1][1], attach_pose[1][2]],
        point=list(attach_pose[0]),
    )
    s = start_state.copy()
    s.rigid_body_states[active_bar_id].attachment_frame = attach_frame
    opts = {
        "_skip_cc1": True, "_skip_cc2": True, "_skip_cc3": True,
        "_skip_cc4": False, "_skip_cc5": True,
        "full_report": True, "verbose": False,
    }
    try:
        planner.check_collision(s, options=opts)
    except CollisionCheckError as exc:
        out = []
        for a, b in (getattr(exc, "collision_pairs", []) or []):
            an = getattr(a, "name", repr(a))
            bn = getattr(b, "name", repr(b))
            out.append(f"{an} <-> {bn}")
        return out
    return []


def pick_scenario_targets(start_state, active_bar_id):
    """Pick (bar_target_rb, joint_target_rb) from the cell to use for
    scenarios 2 and 3.

    Returns (other_bar_name, other_bar_world_pose, joint_name, joint_world_pose).
    Each "*_world_pose" is a pybullet (pos, quat_xyzw) pair.
    """
    rbs_map = start_state.rigid_body_states
    touch_bodies = set(rbs_map[active_bar_id].touch_bodies or [])

    other_bar_name = None
    other_bar_pose = None
    for name, rbs in rbs_map.items():
        if not name.startswith("bar_"):
            continue
        if name == active_bar_id:
            continue
        if rbs.frame is None:
            continue
        other_bar_name = name
        other_bar_pose = pp_pose_from_frame(rbs.frame)
        break

    joint_name = None
    joint_pose = None
    for name, rbs in rbs_map.items():
        if not name.startswith("joint_"):
            continue
        if name in touch_bodies:
            continue
        if rbs.frame is None:
            continue
        joint_name = name
        joint_pose = pp_pose_from_frame(rbs.frame)
        break

    return other_bar_name, other_bar_pose, joint_name, joint_pose


def run_scenario(idx, label, sample_pose, expected_collision, *, planner, start_state,
                 active_bar_id, pose_collision_fn, wait_for_user: bool):
    import pybullet_planning as pp

    print(f"\n=== Scenario {idx}: {label} ===")
    print(f"  sample pose: pos={sample_pose[0]}, quat_xyzw={sample_pose[1]}")

    # Move bar to sampled pose in the GUI so the user can see it.
    apply_bar_pose_to_planner(planner, start_state, active_bar_id, sample_pose)
    # Draw the raw sampled pose axes (should coincide with the bar mesh).
    pp.draw_pose(sample_pose, length=0.3)

    # Run the closure.
    is_colliding = pose_collision_fn(sample_pose)
    if is_colliding:
        pairs = collision_pairs_for(planner, start_state, active_bar_id, sample_pose)
        pair_str = "; ".join(pairs[:5]) if pairs else "<no pairs>"
        print(f"  pose_collision_fn -> True  ({pair_str})")
    else:
        print(f"  pose_collision_fn -> False")

    verdict = "PASS" if is_colliding == expected_collision else "FAIL"
    print(f"  expected={expected_collision}, got={is_colliding} -> {verdict}")

    if wait_for_user:
        try:
            input("  Press <enter> in this terminal to proceed to the next scenario...")
        except EOFError:
            pass

    return verdict == "PASS"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("data_root", nargs="?", default=DEFAULT_DATA_ROOT)
    parser.add_argument("--problem", default=DEFAULT_PROBLEM)
    parser.add_argument("--bar-action", default=DEFAULT_BAR_ACTION)
    parser.add_argument("--gui", action="store_true",
                        help="REQUIRED — this is a viz test.")
    parser.add_argument("--scenario", default="all",
                        choices=("1", "2", "3", "4", "all"))
    parser.add_argument("--no-wait", action="store_true",
                        help="Don't pause between scenarios.")
    parser.add_argument("--cell", default=None)
    args = parser.parse_args()

    if not args.gui:
        print("[X] --gui is required for the viz test.")
        return 1

    data_root = os.path.abspath(args.data_root)
    problem_dir = os.path.join(data_root, args.problem)
    action_path = os.path.join(problem_dir, "BarActions", args.bar_action)
    cell_path = args.cell or os.path.join(problem_dir, "RobotCell.json")
    for p in (data_root, problem_dir, action_path, cell_path):
        if not os.path.exists(p):
            print(f"[X] missing: {p}")
            return 1

    from compas.data import json_load
    import pybullet_planning as pp

    print(f"[load] RobotCell <- {cell_path}")
    rcell = json_load(cell_path)
    print(f"[load] BarAction <- {action_path}")
    action = json_load(action_path)
    active_bar_id_bare = getattr(action, "active_bar_id", "") or ""
    active_bar_id = resolve_bar_rb_name(rcell, active_bar_id_bare)
    if not active_bar_id:
        print(f"[X] active_bar_id {active_bar_id_bare!r} not in cell.")
        return 1
    print(f"  active_bar_id (rigid-body name): {active_bar_id}")

    # Pick M1.
    selected = None
    for mv in action.movements:
        if match_role(mv) == "M1":
            selected = mv
            break
    if selected is None:
        print("[X] no M1 movement in this BarAction.")
        return 1
    print(f"[pick] M1 -> {type(selected).__name__} {selected.movement_id}")

    from core import config as _config

    print(f"\n[pb] starting PyBullet GUI")
    _client, planner = start_planner(rcell, use_gui=True)

    rc = 0
    try:
        state = selected.start_state
        fill_missing_config(state, rcell, _config.HOME_CONFIG_LEFT, _config.HOME_CONFIG_RIGHT)
        with pp.LockRenderer(False):
            planner.set_robot_cell_state(state)
        # Tint the active bar vivid blue so it's easy to track across samples.
        color_rigid_body(planner, active_bar_id, rgba=(0.1, 0.4, 1.0, 1.0))

        # Build the real closure — imported from core.
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
            build_cfab_pose_collision_fn,
        )
        pose_collision_fn = build_cfab_pose_collision_fn(planner, state, active_bar_id)

        # Read the bar's current world pose (the "clean" sample).
        # active bar is attached to its link; read live pybullet pose for the bar body.
        bar_puids = planner.client.rigid_bodies_puids[active_bar_id]
        bar_body = bar_puids[0]
        clean_sample = pp.get_pose(bar_body)

        # Find a "built" bar + an unrelated joint from the cell.
        other_bar_name, other_bar_pose, joint_name, joint_pose = pick_scenario_targets(
            state, active_bar_id,
        )

        # Scenario 4 sample: place bar inside robot body at a fixed point under
        # the arms. Husky base is centered around the world origin; arms reach
        # ~0.5–1.5m up. (0, 0, 0.5) sits inside the husky chassis.
        robot_inside_sample = ((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))

        scenarios = []
        if args.scenario in ("1", "all"):
            scenarios.append((1, "clean (bar at current attached pose)", clean_sample, False))
        if args.scenario in ("2", "all"):
            if other_bar_pose is None:
                print("[skip] scenario 2: no other built bar with a frame in start_state.")
            else:
                scenarios.append((
                    2, f"bar vs bar ({other_bar_name})", other_bar_pose, True,
                ))
        if args.scenario in ("3", "all"):
            if joint_pose is None:
                print("[skip] scenario 3: no non-touch joint with a frame in start_state.")
            else:
                scenarios.append((
                    3, f"bar vs unrelated joint ({joint_name})", joint_pose, True,
                ))
        if args.scenario in ("4", "all"):
            scenarios.append((
                4, "bar inside robot body (CC.3 skipped -> expect no flag)",
                robot_inside_sample, False,
            ))

        all_pass = True
        for idx, label, sample, expected in scenarios:
            ok = run_scenario(
                idx, label, sample, expected,
                planner=planner, start_state=state,
                active_bar_id=active_bar_id,
                pose_collision_fn=pose_collision_fn,
                wait_for_user=not args.no_wait,
            )
            all_pass = all_pass and ok

        print(f"\n[viz] overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")
        rc = 0 if all_pass else 2
    finally:
        try:
            pp.disconnect()
        except Exception as exc:
            print(f"[pb] disconnect raised ({exc}); continuing.")

    return rc


if __name__ == "__main__":
    raise SystemExit(main())
