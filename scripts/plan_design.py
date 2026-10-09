"""Plan Cindy's movements of a schema 2 design folder with tamp, through the core, and write `solutions/`.

    python scripts/plan_design.py <design folder> [--bars B3,B10] [--free-time 10] [--transfer-time 30]

Headless (not a Rhino command): design -> `scene_at` -> `CompasFabMirror` -> tamp's motion API -> the core's
`write_solution`. It replaces tamp's `headless_bar_action_planner.py` for schema 2 designs:

- transfer: `plan_constrained_dual_arm` (its start is left open by the design, tamp derives it);
- insert: `plan_constrained_dual_arm_linear`; retreat: `plan_dual_arm_linear_independent`; home: `plan_free_dual_arm`;
- the design wins: each trajectory's last waypoint is snapped to the design's target joints;
- movements no arm moves in (grips, drives, the operator's mount) are `solved` and still
  (`MovementResult.still`), at the joints the robot has there;
- the load (target null) and every support-robot movement stay `not_planned`.

Needs the core's `[mirrors]` stack and husky_assembly_tamp (`external/husky_assembly_tamp`, or `--tamp`).
`HUSKY_IK_BACKEND=gradient` runs without the ssik sidecar.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import traceback

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import core_lib  # noqa: E402,F401  (puts bar_assembly_core on sys.path)
from core.design_planning import close_mirrors, mirror_for  # noqa: E402
from bar_assembly_core.design import read  # noqa: E402
from bar_assembly_core.design.carry import assumed_start_all  # noqa: E402
from bar_assembly_core.design.scenes import seeded  # noqa: E402
from bar_assembly_core.design.solutions import (MovementResult, Planner, Solution, Trajectory,  # noqa: E402
                                                solved_against, write_solution)

#: Movement name suffix -> what plans it.
PLANNED = ("J_transfer", "J_insert", "R_retreat", "R_home")


def _rows(points) -> tuple:
    """Waypoints as rows of floats."""
    return tuple(tuple(float(v) for v in q) for q in points)


def plan_movement(api, mirror, design, action, m, names, seeds, args, joints_now=None):
    """Plan one movement on the mirror. Returns a MovementResult.

    Args:
        joints_now: The robot's joints when the movement starts, if known (the previous result's end): a movement no
            arm moves in is still there.
    """
    robot = action.robot
    kind = m.id[len(action.bar.split("/")[1]) + 1:]
    if not m.arms:
        # * The design wins: its start joints; else where the previous movement's solution ended.
        here = (m.start.robots[robot].joints if m.start.robots.get(robot) else None) or joints_now
        if not here:
            return MovementResult("not_planned", reason="no arm motion, and the robot's joints are unknown here")
        return MovementResult.still({robot: dict(here)})
    if kind not in PLANNED:
        return MovementResult("not_planned", reason="target null: planned live")
    goal = (m.target.joints or {}).get(robot) if m.target else None
    if not goal:
        return MovementResult("failed", reason="the design gives no target joints (keyframes not solved)")
    start = m.start.robots[robot]
    seed = None
    if start.joints is None:
        seed, _source = seeds.get((action.id, m.id), (None, None))
        if kind != "J_transfer" or seed is None:
            return MovementResult("failed", reason="the design gives no start joints")
    scene = design.scene_at(m)
    seeded(scene, robot, seed)
    mirror.sync(scene)
    if mirror.parked:
        print(f"{m.id}: {', '.join(mirror.parked)} parked (base or joints not solved yet)")
    conf = [float(goal[n]) for n in names]
    with mirror.lend() as planner:
        state = mirror.state
        if kind == "J_transfer":
            path, info = api.plan_constrained_dual_arm(planner, state, active_bar_id=action.bar, goal_conf=conf,
                                                       derive_start=True, max_time=args.transfer_time)
            rows = _rows(path) if path is not None else None
            reason = None if path is not None else str(info.get("failure_reason"))
        elif kind == "J_insert":
            jt = api.plan_constrained_dual_arm_linear(planner, state, active_bar_id=action.bar, goal_conf=conf)
            rows, reason = _trajectory_rows(jt, names), "plan_constrained_dual_arm_linear returned None"
        elif kind == "R_retreat":
            jt = api.plan_dual_arm_linear_independent(planner, state, goal_conf=conf)
            rows, reason = _trajectory_rows(jt, names), "plan_dual_arm_linear_independent returned None"
        else:
            path, info = api.plan_free_dual_arm(planner, state, conf, max_time=args.free_time)
            rows = _rows(path) if path is not None else None
            reason = None if path is not None else str(info.get("failure_reason"))
    if not rows:
        return MovementResult("failed", reason=reason or "no path")
    # * The design wins: the last waypoint is the design's target (the planner reached it within its tolerance).
    rows = rows[:-1] + (tuple(conf),)
    return MovementResult("solved", start={robot: dict(zip(names, rows[0]))}, end={robot: dict(zip(names, rows[-1]))},
                          trajectory=Trajectory(robot, tuple(names), rows))


def _trajectory_rows(jt, names):
    """A compas JointTrajectory as rows in `names` order; None for no trajectory."""
    if jt is None:
        return None
    return tuple(tuple(float(dict(zip(p.joint_names, p.joint_values))[n]) for n in names) for p in jt.points)


def main(argv=None) -> int:
    """Plan and write solutions; returns the number of failed movements."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("design")
    parser.add_argument("--bars", default="", help="comma list of bar ids, e.g. B3,B10; default all")
    parser.add_argument("--tamp", default=os.path.join(os.path.dirname(SCRIPT_DIR), "external", "husky_assembly_tamp"))
    parser.add_argument("--free-time", type=float, default=10.0)
    parser.add_argument("--transfer-time", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.tamp not in sys.path:
        sys.path.insert(0, args.tamp)
    from husky_assembly_tamp.motion_planner import api

    design = read(args.design)
    bars = {f"bars/{b.strip()}" for b in args.bars.split(",") if b.strip()}
    seeds = assumed_start_all(design)
    store, failed = {}, 0
    planner_block = Planner(name="husky_assembly_tamp", repo="husky_assembly_tamp",
                            ik_backend=os.environ.get("HUSKY_IK_BACKEND", "ssik"),
                            settings={"free_max_time": args.free_time, "transfer_max_time": args.transfer_time})
    try:
        for action_id in design.schedule:
            action = design.actions[action_id]
            if action.type not in ("bar_jointing", "bar_release") or (bars and action.bar not in bars):
                continue
            mirror = mirror_for(store, action.robot)
            names = None
            results = {}
            joints_now = None
            for m in action.movements:
                started = time.monotonic()
                try:
                    if names is None:
                        mirror.sync(_any_plannable(design, m, action.robot))
                        names = list(api._arm_joint_names(mirror.cell)[0]) + list(api._arm_joint_names(mirror.cell)[1])
                    result = plan_movement(api, mirror, design, action, m, names, seeds, args, joints_now)
                except Exception as exc:  # noqa: BLE001 -- one movement must not stop the run
                    traceback.print_exc()
                    result = MovementResult("failed", reason=f"{type(exc).__name__}: {exc}")
                failed += result.status == "failed"
                results[m.id] = result
                joints_now = result.end.get(action.robot) if result.status == "solved" else None
                print(f"{m.id}: {result.status} in {time.monotonic() - started:.1f} s"
                      + (f" ({result.reason})" if result.reason else ""))
            path = write_solution(design, Solution(action_id, solved_against(design, action_id), planner_block,
                                                   results))
            print(f"wrote {path}")
    finally:
        close_mirrors(store)
    return failed


def _any_plannable(design, m, robot):
    """A plannable scene of one movement, only to build the cell and read the arm joint names."""
    scene = design.scene_at(m)
    seeded(scene, robot)
    return scene


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
