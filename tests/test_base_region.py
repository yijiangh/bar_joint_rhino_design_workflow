"""The base-region debug view's math: never rules out a base the arms really reach from, rules out far ones."""

from itertools import islice
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "asset/husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
SQUARE = (np.array([[0.0, 0.0, 10.0], [1000.0, 0.0, 10.0], [1000.0, 1000.0, 10.0], [0.0, 1000.0, 10.0]]),
          np.array([[0, 1, 2], [0, 2, 3]]))

needs_urdf = pytest.mark.skipif(not URDF.is_file(), reason="husky_urdf submodule not checked out")


@pytest.fixture(scope="module")
def reach():
    from core.base_region import wrist_reach
    return {side: wrist_reach(URDF, f"{side}_ur_arm_tool0") for side in ("left", "right")}


@needs_urdf
def test_wrist_point_is_behind_the_flange_and_tightens_the_reach(reach):
    from core.reach_check import arm_reach

    for side, (pivot, wrist_tip, limit) in reach.items():
        flange_pivot, flange_limit = arm_reach(URDF, f"{side}_ur_arm_tool0")
        assert np.allclose(pivot, flange_pivot)
        assert 0.05 < np.linalg.norm(wrist_tip) < 0.15   # one UR5e wrist length
        assert limit < flange_limit


@needs_urdf
def test_no_base_the_arms_reach_from_is_ruled_out(reach):
    from bar_assembly_core.geometry import Pose
    from bar_assembly_core.kinematics import link_pose
    from core.base_region import constraints, feasible_yaws

    rng = np.random.default_rng(1)
    for _ in range(200):
        yaw = rng.uniform(-np.pi, np.pi)
        world = Pose((1.2, -0.34, 0.015), (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)))
        joints = {f"{side}_ur_arm_{j}": v for side in reach for j, v in zip(JOINTS, rng.uniform(-np.pi, np.pi, 6))}
        targets = {"M1": {side: link_pose(URDF, world, joints, f"{side}_ur_arm_tool0").matrix() for side in reach}}
        ok = feasible_yaws([[1200.0, -340.0, 15.0]], [[0.0, 0.0, 1.0]], constraints(reach, targets), [yaw])
        assert ok.all()


@needs_urdf
def test_a_base_too_far_away_is_ruled_out(reach):
    from core.base_region import constraints, feasible_yaws, window

    target = np.eye(4)
    target[:3, 3] = (0.0, 0.0, 0.6)
    limits = constraints(reach, {"M2": {"left": target}})
    yaws = np.linspace(-np.pi, np.pi, 72, endpoint=False)
    ok = feasible_yaws([[-500.0, 0.0, 0.0], [-2500.0, 0.0, 0.0]], [[0.0, 0.0, 1.0]] * 2, limits, yaws)
    assert ok[0].any() and not ok[1].any()
    lower, upper = window(limits)
    assert (lower < -900.0).all() and (upper > 900.0).all() and (upper < 2500.0).all()


def test_widest_run_wraps_around_and_counts_separate_runs():
    from core.base_region import widest_run

    ok = np.array([[1, 1, 0, 0, 0, 0, 1, 1],    # one run across the wrap: columns 6, 7, 0, 1
                   [1, 1, 1, 0, 1, 0, 0, 0],    # two runs, the longer one first
                   [1, 1, 1, 1, 1, 1, 1, 1],
                   [0, 0, 0, 0, 0, 0, 0, 0]], dtype=bool)
    length, middle, runs = widest_run(ok)
    assert list(length) == [4, 3, 8, 0]
    assert middle[0] == 7.5 and middle[1] == 1.0
    assert list(runs) == [1, 2, 1, 0]


def test_base_frames_stand_on_the_ground_and_face_the_heading():
    from core.base_region import base_frames

    frames = base_frames([[100.0, 200.0, 30.0]] * 2, [[0.0, 0.0, 1.0]] * 2, [0.0, np.pi / 2.0])
    assert np.allclose(frames[:, :3, 3], (100.0, 200.0, 30.0)) and np.allclose(frames[:, :3, 2], (0.0, 0.0, 1.0))
    assert np.allclose(frames[0, :3, 0], (1.0, 0.0, 0.0)) and np.allclose(frames[1, :3, 0], (0.0, 1.0, 0.0))
    assert np.allclose(np.linalg.det(frames[:, :3, :3]), 1.0)


def test_extension_cost_is_zero_at_the_preferred_extension_and_one_at_both_ends():
    from core.base_region import extension_cost

    # One arm whose pivot is the base origin, reach 1 m: the cost depends on the distance to the wrist point only.
    up, yaws = [[0.0, 0.0, 1.0]] * 4, [0.0, 1.0]
    limits = [(np.zeros(3), np.array([0.0, 0.0, 0.0]), 1.0)]
    points = [[600.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 0.0], [0.0, -800.0, 0.0]]
    cost = extension_cost(points, up, limits, yaws, 0.6)
    assert np.allclose(cost[:, 0], [0.0, 1.0, 1.0, 0.25]) and np.allclose(cost[:, 0], cost[:, 1])


def test_extension_cost_stays_within_one_where_the_arms_reach(reach):
    from core.base_region import constraints, extension_cost, feasible_yaws

    target = np.eye(4)
    target[:3, 3] = (0.6, 0.0, 1.0)
    limits = constraints(reach, {"M1": {"left": target, "right": target}})
    rng = np.random.default_rng(2)
    points = np.column_stack([rng.uniform(-1500.0, 1500.0, (400, 2)), np.zeros(400)])
    up, yaws = np.tile([0.0, 0.0, 1.0], (400, 1)), np.linspace(-np.pi, np.pi, 36, endpoint=False)
    ok, cost = feasible_yaws(points, up, limits, yaws), extension_cost(points, up, limits, yaws, 0.65)
    assert ok.any() and (cost >= 0.0).all() and (cost[ok] <= 1.0 + 1e-9).all()


def test_base_search_proposes_usable_bases_apart_from_each_other(reach):
    from core.base_region import constraints, extension_cost, feasible_yaws
    from core.base_search import crowding, proposals

    def target(y):
        matrix = np.eye(4)
        matrix[:3, 3] = (0.0, y, 1.0)
        return matrix

    limits = constraints(reach, {"M1": {"left": target(0.35), "right": target(-0.35)}})
    ground = (np.array([[-3000.0, -3000.0, 0.0], [3000.0, -3000.0, 0.0], [3000.0, 3000.0, 0.0], [-3000.0, 3000.0, 0.0]]),
              np.array([[0, 1, 2], [0, 2, 3]]))
    up = np.array([0.0, 0.0, 1.0])

    def check(frame):  # a pillar of radius 0.4 m under the targets: the cheapest arm poses stand inside it
        gap = float(np.hypot(frame[0, 3], frame[1, 3])) / 1000.0 - 0.4
        return None if gap < 0.0 else min(gap, 0.3)

    def snap(point):
        inside = abs(point[0]) <= 3000.0 and abs(point[1]) <= 3000.0
        return (np.array([point[0], point[1], 0.0]), up) if inside else (None, None)

    lines = []
    options = dict(step_mm=50.0, yaw_steps=72, prefer=0.65, near_m=0.3, weight=0.25, spacing_mm=200.0,
                   max_checks=300, log=lines.append)
    plain = next(proposals([ground], limits, check, snap, refine_evals=0, **options))
    frames = list(islice(proposals([ground], limits, check, snap, refine_evals=40, **options), 3))
    assert len(frames) == 3 and len(lines) >= 4

    def cost(frame):
        yaw = [np.arctan2(frame[1, 0], frame[0, 0])]
        assert feasible_yaws(frame[None, :3, 3], up[None, :], limits, yaw)[0, 0] and check(frame) is not None
        return float(extension_cost(frame[None, :3, 3], up[None, :], limits, yaw, 0.65)[0, 0]
                     + crowding(check(frame), 0.3, 0.25))

    costs = [cost(frame) for frame in frames]
    assert costs[0] <= cost(plain) + 1e-9          # the descent never makes the grid cell worse
    assert np.hypot(*(frames[1][:2, 3] - frames[0][:2, 3])) >= 200.0 - 1e-6

    # Around a picked base: the first proposal descends from it, and all stay in its circle, facing its way.
    from core.base_region import base_frames
    from core.base_search import frame_yaw

    seed = base_frames([[-900.0, 100.0, 0.0]], [up], [0.2])[0]
    assert abs(frame_yaw(seed) - 0.2) < 1e-9
    near = list(islice(proposals([ground], limits, check, snap, refine_evals=40, seed_frame_mm=seed,
                                 radius_mm=600.0, facing_range=np.pi / 2.0, **options), 4))
    assert len(near) == 4 and "picked base" in lines[-len(near)]
    assert cost(near[0]) < cost(seed) and np.hypot(*(near[0][:2, 3] - seed[:2, 3])) < 300.0
    for frame in near:
        cost(frame)
        assert np.hypot(*(frame[:2, 3] - seed[:2, 3])) <= 600.0 + 1e-6
        assert abs((frame_yaw(frame) - 0.2 + np.pi) % (2.0 * np.pi) - np.pi) <= np.pi / 2.0 + 1e-9
    apart = [np.hypot(*(a[:2, 3] - b[:2, 3])) for i, a in enumerate(near) for b in near[:i]]
    assert min(apart) >= 200.0 - 1e-6


def test_ground_grid_keeps_the_points_over_the_ground():
    from core.base_region import ground_grid

    points, normals = ground_grid([SQUARE], np.array([-500.0, 250.0]), np.array([1500.0, 750.0]), 250.0)
    assert len(points) == 5 * 3                       # x = 0 .. 1000, y = 250 .. 750
    assert np.allclose(points[:, 2], 10.0) and np.allclose(normals, (0.0, 0.0, 1.0))
    assert points[:, 0].min() == 0.0 and points[:, 0].max() == 1000.0
