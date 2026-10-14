"""The distance-only reach check: never rejects a pose the arm can take, rejects a base that is plainly too far."""

from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "asset/husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

pytestmark = pytest.mark.skipif(not URDF.is_file(), reason="husky_urdf submodule not checked out")


@pytest.fixture(scope="module")
def reach():
    from core.reach_check import arm_reach
    return {side: arm_reach(URDF, f"{side}_ur_arm_tool0") for side in ("left", "right")}


def test_reach_is_a_ur5e(reach):
    for pivot, limit in reach.values():
        assert 0.9 < limit < 1.1          # stretched out, shoulder to flange
        assert 0.3 < pivot[2] < 0.9       # the shoulder sits on the husky, above the ground


def test_no_pose_the_arm_takes_is_out_of_reach(reach):
    from scaffolding_assembly_core.geometry import Pose
    from scaffolding_assembly_core.kinematics import link_pose
    from core.reach_check import MARGIN_M, out_of_reach

    rng = np.random.default_rng(1)
    base = np.eye(4)
    base[:3, 3] = (1200.0, -340.0, 15.0)
    world = Pose((1.2, -0.34, 0.015))
    farthest = 0.0
    for _ in range(300):
        joints = {f"{side}_ur_arm_{j}": v for side in reach for j, v in zip(JOINTS, rng.uniform(-np.pi, np.pi, 6))}
        targets = {"M1": {side: link_pose(URDF, world, joints, f"{side}_ur_arm_tool0").position for side in reach}}
        assert out_of_reach(base, reach, targets) == []
        pivot, limit = reach["left"]
        farthest = max(farthest, np.linalg.norm(np.array(targets["M1"]["left"]) - (np.array(pivot) + world.position)))
    assert farthest < reach["left"][1] - MARGIN_M + 1e-6   # the bound holds without its margin too


def test_a_base_too_far_away_is_rejected(reach):
    from core.reach_check import out_of_reach

    targets = {"M1": {"left": (0.0, 0.2, 0.6), "right": (0.0, -0.2, 0.6)}, "M2": {"left": (0.0, 0.2, 0.6)}}
    near, far = np.eye(4), np.eye(4)
    near[:3, 3] = (-500.0, 0.0, 0.0)
    far[:3, 3] = (-1600.0, 0.0, 0.0)
    assert out_of_reach(near, reach, targets) == []
    lines = out_of_reach(far, reach, targets)
    assert len(lines) == 3 and lines[0].startswith("M1 left arm: target 1.")
