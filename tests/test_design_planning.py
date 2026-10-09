"""Planning on the core (`core.design_planning`): design -> scene_at -> CompasFabMirror -> tamp / compas_fab solvers.

The design is made reachable by construction: Cindy's flanges at a known joint configuration hold a ground bar, and
Alice's flange at a known configuration grasps it. Needs compas_fab and PyBullet (the core's `[mirrors]` extra) and
husky_assembly_tamp (the submodule, or HUSKY_TAMP naming a checkout); skipped otherwise. tamp runs its gradient IK.
"""

import os
import sys
from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pytest

from test_design_builder import ALICE, ALICE_FLANGE, CINDY, L, PKG, R, TCP_L, TCP_R, _design

REPO = Path(__file__).resolve().parents[1]
TAMP = Path(os.environ.get("HUSKY_TAMP", REPO / "external" / "husky_assembly_tamp"))
pytestmark = [
    pytest.mark.skipif(find_spec("compas_fab") is None or find_spec("pybullet") is None,
                       reason="compas_fab / PyBullet not installed"),
    pytest.mark.skipif(not (TAMP / "husky_assembly_tamp").is_dir(), reason="husky_assembly_tamp not found"),
]


@pytest.fixture(scope="module")
def setup():
    """A one-bar design whose keyframes are reachable, and a store for the mirrors."""
    os.environ["HUSKY_IK_BACKEND"] = "gradient"
    sys.path.insert(0, str(TAMP))
    from dataclasses import replace
    from bar_assembly_core.design import BodySpec
    from bar_assembly_core.design.plan_check import PART_SEATS
    from bar_assembly_core.geometry import CylinderShape, Geometry, Pose, compose, invert
    from bar_assembly_core.kinematics import link_pose
    from bar_assembly_core.urdf import UR_JOINT_NAMES
    from core import config
    from core.design_builder import BarRecord, HoldRecord, build_actions, grasp_of, z_axis

    from core.design_source import geometry_cache, robot_files
    design = _design()
    cache = geometry_cache({})
    design = replace(design, robots={rid: replace(spec, **dict(zip(("urdf", "srdf"), robot_files(
        cache, spec.name, spec.urdf, spec.srdf)))) for rid, spec in design.robots.items()})
    home = dict(zip([f"{s}_ur_arm_{j}" for s in ("left", "right") for j in UR_JOINT_NAMES],
                    (float(v) for v in config.HUSKY_DUAL_ARM_HOME_CONF_12)))
    reach = dict(home, left_ur_arm_elbow_joint=-1.9, right_ur_arm_elbow_joint=1.9)
    base = Pose()
    cindy_urdf = design.robots[CINDY].urdf
    tool0 = {f: link_pose(cindy_urdf, base, reach, f.split("/")[-1]) for f in (L, R)}
    seat = PART_SEATS["T20/Ground"]
    halves = {L: compose(compose(tool0[L], TCP_L), seat), R: compose(compose(tool0[R], TCP_R), seat)}
    a, b = (np.array(halves[f].position) for f in (L, R))
    axis = (b - a) / np.linalg.norm(b - a)
    from scipy.spatial.transform import Rotation
    bar_pose = Pose(tuple((a + b) / 2), tuple(Rotation.align_vectors([axis], [[0, 0, 1]])[0].as_quat()))
    cylinder = CylinderShape(0.01, float(np.linalg.norm(b - a)) + 0.1)
    bodies = {k: v for k, v in design.bodies.items() if k.startswith("ground/")}
    bodies["ground/WG0"] = replace(bodies["ground/WG0"], pose=Pose((0, 0, -2.0)))
    bodies["bars/B1"] = BodySpec("bars/B1", bar_pose, Geometry((cylinder,), (cylinder,)))
    for name, f in (("joints/G1-T20-0_ground", L), ("joints/G1-T20-1_ground", R)):
        bodies[name] = BodySpec(name, halves[f], design.bodies["joints/G1-T20-0_ground"].geometry,
                                part="T20/Ground", mount="bars/B1")
    design = replace(design, bodies=bodies, mates=frozenset({("ground/WG0", "joints/G1-T20-0_ground"),
                                                             ("ground/WG0", "joints/G1-T20-1_ground")}))
    record = BarRecord(bar="bars/B1", robot=CINDY, tools={L: "tools/AT3L", R: "tools/AT3R"}, tool0=tool0,
                       on={L: "joints/G1-T20-0_ground", R: "joints/G1-T20-1_ground"},
                       insert_direction=(0.0, 0.0, -1.0),
                       retreat_direction={f: tuple(-v for v in z_axis(halves[f])) for f in (L, R)},
                       base=base, home=home)
    alice_base = Pose((bar_pose.position[0], bar_pose.position[1] + 1.2, 0.0))
    alice_joints = {"ur_arm_shoulder_pan_joint": 1.5, "ur_arm_shoulder_lift_joint": -1.2, "ur_arm_elbow_joint": 1.2,
                    "ur_arm_wrist_1_joint": -1.5, "ur_arm_wrist_2_joint": -1.5, "ur_arm_wrist_3_joint": 0.0}
    alice_tool0 = link_pose(design.robots[ALICE].urdf, alice_base, alice_joints, "ur_arm_tool0")
    hold = HoldRecord(bar="bars/B1", robot=ALICE, flange=ALICE_FLANGE, tool="tools/alice/SupportGripper",
                      tool0=alice_tool0, grasp=grasp_of(alice_tool0, bar_pose), base=alice_base,
                      approach=alice_joints, held=alice_joints)
    schedule = (("J", "bars/B1"), ("H", "bars/B1"), ("R", "bars/B1"), ("HR", "bars/B1"))
    built = build_actions(design, schedule, {"bars/B1": record}, {"bars/B1": hold}).design
    store = {}
    yield built, store, reach, alice_tool0
    from core.design_planning import close_mirrors
    close_mirrors(store)


def test_keyframe_chain_solves_on_the_mirror(setup):
    """tamp's solve_keyframe_chain, given the mirror's planner and the design's scenes, reaches all three keyframes."""
    from husky_assembly_tamp.keyframe.ik_keyframe import solve_keyframe_chain
    from core.design_planning import keyframe_movements

    design, store, reach, _ = setup
    mirror, movements = keyframe_movements(design, store, "bars/B1")
    assert set(movements) == {"M1", "M2", "M3", "M4"} and movements["M4"].target_configuration is not None
    assert movements["M2"].start_state.rigid_body_states["bars/B1"].attached_to_link == "left_ur_arm_tool0"
    assert movements["M3"].start_state.rigid_body_states["bars/B1"].attached_to_link is None
    ordered = [(role, movements[role]) for role in ("M1", "M2", "M3")]
    with mirror.lend() as planner:
        solved = solve_keyframe_chain(planner, ordered, np.eye(4), check_collision=False)
    assert solved is not None
    cfg = solved["M2"].robot_configuration
    from bar_assembly_core.kinematics import link_pose
    from bar_assembly_core.geometry import Pose
    joints = dict(zip(cfg.joint_names, cfg.joint_values))
    flange = link_pose(design.robots["robots/cindy"].urdf, Pose(), joints, "left_ur_arm_tool0")
    target = link_pose(design.robots["robots/cindy"].urdf, Pose(), reach, "left_ur_arm_tool0")
    assert np.allclose(flange.position, target.position, atol=1e-3)


def test_geometry_unchanged_keeps_the_cell(setup):
    """A second sync of the same design reuses the mirror's cell (no rebuild)."""
    from core.design_planning import keyframe_movements

    design, store, _, _ = setup
    mirror, _ = keyframe_movements(design, store, "bars/B1")
    cell = mirror.cell
    keyframe_movements(design, store, "bars/B1")
    assert mirror.cell is cell


def test_support_ik_and_release_check(setup):
    """Alice's hold: compas_fab IK on her mirror reaches the grasp; the release check runs in its own scene."""
    from compas_fab.robots import FrameTarget, TargetMode
    from bar_assembly_core.mirrors.compas import frame_from_pose
    from core.design_planning import hold_solve_state, release_check

    design, store, _, alice_tool0 = setup
    mirror, state = hold_solve_state(design, store, "bars/B1")
    assert state.rigid_body_states["bars/B1"].is_hidden  # the gripper wraps the held bar
    target = FrameTarget(frame_from_pose(alice_tool0), TargetMode.ROBOT, tolerance_position=1e-4,
                         tolerance_orientation=1e-3)
    with mirror.lend() as planner:
        cfg = planner.inverse_kinematics(target, state, group="manipulator",
                                         options={"max_results": 20, "check_collision": False})
    assert cfg is not None
    verdict = release_check(design, store, "bars/B1")
    assert verdict.startswith(("OK", "collides")), verdict
