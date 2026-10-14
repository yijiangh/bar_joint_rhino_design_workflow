"""The schema 2 builder (`core.process_builder`) on a small three-bar process with a hold, with the real robot files.

Bars: B1 stands on the ground (two ground halves), B2 joins B1 and the ground and is held by Alice until B3 is
built, B3 joins B1 and B2. Nothing is solved (null bases and joints), as when designing in Rhino.
"""

import os
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from core import core_lib  # noqa: F401
from core.process_builder import BarRecord, HoldRecord, build_actions, grasp_of, z_axis
from bar_assembly_core.process import BodySpec, Process, RobotSpec, read, validate, write
from bar_assembly_core.process.plan_check import check_plan
from bar_assembly_core.process.version import writer_info
from bar_assembly_core.geometry import BoxShape, CylinderShape, Geometry, Pose, compose, invert
from bar_assembly_core.robot import Tool

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "asset" / "husky_urdf"
CINDY_URDF = PKG / "mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
CINDY_SRDF = PKG / "mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf"
ALICE_URDF = PKG / "mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf"
ALICE_SRDF = PKG / "mt_husky_moveit_config/config/husky.srdf"

pytestmark = pytest.mark.skipif(not CINDY_URDF.is_file(), reason="asset/husky_urdf submodule not checked out")

CINDY, ALICE = "robots/cindy", "robots/alice"
L, R = f"{CINDY}/left_ur_arm_tool0", f"{CINDY}/right_ur_arm_tool0"
ALICE_FLANGE = f"{ALICE}/ur_arm_tool0"
TCP_L, TCP_R = Pose((-0.07, 0.0, 0.08)), Pose((0.07, 0.0, 0.08))
WHEELS = ("front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link")
BOX = Geometry((BoxShape((0.02, 0.02, 0.02)),), (BoxShape((0.02, 0.02, 0.02)),))
#: Part -> the half's pose in the frame of the tool centre point on it (a body's `pose_in_tcp`).
POSE_IN_TCP = {"T20/Male": Pose(), "T20/Ground": Pose(orientation=(0.0, 0.0, 1.0, 0.0))}


def _bar(y: float, z: float) -> BodySpec:
    """A 1 m bar along world x at (0.5, y, z)."""
    pose = Pose((0.5, y, z), tuple(Rotation.from_euler("y", 90, degrees=True).as_quat()))
    cylinder = CylinderShape(0.01, 1.0)
    return BodySpec(id="", assembly_pose=pose, geometry=Geometry((cylinder,), (cylinder,)))


def _half(half_id: str, bar: str, part: str, position) -> BodySpec:
    """A half at `position`, its z axis pointing down (into the ground or the receiving bar)."""
    pose = Pose(tuple(position), tuple(Rotation.from_euler("x", 180, degrees=True).as_quat()))
    return BodySpec(id=half_id, assembly_pose=pose, geometry=BOX, part=part, mount=bar, pose_in_tcp=POSE_IN_TCP.get(part))


def _process() -> Process:
    """Robots, tools, bodies and mates; no actions yet."""
    bodies = {"ground/WG0": BodySpec("ground/WG0", Pose(), Geometry((BoxShape((5, 5, 0.01)),),
                                                                    (BoxShape((5, 5, 0.01)),)))}
    for bar, y, z in (("bars/B1", 0.0, 0.05), ("bars/B2", 0.3, 0.05), ("bars/B3", 0.15, 0.1)):
        bodies[bar] = replace(_bar(y, z), id=bar)
    halves = [("joints/G1-T20-0_ground", "bars/B1", "T20/Ground", (0.2, 0.0, 0.0)),
              ("joints/G1-T20-1_ground", "bars/B1", "T20/Ground", (0.8, 0.0, 0.0)),
              ("joints/G2-T20-0_ground", "bars/B2", "T20/Ground", (0.8, 0.3, 0.0)),
              ("joints/J1-2_female", "bars/B1", "T20/Female", (0.5, 0.0, 0.05)),
              ("joints/J1-2_male", "bars/B2", "T20/Male", (0.2, 0.3, 0.05)),
              ("joints/J1-3_female", "bars/B1", "T20/Female", (0.3, 0.0, 0.05)),
              ("joints/J1-3_male", "bars/B3", "T20/Male", (0.3, 0.05, 0.1)),
              ("joints/J2-3_female", "bars/B2", "T20/Female", (0.6, 0.3, 0.05)),
              ("joints/J2-3_male", "bars/B3", "T20/Male", (0.6, 0.25, 0.1))]
    bodies.update({h: _half(h, bar, part, position) for h, bar, part, position in halves})
    mates = frozenset(tuple(sorted(m)) for m in (
        ("ground/WG0", "joints/G1-T20-0_ground"), ("ground/WG0", "joints/G1-T20-1_ground"),
        ("ground/WG0", "joints/G2-T20-0_ground"), ("joints/J1-2_female", "joints/J1-2_male"),
        ("joints/J1-3_female", "joints/J1-3_male"), ("joints/J2-3_female", "joints/J2-3_male")))
    tools = {"tools/cindy/AT3L": Tool("tools/cindy/AT3L", BOX, TCP_L, "scaffolding_v3"),
             "tools/cindy/AT3R": Tool("tools/cindy/AT3R", BOX, TCP_R, "scaffolding_v3"),
             "tools/alice/SupportGripper": Tool("tools/alice/SupportGripper", BOX, Pose(), "robotiq")}
    robots = {CINDY: RobotSpec(CINDY, CINDY_URDF, CINDY_SRDF, "0806",
                               {"left_ur_arm_tool0": "tools/cindy/AT3L", "right_ur_arm_tool0": "tools/cindy/AT3R"}, WHEELS),
              ALICE: RobotSpec(ALICE, ALICE_URDF, ALICE_SRDF, "0804", {"ur_arm_tool0": "tools/alice/SupportGripper"},
                               WHEELS)}
    return Process(None, writer_info(), robots, tools, bodies, (), {}, mates)


def _record(process: Process, bar: str, left_half: str, right_half: str, **keyframes) -> BarRecord:
    """Cindy's record for a bar: each flange placed so its TCP sits on its half at the half's `pose_in_tcp`."""
    tool0, retreat = {}, {}
    for flange, half, tcp in ((L, left_half, TCP_L), (R, right_half, TCP_R)):
        seat = process.bodies[half].pose_in_tcp
        tool0[flange] = compose(process.bodies[half].assembly_pose, invert(compose(tcp, seat)))
        retreat[flange] = tuple(-v for v in z_axis(process.bodies[half].assembly_pose))
    return BarRecord(bar=bar, robot=CINDY, tools={L: "tools/cindy/AT3L", R: "tools/cindy/AT3R"}, tool0=tool0,
                     on={L: left_half, R: right_half}, insert_direction=(0.0, 0.0, -1.0), retreat_direction=retreat,
                     ground=("ground/WG0",), **keyframes)


def _records(process: Process):
    bars = {"bars/B1": _record(process, "bars/B1", "joints/G1-T20-0_ground", "joints/G1-T20-1_ground"),
            "bars/B2": _record(process, "bars/B2", "joints/J1-2_male", "joints/G2-T20-0_ground"),
            "bars/B3": _record(process, "bars/B3", "joints/J1-3_male", "joints/J2-3_male")}
    tool0 = compose(process.bodies["bars/B2"].assembly_pose, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
    holds = {"bars/B2": HoldRecord(bar="bars/B2", robot=ALICE, flange=ALICE_FLANGE, tool="tools/alice/SupportGripper",
                                   tool0=tool0, grasp=grasp_of(tool0, process.bodies["bars/B2"].assembly_pose),
                                   supports_until=("bars/B3",), ground=("ground/WG0",))}
    return bars, holds


SCHEDULE = (("J", "bars/B1"), ("R", "bars/B1"), ("J", "bars/B2"), ("H", "bars/B2"), ("R", "bars/B2"),
            ("J", "bars/B3"), ("R", "bars/B3"), ("HR", "bars/B2"))


def test_unsolved_process_passes_file_and_plan_checks(tmp_path):
    """Built, written and read back: A passes, B has no errors; every action has its movements in order."""
    process = _process()
    bars, holds = _records(process)
    result = build_actions(process, SCHEDULE, bars, holds)
    assert not result.problems
    validate(result.process, check_robot_meshes=False)
    written = write(result.process, tmp_path / "process", package_dirs=[PKG])
    assert written.schedule == ("B1_J", "B1_R", "B2_J", "B2_H", "B2_R", "B3_J", "B3_R", "B2_HR")
    report = check_plan(written)
    assert report.ok, report.errors
    assert [m.id for m in written.actions["B2_J"].movements] == [
        "B2_J_load", "B2_J_place_bar", "B2_J_grasp", "B2_J_transfer", "B2_J_insert"]
    insert = written.actions["B2_J"].movements[-1]
    assert insert.drives == {"tools/cindy/AT3L": "tighten", "tools/cindy/AT3R": "tighten"} and insert.ends_on == "tools"
    assert insert.target.built == {"bars/B1", "bars/B2"} and insert.coupled and insert.controller == "compliant"
    # * Alice holds B2 together with Cindy until Cindy lets go.
    close = written.actions["B2_H"].movements[-1]
    assert [h.to for h in close.target.attached["bars/B2"]] == [L, R, ALICE_FLANGE]
    assert written.actions["B2_H"].supports_until == ("bars/B3",)
    assert read(tmp_path / "process").schedule == written.schedule


def test_keyframe_joints_chain(tmp_path):
    """Solved keyframes become targets and the next starts (B11 holds without geometry checks)."""
    process = _process()
    bars, holds = _records(process)
    rng = np.random.default_rng(0)
    from bar_assembly_core.urdf import movable_joints
    names = movable_joints(CINDY_URDF, CINDY_SRDF)

    def joints():
        return {name: float(v) for name, v in zip(names, rng.uniform(-1, 1, len(names)))}

    bars["bars/B1"] = replace(bars["bars/B1"], base=Pose((0.0, -1.0, 0.0)), approach=joints(), assembled=joints(),
                              retreat=joints(), home=joints())
    result = build_actions(process, SCHEDULE, bars, holds)
    report = check_plan(result.process, geometry=False)
    assert report.ok, report.errors
    insert = result.process.actions["B1_J"].movements[-1]
    assert insert.start.robots[CINDY].joints == bars["bars/B1"].approach
    retreat = result.process.actions["B1_R"].movements[1]
    assert retreat.start.robots[CINDY].joints == bars["bars/B1"].assembled


def test_missing_records_are_reported():
    """A bar or hold without a record is left out and named, the rest still builds."""
    process = _process()
    bars, holds = _records(process)
    del holds["bars/B2"]
    result = build_actions(process, SCHEDULE, bars, holds)
    assert result.process.schedule == ("B1_J", "B1_R", "B2_J", "B2_R", "B3_J", "B3_R")
    assert len(result.problems) == 2 and all("bars/B2" in p for p in result.problems)


def test_bars_without_a_record_are_placed_by_hand_at_their_place(tmp_path):
    """No J or R but a `bar_placing` action where the jointing would be: absent before it, built after it."""
    process = _process()
    bars, holds = _records(process)
    del bars["bars/B2"], holds["bars/B2"]
    result = build_actions(process, SCHEDULE, bars, holds)
    assert result.process.schedule == ("B1_J", "B1_R", "B2_P", "B3_J", "B3_R")
    assert sum("placed by hand" in p for p in result.problems) == 1
    placing = result.process.actions["B2_P"]
    assert (placing.type, placing.robot, len(placing.movements)) == ("bar_placing", "", 1)
    for action, movement in result.process.movements():
        there = action.id in ("B3_J", "B3_R")
        assert ("bars/B2" in movement.start.built) == there and ("bars/B2" in movement.start.present) == there
    assert not result.process.scene_at(result.process.actions["B1_J"].movements[0]).bodies["bars/B2"].enabled

    del bars["bars/B1"]
    result = build_actions(process, SCHEDULE, bars, holds)
    assert result.process.schedule == ("B1_P", "B2_P", "B3_J", "B3_R")
    transfer = next(m for _, m in result.process.movements() if m.id == "B3_J_transfer")
    scene = result.process.scene_at(transfer)
    assert scene.bodies["bars/B1"].enabled and scene.bodies["joints/J1-3_female"].enabled
    # * The half B3 is inserted into may be touched by B3's half; a bar it does not join onto may not.
    assert "joints/J1-3_male" in scene.bodies["joints/J1-3_female"].touches
    validate(result.process, check_robot_meshes=False)
    written = write(result.process, tmp_path / "process", package_dirs=[PKG])
    report = check_plan(written)
    assert report.ok, report.errors


def test_on_lives_in_targets_and_halves_know_their_tool_pose():
    """Decisions of 2026-10-09: `on` is set and cleared by targets only (B11), the ungrasp loosens, a hold release
    leaves, and the joint registry gives the pose of each tool-carrying half in its tool's TCP frame."""
    from core.process_builder import poses_in_tcp
    from core.joint_pair import load_joint_registry

    process = _process()
    bars, holds = _records(process)
    process = build_actions(process, SCHEDULE, bars, holds).process
    by_id = {m.id: m for _a, m in process.movements()}
    tools = {L: "tools/cindy/AT3L", R: "tools/cindy/AT3R"}
    assert by_id["B2_J_place_bar"].target.on == {tools[L]: "joints/J1-2_male", tools[R]: "joints/G2-T20-0_ground"}
    assert by_id["B2_R_ungrasp"].drives == {"tools/cindy/AT3L": "loosen", "tools/cindy/AT3R": "loosen"}
    assert by_id["B2_R_retreat"].target.on == {"tools/cindy/AT3L": None, "tools/cindy/AT3R": None}
    assert by_id["B2_H_to_grasp"].target.on == {"tools/alice/SupportGripper": "bars/B2"}
    assert by_id["B2_HR_retreat"].target.on == {"tools/alice/SupportGripper": None}
    assert [m.id for m in process.actions["B2_HR"].movements] == ["B2_HR_open", "B2_HR_retreat", "B2_HR_leave"]
    report = check_plan(process, geometry=False)
    assert not [e for e in report.errors if e.startswith("B11")], report.errors

    parts = poses_in_tcp(load_joint_registry())
    assert parts["T20/Male"] == Pose() and "T20Deck12/Male" in parts
    assert "T20/Female" not in parts and "T20/MoCap" not in parts
    turned = parts["T20/Ground"]
    expected = POSE_IN_TCP["T20/Ground"]
    assert np.allclose(turned.matrix(), expected.matrix(), atol=1e-9)
    assert all(v == 0.0 or abs(v) == 1.0 for v in turned.orientation), "a half turn is written exactly"
