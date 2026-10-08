"""The schema 2 builder (`core.design_builder`) on a small three-bar design with a hold, with the real robot files.

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
from core.design_builder import BarRecord, HoldRecord, build_actions, grasp_of, z_axis
from bar_assembly_core.design import BodySpec, Design, RobotSpec, read, validate, write
from bar_assembly_core.design.plan_check import PART_SEATS, check_plan
from bar_assembly_core.design.version import writer_info
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


def _bar(y: float, z: float) -> BodySpec:
    """A 1 m bar along world x at (0.5, y, z)."""
    pose = Pose((0.5, y, z), tuple(Rotation.from_euler("y", 90, degrees=True).as_quat()))
    cylinder = CylinderShape(0.01, 1.0)
    return BodySpec(id="", pose=pose, geometry=Geometry((cylinder,), (cylinder,)))


def _half(half_id: str, bar: str, part: str, position) -> BodySpec:
    """A half at `position`, its z axis pointing down (into the ground or the receiving bar)."""
    pose = Pose(tuple(position), tuple(Rotation.from_euler("x", 180, degrees=True).as_quat()))
    return BodySpec(id=half_id, pose=pose, geometry=BOX, part=part, mount=bar)


def _design() -> Design:
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
    tools = {"tools/AT3L": Tool("tools/AT3L", BOX, TCP_L, "scaffolding_v3", (f"{CINDY}/left_ur_arm_wrist_3_link",)),
             "tools/AT3R": Tool("tools/AT3R", BOX, TCP_R, "scaffolding_v3", (f"{CINDY}/right_ur_arm_wrist_3_link",)),
             "tools/alice/SupportGripper": Tool("tools/alice/SupportGripper", BOX, Pose(), "robotiq")}
    robots = {CINDY: RobotSpec(CINDY, CINDY_URDF, CINDY_SRDF, "0806",
                               {"left_ur_arm_tool0": "tools/AT3L", "right_ur_arm_tool0": "tools/AT3R"}, WHEELS),
              ALICE: RobotSpec(ALICE, ALICE_URDF, ALICE_SRDF, "0804", {"ur_arm_tool0": "tools/alice/SupportGripper"},
                               WHEELS)}
    return Design(None, writer_info(), robots, tools, bodies, (), {}, mates)


def _record(design: Design, bar: str, left_half: str, right_half: str, **keyframes) -> BarRecord:
    """Cindy's record for a bar: each flange placed so its TCP sits on its half at the part's seat."""
    tool0, retreat = {}, {}
    for flange, half, tcp in ((L, left_half, TCP_L), (R, right_half, TCP_R)):
        seat = PART_SEATS[design.bodies[half].part]
        tool0[flange] = compose(design.bodies[half].pose, invert(compose(tcp, seat)))
        retreat[flange] = tuple(-v for v in z_axis(design.bodies[half].pose))
    return BarRecord(bar=bar, robot=CINDY, tools={L: "tools/AT3L", R: "tools/AT3R"}, tool0=tool0,
                     on={L: left_half, R: right_half}, insert_direction=(0.0, 0.0, -1.0), retreat_direction=retreat,
                     ground=("ground/WG0",), **keyframes)


def _records(design: Design):
    bars = {"bars/B1": _record(design, "bars/B1", "joints/G1-T20-0_ground", "joints/G1-T20-1_ground"),
            "bars/B2": _record(design, "bars/B2", "joints/J1-2_male", "joints/G2-T20-0_ground"),
            "bars/B3": _record(design, "bars/B3", "joints/J1-3_male", "joints/J2-3_male")}
    tool0 = compose(design.bodies["bars/B2"].pose, Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
    holds = {"bars/B2": HoldRecord(bar="bars/B2", robot=ALICE, flange=ALICE_FLANGE, tool="tools/alice/SupportGripper",
                                   tool0=tool0, grasp=grasp_of(tool0, design.bodies["bars/B2"].pose),
                                   supports_until=("bars/B3",), ground=("ground/WG0",))}
    return bars, holds


SCHEDULE = (("J", "bars/B1"), ("R", "bars/B1"), ("J", "bars/B2"), ("H", "bars/B2"), ("R", "bars/B2"),
            ("J", "bars/B3"), ("R", "bars/B3"), ("HR", "bars/B2"))


def test_unsolved_design_passes_file_and_plan_checks(tmp_path):
    """Built, written and read back: A passes, B has no errors; every action has its movements in order."""
    design = _design()
    bars, holds = _records(design)
    result = build_actions(design, SCHEDULE, bars, holds)
    assert not result.problems
    validate(result.design, check_robot_meshes=False)
    written = write(result.design, tmp_path / "design", package_dirs=[PKG])
    assert written.schedule == ("B1_J", "B1_R", "B2_J", "B2_H", "B2_R", "B3_J", "B3_R", "B2_HR")
    report = check_plan(written)
    assert report.ok, report.errors
    assert [m.id for m in written.actions["B2_J"].movements] == [
        "B2_J_load", "B2_J_mount", "B2_J_grasp", "B2_J_transfer", "B2_J_insert"]
    insert = written.actions["B2_J"].movements[-1]
    assert insert.drives == {"tools/AT3L": "tighten", "tools/AT3R": "tighten"} and insert.ends_on == "tools"
    assert insert.target.built == {"bars/B1", "bars/B2"} and insert.coupled and insert.controller == "compliant"
    # * Alice holds B2 together with Cindy until Cindy lets go.
    close = written.actions["B2_H"].movements[-1]
    assert [h.to for h in close.target.attached["bars/B2"]] == [L, R, ALICE_FLANGE]
    assert written.actions["B2_H"].supports_until == ("bars/B3",)
    assert read(tmp_path / "design").schedule == written.schedule


def test_keyframe_joints_chain(tmp_path):
    """Solved keyframes become targets and the next starts (B11 holds without geometry checks)."""
    design = _design()
    bars, holds = _records(design)
    rng = np.random.default_rng(0)
    from bar_assembly_core.urdf import movable_joints
    names = movable_joints(CINDY_URDF, CINDY_SRDF)

    def joints():
        return {name: float(v) for name, v in zip(names, rng.uniform(-1, 1, len(names)))}

    bars["bars/B1"] = replace(bars["bars/B1"], base=Pose((0.0, -1.0, 0.0)), approach=joints(), assembled=joints(),
                              retreat=joints(), home=joints())
    result = build_actions(design, SCHEDULE, bars, holds)
    report = check_plan(result.design, geometry=False)
    assert report.ok, report.errors
    insert = result.design.actions["B1_J"].movements[-1]
    assert insert.start.robots[CINDY].joints == bars["bars/B1"].approach
    retreat = result.design.actions["B1_R"].movements[1]
    assert retreat.start.robots[CINDY].joints == bars["bars/B1"].assembled


def test_missing_records_are_reported():
    """A bar or hold without a record is left out and named, the rest still builds."""
    design = _design()
    bars, holds = _records(design)
    del holds["bars/B2"]
    result = build_actions(design, SCHEDULE, bars, holds)
    assert result.design.schedule == ("B1_J", "B1_R", "B2_J", "B2_R", "B3_J", "B3_R")
    assert len(result.problems) == 2 and all("bars/B2" in p for p in result.problems)
