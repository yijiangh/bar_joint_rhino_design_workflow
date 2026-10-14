"""Stale IK: the core says which saved poses miss their targets (plan check B15); `core.ik_status` names the bars."""

from dataclasses import replace

import numpy as np

from core import core_lib  # noqa: F401
from core.process_builder import build_actions
from core.ik_status import stale_from, stale_ik_bars, stale_ik_lines
from scaffolding_assembly_core.process.plan_check import UnreachedTarget, check_plan
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.kinematics import link_poses
from scaffolding_assembly_core.urdf import movable_joints
from test_process_builder import CINDY, CINDY_SRDF, CINDY_URDF, L, R, SCHEDULE, _process, _records


def _solved(bars, bar, seed=0):
    """The bar's record with a base and joints, and the flange targets put where those joints reach: a valid solve."""
    rng = np.random.default_rng(seed)
    names = movable_joints(CINDY_URDF, CINDY_SRDF)
    joints = {name: float(v) for name, v in zip(names, rng.uniform(-1, 1, len(names)))}
    base = Pose((0.0, -1.0, 0.0))
    reached = link_poses(CINDY_URDF, base, joints, [L.rsplit("/", 1)[-1], R.rsplit("/", 1)[-1]])
    tool0 = {flange: reached[flange.rsplit("/", 1)[-1]] for flange in (L, R)}
    return replace(bars[bar], base=base, assembled=joints, tool0=tool0)


def test_a_valid_solve_is_not_stale():
    process = _process()
    bars, holds = _records(process)
    bars["bars/B1"] = _solved(bars, "bars/B1")
    built = build_actions(process, SCHEDULE, bars, holds).process
    assert stale_ik_bars(built) == {}


def test_a_solve_whose_targets_moved_is_stale():
    """The flange targets come from the document as it is now; the joints were solved for where they were."""
    process = _process()
    bars, holds = _records(process)
    solved = _solved(bars, "bars/B1")
    moved = {flange: Pose(tuple(p + d for p, d in zip(pose.position, (0.0, 2.5, 0.0))), pose.orientation)
             for flange, pose in solved.tool0.items()}
    bars["bars/B1"] = replace(solved, tool0=moved)
    built = build_actions(process, SCHEDULE, bars, holds).process
    stale = stale_ik_bars(built)
    assert list(stale) == ["B1"]
    assert stale["B1"] == "assembly pose misses its target by 2.5 m"
    # The same verdict as the plan check's, which is what an export prints.
    report = check_plan(built)
    assert stale_from(report.unreached) == stale
    assert any(e.startswith("B15: B1_J_insert") for e in report.errors)


def test_unsolved_bars_are_not_stale():
    process = _process()
    bars, holds = _records(process)
    assert stale_ik_bars(build_actions(process, SCHEDULE, bars, holds).process) == {}


def test_reasons_name_the_role_and_the_worst_miss():
    misses = [UnreachedTarget("B7_J", "B7_J_insert", "bars/B7", f"{CINDY}/left_ur_arm_tool0", 0.004, 0.0),
              UnreachedTarget("B7_R", "B7_R_retreat", "bars/B7", f"{CINDY}/left_ur_arm_tool0", 0.012, 0.0),
              UnreachedTarget("B7_H", "B7_H_to_grasp", "bars/B7", "robots/alice/ur_arm_tool0", 0.0, 0.02),
              UnreachedTarget("B9_J", "B9_J_insert", "bars/B9", f"{CINDY}/left_ur_arm_tool0", 5.93, 3.14)]
    assert stale_from(misses) == {
        "B7": "assembly pose misses its target by 12.0 mm; hold pose misses its target by 1.15 deg",
        "B9": "assembly pose misses its target by 5.9 m"}


def test_lines_list_the_first_bars_and_name_the_rest():
    assert stale_ik_lines({}) == [] and stale_ik_lines(None) == []
    lines = stale_ik_lines({f"B{i}": "assembly pose misses its target by 1.0 m" for i in range(11)}, limit=8)
    assert lines[0].startswith("STALE IK on 11 bar(s)") and len(lines) == 10
    assert lines[-1] == "  ... and 3 more: B8, B9, B10"
