"""`core.plan_steps`: the viewers' poses as moments of a process, the timeline, and solutions read back as keyframes.

The three-bar process of `test_process_builder.py` (B2 held by Alice until B3 is built), with solved keyframes.
"""

from dataclasses import replace

import numpy as np
import pytest

from test_process_builder import ALICE, ALICE_SRDF, ALICE_URDF, CINDY, CINDY_SRDF, CINDY_URDF, SCHEDULE, _process, \
    _records

pytestmark = pytest.mark.skipif(not CINDY_URDF.is_file(), reason="asset/husky_urdf submodule not checked out")


def _solved(unsolved=()):
    """The process with every bar's keyframe solved except `unsolved`, and Alice's hold of B2 solved."""
    from scaffolding_assembly_core.geometry import Pose
    from scaffolding_assembly_core.urdf import movable_joints
    from core.process_builder import build_actions

    process = _process()
    bars, holds = _records(process)
    cindy, alice = movable_joints(CINDY_URDF, CINDY_SRDF), movable_joints(ALICE_URDF, ALICE_SRDF)

    def joints(names, value):
        return {name: value for name in names}

    for i, bar in enumerate(sorted(bars)):
        if bar not in unsolved:
            bars[bar] = replace(bars[bar], base=Pose((0.0, -1.0 - i, 0.0)), approach=joints(cindy, 0.1),
                                assembled=joints(cindy, 0.2), retreat=joints(cindy, 0.3), home=joints(cindy, 0.4))
    holds["bars/B2"] = replace(holds["bars/B2"], base=Pose((0.0, 1.0, 0.0)), approach=joints(alice, 0.5),
                               held=joints(alice, 0.6))
    return build_actions(process, SCHEDULE, bars, holds).process


def test_bar_steps_follow_the_schedule():
    """Each bar's poses; B2 is held, B3 releases Alice's hold of B2."""
    from core.plan_steps import bar_steps

    process = _solved()
    assert [s.label for s in bar_steps(process, "bars/B1")] == ["B1 approach", "B1 assembled", "B1 retreat", "B1 home"]
    b2 = bar_steps(process, "bars/B2")
    assert [s.pose for s in b2] == ["approach", "assembled", "hold", "retreat", "home"]
    assert b2[0].movement.id == "B2_J_insert" and b2[1].movement.id == "B2_H_approach"
    assert b2[2].movement.id == "B2_R_ungrasp" and b2[4].at_end
    b3 = bar_steps(process, "bars/B3")
    assert b3[-1].pose == ("release", "bars/B2") and b3[-1].label == "B3 release B2"
    assert b3[-1].movement.id == "B2_HR_retreat" and b3[-1].at_end


def test_step_scenes():
    """Scenes put the robots where the pose says: approach, home (target applied), Alice holding, Alice let go."""
    from core.plan_steps import bar_steps, step_scene

    process = _solved()
    approach, assembled, hold, _retreat, home = bar_steps(process, "bars/B2")
    cindy = step_scene(process, approach).robots[CINDY]
    assert cindy.joints["left_ur_arm_elbow_joint"] == pytest.approx(0.1)
    assert step_scene(process, home).robots[CINDY].joints["left_ur_arm_elbow_joint"] == pytest.approx(0.4)
    held = step_scene(process, hold)
    assert held.robots[ALICE].enabled and held.robots[ALICE].joints["ur_arm_elbow_joint"] == pytest.approx(0.6)
    assert held.bodies["bars/B2"].enabled
    release = step_scene(process, bar_steps(process, "bars/B3")[-1])
    assert release.robots[ALICE].joints["ur_arm_elbow_joint"] == pytest.approx(0.5)  # retreated to the approach


def test_timeline_keeps_releases_of_unsolved_bars():
    """An unsolved bar adds no assembly step but keeps its release step, and is named."""
    from core.plan_steps import timeline

    steps, skipped = timeline(_solved(unsolved=("bars/B3",)))
    labels = [s.label for s in steps]
    assert labels[:4] == ["B1 approach", "B1 assembled", "B1 retreat", "B1 home"]
    assert "B3 approach" not in labels and labels[-1] == "B3 release B2"
    assert skipped and skipped[0][0] == "bars/B3" and "B2" in skipped[0][1]


def test_solutions_become_frames_and_keyframes():
    """Trajectory rows become frames in schedule order; solved ends become Rhino's keyframe record."""
    from scaffolding_assembly_core.process.solutions import MovementResult, Planner, SolvedAgainst, Solution, Trajectory
    from scaffolding_assembly_core.geometry import Pose
    from core.plan_steps import arm_groups, solved_keyframes, trajectory_frames

    process = _solved(unsolved=("bars/B1",))
    names = ("left_ur_arm_elbow_joint", "right_ur_arm_elbow_joint")
    base = Pose((5.0, 0.0, 0.0))

    def result(value, rows=0):
        end = {CINDY: {n: value for n in names}}
        trajectory = Trajectory(CINDY, names, tuple((value, value) for _ in range(rows))) if rows else None
        return MovementResult("solved", bases={CINDY: base}, start=end, end=end, trajectory=trajectory)

    against, planner = SolvedAgainst("d", "a"), Planner("test")
    solutions = {
        "B1_J": Solution("B1_J", against, planner, {"B1_J_transfer": result(0.7, rows=3),
                                                    "B1_J_insert": result(0.8, rows=2)}),
        "B1_R": Solution("B1_R", against, planner, {"B1_R_retreat": result(0.9, rows=4)}),
    }
    frames = trajectory_frames(process, solutions, "bars/B1")
    assert [f.movement.id for f in frames] == ["B1_J_transfer"] * 3 + ["B1_J_insert"] * 2 + ["B1_R_retreat"] * 4
    assert frames[0].base == base and frames[3].label == "B1_J_insert 1/2"
    keyframes = solved_keyframes(process, solutions, "bars/B1")
    assert keyframes["base"] == base and keyframes["assembled"][names[0]] == pytest.approx(0.8)
    assert keyframes["approach"][names[0]] == pytest.approx(0.7) and keyframes["retreat"][names[0]] == 0.9
    assert solved_keyframes(process, {}, "bars/B1") is None

    groups = arm_groups({"left_ur_arm_wrist_3_joint": 3.0, "left_ur_arm_shoulder_pan_joint": 1.0,
                         "right_ur_arm_elbow_joint": 2.0})
    assert groups["left"]["joint_names"] == ["left_ur_arm_shoulder_pan_joint", "left_ur_arm_wrist_3_joint"]
    assert groups["right"]["joint_values"] == [2.0]
    assert np.isclose(sum(groups["left"]["joint_values"]), 4.0)


def test_changed_since_planning(tmp_path):
    """Solutions planned on a written process are current for the same process in memory; a re-solved bar is not."""
    from scaffolding_assembly_core.process import read, write
    from scaffolding_assembly_core.process.solutions import Planner, Solution, solved_against
    from core.plan_steps import changed_since_planning
    from test_process_builder import PKG

    process = _solved()
    written = write(process, tmp_path / "process", package_dirs=[PKG])
    solutions = {a: Solution(a, solved_against(written, a), Planner("test"), {}) for a in ("B1_J", "B2_J")}
    assert changed_since_planning(read(tmp_path / "process"), solutions, [PKG]) == []
    assert changed_since_planning(process, solutions, [PKG]) == []          # the same process, still in memory
    assert changed_since_planning(_solved(unsolved=("bars/B2",)), solutions, [PKG]) == ["B2_J"]
