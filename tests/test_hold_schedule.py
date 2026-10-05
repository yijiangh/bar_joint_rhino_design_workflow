"""Tests for core.hold_schedule (Rhino-free hold derivation).

The main scenario mirrors docs/support_demo: B2 must be held until B8 is
built (its stabilizers B5/B7/B8), and B5 likewise — so Alice takes B2,
Belle takes B5, and both release right after B8.
"""

import pytest

from core.hold_schedule import (
    bodies_built_after,
    build_action_schedule,
    derive_hold_plan,
    frozen_robot_leave_steps,
    holds_present_at_release,
    robots_holding_at_step,
)


# * ---- the docs/support_demo scenario ----
DEMO_BAR_SEQ = {f"B{i}": i for i in range(1, 10)}  # B1..B9 at steps 1..9
DEMO_SUPPORTED_UNTIL = {
    "B2": ["B5", "B7", "B8"],
    "B5": ["B6", "B7", "B8"],
}


def test_demo_two_holds_alice_then_belle():
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)

    assert set(plan) == {"B2", "B5"}
    # Alice is free at B2's step, so she takes the first hold.
    assert plan["B2"] == {
        "robot_name": "Alice",
        "hold_start_seq": 2,
        "release_after_seq": 8,
        "release_after_bar_id": "B8",
    }
    # Alice is still busy at B5's step (releases only after B8) -> Belle.
    assert plan["B5"] == {
        "robot_name": "Belle",
        "hold_start_seq": 5,
        "release_after_seq": 8,
        "release_after_bar_id": "B8",
    }


def test_demo_robots_holding_at_step():
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)

    # While B2 itself is assembled: nobody is frozen in the scene yet
    # (Alice only arrives mid-step, after jointing).
    assert robots_holding_at_step(plan, 2) == {}
    # While B4 is assembled: Alice stands frozen holding B2.
    assert robots_holding_at_step(plan, 4) == {"Alice": "B2"}
    # While B8 (the last stabilizer) is assembled: both robots present.
    assert robots_holding_at_step(plan, 8) == {"Alice": "B2", "Belle": "B5"}
    # B9: both released after B8 -> scene is robot-free again.
    assert robots_holding_at_step(plan, 9) == {}


def test_demo_action_schedule_interleaving():
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)
    assembly_seq = [f"B{i}" for i in range(1, 10)]
    schedule = build_action_schedule(assembly_seq, plan)

    as_tuples = [(e["kind"], e["bar_id"], e["robot_name"]) for e in schedule]

    # B2: jointing -> Alice holds -> release.
    i_j2 = as_tuples.index(("jointing", "B2", "Cindy"))
    assert as_tuples[i_j2 + 1] == ("holding", "B2", "Alice")
    assert as_tuples[i_j2 + 2] == ("release", "B2", "Cindy")

    # B8: jointing -> release -> BOTH holding releases, first-held first.
    i_j8 = as_tuples.index(("jointing", "B8", "Cindy"))
    assert as_tuples[i_j8 + 1] == ("release", "B8", "Cindy")
    assert as_tuples[i_j8 + 2] == ("holding_release", "B2", "Alice")
    assert as_tuples[i_j8 + 3] == ("holding_release", "B5", "Belle")

    # B9 is plain jointing + release, with no support actions after it.
    assert as_tuples[-2:] == [("jointing", "B9", "Cindy"), ("release", "B9", "Cindy")]

    # Every bar contributes exactly one jointing + one release, plus the
    # two holds and two holding-releases.
    assert len(schedule) == 2 * 9 + 2 + 2


def test_robot_reused_after_release():
    # B2 held until B4 only; B6 held until B8: Alice serves both in turn.
    bar_seq = {f"B{i}": i for i in range(1, 9)}
    supported = {"B2": ["B4"], "B6": ["B8"]}
    plan = derive_hold_plan(bar_seq, supported)
    assert plan["B2"]["robot_name"] == "Alice"
    assert plan["B6"]["robot_name"] == "Alice"


def test_hold_at_release_step_needs_second_robot():
    # B4's hold starts AT B2's release step (4): the release action runs
    # after B4's assembly, so Alice is still busy and Belle must take it.
    bar_seq = {f"B{i}": i for i in range(1, 7)}
    supported = {"B2": ["B4"], "B4": ["B6"]}
    plan = derive_hold_plan(bar_seq, supported)
    assert plan["B2"]["robot_name"] == "Alice"
    assert plan["B4"]["robot_name"] == "Belle"


def test_three_concurrent_holds_raise():
    bar_seq = {f"B{i}": i for i in range(1, 10)}
    supported = {
        "B2": ["B8"],
        "B3": ["B9"],
        "B5": ["B8"],  # third simultaneous hold -> no robot left
    }
    with pytest.raises(RuntimeError, match="No free support robot for bar 'B5'"):
        derive_hold_plan(bar_seq, supported)


def test_earlier_only_stabilizers_mean_no_hold():
    # Every listed stabilizer is built before the bar itself -> stable on
    # placement, no hold entry.
    bar_seq = {"B1": 1, "B2": 2, "B3": 3}
    supported = {"B3": ["B1", "B2"]}
    assert derive_hold_plan(bar_seq, supported) == {}


def test_unknown_stabilizer_raises():
    bar_seq = {"B1": 1, "B2": 2}
    supported = {"B2": ["B99"]}
    with pytest.raises(RuntimeError, match="unknown stabilizing bar"):
        derive_hold_plan(bar_seq, supported)


def test_duplicate_steps_raise():
    bar_seq = {"B1": 1, "B2": 1}
    with pytest.raises(RuntimeError, match="Duplicate assembly step"):
        derive_hold_plan(bar_seq, {})


# * ---- who is in a hold's scenes (issues D1 + D5) ----


def test_same_step_release_drops_the_earlier_hold():
    """B2 and B5 both release after B8; B2 goes first, so B5's release no longer sees Alice."""
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)
    assert holds_present_at_release(plan, "B2") == ["B5"]
    assert holds_present_at_release(plan, "B5") == []


def test_hold_starting_at_release_step_is_present():
    """B4's hold starts AT B2's release step, so it is there when B2 is released."""
    bar_seq = {f"B{i}": i for i in range(1, 7)}
    plan = derive_hold_plan(bar_seq, {"B2": ["B4"], "B4": ["B6"]})
    assert holds_present_at_release(plan, "B2") == ["B4"]
    assert holds_present_at_release(plan, "B4") == []


def test_frozen_robot_leave_steps_demo():
    """In B5's grasp scene: Cindy leaves after B5, Alice (holding B2) after B8."""
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)
    assert frozen_robot_leave_steps(plan, "B5") == {"Cindy": 5, "Alice": 8}
    # B2 is the first hold: only Cindy is frozen in its grasp scene.
    assert frozen_robot_leave_steps(plan, "B2") == {"Cindy": 2}


def test_bodies_built_after():
    steps = {"bar_B1": 1, "bar_B4": 4, "joint_J4-5_female": 4, "bar_B9": 9}
    assert bodies_built_after(steps, 4) == ["bar_B9"]
    assert bodies_built_after(steps, 3) == ["bar_B4", "bar_B9", "joint_J4-5_female"]
