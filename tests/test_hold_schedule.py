"""Tests for core.hold_schedule (Rhino-free hold derivation).

The main scenario mirrors docs/support_demo: B2 must be held until B8 is
built (its stabilizers B5/B7/B8), and B5 likewise — so Alice takes B2,
Belle takes B5, and both release right after B8.
"""

import pytest

from core.hold_schedule import (
    AFTER_INSERT,
    AFTER_RELEASE,
    BEFORE_INSERT,
    build_action_schedule,
    derive_hold_plan,
    describe_release,
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
        "release_timing": AFTER_RELEASE,
    }
    # Alice is still busy at B5's step (releases only after B8) -> Belle.
    assert plan["B5"] == {
        "robot_name": "Belle",
        "hold_start_seq": 5,
        "release_after_seq": 8,
        "release_after_bar_id": "B8",
        "release_timing": AFTER_RELEASE,
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


def test_range_schedule_follows_the_releasing_bar():
    """A range export walks only its bars (RSExportAllBarActions From/Until).

    Range B3..B8 of the demo: B2's hold started before the range, yet it is
    released right after B8, inside the range, so its release is scheduled.
    Range B3..B6: B5's hold starts inside but releases after B8, outside the
    range -- the holding step is scheduled, its release is not.
    """
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)

    in_range = [(e["kind"], e["bar_id"]) for e in build_action_schedule(
        [f"B{i}" for i in range(3, 9)], plan)]
    assert ("jointing", "B2") not in in_range
    assert ("holding", "B2") not in in_range
    assert ("holding_release", "B2") in in_range
    assert ("holding", "B5") in in_range and ("holding_release", "B5") in in_range

    open_end = [(e["kind"], e["bar_id"]) for e in build_action_schedule(
        [f"B{i}" for i in range(3, 7)], plan)]
    assert ("holding", "B5") in open_end
    assert not any(kind == "holding_release" for kind, _bar in open_end)


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


# * ---- release timings: when the robot lets go, relative to the last bar ----


def _schedule_around(bar_id, timing):
    """B2 held until B4 with `timing`, B4 held until B6: the actions from B3's release to B5."""
    bar_seq = {f"B{i}": i for i in range(1, 7)}
    plan = derive_hold_plan(bar_seq, {"B2": ["B4"], "B4": ["B6"]}, release_timing={"B2": timing})
    schedule = [(e["kind"], e["bar_id"], e["robot_name"])
                for e in build_action_schedule([f"B{i}" for i in range(1, 7)], plan)]
    start = schedule.index(("release", "B3", "Cindy")) + 1
    end = schedule.index(("jointing", "B5", "Cindy"))
    return plan, schedule[start:end]


def test_release_timing_orders_the_last_bar():
    """B4 is B2's last bar: the hold lets go before its insert, after it, or after Cindy's release."""
    _plan, before = _schedule_around("B4", BEFORE_INSERT)
    assert before == [("holding_release", "B2", "Alice"), ("jointing", "B4", "Cindy"),
                      ("holding", "B4", "Alice"), ("release", "B4", "Cindy")]
    _plan, after_insert = _schedule_around("B4", AFTER_INSERT)
    assert after_insert == [("jointing", "B4", "Cindy"), ("holding_release", "B2", "Alice"),
                            ("holding", "B4", "Alice"), ("release", "B4", "Cindy")]
    _plan, after_release = _schedule_around("B4", AFTER_RELEASE)
    assert after_release == [("jointing", "B4", "Cindy"), ("holding", "B4", "Belle"),
                             ("release", "B4", "Cindy"), ("holding_release", "B2", "Alice")]


def test_release_timing_frees_the_robot_at_its_last_bar():
    """Letting go before or after the insert frees Alice for B4's own hold; after Cindy's release it does not."""
    for timing, robot in ((BEFORE_INSERT, "Alice"), (AFTER_INSERT, "Alice"), (AFTER_RELEASE, "Belle")):
        plan, _actions = _schedule_around("B4", timing)
        assert plan["B2"]["release_timing"] == timing
        assert plan["B4"]["robot_name"] == robot, timing


def test_no_timing_is_after_release():
    """A bar with a list but no saved timing keeps the old behaviour."""
    plan = derive_hold_plan(DEMO_BAR_SEQ, DEMO_SUPPORTED_UNTIL)
    assert {e["release_timing"] for e in plan.values()} == {AFTER_RELEASE}
    assert describe_release(plan["B2"]) == "after B8's release (step 8)"


def test_before_insert_hold_is_gone_while_its_last_bar_is_inserted():
    bar_seq = {f"B{i}": i for i in range(1, 7)}
    plan = derive_hold_plan(bar_seq, {"B2": ["B4"]}, release_timing={"B2": BEFORE_INSERT})
    assert robots_holding_at_step(plan, 3) == {"Alice": "B2"}
    assert robots_holding_at_step(plan, 4) == {}
    plan = derive_hold_plan(bar_seq, {"B2": ["B4"]}, release_timing={"B2": AFTER_INSERT})
    assert robots_holding_at_step(plan, 4) == {"Alice": "B2"}


def test_unknown_timing_raises():
    with pytest.raises(RuntimeError, match="unknown release timing"):
        derive_hold_plan({"B1": 1, "B2": 2}, {"B1": ["B2"]}, release_timing={"B1": "whenever"})


def test_demo_2modules_hand_over():
    """261019_2ModulesDemo with B25 added to B21's list and B39 to B35's, both AfterInsert.

    Steps from the file: B21 3, B22 4, B35 7, B36 8, B37 9, B25 11, B169 12, B26 13, B39 16, B40 17, B43 21.
    Cindy inserts B25 -> Alice lets go of B21 -> Alice holds B25 -> Cindy releases B25; Cindy inserts B39 ->
    Belle lets go of B35 -> Belle holds B39 -> Cindy releases B39.
    """
    steps = {"B21": 3, "B22": 4, "B34": 5, "B35": 7, "B36": 8, "B37": 9, "B25": 11, "B169": 12, "B26": 13,
             "B165": 14, "B39": 16, "B40": 17, "B38": 18, "B41": 19, "B42": 20, "B43": 21}
    supported = {"B21": ["B22", "B37", "B25"], "B35": ["B36", "B37", "B39"],
                 "B25": ["B26", "B43", "B169"], "B39": ["B40", "B43"]}
    timing = {"B21": AFTER_INSERT, "B35": AFTER_INSERT}
    plan = derive_hold_plan(steps, supported, ("Alice", "Belle"), timing)
    assert {b: (e["robot_name"], e["release_after_bar_id"]) for b, e in plan.items()} == {
        "B21": ("Alice", "B25"), "B35": ("Belle", "B39"), "B25": ("Alice", "B43"), "B39": ("Belle", "B43")}
    assert describe_release(plan["B21"]) == "after B25's insert (step 11)"

    schedule = [(e["kind"], e["bar_id"], e["robot_name"])
                for e in build_action_schedule(sorted(steps, key=steps.get), plan)]
    for last, held, robot in (("B25", "B21", "Alice"), ("B39", "B35", "Belle")):
        i = schedule.index(("jointing", last, "Cindy"))
        assert schedule[i:i + 4] == [("jointing", last, "Cindy"), ("holding_release", held, robot),
                                     ("holding", last, robot), ("release", last, "Cindy")]
    # While B37 is inserted both hold; while B40 is, Alice holds B25 and Belle holds B39.
    assert robots_holding_at_step(plan, 9) == {"Alice": "B21", "Belle": "B35"}
    assert robots_holding_at_step(plan, 17) == {"Alice": "B25", "Belle": "B39"}

