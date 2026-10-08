"""Headless tests for the movement timeline of one bar (issues D7 + D9 + D10).

- Normal bar: load, mount, grasp, transfer, tighten (runs through the insert),
  insert. Only the arms whose male has its female in the scene tighten (D10:
  on a one-sided bar the other female sits on a staging bar).
- Ground bar: load, mount, grasp, transfer, insert, operator fixes the
  foundation. No jointing motor runs.
- Release, every bar: ungrasp, retreat, home (no untighten step).

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_bar_action_timeline.py -v
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import numpy as np
import pytest

from core import bar_action
from core.ik_collision_setup import mixed_ground_male_error
from rs_data_structure.bar_action import ManualMovement, ScaffoldingToolMovement


TOOLS = ["AT3L", "AT3R"]


class _FakeState:
    """Duck-typed stand-in for RobotCellState: only what the timeline touches."""

    def __init__(self, label: str, configuration=None):
        self.label = label
        self.robot_configuration = configuration

    def copy(self):
        return copy.deepcopy(self)


class _FakeConfig:
    """Duck-typed stand-in for a compas Configuration (copy only)."""

    def __init__(self, values):
        self.values = list(values)

    def copy(self):
        return _FakeConfig(self.values)


def _arm_movement(name: str, label: str, configuration=None):
    """A stand-in arm movement: a name-only id and a labelled start state."""
    return SimpleNamespace(
        movement_id=name,
        start_state=_FakeState(label, configuration),
        notes={},
    )


def _timeline(is_ground_bar: bool, tighten_tools: list = TOOLS):
    """Run the timeline on stand-in arm movements; return (jointing, release, arms).

    Args:
        is_ground_bar (bool): build the ground-bar shape.
        tighten_tools (list): the tools that screw a joint (both by default).
    """
    arms = {
        "m0": _arm_movement(bar_action.MV_FREE_TO_LOAD, "load"),
        "m1": _arm_movement(bar_action.MV_TRANSFER, "transfer start"),
        "m2": _arm_movement(bar_action.MV_LM_INSERT, "insert start", _FakeConfig([1.0])),
        "m3": _arm_movement(bar_action.MV_LM_RETREAT, "retreat start", _FakeConfig([2.0])),
        "m4": _arm_movement(bar_action.MV_FREE_HOME, "home start"),
    }
    jointing, release = bar_action.assemble_timeline(
        "B7", arms["m0"], arms["m1"], arms["m2"], arms["m3"], arms["m4"],
        TOOLS, tighten_tools, is_ground_bar,
    )
    return jointing, release, arms


# * ---------------------------------------------------------------- normal bar


def test_normal_bar_jointing_order_and_ids():
    """Normal bar: six movements, tighten right before the insert, ids numbered."""
    jointing, _release, _arms = _timeline(is_ground_bar=False)
    assert list(jointing) == [
        bar_action.MV_FREE_TO_LOAD, bar_action.MV_MANUAL_MOUNT_BAR,
        bar_action.MV_TOOL_GRASP_BAR, bar_action.MV_TRANSFER,
        bar_action.MV_TOOL_TIGHTEN, bar_action.MV_LM_INSERT,
    ]
    assert [m.movement_id for m in jointing.values()] == [
        "B7_J_M0_free_to_load", "B7_J_M1_manual_mount_bar", "B7_J_M2_tool_grasp_bar",
        "B7_J_M3_CDFM_transfer_to_approach", "B7_J_M4_tool_tighten_joint",
        "B7_J_M5_LM_insert",
    ]
    tighten = jointing[bar_action.MV_TOOL_TIGHTEN]
    assert isinstance(tighten, ScaffoldingToolMovement)
    assert tighten.tool_action == "tighten"
    assert tighten.overlaps_next is True
    assert tighten.tool_names == TOOLS
    # The tighten step starts where the insert starts (the approach).
    assert tighten.start_state.label == "insert start"


def test_one_sided_bar_tightens_one_tool_but_grasps_with_both():
    """One-sided bar (D10): only the screwing arm tightens; both arms clamp the bar."""
    jointing, release, arms = _timeline(is_ground_bar=False, tighten_tools=["AT3L"])
    assert jointing[bar_action.MV_TOOL_TIGHTEN].tool_names == ["AT3L"]
    assert jointing[bar_action.MV_TOOL_GRASP_BAR].tool_names == TOOLS
    assert release[bar_action.MV_TOOL_UNGRASP_BAR].tool_names == TOOLS
    # The insert says whose stall ends it.
    assert arms["m2"].notes["stall_tools"] == ["AT3L"]


def test_normal_bar_has_no_operator_fix_step():
    jointing, _release, _arms = _timeline(is_ground_bar=False)
    assert bar_action.MV_MANUAL_FIX_FOUNDATION not in jointing


# * ---------------------------------------------------------------- ground bar


def test_ground_bar_jointing_order_and_ids():
    """Ground bar: no tighten; insert then the operator's fix step, ids renumbered."""
    jointing, _release, _arms = _timeline(is_ground_bar=True)
    assert list(jointing) == [
        bar_action.MV_FREE_TO_LOAD, bar_action.MV_MANUAL_MOUNT_BAR,
        bar_action.MV_TOOL_GRASP_BAR, bar_action.MV_TRANSFER,
        bar_action.MV_LM_INSERT, bar_action.MV_MANUAL_FIX_FOUNDATION,
    ]
    assert jointing[bar_action.MV_LM_INSERT].movement_id == "B7_J_M4_LM_insert"
    assert jointing[bar_action.MV_MANUAL_FIX_FOUNDATION].movement_id == "B7_J_M5_manual_fix_foundation"
    assert not any(
        getattr(m, "tool_action", None) == "tighten" for m in jointing.values()
    )


def test_ground_bar_fix_step_holds_bar_at_assembled_config():
    """The fix step: insert's state (bar held) with the arms at the assembled config."""
    jointing, _release, arms = _timeline(is_ground_bar=True)
    fix = jointing[bar_action.MV_MANUAL_FIX_FOUNDATION]
    assert isinstance(fix, ManualMovement)
    assert "foundation" in fix.tag
    assert fix.start_state.label == "insert start"
    # Assembled config = the retreat's start config, copied (not shared).
    assert fix.start_state.robot_configuration.values == [2.0]
    assert fix.start_state.robot_configuration is not arms["m3"].start_state.robot_configuration


def test_ground_bar_fix_step_unsolved_has_no_config():
    """Before IK (no assembled config) the fix step's config stays None."""
    arms = [_arm_movement(n, n) for n in (
        bar_action.MV_FREE_TO_LOAD, bar_action.MV_TRANSFER, bar_action.MV_LM_INSERT,
        bar_action.MV_LM_RETREAT, bar_action.MV_FREE_HOME,
    )]
    jointing, _release = bar_action.assemble_timeline("B1", *arms, TOOLS, [], True)
    assert jointing[bar_action.MV_MANUAL_FIX_FOUNDATION].start_state.robot_configuration is None


# * ---------------------------------------------------------------- release


@pytest.mark.parametrize("is_ground_bar", [False, True])
def test_release_is_ungrasp_retreat_home(is_ground_bar):
    """Every bar: three release movements, no untighten anywhere."""
    jointing, release, _arms = _timeline(is_ground_bar)
    assert [m.movement_id for m in release.values()] == [
        "B7_R_M0_tool_ungrasp_bar", "B7_R_M1_LM_retreat", "B7_R_M2_free_home",
    ]
    ungrasp = release[bar_action.MV_TOOL_UNGRASP_BAR]
    assert ungrasp.tool_action == "ungrasp"
    # The ungrasp is the attachment boundary: the retreat's start state.
    assert ungrasp.start_state.label == "retreat start"
    every = list(jointing.values()) + list(release.values())
    assert not any(getattr(m, "tool_action", None) == "untighten" for m in every)


# * ---------------------------------------------------------------- lookups


def _action(ids):
    return SimpleNamespace(movements=[SimpleNamespace(movement_id=i) for i in ids])


def test_movement_by_name_new_and_old_files():
    """Lookups by name work for the new shape and for files with the old untighten step."""
    new = _action(["B1_J_M4_LM_insert", "B1_J_M5_manual_fix_foundation"])
    old = _action([
        "B3_R_M0_tool_untighten_joint", "B3_R_M1_tool_ungrasp_bar",
        "B3_R_M2_LM_retreat", "B3_R_M3_free_home",
    ])
    found = bar_action.movement_by_name(new, bar_action.KIND_JOINTING, bar_action.MV_LM_INSERT)
    assert found.movement_id == "B1_J_M4_LM_insert"
    found = bar_action.movement_by_name([new, old], bar_action.KIND_RELEASE, bar_action.MV_FREE_HOME)
    assert found.movement_id == "B3_R_M3_free_home"
    # Same name, wrong kind -> not found.
    assert bar_action.movement_by_name(old, bar_action.KIND_JOINTING, bar_action.MV_FREE_HOME) is None


def test_movement_by_name_needs_exact_name():
    """``LM_retreat`` must not match ``HR_M1_LM_retreat`` (different kind) or a longer name."""
    action = _action(["B3_HR_M1_LM_retreat", "B3_R_M1_LM_retreat_extra"])
    assert bar_action.movement_by_name(action, bar_action.KIND_RELEASE, bar_action.MV_LM_RETREAT) is None


# * ---------------------------------------------------------------- tighten tools (D10)


ARM_TOOLS = {"left": "AT3L", "right": "AT3R"}


def test_tighten_tools_both_females_in_scene():
    """Both males have their female in the scene: both tools tighten."""
    env_geom = {"joint_J1-3_female": {}, "joint_J2-3_female": {}}
    tools = bar_action.tighten_tools_for(
        "B3", {"J1-3": "left", "J2-3": "right"}, ARM_TOOLS, env_geom, False,
    )
    assert tools == ["AT3L", "AT3R"]


@pytest.mark.parametrize("present,expected", [
    ("joint_J1-3_female", ["AT3L"]),   # B3: right female on fake B2
    ("joint_J2-3_female", ["AT3R"]),   # mirror case (B12 / B15)
])
def test_tighten_tools_one_sided(present, expected):
    """A male whose female is not in the scene (staging bar) does not tighten."""
    tools = bar_action.tighten_tools_for(
        "B3", {"J1-3": "left", "J2-3": "right"}, ARM_TOOLS, {present: {}}, False,
    )
    assert tools == expected


def test_tighten_tools_mocap_receiver_counts():
    """A male mating into a MoCap half (a Female with a marker plate) screws too."""
    env_geom = {"joint_J1-3_mocap": {}, "joint_J2-3_female": {}}
    tools = bar_action.tighten_tools_for(
        "B3", {"J1-3": "left", "J2-3": "right"}, ARM_TOOLS, env_geom, False,
    )
    assert tools == ["AT3L", "AT3R"]


def test_tighten_tools_no_mating_male_raises():
    """A normal bar where no male has a female in the scene is not a valid design."""
    with pytest.raises(RuntimeError, match="B3"):
        bar_action.tighten_tools_for(
            "B3", {"J1-3": "left", "J2-3": "right"}, ARM_TOOLS, {}, False,
        )


def test_tighten_tools_ground_bar_is_empty():
    """A ground bar runs no jointing motor."""
    assert bar_action.tighten_tools_for("B1", {}, ARM_TOOLS, {}, True) == []


# * ---------------------------------------------------------------- mixed bars


def test_mixed_ground_and_male_is_an_error():
    msg = mixed_ground_male_error("B9", ["J3-9"], ["G9-T20Ground-0"])
    assert msg is not None
    assert "B9" in msg and "J3-9" in msg and "G9-T20Ground-0" in msg


@pytest.mark.parametrize("males,grounds", [(["J1-2", "J3-2"], []), ([], ["G1-a", "G1-b"]), ([], [])])
def test_unmixed_bars_pass(males, grounds):
    assert mixed_ground_male_error("B2", males, grounds) is None


# * ---------------------------------------------------------------- insert notes


class _FakeCellState:
    """Duck-typed RobotCellState with rigid-body states (for the arm builders)."""

    def __init__(self, keys):
        self.rigid_body_states = {
            k: SimpleNamespace(
                touch_bodies=[], touch_links=[], attached_to_link=None,
                attached_to_tool=None, attachment_frame=None, frame=None,
                is_hidden=False,
            )
            for k in keys
        }
        self.robot_configuration = None
        self.robot_base_frame = None

    def copy(self):
        return copy.deepcopy(self)


def test_build_m2_records_what_ends_the_insert():
    """The insert carries the given end condition (ground bar: target reached)."""
    keys = {"bar_B20"}
    state = _FakeCellState(keys)
    env_geom = {"bar_B20": {"frame_world_mm": np.eye(4), "kind": "bar", "parent_bar_id": "B20"}}
    m2 = bar_action._build_m2(
        state, "B20", env_geom, keys, {}, {}, "bar_B20", {"left": "AT3L", "right": "AT3R"},
        frozenset(), np.eye(4), np.eye(4), np.eye(4), None, 15.0,
        ends_on=bar_action.ENDS_ON_TARGET_REACHED,
    )
    assert m2.notes["ends_on"] == "target_reached"
    assert m2.movement_id == bar_action.MV_LM_INSERT
