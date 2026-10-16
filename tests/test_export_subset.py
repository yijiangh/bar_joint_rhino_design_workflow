"""Headless tests for ``core.export_subset`` (the ``from .. until`` partial export).

- the sequence cuts (export range, assembly sequence up to ``until``);
- which bodies a range drops (later bars + the joint halves on them, unused floors);
- trimming a cell copy and the action states with one shared set of names, so
  compas_fab's cell / state match still holds.

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_export_subset.py -v
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from compas.geometry import Frame
from compas_fab.robots import RigidBody, RigidBodyState, RobotCell, RobotCellState

from core.export_subset import (
    assert_states_match_cell,
    assert_states_name_bodies,
    bodies_outside_cell,
    cut_assembly_seq,
    dropped_body_names,
    range_bar_ids,
    trim_action_states,
    trim_robot_cell,
    used_ground_ids,
)


# Real bars only; steps have a gap where a fake bar was (B2 at step 2).
BAR_MAP = {"B1": ("o1", 1), "B3": ("o3", 3), "B4": ("o4", 4), "B5": ("o5", 5)}

# The assembly cell's bodies: a female half on B1 waits for B5's male.
COLLISION_BODIES = {
    "bar_B1": {"kind": "bar", "parent_bar_id": "B1"},
    "bar_B3": {"kind": "bar", "parent_bar_id": "B3"},
    "bar_B4": {"kind": "bar", "parent_bar_id": "B4"},
    "bar_B5": {"kind": "bar", "parent_bar_id": "B5"},
    "joint_J1-3_female": {"kind": "joint", "parent_bar_id": "B1"},
    "joint_J1-3_male": {"kind": "joint", "parent_bar_id": "B3"},
    "joint_J1-5_female": {"kind": "joint", "parent_bar_id": "B1"},
    "joint_J1-5_male": {"kind": "joint", "parent_bar_id": "B5"},
    "obstacle_table": {"kind": "environment"},
    "ground_WG0": {"kind": "floor", "ground_id": "WG0"},
    "ground_WG1": {"kind": "floor", "ground_id": "WG1"},
}


# * ---------------------------------------------------------------- sequence


def test_cut_assembly_seq_keeps_the_built_prefix():
    assert cut_assembly_seq(BAR_MAP, "B4") == ["B1", "B3", "B4"]
    assert cut_assembly_seq(BAR_MAP, "B5") == ["B1", "B3", "B4", "B5"]


def test_range_bar_ids_inclusive():
    assert range_bar_ids(BAR_MAP, "B3", "B4") == ["B3", "B4"]
    assert range_bar_ids(BAR_MAP, "B4", "B4") == ["B4"]


def test_range_must_not_run_backwards():
    with pytest.raises(RuntimeError, match="must not come after"):
        range_bar_ids(BAR_MAP, "B5", "B3")


def test_unknown_range_bar_raises():
    """A fake bar (not in the real map) cannot be a range end."""
    with pytest.raises(RuntimeError, match="B2"):
        cut_assembly_seq(BAR_MAP, "B2")


# * ---------------------------------------------------------------- dropped bodies


def test_dropped_bodies_until_b4():
    """Until B4: B5 and its male go; B1's female for B5 stays; WG1 is unused."""
    dropped = dropped_body_names(COLLISION_BODIES, BAR_MAP, "B4", {"WG0"})
    assert dropped == {"bar_B5", "joint_J1-5_male", "ground_WG1"}


def test_full_range_drops_only_unused_floors():
    dropped = dropped_body_names(COLLISION_BODIES, BAR_MAP, "B5", {"WG0", "WG1"})
    assert dropped == set()


def test_body_on_unknown_bar_raises():
    """A body on a bar that is not registered means the cell is stale."""
    bodies = dict(COLLISION_BODIES)
    bodies["joint_J2-3_female"] = {"kind": "joint", "parent_bar_id": "B2"}
    with pytest.raises(RuntimeError, match="RSRebuildRobotCell"):
        dropped_body_names(bodies, BAR_MAP, "B4", {"WG0"})


def test_used_ground_ids_is_the_union():
    actions = [SimpleNamespace(walkable_ground_ids=["WG0"]), SimpleNamespace(walkable_ground_ids=[])]
    assert used_ground_ids(actions) == {"WG0"}


# * ---------------------------------------------------------------- trimming


def _cell() -> RobotCell:
    """A robot cell holding every body of COLLISION_BODIES (no robot model needed)."""
    return RobotCell(rigid_body_models={name: RigidBody([], []) for name in COLLISION_BODIES})


def _action(action_id: str = "B4_J_joint") -> SimpleNamespace:
    """An action with two movements whose states name every cell body.

    B4's male touches B1's female for B5 and B5's male (a contact list that
    must lose the dropped name).
    """
    movements = []
    for index in range(2):
        states = {name: RigidBodyState(Frame.worldXY()) for name in COLLISION_BODIES}
        states["bar_B4"].touch_bodies = ["joint_J1-5_female", "joint_J1-5_male"]
        movements.append(SimpleNamespace(
            movement_id=f"B4_J_M{index}",
            start_state=RobotCellState(rigid_body_states=states),
        ))
    # A step without a state (nothing to trim).
    movements.append(SimpleNamespace(movement_id="B4_J_M2", start_state=None))
    return SimpleNamespace(
        action_id=action_id, movements=movements,
        assembly_seq=["B1", "B3", "B4", "B5"], walkable_ground_ids=["WG0"],
    )


def test_trim_cell_and_states_still_match():
    """One dropped set applied to the cell copy and the states keeps them in step."""
    cell = _cell()
    action = _action()
    dropped = dropped_body_names(COLLISION_BODIES, BAR_MAP, "B4", used_ground_ids([action]))

    trimmed = trim_robot_cell(cell, dropped)
    n_removed = trim_action_states(action, dropped, cut_assembly_seq(BAR_MAP, "B4"))

    # The cached cell is not touched; the copy lacks exactly the dropped names.
    assert set(cell.rigid_body_models) == set(COLLISION_BODIES)
    assert set(trimmed.rigid_body_models) == set(COLLISION_BODIES) - dropped
    assert n_removed == 2 * len(dropped)
    assert action.assembly_seq == ["B1", "B3", "B4"]
    # The contact with a dropped body is gone; the one with a kept body stays.
    for movement in action.movements[:2]:
        assert movement.start_state.rigid_body_states["bar_B4"].touch_bodies == ["joint_J1-5_female"]
    assert_states_match_cell(trimmed, [action], "RobotCell.json")


def test_untrimmed_state_does_not_match_trimmed_cell():
    trimmed = trim_robot_cell(_cell(), {"bar_B5"})
    with pytest.raises(RuntimeError, match="B4_J_M0.*RobotCell.json"):
        assert_states_match_cell(trimmed, [_action()], "RobotCell.json")


def test_bodies_outside_cell_ignores_obstacles():
    """Re-exporting one bar into a cut bundle drops what the bundle's cell lacks."""
    in_cell = set(COLLISION_BODIES) - {"bar_B5", "joint_J1-5_male", "obstacle_table"}
    # The obstacle is never trimmed (not a bar / joint / floor), even if missing.
    assert bodies_outside_cell([_action()], in_cell) == {"bar_B5", "joint_J1-5_male"}


def test_states_name_recorded_bodies():
    """The single-bar refresh checks its cut states against the recorded names."""
    action = _action()
    dropped = {"bar_B5", "joint_J1-5_male"}
    trim_action_states(action, dropped)
    assert_states_name_bodies([action], set(COLLISION_BODIES) - dropped, "RobotCell.json")
    with pytest.raises(RuntimeError, match="missing.*bar_B5"):
        assert_states_name_bodies([action], set(COLLISION_BODIES), "RobotCell.json")
