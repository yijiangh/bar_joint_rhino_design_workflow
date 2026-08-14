"""Headless tests for the subfloor cradle path in ``core.bar_action``.

A subfloor bar is received by big cradle-style female blocks
(``T20SubLeft/Right_Female``, flagged ``bar_cradle`` in joint_pairs.json): the
incoming bar physically rests INSIDE the cradles at the assembled pose, and the
cradle mouth already wraps the bar/male at the 15 mm approach pose. These tests
drive the allowed-touch policy with ``cradle_female_keys`` passed in directly
(no Rhino), plus the ``bar_cradle`` registry round-trip.

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_bar_action_subfloor.py -v
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import numpy as np
import pytest

from core import bar_action
from core.joint_pair import JointHalfDef, load_joint_registry


# ---------------------------------------------------------------------------
# Fixture scene: subfloor bar B41 with two males mating into cradle females
# ---------------------------------------------------------------------------

BAR_KEY = "bar_B41"
MALE_L = "joint_J26-41_male"       # grasped by the left tool, mates a cradle on B26
MALE_R = "joint_J40-41_male"       # grasped by the right tool, mates a cradle on B40
CRADLE_L = "joint_J26-41_female"   # T20Sub* cradle on built bar B26
CRADLE_R = "joint_J40-41_female"   # T20Sub* cradle on built bar B40
TOOL_IDS = {"left": "AT3L", "right": "AT3R"}
ARM_TO_MALE = {"J26-41": "left", "J40-41": "right"}
CRADLE_KEYS = frozenset({CRADLE_L, CRADLE_R})

ENV_GEOM = {
    BAR_KEY: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    MALE_L: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    MALE_R: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    CRADLE_L: {
        "frame_world_mm": np.eye(4),
        "parent_bar_id": "B26",
        "block_name": "T20SubLeft_Female",
    },
    CRADLE_R: {
        "frame_world_mm": np.eye(4),
        "parent_bar_id": "B40",
        "block_name": "T20SubRight_Female",
    },
}
ACTIVE_KEYS = {BAR_KEY, MALE_L, MALE_R}


class _FakeState:
    """Duck-typed stand-in for RobotCellState (same shape as the ground tests)."""

    def __init__(self, keys):
        self.rigid_body_states = {
            k: SimpleNamespace(
                touch_bodies=[],
                attached_to_link=None,
                attached_to_tool=None,
                attachment_frame=None,
                frame=None,
                is_hidden=False,
            )
            for k in keys
        }
        self.robot_configuration = None
        self.robot_base_frame = None

    def copy(self):
        return copy.deepcopy(self)


def _policy(movement, cradle_female_keys):
    st = _FakeState(set(ENV_GEOM))
    bar_action._apply_movement_touch_policy(
        st, movement, ACTIVE_KEYS, ENV_GEOM, ARM_TO_MALE, {}, BAR_KEY, TOOL_IDS,
        cradle_female_keys,
    )
    return st


# ---------------------------------------------------------------------------
# Touch policy with cradle mates
# ---------------------------------------------------------------------------


def test_cradle_male_allows_mate_in_m1_and_m2():
    """A male whose mate is a cradle allows the mate in M1 too (mouth wraps it)."""
    m1 = _policy("M1", CRADLE_KEYS)
    assert m1.rigid_body_states[MALE_L].touch_bodies == sorted(
        {"AT3L", BAR_KEY, CRADLE_L}
    )
    # M2 keeps the existing mate extras (mate + the mate's parent bar).
    m2 = _policy("M2", CRADLE_KEYS)
    assert m2.rigid_body_states[MALE_L].touch_bodies == sorted(
        {"AT3L", BAR_KEY, CRADLE_L, "bar_B26"}
    )
    assert m2.rigid_body_states[MALE_R].touch_bodies == sorted(
        {"AT3R", BAR_KEY, CRADLE_R, "bar_B40"}
    )


def test_bar_allows_cradles_in_m1_m2_only():
    """The incoming bar whitelists its cradle mates while approaching/inserting."""
    for movement in ("M1", "M2"):
        st = _policy(movement, CRADLE_KEYS)
        assert st.rigid_body_states[BAR_KEY].touch_bodies == sorted(
            {"AT3L", "AT3R", CRADLE_L, CRADLE_R}
        )
    # M3: bar released/static -> tools only. M0/M4: cleared.
    assert _policy("M3", CRADLE_KEYS).rigid_body_states[BAR_KEY].touch_bodies == ["AT3L", "AT3R"]
    assert _policy("M0", CRADLE_KEYS).rigid_body_states[BAR_KEY].touch_bodies == []
    assert _policy("M4", CRADLE_KEYS).rigid_body_states[BAR_KEY].touch_bodies == []


def test_clamp_style_bar_policy_unchanged():
    """Regression: with no cradle mates, today's exact whitelists are reproduced."""
    m1 = _policy("M1", frozenset())
    # M1 males: tool + bar only -- NO mate whitelist for clamp-style females.
    assert m1.rigid_body_states[MALE_L].touch_bodies == sorted({"AT3L", BAR_KEY})
    assert m1.rigid_body_states[BAR_KEY].touch_bodies == ["AT3L", "AT3R"]
    # M2 males: the standard mate extras still apply regardless of the flag.
    m2 = _policy("M2", frozenset())
    assert m2.rigid_body_states[MALE_L].touch_bodies == sorted(
        {"AT3L", BAR_KEY, CRADLE_L, "bar_B26"}
    )


# ---------------------------------------------------------------------------
# Registry flag round-trip
# ---------------------------------------------------------------------------


def _half(**overrides):
    kwargs = dict(
        block_name="X_Female",
        M_block_from_bar=np.eye(4),
        M_screw_from_block=np.eye(4),
        kind="female",
    )
    kwargs.update(overrides)
    return JointHalfDef(**kwargs)


def test_bar_cradle_defaults_false_and_roundtrips():
    """`bar_cradle` defaults False, survives to_dict/from_dict, old JSON loads."""
    assert _half().bar_cradle is False
    cradle = _half(bar_cradle=True)
    assert JointHalfDef.from_dict(cradle.to_dict()).bar_cradle is True
    # A registry entry written before the flag existed simply omits the key.
    legacy = cradle.to_dict()
    legacy.pop("bar_cradle")
    assert JointHalfDef.from_dict(legacy).bar_cradle is False


def test_shipped_registry_flags_subfloor_females():
    """The checked-in joint_pairs.json marks exactly the T20Sub* females as cradles."""
    halves = load_joint_registry().halves
    cradles = sorted(name for name, h in halves.items() if h.bar_cradle)
    assert cradles == ["T20SubLeft_Female", "T20SubRight_Female"]
