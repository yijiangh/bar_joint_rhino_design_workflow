"""Headless tests for mocap RECEIVER halves in ``core.bar_action``.

A mocap half is a female with a marker plate bolted to its back: a male seats
into it exactly as into a female.  Its canonical collision key, though, ends in
``_mocap`` rather than ``_female`` -- and every touch-policy whitelist was
written against ``_female``.  Without the fix, the M2 mate contact is not
whitelisted and every bar carrying a mocap joint fails IK on a false collision.

Scene: bar B41 is being assembled.  Its left male (J7-41) seats into a MOCAP
half on built bar B7; its right male (J9-41) seats into an ordinary female on
built bar B9.  B41 also carries a mocap half of its own (J41-50, the receiver
for a later bar), which rides with the bar like a carried female.

Same harness as ``test_bar_action_ground.py`` / ``test_bar_action_subfloor.py``.
Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_bar_action_mocap.py -v
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import numpy as np
import pytest

from core import bar_action


BAR_KEY = "bar_B41"
MALE_L = "joint_J7-41_male"       # left tool; mates the mocap half on B7
MALE_R = "joint_J9-41_male"       # right tool; mates the female on B9
MOCAP_MATE = "joint_J7-41_mocap"  # built, on B7
FEMALE_MATE = "joint_J9-41_female"  # built, on B9
CARRIED_MOCAP = "joint_J41-50_mocap"  # on B41 itself, carried with the bar
TOOL_IDS = {"left": "AT3L", "right": "AT3R"}
ARM_TO_MALE = {"J7-41": "left", "J9-41": "right"}

ENV_GEOM = {
    BAR_KEY: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    MALE_L: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    MALE_R: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    CARRIED_MOCAP: {"frame_world_mm": np.eye(4), "parent_bar_id": "B41"},
    MOCAP_MATE: {
        "frame_world_mm": np.eye(4),
        "parent_bar_id": "B7",
        "block_name": "T20_MoCap",
    },
    FEMALE_MATE: {
        "frame_world_mm": np.eye(4),
        "parent_bar_id": "B9",
        "block_name": "T20_Female",
    },
}
ACTIVE_KEYS = {BAR_KEY, MALE_L, MALE_R, CARRIED_MOCAP}


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


def _policy(movement):
    st = _FakeState(set(ENV_GEOM))
    bar_action._apply_movement_touch_policy(
        st, movement, ACTIVE_KEYS, ENV_GEOM, ARM_TO_MALE, {}, BAR_KEY, TOOL_IDS,
    )
    return st


# ---------------------------------------------------------------------------
# _receiver_key
# ---------------------------------------------------------------------------


def test_receiver_key_finds_the_mocap_half():
    assert bar_action._receiver_key("J7-41", ENV_GEOM) == MOCAP_MATE


def test_receiver_key_finds_the_female_half():
    assert bar_action._receiver_key("J9-41", ENV_GEOM) == FEMALE_MATE


def test_receiver_key_falls_back_to_female_when_absent():
    assert bar_action._receiver_key("J1-2", ENV_GEOM) == "joint_J1-2_female"


# ---------------------------------------------------------------------------
# M2: the mate contact -- the fix that matters
# ---------------------------------------------------------------------------


def test_m2_male_into_mocap_whitelists_the_mate_and_its_bar():
    """Without the fix this list is just {tool, bar}: the mate is never found."""
    m2 = _policy("M2")
    assert m2.rigid_body_states[MALE_L].touch_bodies == sorted(
        {"AT3L", BAR_KEY, MOCAP_MATE, "bar_B7"}
    )


def test_m2_male_into_female_is_unchanged():
    m2 = _policy("M2")
    assert m2.rigid_body_states[MALE_R].touch_bodies == sorted(
        {"AT3R", BAR_KEY, FEMALE_MATE, "bar_B9"}
    )


def test_m1_male_into_mocap_gets_no_mate_whitelist():
    """A mocap half is clamp-style, not a cradle: no mate contact before M2."""
    m1 = _policy("M1")
    assert m1.rigid_body_states[MALE_L].touch_bodies == sorted({"AT3L", BAR_KEY})


def test_built_mocap_mate_never_gets_the_tool():
    """The cradle tool relaxation is cradle-only; a mocap mate is not a cradle."""
    for movement in ("M1", "M2", "M3"):
        assert _policy(movement).rigid_body_states[MOCAP_MATE].touch_bodies == []


# ---------------------------------------------------------------------------
# A mocap half CARRIED on the bar being assembled
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("movement", ["M1", "M2"])
def test_carried_mocap_touches_its_bar_while_held(movement):
    st = _policy(movement)
    assert st.rigid_body_states[CARRIED_MOCAP].touch_bodies == [BAR_KEY]


@pytest.mark.parametrize("movement", ["M0", "M3", "M4"])
def test_carried_mocap_whitelist_cleared_when_not_held(movement):
    st = _FakeState(set(ENV_GEOM))
    st.rigid_body_states[CARRIED_MOCAP].touch_bodies = ["stale"]
    bar_action._apply_movement_touch_policy(
        st, movement, ACTIVE_KEYS, ENV_GEOM, ARM_TO_MALE, {}, BAR_KEY, TOOL_IDS,
    )
    assert st.rigid_body_states[CARRIED_MOCAP].touch_bodies == []


@pytest.mark.parametrize("bar_arm_side", ["left", "right"])
def test_carried_mocap_rides_the_bar_arm(bar_arm_side):
    """Bonded to the bar like a female -> attaches to the bar's arm."""
    st = _FakeState(ACTIVE_KEYS)
    bar_action._set_active_attachments(
        st, ACTIVE_KEYS, ENV_GEOM, ARM_TO_MALE, {}, np.eye(4), np.eye(4),
        bar_arm_side=bar_arm_side,
    )
    expected = bar_action._ARM_TOOL_LINKS[bar_arm_side]
    assert st.rigid_body_states[CARRIED_MOCAP].attached_to_link == expected
    assert st.rigid_body_states[BAR_KEY].attached_to_link == expected
