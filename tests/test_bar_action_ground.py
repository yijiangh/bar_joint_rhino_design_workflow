"""Headless tests for the ground-bar path in ``core.bar_action``.

A ground bar has no male joint halves: the arm tools grasp its GROUND joints
directly, and the ground joints behave like female halves permanently bonded to
the bar (ride with it while gripped, stay in world after release). These tests
drive the attachment / allowed-touch / retreat / approach logic with the
classification dicts passed in directly, so no Rhino is needed (the Rhino-bound
classifiers and the walkable-ground normal helper are verified manually in the
toolbar workflow).

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_bar_action_ground.py -v
"""

from __future__ import annotations

import copy
from types import SimpleNamespace

import numpy as np
import pytest

from core import bar_action


# ---------------------------------------------------------------------------
# Fixture scene: bar B20 carrying two grasped ground joints
# ---------------------------------------------------------------------------

BAR_KEY = "bar_B20"
GROUND_L = "joint_G20-T20Ground-0_ground"  # grasped by the left tool
GROUND_R = "joint_G20-T20Ground-1_ground"  # grasped by the right tool
TOOL_IDS = {"left": "AT3L", "right": "AT3R"}
ARM_TO_GROUND = {"G20-T20Ground-0": "left", "G20-T20Ground-1": "right"}
LM = 15.0  # retreat / approach offset distance in mm


def _mm4(rot_cols, origin):
    """Build a 4x4 mm pose from three rotation columns + an origin."""
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2] = rot_cols
    m[:3, 3] = origin
    return m


# Both tool0 flanges look straight down (local +Z = world -Z), 200 mm above
# their ground joint blocks.
_TOOL_ROT = [(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)]
TOOL0_LEFT = _mm4(_TOOL_ROT, (100.0, 0.0, 500.0))
TOOL0_RIGHT = _mm4(_TOOL_ROT, (900.0, 0.0, 500.0))

# Ground blocks with deliberately non-world-aligned local Z axes, so the
# retreat math (joint local -Z) is actually exercised:
#   ground 0 local Z = world +X  -> retreat axis (-1, 0, 0)
#   ground 1 local Z = world +Y  -> retreat axis (0, -1, 0)
_G0_ROT = [(0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)]
_G1_ROT = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]

ENV_GEOM = {
    BAR_KEY: {
        "frame_world_mm": _mm4([(0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)], (0.0, 0.0, 300.0)),
        "kind": "bar",
        "parent_bar_id": "B20",
    },
    GROUND_L: {
        "frame_world_mm": _mm4(_G0_ROT, (100.0, 0.0, 300.0)),
        "kind": "joint",
        "subtype": "ground",
        "parent_bar_id": "B20",
    },
    GROUND_R: {
        "frame_world_mm": _mm4(_G1_ROT, (900.0, 0.0, 300.0)),
        "kind": "joint",
        "subtype": "ground",
        "parent_bar_id": "B20",
    },
}
ACTIVE_KEYS = set(ENV_GEOM)


class _FakeState:
    """Duck-typed stand-in for RobotCellState: just the fields the code touches."""

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


@pytest.fixture
def state():
    return _FakeState(ACTIVE_KEYS)


# ---------------------------------------------------------------------------
# Attachments
# ---------------------------------------------------------------------------


def test_attachments_classified_ground_goes_to_its_own_arm(state):
    """Each grasped ground joint attaches to ITS OWN arm's flange, not all-left."""
    bar_action._set_active_attachments(
        state, ACTIVE_KEYS, ENV_GEOM, {}, ARM_TO_GROUND, TOOL0_LEFT, TOOL0_RIGHT,
    )
    assert state.rigid_body_states[GROUND_L].attached_to_link == "left_ur_arm_tool0"
    assert state.rigid_body_states[GROUND_R].attached_to_link == "right_ur_arm_tool0"
    # The bar tube still rides the default bar arm (left).
    assert state.rigid_body_states[BAR_KEY].attached_to_link == "left_ur_arm_tool0"

    # Spot-check the stored grasp offset = inv(tool0_right) @ ground1_world.
    expected = np.linalg.inv(TOOL0_RIGHT) @ ENV_GEOM[GROUND_R]["frame_world_mm"]
    frame = state.rigid_body_states[GROUND_R].attachment_frame
    assert np.allclose(frame.point, expected[:3, 3] / 1000.0)
    assert np.allclose(frame.xaxis, expected[:3, 0])
    assert np.allclose(frame.yaxis, expected[:3, 1])


def test_attachments_unclassified_ground_rides_bar_arm(state):
    """A tool-less ground joint keeps the carried-female behavior (bar's arm)."""
    bar_action._set_active_attachments(
        state, ACTIVE_KEYS, ENV_GEOM, {}, {}, TOOL0_LEFT, TOOL0_RIGHT,
    )
    assert state.rigid_body_states[GROUND_L].attached_to_link == "left_ur_arm_tool0"
    assert state.rigid_body_states[GROUND_R].attached_to_link == "left_ur_arm_tool0"


# ---------------------------------------------------------------------------
# Touch policy (allowed contacts)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("movement", ["M1", "M2"])
def test_touch_policy_ground_held_movements(state, movement):
    """Held movements: each grasped ground allows {its own tool, the bar} only."""
    bar_action._apply_movement_touch_policy(
        state, movement, ACTIVE_KEYS, ENV_GEOM, {}, ARM_TO_GROUND, BAR_KEY, TOOL_IDS,
    )
    assert state.rigid_body_states[GROUND_L].touch_bodies == sorted({"AT3L", BAR_KEY})
    assert state.rigid_body_states[GROUND_R].touch_bodies == sorted({"AT3R", BAR_KEY})
    # No mate extras ever (the floor is not collision geometry today).
    assert "obstacle" not in " ".join(state.rigid_body_states[GROUND_L].touch_bodies)
    # The bar whitelists both gripper tools while held.
    assert state.rigid_body_states[BAR_KEY].touch_bodies == ["AT3L", "AT3R"]


def test_touch_policy_ground_retreat(state):
    """M3 (tool peeling off): only the ground joint's own tool stays allowed."""
    bar_action._apply_movement_touch_policy(
        state, "M3", ACTIVE_KEYS, ENV_GEOM, {}, ARM_TO_GROUND, BAR_KEY, TOOL_IDS,
    )
    assert state.rigid_body_states[GROUND_L].touch_bodies == ["AT3L"]
    assert state.rigid_body_states[GROUND_R].touch_bodies == ["AT3R"]


@pytest.mark.parametrize("movement", ["M0", "M4"])
def test_touch_policy_ground_cleared_m0_m4(state, movement):
    """M0/M4: the ground whitelists are actively cleared, not left over."""
    # Pre-load stale entries to prove the policy assigns rather than skips.
    state.rigid_body_states[GROUND_L].touch_bodies = ["stale"]
    state.rigid_body_states[GROUND_R].touch_bodies = ["stale"]
    bar_action._apply_movement_touch_policy(
        state, movement, ACTIVE_KEYS, ENV_GEOM, {}, ARM_TO_GROUND, BAR_KEY, TOOL_IDS,
    )
    assert state.rigid_body_states[GROUND_L].touch_bodies == []
    assert state.rigid_body_states[GROUND_R].touch_bodies == []


@pytest.mark.parametrize(
    "movement,expected", [("M1", [BAR_KEY]), ("M2", [BAR_KEY]), ("M3", []), ("M4", [])],
)
def test_touch_policy_unclassified_ground_female_like(state, movement, expected):
    """A tool-less ground joint gets the carried-female policy ([bar] while held)."""
    bar_action._apply_movement_touch_policy(
        state, movement, ACTIVE_KEYS, ENV_GEOM, {}, {}, BAR_KEY, TOOL_IDS,
    )
    assert state.rigid_body_states[GROUND_L].touch_bodies == expected
    assert state.rigid_body_states[GROUND_R].touch_bodies == expected


def test_touch_policy_male_path_unchanged():
    """Regression: a normal male+female bar reproduces today's exact whitelists."""
    bar_key = "bar_B9"
    male = "joint_J35-9_male"          # grasped by the right tool
    carried = "joint_J9-1_female"      # carried female bonded to B9
    mate = "joint_J35-9_female"        # built mate on bar_B35
    env_geom = {
        bar_key: {"frame_world_mm": np.eye(4), "parent_bar_id": "B9"},
        male: {"frame_world_mm": np.eye(4), "parent_bar_id": "B9"},
        carried: {"frame_world_mm": np.eye(4), "parent_bar_id": "B9"},
        mate: {"frame_world_mm": np.eye(4), "parent_bar_id": "B35"},
    }
    active_keys = {bar_key, male, carried}
    st = _FakeState(set(env_geom))

    bar_action._apply_movement_touch_policy(
        st, "M2", active_keys, env_geom, {"J35-9": "right"}, {}, bar_key, TOOL_IDS,
    )
    # M2 mate extras: the mate female AND the mate female's parent bar.
    assert st.rigid_body_states[male].touch_bodies == sorted(
        {"AT3R", bar_key, mate, "bar_B35"}
    )
    assert st.rigid_body_states[carried].touch_bodies == [bar_key]
    assert st.rigid_body_states[bar_key].touch_bodies == ["AT3L", "AT3R"]


# ---------------------------------------------------------------------------
# M3 retreat targets (ground fallback)
# ---------------------------------------------------------------------------


def _build_m3(arm_to_male, arm_to_ground):
    return bar_action._build_m3(
        _FakeState(ACTIVE_KEYS), "B20", ENV_GEOM, ACTIVE_KEYS,
        arm_to_male, arm_to_ground, BAR_KEY, TOOL_IDS, frozenset(),
        TOOL0_LEFT, TOOL0_RIGHT, np.eye(4),
        assembled_groups=None, retreat_distance_mm=LM,
    )


def test_build_m3_ground_retreat_targets():
    """Each arm retreats along ITS grasped ground block's world -Z by LM mm."""
    m3 = _build_m3({}, ARM_TO_GROUND)
    # ground 0 local Z = +X -> left retreats along -X; ground 1 local Z = +Y ->
    # right retreats along -Y. Frames are meters (mm / 1000).
    assert np.allclose(m3.target_ee_frames["left"].point, [(100.0 - LM) / 1000.0, 0.0, 0.5])
    assert np.allclose(m3.target_ee_frames["right"].point, [0.9, -LM / 1000.0, 0.5])
    axes = m3.notes["retreat_axes_world"]
    assert np.allclose(axes["left"], [-1.0, 0.0, 0.0])
    assert np.allclose(axes["right"], [0.0, -1.0, 0.0])


def test_build_m3_no_anchor_prints_note_and_keeps_assembled(capsys):
    """With no classified anchors the targets stay at the assembled flanges, loudly."""
    m3 = _build_m3({}, {})
    assert np.allclose(m3.target_ee_frames["left"].point, [0.1, 0.0, 0.5])
    assert np.allclose(m3.target_ee_frames["right"].point, [0.9, 0.0, 0.5])
    assert m3.notes["retreat_axes_world"] == {}
    out = capsys.readouterr().out
    assert "no classified male/ground joint" in out


# ---------------------------------------------------------------------------
# M1 approach direction (ground bars: perpendicular to the walkable ground)
# ---------------------------------------------------------------------------


def _build_m1(approach_dir_mm):
    return bar_action._build_m1(
        _FakeState(ACTIVE_KEYS), "B20", ENV_GEOM, ACTIVE_KEYS,
        {}, ARM_TO_GROUND, BAR_KEY, TOOL_IDS, frozenset(),
        TOOL0_LEFT, TOOL0_RIGHT, np.eye(4),
        approach_distance_mm=LM, approach_dir_mm=approach_dir_mm,
    )


def test_build_m1_ground_normal_approach_direction():
    """With a ground normal given, both approach targets shift along it by LM mm."""
    m1 = _build_m1(np.array([1.0, 0.0, 0.0]))  # a deliberately non-vertical normal
    assert np.allclose(m1.target_ee_frames["left"].point, [(100.0 + LM) / 1000.0, 0.0, 0.5])
    assert np.allclose(m1.target_ee_frames["right"].point, [(900.0 + LM) / 1000.0, 0.0, 0.5])
    assert m1.notes["approach_axis"] == "walkable_ground_normal"


def test_build_m1_default_approach_direction_unchanged():
    """Regression: without a normal the legacy -avg(tool z) offset is reproduced."""
    m1 = _build_m1(None)
    # Both tool z axes point world-down, so -avg(tool z) = world up (+Z).
    assert np.allclose(m1.target_ee_frames["left"].point, [0.1, 0.0, (500.0 + LM) / 1000.0])
    assert np.allclose(m1.target_ee_frames["right"].point, [0.9, 0.0, (500.0 + LM) / 1000.0])
    assert m1.notes["approach_axis"] == "neg_avg_tool0_z"
