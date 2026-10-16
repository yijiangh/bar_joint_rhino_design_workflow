"""``core.mocap_markers`` -- the ``MoCapMarkers.json`` payload.

The real ``T20_MoCap`` marker points from the registry, placed on two
synthetic bars the way the assembly cell describes them.  Pure numpy.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from core import joint_name_conventions as jnc
from core import mocap_markers as mm
from core.joint_pair import canonical_bar_frame_from_line, load_joint_registry
from core.single_sided_placement import fk_single_block_frame
from core.transforms import transform_point

T20_MOCAP = load_joint_registry().halves["T20_MoCap"]
MARKERS = {"T20_MoCap": T20_MOCAP.marker_points_mm}

B20 = ((1325.0, -2250.0, 150.0), (-475.0, -2250.0, 150.0))   # horizontal, 1800 mm
B21 = ((925.0, -2314.0, 50.0), (925.0, -2314.0, 1850.0))     # vertical, 1800 mm


def _mocap_body(bar, along_mm, spin_deg, block="T20_MoCap"):
    """A MoCap block placed on *bar* the way RSJointPlace places it."""
    start, end = (np.asarray(p, dtype=float) for p in bar)
    world = fk_single_block_frame(start, end, along_mm, np.radians(spin_deg), T20_MOCAP)
    return {"kind": "joint", "frame_world_mm": world, "block_name": block, "subtype": "MoCap"}


@pytest.fixture
def bodies():
    out = {
        # Listed out of order along B20 on purpose.
        jnc.joint_key("J20-169", "MoCap"): _mocap_body(B20, 1145.3, 0) | {"parent_bar_id": "B20"},
        jnc.joint_key("J20-22", "MoCap"): _mocap_body(B20, 84.9, 90) | {"parent_bar_id": "B20"},
        jnc.joint_key("J20-21", "MoCap"): _mocap_body(B20, 420.0, 180) | {"parent_bar_id": "B20"},
        # Three on B21, one standalone.
        jnc.joint_key("J21-37", "MoCap"): _mocap_body(B21, 1471.9, 0) | {"parent_bar_id": "B21"},
        jnc.joint_key("M21-T20-0", "MoCap"): _mocap_body(B21, 396.0, 45) | {"parent_bar_id": "B21"},
        jnc.joint_key("J21-26", "MoCap"): _mocap_body(B21, 1080.0, 270) | {"parent_bar_id": "B21"},
        # Not MoCap: ignored.
        jnc.joint_key("J20-21", "Male"): _mocap_body(B20, 420.0, 0, "T20_Male") | {"parent_bar_id": "B20"},
        "bar_B20": {"kind": "bar", "parent_bar_id": "B20"},
    }
    out[jnc.joint_key("J20-21", "Male")]["subtype"] = "Male"
    return out


@pytest.fixture
def payload(bodies):
    lines = {"B20": B20, "B21": B21, "B22": ((0, 0, 0), (0, 0, 1000))}
    return mm.designed_markers(bodies, MARKERS, lines, ["B20", "B21", "B22"], source="test.3dm")


def test_marker_name_rule():
    assert jnc.marker_name("J20-21", "MoCap", "M1") == "J20-21_mocap_M1"


def test_bars_in_assembly_order_and_every_bar_listed(payload):
    assert [b["bar_id"] for b in payload["bars"]] == ["B20", "B21", "B22"]
    assert payload["bars"][2]["markers"] == [] and payload["bars"][2]["joints"] == []


def test_joints_ordered_along_the_bar_and_markers_numbered(payload):
    b20 = payload["bars"][0]
    assert [j["joint_id"] for j in b20["joints"]] == ["J20-22", "J20-21", "J20-169"]
    assert [m["index"] for m in b20["markers"]] == [1, 2, 3, 4, 5, 6]
    assert [m["name"] for m in b20["markers"][:2]] == ["J20-22_mocap_M1", "J20-22_mocap_M2"]
    assert b20["joints"][0]["marker_indices"] == [1, 2]
    positions = [j["position_mm"] for j in b20["joints"]]
    assert positions == sorted(positions)


def test_three_joints_and_a_standalone_mocap(payload):
    b21 = payload["bars"][1]
    assert [j["joint_id"] for j in b21["joints"]] == ["M21-T20-0", "J21-26", "J21-37"]
    assert len(b21["markers"]) == 6
    assert [j["paired"] for j in b21["joints"]] == [False, True, True]


def test_world_and_bar_frame_positions_agree(payload, bodies):
    bar = payload["bars"][0]
    frame = canonical_bar_frame_from_line(*B20)
    block = np.asarray(bodies[jnc.joint_key("J20-22", "MoCap")]["frame_world_mm"])
    m1 = bar["markers"][0]
    expected = transform_point(block, MARKERS["T20_MoCap"]["M1"])
    assert m1["world_mm"] == pytest.approx(expected, abs=0.01)
    assert transform_point(frame, m1["bar_mm"]) == pytest.approx(m1["world_mm"], abs=0.02)


def test_markers_sit_25_5_mm_off_the_bar_axis_and_48_3_mm_apart(payload):
    for bar in payload["bars"][:2]:
        for joint in bar["joints"]:
            a, b = (np.asarray(bar["markers"][i - 1]["bar_mm"]) for i in joint["marker_indices"])
            assert np.linalg.norm(a - b) == pytest.approx(48.3, abs=0.02)
            mid = (a + b) / 2.0
            assert np.linalg.norm(mid[:2]) == pytest.approx(25.5, abs=0.02)


def test_problems_become_warnings(bodies):
    bodies[jnc.joint_key("J99-1", "MoCap")] = _mocap_body(B20, 10, 0) | {"parent_bar_id": "B99"}
    bodies[jnc.joint_key("J20-30", "MoCap")] = _mocap_body(B20, 10, 0, "T40_MoCap") | {"parent_bar_id": "B20"}
    out = mm.designed_markers(bodies, MARKERS, {"B20": B20, "B21": B21})
    assert len(out["warnings"]) == 2
    assert "B99" in out["warnings"][0] or "B99" in out["warnings"][1]


def test_label_order_is_numeric():
    assert sorted(["M10", "M2", "M1"], key=mm.label_sort_key) == ["M1", "M2", "M10"]


def test_payload_is_plain_json(payload):
    text = json.dumps(payload)
    assert json.loads(text)["bars"][0]["markers"][0]["name"] == "J20-22_mocap_M1"
    assert mm.marker_count(payload) == 12
    assert mm.summary(payload) == "B20: 6 markers on 3 joints; B21: 6 markers on 3 joints"
