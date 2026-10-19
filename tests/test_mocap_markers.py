"""``core.mocap_markers`` -- the ``MoCapMarkers.json`` payload (schema 2).

The real ``T20_MoCap`` registry points, placed on two synthetic bars the way
the assembly cell describes them (bar bodies with their frames, joint bodies
with their block transforms).  Pure numpy.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from core import joint_name_conventions as jnc
from core import mocap_markers as mm
from core.joint_pair import JointHalfDef, canonical_bar_frame_from_line, load_joint_registry
from core.mocap_layout_check import MoCapLayoutError
from core.single_sided_placement import fk_single_block_frame
from core.transforms import transform_point

REGISTRY = load_joint_registry()
HALVES = dict(REGISTRY.halves)
T20_MOCAP = HALVES["T20_MoCap"]

B20 = ((1325.0, -2250.0, 150.0), (-475.0, -2250.0, 150.0))   # horizontal, 1800 mm
B21 = ((925.0, -2314.0, 50.0), (925.0, -2314.0, 1850.0))     # vertical, 1800 mm
B22 = ((0.0, 0.0, 0.0), (0.0, 0.0, 1000.0))                   # no MoCap joints


def _bar_body(bar_id, bar):
    start, end = (np.asarray(p, dtype=float) for p in bar)
    return {"kind": "bar", "parent_bar_id": bar_id, "frame_world_mm": canonical_bar_frame_from_line(start, end)}


def _mocap_body(bar_id, bar, along_mm, spin_deg, block="T20_MoCap"):
    """A MoCap block placed on *bar* the way RSJointPlace places it."""
    start, end = (np.asarray(p, dtype=float) for p in bar)
    world = fk_single_block_frame(start, end, along_mm, np.radians(spin_deg), T20_MOCAP)
    return {"kind": "joint", "frame_world_mm": world, "block_name": block, "subtype": "MoCap",
            "parent_bar_id": bar_id}


@pytest.fixture
def bodies():
    out = {
        "bar_B20": _bar_body("B20", B20),
        "bar_B21": _bar_body("B21", B21),
        "bar_B22": _bar_body("B22", B22),
        # Listed out of order along B20 on purpose; three joints, mixed facing.
        jnc.joint_key("J20-169", "MoCap"): _mocap_body("B20", B20, 1145.3, 0),
        jnc.joint_key("J20-22", "MoCap"): _mocap_body("B20", B20, 84.9, 90),
        jnc.joint_key("J20-21", "MoCap"): _mocap_body("B20", B20, 420.0, 180),
        # Three on B21, one standalone.
        jnc.joint_key("J21-37", "MoCap"): _mocap_body("B21", B21, 1471.9, 0),
        jnc.joint_key("M21-T20-0", "MoCap"): _mocap_body("B21", B21, 396.0, 45),
        jnc.joint_key("J21-26", "MoCap"): _mocap_body("B21", B21, 1080.0, 270),
        # Not MoCap: ignored.
        jnc.joint_key("J20-21", "Male"): _mocap_body("B20", B20, 420.0, 0, "T20_Male") | {"subtype": "Male"},
    }
    return out


SEQ = {"B20": 3, "B21": 7, "B22": None}


@pytest.fixture
def payload(bodies):
    return mm.build_payload(bodies, HALVES, SEQ, source="test.3dm")


# ---------------------------------------------------------------------------
# Shape of the file
# ---------------------------------------------------------------------------


def test_schema_header(payload):
    assert payload["schema_version"] == 2
    assert payload["units"] == "m"
    assert payload["cell_file"] == "RobotCell.json"
    assert payload["source"] == "test.3dm"
    assert payload["layout_tolerance_m"] == pytest.approx(0.002)


def test_marker_blocks_carry_the_registry_points_in_metres(payload):
    block = payload["marker_blocks"]["T20_MoCap"]
    assert block["marker_diameter_m"] == pytest.approx(0.019)
    for label, point in T20_MOCAP.marker_points_mm.items():
        assert block["points_block_m"][label] == pytest.approx(np.asarray(point) / 1000.0, abs=1e-5)
    assert list(payload["marker_blocks"]) == ["T20_MoCap"]


def test_bars_are_keyed_by_body_name_in_assembly_order(payload):
    assert list(payload["bars"]) == ["bar_B20", "bar_B21"]
    assert payload["bars"]["bar_B20"]["bar_id"] == "B20"
    assert payload["bars"]["bar_B20"]["assembly_seq"] == 3
    assert payload["bars"]["bar_B21"]["assembly_seq"] == 7
    assert payload["bars_without_markers"] == ["bar_B22"]


def test_bars_without_a_sequence_number_come_last(bodies):
    bodies[jnc.joint_key("M22-T20-0", "MoCap")] = _mocap_body("B22", B22, 100.0, 0)
    bodies[jnc.joint_key("M22-T20-1", "MoCap")] = _mocap_body("B22", B22, 500.0, 90)
    bodies[jnc.joint_key("M22-T20-2", "MoCap")] = _mocap_body("B22", B22, 900.0, 0)
    out = mm.build_payload(bodies, HALVES, SEQ)
    assert list(out["bars"]) == ["bar_B20", "bar_B21", "bar_B22"]
    assert out["bars"]["bar_B22"]["assembly_seq"] is None
    assert out["bars_without_markers"] == []


def test_markers_ordered_along_the_bar_then_by_label(payload):
    b20 = payload["bars"]["bar_B20"]["markers"]
    assert [m["joint"] for m in b20[::2]] == [
        jnc.joint_key("J20-22", "MoCap"), jnc.joint_key("J20-21", "MoCap"), jnc.joint_key("J20-169", "MoCap"),
    ]
    assert [m["label"] for m in b20] == ["M1", "M2"] * 3
    assert [m["name"] for m in b20[:2]] == ["J20-22_mocap_M1", "J20-22_mocap_M2"]
    assert all(m["block"] == "T20_MoCap" for m in b20)
    along = [m["bar_m"][2] for m in b20[::2]]
    assert along == sorted(along)


def test_standalone_and_paired_joints_both_listed(payload):
    b21 = payload["bars"]["bar_B21"]["markers"]
    assert [m["joint"] for m in b21[::2]] == [
        jnc.joint_key("M21-T20-0", "MoCap"), jnc.joint_key("J21-26", "MoCap"), jnc.joint_key("J21-37", "MoCap"),
    ]
    assert b21[0]["name"] == "M21-T20-0_mocap_M1"


# ---------------------------------------------------------------------------
# The numbers
# ---------------------------------------------------------------------------


def test_bar_frame_positions_agree_with_the_block_transform(payload, bodies):
    """The invariant the monitor relies on: bar_frame @ bar_m == joint_frame @ block_point."""
    for bar_key, bar in payload["bars"].items():
        bar_frame = np.asarray(bodies[bar_key]["frame_world_mm"])
        for marker in bar["markers"]:
            joint_frame = np.asarray(bodies[marker["joint"]]["frame_world_mm"])
            expected = transform_point(joint_frame, T20_MOCAP.marker_points_mm[marker["label"]])
            got = transform_point(bar_frame, np.asarray(marker["bar_m"]) * 1000.0)
            assert got == pytest.approx(expected, abs=0.02)


def test_world_points_match_the_same_invariant(payload, bodies):
    world = mm.world_points_mm(payload, bodies)
    assert len(world) == 12
    joint = np.asarray(bodies[jnc.joint_key("J20-22", "MoCap")]["frame_world_mm"])
    assert world["J20-22_mocap_M1"] == pytest.approx(
        transform_point(joint, T20_MOCAP.marker_points_mm["M1"]), abs=0.02
    )


def test_markers_sit_25_5_mm_off_the_bar_axis_and_48_3_mm_apart(payload):
    for bar in payload["bars"].values():
        pts = [np.asarray(m["bar_m"]) * 1000.0 for m in bar["markers"]]
        for a, b in zip(pts[::2], pts[1::2]):
            assert np.linalg.norm(a - b) == pytest.approx(48.3, abs=0.02)
            assert np.linalg.norm(((a + b) / 2.0)[:2]) == pytest.approx(25.5, abs=0.02)


def test_positions_are_rounded_to_five_decimals(payload):
    for bar in payload["bars"].values():
        for marker in bar["markers"]:
            for c in marker["bar_m"]:
                assert c == round(c, 5)


# ---------------------------------------------------------------------------
# A range export: the cut body set
# ---------------------------------------------------------------------------


def test_cut_body_set_drops_the_bar_and_its_joints_without_error(bodies):
    cut = {k: v for k, v in bodies.items() if v.get("parent_bar_id") != "B21"}
    out = mm.build_payload(cut, HALVES, SEQ)
    assert list(out["bars"]) == ["bar_B20"]
    assert out["bars_without_markers"] == ["bar_B22"]


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_joint_whose_bar_is_missing_stops_the_export(bodies):
    bodies[jnc.joint_key("J99-1", "MoCap")] = _mocap_body("B99", B20, 10.0, 0)
    with pytest.raises(mm.MoCapExportError, match="B99"):
        mm.build_payload(bodies, HALVES, SEQ)


def test_block_without_marker_points_stops_the_export(bodies):
    bodies[jnc.joint_key("J20-30", "MoCap")] = _mocap_body("B20", B20, 10.0, 0, "T40_MoCap")
    with pytest.raises(mm.MoCapExportError, match="T40_MoCap"):
        mm.build_payload(bodies, HALVES, SEQ)


def test_block_without_marker_diameter_stops_the_export(bodies):
    halves = dict(HALVES)
    data = T20_MOCAP.to_dict()
    data["marker_diameter_mm"] = 0.0
    halves["T20_MoCap"] = JointHalfDef.from_dict(data)
    with pytest.raises(mm.MoCapExportError, match="diameter"):
        mm.build_payload(bodies, halves, SEQ)


def test_two_joint_bar_is_refused_by_name(bodies):
    del bodies[jnc.joint_key("J20-169", "MoCap")]
    with pytest.raises(MoCapLayoutError, match="bar B20") as info:
        mm.build_payload(bodies, HALVES, SEQ)
    assert "three MoCap joints" in str(info.value)


def test_three_same_facing_joints_are_refused(bodies):
    # Turn B20's three joints to face the same way: the pair swap becomes a symmetry.
    bodies[jnc.joint_key("J20-22", "MoCap")] = _mocap_body("B20", B20, 84.9, 0)
    bodies[jnc.joint_key("J20-21", "MoCap")] = _mocap_body("B20", B20, 420.0, 0)
    with pytest.raises(MoCapLayoutError, match="bar B20"):
        mm.build_payload(bodies, HALVES, SEQ)


# ---------------------------------------------------------------------------
# Reading back
# ---------------------------------------------------------------------------


def test_label_order_is_numeric():
    assert sorted(["M10", "M2", "M1"], key=mm.label_sort_key) == ["M1", "M2", "M10"]


def test_payload_is_plain_json_and_summarises(payload):
    text = json.dumps(payload)
    back = json.loads(text)
    assert back["bars"]["bar_B20"]["markers"][0]["name"] == "J20-22_mocap_M1"
    assert mm.marker_count(payload) == 12
    assert mm.summary(payload) == "B20: 6 markers on 3 joints; B21: 6 markers on 3 joints"


def test_layout_notes_flag_a_narrow_pass():
    """A bar whose nearest miss is under 3x the tolerance gets a note; a wide miss does not."""
    narrow = {"layout_tolerance_m": 0.002, "bars": {
        "bar_B1": {"bar_id": "B1", "layout": {"n_markers": 6, "nearest_miss_m": 0.004}},
        "bar_B2": {"bar_id": "B2", "layout": {"n_markers": 6, "nearest_miss_m": 0.040}},
        "bar_B3": {"bar_id": "B3", "layout": {"n_markers": 6, "nearest_miss_m": None}},
    }}
    notes = mm.layout_notes(narrow)
    assert len(notes) == 1 and notes[0].startswith("B1:") and "4.0 mm" in notes[0]
