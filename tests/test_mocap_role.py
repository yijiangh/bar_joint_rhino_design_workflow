"""The MoCap half in the joint registry (``core.joint_pair``).

A MoCap half is a Female with a marker plate bolted to its back: a male seats
into it exactly as into a Female, but it is fitted by hand and carries
OptiTrack marker spheres.  It has no mate of its own -- it is placed through
its Type's Female mate (``with_receiver``).  These tests pin the registry-level
facts the placement code leans on: the half kind comes from the block name,
the receiver swap stays inside one Type, and ``marker_points_mm`` survives a
JSON round trip.

Pure Python -- no Rhino.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from core.joint_pair import (
    GroundJointDef,
    JointHalfDef,
    JointPairDef,
    JointRegistry,
    get_joint_pair,
    load_joint_registry,
    receiver_subtypes,
    save_joint_registry,
    swapped_receiver,
    with_receiver,
)


def _half(block_name: str, kind: str = "", **kw) -> JointHalfDef:
    return JointHalfDef(
        block_name=block_name,
        kind=kind,
        M_block_from_bar=np.eye(4),
        M_screw_from_block=np.eye(4),
        **kw,
    )


def _pair(name: str, receiver: JointHalfDef, male: JointHalfDef) -> JointPairDef:
    return JointPairDef(name=name, female=receiver, male=male, contact_distance_mm=36.0)


@pytest.fixture
def registry():
    """Today's registry plus a T20_MoCap half -- and no T20MoCap mate."""
    halves = {
        h.block_name: h
        for h in (
            _half("T20_Female"),
            _half("T20_Male"),
            _half("T20_MoCap"),
            _half("T20SubLeft_Female", bar_cradle=True),
            _half("T20Deck12_Male"),
        )
    }
    mates = {
        p.name: p
        for p in (
            _pair("T20", halves["T20_Female"], halves["T20_Male"]),
            _pair("T20SubLeft", halves["T20SubLeft_Female"], halves["T20_Male"]),
            _pair("T20Deck12", halves["T20_Female"], halves["T20Deck12_Male"]),
        )
    }
    return JointRegistry(halves=halves, mates=mates)


# ---------------------------------------------------------------------------
# The kind comes from the block name
# ---------------------------------------------------------------------------


def test_half_kinds_are_every_role_but_ground():
    from core import joint_name_conventions as jnc

    assert [jnc.role(s) for s in jnc.HALF_SUBTYPES] == ["female", "male", "mocap"]


@pytest.mark.parametrize(
    "block_name, kind",
    [("T20_Female", "female"), ("T20_Male", "male"), ("T20_MoCap", "mocap")],
)
def test_kind_is_derived_from_the_block_name(block_name, kind):
    half = _half(block_name)
    assert half.kind == kind
    assert half.subtype == block_name.rpartition("_")[2]
    assert half.type == "T20"


def test_a_kind_that_contradicts_the_block_name_is_rejected():
    with pytest.raises(ValueError, match="block name says"):
        _half("T20_MoCap", "female")


@pytest.mark.parametrize("block_name", ["T20Ground", "T20_Ground", "T20_Marker"])
def test_non_half_blocks_are_rejected(block_name):
    with pytest.raises(ValueError):
        _half(block_name)


def test_ground_def_needs_a_ground_block():
    GroundJointDef(name="T20_Ground", block_name="T20_Ground", M_block_from_bar=np.eye(4))
    with pytest.raises(ValueError):
        GroundJointDef(name="T20Ground", block_name="T20Ground", M_block_from_bar=np.eye(4))


def test_a_mate_needs_a_receiver_and_a_male(registry):
    halves = registry.halves
    with pytest.raises(ValueError):
        _pair("bad", halves["T20_Male"], halves["T20_Male"])
    with pytest.raises(ValueError):
        _pair("bad", halves["T20_Female"], halves["T20_Female"])
    # A MoCap receiver is a valid slot filler (that is what with_receiver builds).
    assert _pair("ok", halves["T20_MoCap"], halves["T20_Male"]).receiver_subtype == "MoCap"


# ---------------------------------------------------------------------------
# marker_points_mm
# ---------------------------------------------------------------------------


def test_marker_points_are_normalized_to_plain_floats():
    half = _half("T20_MoCap", marker_points_mm={"M1": np.array([1, 2, 3])})
    assert half.marker_points_mm == {"M1": (1.0, 2.0, 3.0)}
    assert all(type(c) is float for c in half.marker_points_mm["M1"])
    json.dumps(half.to_dict())  # numpy values would fail here


def test_marker_point_with_wrong_length_names_its_label():
    with pytest.raises(ValueError, match="M2"):
        _half("T20_MoCap", marker_points_mm={"M2": (1.0, 2.0)})


def test_marker_points_default_empty_and_not_shared():
    a = _half("A_Female")
    b = _half("B_Female")
    assert a.marker_points_mm == {}
    assert a.marker_points_mm is not b.marker_points_mm


def test_old_registry_entry_without_marker_points_or_kind_loads():
    data = _half("T20_Female").to_dict()
    del data["marker_points_mm"]
    del data["kind"]
    half = JointHalfDef.from_dict(data)
    assert half.marker_points_mm == {}
    assert half.kind == "female"


def test_registry_round_trip_keeps_marker_points(tmp_path, registry):
    pts = {"M1": (10.0, 0.0, 5.5), "M2": (-10.0, 0.0, 5.5)}
    halves = dict(registry.halves)
    halves["T20_MoCap"] = _half("T20_MoCap", marker_points_mm=pts)
    path = str(tmp_path / "joint_pairs.json")
    save_joint_registry(JointRegistry(halves=halves, mates=registry.mates), path)
    loaded = load_joint_registry(path)
    assert loaded.halves["T20_MoCap"].marker_points_mm == pts
    assert loaded.halves["T20_MoCap"].kind == "mocap"


# ---------------------------------------------------------------------------
# Receiver variants -- MoCap rides on its Type's Female mate
# ---------------------------------------------------------------------------


def test_receiver_is_the_female_slot(registry):
    for pair in registry.mates.values():
        assert pair.receiver is pair.female


def test_receiver_subtypes_lists_what_the_type_has(registry):
    halves = registry.halves
    assert receiver_subtypes(registry.mates["T20"], halves) == ["Female", "MoCap"]
    # T20SubLeft has no MoCap block; T20Deck12's receiver is a T20 Female.
    assert receiver_subtypes(registry.mates["T20SubLeft"], halves) == ["Female"]
    assert receiver_subtypes(registry.mates["T20Deck12"], halves) == ["Female", "MoCap"]


def test_with_receiver_swaps_only_the_receiving_half(registry):
    t20 = registry.mates["T20"]
    mocap = with_receiver(t20, "MoCap", registry.halves)
    assert mocap.name == "T20"
    assert mocap.receiver.block_name == "T20_MoCap"
    assert mocap.receiver_subtype == "MoCap"
    assert mocap.male is t20.male
    assert mocap.contact_distance_mm == t20.contact_distance_mm
    assert with_receiver(mocap, "Female", registry.halves).receiver is t20.receiver


def test_with_receiver_is_identity_for_the_current_receiver(registry):
    t20 = registry.mates["T20"]
    assert with_receiver(t20, "Female", registry.halves) is t20


def test_with_receiver_raises_when_the_type_has_no_such_block(registry):
    with pytest.raises(KeyError, match="T20SubLeft_MoCap"):
        with_receiver(registry.mates["T20SubLeft"], "MoCap", registry.halves)


def test_mocap_needs_no_mate_row(tmp_path, registry):
    """The mates table is unchanged by a MoCap half: no T20MoCap row."""
    path = str(tmp_path / "joint_pairs.json")
    save_joint_registry(registry, path)
    data = json.load(open(path, encoding="utf-8"))
    assert sorted(m["name"] for m in data["mates"]) == ["T20", "T20Deck12", "T20SubLeft"]
    assert get_joint_pair("T20", "MoCap", path=path).receiver.block_name == "T20_MoCap"
    assert get_joint_pair("T20", path=path).receiver.block_name == "T20_Female"


# ---------------------------------------------------------------------------
# swapped_receiver -- what RSJointEdit > ReplaceJoint applies
# ---------------------------------------------------------------------------


def test_swapped_receiver_goes_female_to_mocap_and_back(registry):
    t20 = registry.mates["T20"]
    to_mocap = swapped_receiver(t20, registry.halves)
    assert to_mocap.receiver.block_name == "T20_MoCap"
    assert to_mocap.name == "T20" and to_mocap.male is t20.male
    back = swapped_receiver(to_mocap, registry.halves)
    assert back.receiver is t20.receiver


def test_swapped_receiver_needs_the_other_block(registry):
    with pytest.raises(KeyError, match="T20SubLeft_MoCap"):
        swapped_receiver(registry.mates["T20SubLeft"], registry.halves)


def test_registry_definition_finds_halves_and_grounds(registry):
    from core.joint_pair import GroundJointDef

    ground = GroundJointDef(name="T20_Ground", block_name="T20_Ground", M_block_from_bar=np.eye(4))
    registry.ground_joints[ground.name] = ground
    assert registry.definition("T20_MoCap") is registry.halves["T20_MoCap"]
    assert registry.definition("T20_Ground") is ground
    assert registry.definition("T20_Nothing") is None
    assert ground in registry.definitions()
