"""Every rule in ``core.joint_name_conventions``.

The module is the only place a joint, bar, tool or collision-body name is built
or taken apart, so these tests are the specification: each one states a rule
with real names from this repo, and round-trips it where the rule has an
inverse.  Some strings here are recorded inside saved .3dm files; those are
pinned verbatim and must never change.

Pure Python -- no Rhino.
"""

from __future__ import annotations

import pytest

from core import joint_name_conventions as jnc


# ---------------------------------------------------------------------------
# Subtypes and roles
# ---------------------------------------------------------------------------


def test_the_four_subtypes():
    assert jnc.SUBTYPES == ("Female", "Male", "Ground", "MoCap")


def test_role_is_the_lowercase_subtype_and_round_trips():
    for subtype in jnc.SUBTYPES:
        assert jnc.role(subtype) == subtype.lower()
        assert jnc.subtype_of_role(jnc.role(subtype)) == subtype
    assert jnc.ROLES == ("female", "male", "ground", "mocap")


def test_unknown_subtype_and_role_raise():
    with pytest.raises(ValueError):
        jnc.role("female")  # a role, not a Subtype
    with pytest.raises(ValueError):
        jnc.subtype_of_role("marker")


def test_tool_bearing_is_exactly_male_and_ground():
    """``rs_ik_keyframe`` requires exactly two tool-bearing halves per bar."""
    assert jnc.TOOL_BEARING_SUBTYPES == ("Male", "Ground")


def test_mocap_receives_is_single_sided_and_never_bears_a_tool():
    assert "MoCap" in jnc.RECEIVER_SUBTYPES
    assert "MoCap" in jnc.SINGLE_SIDED_SUBTYPES
    assert "MoCap" not in jnc.TOOL_BEARING_SUBTYPES


def test_receiver_and_tool_bearing_do_not_overlap():
    assert not set(jnc.RECEIVER_SUBTYPES) & set(jnc.TOOL_BEARING_SUBTYPES)


def test_ground_is_the_only_subtype_that_is_not_a_half():
    assert set(jnc.SUBTYPES) - set(jnc.HALF_SUBTYPES) == {"Ground"}


# ---------------------------------------------------------------------------
# Layers
# ---------------------------------------------------------------------------

#: Recorded inside every saved .3dm -- never respell.
SAVED_LAYER_NAMES = {
    "Female": "MANAGED Scaffolding::Joint Female Instances",
    "Male": "MANAGED Scaffolding::Joint Male Instances",
    "Ground": "MANAGED Scaffolding::Joint Ground Instances",
}


def test_saved_layer_names_are_unchanged():
    for subtype, layer in SAVED_LAYER_NAMES.items():
        assert jnc.joint_layer(subtype) == layer


def test_mocap_layer():
    assert jnc.LAYER_MOCAP == "MANAGED Scaffolding::Joint MoCap Instances"


def test_layer_round_trips_and_other_layers_have_no_subtype():
    for subtype in jnc.SUBTYPES:
        assert jnc.subtype_of_layer(jnc.joint_layer(subtype)) == subtype
    assert jnc.subtype_of_layer("MANAGED Scaffolding::Robotic Tool Instances") is None


def test_layer_sets():
    assert jnc.JOINT_LAYERS == tuple(jnc.joint_layer(s) for s in jnc.SUBTYPES)
    assert jnc.TOOL_BEARING_LAYERS == (jnc.LAYER_MALE, jnc.LAYER_GROUND)
    assert jnc.RECEIVER_LAYERS == (jnc.LAYER_FEMALE, jnc.LAYER_MOCAP)
    assert set(jnc.PAIRED_LAYERS) == {jnc.LAYER_FEMALE, jnc.LAYER_MOCAP, jnc.LAYER_MALE}
    assert jnc.LAYER_GROUND not in jnc.PAIRED_LAYERS


# ---------------------------------------------------------------------------
# Block names
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name, type_, subtype",
    [
        ("T20_Female", "T20", "Female"),
        ("T20_Male", "T20", "Male"),
        ("T20_Ground", "T20", "Ground"),
        ("T20_MoCap", "T20", "MoCap"),
        ("T20SubLeft_Female", "T20SubLeft", "Female"),
        ("T20SubRight_Female", "T20SubRight", "Female"),
        ("T20Deck12_Male", "T20Deck12", "Male"),
        ("AT3_E1_Female", "AT3_E1", "Female"),  # a Type may contain "_"
    ],
)
def test_block_name_round_trip(name, type_, subtype):
    assert jnc.split_block_name(name) == (type_, subtype)
    assert jnc.block_type(name) == type_
    assert jnc.block_subtype(name) == subtype
    assert jnc.block_name(type_, subtype) == name


@pytest.mark.parametrize("name", ["T20Ground", "T20_female", "_Female", "T20_Marker", ""])
def test_malformed_block_names_are_rejected(name):
    assert not jnc.is_block_name(name)
    with pytest.raises(ValueError):
        jnc.split_block_name(name)


# ---------------------------------------------------------------------------
# Bars
# ---------------------------------------------------------------------------


def test_bar_ids():
    assert jnc.bar_id(7) == "B7"
    assert jnc.bar_num("B7") == "7"
    assert jnc.bar_num("") == "?"
    assert jnc.bar_number("B12") == 12
    assert jnc.bar_number("X12") is None
    assert jnc.parse_bar_id(" b012 ") == "B12"
    assert jnc.parse_bar_id("12") == "B12"
    assert jnc.parse_bar_id("B1x") is None


# ---------------------------------------------------------------------------
# Joint ids
# ---------------------------------------------------------------------------


def test_pair_joint_id_round_trip():
    assert jnc.pair_joint_id("B40", "B53") == "J40-53"
    assert jnc.split_pair_joint_id("J40-53") == ("B40", "B53")
    assert jnc.split_pair_joint_id("G4-T20-0") is None


@pytest.mark.parametrize(
    "subtype, bar, type_, index, jid",
    [
        ("Ground", "B4", "T20", 0, "G4-T20-0"),
        ("MoCap", "B7", "T20", 2, "M7-T20-2"),
        ("Ground", "B4", "T20Sub", 11, "G4-T20Sub-11"),
    ],
)
def test_single_joint_id_round_trip(subtype, bar, type_, index, jid):
    assert jnc.single_joint_id(subtype, bar, type_, index) == jid
    assert jnc.split_single_joint_id(jid) == (subtype, bar, type_, index)
    assert jnc.single_sided_subtype_of_id(jid) == subtype


def test_pair_ids_are_not_single_sided():
    assert jnc.single_sided_subtype_of_id("J40-53") is None


def test_single_joint_id_rejects_a_paired_subtype():
    with pytest.raises(ValueError):
        jnc.single_joint_id("Female", "B4", "T20", 0)


def test_rebar_single_joint_id_keeps_type_and_index():
    assert jnc.rebar_single_joint_id("G7-T20-2", "B12") == "G12-T20-2"


# ---------------------------------------------------------------------------
# Object names, collision keys, tool ids
# ---------------------------------------------------------------------------


def test_object_name_round_trip():
    assert jnc.object_name("J40-53", "MoCap") == "J40-53_mocap"
    assert jnc.split_object_name("J40-53_mocap") == ("J40-53", "MoCap")
    assert jnc.split_object_name("G4-T20-0_ground") == ("G4-T20-0", "Ground")
    assert jnc.split_object_name("B40") == ("B40", None)


def test_joint_key_round_trip_including_underscore_in_the_id():
    key = jnc.joint_key("J40-53", "MoCap")
    assert key == "joint_J40-53_mocap"
    assert jnc.split_joint_key(key) == ("J40-53", "MoCap")
    # rsplit at the LAST underscore: an id containing "_" still splits right.
    assert jnc.split_joint_key("joint_X_1_female") == ("X_1", "Female")


def test_env_and_plain_keys_are_separate_namespaces():
    assert jnc.joint_key("J1-2", "Male", env=True) == "env_joint_J1-2_male"
    assert jnc.split_joint_key("env_joint_J1-2_male") is None
    assert jnc.split_joint_key("env_joint_J1-2_male", env=True) == ("J1-2", "Male")
    assert jnc.bar_key("B40") == "bar_B40"
    assert jnc.bar_key("B40", env=True) == "env_bar_B40"
    assert jnc.obstacle_key("table") == "obstacle_table"


def test_is_joint_key_and_receiver_keys():
    assert jnc.is_joint_key("joint_J7-41_mocap", jnc.RECEIVER_SUBTYPES)
    assert not jnc.is_joint_key("joint_J7-41_male", jnc.RECEIVER_SUBTYPES)
    assert not jnc.is_joint_key("bar_B7")
    assert jnc.receiver_keys("J7-41") == ("joint_J7-41_female", "joint_J7-41_mocap")


def test_tool_id():
    assert jnc.tool_id("J40-53") == "TJ40-53"
    assert jnc.tool_id("G4-T20-0") == "TG4-T20-0"


def test_user_text_keys_are_the_strings_on_disk():
    assert jnc.UT_JOINT_ID == "joint_id"
    assert jnc.UT_PARENT_BAR == "parent_bar_id"
    assert jnc.UT_RECEIVER_BAR == "female_parent_bar"
    assert jnc.UT_MALE_BAR == "male_parent_bar"
    assert jnc.UT_PAIR_NAME == "joint_pair_name"


# ---------------------------------------------------------------------------
# Mate names
# ---------------------------------------------------------------------------

REGISTERED = [
    "T20_Female", "T20_Male", "T20SubLeft_Female", "T20SubRight_Female", "T20Deck12_Male",
]


@pytest.mark.parametrize(
    "receiver, male, expected",
    [
        ("T20_Female", "T20_Male", "T20"),
        ("T20_Female", "T20Deck12_Male", "T20Deck12"),
        ("T20SubLeft_Female", "T20_Male", "T20SubLeft"),
        ("T20SubRight_Female", "T20_Male", "T20SubRight"),
    ],
)
def test_default_mate_name(receiver, male, expected):
    assert jnc.default_mate_name(receiver, male, REGISTERED) == expected


# ---------------------------------------------------------------------------
# Legacy migration
# ---------------------------------------------------------------------------


def test_legacy_ground_id_migrates_to_the_type():
    assert jnc.migrate_joint_id("G4-T20Ground-0") == "G4-T20-0"
    assert jnc.migrate_tool_id("TG4-T20Ground-0") == "TG4-T20-0"


def test_migration_leaves_current_ids_alone():
    for jid in ("G4-T20-0", "J40-53", "M7-T20-1", ""):
        assert jnc.migrate_joint_id(jid) == jid
    assert jnc.migrate_tool_id("TJ40-53") == "TJ40-53"


def test_legacy_renames_land_on_valid_names():
    for new in jnc.LEGACY_BLOCK_RENAMES.values():
        assert jnc.is_block_name(new)
    assert jnc.LEGACY_MATE_RENAMES == {
        "T20Deck12_Pair": "T20Deck12",
        "T20SFloorLeft": "T20SubLeft",
        "T20SFloorRight": "T20SubRight",
    }
