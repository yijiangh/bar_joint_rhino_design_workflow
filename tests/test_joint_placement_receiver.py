"""Paired placement is receiver-role-aware (``core.joint_placement``).

One ``place_joint_blocks`` places both a Female pair and a MoCap pair; nothing
branches on the role.  What has to come out different is exactly the places a
placed half records its role -- its LAYER (which the collision key is built
from), its object NAME and its ``joint_subtype`` user text.

Headless: Rhino is stubbed the same way ``test_rhino_tool_place.py`` does it.
"""

from __future__ import annotations

import contextlib
import importlib
import sys
from collections import defaultdict
from types import SimpleNamespace

import numpy as np
import pytest

from core import joint_name_conventions as jnc
from core.joint_pair import JointHalfDef, JointPairDef


@pytest.fixture
def jp(monkeypatch):
    """A fresh ``core.joint_placement`` bound to stubbed Rhino helpers."""
    fake_helpers = SimpleNamespace(
        curve_endpoints=lambda _cid: ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        numpy_to_xform=lambda matrix, *_a: matrix,
        objects_on_layers=lambda *_layers: [],
        set_object_color=lambda *_a, **_k: None,
        set_objects_layer=lambda *_a, **_k: None,
        suspend_redraw=contextlib.nullcontext,
    )
    monkeypatch.setitem(sys.modules, "core.rhino_helpers", fake_helpers)
    monkeypatch.delitem(sys.modules, "core.joint_placement", raising=False)
    return importlib.import_module("core.joint_placement")


@pytest.fixture
def placed(jp, monkeypatch):
    """Run ``place_joint_blocks`` for a pair; return what landed in the doc."""
    names, text, layers = {}, defaultdict(dict), {}
    fake_rs = SimpleNamespace(
        ObjectName=lambda oid, name: names.__setitem__(oid, name),
        SetUserText=lambda oid, k, v: text[oid].__setitem__(k, v),
    )
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake_rs)
    monkeypatch.setattr(jp, "require_block_definition", lambda name, **_k: name)

    def fake_insert(block_name, _frame, *, layer_name=None, **_k):
        oid = f"oid-{block_name}"
        layers[oid] = layer_name
        return oid

    monkeypatch.setattr(jp, "insert_block_instance", fake_insert)

    result = {
        "female_frame": np.eye(4),
        "male_frame": np.eye(4),
        "fjp": 0.0, "fjr": 0.0, "mjp": 0.0, "mjr": 0.0,
        "residual": 0.0, "variant_index": 0,
        "origin_error_mm": 0.0, "z_axis_error_rad": 0.0,
    }

    def run(receiver_block, receiver_kind):
        pair = JointPairDef(
            name="P",
            female=JointHalfDef(
                block_name=receiver_block, kind=receiver_kind,
                M_block_from_bar=np.eye(4), M_screw_from_block=np.eye(4),
            ),
            male=JointHalfDef(
                block_name="T20_Male", kind="male",
                M_block_from_bar=np.eye(4), M_screw_from_block=np.eye(4),
            ),
            contact_distance_mm=1.0,
        )
        rid, mid, jid = jp.place_joint_blocks(
            result, "le", "ln", "B40", "B53", pair=pair
        )
        return SimpleNamespace(
            jid=jid,
            receiver=SimpleNamespace(name=names[rid], layer=layers[rid], text=text[rid]),
            male=SimpleNamespace(name=names[mid], layer=layers[mid], text=text[mid]),
        )

    return run


def test_female_pair_is_unchanged(placed):
    out = placed("T20_Female", "female")
    assert out.jid == "J40-53"
    assert out.receiver.layer == jnc.LAYER_FEMALE
    assert out.receiver.name == "J40-53_female"
    assert out.receiver.text["joint_type"] == "T20"
    assert out.receiver.text["joint_subtype"] == "Female"
    assert out.male.layer == jnc.LAYER_MALE
    assert out.male.name == "J40-53_male"
    assert out.male.text["joint_subtype"] == "Male"


def test_mocap_pair_lands_on_the_mocap_layer(placed):
    out = placed("T20_MoCap", "mocap")
    assert out.jid == "J40-53"
    assert out.receiver.layer == jnc.LAYER_MOCAP
    assert out.receiver.name == "J40-53_mocap"
    assert out.receiver.text["joint_subtype"] == "MoCap"
    # The male side is untouched by the receiver's kind.
    assert out.male.layer == jnc.LAYER_MALE
    assert out.male.name == "J40-53_male"


def test_mocap_layer_gives_the_mocap_collision_key(placed):
    """``env_collision`` builds the key from the block's LAYER."""
    out = placed("T20_MoCap", "mocap")
    subtype = jnc.subtype_of_layer(out.receiver.layer)
    assert jnc.joint_key(out.jid, subtype) == "joint_J40-53_mocap"


@pytest.mark.parametrize("block, kind", [("T20_Female", "female"), ("T20_MoCap", "mocap")])
def test_bar_keys_name_the_receiver_bar_whatever_its_kind(placed, block, kind):
    out = placed(block, kind)
    # ``female_parent_bar`` keeps its on-disk name for a MoCap receiver too.
    for half in (out.receiver, out.male):
        assert half.text["female_parent_bar"] == "B40"
        assert half.text["male_parent_bar"] == "B53"
        assert half.text["joint_pair_name"] == "P"
    assert "receiver_role" not in out.receiver.text


def test_recover_sides(jp):
    assert jp.RECOVER_SIDES == ("receiver", "male")


@pytest.mark.parametrize("side", ["female", "mocap", "ground"])
def test_recover_side_takes_only_receiver_or_male(jp, side):
    with pytest.raises(ValueError, match="recover_side"):
        jp.compute_variant_with_recovery(
            None, None, None, None, False, False, pair=None, recover_side=side
        )


def test_recover_side_rejects_unknown(jp):
    with pytest.raises(ValueError, match="recover_side"):
        jp.compute_variant_with_recovery(
            None, None, None, None, False, False, pair=None, recover_side="ground"
        )
