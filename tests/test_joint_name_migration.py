"""``core.joint_name_migration`` converts a document saved before the
``<Type>_<Subtype>`` rule, and leaves a converted one alone.

Headless: a small fake ``rhinoscriptsyntax`` holds one legacy ground joint on
bar B4 with its tool, one T20SubLeft pair, and the old "last used pair"
settings.
"""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from core import config
from core import joint_name_conventions as jnc


class _FakeDoc:
    """Just enough of rhinoscriptsyntax for the migration."""

    def __init__(self):
        self.blocks = {"T20Ground", "T20_Female", "T20_Male", "T20SubLeft_Female"}
        self.layers = {}   # layer -> [oid]
        self.text = {}     # oid -> {key: value}
        self.names = {}    # oid -> object name
        self.instance_of = {}  # oid -> block definition name
        self.strings = {}  # document strings

    def add(self, oid, layer, block, name, text):
        self.layers.setdefault(layer, []).append(oid)
        self.instance_of[oid] = block
        self.names[oid] = name
        self.text[oid] = dict(text)

    # --- rhinoscriptsyntax surface ---
    def IsBlock(self, name):
        return name in self.blocks

    def RenameBlock(self, old, new):
        self.blocks.discard(old)
        self.blocks.add(new)
        for oid, block in self.instance_of.items():
            if block == old:
                self.instance_of[oid] = new
        return True

    def IsLayer(self, layer):
        return layer in self.layers

    def ObjectsByLayer(self, layer):
        return list(self.layers.get(layer, []))

    def GetUserText(self, oid, key):
        return self.text[oid].get(key)

    def SetUserText(self, oid, key, value=None):
        if value is None:
            self.text[oid].pop(key, None)
        else:
            self.text[oid][key] = value
        return True

    def ObjectName(self, oid, name=None):
        if name is not None:
            self.names[oid] = name
        return self.names[oid]

    def BlockInstanceName(self, oid):
        return self.instance_of[oid]


@pytest.fixture
def doc(monkeypatch):
    fake = _FakeDoc()
    fake.add("g", jnc.LAYER_GROUND, "T20Ground", "G4-T20Ground-0_ground", {
        "joint_id": "G4-T20Ground-0", "joint_type": "ground",
        "ground_joint_name": "T20Ground", "block_name": "T20Ground",
        "parent_bar_id": "B4",
    })
    fake.add("t", config.LAYER_TOOL_INSTANCES, "AT3L", "TG4-T20Ground-0", {
        "joint_id": "G4-T20Ground-0", "tool_id": "TG4-T20Ground-0",
        "block_name": "AT3L", "tool_name": "AT3L",
    })
    fake.add("f", jnc.LAYER_FEMALE, "T20SubLeft_Female", "J4-9_female", {
        "joint_id": "J4-9", "joint_pair_name": "T20SFloorLeft",
        "joint_type": "T20SubLeft", "joint_subtype": "Female",
    })
    fake.add("m", jnc.LAYER_MALE, "T20_Male", "J4-9_male", {
        "joint_id": "J4-9", "joint_pair_name": "T20SFloorLeft",
        "joint_type": "T20", "joint_subtype": "Male",
    })
    fake.strings = {
        "scaffolding.last_joint_pair": "T20Deck12_Pair",
        "scaffolding.last_subfloor_left_pair": "T20SFloorLeft",
        "scaffolding.last_subfloor_right_pair": "T20SFloorRight",
    }

    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake)
    monkeypatch.setitem(sys.modules, "core.rhino_bar_pick", SimpleNamespace(
        _DOC_USERTEXT_PAIR_KEY="scaffolding.last_joint_pair",
        _DOC_USERTEXT_SUBFLOOR_LEFT_KEY="scaffolding.last_subfloor_left_pair",
        _DOC_USERTEXT_SUBFLOOR_RIGHT_KEY="scaffolding.last_subfloor_right_pair",
    ))
    monkeypatch.setitem(sys.modules, "core.rhino_helpers", SimpleNamespace(
        get_doc_string=lambda key: fake.strings.get(key) or None,
        set_doc_string=lambda key, value: fake.strings.__setitem__(key, value),
    ))
    return fake


def _migrate():
    from core.joint_name_migration import migrate_legacy_joint_names

    return migrate_legacy_joint_names("test")


def test_ground_block_definition_is_renamed(doc):
    _migrate()
    assert "T20_Ground" in doc.blocks and "T20Ground" not in doc.blocks
    assert doc.BlockInstanceName("g") == "T20_Ground"


def test_ground_joint_takes_the_new_id_name_and_user_text(doc):
    _migrate()
    assert doc.text["g"]["joint_id"] == "G4-T20-0"
    assert doc.names["g"] == "G4-T20-0_ground"
    assert doc.text["g"]["joint_type"] == "T20"
    assert doc.text["g"]["joint_subtype"] == "Ground"
    assert doc.text["g"]["block_name"] == "T20_Ground"
    assert "ground_joint_name" not in doc.text["g"]


def test_tool_follows_its_ground_joint(doc):
    _migrate()
    assert doc.text["t"]["joint_id"] == "G4-T20-0"
    assert doc.text["t"]["tool_id"] == "TG4-T20-0"
    assert doc.names["t"] == "TG4-T20-0"
    assert doc.text["t"]["block_name"] == "AT3L"  # a tool's own block is untouched


def test_pair_names_and_settings_are_renamed(doc):
    _migrate()
    assert doc.text["f"]["joint_pair_name"] == "T20SubLeft"
    assert doc.text["m"]["joint_pair_name"] == "T20SubLeft"
    assert doc.strings == {
        "scaffolding.last_joint_pair": "T20Deck12",
        "scaffolding.last_subfloor_left_pair": "T20SubLeft",
        "scaffolding.last_subfloor_right_pair": "T20SubRight",
    }


def test_pair_ids_and_names_are_untouched(doc):
    _migrate()
    assert doc.text["f"]["joint_id"] == "J4-9"
    assert doc.names["f"] == "J4-9_female"


def test_second_run_changes_nothing(doc):
    assert _migrate() > 0
    snapshot = (dict(doc.names), {k: dict(v) for k, v in doc.text.items()}, dict(doc.strings))
    assert _migrate() == 0
    assert snapshot == (doc.names, doc.text, doc.strings)
