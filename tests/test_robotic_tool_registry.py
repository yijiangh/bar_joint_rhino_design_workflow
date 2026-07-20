"""Tests for the robotic-tool registry: naming rule + active-pair API.

Headless (no Rhino): exercises ``scripts/core/robotic_tool.py`` against
temporary registry files. Covers:
- the L/R name-suffix rule (``arm_side_from_tool_name``)
- pair resolution from either member (``resolve_pair_for_tool``)
- active-pair persistence + the no-'active'-key inference rule
- ``save_robotic_tool`` replacing exact names while preserving ``active``
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

from core.robotic_tool import (
    RoboticToolDef,
    arm_side_from_tool_name,
    get_active_pair,
    get_active_pair_names,
    load_robotic_tools,
    resolve_pair_for_tool,
    save_robotic_tool,
    set_active_pair,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_tool(name: str) -> RoboticToolDef:
    """A minimal valid tool definition (identity TCP) for registry tests."""
    return RoboticToolDef(
        name=name,
        block_name=f"{name}_block",
        M_tcp_from_block=np.eye(4),
        asset_filename=f"{name}_block.3dm",
        collision_filename=f"{name}_block.obj",
    )


def _registry_with(tmp_path, names: list[str]) -> str:
    """Write a registry at a temp path holding the given tool names."""
    path = os.path.join(str(tmp_path), "robotic_tools.json")
    for name in names:
        save_robotic_tool(_make_tool(name), path)
    return path


# ---------------------------------------------------------------------------
# arm_side_from_tool_name
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name, expected",
    [
        ("AT3L", "left"),
        ("AT3R", "right"),
        ("at4l", "left"),  # case-insensitive suffix
        ("at4r", "right"),
        ("Tool_1", None),  # no L/R suffix
        # ! Sharp edge of the rule: ANY trailing L/R counts, so a word that
        # ! happens to end in 'r' classifies as right. Avoid such names.
        ("Gripper", "right"),
        ("", None),
        ("   ", None),  # whitespace only
    ],
)
def test_arm_side_from_tool_name(name, expected):
    assert arm_side_from_tool_name(name) == expected


# ---------------------------------------------------------------------------
# resolve_pair_for_tool
# ---------------------------------------------------------------------------


def test_resolve_pair_from_either_member(tmp_path):
    path = _registry_with(tmp_path, ["AT4L", "AT4R"])
    tools = load_robotic_tools(path)
    # Clicking the left or the right member must resolve the same pair.
    for member in ("AT4L", "AT4R"):
        pair = resolve_pair_for_tool(member, tools)
        assert pair["left"].name == "AT4L"
        assert pair["right"].name == "AT4R"


def test_resolve_pair_errors(tmp_path):
    path = _registry_with(tmp_path, ["AT4L", "AT3L", "AT3R"])
    tools = load_robotic_tools(path)
    # Partner AT4R was never defined.
    with pytest.raises(ValueError, match="AT4R"):
        resolve_pair_for_tool("AT4L", tools)
    # Name without an L/R suffix.
    with pytest.raises(ValueError, match="suffix"):
        resolve_pair_for_tool("Tool_1", tools)
    # Name not in the registry at all.
    with pytest.raises(ValueError, match="not in the registry"):
        resolve_pair_for_tool("AT9L", tools)


# ---------------------------------------------------------------------------
# Active pair: persistence + round trip
# ---------------------------------------------------------------------------


def test_set_and_get_active_pair(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R", "AT4L", "AT4R"])
    set_active_pair("AT4L", "AT4R", path)
    assert get_active_pair_names(path) == {"left": "AT4L", "right": "AT4R"}
    pair = get_active_pair(path)
    assert pair["left"].name == "AT4L"
    assert pair["right"].name == "AT4R"
    # Switching back works too.
    set_active_pair("AT3L", "AT3R", path)
    assert get_active_pair_names(path) == {"left": "AT3L", "right": "AT3R"}


def test_save_robotic_tool_preserves_active_key(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R"])
    set_active_pair("AT3L", "AT3R", path)
    # Defining a NEW candidate must not wipe the active pair.
    save_robotic_tool(_make_tool("AT4L"), path)
    assert get_active_pair_names(path) == {"left": "AT3L", "right": "AT3R"}
    # And the new tool is really in the file.
    assert "AT4L" in load_robotic_tools(path)


def test_save_robotic_tool_overwrites_same_name_without_duplicate(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R", "AT4L"])
    set_active_pair("AT3L", "AT3R", path)

    replacement = RoboticToolDef(
        name="AT4L",
        block_name="AT4L_revised_block",
        M_tcp_from_block=np.eye(4),
        asset_filename="AT4L_revised_block.3dm",
        collision_filename="AT4L_revised_block.obj",
    )
    save_robotic_tool(replacement, path)

    with open(path, "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    assert [entry["name"] for entry in payload["tools"]].count("AT4L") == 1
    assert payload["active"] == {"left": "AT3L", "right": "AT3R"}

    loaded = load_robotic_tools(path)["AT4L"]
    assert loaded.block_name == "AT4L_revised_block"
    assert loaded.asset_filename == "AT4L_revised_block.3dm"
    assert loaded.collision_filename == "AT4L_revised_block.obj"


def test_set_active_pair_errors(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R"])
    # Unknown tool name.
    with pytest.raises(ValueError, match="unknown tool"):
        set_active_pair("AT9L", "AT3R", path)
    # Sides swapped: suffix does not match the assigned side.
    with pytest.raises(ValueError, match="suffix"):
        set_active_pair("AT3R", "AT3L", path)


# ---------------------------------------------------------------------------
# Active pair: no-'active'-key rules (old-schema files)
# ---------------------------------------------------------------------------


def test_missing_active_key_with_single_pair_is_inferred(tmp_path, capsys):
    # save_robotic_tool never writes 'active' by itself, so this file has none.
    path = _registry_with(tmp_path, ["AT3L", "AT3R"])
    assert get_active_pair_names(path) == {"left": "AT3L", "right": "AT3R"}
    # The inference must be loud, not silent.
    assert "no 'active' entry" in capsys.readouterr().out


def test_missing_active_key_with_multiple_candidates_raises(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R", "AT4L", "AT4R"])
    with pytest.raises(RuntimeError, match="RSSwapRoboticTool"):
        get_active_pair_names(path)


def test_empty_registry_raises(tmp_path):
    path = os.path.join(str(tmp_path), "robotic_tools.json")
    with pytest.raises(RuntimeError, match="No robotic tools registered"):
        get_active_pair_names(path)


def test_invalid_active_entry_raises(tmp_path):
    path = _registry_with(tmp_path, ["AT3L", "AT3R"])
    # Corrupt the 'active' entry by hand (points at an unregistered tool).
    with open(path, "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    payload["active"] = {"left": "AT9L", "right": "AT3R"}
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream)
    with pytest.raises(RuntimeError, match="invalid"):
        get_active_pair_names(path)
