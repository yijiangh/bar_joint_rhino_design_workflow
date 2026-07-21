"""Headless tests for robotic-tool document replacement behavior."""

from __future__ import annotations

import contextlib
import sys
from types import SimpleNamespace

from core import rhino_tool_place


def test_replace_all_tool_instances_imports_pair_when_layer_is_missing(
    monkeypatch,
):
    """A separate IK document still gets both definitions without any picks."""
    fake_rs = SimpleNamespace(IsLayer=lambda _name: False)
    fake_helpers = SimpleNamespace(suspend_redraw=contextlib.nullcontext)
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake_rs)
    monkeypatch.setitem(sys.modules, "core.rhino_helpers", fake_helpers)

    refreshed = []
    monkeypatch.setattr(
        rhino_tool_place,
        "refresh_block_definition",
        lambda block_name, asset_path: refreshed.append((block_name, asset_path)),
    )

    pair = {
        "left": SimpleNamespace(
            block_name="AT4L", asset_path=lambda: "asset/AT4L.3dm"
        ),
        "right": SimpleNamespace(
            block_name="AT4R", asset_path=lambda: "asset/AT4R.3dm"
        ),
    }

    summary = rhino_tool_place.replace_all_tool_instances(pair)

    assert summary == {"replaced": 0, "left": 0, "right": 0}
    assert refreshed == [
        ("AT4L", "asset/AT4L.3dm"),
        ("AT4R", "asset/AT4R.3dm"),
    ]
