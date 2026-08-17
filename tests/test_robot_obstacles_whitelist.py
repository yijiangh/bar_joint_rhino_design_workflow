"""Headless tests for ``robot_obstacles.whitelist_frozen_contact``.

A frozen holding robot (an unattached obstacle ToolModel) is clamped around its
held bar, so that one tool<->bar contact must be allowed or every IK candidate
in the scene is vetoed. These tests drive the whitelist helper with fake scene
states covering both bar naming schemes (`bar_<id>` in the assembly cell,
`env_bar_<id>` in the support cells).

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_robot_obstacles_whitelist.py -v
"""

from __future__ import annotations

from types import SimpleNamespace

from core import config
from core.robot_obstacles import whitelist_frozen_contact

ALICE_TOOL = config.OBSTACLE_TOOL_NAMES["Alice"]
BELLE_TOOL = config.OBSTACLE_TOOL_NAMES["Belle"]


def _state(body_names, preload=None):
    """A duck-typed scene state: just rigid_body_states with touch_bodies."""
    preload = preload or {}
    return SimpleNamespace(
        rigid_body_states={
            name: SimpleNamespace(touch_bodies=list(preload.get(name, [])))
            for name in body_names
        }
    )


def test_assembly_cell_naming():
    """In Cindy's cell the held bar is `bar_<id>` and gets the obstacle tool."""
    st = _state(["bar_B21", "bar_B34"])
    touched = whitelist_frozen_contact(st, "Alice", ["B21"])
    assert touched == ["bar_B21"]
    assert st.rigid_body_states["bar_B21"].touch_bodies == [ALICE_TOOL]
    # Only the held bar is touched.
    assert st.rigid_body_states["bar_B34"].touch_bodies == []


def test_support_cell_naming():
    """In a support cell the held bar is `env_bar_<id>` -- probed second."""
    st = _state(["env_bar_B39"])
    touched = whitelist_frozen_contact(st, "Belle", ["B39"])
    assert touched == ["env_bar_B39"]
    assert st.rigid_body_states["env_bar_B39"].touch_bodies == [BELLE_TOOL]


def test_appends_without_clobbering_and_is_idempotent():
    """Existing whitelist entries survive; a second call adds nothing new."""
    st = _state(["bar_B21"], preload={"bar_B21": ["AT3L"]})
    whitelist_frozen_contact(st, "Alice", ["B21"])
    assert st.rigid_body_states["bar_B21"].touch_bodies == sorted({"AT3L", ALICE_TOOL})
    whitelist_frozen_contact(st, "Alice", ["B21"])
    assert st.rigid_body_states["bar_B21"].touch_bodies == sorted({"AT3L", ALICE_TOOL})


def test_missing_bar_prints_note_and_skips(capsys):
    """A bar absent under both naming schemes is skipped LOUDLY, not silently."""
    st = _state(["bar_B34"])
    touched = whitelist_frozen_contact(st, "Alice", ["B21"])
    assert touched == []
    out = capsys.readouterr().out
    assert "frozen Alice is clamped onto bar B21" in out
    assert "bar_B21" in out and "env_bar_B21" in out
