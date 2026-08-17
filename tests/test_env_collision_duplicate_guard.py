"""Headless tests for the duplicate-joint-name guard in ``core.env_collision``.

Canonical collision-body names are built from the ``joint_id`` + subtype USER
TEXT. Two blocks carrying the same id (a Rhino copy-paste clones user text)
compute the same key, and a plain dict write would silently drop one of them --
its geometry then exists in NO collision scene, so IK and the release checks
approve poses that pass straight through it. The collectors must refuse to
build a scene in that state.

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_env_collision_duplicate_guard.py -v
"""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from core import env_collision


@pytest.fixture
def fake_rs(monkeypatch):
    """A minimal rhinoscriptsyntax stub: object names only."""
    names = {"oid-A": "J40-43_female", "oid-B": "J40-43_female_copy"}
    stub = SimpleNamespace(ObjectName=lambda oid: names.get(str(oid), ""))
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", stub)
    return stub


def test_duplicate_key_from_two_blocks_raises(fake_rs):
    """Two different blocks claiming one name is a hard error, naming both."""
    out = {"env_joint_J40-43_female": {"source_oid": "oid-A"}}
    with pytest.raises(RuntimeError) as excinfo:
        env_collision._raise_on_duplicate_joint_key(
            out, "env_joint_J40-43_female", "oid-B", "collect_built_geometry"
        )
    message = str(excinfo.value)
    assert "J40-43_female" in message and "J40-43_female_copy" in message
    # The message must point at the repair path, not just complain.
    assert "RSReorderBarID" in message


def test_same_block_seen_twice_is_tolerated(fake_rs):
    """Re-adding the SAME object (layer listed twice) is harmless, not an error."""
    out = {"env_joint_J40-43_female": {"source_oid": "oid-A"}}
    env_collision._raise_on_duplicate_joint_key(
        out, "env_joint_J40-43_female", "oid-A", "collect_built_geometry"
    )


def test_fresh_key_passes(fake_rs):
    """A name nobody has claimed yet is accepted silently."""
    env_collision._raise_on_duplicate_joint_key(
        {}, "env_joint_J26-41_female", "oid-B", "collect_assembly_geometry"
    )
