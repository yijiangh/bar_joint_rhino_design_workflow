"""``env_collision.clear_joint_obj_path_cache`` really forgets a joint's OBJ.

The ``block_name -> OBJ`` map and the per-block ``RigidBody`` cache both live
in ``sc.sticky`` for the whole Rhino session.  A block looked up before its OBJ
was registered (RSDefineJointHalf, same session) is cached as ``None`` --
"missing, skip this joint" -- so it stays out of collision until Rhino restarts
unless BOTH caches are dropped.  Headless: ``_sticky_dict`` falls back to a
module dict outside Rhino.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from core import env_collision


@pytest.fixture
def sticky(monkeypatch):
    store = {}
    monkeypatch.setattr(env_collision, "_sticky_dict", lambda: store)
    return store


def _registry(halves):
    """Monkeypatchable stand-in for ``load_joint_registry``."""
    return SimpleNamespace(
        halves={
            name: SimpleNamespace(
                block_name=name,
                collision_filename=f"{name}.obj",
                collision_path=lambda asset_dir, n=name: f"{asset_dir}/{n}.obj",
            )
            for name in halves
        },
        ground_joints={},
    )


def test_new_half_is_found_after_clearing(sticky, monkeypatch):
    from core import joint_pair

    monkeypatch.setattr(joint_pair, "load_joint_registry", lambda: _registry(["T20_Female"]))
    assert "T20_MoCap" not in env_collision._joint_obj_path_map()

    # RSDefineJointHalf registers T20_MoCap in the same session.
    monkeypatch.setattr(
        joint_pair, "load_joint_registry", lambda: _registry(["T20_Female", "T20_MoCap"])
    )
    assert "T20_MoCap" not in env_collision._joint_obj_path_map()  # the stale map

    env_collision.clear_joint_obj_path_cache()
    assert "T20_MoCap" in env_collision._joint_obj_path_map()


def test_clear_drops_a_cached_missing_rigid_body(sticky):
    """The ``None`` a too-early lookup left behind must not survive the clear."""
    env_collision._joint_rb_cache()["T20_MoCap"] = None
    env_collision.clear_joint_obj_path_cache()
    assert "T20_MoCap" not in env_collision._joint_rb_cache()


def test_clear_on_an_empty_session_is_harmless(sticky):
    env_collision.clear_joint_obj_path_cache()
    assert sticky == {}
