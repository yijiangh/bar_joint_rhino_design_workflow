"""Headless tests for robotic-tool document replacement behavior."""

from __future__ import annotations

import contextlib
import sys
from types import SimpleNamespace

import pytest

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
    # `replace_all_tool_instances` also pins the import onto the tool layer via
    # a `layer_name` keyword; swallow it so the stub tracks the real signature.
    monkeypatch.setattr(
        rhino_tool_place,
        "refresh_block_definition",
        lambda block_name, asset_path, **_kwargs: refreshed.append(
            (block_name, asset_path)
        ),
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


# ---------------------------------------------------------------------------
# enforce_bar_tool_sides
# ---------------------------------------------------------------------------


def _side_fixture(monkeypatch, tool_by_joint):
    """Wire up a one-bar document with two tool-bearing joints.

    ``tool_by_joint`` maps ``joint_id -> tool_name`` for the tools currently
    placed.  ``G1-0`` sits at 100 mm along the bar, ``G1-1`` at 400 mm, so the
    deterministic rule must hand ``G1-0`` the document default's side.
    Returns the list that records ``(joint_id, tool_name)`` re-placements.
    """
    from core import config

    user_text = {
        "j-near": {"joint_id": "G1-0", "parent_bar_id": "B1", "position_mm": "100.0"},
        "j-far": {"joint_id": "G1-1", "parent_bar_id": "B1", "position_mm": "400.0"},
    }
    tool_oid_by_joint = {joint: f"t-{joint}" for joint in tool_by_joint}

    # The male layer is empty; both joints are ground joints on one bar. Listing
    # the far joint FIRST is the point: the result must not depend on this order.
    by_layer = {
        config.LAYER_JOINT_MALE_INSTANCES: [],
        config.LAYER_JOINT_GROUND_INSTANCES: ["j-far", "j-near"],
    }

    fake_rs = SimpleNamespace(
        IsLayer=lambda name: name in by_layer,
        ObjectsByLayer=lambda name: by_layer.get(name, []),
        GetUserText=lambda oid, key: (
            user_text.get(oid, {}).get(key)
            if oid in user_text
            else {"tool_name": tool_by_joint.get(oid[2:], "")}.get(key)
        ),
    )
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake_rs)

    left = SimpleNamespace(name="AT4L")
    right = SimpleNamespace(name="AT4R")
    monkeypatch.setattr(
        rhino_tool_place, "_get_active_pair_or_none", lambda: {"left": left, "right": right}
    )
    monkeypatch.setattr(rhino_tool_place, "_resolve_default_active_tool", lambda _a: left)
    monkeypatch.setattr(
        rhino_tool_place,
        "find_tool_for_joint",
        lambda joint_id: tool_oid_by_joint.get(joint_id),
    )

    placed = []

    def _fake_place(block_id, joint_id, tool):
        # Must return an object id: the caller counts a placement only when the
        # real routine hands back a non-None oid.
        placed.append((joint_id, tool.name))
        return f"new-{joint_id}"

    monkeypatch.setattr(rhino_tool_place, "place_tool_at_block_instance", _fake_place)
    return placed


def test_enforce_bar_tool_sides_fixes_two_same_side_tools(monkeypatch):
    """Both joints holding the left tool: only the FAR one flips to right."""
    placed = _side_fixture(monkeypatch, {"G1-0": "AT4L", "G1-1": "AT4L"})

    assert rhino_tool_place.enforce_bar_tool_sides() == 1
    assert placed == [("G1-1", "AT4R")]


def test_enforce_bar_tool_sides_fixes_two_right_tools(monkeypatch):
    """Mirror case: two right tools -> the far one becomes left."""
    placed = _side_fixture(monkeypatch, {"G1-0": "AT4R", "G1-1": "AT4R"})

    assert rhino_tool_place.enforce_bar_tool_sides() == 1
    assert placed == [("G1-1", "AT4L")]


def test_enforce_bar_tool_sides_keeps_a_valid_pair_whichever_end_holds_which(
    monkeypatch,
):
    """One L + one R is correct regardless of which end holds which.

    Which arm reaches which joint comes from the robot's approach, not from the
    bar's geometry, so re-deriving it from position swaps correct pairs into
    wrong ones. Here the FAR joint holds the left tool and must stay that way.
    """
    placed = _side_fixture(monkeypatch, {"G1-0": "AT4R", "G1-1": "AT4L"})

    assert rhino_tool_place.enforce_bar_tool_sides() == 0
    assert placed == []


def test_enforce_bar_tool_sides_leaves_a_correct_bar_alone(monkeypatch):
    """Idempotence: nothing is re-placed when the sides already agree."""
    placed = _side_fixture(monkeypatch, {"G1-0": "AT4L", "G1-1": "AT4R"})

    assert rhino_tool_place.enforce_bar_tool_sides() == 0
    assert placed == []


# ---------------------------------------------------------------------------
# is_tool_on_joint / find_detached_tools
# ---------------------------------------------------------------------------


def _identity():
    import numpy as np

    return np.eye(4)


def _translation(dx):
    import numpy as np

    xform = np.eye(4)
    xform[0, 3] = dx
    return xform


def test_is_tool_on_joint_matches_the_placement_invariant(monkeypatch):
    """`place_tool_at_block_instance` poses tool_world = attach @ inv(M_tcp).

    So the tool is on the joint exactly when tool_world @ M_tcp == attach.  With
    an identity offset (every male/female joint) the attach frame IS the block
    frame, so this is the untouched-behaviour guard.
    """
    import numpy as np

    m_tcp = _translation(7.0)
    block_world = _translation(100.0)
    tool_world = block_world @ np.linalg.inv(m_tcp)

    frames = {"tool": tool_world, "block": block_world}
    monkeypatch.setattr(
        rhino_tool_place, "_block_instance_world_xform", lambda oid: frames[oid]
    )
    monkeypatch.setattr(rhino_tool_place, "tool_attach_offset", lambda _oid: _identity())
    tool = SimpleNamespace(M_tcp_from_block=m_tcp)

    assert rhino_tool_place.is_tool_on_joint("tool", "block", tool) is True

    # Nudge the tool: it is no longer on its joint.
    frames["tool"] = tool_world @ _translation(5.0)
    assert rhino_tool_place.is_tool_on_joint("tool", "block", tool) is False


def test_find_detached_tools_reports_a_tool_that_drifted_off_its_joint(monkeypatch):
    """The flying-tool case: metadata resolves fine, geometry has come adrift."""
    import numpy as np
    from core import config

    m_tcp = _identity()
    block_world = _translation(100.0)

    # `tool_attach_offset` also asks the JOINT block (b-J1-2 / b-J3-4) for its
    # `ground_joint_name`, so every lookup must tolerate an unknown oid/key and
    # answer "" -- which is exactly the male-joint path: no name -> identity.
    fake_rs = SimpleNamespace(
        IsLayer=lambda name: name == config.LAYER_TOOL_INSTANCES,
        ObjectsByLayer=lambda _name: ["t-attached", "t-flying"],
        IsObject=lambda _oid: True,
        GetUserText=lambda oid, key: {
            "joint_id": {"t-attached": "J1-2", "t-flying": "J3-4"}.get(oid, ""),
            "tool_name": "AT4L",
            "ground_joint_name": "",
        }.get(key, ""),
    )
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake_rs)
    monkeypatch.setattr(
        rhino_tool_place._robotic_tool,
        "load_robotic_tools",
        lambda: {"AT4L": SimpleNamespace(M_tcp_from_block=m_tcp)},
    )
    monkeypatch.setattr(
        rhino_tool_place,
        "find_attached_block_for_joint",
        lambda joint_id: f"b-{joint_id}",
    )
    frames = {
        "t-attached": block_world,      # sits exactly on its joint
        "b-J1-2": block_world,
        "t-flying": _translation(9999.0),  # adrift
        "b-J3-4": block_world,
    }
    monkeypatch.setattr(
        rhino_tool_place, "_block_instance_world_xform", lambda oid: frames[oid]
    )

    detached = rhino_tool_place.find_detached_tools()

    assert [(oid, joint_id) for oid, joint_id, _reason in detached] == [
        ("t-flying", "J3-4")
    ]


def test_find_detached_tools_reports_a_tool_whose_joint_is_gone(monkeypatch):
    """A tool tagged to a joint that no longer exists is detached too."""
    from core import config

    fake_rs = SimpleNamespace(
        IsLayer=lambda name: name == config.LAYER_TOOL_INSTANCES,
        ObjectsByLayer=lambda _name: ["t-orphan"],
        IsObject=lambda _oid: True,
        GetUserText=lambda _oid, key: {"joint_id": "J9-9", "tool_name": "AT4L"}[key],
    )
    monkeypatch.setitem(sys.modules, "rhinoscriptsyntax", fake_rs)
    monkeypatch.setattr(
        rhino_tool_place._robotic_tool,
        "load_robotic_tools",
        lambda: {"AT4L": SimpleNamespace(M_tcp_from_block=_identity())},
    )
    monkeypatch.setattr(
        rhino_tool_place, "find_attached_block_for_joint", lambda _joint_id: None
    )

    detached = rhino_tool_place.find_detached_tools()

    assert len(detached) == 1
    assert detached[0][1] == "J9-9"
    assert "joint block is gone" in detached[0][2]


# ---------------------------------------------------------------------------
# tool_attach_frame / tool_attach_offset  (ground tool-attach offset)
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _clear_attach_cache():
    """The offset cache is module-level state shared across tests."""
    rhino_tool_place.clear_tool_attach_cache()
    yield
    rhino_tool_place.clear_tool_attach_cache()


def _rz(angle_rad):
    """4x4 rotation about local +Z (mirrors core.transforms.rotation_about_local_z)."""
    import numpy as np

    xform = np.eye(4)
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
    xform[0, 0], xform[0, 1] = cos_a, -sin_a
    xform[1, 0], xform[1, 1] = sin_a, cos_a
    return xform


def _write_registry(tmp_path, ground_entry):
    """Write a minimal joint registry holding one ground joint; return its path."""
    import json

    path = tmp_path / "joint_pairs.json"
    entry = {
        "name": "T20Ground",
        "block_name": "T20Ground",
        "M_block_from_bar": _identity().tolist(),
    }
    entry.update(ground_entry)
    path.write_text(
        json.dumps({"halves": [], "mates": [], "ground_joints": [entry]}),
        encoding="utf-8",
    )
    return str(path)


def test_ground_tool_attach_offset_reads_the_stored_rotation(tmp_path):
    """The registry's M_tool_from_block comes back verbatim."""
    import numpy as np

    path = _write_registry(tmp_path, {"M_tool_from_block": _rz(np.pi).tolist()})

    offset = rhino_tool_place.ground_tool_attach_offset("T20Ground", path)

    assert np.allclose(offset, _rz(np.pi), atol=1e-12)
    # Pure rotation: the TCP stays on the block origin, so joint_relink's
    # 50 mm TCP probe is unaffected.
    assert np.allclose(offset[:3, 3], 0.0)


def test_ground_tool_attach_offset_defaults_to_identity_for_a_legacy_entry(tmp_path):
    """A ground joint saved before the attach frame existed keeps its behaviour."""
    import numpy as np

    path = _write_registry(tmp_path, {})

    assert np.allclose(
        rhino_tool_place.ground_tool_attach_offset("T20Ground", path), _identity()
    )


def test_ground_tool_attach_offset_is_identity_for_unknown_and_empty_names(tmp_path):
    import numpy as np

    path = _write_registry(tmp_path, {"M_tool_from_block": _rz(np.pi).tolist()})

    assert np.allclose(
        rhino_tool_place.ground_tool_attach_offset("nope", path), _identity()
    )
    assert np.allclose(rhino_tool_place.ground_tool_attach_offset("", path), _identity())


def test_tool_attach_offset_is_identity_for_a_male_block(monkeypatch):
    """Male/female blocks carry no `ground_joint_name` -> untouched behaviour."""
    import numpy as np

    monkeypatch.setitem(
        sys.modules,
        "rhinoscriptsyntax",
        SimpleNamespace(GetUserText=lambda _oid, _key: None),
    )

    assert np.allclose(rhino_tool_place.tool_attach_offset("b-J1-2"), _identity())


def test_clear_tool_attach_cache_picks_up_a_registry_edit(tmp_path):
    """A re-saved registry is re-read (the mtime stamp is not relied on here)."""
    import json

    import numpy as np

    path = _write_registry(tmp_path, {"M_tool_from_block": _rz(np.pi).tolist()})
    assert np.allclose(
        rhino_tool_place.ground_tool_attach_offset("T20Ground", path), _rz(np.pi)
    )

    data = json.loads(open(path, encoding="utf-8").read())
    data["ground_joints"][0]["M_tool_from_block"] = _identity().tolist()
    open(path, "w", encoding="utf-8").write(json.dumps(data))
    rhino_tool_place.clear_tool_attach_cache()

    assert np.allclose(
        rhino_tool_place.ground_tool_attach_offset("T20Ground", path), _identity()
    )


def test_place_and_check_agree_under_a_ground_tool_offset(monkeypatch):
    """The regression that keeps RSUpdatePreview from snapping a rolled tool back.

    `place_tool_at_block_instance` and `is_tool_on_joint` must read the attach
    frame from the SAME function.  If they ever diverge, `resync_tools_to_joints`
    calls a correctly-placed ground tool "drifted" and re-places it on every run.
    """
    import numpy as np

    m_tcp = _translation(7.0)
    block_world = _translation(100.0)
    offset = _rz(np.pi)

    monkeypatch.setattr(rhino_tool_place, "tool_attach_offset", lambda _oid: offset)
    frames = {"block": block_world}
    monkeypatch.setattr(
        rhino_tool_place, "_block_instance_world_xform", lambda oid: frames[oid]
    )
    tool = SimpleNamespace(M_tcp_from_block=m_tcp)

    # Posed the way place_tool_at_block_instance now poses it.
    attach = rhino_tool_place.tool_attach_frame("block")
    assert np.allclose(attach, block_world @ offset)
    frames["tool"] = attach @ np.linalg.inv(m_tcp)
    assert rhino_tool_place.is_tool_on_joint("tool", "block", tool) is True

    # The OLD, un-offset placement is now (correctly) detected as drift.
    frames["tool"] = block_world @ np.linalg.inv(m_tcp)
    assert rhino_tool_place.is_tool_on_joint("tool", "block", tool) is False


def test_shipped_ground_offsets_are_pure_rotations():
    """Standing guard for `joint_relink._TOOL_TCP_TOL_MM`.

    The tool->joint match probes `(tool_frame @ M_tcp_from_block)[:3, 3]` against
    joint block origins with a 50 mm tolerance.  A rotation leaves that point
    untouched; a translating offset would silently break relink.
    """
    import numpy as np
    from core import joint_pair

    registry = joint_pair.load_joint_registry()
    for name, ground in registry.ground_joints.items():
        assert np.allclose(ground.M_tool_from_block[:3, 3], 0.0), name
