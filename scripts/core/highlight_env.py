"""Focused env-collision overlay for IK debugging.

Mutates only built bars + their joints (seq < active); leaves active bar,
unbuilt bars, and tools untouched so the holistic sequencing colors remain
authoritative outside this preview. Returns a token that
``revert_env_highlight`` consumes to restore by-layer color.

NOT a replacement for ``rhino_bar_registry.show_sequence_colors``.
"""

from __future__ import annotations

import rhinoscriptsyntax as rs

from core import joint_name_conventions as jnc
from core.rhino_bar_registry import (
    _bar_curve_and_tube,
    get_bar_seq_map,
)


ENV_GREEN = (60, 179, 60)


def _set_obj_color(oid, color):
    if not rs.IsObject(oid):
        return False
    rs.ObjectColorSource(oid, 1)
    rs.ObjectColor(oid, color)
    return True


def _reset_obj_color(oid):
    if not rs.IsObject(oid):
        return
    rs.ObjectColorSource(oid, 0)


def highlight_env_for_ik(active_bar_id, scene=None):
    """Paint built bars + joints green; record originals for revert.

    Args:
        active_bar_id (str): The bar being solved; never painted.
        scene: The core scene the solve plans in. Given, the bars painted are exactly the ones in it (what the
            planner collides with); None: every bar earlier in the sequence, as before.

    Returns a token (list of oids touched) for ``revert_env_highlight``.
    """
    bar_map = get_bar_seq_map()
    if active_bar_id not in bar_map:
        return []
    _, active_seq = bar_map[active_bar_id]

    if scene is not None:
        in_scene = {body_id.split("/", 1)[1] for body_id, body in scene.bodies.items()
                    if body_id.startswith("bars/") and body.enabled}
        built_bar_ids = {bid for bid in bar_map if bid in in_scene and bid != active_bar_id}
    else:
        built_bar_ids = {bid for bid, (_, seq) in bar_map.items() if seq < active_seq}
    if not built_bar_ids:
        return []

    touched = []
    rs.EnableRedraw(False)
    try:
        for bid in built_bar_ids:
            oid = bar_map[bid][0]
            for obj in _bar_curve_and_tube(oid):
                if _set_obj_color(obj, ENV_GREEN):
                    touched.append(obj)

        for layer in jnc.JOINT_LAYERS:
            if not rs.IsLayer(layer):
                continue
            for joint_oid in rs.ObjectsByLayer(layer) or []:
                if rs.GetUserText(joint_oid, jnc.UT_PARENT_BAR) in built_bar_ids:
                    if _set_obj_color(joint_oid, ENV_GREEN):
                        touched.append(joint_oid)
    finally:
        rs.EnableRedraw(True)
    return touched


def revert_env_highlight(token):
    """Restore by-layer color on every oid the matching highlight call touched."""
    if not token:
        return
    rs.EnableRedraw(False)
    try:
        for oid in token:
            _reset_obj_color(oid)
    finally:
        rs.EnableRedraw(True)
