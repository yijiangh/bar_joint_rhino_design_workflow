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
from core.rhino_helpers import objects_on_layers, reset_object_color, set_object_color


ENV_GREEN = (60, 179, 60)


def highlight_env_for_ik(active_bar_id):
    """Paint built bars + joints green; record originals for revert.

    Returns a token (list of oids touched) for ``revert_env_highlight``.
    """
    bar_map = get_bar_seq_map()
    if active_bar_id not in bar_map:
        return []
    _, active_seq = bar_map[active_bar_id]

    built_bar_ids = {bid for bid, (_, seq) in bar_map.items() if seq < active_seq}
    if not built_bar_ids:
        return []

    touched = []
    rs.EnableRedraw(False)
    try:
        for bid in built_bar_ids:
            oid = bar_map[bid][0]
            for obj in _bar_curve_and_tube(oid):
                if rs.IsObject(obj):
                    set_object_color(obj, ENV_GREEN)
                    touched.append(obj)

        for joint_oid in objects_on_layers(*jnc.JOINT_LAYERS):
            if rs.GetUserText(joint_oid, jnc.UT_PARENT_BAR) in built_bar_ids:
                if rs.IsObject(joint_oid):
                    set_object_color(joint_oid, ENV_GREEN)
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
        reset_object_color(token)
    finally:
        rs.EnableRedraw(True)
