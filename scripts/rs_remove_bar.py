#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSRemoveBar - Remove selected bars and clean up all associated data.

Select one or more bar tube previews (the visible cylinders).  For each
selected bar the following objects are deleted:

* All joint block instances (female + male) where the bar is a participant.
* All tool instances attached to those joints.
* The tube preview cylinder.
* The bar centerline curve.

After deletion ``repair_bar_sequences()`` is called, which:
* Compacts the remaining assembly sequence numbers to fill gaps.
* Strips any stale ``supported_until`` references pointing at the removed
  bar IDs from every remaining bar.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_bar_registry import (
    get_all_bars,
    repair_bar_sequences,
    repair_on_entry,
)
from core.rhino_tool_place import remove_tool_for_joint

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _find_joint_ids_for_bar(bar_id):
    """Return a list of joint_ids that involve *bar_id* as female or male."""
    found = set()
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            fb = rs.GetUserText(oid, "female_parent_bar")
            mb = rs.GetUserText(oid, "male_parent_bar")
            if fb == bar_id or mb == bar_id:
                jid = rs.GetUserText(oid, "joint_id")
                if jid:
                    found.add(jid)
    return list(found)


def _remove_joint_blocks(joint_id):
    """Delete placed female and male block instances for *joint_id*."""
    to_delete = []
    for suffix in ("_female", "_male"):
        ids = rs.ObjectsByName(f"{joint_id}{suffix}")
        if ids:
            to_delete.extend(ids)
    if to_delete:
        rs.DeleteObjects(to_delete)


def _remove_tube_preview(bar_id):
    """Delete tube preview cylinder(s) tagged with *bar_id*."""
    if not rs.IsLayer(config.LAYER_BAR_TUBE_PREVIEWS):
        return
    to_delete = [
        oid
        for oid in rs.ObjectsByLayer(config.LAYER_BAR_TUBE_PREVIEWS) or []
        if rs.GetUserText(oid, "tube_bar_id") == bar_id
    ]
    if to_delete:
        rs.DeleteObjects(to_delete)


def _remove_bar(bar_id, all_bars):
    """Remove one bar and all its associated data.

    Parameters
    ----------
    bar_id : str
        e.g. ``"B1"``
    all_bars : dict
        ``{bar_id: guid}`` snapshot taken **before** any deletion begins so
        that GUIDs for bars-to-remove are still valid.

    Returns
    -------
    list[str]
        Joint IDs that were removed.
    """
    # 1. Joints (tools first, then blocks)
    joint_ids = _find_joint_ids_for_bar(bar_id)
    for jid in joint_ids:
        remove_tool_for_joint(jid)
        _remove_joint_blocks(jid)

    # 2. Tube preview
    _remove_tube_preview(bar_id)

    # 3. Bar centerline curve
    bar_guid = all_bars.get(bar_id)
    if bar_guid and rs.IsObject(bar_guid):
        rs.DeleteObject(bar_guid)

    return joint_ids


# ---------------------------------------------------------------------------
# Custom pick filter
# ---------------------------------------------------------------------------


def _tube_preview_filter(rh_object, geometry, component_index):
    """Accept only tube preview objects (have ``tube_bar_id`` UserText)."""
    return bool(rs.GetUserText(rh_object.Id, "tube_bar_id"))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), "RSRemoveBar")

    tube_ids = rs.GetObjects(
        "Select bar(s) to remove",
        rs.filter.surface | rs.filter.polysurface,
        custom_filter=_tube_preview_filter,
    )
    if not tube_ids:
        return

    # Collect unique bar IDs from the selected tube previews
    bar_ids_to_remove = []
    for tid in tube_ids:
        bid = rs.GetUserText(tid, "tube_bar_id")
        if bid and bid not in bar_ids_to_remove:
            bar_ids_to_remove.append(bid)

    if not bar_ids_to_remove:
        print("RSRemoveBar: No registered bars found in selection.")
        return

    # Snapshot all bars before any deletion (GUIDs are still valid here)
    all_bars = get_all_bars()

    total_joints = 0
    for bar_id in bar_ids_to_remove:
        joint_ids = _remove_bar(bar_id, all_bars)
        total_joints += len(joint_ids)
        print(f"RSRemoveBar: Removed {bar_id} ({len(joint_ids)} joint(s))")

    # Compact sequences and strip stale supported_until refs
    changed = repair_bar_sequences()
    if changed:
        summary = ", ".join(f"{bid}→seq{seq}" for bid, seq in sorted(changed.items()))
        print(f"RSRemoveBar: Sequence repaired ({len(changed)} bar(s)): {summary}")

    print(
        f"RSRemoveBar: Done. {len(bar_ids_to_remove)} bar(s) removed,"
        f" {total_joints} joint(s) removed."
    )


if __name__ == "__main__":
    main()
