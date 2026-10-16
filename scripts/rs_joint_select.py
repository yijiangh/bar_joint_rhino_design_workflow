#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSJointSelect - Select a joint block by typing its id.

Type a joint id like ``J40-53_female`` (case-insensitive) and this selects that
placed joint block instance and zooms to it, so you can locate one joint in a
large model without hunting for it by eye. The role suffix is optional:

* ``J40-53_female`` / ``J40-53_mocap`` / ``J40-53_male`` -- one half of a
  bar-pair joint.
* ``J40-53``                           -- BOTH halves of that joint at once.
* ``40-53``                            -- bare pair numbers; the ``J`` is added.
* ``G4-T20-0`` or ``G4-T20-0_ground``  -- a ground joint block.
* ``joint_J25-26_male``                -- canonical PyBullet body key, pasted
  straight from a collision log; the ``joint_`` prefix is stripped.

Enter several ids separated by commas (``J40-53_female,J12-7``) to select more
than one at once. The prompt loops so you can jump from joint to joint -- each
entry REPLACES the previous selection; press Enter on an empty prompt (or Esc)
to finish.

Read-only: it never edits the document (it reads each block's stored name /
user text as is), so it is safe to run any time. No PyBullet needed.
"""

from __future__ import annotations

import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Every name below -- the "_female" suffix of an object name, the "joint_"
# prefix of a pasted PyBullet key, the layer a block's role is read from --
# comes from core.joint_name_conventions (pure Python, so this read-only command
# stays free of heavy imports).
from core import joint_name_conventions as jnc


# Command name used in every command-line message + dialog title.
CMD = "RSJointSelect"


def _normalize_token(token: str):
    """Turn one raw user token into a lookup key ``(joint_id_upper, role)``.

    Accepts ``J40-53_female`` / ``j40-53`` / bare pair numbers ``40-53`` (the
    ``J`` prefix is added) / ground ids like ``G4-T20-0`` / canonical
    PyBullet body keys like ``joint_J25-26_male`` (the ``joint_`` prefix is
    stripped). Returns ``None`` when the token is empty (so the caller can
    skip it).

    Args:
        token (str): one raw id typed by the user.

    Returns:
        tuple[str, str | None] | None: uppercased joint id + optional Subtype,
        or ``None`` for an empty token.
    """
    text = (token or "").strip()
    if not text:
        return None
    # A pasted canonical body key ("joint_J25-26_male") carries a "joint_"
    # prefix that is not part of the id stored on the block -- drop it.
    if text.lower().startswith(jnc.JOINT_KEY_PREFIX):
        text = text[len(jnc.JOINT_KEY_PREFIX):]
    jid, subtype = jnc.split_object_name(text)
    jid = jid.upper()
    # A bare "40-53" means the bar-pair joint J40-53; add the missing prefix.
    if jid and jid[0].isdigit() and "-" in jid:
        jid = jnc.PAIR_ID_PREFIX + jid
    return jid, subtype


def _scan_joints() -> dict:
    """Read-only scan of the document for every placed joint block instance.

    Primary match: the object-name convention ``<joint_id>_<role>`` written by
    the joint / ground placement code. Fallback for unnamed blocks: the
    ``joint_id`` user text plus the joint-instance layer the block sits on
    (which tells us the role). Everything is keyed uppercase so lookups are
    case-insensitive; the id as stored in the document is kept for reporting.

    Returns:
        dict: ``{joint_id_upper: {"id": stored_id, "roles": {subtype: [oid, ...]}}}``.
    """
    out = {}

    def _file(jid: str, subtype: str, oid):
        """Insert one block under its joint id + Subtype (helper for both paths)."""
        entry = out.setdefault(jid.upper(), {"id": jid, "roles": {}})
        entry["roles"].setdefault(subtype, []).append(oid)

    for oid in rs.AllObjects() or []:
        # * Path 1: parse the conventional object name "J40-53_female".
        name = rs.ObjectName(oid) or ""
        jid, subtype = jnc.split_object_name(name.strip())
        if subtype is not None and jid:
            _file(jid, subtype, oid)
            continue
        # * Path 2: unnamed / renamed block -- fall back to user text + layer.
        # Tool blocks also carry joint_id user text but sit on the tool layer,
        # which has no Subtype, so they stay out.
        jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
        if not jid:
            continue
        subtype = jnc.subtype_of_layer(rs.ObjectLayer(oid))
        if subtype is not None:
            _file(jid, subtype, oid)
    return out


def _select_joints(tokens, joints) -> int:
    """Select every joint block named by ``tokens``; report matches + misses.

    Replaces the current selection (unselect-all first), selects the matched
    blocks, and zooms to them.

    Args:
        tokens (list[str]): raw id tokens typed by the user (already comma-split).
        joints (dict): scan result from :func:`_scan_joints`.

    Returns:
        int: how many tokens matched at least one block.
    """
    to_select = []
    found_labels = []
    missing = []
    for token in tokens:
        key = _normalize_token(token)
        if key is None:
            continue
        jid_upper, subtype = key
        entry = joints.get(jid_upper)
        if entry is None:
            missing.append(token.strip())
            continue
        if subtype is None:
            # No suffix -> every placed half of this joint (female + male, or
            # the single ground block).
            oids = [oid for ids in entry["roles"].values() for oid in ids]
            label = entry["id"]
        else:
            oids = entry["roles"].get(subtype, [])
            label = jnc.object_name(entry["id"], subtype)
        if not oids:
            # The joint exists but not with the asked-for role (e.g. asked for
            # _ground on a bar-pair joint).
            missing.append(f"{token.strip()} (joint {entry['id']} has "
                           f"{'/'.join(sorted(jnc.role(r) for r in entry['roles']))} only)")
            continue
        found_labels.append(label)
        to_select.extend(oids)

    rs.UnselectAllObjects()
    if to_select:
        rs.SelectObjects(to_select)
        try:
            rs.ZoomSelected()
        except Exception:
            pass  # zoom is a convenience -- never let it break the selection
    if found_labels:
        print(f"{CMD}: selected {', '.join(found_labels)} "
              f"({len(to_select)} object(s)).")
    if missing:
        print(f"{CMD}: no joint block found for: {', '.join(missing)}.")
    return len(found_labels)


def main() -> None:
    """Prompt for joint id(s) and select the matching block(s), in a loop."""
    joints = _scan_joints()
    if not joints:
        rs.MessageBox("No placed joint blocks found in this document.", 0, CMD)
        return

    # Loop so the user can jump from joint to joint. Each entry replaces the
    # previous selection; an empty entry / Esc ends the command.
    while True:
        raw = rs.GetString(
            "Joint id to select (e.g. J40-53_female, or J40-53 for both halves; "
            "comma-separated for several; Enter to finish)"
        )
        if not raw or not raw.strip():
            break
        tokens = [tok for tok in raw.split(",") if tok.strip()]
        _select_joints(tokens, joints)
        # A joint may have been placed / removed between iterations; refresh.
        joints = _scan_joints()


if __name__ == "__main__":
    main()
