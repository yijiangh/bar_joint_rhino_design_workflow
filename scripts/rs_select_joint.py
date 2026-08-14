#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSSelectJoint - Select a joint block by typing its id.

Type a joint id like ``J40-53_female`` (case-insensitive) and this selects that
placed joint block instance and zooms to it, so you can locate one joint in a
large model without hunting for it by eye. The role suffix is optional:

* ``J40-53_female`` / ``J40-53_male``  -- one specific half of a bar-pair joint.
* ``J40-53``                           -- BOTH halves of that joint at once.
* ``40-53``                            -- bare pair numbers; the ``J`` is added.
* ``G4-floor-0`` or ``G4-floor-0_ground`` -- a ground joint block.
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

from core import config


# Command name used in every command-line message + dialog title.
CMD = "RSSelectJoint"

# The three role suffixes the placement code appends to a joint id when it
# names a block instance (e.g. "J40-53_female"). Order matters only for the
# help text; matching is case-insensitive.
ROLES = ("female", "male", "ground")

# Canonical PyBullet body keys look like "joint_J25-26_male" (the "joint_"
# prefix is CANONICAL_JOINT_PREFIX in core/bar_action.py + core/env_collision.py;
# kept as a plain string here so this read-only command stays free of heavy
# imports). Users paste those keys straight from collision logs, so the prefix
# is accepted and stripped when parsing a token.
CANONICAL_JOINT_PREFIX = "joint_"

# Fallback for blocks that lost their object name: map each joint-instance
# layer to the role it holds, so the block can still be filed correctly from
# its "joint_id" user text alone. Tool blocks also carry "joint_id" user text
# but live on the tool layer, so this mapping naturally keeps them out.
LAYER_TO_ROLE = {
    config.LAYER_JOINT_FEMALE_INSTANCES: "female",
    config.LAYER_JOINT_MALE_INSTANCES: "male",
    config.LAYER_JOINT_GROUND_INSTANCES: "ground",
}


def _split_role(text: str):
    """Split an id like ``J40-53_female`` into the id part and the role part.

    Args:
        text (str): a raw name or user token, already stripped of whitespace.

    Returns:
        tuple[str, str | None]: ``(joint_id, role)`` where ``role`` is one of
        :data:`ROLES`, or ``(text, None)`` when there is no role suffix.
    """
    base, _, tail = text.rpartition("_")
    if base and tail.lower() in ROLES:
        return base, tail.lower()
    return text, None


def _normalize_token(token: str):
    """Turn one raw user token into a lookup key ``(joint_id_upper, role)``.

    Accepts ``J40-53_female`` / ``j40-53`` / bare pair numbers ``40-53`` (the
    ``J`` prefix is added) / ground ids like ``G4-floor-0`` / canonical
    PyBullet body keys like ``joint_J25-26_male`` (the ``joint_`` prefix is
    stripped). Returns ``None`` when the token is empty (so the caller can
    skip it).

    Args:
        token (str): one raw id typed by the user.

    Returns:
        tuple[str, str | None] | None: uppercased joint id + optional role, or
        ``None`` for an empty token.
    """
    text = (token or "").strip()
    if not text:
        return None
    # A pasted canonical body key ("joint_J25-26_male") carries a "joint_"
    # prefix that is not part of the id stored on the block -- drop it.
    if text.lower().startswith(CANONICAL_JOINT_PREFIX):
        text = text[len(CANONICAL_JOINT_PREFIX):]
    jid, role = _split_role(text)
    jid = jid.upper()
    # A bare "40-53" means the bar-pair joint J40-53; add the missing prefix.
    if jid and jid[0].isdigit() and "-" in jid:
        jid = "J" + jid
    return jid, role


def _scan_joints() -> dict:
    """Read-only scan of the document for every placed joint block instance.

    Primary match: the object-name convention ``{joint_id}_{role}`` written by
    the joint / ground placement code. Fallback for unnamed blocks: the
    ``joint_id`` user text plus the joint-instance layer the block sits on
    (which tells us the role). Everything is keyed uppercase so lookups are
    case-insensitive; the id as stored in the document is kept for reporting.

    Returns:
        dict: ``{joint_id_upper: {"id": stored_id, "roles": {role: [oid, ...]}}}``.
    """
    out = {}

    def _file(jid: str, role: str, oid):
        """Insert one block under its joint id + role (helper for both paths)."""
        entry = out.setdefault(jid.upper(), {"id": jid, "roles": {}})
        entry["roles"].setdefault(role, []).append(oid)

    for oid in rs.AllObjects() or []:
        # * Path 1: parse the conventional object name "J40-53_female".
        name = rs.ObjectName(oid) or ""
        jid, role = _split_role(name.strip())
        if role is not None and jid:
            _file(jid, role, oid)
            continue
        # * Path 2: unnamed / renamed block -- fall back to user text + layer.
        jid = rs.GetUserText(oid, "joint_id")
        if not jid:
            continue
        role = LAYER_TO_ROLE.get(rs.ObjectLayer(oid))
        if role is not None:
            _file(jid, role, oid)
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
        jid_upper, role = key
        entry = joints.get(jid_upper)
        if entry is None:
            missing.append(token.strip())
            continue
        if role is None:
            # No suffix -> every placed half of this joint (female + male, or
            # the single ground block).
            oids = [oid for ids in entry["roles"].values() for oid in ids]
            label = entry["id"]
        else:
            oids = entry["roles"].get(role, [])
            label = f"{entry['id']}_{role}"
        if not oids:
            # The joint exists but not with the asked-for role (e.g. asked for
            # _ground on a bar-pair joint).
            missing.append(f"{token.strip()} (joint {entry['id']} has "
                           f"{'/'.join(sorted(entry['roles']))} only)")
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
