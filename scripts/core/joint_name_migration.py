"""Rename joint names in documents saved before the ``<Type>_<Subtype>`` rule.

Three names changed when every joint block became ``<Type>_<Subtype>``
(``core.joint_name_conventions``):

    block definition   T20Ground          -> T20_Ground
    ground joint id    G4-T20Ground-0     -> G4-T20-0     (and its tool TG4-...)
    mate names         T20Deck12_Pair     -> T20Deck12
                       T20SFloorLeft      -> T20SubLeft
                       T20SFloorRight     -> T20SubRight

A ``.3dm`` saved earlier still carries the old strings -- as the block
definition name, in each block's user text and object name, on each tool, and
in the document's "last used pair" settings.  :func:`migrate_legacy_joint_names`
rewrites them in place.  ``repair_on_entry`` runs it at the start of every
command, so an old file is converted the first time any command touches it.

Idempotent: on a converted document it reads a few user-text values and
changes nothing.  Removable once every lab file has been re-saved.
"""

from __future__ import annotations

from core import joint_name_conventions as jnc

#: User text the old ground placement wrote and nothing reads any more: the
#: ground definition is now found from the block name.
_LEGACY_GROUND_NAME_KEY = "ground_joint_name"
#: The value ground blocks used to carry as ``joint_type``.
_LEGACY_GROUND_TYPE = "ground"


def _rename_block_definitions(rs, caller):
    n = 0
    for old, new in jnc.LEGACY_BLOCK_RENAMES.items():
        if not rs.IsBlock(old):
            continue
        if rs.IsBlock(new):
            print(
                f"{caller}: NOTE - both block definitions '{old}' and '{new}' exist; "
                f"instances of '{old}' were left as they are.  Replace them by hand."
            )
            continue
        if rs.RenameBlock(old, new):
            n += 1
    return n


def _migrate_ground_blocks(rs):
    n = 0
    if not rs.IsLayer(jnc.LAYER_GROUND):
        return n
    for oid in rs.ObjectsByLayer(jnc.LAYER_GROUND) or []:
        changed = False
        old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID) or ""
        new_jid = jnc.migrate_joint_id(old_jid)
        if new_jid != old_jid:
            rs.SetUserText(oid, jnc.UT_JOINT_ID, new_jid)
            rs.ObjectName(oid, jnc.object_name(new_jid, jnc.GROUND))
            changed = True
        old_block = rs.GetUserText(oid, jnc.UT_BLOCK_NAME) or ""
        if old_block in jnc.LEGACY_BLOCK_RENAMES:
            rs.SetUserText(oid, jnc.UT_BLOCK_NAME, jnc.LEGACY_BLOCK_RENAMES[old_block])
            changed = True
        if (rs.GetUserText(oid, jnc.UT_JOINT_TYPE) or "") == _LEGACY_GROUND_TYPE:
            block_name = rs.BlockInstanceName(oid) or ""
            if jnc.is_block_name(block_name):
                type_, subtype = jnc.split_block_name(block_name)
                rs.SetUserText(oid, jnc.UT_JOINT_TYPE, type_)
                rs.SetUserText(oid, jnc.UT_JOINT_SUBTYPE, subtype)
                changed = True
        if rs.GetUserText(oid, _LEGACY_GROUND_NAME_KEY) is not None:
            rs.SetUserText(oid, _LEGACY_GROUND_NAME_KEY)  # no value -> delete key
            changed = True
        n += int(changed)
    return n


def _migrate_tools(rs, tool_layer):
    n = 0
    if not rs.IsLayer(tool_layer):
        return n
    for oid in rs.ObjectsByLayer(tool_layer) or []:
        old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID) or ""
        new_jid = jnc.migrate_joint_id(old_jid)
        if new_jid == old_jid:
            continue
        rs.SetUserText(oid, jnc.UT_JOINT_ID, new_jid)
        rs.SetUserText(oid, jnc.UT_TOOL_ID, jnc.tool_id(new_jid))
        rs.ObjectName(oid, jnc.tool_id(new_jid))
        n += 1
    return n


def _migrate_pair_names(rs):
    n = 0
    for layer in jnc.PAIRED_LAYERS:
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            old = rs.GetUserText(oid, jnc.UT_PAIR_NAME) or ""
            if old in jnc.LEGACY_MATE_RENAMES:
                rs.SetUserText(oid, jnc.UT_PAIR_NAME, jnc.LEGACY_MATE_RENAMES[old])
                n += 1
    return n


def _migrate_doc_pair_settings(get_doc_string, set_doc_string, keys):
    n = 0
    for key in keys:
        old = get_doc_string(key)
        if old in jnc.LEGACY_MATE_RENAMES:
            set_doc_string(key, jnc.LEGACY_MATE_RENAMES[old])
            n += 1
    return n


def migrate_legacy_joint_names(caller: str = "RSScaffolding") -> int:
    """Rewrite every legacy joint name in the active document.  Returns the count.

    Prints one line when anything changed, nothing otherwise.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    from core import config  # noqa: PLC0415
    from core import rhino_bar_pick  # noqa: PLC0415
    from core.rhino_helpers import get_doc_string, set_doc_string  # noqa: PLC0415

    doc_keys = (
        rhino_bar_pick._DOC_USERTEXT_PAIR_KEY,
        rhino_bar_pick._DOC_USERTEXT_SUBFLOOR_LEFT_KEY,
        rhino_bar_pick._DOC_USERTEXT_SUBFLOOR_RIGHT_KEY,
    )
    counts = {
        "block definition(s)": _rename_block_definitions(rs, caller),
        "ground block(s)": _migrate_ground_blocks(rs),
        "tool(s)": _migrate_tools(rs, config.LAYER_TOOL_INSTANCES),
        "pair-name tag(s)": _migrate_pair_names(rs),
        "default-pair setting(s)": _migrate_doc_pair_settings(
            get_doc_string, set_doc_string, doc_keys
        ),
    }
    total = sum(counts.values())
    if total:
        detail = ", ".join(f"{n} {what}" for what, n in counts.items() if n)
        print(
            f"{caller} (startup): renamed legacy joint names to the "
            f"<Type>_<Subtype> rule -- {detail}."
        )
    return total
