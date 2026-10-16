#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSReorderBarID - Renumber bars, or geometrically relink joints & tools.

On entry the command line offers two operations:

- **Renumber bars** -- renumber every bar so ``B<n>`` matches assembly seq
  ``n`` and propagate the rename through joints/tools via their (valid) string
  references. Use when the joint/tool -> bar links are intact.
- **Relink joints and tools** -- re-derive each joint/tool block's parent
  bar(s) from geometry (``core.joint_relink``) and rewrite ``joint_id`` /
  ``tool_id`` + parent-ref fields to match the bars' current ids. Use after a
  copy/paste, which re-issues fresh ``bar_id``s to the copied curves but leaves
  the copied joints/tools pointing at the *source* bars. Their string refs are
  unrecoverable (the originals may still be in the document, duplicating them),
  but their positions survived the copy. Does not renumber bars.

Renumber-bars workflow
----------------------
1. Run the standard ``repair_on_entry`` pass (heals duplicate seqs etc.).
2. Read every registered bar; assert that:
   - every bar has an integer ``bar_seq`` user-text,
   - the set of seq values is exactly ``{1, 2, ..., N}`` (no gaps, no dups).
   If either check fails, print and abort *before* changing anything.
3. Build the bar-id rename map ``{old_bar_id -> "B<seq>"}`` and the derived
   joint-id rename map (joint blocks ``J<a>-<b>``, ground blocks
   ``G<n>-<name>-<idx>``, tools ``T<joint_id>``).
4. Print both tables (capped at 30 rows each) and ask the user to confirm.
5. On confirm, rewrite every storage location in a single pass inside
   ``suspend_redraw``. No two-phase prefix is needed because each field is
   read-then-written exactly once via the rename map.
6. Re-verify that the resulting bar IDs and seqs are tight ``B1..BN``.

Storage locations rewritten (all inside the Rhino doc):
- Bar centerline curve: ``bar_id`` user-text, ObjectName, ``bar_seq``
- Tube preview Brep: ``tube_bar_id`` user-text
- Bar centerline curve: ``supported_until`` comma-list (per-token remap)
- Joint receiver/male block instance: ``parent_bar_id``, ``connected_bar_id``,
  ``female_parent_bar``, ``male_parent_bar``, ``joint_id``, ObjectName
- Ground block instance: ``parent_bar_id``, ``joint_id``, ObjectName
- Robotic tool block instance: ``joint_id``, ``tool_id``, ObjectName
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import joint_name_conventions as jnc
from core import joint_relink
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    BAR_SEQ_KEY,
    BAR_SUPPORTED_UNTIL_KEY,
    TUBE_BAR_ID_KEY,
    TUBE_LAYER,
    get_all_bars,
    get_bar_seq_map,
    repair_on_entry,
)
from core.rhino_helpers import ask_option, objects_on_layers, suspend_redraw
from core.single_sided_placement import is_single_sided_block


_PRINT_CAP = 30


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------


def _assert_seqs_tight(seq_map, all_bars):
    """Raise AssertionError if seqs aren't exactly {1..N} for every bar."""
    missing = [bid for bid in all_bars if bid not in seq_map]
    assert not missing, (
        "RSReorderBarID: bars without a sequence number: "
        + ", ".join(sorted(missing))
        + ". Run RSUpdatePreview / RSCreateBar to repair, or assign seqs first."
    )
    seqs = sorted(s for (_, s) in seq_map.values())
    expected = list(range(1, len(seqs) + 1))
    assert seqs == expected, (
        f"RSReorderBarID: bar sequences are not a tight 1..N list. "
        f"Got {seqs}, expected {expected}. Fix duplicates/gaps before reordering."
    )


def _build_bar_rename(seq_map):
    """seq_map: {bar_id: (oid, seq)}.  Returns {old_bar_id: new_bar_id}."""
    return {bid: jnc.bar_id(seq) for bid, (_, seq) in seq_map.items()}


def _iter_joint_block_oids():
    """Halves placed by a mate: receiver (Female / MoCap) + Male, ``J…`` ids."""
    return [
        oid for oid in objects_on_layers(*jnc.PAIRED_LAYERS) if not is_single_sided_block(oid)
    ]


def _iter_single_sided_block_oids():
    """Ground blocks, and MoCap blocks placed alone on a bar (``M…`` ids)."""
    return [
        oid for oid in objects_on_layers(*jnc.SINGLE_SIDED_LAYERS) if is_single_sided_block(oid)
    ]


def _build_joint_id_remap(bar_rename):
    """Walk joint + ground instances and return ``{old_jid: new_jid}``.

    Tool instances don't carry parent_bar_id; their joint_id remap is
    inherited from joint/ground via this same dict (looked up at apply time).
    """
    remap = {}

    for oid in _iter_joint_block_oids():
        old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
        if not old_jid:
            continue
        le_old = rs.GetUserText(oid, jnc.UT_RECEIVER_BAR)
        ln_old = rs.GetUserText(oid, jnc.UT_MALE_BAR)
        # Fallbacks - if female_/male_parent_bar were ever missing.
        if not le_old or not ln_old:
            parent = rs.GetUserText(oid, jnc.UT_PARENT_BAR)
            connected = rs.GetUserText(oid, jnc.UT_CONNECTED_BAR)
            # The LAYER tells us which side `parent` is on -- a receiver
            # (female or mocap) sits on the le bar.
            if rs.ObjectLayer(oid) in jnc.RECEIVER_LAYERS:
                le_old, ln_old = parent, connected
            else:
                le_old, ln_old = connected, parent
        if not (le_old and ln_old):
            continue
        le_new = bar_rename.get(le_old, le_old)
        ln_new = bar_rename.get(ln_old, ln_old)
        remap[old_jid] = jnc.pair_joint_id(le_new, ln_new)

    for oid in _iter_single_sided_block_oids():
        old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
        parent_old = rs.GetUserText(oid, jnc.UT_PARENT_BAR)
        if not (old_jid and parent_old) or jnc.split_single_joint_id(old_jid) is None:
            continue
        # Same Type and index, new bar: "G7-T20-2" on B7 -> B12 is "G12-T20-2".
        parent_new = bar_rename.get(parent_old, parent_old)
        remap[old_jid] = jnc.rebar_single_joint_id(old_jid, parent_new)

    return remap


# ---------------------------------------------------------------------------
# Pretty print
# ---------------------------------------------------------------------------


def _print_bar_table(bar_rename, seq_map):
    rows = sorted(
        bar_rename.items(),
        key=lambda kv: seq_map[kv[0]][1],
    )
    n_change = sum(1 for o, n in rows if o != n)
    print("")
    print(f"  Bar rename plan ({n_change} of {len(rows)} will change):")
    print("    seq | old   -> new")
    print("    ----+----------------")
    for old, new in rows[:_PRINT_CAP]:
        seq = seq_map[old][1]
        marker = "  " if old == new else "->"
        print(f"    {seq:>3} | {old:<5} {marker} {new}")
    if len(rows) > _PRINT_CAP:
        print(f"    ... {len(rows) - _PRINT_CAP} more")


def _print_joint_table(jid_remap):
    changed = [(o, n) for o, n in jid_remap.items() if o != n]
    print("")
    print(f"  Derived joint-id remap ({len(changed)} of {len(jid_remap)} will change):")
    if not changed:
        print("    (none)")
        return
    for old, new in sorted(changed)[:_PRINT_CAP]:
        print(f"    {old:<20} -> {new}")
    if len(changed) > _PRINT_CAP:
        print(f"    ... {len(changed) - _PRINT_CAP} more")


def _choose_operation():
    """Top-level prompt: renumber bars vs. relink joints/tools.

    Returns ``"renumber"``, ``"relink"``, or ``None`` (cancelled).
    """
    choice = ask_option(
        "RSReorderBarID - choose operation", ("RenumberBars", "RelinkJointsAndTools")
    )
    return {"RenumberBars": "renumber", "RelinkJointsAndTools": "relink"}.get(choice)


def _confirm_apply(prompt="Apply?"):
    return ask_option(prompt, ("Apply", "Cancel")) == "Apply"


# ---------------------------------------------------------------------------
# Apply
# ---------------------------------------------------------------------------


def _remap_supported_until(curve_oid, bar_rename):
    raw = rs.GetUserText(curve_oid, BAR_SUPPORTED_UNTIL_KEY)
    if not raw:
        return False
    tokens = [t.strip() for t in raw.split(",") if t.strip()]
    new_tokens = [bar_rename.get(t, t) for t in tokens]
    if new_tokens == tokens:
        return False
    rs.SetUserText(curve_oid, BAR_SUPPORTED_UNTIL_KEY, ",".join(new_tokens))
    return True


# NOTE the split KEY_SUPPORT_* support keyframe needs NO remap on renumber:
# it stores a robot name, frames, and joint configs — no bar-id references
# (its validity vs the sequence is re-checked against a fresh
# core.hold_schedule.derive_hold_plan on every read instead). The old
# ``ik_support`` blob's linked_assembled_bar_id remap is gone with the blob.


def _apply_rename(bar_rename, jid_remap, seq_map):
    n_bars = n_tubes = n_supp = n_joints = n_singles = n_tools = 0

    bar_oids = {bid: oid for bid, (oid, _) in seq_map.items()}

    with suspend_redraw():
        # 1. Bar centerlines: bar_id, ObjectName, bar_seq (already correct, but rewrite to be safe).
        for old, new in bar_rename.items():
            oid = bar_oids[old]
            rs.SetUserText(oid, BAR_ID_KEY, new)
            rs.ObjectName(oid, new)
            rs.SetUserText(oid, BAR_SEQ_KEY, jnc.bar_num(new))
            if old != new:
                n_bars += 1

        # 2. Tube previews: tube_bar_id user-text + ObjectName + reference_label.
        for oid in rs.ObjectsByLayer(TUBE_LAYER) or []:
            old = rs.GetUserText(oid, TUBE_BAR_ID_KEY)
            if not old:
                continue
            new = bar_rename.get(old)
            if new and new != old:
                rs.SetUserText(oid, TUBE_BAR_ID_KEY, new)
                new_label = f"{new}_tube"
                rs.ObjectName(oid, new_label)
                # `reference_label` mirrors the ObjectName (set by
                # apply_object_display); keep them in sync.
                if rs.GetUserText(oid, "reference_label"):
                    rs.SetUserText(oid, "reference_label", new_label)
                n_tubes += 1

        # 3. supported_until lives on the bar curves themselves.
        for oid in bar_oids.values():
            if _remap_supported_until(oid, bar_rename):
                n_supp += 1

        # 5. Joint receiver/male instances.
        for oid in _iter_joint_block_oids():
            old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
            new_jid = jid_remap.get(old_jid, old_jid)
            for key in (
                jnc.UT_PARENT_BAR,
                jnc.UT_CONNECTED_BAR,
                jnc.UT_RECEIVER_BAR,
                jnc.UT_MALE_BAR,
            ):
                v = rs.GetUserText(oid, key)
                if v and v in bar_rename and bar_rename[v] != v:
                    rs.SetUserText(oid, key, bar_rename[v])
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, jnc.UT_JOINT_ID, new_jid)
                # Subtype from the layer, the role authority.
                subtype = jnc.subtype_of_layer(rs.ObjectLayer(oid))
                rs.ObjectName(oid, jnc.object_name(new_jid, subtype))
                n_joints += 1

        # 6. Single-sided instances (Ground, standalone MoCap).
        for oid in _iter_single_sided_block_oids():
            old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
            new_jid = jid_remap.get(old_jid, old_jid)
            parent_old = rs.GetUserText(oid, jnc.UT_PARENT_BAR)
            if parent_old and parent_old in bar_rename and bar_rename[parent_old] != parent_old:
                rs.SetUserText(oid, jnc.UT_PARENT_BAR, bar_rename[parent_old])
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, jnc.UT_JOINT_ID, new_jid)
                subtype = jnc.subtype_of_layer(rs.ObjectLayer(oid))
                rs.ObjectName(oid, jnc.object_name(new_jid, subtype))
                n_singles += 1

        # 7. Tool instances - inherit joint_id from jid_remap.
        for oid in objects_on_layers(config.LAYER_TOOL_INSTANCES):
            old_jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
            new_jid = jid_remap.get(old_jid, old_jid)
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, jnc.UT_JOINT_ID, new_jid)
                new_tool_id = jnc.tool_id(new_jid)
                rs.SetUserText(oid, jnc.UT_TOOL_ID, new_tool_id)
                rs.ObjectName(oid, new_tool_id)
                n_tools += 1

    sc.doc.Views.Redraw()
    print(
        f"RSReorderBarID: applied. bars={n_bars}, tubes={n_tubes}, "
        f"supported_until={n_supp}, "
        f"joints={n_joints}, ground/mocap singles={n_singles}, tools={n_tools}"
    )


def _verify_after(expected_n):
    """Re-read seq_map and assert everything is tight B1..BN."""
    seq_map = get_bar_seq_map()
    bars = sorted(seq_map.keys(), key=lambda b: seq_map[b][1])
    assert len(bars) == expected_n, (
        f"RSReorderBarID: post-check found {len(bars)} bars, expected {expected_n}."
    )
    for i, bid in enumerate(bars, start=1):
        seq = seq_map[bid][1]
        assert seq == i, (
            f"RSReorderBarID: post-check seq mismatch: bar {bid} has seq={seq}, "
            f"expected {i}."
        )
        assert bid == jnc.bar_id(i), (
            f"RSReorderBarID: post-check id mismatch: seq {i} -> bar {bid}, "
            f"expected B{i}."
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _run_renumber(all_bars):
    """Renumber bars to ``B1..BN`` and propagate through valid joint/tool refs."""
    if len(all_bars) == 1:
        print("RSReorderBarID: only 1 bar; nothing to reorder.")
        return

    seq_map = get_bar_seq_map()
    try:
        _assert_seqs_tight(seq_map, all_bars)
    except AssertionError as exc:
        print(str(exc))
        return

    bar_rename = _build_bar_rename(seq_map)
    jid_remap = _build_joint_id_remap(bar_rename)

    n_bar_changes = sum(1 for o, n in bar_rename.items() if o != n)
    n_jid_changes = sum(1 for o, n in jid_remap.items() if o != n)
    if n_bar_changes == 0 and n_jid_changes == 0:
        print("RSReorderBarID: bars are already in canonical B1..BN order. Nothing to do.")
        return

    _print_bar_table(bar_rename, seq_map)
    _print_joint_table(jid_remap)

    if not _confirm_apply("Apply renumber?"):
        print("RSReorderBarID: cancelled. No changes applied.")
        return

    _apply_rename(bar_rename, jid_remap, seq_map)
    _verify_after(len(all_bars))
    print("RSReorderBarID: post-check OK (B1..B{} contiguous).".format(len(all_bars)))


def _run_relink():
    """Geometrically relink every joint/tool block to its current parent bar(s)."""
    print(
        "RSReorderBarID: KNOWN ISSUE -- RelinkJointsAndTools can bind a joint or a "
        "tool to the wrong bar when bars or joints sit close together.  Rows "
        "marked '?' are the suspect ones; check each before applying (todos.md)."
    )
    plan = joint_relink.build_plan()
    if not plan["edits"]:
        print("RSReorderBarID: no joint/tool blocks found to relink.")
        return
    if plan["n_changed"] == 0:
        print(
            "RSReorderBarID: all joints/tools already consistent with their bars. "
            "Nothing to do."
        )
        if plan["n_uncertain"]:
            joint_relink.print_plan(plan)  # still show the suspect matches
        return

    joint_relink.print_plan(plan)

    if not _confirm_apply("Apply relink?"):
        print("RSReorderBarID: cancelled. No changes applied.")
        return

    joint_relink.apply_plan(plan)
    problems = joint_relink.verify_links()
    if problems:
        print("RSReorderBarID: relink applied WITH warnings:")
        for problem in problems:
            print(f"  - {problem}")
    else:
        print("RSReorderBarID: relink post-check OK (all parent/joint refs resolve).")


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), caller="RSReorderBarID")

    all_bars = get_all_bars()
    if not all_bars:
        print("RSReorderBarID: no registered bars in document.")
        return

    operation = _choose_operation()
    if operation is None:
        print("RSReorderBarID: cancelled.")
        return
    if operation == "renumber":
        _run_renumber(all_bars)
    elif operation == "relink":
        _run_relink()


if __name__ == "__main__":
    main()
