#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSReorderBarID - Renumber every bar so ``B<n>`` matches assembly seq ``n``.

Workflow
--------
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
- Bar centerline curve: ``ik_support`` JSON blob -> ``linked_assembled_bar_id``
- Joint female/male block instance: ``parent_bar_id``, ``connected_bar_id``,
  ``female_parent_bar``, ``male_parent_bar``, ``joint_id``, ObjectName
- Ground block instance: ``parent_bar_id``, ``joint_id``, ObjectName
- Robotic tool block instance: ``joint_id``, ``tool_id``, ObjectName
"""

import importlib
import json
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
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
from core.rhino_helpers import suspend_redraw


_PRINT_CAP = 30
_IK_SUPPORT_KEY = config.IK_SUPPORT_KEY  # "ik_support"


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
    return {bid: f"B{seq}" for bid, (_, seq) in seq_map.items()}


def _num(bar_id):
    """'B7' -> '7'.  Caller must guarantee bar_id is valid."""
    return bar_id.lstrip("B")


def _layers_for(*names):
    return [n for n in names if rs.IsLayer(n)]


def _iter_joint_block_oids():
    """All female + male joint instance object IDs (oids)."""
    out = []
    for layer in _layers_for(
        config.LAYER_JOINT_FEMALE_INSTANCES, config.LAYER_JOINT_MALE_INSTANCES
    ):
        out.extend(rs.ObjectsByLayer(layer) or [])
    return out


def _iter_ground_block_oids():
    if not rs.IsLayer(config.LAYER_JOINT_GROUND_INSTANCES):
        return []
    return list(rs.ObjectsByLayer(config.LAYER_JOINT_GROUND_INSTANCES) or [])


def _iter_tool_block_oids():
    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return []
    return list(rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or [])


def _build_joint_id_remap(bar_rename):
    """Walk joint + ground instances and return ``{old_jid: new_jid}``.

    Tool instances don't carry parent_bar_id; their joint_id remap is
    inherited from joint/ground via this same dict (looked up at apply time).
    """
    remap = {}

    for oid in _iter_joint_block_oids():
        old_jid = rs.GetUserText(oid, "joint_id")
        if not old_jid:
            continue
        le_old = rs.GetUserText(oid, "female_parent_bar")
        ln_old = rs.GetUserText(oid, "male_parent_bar")
        # Fallbacks - if female_/male_parent_bar were ever missing.
        if not le_old or not ln_old:
            parent = rs.GetUserText(oid, "parent_bar_id")
            connected = rs.GetUserText(oid, "connected_bar_id")
            # Subtype tells us which side `parent` is on.
            if rs.GetUserText(oid, "joint_subtype") == "Female":
                le_old, ln_old = parent, connected
            else:
                le_old, ln_old = connected, parent
        if not (le_old and ln_old):
            continue
        le_new = bar_rename.get(le_old, le_old)
        ln_new = bar_rename.get(ln_old, ln_old)
        new_jid = f"J{_num(le_new)}-{_num(ln_new)}"
        remap[old_jid] = new_jid

    for oid in _iter_ground_block_oids():
        old_jid = rs.GetUserText(oid, "joint_id")
        parent_old = rs.GetUserText(oid, "parent_bar_id")
        ground_name = rs.GetUserText(oid, "ground_joint_name")
        if not (old_jid and parent_old and ground_name):
            continue
        # Trailing index from the old jid: "G7-Wood-2" -> "2"
        idx_tail = old_jid.rsplit("-", 1)[-1]
        parent_new = bar_rename.get(parent_old, parent_old)
        new_jid = f"G{_num(parent_new)}-{ground_name}-{idx_tail}"
        remap[old_jid] = new_jid

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


def _confirm_apply():
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("Apply rename?")
    apply_idx = go.AddOption("Apply")
    cancel_idx = go.AddOption("Cancel")
    go.AcceptNothing(False)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == apply_idx:
                return True
            if chosen == cancel_idx:
                return False
        else:
            return False


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


def _remap_ik_support_blob(curve_oid, bar_rename):
    raw = rs.GetUserText(curve_oid, _IK_SUPPORT_KEY)
    if not raw:
        return False
    try:
        data = json.loads(raw)
    except Exception as exc:
        print(f"  WARNING: could not parse '{_IK_SUPPORT_KEY}' on {curve_oid}: {exc}")
        return False
    linked = data.get("linked_assembled_bar_id")
    if not linked:
        return False
    new_linked = bar_rename.get(linked)
    if new_linked is None:
        print(
            f"  WARNING: 'ik_support.linked_assembled_bar_id' = '{linked}' on "
            f"{curve_oid} doesn't match any current bar; left as-is."
        )
        return False
    if new_linked == linked:
        return False
    data["linked_assembled_bar_id"] = new_linked
    rs.SetUserText(curve_oid, _IK_SUPPORT_KEY, json.dumps(data))
    return True


def _apply_rename(bar_rename, jid_remap, seq_map):
    n_bars = n_tubes = n_supp = n_ik = n_joints = n_grounds = n_tools = 0

    bar_oids = {bid: oid for bid, (oid, _) in seq_map.items()}

    with suspend_redraw():
        # 1. Bar centerlines: bar_id, ObjectName, bar_seq (already correct, but rewrite to be safe).
        for old, new in bar_rename.items():
            oid = bar_oids[old]
            rs.SetUserText(oid, BAR_ID_KEY, new)
            rs.ObjectName(oid, new)
            rs.SetUserText(oid, BAR_SEQ_KEY, _num(new))
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

        # 3 + 4. supported_until + ik_support live on the bar curves themselves.
        for oid in bar_oids.values():
            if _remap_supported_until(oid, bar_rename):
                n_supp += 1
            if _remap_ik_support_blob(oid, bar_rename):
                n_ik += 1

        # 5. Joint female/male instances.
        for oid in _iter_joint_block_oids():
            old_jid = rs.GetUserText(oid, "joint_id")
            new_jid = jid_remap.get(old_jid, old_jid)
            for key in (
                "parent_bar_id",
                "connected_bar_id",
                "female_parent_bar",
                "male_parent_bar",
            ):
                v = rs.GetUserText(oid, key)
                if v and v in bar_rename and bar_rename[v] != v:
                    rs.SetUserText(oid, key, bar_rename[v])
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, "joint_id", new_jid)
                subtype = rs.GetUserText(oid, "joint_subtype") or ""
                suffix = "female" if subtype.lower() == "female" else "male"
                rs.ObjectName(oid, f"{new_jid}_{suffix}")
                n_joints += 1

        # 6. Ground instances.
        for oid in _iter_ground_block_oids():
            old_jid = rs.GetUserText(oid, "joint_id")
            new_jid = jid_remap.get(old_jid, old_jid)
            parent_old = rs.GetUserText(oid, "parent_bar_id")
            if parent_old and parent_old in bar_rename and bar_rename[parent_old] != parent_old:
                rs.SetUserText(oid, "parent_bar_id", bar_rename[parent_old])
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, "joint_id", new_jid)
                rs.ObjectName(oid, f"{new_jid}_ground")
                n_grounds += 1

        # 7. Tool instances - inherit joint_id from jid_remap.
        for oid in _iter_tool_block_oids():
            old_jid = rs.GetUserText(oid, "joint_id")
            new_jid = jid_remap.get(old_jid, old_jid)
            if new_jid and new_jid != old_jid:
                rs.SetUserText(oid, "joint_id", new_jid)
                new_tool_id = f"T{new_jid}"
                rs.SetUserText(oid, "tool_id", new_tool_id)
                rs.ObjectName(oid, new_tool_id)
                n_tools += 1

    sc.doc.Views.Redraw()
    print(
        f"RSReorderBarID: applied. bars={n_bars}, tubes={n_tubes}, "
        f"supported_until={n_supp}, ik_support={n_ik}, "
        f"joints={n_joints}, grounds={n_grounds}, tools={n_tools}"
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
        assert bid == f"B{i}", (
            f"RSReorderBarID: post-check id mismatch: seq {i} -> bar {bid}, "
            f"expected B{i}."
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), caller="RSReorderBarID")

    all_bars = get_all_bars()
    if not all_bars:
        print("RSReorderBarID: no registered bars in document.")
        return
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

    if not _confirm_apply():
        print("RSReorderBarID: cancelled. No changes applied.")
        return

    _apply_rename(bar_rename, jid_remap, seq_map)
    _verify_after(len(all_bars))
    print("RSReorderBarID: post-check OK (B1..B{} contiguous).".format(len(all_bars)))


if __name__ == "__main__":
    main()
