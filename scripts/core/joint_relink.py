"""Geometric re-link of joint & tool block instances to their bar curves.

Motivation
----------
Bars (curves), joint blocks (female/male/ground) and robotic-tool blocks are
tied together purely by *string* ids -- a joint stores ``parent_bar_id`` /
``female_parent_bar`` / ``male_parent_bar`` (a bar's ``bar_id``), and a tool
stores the ``joint_id`` string of the joint it holds. None of them carry a
GUID, so nothing detects a copy/paste.

When a user duplicates a group of (curves + joints + tools), ``ensure_bar_id``
(``core.rhino_bar_registry``) re-issues fresh ``bar_id``s to the copied curves
via its self-GUID copy check, but the copied joints/tools keep their **original**
string references pointing at the *source* bars. With the originals still in the
document those stale strings are also duplicated, so they cannot be repaired by
string remapping (as ``rs_reorder_bar_id`` does for a plain renumber).

The only signal that survives a copy is **geometry**: each block was translated
with its bars, so its world transform still physically sits on the copied bar.
This module re-derives every joint/tool block's parent bar(s) from that geometry
and rewrites the id fields to match the bars' *current* ids.

How the geometry pins a block to a bar
--------------------------------------
A joint block is posed as (``core.joint_pair.fk_half_from_bar_frame``)::

    block_frame = bar_frame @ T_z(jp) @ R_z(jr) @ M_block_from_bar

``M_block_from_bar`` (from ``joint_pairs.json``) has a near-zero radial component
(~0-3 mm), so the block ORIGIN lies essentially on its own bar's axis, while the
mating bar is ``contact_distance`` (~36-40 mm) away. Hence the nearest bar (by
perpendicular distance to the finite axis) is the block's own parent, and the
2nd-nearest is the mate. A robotic tool is posed as
(``core.rhino_tool_place.place_tool_at_block_instance``)::

    world_tool_block = tool_attach_frame(block_id) @ inv(M_tcp_from_block)

so ``tool_frame @ M_tcp_from_block`` reproduces the male/ground block origin --
we match a tool to the joint block whose origin coincides with that TCP point.

``tool_attach_frame`` is the block's own world frame post-multiplied by the
ground definition's ``M_tool_from_block`` (identity for male/female halves).
That offset is a PURE ROTATION about the block origin -- ``GroundJointDef``
zeroes its translation -- so the TCP probe point below is bit-identical either
way and the ``_TOOL_TCP_TOL_MM`` match is unaffected.  An offset carrying a
translation would move the probe and break :func:`_tool_edit`; that is exactly
why the translation is forced to zero rather than merely expected to be.

All distances are millimetres; the document is assumed to be in mm like the rest
of the toolchain (BAR_RADIUS, contact_distance, ... are all mm).

Public API
----------
- :func:`build_plan` -> a plan dict (``edits`` + summary + ``warnings``).
- :func:`print_plan` -> print the human-readable relink table.
- :func:`apply_plan` -> rewrite UserText + ObjectName for the changed blocks.
- :func:`verify_links` -> post-apply sanity check (returns a list of problems).

This module depends on rhinoscriptsyntax / RhinoCommon -- Rhino 8 only.
"""

from collections import defaultdict

import numpy as np
import rhinoscriptsyntax as rs

from core import config
from core.rhino_helpers import curve_endpoints, suspend_redraw
from core.joint_pick_helpers import block_instance_frame
from core.rhino_bar_registry import get_bar_seq_map
from core.robotic_tool import load_robotic_tools


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------

_PRINT_CAP = 40

# A block's nearest-bar perpendicular distance is normally ~0-3 mm (see module
# docstring). Flag the match as UNCERTAIN when the block does not clearly hug a
# single bar, so overlapping copies / manually-moved blocks get surfaced for
# review instead of silently mis-binding.
_UNCERTAIN_ABS_MM = 100.0   # nearest bar this far away -> block floats off any bar
_UNCERTAIN_RATIO = 0.5      # own_dist must be < ratio * connected_dist to be confident
# A tool's TCP point should land ~exactly on its joint block's origin; allow a
# little slack for unit/rounding noise before flagging the tool match uncertain.
_TOOL_TCP_TOL_MM = 50.0


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def _num(bar_id):
    """``'B7'`` -> ``'7'``; safe on empty / already-numeric input."""
    return str(bar_id).lstrip("B") if bar_id else "?"


def _parse_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _layer_oids(layer):
    if not rs.IsLayer(layer):
        return []
    return list(rs.ObjectsByLayer(layer) or [])


def _point_segment_distance(p, a, b):
    """Perpendicular distance from point ``p`` to the finite segment ``a``-``b``."""
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = min(1.0, max(0.0, t))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def _bar_segments():
    """Return ``[(bar_id, oid, seq, start, end)]`` for every registered bar.

    Uses the bars' CURRENT ids (the relink matches blocks against whatever the
    bars are named now; it never renumbers bars).
    """
    segs = []
    for bar_id, (oid, seq) in get_bar_seq_map().items():
        start, end = curve_endpoints(oid)
        segs.append((bar_id, oid, seq, start, end))
    return segs


def _rank_bars(point, segs):
    """Return ``[(dist, seg)]`` sorted nearest-first for ``point`` vs each bar."""
    scored = [(_point_segment_distance(point, s[3], s[4]), s) for s in segs]
    scored.sort(key=lambda x: x[0])
    return scored


def _mark_changed(edit):
    """Set ``edit['changed']`` by comparing the planned writes to the live block.

    A block that already carries the exact ids/name (e.g. a correctly-linked
    ORIGINAL) is left untouched -- this is what makes the relink idempotent.
    """
    oid = edit["oid"]
    changed = rs.ObjectName(oid) != edit["new_name"]
    if not changed:
        for key, value in edit["usertext"].items():
            if (rs.GetUserText(oid, key) or "") != value:
                changed = True
                break
    edit["changed"] = changed
    return edit


# ---------------------------------------------------------------------------
# Female / male joint matching
# ---------------------------------------------------------------------------


def _pair_confidence(own_dist, conn_dist):
    """Uncertainty verdict for a two-bar (female/male) match."""
    if own_dist > _UNCERTAIN_ABS_MM:
        return True, f"nearest bar {own_dist:.0f}mm away (block not on a bar?)"
    if conn_dist <= 1e-9:
        return True, "two bars coincide at the block"
    if own_dist > _UNCERTAIN_RATIO * conn_dist:
        return True, f"ambiguous: nearest {own_dist:.0f}mm vs 2nd {conn_dist:.0f}mm"
    return False, ""


def _degenerate_edit(oid, kind, reason):
    """An edit that proposes no change (kept as-is) but is flagged uncertain."""
    old_jid = rs.GetUserText(oid, "joint_id") or ""
    edit = {
        "oid": oid,
        "kind": kind,
        "old_jid": old_jid,
        "new_jid": old_jid,
        "old_tid": rs.GetUserText(oid, "tool_id") or "",
        "new_tid": rs.GetUserText(oid, "tool_id") or "",
        "new_name": rs.ObjectName(oid) or "",
        "usertext": {},
        "parents": "?",
        "dist": float("nan"),
        "uncertain": True,
        "reason": reason,
        "_origin": None,
    }
    edit["changed"] = False
    return edit


def _female_male_edit(oid, role, segs):
    """Plan the relink of one female or male joint block from its position."""
    try:
        frame, _block_name = block_instance_frame(oid)
    except Exception as exc:  # not a block instance / exploded
        return _degenerate_edit(oid, role, f"frame read failed ({exc})")

    origin = frame[:3, 3]
    ranked = _rank_bars(origin, segs)
    if len(ranked) < 2:
        return _degenerate_edit(oid, role, "fewer than 2 bars in document")

    own_dist, own_seg = ranked[0]
    conn_dist, conn_seg = ranked[1]
    own_bar, conn_bar = own_seg[0], conn_seg[0]
    uncertain, reason = _pair_confidence(own_dist, conn_dist)

    if role == "female":
        female_bar, male_bar = own_bar, conn_bar
    else:
        male_bar, female_bar = own_bar, conn_bar
    new_jid = f"J{_num(female_bar)}-{_num(male_bar)}"

    edit = {
        "oid": oid,
        "kind": role,
        "old_jid": rs.GetUserText(oid, "joint_id") or "",
        "new_jid": new_jid,
        "old_tid": None,
        "new_tid": None,
        "new_name": f"{new_jid}_{role}",
        "usertext": {
            "joint_id": new_jid,
            "parent_bar_id": own_bar,
            "connected_bar_id": conn_bar,
            "female_parent_bar": female_bar,
            "male_parent_bar": male_bar,
        },
        "parents": f"F={female_bar} M={male_bar}",
        "dist": own_dist,
        "uncertain": uncertain,
        "reason": reason,
        "_origin": origin,
    }
    return _mark_changed(edit)


# ---------------------------------------------------------------------------
# Ground joint matching
# ---------------------------------------------------------------------------


def _ground_edit_raw(oid, segs):
    """First pass for a ground block: resolve its bar; index assigned later."""
    try:
        frame, _block_name = block_instance_frame(oid)
    except Exception as exc:
        return _degenerate_edit(oid, "ground", f"frame read failed ({exc})")

    origin = frame[:3, 3]
    ranked = _rank_bars(origin, segs)
    if not ranked:
        return _degenerate_edit(oid, "ground", "no bars in document")

    own_dist, own_seg = ranked[0]
    own_bar = own_seg[0]
    conn_dist = ranked[1][0] if len(ranked) > 1 else float("inf")
    uncertain, reason = _pair_confidence(own_dist, conn_dist)

    return {
        "oid": oid,
        "kind": "ground",
        "old_jid": rs.GetUserText(oid, "joint_id") or "",
        "old_tid": None,
        "new_tid": None,
        "dist": own_dist,
        "uncertain": uncertain,
        "reason": reason,
        "_origin": origin,
        "own_bar": own_bar,
        "ground_name": rs.GetUserText(oid, "ground_joint_name") or "",
        "position_mm": _parse_float(rs.GetUserText(oid, "position_mm")),
    }


def _finalize_ground_edit(edit, own_bar, ground_name, index):
    from core.ground_placement import make_ground_joint_id

    new_jid = make_ground_joint_id(own_bar, ground_name, index=index)
    edit["new_jid"] = new_jid
    edit["new_name"] = f"{new_jid}_ground"
    edit["usertext"] = {"joint_id": new_jid, "parent_bar_id": own_bar}
    edit["parents"] = f"{own_bar}"
    _mark_changed(edit)


def _assign_ground_indices(ground_edits):
    """Assign per-``(bar, ground_name)`` indices, preserving already-valid ones.

    A correctly-linked ORIGINAL ground whose ``joint_id`` already matches its
    resolved bar keeps its index; copied grounds (whose old id encodes the
    *source* bar) get the next free index in deterministic position order.
    """
    groups = defaultdict(list)
    for edit in ground_edits:
        if edit.get("own_bar") is None:  # degenerate; already finalized
            continue
        groups[(edit["own_bar"], edit["ground_name"])].append(edit)

    for (own_bar, ground_name), members in groups.items():
        base = f"G{_num(own_bar)}-{ground_name}-"
        used = set()
        pending = []
        for edit in members:
            old_jid = edit.get("old_jid") or ""
            tail = old_jid[len(base):] if old_jid.startswith(base) else ""
            if tail.isdigit() and int(tail) not in used:
                edit["_idx"] = int(tail)
                used.add(int(tail))
            else:
                pending.append(edit)
        for edit in sorted(
            pending, key=lambda e: (e.get("position_mm") or 0.0, str(e["oid"]))
        ):
            idx = 0
            while idx in used:
                idx += 1
            edit["_idx"] = idx
            used.add(idx)
        for edit in members:
            _finalize_ground_edit(edit, own_bar, ground_name, edit["_idx"])


# ---------------------------------------------------------------------------
# Tool matching
# ---------------------------------------------------------------------------


def _tool_edit(oid, targets, tools_reg):
    """Relink one tool block to the male/ground block its TCP coincides with.

    ``targets`` is ``[(block_oid, origin_xyz, new_joint_id)]`` for every male /
    ground joint block (with its freshly-derived joint id).
    """
    try:
        frame, _block_name = block_instance_frame(oid)
    except Exception as exc:
        return _degenerate_edit(oid, "tool", f"frame read failed ({exc})")

    tool_name = rs.GetUserText(oid, "tool_name") or ""
    tdef = tools_reg.get(tool_name)
    if tdef is not None:
        # TCP point reproduces the joint block origin the tool was placed on.
        probe = (frame @ np.asarray(tdef.M_tcp_from_block, dtype=float))[:3, 3]
        tol = _TOOL_TCP_TOL_MM
    else:
        probe = frame[:3, 3]  # fallback: raw tool origin (less precise)
        tol = None

    best = None
    for _t_oid, t_origin, t_jid in targets:
        if t_origin is None:
            continue
        d = float(np.linalg.norm(probe - t_origin))
        if best is None or d < best[0]:
            best = (d, t_jid)

    old_jid = rs.GetUserText(oid, "joint_id") or ""
    old_tid = rs.GetUserText(oid, "tool_id") or ""
    if best is None:
        new_jid, uncertain, reason = old_jid, True, "no male/ground joint block to match"
    else:
        d, new_jid = best
        if tdef is None:
            uncertain = True
            reason = f"tool '{tool_name}' not in registry; matched nearest block ({d:.0f}mm)"
        elif d > tol:
            uncertain = True
            reason = f"TCP {d:.0f}mm from nearest joint block"
        else:
            uncertain, reason = False, ""

    new_tid = f"T{new_jid}"
    edit = {
        "oid": oid,
        "kind": "tool",
        "old_jid": old_jid,
        "new_jid": new_jid,
        "old_tid": old_tid,
        "new_tid": new_tid,
        "new_name": new_tid,
        "usertext": {"joint_id": new_jid, "tool_id": new_tid},
        "parents": f"@{new_jid}",
        "dist": best[0] if best else float("nan"),
        "uncertain": uncertain,
        "reason": reason,
        "_origin": None,
    }
    return _mark_changed(edit)


# ---------------------------------------------------------------------------
# Consistency checks
# ---------------------------------------------------------------------------


def _consistency_warnings(joint_edits):
    """Flag joints whose female + male halves disagree on the bar pair.

    Female and male blocks are matched independently; if they resolve to the
    same joint they must produce the SAME ``joint_id``. A ``new_jid`` that is
    not backed by exactly one female + one male signals a bad/ambiguous match.
    """
    by_jid = defaultdict(lambda: {"female": [], "male": []})
    for edit in joint_edits:
        if edit["kind"] in ("female", "male") and edit.get("new_jid"):
            by_jid[edit["new_jid"]][edit["kind"]].append(edit)

    warnings = []
    for jid, halves in sorted(by_jid.items()):
        nf, nm = len(halves["female"]), len(halves["male"])
        if nf != 1 or nm != 1:
            warnings.append(
                f"joint {jid}: resolved {nf} female + {nm} male half(s) "
                "(expected 1 + 1) -- check the flagged rows."
            )
    return warnings


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build_plan():
    """Return a relink plan for every joint/tool block in the document.

    Plan dict::

        {
          "edits": [edit, ...],   # one per female/male/ground/tool block
          "n_changed": int,
          "n_uncertain": int,
          "warnings": [str, ...],
        }
    """
    segs = _bar_segments()
    if not segs:
        return {"edits": [], "n_changed": 0, "n_uncertain": 0,
                "warnings": ["no registered bars in document"]}

    joint_edits = []
    for layer, role in (
        (config.LAYER_JOINT_FEMALE_INSTANCES, "female"),
        (config.LAYER_JOINT_MALE_INSTANCES, "male"),
    ):
        for oid in _layer_oids(layer):
            joint_edits.append(_female_male_edit(oid, role, segs))

    ground_edits = [_ground_edit_raw(oid, segs)
                    for oid in _layer_oids(config.LAYER_JOINT_GROUND_INSTANCES)]
    _assign_ground_indices(ground_edits)
    joint_edits.extend(ground_edits)

    # Male + ground blocks are the tool anchors (females never carry a tool).
    targets = [
        (e["oid"], e.get("_origin"), e["new_jid"])
        for e in joint_edits
        if e["kind"] in ("male", "ground") and e.get("new_jid") and not e["uncertain"]
    ]
    tools_reg = load_robotic_tools()
    tool_edits = [_tool_edit(oid, targets, tools_reg)
                  for oid in _layer_oids(config.LAYER_TOOL_INSTANCES)]

    edits = joint_edits + tool_edits
    warnings = _consistency_warnings(joint_edits)
    return {
        "edits": edits,
        "n_changed": sum(1 for e in edits if e["changed"]),
        "n_uncertain": sum(1 for e in edits if e["uncertain"]),
        "warnings": warnings,
    }


def _sort_key(edit):
    order = {"female": 0, "male": 1, "ground": 2, "tool": 3}
    return (order.get(edit["kind"], 9), edit["new_jid"] or "", str(edit["oid"]))


def print_plan(plan):
    """Print the relink table (changed rows first, then uncertain, capped)."""
    edits = plan["edits"]
    changed = [e for e in edits if e["changed"]]
    print("")
    print(
        f"  Joint/tool relink plan ({len(changed)} of {len(edits)} block(s) will "
        f"change; {plan['n_uncertain']} uncertain):"
    )
    print("    kind   | old id            -> new id            | bars        | flag")
    print("    -------+-------------------+--------------------+-------------+-----")
    # Show changed rows first, then any remaining uncertain ones.
    rows = [e for e in sorted(changed, key=_sort_key)]
    shown = set(id(e) for e in rows)
    rows += [e for e in sorted(edits, key=_sort_key)
             if e["uncertain"] and id(e) not in shown]
    for edit in rows[:_PRINT_CAP]:
        if edit["kind"] == "tool":
            old_id = edit["old_tid"] or ""
            new_id = edit["new_tid"] or ""
        else:
            old_id = edit["old_jid"] or ""
            new_id = edit["new_jid"] or ""
        flag = "  ?" if edit["uncertain"] else ("  ->" if edit["changed"] else "   =")
        print(
            f"    {edit['kind']:<6} | {old_id:<17} -> {new_id:<18} "
            f"| {edit['parents']:<11} | {flag.strip()}"
        )
        if edit["uncertain"] and edit["reason"]:
            print(f"           ^ uncertain: {edit['reason']}")
    if len(rows) > _PRINT_CAP:
        print(f"    ... {len(rows) - _PRINT_CAP} more")
    for warn in plan["warnings"]:
        print(f"  WARNING: {warn}")


def apply_plan(plan):
    """Write the planned UserText + ObjectName for every changed block.

    Returns the number of blocks rewritten.
    """
    n = 0
    with suspend_redraw():
        for edit in plan["edits"]:
            if not edit["changed"]:
                continue
            for key, value in edit["usertext"].items():
                rs.SetUserText(edit["oid"], key, value)
            rs.ObjectName(edit["oid"], edit["new_name"])
            n += 1
    print(f"core.joint_relink.apply_plan: rewrote {n} joint/tool block(s).")
    return n


def verify_links():
    """Post-apply sanity check. Returns a list of human-readable problems.

    Confirms every joint block's ``parent_bar_id`` resolves to a live bar and
    every tool's ``joint_id`` matches some joint block.
    """
    problems = []
    bar_ids = set(get_bar_seq_map().keys())

    joint_ids = set()
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        for oid in _layer_oids(layer):
            jid = rs.GetUserText(oid, "joint_id")
            if jid:
                joint_ids.add(jid)
            pid = rs.GetUserText(oid, "parent_bar_id")
            if pid and pid not in bar_ids:
                problems.append(
                    f"{rs.ObjectName(oid) or oid}: parent_bar_id '{pid}' is not a live bar"
                )

    for oid in _layer_oids(config.LAYER_TOOL_INSTANCES):
        jid = rs.GetUserText(oid, "joint_id")
        if jid and jid not in joint_ids:
            problems.append(
                f"{rs.ObjectName(oid) or oid}: tool joint_id '{jid}' matches no joint block"
            )
    return problems
