"""Bar identity and tube-preview management for the Rhino scaffolding workflow.

Every scaffolding bar (a Rhino curve) gets:
  - ``bar_type``  user-text key  → ``"scaffolding_bar"``
  - ``bar_id``    user-text key  → ``"B1"``, ``"B2"``, …
  - ``bar_guid``  user-text key  → the Rhino GUID at assignment time
  - ObjectName                   → same as ``bar_id``

Tube previews (display cylinders) live on the ``"Tube preview"`` layer and
cache the axis endpoints so stale tubes can be detected and regenerated.

This module depends on rhinoscriptsyntax / scriptcontext (Rhino 8 only).
"""

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core import config
from core.build_stage import (
    BUILD_STAGE_KEY,
    STATUS_CLEAR,
    STATUS_FALLBACK,
    STATUS_OFF,
    format_build_stage,
    parse_build_stage,
    resolve_build_stage_seq,
    stage_filter_note,
)
from core.rhino_helpers import (
    as_object_id_list,
    apply_object_display,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    get_doc_string,
    point_to_array,
    set_doc_string,
    suspend_redraw,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAR_TYPE_KEY = "bar_type"
BAR_ID_KEY = "bar_id"
BAR_GUID_KEY = "bar_guid"
BAR_SEQ_KEY = "bar_seq"
BAR_SUPPORTED_UNTIL_KEY = "supported_until"
BAR_TYPE_VALUE = "scaffolding_bar"

# A "fake" bar is a MODELING ARTIFACT that will not be fabricated and is not
# physically present.  It exists only to give a real bar's male joint something
# to be modeled against, so the male the robot grasps sits at the right pose.
# It stays a full registered bar in most respects (bar id, assembly step, its
# joint halves) so the joint math and the sequence display keep working.
#
# ! It is NOT collision geometry: the fake bar and every joint half mounted on
# it (its females) are excluded from every collision scene -- assembly and
# support alike -- by `core.env_collision`.  Only the real bar's male half,
# which is parented to the real bar, survives.  Toggling this mark therefore
# changes the collision scene, so it is part of the cell staleness fingerprint
# (`core.robot_cell._live_assembly_fingerprint`).
#
# Value is "1" when set; the key is removed otherwise, so the common case (a
# real bar) costs one absent-key read.
BAR_IS_FAKE_KEY = "scaffolding.fake_bar"

# Layer names are owned by ``core.config`` so the whole toolchain agrees.
TUBE_LAYER = config.LAYER_BAR_TUBE_PREVIEWS
BAR_CENTERLINE_LAYER = config.LAYER_BAR_CENTERLINES
TUBE_BAR_ID_KEY = "tube_bar_id"
TUBE_AXIS_GUID_KEY = "tube_axis_id"
TUBE_SELF_GUID_KEY = "tube_self_guid"
TUBE_CACHE_START = "tube_cache_start"
TUBE_CACHE_END = "tube_cache_end"
BAR_RADIUS_KEY = "tube_radius"

_POINT_TOL = 1e-3  # mm tolerance for endpoint cache comparison


# ---------------------------------------------------------------------------
# Bar ID helpers
# ---------------------------------------------------------------------------


def _parse_bar_number(bar_id):
    """Extract integer from a bar_id like 'B7' → 7.  Return None on failure."""
    if not bar_id or not bar_id.upper().startswith("B"):
        return None
    try:
        return int(bar_id[1:])
    except (ValueError, IndexError):
        return None


def _parse_bar_seq(s):
    """Parse a stored sequence string to int, or None on failure."""
    if not s:
        return None
    try:
        return int(s)
    except (ValueError, TypeError):
        return None


def next_bar_id():
    """Return the next available bar ID (e.g. ``'B4'``)."""
    max_num = 0
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
            num = _parse_bar_number(rs.GetUserText(oid, BAR_ID_KEY))
            if num is not None and num > max_num:
                max_num = num
    return f"B{max_num + 1}"


def next_bar_seq():
    """Return the next available assembly sequence number (max existing + 1)."""
    max_seq = 0
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
            seq = _parse_bar_seq(rs.GetUserText(oid, BAR_SEQ_KEY))
            if seq is not None and seq > max_seq:
                max_seq = seq
    return max_seq + 1


def ensure_bar_seq(curve_id):
    """Ensure *curve_id* has an assembly sequence number.

    Assigns the next available integer if the key is missing.  Does **not**
    resolve duplicate sequence numbers — call :func:`repair_bar_sequences`
    after processing a batch of bars to fix those.

    Returns the sequence number as an ``int``.
    """
    existing = _parse_bar_seq(rs.GetUserText(curve_id, BAR_SEQ_KEY))
    if existing is not None:
        return existing
    new_seq = next_bar_seq()
    rs.SetUserText(curve_id, BAR_SEQ_KEY, str(new_seq))
    return new_seq


def ensure_bar_id(curve_id, bar_type=BAR_TYPE_VALUE):
    """Ensure *curve_id* has a bar ID.  Assigns one if missing or copy-pasted.

    Also calls :func:`ensure_bar_seq` so every registered bar always carries
    both keys.  For copy-pasted bars the old sequence value is intentionally
    preserved here — it gives :func:`repair_bar_sequences` a relative-order
    hint when it appends those bars after the primary sequence.

    Returns the ``bar_id`` string (e.g. ``'B3'``).
    """
    current_guid = str(rs.coerceguid(curve_id))
    existing_type = rs.GetUserText(curve_id, BAR_TYPE_KEY)
    existing_guid = rs.GetUserText(curve_id, BAR_GUID_KEY)

    if existing_type == bar_type and existing_guid == current_guid:
        # Already registered and GUID matches — just ensure ObjectName is in sync.
        bar_id = rs.GetUserText(curve_id, BAR_ID_KEY)
        if bar_id and rs.ObjectName(curve_id) != bar_id:
            rs.ObjectName(curve_id, bar_id)
        # Guard against older-format bars that predate sequence tracking.
        ensure_bar_seq(curve_id)
        return bar_id

    # New bar or copy-paste detected — assign a fresh ID.
    new_id = next_bar_id()
    rs.SetUserText(curve_id, BAR_TYPE_KEY, bar_type)
    rs.SetUserText(curve_id, BAR_ID_KEY, new_id)
    rs.SetUserText(curve_id, BAR_GUID_KEY, current_guid)
    rs.ObjectName(curve_id, new_id)
    # Move the centerline curve onto the managed centerline layer.
    ensure_layer(BAR_CENTERLINE_LAYER)
    rs.ObjectLayer(curve_id, BAR_CENTERLINE_LAYER)
    # ensure_bar_seq is a no-op for copy-pastes (old seq survives), and assigns
    # the next integer for brand-new curves (no user text at all).
    ensure_bar_seq(curve_id)
    return new_id


def is_bar(curve_id):
    """Return True if *curve_id* is a registered scaffolding bar."""
    return rs.GetUserText(curve_id, BAR_TYPE_KEY) == BAR_TYPE_VALUE


def get_all_bars():
    """Scan the document for all registered bars.  Heals copy-paste artifacts.

    Returns ``{bar_id: curve_guid}`` dict.
    """
    bars = {}
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE:
            bar_id = ensure_bar_id(oid)  # heals guid mismatch / name drift
            bars[bar_id] = rs.coerceguid(oid)
    return bars


# Legacy bundled IK blob keys (written by older builds before the solutions
# were split into the KEY_ASSEMBLY_* / KEY_SUPPORT_* keys in core.config).
# Cleared for back-compat so an old bar can't keep a stale IK record.
LEGACY_IK_ASSEMBLY_KEY = "ik_assembly"
LEGACY_IK_SUPPORT_KEY = "ik_support"


def clear_support_ik_keyframe(bar_oid):
    """Delete a bar's saved support-robot hold keyframe (all split keys).

    The support keyframe is all-or-nothing (robot name + base + grasp +
    approach + held), so there are no partial-clear switches — everything
    goes, including the legacy ``ik_support`` blob from older builds.

    Args:
        bar_oid: Rhino object id of the held bar's curve.

    Returns:
        list[str]: the user-text keys that were present and removed.
    """
    keys = [
        config.KEY_SUPPORT_ROBOT,
        config.KEY_SUPPORT_BASE_FRAME,
        config.KEY_SUPPORT_GRASP_FRAME,
        config.KEY_SUPPORT_IK_APPROACH,
        config.KEY_SUPPORT_IK_HELD,
        LEGACY_IK_SUPPORT_KEY,
    ]
    removed = []
    for key in keys:
        if rs.GetUserText(bar_oid, key):
            rs.SetUserText(bar_oid, key)  # 2-arg form deletes the key/value pair
            removed.append(key)
    return removed


def clear_assembly_ik_keyframe(bar_oid, clear_keyframe=True, clear_base_frame=True):
    """Delete a bar's saved dual-arm IK data (arm configs and/or base frame).

    Two INDEPENDENT switches:

    - ``clear_keyframe`` removes the M1-M3 arm IK configs (approach / assembled /
      retreat).
    - ``clear_base_frame`` removes the robot BASE frame (the mobile base position).
      Keeping the base (``clear_base_frame=False``) lets the bar be re-solved at the
      same base via RSIKKeyframe's "reuse saved base" path.

    The legacy ``ik_assembly`` blob bundles base + configs in a single string that
    cannot be split, so it is removed whenever EITHER switch is on (the split keys
    are the authoritative ones; leaving a half-stale blob for legacy readers like
    RSShowIK would be worse than dropping it). M4 returns the arms to the fixed
    dual-arm HOME configuration, a module constant (``config.HOME_CONF_*``) not
    stored per bar, so it is untouched.

    Args:
        bar_oid: Rhino object id of the bar curve.
        clear_keyframe (bool): remove the M1-M3 arm IK configs (default True).
        clear_base_frame (bool): remove the robot base frame (default True).

    Returns:
        list[str]: the user-text keys that were present and removed (empty if the
        bar had nothing matching the switches to clear).
    """
    keys = []
    if clear_keyframe:
        keys += [
            config.KEY_ASSEMBLY_IK_APPROACH,
            config.KEY_ASSEMBLY_IK_ASSEMBLED,
            config.KEY_ASSEMBLY_IK_RETREAT,
        ]
    if clear_base_frame:
        keys.append(config.KEY_ASSEMBLY_BASE_FRAME)
    if clear_keyframe or clear_base_frame:
        keys.append(LEGACY_IK_ASSEMBLY_KEY)
    removed = []
    for key in keys:
        if rs.GetUserText(bar_oid, key):
            rs.SetUserText(bar_oid, key)  # 2-arg form deletes the key/value pair
            removed.append(key)
    return removed


# ---------------------------------------------------------------------------
# Assembly sequence helpers
# ---------------------------------------------------------------------------


def get_bar_seq_map():
    """Return assembly-sequence information for all registered bars.

    Returns ``{bar_id: (oid, seq)}`` where *oid* is usable by rhinoscriptsyntax
    and *seq* is the integer assembly sequence number.
    """
    result = {}
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE:
            continue
        bar_id = rs.GetUserText(oid, BAR_ID_KEY)
        seq = _parse_bar_seq(rs.GetUserText(oid, BAR_SEQ_KEY))
        if bar_id and seq is not None:
            result[bar_id] = (oid, seq)
    return result


def repair_bar_sequences():
    """Resolve duplicate and missing sequence numbers across all registered bars.

    Called automatically by RSCreateBar after registering a batch.  May also
    be called standalone to heal inconsistencies from copy-paste or manual
    curve editing.

    Rules
    -----
    - Among bars sharing a sequence number, the bar with the lowest numeric bar
      ID is treated as the *primary* (the original); all others are *secondary*
      (copy-paste artifacts).
    - Bars with no sequence number are also secondary.
    - Primary bars are compacted to fill any gaps left by deleted bars, while
      preserving their relative assembly order.
    - Secondary bars are appended after all primary bars, sorted by their old
      sequence number (preserving the relative order they were copied from),
      with ties broken by bar ID number.

    Returns ``{bar_id: new_seq}`` for every bar whose sequence was changed.
    """
    from collections import defaultdict

    # Scrub stale ``supported_until`` refs FIRST -- before we touch any
    # sequence numbers and (more importantly) before ``ensure_bar_id``
    # gets a chance to reassign bar IDs to copy-pasted curves further
    # downstream.  Otherwise a dependency on a deleted bar id could
    # silently rebind to a freshly-issued bar id of an unrelated copy.
    stale = cleanup_stale_supports()
    if stale:
        details = ", ".join(
            f"{bid}->[{','.join(removed)}]" for bid, removed in stale.items()
        )
        print(
            f"repair_bar_sequences: scrubbed stale supported_until refs: {details}"
        )

    bar_data = []  # (oid, bar_id, bar_id_num, old_seq)
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE:
            continue
        bar_id = rs.GetUserText(oid, BAR_ID_KEY)
        bar_id_num = _parse_bar_number(bar_id) or 0
        old_seq = _parse_bar_seq(rs.GetUserText(oid, BAR_SEQ_KEY))
        bar_data.append((oid, bar_id, bar_id_num, old_seq))

    if not bar_data:
        return {}

    seq_groups = defaultdict(list)
    no_seq_items = []
    for item in bar_data:
        old_seq = item[3]
        if old_seq is None:
            no_seq_items.append(item)
        else:
            seq_groups[old_seq].append(item)

    primaries = []
    secondaries = []
    for group in seq_groups.values():
        if len(group) == 1:
            primaries.append(group[0])
        else:
            # Lowest bar_id_num is the oldest (original); the rest are copies.
            group.sort(key=lambda x: x[2])
            primaries.append(group[0])
            secondaries.extend(group[1:])
    secondaries.extend(no_seq_items)

    # Sort primaries by their old seq to compact in original assembly order.
    primaries.sort(key=lambda x: x[3])

    # Sort secondaries by (old_seq or ∞, bar_id_num) to preserve copied order.
    def _secondary_key(item):
        seq = item[3] if item[3] is not None else float("inf")
        return (seq, item[2])

    secondaries.sort(key=_secondary_key)

    changed = {}
    for new_seq, (oid, bar_id, _, old_seq) in enumerate(primaries, start=1):
        if old_seq != new_seq:
            rs.SetUserText(oid, BAR_SEQ_KEY, str(new_seq))
            changed[bar_id] = new_seq

    offset = len(primaries) + 1
    for i, (oid, bar_id, _, _) in enumerate(secondaries):
        new_seq = offset + i
        rs.SetUserText(oid, BAR_SEQ_KEY, str(new_seq))
        changed[bar_id] = new_seq

    return changed


def _apply_seq_order(ordered_bar_ids):
    """Reassign sequence numbers 1..N given an explicit ordered list of bar IDs.

    Internal helper for the reorder operations.  Bars not present in
    *ordered_bar_ids* are left unchanged.

    Returns ``{bar_id: new_seq}`` for bars whose sequence changed.
    """
    bar_map = get_bar_seq_map()
    changed = {}
    for new_seq, bar_id in enumerate(ordered_bar_ids, start=1):
        if bar_id not in bar_map:
            continue
        oid, old_seq = bar_map[bar_id]
        if old_seq != new_seq:
            rs.SetUserText(oid, BAR_SEQ_KEY, str(new_seq))
            changed[bar_id] = new_seq
    return changed


def reorder_bars(ordered_bar_ids):
    """Reassign assembly sequences 1..N matching the given order.

    Parameters
    ----------
    ordered_bar_ids : list[str]
        Bar IDs in the desired assembly order, e.g. ``['B3', 'B1', 'B2']``.

    Returns
    -------
    dict
        ``{bar_id: new_seq}`` for bars whose sequence changed.
    """
    return _apply_seq_order(ordered_bar_ids)


def move_bar_earlier(bar_id):
    """Swap *bar_id*'s assembly sequence with the immediately preceding bar.

    No-op if *bar_id* is already first.

    Returns ``{bar_id: new_seq, other_bar_id: new_seq}`` or ``{}`` if no change.
    """
    bar_map = get_bar_seq_map()
    if bar_id not in bar_map:
        return {}
    oid_a, seq_a = bar_map[bar_id]
    if seq_a <= 1:
        return {}
    prev_bar = next((bid for bid, (_, s) in bar_map.items() if s == seq_a - 1), None)
    if prev_bar is None:
        return {}
    oid_b, seq_b = bar_map[prev_bar]
    rs.SetUserText(oid_a, BAR_SEQ_KEY, str(seq_b))
    rs.SetUserText(oid_b, BAR_SEQ_KEY, str(seq_a))
    return {bar_id: seq_b, prev_bar: seq_a}


def move_bar_later(bar_id):
    """Swap *bar_id*'s assembly sequence with the immediately following bar.

    No-op if *bar_id* is already last.

    Returns ``{bar_id: new_seq, other_bar_id: new_seq}`` or ``{}`` if no change.
    """
    bar_map = get_bar_seq_map()
    if bar_id not in bar_map:
        return {}
    oid_a, seq_a = bar_map[bar_id]
    next_bar = next((bid for bid, (_, s) in bar_map.items() if s == seq_a + 1), None)
    if next_bar is None:
        return {}
    oid_b, seq_b = bar_map[next_bar]
    rs.SetUserText(oid_a, BAR_SEQ_KEY, str(seq_b))
    rs.SetUserText(oid_b, BAR_SEQ_KEY, str(seq_a))
    return {bar_id: seq_b, next_bar: seq_a}


def insert_bar_after(bar_id, target_bar_id):
    """Move *bar_id* to come immediately after *target_bar_id* in assembly order.

    All bars between the current and destination positions shift by one to
    accommodate.  Pass ``target_bar_id=None`` to move *bar_id* to position 1
    (the very first bar assembled).

    Parameters
    ----------
    bar_id : str
        The bar to relocate.
    target_bar_id : str or None
        The bar that should immediately precede *bar_id* after the move.
        Pass ``None`` to make *bar_id* the first bar assembled.

    Returns ``{bar_id: new_seq, …}`` for every bar whose sequence changed,
    or ``{}`` if *bar_id* is already in the requested position.
    """
    bar_map = get_bar_seq_map()
    if bar_id not in bar_map:
        return {}
    if target_bar_id is not None and target_bar_id not in bar_map:
        return {}
    sorted_bars = sorted(bar_map.keys(), key=lambda b: bar_map[b][1])
    sorted_bars.remove(bar_id)
    if target_bar_id is None:
        sorted_bars.insert(0, bar_id)
    else:
        idx = sorted_bars.index(target_bar_id)
        sorted_bars.insert(idx + 1, bar_id)
    return _apply_seq_order(sorted_bars)


def insert_bar_before(bar_id, target_bar_id):
    """Move *bar_id* to come immediately before *target_bar_id* in assembly order.

    All bars between the current and destination positions shift by one to
    accommodate.

    Parameters
    ----------
    bar_id : str
        The bar to relocate.
    target_bar_id : str
        The bar that should immediately follow *bar_id* after the move.

    Returns ``{bar_id: new_seq, …}`` for every bar whose sequence changed,
    or ``{}`` if *bar_id* is already in the requested position.
    """
    bar_map = get_bar_seq_map()
    if bar_id not in bar_map or target_bar_id not in bar_map:
        return {}
    sorted_bars = sorted(bar_map.keys(), key=lambda b: bar_map[b][1])
    sorted_bars.remove(bar_id)
    idx = sorted_bars.index(target_bar_id)
    sorted_bars.insert(idx, bar_id)
    return _apply_seq_order(sorted_bars)


# ---------------------------------------------------------------------------
# Temporary-support metadata (``supported_until``)
# ---------------------------------------------------------------------------
#
# A bar may need temporary robotic support after it is placed, until some
# *other* bars further down the assembly sequence are installed.  We store
# that requirement as a comma-separated list of bar IDs (e.g. ``"B3,B7"``)
# in the ``supported_until`` UserText key on the bar centerline curve.


def _split_bar_id_list(s):
    if not s:
        return []
    return [tok.strip() for tok in s.split(",") if tok.strip()]


def get_supported_until(curve_id):
    """Return the list of bar IDs that must be assembled before *curve_id*
    is considered stable (empty list if the key is unset)."""
    return _split_bar_id_list(rs.GetUserText(curve_id, BAR_SUPPORTED_UNTIL_KEY))


def set_supported_until(curve_id, bar_ids):
    """Persist *bar_ids* (list of bar-id strings) on *curve_id*.  Pass an
    empty list to clear the requirement."""
    cleaned = [bid for bid in (bar_ids or []) if bid]
    if cleaned:
        rs.SetUserText(curve_id, BAR_SUPPORTED_UNTIL_KEY, ",".join(cleaned))
    else:
        # Setting to empty string removes the key from the UserText dict.
        rs.SetUserText(curve_id, BAR_SUPPORTED_UNTIL_KEY, "")


def is_fake_bar(curve_id):
    """True when *curve_id* is staging that will not be fabricated.

    See :data:`BAR_IS_FAKE_KEY`.  Marked in ``RSBarEdit > FakeBar``.
    """
    return rs.GetUserText(curve_id, BAR_IS_FAKE_KEY) == "1"


def set_fake_bar(curve_id, fake):
    """Mark or unmark *curve_id* as a non-fabricated staging bar."""
    rs.SetUserText(curve_id, BAR_IS_FAKE_KEY, "1" if fake else "")


def get_fake_bar_ids(bar_map=None):
    """Return the set of bar ids currently marked fake.

    *bar_map* is a :func:`get_bar_seq_map` result; one is fetched when omitted.
    Returned as ids, not oids, because every consumer -- the export filters, the
    sequence display, the joint pass -- works in bar ids.
    """
    if bar_map is None:
        bar_map = get_bar_seq_map()
    return {
        bar_id for bar_id, (oid, _seq) in bar_map.items() if is_fake_bar(oid)
    }


def cleanup_stale_supports():
    """Remove dangling bar-id refs from every ``supported_until`` list.

    A reference is *stale* if no registered bar carries that ``bar_id``
    in its UserText.  Stale refs are dangerous because:

    * They make the dependent bar perpetually unstable in our analysis
      (the awaited bar will never appear).
    * If the bar id later gets reused for a new bar, the dependency
      silently rebinds to the wrong bar.

    This function MUST run before any pass that reassigns ``bar_id``
    values (e.g. ``ensure_bar_id`` healing a copy-pasted curve).  It is
    invoked at the top of :func:`repair_bar_sequences` so every standard
    entry-point picks it up automatically via ``repair_on_entry``.

    Returns ``{bar_id: removed_dep_ids}`` for bars whose lists changed.
    """
    live_ids = set()
    bar_records = []  # (oid, bar_id)
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE:
            continue
        bid = rs.GetUserText(oid, BAR_ID_KEY)
        bar_records.append((oid, bid))
        if bid:
            live_ids.add(bid)

    changed = {}
    for oid, bid in bar_records:
        deps = get_supported_until(oid)
        if not deps:
            continue
        kept = [d for d in deps if d in live_ids]
        if len(kept) != len(deps):
            removed = [d for d in deps if d not in live_ids]
            set_supported_until(oid, kept)
            changed[bid or str(oid)] = removed
    return changed


def get_unstable_bars(active_bar_id, bar_map=None):
    """Return the set of bar IDs that are unstable *at the moment* the
    active step is being assembled.

    A built bar (``seq < active_seq``) is unstable if its
    ``supported_until`` list contains any bar whose sequence is **>=**
    ``active_seq`` -- i.e. the stabilising bar has not yet finished being
    assembled (the active bar itself only releases supports *after* its
    own assembly completes, so dependencies on the active bar still
    count as unstable during this step).

    Returns an empty set if *active_bar_id* is unknown.
    """
    if bar_map is None:
        bar_map = get_bar_seq_map()
    if active_bar_id not in bar_map:
        return set()
    _, active_seq = bar_map[active_bar_id]
    unstable = set()
    for bar_id, (oid, seq) in bar_map.items():
        if seq >= active_seq:
            continue
        for dep_id in get_supported_until(oid):
            dep_entry = bar_map.get(dep_id)
            if dep_entry is None:
                # Stale dep (bar was deleted) -- treat as still unsatisfied.
                # Note: ``repair_bar_sequences`` now scrubs these on entry,
                # so this branch should only fire mid-session before the
                # next repair pass.
                unstable.add(bar_id)
                break
            if dep_entry[1] >= active_seq:
                unstable.add(bar_id)
                break
    return unstable


def collect_hold_inputs(bar_map=None):
    """Read the two inputs ``core.hold_schedule.derive_hold_plan`` needs.

    The one Rhino-side gateway into the (Rhino-free) hold derivation, so
    every consumer — the IK button, the exporters, the schedule builder —
    reads the SAME document data the same way.

    Args:
        bar_map (dict): a :func:`get_bar_seq_map` result; fetched when omitted.

    Returns:
        tuple: ``(bar_seq, supported_until)`` where ``bar_seq`` is
        ``{bar_id: step int}`` and ``supported_until`` is
        ``{bar_id: [stabilizing bar ids]}`` (only bars with a non-empty list).
    """
    if bar_map is None:
        bar_map = get_bar_seq_map()
    bar_seq = {bar_id: seq for bar_id, (_oid, seq) in bar_map.items()}
    supported_until = {}
    for bar_id, (oid, _seq) in bar_map.items():
        deps = get_supported_until(oid)
        if deps:
            supported_until[bar_id] = deps
    return bar_seq, supported_until


# ---------------------------------------------------------------------------
# Sequence colour-coding and visibility
# ---------------------------------------------------------------------------

#: Colour for bars that have already been assembled (earlier in sequence).
SEQ_COLOR_BUILT = (60, 179, 60)  # green — already assembled
#: Colour for the bar currently being assembled (active step).
SEQ_COLOR_ACTIVE = (30, 100, 220)  # blue — current step
#: Colour for bars not yet assembled (later in sequence).
SEQ_COLOR_UNBUILT = (160, 160, 160)  # grey — not yet assembled
#: Colour for built bars that still need temporary support (teal — between
#: built-green and active-blue).
SEQ_COLOR_UNSTABLE = (40, 170, 160)
#: Colour used to overlay bars currently selected as supports inside the
#: ``EditSupports`` toggle-pick mode (dark purple).
SEQ_COLOR_SUPPORT_PICK = (110, 40, 160)
#: Non-fabricated staging bar (pink) -- the same pink as :data:`COLOR_FAILED`:
#: both mean "the robot will not be building this one".
SEQ_COLOR_FAKE = (230, 115, 150)


def _set_obj_color(oid, color):
    """Set by-object colour on *oid*."""
    rs.ObjectColorSource(oid, 1)  # 1 = by object
    rs.ObjectColor(oid, color)


def _reset_obj_color(oid):
    """Restore by-layer colour on *oid*."""
    rs.ObjectColorSource(oid, 0)  # 0 = by layer


def _bar_curve_and_tube(curve_id, tube_index=None):
    """Return ``[curve_id]`` plus the tube GUID if one exists.

    *tube_index* is an optional ``{axis_guid_str: tube_oid}`` map from
    :func:`_tube_index`.  Pass one whenever looping over many bars:
    :func:`_find_existing_tube` rescans the entire tube-preview layer on every
    call, so without an index a pass over N bars costs N layer scans.
    """
    ids = [curve_id]
    if tube_index is None:
        tube = _find_existing_tube(curve_id)
    else:
        tube = tube_index.get(str(rs.coerceguid(curve_id)))
    if tube is not None:
        ids.append(tube)
    return ids


# ---------------------------------------------------------------------------
# Joint / tool lookup by parent bar
# ---------------------------------------------------------------------------


def _joint_layer_objects():
    """All joint block instance ids on the female + male + ground layers."""
    out = []
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if rs.IsLayer(layer):
            out.extend(rs.ObjectsByLayer(layer) or [])
    return out


def _tool_layer_objects():
    """All robotic-tool block instance ids on the tool-instances layer."""
    layer = config.LAYER_TOOL_INSTANCES
    if not rs.IsLayer(layer):
        return []
    return list(rs.ObjectsByLayer(layer) or [])


def get_active_tool_oids(active_bar_id):
    """Return tool block instance ids whose male joint is parented to *active_bar_id*.

    A robotic tool is conceptually 'used to assemble' the bar that owns
    its male joint -- so the active tool for a given step is the tool
    whose joint's male side has ``parent_bar_id == active_bar_id``.
    """
    if not active_bar_id:
        return []
    active_joint_ids = set()
    for layer in (
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            if (
                rs.GetUserText(oid, "parent_bar_id") == active_bar_id
                and rs.GetUserText(oid, "joint_id")
            ):
                active_joint_ids.add(rs.GetUserText(oid, "joint_id"))
    if not active_joint_ids:
        return []
    return [
        oid
        for oid in _tool_layer_objects()
        if rs.GetUserText(oid, "joint_id") in active_joint_ids
    ]


def _set_visible(oid, visible):
    if visible:
        rs.ShowObject(oid)
    else:
        rs.HideObject(oid)


def _ensure_preview_linetype(dash_mm, gap_mm):
    """Return the name of a document linetype drawing ``dash_mm`` on / ``gap_mm`` off.

    The pattern is encoded in the NAME (``RS_PreviewDash_4x2``), so a changed
    pattern simply creates a fresh linetype instead of modifying one in place
    -- Rhino's linetype table has no safe in-place edit while objects use the
    style.  Old patterns linger unused in the table, which is harmless.
    """
    def _num(v):
        return ("%g" % float(v)).replace(".", "_")

    name = f"RS_PreviewDash_{_num(dash_mm)}x{_num(gap_mm)}"
    if sc.doc.Linetypes.FindName(name) is None:
        # Positive segment = ink, negative = gap (Rhino's convention).
        sc.doc.Linetypes.Add(name, [float(dash_mm), -abs(float(gap_mm))])
    return name


def _apply_line_style(oid, line_style):
    """Apply a ``line_style`` dict (see ``show_sequence_colors``) to one curve.

    Both attributes are always written explicitly: a style with
    ``dashed=False`` RESETS the linetype to by-layer rather than leaving a
    stale dash from the previous frame, and likewise for thickness.
    """
    thickness = line_style.get("thickness_mm")
    if thickness:
        rs.ObjectPrintWidth(oid, float(thickness))
        rs.ObjectPrintWidthSource(oid, 1)  # 1 = by object
    else:
        rs.ObjectPrintWidthSource(oid, 0)  # 0 = by layer
    if line_style.get("dashed"):
        dash_mm, gap_mm = line_style.get("pattern") or (4.0, 2.0)
        rs.ObjectLinetype(oid, _ensure_preview_linetype(dash_mm, gap_mm))
        rs.ObjectLinetypeSource(oid, 1)
    else:
        rs.ObjectLinetypeSource(oid, 0)


#: Per-class keys accepted by ``show_sequence_colors(color_flags=...)``.  One
#: key per colour a user can reason about from the legend; the two derived
#: tints are deliberately absent (see the ``color_flags`` docs below).
SEQ_COLOR_CLASSES = ("built", "active", "unbuilt", "support")


def _normalize_color_flags(color_flags):
    """Turn the ``color_flags`` argument into a full ``{class: bool}`` dict.

    ``None`` (the default everywhere in-command) means every class is painted,
    which is the behaviour every caller had before the argument existed.
    Unknown keys are ignored rather than raising: a caller passing a stale key
    should lose that switch, not the whole repaint.
    """
    if color_flags is None:
        return {name: True for name in SEQ_COLOR_CLASSES}
    return {name: bool(color_flags.get(name, True)) for name in SEQ_COLOR_CLASSES}


def show_sequence_colors(active_bar_id, show_unbuilt=True, bar_map=None,
                         highlight_supports=True, color_flags=None,
                         show_fake=True, tint_curves_only=False,
                         geom_built_and_active_only=False, line_style=None):
    """Apply sequence colour-coding + visibility to bars, joints, and tools.

    Bar visibility / colour
        - ``seq < active_seq``  -> green (built), shown.  If the bar is
          unstable *during* this active step -- i.e. its
          ``supported_until`` list contains a bar with
          ``seq >= active_seq`` (the active bar itself counts because it
          only finishes at the end of this step) -- it is teal instead.
        - ``seq == active_seq`` -> blue  (active step), shown.
        - ``seq > active_seq``  -> grey  (unbuilt), shown iff *show_unbuilt*.
        - a bar in the ACTIVE bar's ``supported_until`` list -> purple, and
          **always shown**, overriding all three rules above.  Those bars are
          what hold this step up, so they belong in the normal view;
          EditSupports is only for CHANGING the set.  Forced visible because a
          support later than the active step would otherwise be hidden by
          ``show_unbuilt=False`` -- precisely when you most need to see it.

    Joint visibility
        Each joint instance follows the visibility of its parent bar
        (``parent_bar_id`` UserText).  Joints on built / active bars are
        always shown; joints on unbuilt bars follow *show_unbuilt*.

    Tool visibility
        Only the robotic tool(s) belonging to the *active* step are
        shown -- i.e. tools whose joint's male side is parented to
        *active_bar_id*.  All other tools are hidden.

    This is the **in-command** view and it is deliberately pure: it reads the
    document and paints, and never writes the build stage.  Do not confuse it
    with :func:`apply_build_stage_visibility`, the persistent filter, which
    differs in exactly one rule -- it keeps every *built* bar's tools visible
    instead of only the active step's, so RSSwapRoboticTool and
    RSInspectRoboticTool stay usable once the command has exited.  The latch is
    written only by an explicit user toggle (``RSSequenceEdit > HideUnbuilt``).

    Pass *bar_map* (a :func:`get_bar_seq_map` result) when the caller already
    has one -- ``get_bar_seq_map`` walks every object in the document, so an
    interactive caller that repaints and then reads the same numbers back
    should scan once and hand the result down.  Only safe while nothing has
    changed a bar's sequence number since the map was built; leave it ``None``
    after any reorder.

    Args:
        active_bar_id (str): the bar being assembled at this step.
        show_unbuilt (bool): show bars later in the sequence than the active one.
        bar_map (dict | None): a cached :func:`get_bar_seq_map` result.
        highlight_supports (bool): paint the active bar's declared supports purple.
        color_flags (dict | None): per-class tint switches, keys
            ``"built"`` / ``"active"`` / ``"unbuilt"`` / ``"support"``.  A False
            entry leaves that class **by-layer** -- the tint is dropped, the
            object stays exactly as visible as the rules above make it.
            ``None`` (the default) paints everything, so every in-command caller
            is unaffected.  Written for the Grasshopper preview component, where
            the animator turns individual legend colours off to film a clean
            frame; the interactive commands have no use for it.

            Two tints are deliberately not switchable.  ``SEQ_COLOR_UNSTABLE``
            (teal) is a *variant of built*, not a class of its own, so it rides
            on ``"built"``.  ``SEQ_COLOR_FAKE`` (pink) is always painted when
            the bar is visible, matching :func:`clear_ik_preview`, which
            re-asserts it rather than resetting it: a staging bar that renders
            like a real one is a fabrication error waiting to happen.  The
            filming view does not silence the tint -- it hides the bar
            entirely via *show_fake*.
        show_fake (bool): False hides fake (staging) bars outright -- their
            joints follow automatically.  True (the default, every in-command
            caller) keeps today's behaviour: visible per the sequence rules,
            tinted pink.  For the Grasshopper filming view, where a staging
            bar has no business on camera.
        tint_curves_only (bool): True puts the class colours on the bar
            CENTERLINE CURVES only; tubes, joint instances and tools keep
            their normal by-layer look.  The filming split: coloured guide
            lines over an uncoloured model.  False (default) tints everything,
            as before.
        geom_built_and_active_only (bool): True shows tube + joint geometry
            only for built bars and the active bar; bars later in the
            sequence keep at most their centerline (per *show_unbuilt*).
            False (default) shows unbuilt geometry per *show_unbuilt*, as
            before.
        line_style (dict | None): styling for the centerline curves, applied
            per bar:
            ``{"thickness_mm": float | None, "dashed": bool,
            "pattern": (dash_mm, gap_mm)}``.
            Thickness is a per-object print width (viewport shows it only
            while PrintDisplay is on -- the GH component manages that);
            ``dashed`` swaps the curve onto a document linetype built from
            ``pattern`` (default 4 mm ink / 2 mm gap).  ``None`` (default)
            touches neither attribute.  Reset by
            :func:`reset_sequence_colors`.
    """
    if bar_map is None:
        bar_map = get_bar_seq_map()
    if active_bar_id not in bar_map:
        return
    flags = _normalize_color_flags(color_flags)
    active_oid, active_seq = bar_map[active_bar_id]
    unstable_ids = get_unstable_bars(active_bar_id, bar_map)

    # The active bar's declared supports, shown as part of the normal view --
    # you should not have to open EditSupports to see what is holding this step
    # up.  Forced visible even when they are later than the active step and
    # unbuilt bars are hidden: a support you cannot see is the one case where
    # hiding actively misleads.  EditSupports is now only for changing the set.
    # The active bar is dropped so it always keeps its own ACTIVE colour.
    # Pass highlight_supports=False when the caller paints the purple itself --
    # the EditSupports picker does, because it shows the set being EDITED, which
    # differs from the saved set the moment the user toggles anything.  Two
    # sources of purple would leave a de-selected bar still looking selected.
    support_ids = set()
    if highlight_supports:
        support_ids = {
            b for b in get_supported_until(active_oid)
            if b in bar_map and b != active_bar_id
        }

    # Staging bars are tinted but keep the normal visibility rules: one that is
    # later than the active step genuinely is not standing yet, so hiding it is
    # right.  The colour is the point -- it stops a fake bar being read as part
    # of the structure being built.
    fake_ids = get_fake_bar_ids(bar_map)

    # Built once here and reused by the joint pass below, so the two can never
    # disagree about what colour a bar is -- nor about whether it is painted at
    # all, which is why the ``paint`` decision is recorded alongside the colour.
    bar_visible_by_id = {}
    bar_color_by_id = {}
    bar_paint_by_id = {}

    # One tube-layer scan for the whole pass (see _tube_index).
    tube_index = _tube_index()

    rs.EnableRedraw(False)
    for bar_id, (oid, seq) in bar_map.items():
        if seq < active_seq:
            color = SEQ_COLOR_UNSTABLE if bar_id in unstable_ids else SEQ_COLOR_BUILT
            visible = True
            paint = flags["built"]
        elif seq == active_seq:
            color = SEQ_COLOR_ACTIVE
            visible = True
            paint = flags["active"]
        else:
            color = SEQ_COLOR_UNBUILT
            visible = show_unbuilt
            paint = flags["unbuilt"]
        # Precedence, loosest to tightest: sequence state, then fake, then
        # support.  The active bar keeps its blue either way -- it is excluded
        # from support_ids, and skipped here.  ``paint`` follows the same
        # precedence as ``color``: whichever rule wins the colour also decides
        # whether it is applied, so a class switched off can never inherit
        # another class's tint.
        if bar_id in fake_ids and bar_id != active_bar_id:
            color = SEQ_COLOR_FAKE
            paint = True  # never silenced while visible -- see the docstring
            if not show_fake:
                visible = False  # the filming view: staging bars off camera
        if bar_id in support_ids:
            color = SEQ_COLOR_SUPPORT_PICK
            visible = True
            paint = flags["support"]
        # The curve follows the LINE rules computed above; the tube (and, via
        # the maps below, the joints) follows the GEOMETRY rules -- identical
        # by default, restricted to built + active bars in the filming view.
        geom_visible = visible and (
            not geom_built_and_active_only or seq <= active_seq
        )
        geom_paint = paint and not tint_curves_only
        bar_visible_by_id[bar_id] = geom_visible
        bar_color_by_id[bar_id] = color
        bar_paint_by_id[bar_id] = geom_paint
        objs = _bar_curve_and_tube(oid, tube_index)
        curve_obj = objs[0]
        if paint:
            _set_obj_color(curve_obj, color)
        else:
            _reset_obj_color(curve_obj)
        _set_visible(curve_obj, visible)
        if line_style is not None:
            _apply_line_style(curve_obj, line_style)
        for obj in objs[1:]:
            if geom_paint:
                _set_obj_color(obj, color)
            else:
                _reset_obj_color(obj)
            _set_visible(obj, geom_visible)

    # Joints follow their parent bar's visibility AND color.  Setting
    # by-object color on the block instance lets nested sub-objects that
    # are set to "by parent" inherit it automatically.
    for joint_oid in _joint_layer_objects():
        parent_bar_id = rs.GetUserText(joint_oid, "parent_bar_id")
        visible = bar_visible_by_id.get(parent_bar_id, True)
        color = bar_color_by_id.get(parent_bar_id)
        if color is not None:
            # Follow the parent bar's paint decision, not just its colour, so a
            # switched-off class does not leave its joints tinted.
            if bar_paint_by_id.get(parent_bar_id, True):
                _set_obj_color(joint_oid, color)
            else:
                _reset_obj_color(joint_oid)
        _set_visible(joint_oid, visible)

    # Tools: hide all, then show + color the active step's tool(s).  The tint is
    # the ACTIVE blue, so it is gated on the same switch as the active bar --
    # visibility is not, the active step's tool is shown either way.
    active_tool_oids = set(get_active_tool_oids(active_bar_id))
    for tool_oid in _tool_layer_objects():
        is_active = tool_oid in active_tool_oids
        if is_active:
            if flags["active"] and not tint_curves_only:
                _set_obj_color(tool_oid, SEQ_COLOR_ACTIVE)
            else:
                _reset_obj_color(tool_oid)
        _set_visible(tool_oid, is_active)

    rs.EnableRedraw(True)


def reset_sequence_colors():
    """Restore default (by-layer) colour, then re-assert the build-stage filter.

    Colours always go back to by-layer and everything is shown again -- but if the
    document carries a build stage (:func:`get_build_stage`), the last thing this
    does is hide the unbuilt parts again.  That is what makes ``HideUnbuilt``
    survive Esc: this function is the shared exit path of RSSequenceEdit,
    RSShowBarActionPlan and RSIKKeyframe, so all three honour the latch without a
    per-caller edit.

    There is deliberately no "show everything anyway" switch.  An earlier draft
    had one for ``clear_build_stage`` to call, but that caller repaints straight
    afterwards, so the option was left with no users -- an untested branch whose
    only effect would be to silently defeat this feature.  Code that genuinely
    wants everything visible should clear the latch first.
    """
    bar_map = get_bar_seq_map()
    tube_index = _tube_index()
    with suspend_redraw():
        for bar_id, (oid, _) in bar_map.items():
            for obj in _bar_curve_and_tube(oid, tube_index):
                _reset_obj_color(obj)
                # Undo any filming line style (width / dash back to by-layer);
                # harmless on objects that never carried one.
                rs.ObjectLinetypeSource(obj, 0)
                rs.ObjectPrintWidthSource(obj, 0)
                rs.ShowObject(obj)
        for joint_oid in _joint_layer_objects():
            _reset_obj_color(joint_oid)
            rs.ShowObject(joint_oid)
        for tool_oid in _tool_layer_objects():
            _reset_obj_color(tool_oid)
            rs.ShowObject(tool_oid)
        # LAST, inside the same batch: the show pass above just made everything
        # visible, so the filter has to get the final word or the latch leaks.
        apply_build_stage_visibility(verbose=False)


# ---------------------------------------------------------------------------
# Build stage -- the persistent "hide everything after step N" filter
# ---------------------------------------------------------------------------
#
# The saved value and the rule for interpreting it live in ``core.build_stage``
# (Rhino-free, so pytest can cover the awkward renumber/delete cases).  What is
# here is the part that needs a document: read/write the key, and do the hiding.
#
# The invariant everything else depends on: ``apply_build_stage_visibility`` is
# the LAST visibility-touching step of any command or refresh pass.  Anything
# that shows objects afterwards re-opens the leak this feature exists to close.


def get_build_stage():
    """Return the saved ``(bar_id, seq)`` build stage, or ``None`` if not latched.

    Deliberately a bare key read with **no document scan**: this runs on every
    ``repair_on_entry`` even when the filter is off, so the unlatched cost must
    stay at one string read.  The *seq* returned here is the one saved at the
    time -- use :func:`apply_build_stage_visibility` (which re-resolves through
    ``core.build_stage``) when the bar may since have been renumbered or deleted.
    """
    return parse_build_stage(get_doc_string(BUILD_STAGE_KEY))


def set_build_stage(bar_id, seq):
    """Latch the filter at *bar_id* / assembly step *seq*."""
    set_doc_string(BUILD_STAGE_KEY, format_build_stage(bar_id, seq))


def clear_build_stage():
    """Un-latch the filter.  Clears the key only -- it changes no visibility.

    Every caller repaints immediately afterwards (``show_sequence_colors`` or
    ``reset_sequence_colors``), so hiding/showing here as well would only paint
    twice.
    """
    set_doc_string(BUILD_STAGE_KEY, "")


def apply_build_stage_visibility(caller=None, verbose=True):
    """Hide every bar, joint and tool later than the saved build stage.

    The one function that enforces the latch.  Call it as the **final**
    visibility-touching statement of any command or refresh pass.

    Returns ``(bar_id, n_hidden)``, or ``None`` when the filter is not latched
    (or resolved to "switch off", in which case the key is wiped and the reason
    printed).

    Visibility only -- never colour.  ``show_sequence_colors`` owns the
    green/blue/grey palette and still resets on exit; only the hidden state
    persists.

    The rules, which differ from ``show_sequence_colors`` in one place:

    - **bars + tubes**: hidden when ``seq > stage_seq``.
    - **joints**: per block half, following that half's own ``parent_bar_id`` --
      so a joint between built B2 and unbuilt B9 keeps its female half visible
      and hides its male half.
    - **tools**: hidden only when the bar owning the tool's joint is unbuilt.
      ``show_sequence_colors`` instead shows *only the active step's* tool; here
      every built bar keeps its tools, which is what leaves RSSwapRoboticTool and
      RSInspectRoboticTool usable while the filter is on.
    """
    raw = get_doc_string(BUILD_STAGE_KEY)
    if not raw:
        return None  # not latched -- one string read and out

    bar_map = get_bar_seq_map()
    status, stage_seq, stage_bar_id, message = resolve_build_stage_seq(raw, bar_map)
    if status == STATUS_OFF:
        return None
    if status == STATUS_CLEAR:
        # The staged bar is gone beyond rescue.  Drop the latch, show everything
        # and say why -- an empty-looking model with no explanation is exactly
        # what this feature must not produce.  Visibility only, so the caller's
        # colour scheme (if any) survives; and not via reset_sequence_colors,
        # which calls back into this function.
        clear_build_stage()
        print(f"{caller or 'RSScaffolding'}: {message}")
        with suspend_redraw():
            tube_index = _tube_index()
            for _bar_id, (oid, _seq) in bar_map.items():
                for obj in _bar_curve_and_tube(oid, tube_index):
                    rs.ShowObject(obj)
            for oid in _joint_layer_objects() + _tool_layer_objects():
                rs.ShowObject(oid)
        return None
    if status == STATUS_FALLBACK:
        # Re-point the key at the surviving bar so it stops drifting further with
        # every subsequent delete.
        set_build_stage(stage_bar_id, stage_seq)
        print(f"{caller or 'RSScaffolding'}: {message}")

    # Staging bars are exempt: they are put up by hand before the robot starts,
    # so they are standing at every build stage.  Hiding one would also hide the
    # joints the next real bar mates to, which is the opposite of useful.
    fake_ids = get_fake_bar_ids(bar_map)

    n_hidden = 0
    with suspend_redraw():
        # Bars + their tubes.  One tube-layer scan for the whole pass.
        tube_index = _tube_index()
        bar_visible_by_id = {}
        for bar_id, (oid, seq) in bar_map.items():
            visible = seq <= stage_seq or bar_id in fake_ids
            bar_visible_by_id[bar_id] = visible
            for obj in _bar_curve_and_tube(oid, tube_index):
                _set_visible(obj, visible)
            if not visible:
                n_hidden += 1

        # Joints, one pass over the three layers.  Iterated per layer rather than
        # through _joint_layer_objects() because we need to know WHICH layer each
        # instance came from: only the male/ground halves answer "whose tool is
        # this?" (see get_active_tool_oids), and the female half would give the
        # opposite answer for the same joint_id.
        tool_owner_bar_by_joint_id = {}
        for layer in (
            config.LAYER_JOINT_FEMALE_INSTANCES,
            config.LAYER_JOINT_MALE_INSTANCES,
            config.LAYER_JOINT_GROUND_INSTANCES,
        ):
            if not rs.IsLayer(layer):
                continue
            is_tool_side = layer != config.LAYER_JOINT_FEMALE_INSTANCES
            for joint_oid in rs.ObjectsByLayer(layer) or []:
                parent_bar_id = rs.GetUserText(joint_oid, "parent_bar_id")
                # Unknown parent -> leave visible.  An orphaned joint that is also
                # invisible is one the user can never find and fix.
                visible = bar_visible_by_id.get(parent_bar_id, True)
                _set_visible(joint_oid, visible)
                if not visible:
                    n_hidden += 1
                if is_tool_side:
                    joint_id = rs.GetUserText(joint_oid, "joint_id")
                    if joint_id:
                        tool_owner_bar_by_joint_id[joint_id] = parent_bar_id

        # Tools follow the bar owning their male/ground half.
        for tool_oid in _tool_layer_objects():
            owner_bar_id = tool_owner_bar_by_joint_id.get(
                rs.GetUserText(tool_oid, "joint_id")
            )
            visible = bar_visible_by_id.get(owner_bar_id, True)
            _set_visible(tool_oid, visible)
            if not visible:
                n_hidden += 1

    if verbose:
        print(stage_filter_note(caller, stage_bar_id, stage_seq))
    return (stage_bar_id, n_hidden)


def hide_if_beyond_build_stage(object_ids, curve_id):
    """Hide freshly created *object_ids* if their bar is later than the stage.

    A newly created Rhino object is always visible, so anything baked outside a
    repair pass would pop back on screen and make the filter look broken -- most
    visibly a tube floating over its own hidden centerline, since the tube is
    what you actually see.

    Kept deliberately cheap, because it sits on a creation path: one document
    string read (nothing at all when the filter is off), then the bar's own
    ``bar_seq`` user text against the **saved** stage number.  No
    ``get_bar_seq_map()``, no re-resolution -- if the saved number has drifted
    (renumber, deleted stage bar) the next ``repair_on_entry`` corrects it with a
    full :func:`apply_build_stage_visibility`.

    A brand-new bar has no ``bar_seq`` until ``repair_bar_sequences()`` runs, so
    it stays visible for the rest of the command that created it -- intended:
    RSCreateBar warns about it instead, rather than hiding the work you just did.
    """
    stage = get_build_stage()
    if stage is None:
        return
    _stage_bar_id, stage_seq = stage
    seq = _parse_bar_seq(rs.GetUserText(curve_id, BAR_SEQ_KEY))
    if seq is None or seq <= stage_seq:
        return
    for oid in as_object_id_list(object_ids):
        _set_visible(oid, False)


# There is deliberately no per-tool equivalent of the above.  Every path that
# creates a robotic tool already ends with an apply: the three batch routines in
# ``rhino_tool_place`` are reached only from RSUpdatePreview, and
# ``replace_all_tool_instances`` only from RSSwapRoboticTool -- both of which end
# with ``apply_build_stage_visibility``.  Any other tool lands on a just-picked,
# therefore visible, bar.  A per-tool hook could never fire on anything those two
# do not already cover.


# ---------------------------------------------------------------------------
# Tube-preview helpers
# ---------------------------------------------------------------------------


def _format_point(arr):
    """Serialise a 3-element array to ``'x,y,z'`` with 6 decimal places."""
    return f"{float(arr[0]):.6f},{float(arr[1]):.6f},{float(arr[2]):.6f}"


def _parse_cached_point(s):
    """Parse ``'x,y,z'`` back to a 3-tuple of floats, or None on failure."""
    if not s:
        return None
    try:
        parts = s.split(",")
        if len(parts) != 3:
            return None
        return tuple(float(p) for p in parts)
    except (ValueError, TypeError):
        return None


def _tube_index():
    """Return ``{axis_guid_str: tube_oid}`` for the whole tube-preview layer.

    :func:`_find_existing_tube` answers "which tube belongs to this curve?" by
    scanning the whole layer, so asking it once per bar rescans the layer once
    per bar (N bars x N tubes user-text reads).  This builds the same
    curve->tube mapping in ONE scan; callers looping over every bar then look
    each tube up by GUID, which is a dict hash rather than another scan.

    First tube wins on a duplicate ``tube_axis_id``, matching
    :func:`_find_existing_tube`'s first-match return (``repair_on_entry`` purges
    the duplicates that copy/paste leaves behind).
    """
    index = {}
    if not rs.IsLayer(TUBE_LAYER):
        return index
    for oid in rs.ObjectsByLayer(TUBE_LAYER) or []:
        axis_guid = rs.GetUserText(oid, TUBE_AXIS_GUID_KEY)
        if axis_guid:
            index.setdefault(axis_guid, oid)
    return index


def _find_existing_tube(curve_id):
    """Find a tube on *TUBE_LAYER* whose ``tube_axis_id`` matches *curve_id*.

    Returns the tube object GUID or None.
    """
    if not rs.IsLayer(TUBE_LAYER):
        return None
    curve_guid_str = str(rs.coerceguid(curve_id))
    for oid in rs.ObjectsByLayer(TUBE_LAYER):
        if rs.GetUserText(oid, TUBE_AXIS_GUID_KEY) == curve_guid_str:
            return oid
    return None


def _tube_geometry_matches(tube_id, curve_id):
    """Return True if the tube's cached endpoints match the current curve."""
    cached_start = _parse_cached_point(rs.GetUserText(tube_id, TUBE_CACHE_START))
    cached_end = _parse_cached_point(rs.GetUserText(tube_id, TUBE_CACHE_END))
    if cached_start is None or cached_end is None:
        return False
    cur_start = point_to_array(rs.CurveStartPoint(curve_id))
    cur_end = point_to_array(rs.CurveEndPoint(curve_id))
    start_ok = np.linalg.norm(np.array(cached_start) - cur_start) < _POINT_TOL
    end_ok = np.linalg.norm(np.array(cached_end) - cur_end) < _POINT_TOL
    return start_ok and end_ok


def _create_tube_brep(start_xyz, end_xyz, bar_radius):
    """Build a capped cylinder Brep between two endpoints.  Returns GUID or None."""
    axis_vector = end_xyz - start_xyz
    axis_length = float(np.linalg.norm(axis_vector))
    if axis_length <= 1e-9:
        return None
    axis_direction = axis_vector / axis_length
    base_plane = Rhino.Geometry.Plane(
        Rhino.Geometry.Point3d(*start_xyz.tolist()),
        Rhino.Geometry.Vector3d(*axis_direction.tolist()),
    )
    cylinder = Rhino.Geometry.Cylinder(
        Rhino.Geometry.Circle(base_plane, float(bar_radius)),
        axis_length,
    )
    brep = cylinder.ToBrep(True, True)
    if brep is None:
        return None
    tube_id = sc.doc.Objects.AddBrep(brep)
    return tube_id


def ensure_bar_preview(curve_id, bar_radius, color=None, bar_id=None,
                       verbose=False):
    """Make sure a tube preview exists and matches the current curve geometry.

    Creates or regenerates the tube as needed.  Returns
    ``(baked_ids, status)`` where ``status`` is one of ``"reused"``,
    ``"regenerated"``, or ``"created"``.  Pre-existing single-value
    callers that ignore the return are unaffected.

    A newly baked tube is hidden immediately if its bar is beyond the build
    stage (:func:`hide_if_beyond_build_stage`).  RSCreateBar, RSBarEdit,
    RSBarSnap, RSBarBrace and RSBarSubfloor all call this outside any repair
    pass -- and the snap/brace/subfloor trio regenerate tubes for *existing*
    bars after trimming, so without the hide you get a visible tube floating
    over a hidden centerline.  The ``"reused"`` path needs nothing: it hands
    back an object whose hidden state the last filter pass already set.
    """
    # Check for an existing tube
    existing = _find_existing_tube(curve_id)
    if existing is not None:
        if _tube_geometry_matches(existing, curve_id):
            if verbose:
                print(f"  ensure_bar_preview[{bar_id or '?'}]: reused tube {existing}")
            return [existing], "reused"
        # Stale — delete and recreate
        if verbose:
            cached_start = _parse_cached_point(rs.GetUserText(existing, TUBE_CACHE_START))
            cached_end = _parse_cached_point(rs.GetUserText(existing, TUBE_CACHE_END))
            cur_start = tuple(point_to_array(rs.CurveStartPoint(curve_id)).tolist())
            cur_end = tuple(point_to_array(rs.CurveEndPoint(curve_id)).tolist())
            print(f"  ensure_bar_preview[{bar_id or '?'}]: stale tube {existing} -- "
                  f"cached_start={cached_start} cur_start={cur_start} | "
                  f"cached_end={cached_end} cur_end={cur_end}")
        delete_objects([existing])
        status = "regenerated"
    else:
        status = "created"
        if verbose:
            print(f"  ensure_bar_preview[{bar_id or '?'}]: no existing tube, creating new")

    start_xyz, end_xyz = curve_endpoints(curve_id)
    tube_id = _create_tube_brep(start_xyz, end_xyz, bar_radius)
    if tube_id is None:
        return [], "reused"

    # Resolve bar_id
    if bar_id is None:
        bar_id = rs.GetUserText(curve_id, BAR_ID_KEY) or "?"

    # Display setup
    label = f"{bar_id}_tube"
    baked_ids = apply_object_display(tube_id, label, color=color, layer_name=TUBE_LAYER)

    # User text metadata
    curve_guid_str = str(rs.coerceguid(curve_id))
    for oid in baked_ids:
        rs.SetUserText(oid, TUBE_AXIS_GUID_KEY, curve_guid_str)
        rs.SetUserText(oid, TUBE_BAR_ID_KEY, bar_id)
        rs.SetUserText(oid, BAR_RADIUS_KEY, f"{bar_radius:.6f}")
        rs.SetUserText(oid, TUBE_CACHE_START, _format_point(start_xyz))
        rs.SetUserText(oid, TUBE_CACHE_END, _format_point(end_xyz))
        # Store the tube's own GUID so we can later detect copy/paste
        # duplicates: a copy will retain this user text but live under a
        # different object GUID.
        rs.SetUserText(oid, TUBE_SELF_GUID_KEY, str(rs.coerceguid(oid)))

    hide_if_beyond_build_stage(baked_ids, curve_id)
    return baked_ids, status


def update_all_previews(bar_radius, color=None, verbose=False):
    """Ensure every registered bar has an up-to-date tube preview.

    Returns the number of bars whose tube was actually created or
    regenerated (i.e. excludes bars whose existing tube was reused
    as-is).  Pass ``verbose=True`` to log per-bar status (handy when
    debugging stale-preview issues).
    """
    bars = get_all_bars()
    n_created = 0
    n_regen = 0
    n_reused = 0
    for bar_id, guid in bars.items():
        _, status = ensure_bar_preview(
            guid, bar_radius, color=color, bar_id=bar_id, verbose=verbose
        )
        if status == "created":
            n_created += 1
        elif status == "regenerated":
            n_regen += 1
        else:
            n_reused += 1
    if verbose:
        print(f"update_all_previews: total={len(bars)} "
              f"created={n_created} regenerated={n_regen} reused={n_reused}")
    return n_created + n_regen


# ---------------------------------------------------------------------------
# Managed-layer enforcement
# ---------------------------------------------------------------------------


def _move_to_default_layer(object_ids, *, source_layer, caller):
    """Move *object_ids* to ``config.DEFAULT_LAYER``, creating it if needed,
    and print a warning naming the source layer and count."""
    ids = [oid for oid in as_object_id_list(object_ids) if rs.IsObject(oid)]
    if not ids:
        return 0
    ensure_layer(config.DEFAULT_LAYER)
    for oid in ids:
        rs.ObjectLayer(oid, config.DEFAULT_LAYER)
    print(
        f"{caller} (warning): moved {len(ids)} stray object(s) "
        f"from '{source_layer}' to '{config.DEFAULT_LAYER}'."
    )
    return len(ids)


def _is_managed_tube(oid, registered_curve_guids):
    """A tube is *ours* iff it points at a registered bar curve."""
    axis_guid = rs.GetUserText(oid, TUBE_AXIS_GUID_KEY)
    return bool(axis_guid) and axis_guid in registered_curve_guids


def _enforce_tube_layer(caller):
    """Delete orphaned tube previews and evict strays on the tube layer.

    A tube on the managed tube layer is considered an *orphan* (and is
    deleted) if any of the following hold:

    - its ``tube_axis_id`` user text does not point at a currently
      registered bar curve (e.g. the bar was deleted, or the tube was
      copied along with a fresh bar that got reassigned);
    - its stored ``tube_self_guid`` does not match its actual object GUID
      (i.e. the user manually duplicated the tube and the copy now carries
      the same user text as the original);
    - its Rhino ObjectName is identical to that of an earlier tube on the
      same layer (duplicate names are treated as copies; the first
      occurrence wins, the rest are orphans).

    A *stray* is an object on the tube layer with no ``tube_axis_id`` and
    no ``tube_self_guid`` user text (the user drew something there
    manually); strays are moved to ``config.DEFAULT_LAYER``.
    """
    if not rs.IsLayer(TUBE_LAYER):
        return
    registered = {str(g) for g in get_all_bars().values()}
    orphans = []
    strays = []
    seen_names = {}
    for oid in rs.ObjectsByLayer(TUBE_LAYER) or []:
        axis_guid = rs.GetUserText(oid, TUBE_AXIS_GUID_KEY)
        self_guid = rs.GetUserText(oid, TUBE_SELF_GUID_KEY)
        actual_guid = str(rs.coerceguid(oid))
        name = rs.ObjectName(oid)

        if not axis_guid and not self_guid and not name:
            strays.append(oid)
            continue

        is_orphan = False
        if axis_guid and axis_guid not in registered:
            is_orphan = True
        elif self_guid and self_guid != actual_guid:
            # Object was duplicated by the user; the copy still claims to
            # be the original tube via user text but has a new object GUID.
            is_orphan = True
        elif name and name in seen_names:
            # A later tube with the same ObjectName as an earlier one is
            # treated as a duplicate copy.
            is_orphan = True

        if is_orphan:
            orphans.append(oid)
        else:
            if name:
                seen_names[name] = oid

    if orphans:
        delete_objects(orphans)
        print(
            f"{caller} (warning): deleted {len(orphans)} orphaned tube preview(s) "
            f"on '{TUBE_LAYER}' (no matching bar, copy-pasted, or duplicate name)."
        )
    _move_to_default_layer(strays, source_layer=TUBE_LAYER, caller=caller)


def _enforce_centerline_layer(caller):
    """Evict objects on the centerline layer that are not registered bars."""
    layer = BAR_CENTERLINE_LAYER
    if not rs.IsLayer(layer):
        return
    strays = [
        oid
        for oid in (rs.ObjectsByLayer(layer) or [])
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE
    ]
    _move_to_default_layer(strays, source_layer=layer, caller=caller)


def _enforce_joint_layer(caller, layer):
    """Evict objects on a joint-instances layer that lack a ``joint_id``."""
    if not rs.IsLayer(layer):
        return
    strays = [
        oid
        for oid in (rs.ObjectsByLayer(layer) or [])
        if not rs.GetUserText(oid, "joint_id")
    ]
    _move_to_default_layer(strays, source_layer=layer, caller=caller)


def _enforce_tool_layer(caller, layer):
    """Evict objects on the tool-instances layer that lack a ``tool_id``."""
    if not rs.IsLayer(layer):
        return
    strays = [
        oid
        for oid in (rs.ObjectsByLayer(layer) or [])
        if not rs.GetUserText(oid, "tool_id")
    ]
    _move_to_default_layer(strays, source_layer=layer, caller=caller)


def enforce_managed_layers(caller="RSScaffolding"):
    """Make sure every managed layer exists, is visible, and contains only
    objects we recognize.  Strays go to ``config.DEFAULT_LAYER`` (also made
    visible).  Run this at the top of every entry-point command before the
    user is prompted."""
    # Ensure the default + all managed layers exist and are visible.
    _JOINT_LAYER_COLORS = {
        config.LAYER_JOINT_MALE_INSTANCES: (105, 105, 105),
        config.LAYER_JOINT_FEMALE_INSTANCES: (230, 230, 230),
        config.LAYER_JOINT_GROUND_INSTANCES: (180, 120, 60),
    }
    ensure_layer(config.DEFAULT_LAYER)
    ensure_layer(config.MANAGED_LAYER_ROOT)
    for layer in config.MANAGED_LAYERS:
        ensure_layer(layer, color=_JOINT_LAYER_COLORS.get(layer))
    # Sweep each managed sublayer.
    _enforce_tube_layer(caller)
    _enforce_centerline_layer(caller)
    _enforce_joint_layer(caller, config.LAYER_JOINT_FEMALE_INSTANCES)
    _enforce_joint_layer(caller, config.LAYER_JOINT_MALE_INSTANCES)
    _enforce_joint_layer(caller, config.LAYER_JOINT_GROUND_INSTANCES)
    _enforce_tool_layer(caller, config.LAYER_TOOL_INSTANCES)


def repair_on_entry(bar_radius, caller="RSScaffolding"):
    """Standard startup repair for bar-registry-aware entry-point scripts.

    Call this at the top of ``main()``, right after reloading config.  It

    1. Ensures all managed layers exist and are visible, evicting any
       stray objects to ``Default`` and deleting orphaned tube previews.
    2. Repairs duplicate / missing bar sequence numbers.  Centerlines are
       the source of truth for bar identity, so this runs before the
       preview pass below.
    3. Updates tube previews for every registered bar.
    4. Sanity-checks that the centerline and tube-preview counts match.
    5. Re-applies the build-stage filter, if the document carries one.

    Step 5 is what makes ``HideUnbuilt`` stick across every command: steps 1 and
    3 both *show* things (``enforce_managed_layers`` force-shows every managed
    layer, and a regenerated tube is born visible), so the filter has to run last
    or the unbuilt parts leak back one command later.
    """
    enforce_managed_layers(caller)
    changed = repair_bar_sequences()
    n = update_all_previews(bar_radius)
    parts = []
    if n:
        parts.append(f"{n} bar preview(s) updated")
    if changed:
        parts.append(f"{len(changed)} sequence number(s) repaired")
    if parts:
        print(f"{caller} (startup): {', '.join(parts)}.")

    # ------------------------------------------------------------------
    # Sanity check: every registered bar centerline should have exactly
    # one corresponding tube preview.
    # ------------------------------------------------------------------
    if rs.IsLayer(BAR_CENTERLINE_LAYER):
        n_centerlines = sum(
            1
            for oid in (rs.ObjectsByLayer(BAR_CENTERLINE_LAYER) or [])
            if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE
        )
    else:
        n_centerlines = 0
    n_tubes = (
        len(rs.ObjectsByLayer(TUBE_LAYER) or [])
        if rs.IsLayer(TUBE_LAYER)
        else 0
    )
    print(
        f"{caller} (sanity): {n_centerlines} bar centerline(s), "
        f"{n_tubes} tube preview(s)."
    )
    if n_centerlines != n_tubes:
        print(
            f"{caller} (warning): centerline/tube count mismatch "
            f"({n_centerlines} vs {n_tubes})."
        )

    # LAST statement on purpose -- see the docstring.  No-ops (one string read)
    # when the document carries no build stage.
    apply_build_stage_visibility(caller=caller, verbose=True)


# ---------------------------------------------------------------------------
# Selection-color helpers (centerline + tube together)
# ---------------------------------------------------------------------------


def paint_bar(curve_id, color):
    """Color a bar centre-line curve and its tube preview the same color.

    Sets the object color source to ``ByObject`` for both, so the override
    is visible regardless of layer color.  Silently no-ops if *curve_id*
    is missing or no tube preview exists yet.
    """
    if curve_id is None or not rs.IsObject(curve_id):
        return
    if hasattr(rs, "ObjectColorSource"):
        rs.ObjectColorSource(curve_id, 1)
    rs.ObjectColor(curve_id, color)
    tube = _find_existing_tube(curve_id)
    if tube is not None and rs.IsObject(tube):
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(tube, 1)
        rs.ObjectColor(tube, color)


def reset_bar_color(curve_id):
    """Restore layer-default color on a bar centre-line curve and its tube."""
    if curve_id is None or not rs.IsObject(curve_id):
        return
    if hasattr(rs, "ObjectColorSource"):
        rs.ObjectColorSource(curve_id, 0)  # by layer
    tube = _find_existing_tube(curve_id)
    if tube is not None and rs.IsObject(tube):
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(tube, 0)


def snapshot_object_colors(object_ids):
    """Capture the color state of any objects so it can be put back exactly.

    :func:`reset_bar_color` (and ``highlight_env._reset_obj_color``) always
    revert to by-layer, which is the wrong "undo" for something that was
    ALREADY carrying a meaning: ``COLOR_HAS_IK`` on a solved bar,
    :data:`SEQ_COLOR_FAKE` on a staging bar, a sequence color on a joint block.
    A command that paints objects temporarily -- to flag them, highlight them,
    mark them -- should snapshot first and restore after, so it cannot erase
    state it did not set.

    Works on bars, joint blocks, tools, anything: it only reads and writes
    object color.  Returns an opaque token for :func:`restore_object_colors`.
    """
    entries = []
    for oid in object_ids or []:
        if oid is None or not rs.IsObject(oid):
            continue
        entries.append((oid, rs.ObjectColorSource(oid), rs.ObjectColor(oid)))
    return entries


def restore_object_colors(token):
    """Put back exactly what :func:`snapshot_object_colors` captured."""
    for oid, source, color in token or []:
        if not rs.IsObject(oid):
            continue
        # Color first: setting it flips the source to by-object, so the source
        # has to be written afterwards or a by-layer object comes back
        # by-object and keeps the flag color forever.
        rs.ObjectColor(oid, color)
        rs.ObjectColorSource(oid, source)


# ---------------------------------------------------------------------------
# IK color preview (shared single source of truth)
# ---------------------------------------------------------------------------
#
# The multi-bar IK command (rs_ik_keyframe_all) colors bars by IK outcome;
# RSUpdatePreview repaints them at the end of every pass, and its right-click
# companion (rs_clear_color_preview) clears them on demand. All of them import the
# two colors and the show/clear/legend helpers from here so the meanings live in
# ONE place.
#
# These are `paint_bar` overrides (by-object color on the centerline + tube),
# reverted with `reset_bar_color`. COLOR_HAS_IK is PERSISTED state (readable back
# via `core.bar_action.has_ik_keyframe`); COLOR_FAILED is TRANSIENT -- only shown
# live during an rs_ik_keyframe_all solve run, never stored on the bar, so
# `show_all_ik_preview` cannot reconstruct it.
COLOR_HAS_IK = (75, 120, 150)    # bar has a solved IK keyframe
COLOR_FAILED = (230, 115, 150)   # IK attempted at the placed base but failed


def ik_preview_legend_lines():
    """Return the IK color-legend lines (for command-line print / message boxes)."""
    return [
        "IK color preview -- what the bar colors mean:",
        "  gray blue  = bar HAS a solved IK keyframe",
        "  pink  = IK attempted but FAILED (shown live during "
        "RSIKKeyframeAll; not persisted)",
        "  default (by-layer)   = no IK solved yet",
    ]


def print_ik_preview_legend():
    """Print the IK color legend to the Rhino command line."""
    for line in ik_preview_legend_lines():
        print(line)


def clear_ik_preview():
    """Revert every registered bar's color override back to by-layer.

    Clears the IK overlay left by rs_ik_keyframe_all (and by
    :func:`show_all_ik_preview`). Visibility is untouched -- only color is reset.

    Fake bars keep their :data:`SEQ_COLOR_FAKE` tint.  "Which bars are staging"
    is a property of the model, not a diagnostic overlay, so clearing the
    overlay must not clear it -- otherwise the one marker you want on screen
    permanently is the one that disappears every time you tidy up.

    Returns:
        int: the number of bars reset (fake bars are not counted).
    """
    n = 0
    bar_map = get_bar_seq_map()
    fake_ids = get_fake_bar_ids(bar_map)
    rs.EnableRedraw(False)
    for bar_id, (oid, _seq) in bar_map.items():
        if bar_id in fake_ids:
            paint_bar(oid, SEQ_COLOR_FAKE)  # re-assert, in case it was overpainted
            continue
        reset_bar_color(oid)
        n += 1
    rs.EnableRedraw(True)
    return n


def show_all_ik_preview():
    """Color every solved bar ``COLOR_HAS_IK``; leave unsolved bars by-layer.

    Reverts all bar colors first (so a stale color from a previous run does not
    linger), then paints the bars that currently carry a solved IK keyframe. Only
    the persisted "has IK" state can be shown -- the transient "failed" state is
    not stored on the bar (see the section comment above).

    Fake bars are painted :data:`SEQ_COLOR_FAKE` and excluded from both counts:
    the robot never assembles them, so "has IK" is not a question that applies,
    and counting them would make the ratio look worse than it is.

    Returns:
        tuple[int, int]: ``(n_has_ik, n_total)`` over the fabricated bars only.
    """
    # Lazy import: bar_action is heavy and does its own lazy imports; keep this
    # module import-light and avoid an import cycle (bar_action imports us).
    from core.bar_action import has_ik_keyframe

    bar_map = get_bar_seq_map()
    fake_ids = get_fake_bar_ids(bar_map)
    n_has_ik = n_total = 0
    rs.EnableRedraw(False)
    for bar_id, (oid, _seq) in bar_map.items():
        if bar_id in fake_ids:
            paint_bar(oid, SEQ_COLOR_FAKE)
            continue
        n_total += 1
        if has_ik_keyframe(oid):
            paint_bar(oid, COLOR_HAS_IK)
            n_has_ik += 1
        else:
            reset_bar_color(oid)
    rs.EnableRedraw(True)
    return n_has_ik, n_total


# Note: interactive ``pick_bar`` and the ``pick_bar_with_*_option`` family
# now live in :mod:`core.rhino_bar_pick`, which shares a single tube-aware
# geometry filter and tube->centerline resolver.  Import from there.

