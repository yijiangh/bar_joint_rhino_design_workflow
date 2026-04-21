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

from core.rhino_helpers import (
    as_object_id_list,
    apply_object_display,
    curve_endpoints,
    delete_objects,
    ensure_layer,
    point_to_array,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAR_TYPE_KEY = "bar_type"
BAR_ID_KEY = "bar_id"
BAR_GUID_KEY = "bar_guid"
BAR_SEQ_KEY = "bar_seq"
BAR_TYPE_VALUE = "scaffolding_bar"

TUBE_LAYER = "Tube preview"
TUBE_BAR_ID_KEY = "tube_bar_id"
TUBE_AXIS_GUID_KEY = "tube_axis_id"
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
# Sequence colour-coding and visibility
# ---------------------------------------------------------------------------

#: Colour for bars that have already been assembled (earlier in sequence).
SEQ_COLOR_BUILT = (60, 179, 60)  # green — already assembled
#: Colour for the bar currently being assembled (active step).
SEQ_COLOR_ACTIVE = (30, 100, 220)  # blue — current step
#: Colour for bars not yet assembled (later in sequence).
SEQ_COLOR_UNBUILT = (160, 160, 160)  # grey — not yet assembled


def _set_obj_color(oid, color):
    """Set by-object colour on *oid*."""
    rs.ObjectColorSource(oid, 1)  # 1 = by object
    rs.ObjectColor(oid, color)


def _reset_obj_color(oid):
    """Restore by-layer colour on *oid*."""
    rs.ObjectColorSource(oid, 0)  # 0 = by layer


def _bar_curve_and_tube(curve_id):
    """Return ``[curve_id]`` plus the tube GUID if one exists."""
    ids = [curve_id]
    tube = _find_existing_tube(curve_id)
    if tube is not None:
        ids.append(tube)
    return ids


def show_sequence_colors(active_bar_id, show_unbuilt=True):
    """Apply sequence colour-coding to all registered bars in the viewport.

    Parameters
    ----------
    active_bar_id : str
        The bar ID representing the current assembly step.
    show_unbuilt : bool
        When ``False``, bars with a higher sequence number than the active bar
        are hidden so the viewport shows only the built portion of the assembly.
    """
    bar_map = get_bar_seq_map()
    if active_bar_id not in bar_map:
        return
    _, active_seq = bar_map[active_bar_id]

    rs.EnableRedraw(False)
    for bar_id, (oid, seq) in bar_map.items():
        if seq < active_seq:
            color = SEQ_COLOR_BUILT
            visible = True
        elif seq == active_seq:
            color = SEQ_COLOR_ACTIVE
            visible = True
        else:
            color = SEQ_COLOR_UNBUILT
            visible = show_unbuilt

        for obj in _bar_curve_and_tube(oid):
            _set_obj_color(obj, color)
            if visible:
                rs.ShowObject(obj)
            else:
                rs.HideObject(obj)
    rs.EnableRedraw(True)


def reset_sequence_colors():
    """Restore default (by-layer) colour and make all registered bars visible."""
    bar_map = get_bar_seq_map()
    rs.EnableRedraw(False)
    for bar_id, (oid, _) in bar_map.items():
        for obj in _bar_curve_and_tube(oid):
            _reset_obj_color(obj)
            rs.ShowObject(obj)
    rs.EnableRedraw(True)


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


def ensure_bar_preview(curve_id, bar_radius, color=None, bar_id=None):
    """Make sure a tube preview exists and matches the current curve geometry.

    Creates or regenerates the tube as needed.  Returns list of tube GUIDs.
    """
    # Check for an existing tube
    existing = _find_existing_tube(curve_id)
    if existing is not None:
        if _tube_geometry_matches(existing, curve_id):
            return [existing]
        # Stale — delete and recreate
        delete_objects([existing])

    start_xyz, end_xyz = curve_endpoints(curve_id)
    tube_id = _create_tube_brep(start_xyz, end_xyz, bar_radius)
    if tube_id is None:
        return []

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
    return baked_ids


def update_all_previews(bar_radius, color=None):
    """Ensure every registered bar has an up-to-date tube preview.

    Returns the number of bars processed.
    """
    bars = get_all_bars()
    for bar_id, guid in bars.items():
        ensure_bar_preview(guid, bar_radius, color=color, bar_id=bar_id)
    return len(bars)


def repair_on_entry(bar_radius, caller="RSScaffolding"):
    """Standard startup repair for bar-registry-aware entry-point scripts.

    Call this at the top of ``main()``, right after reloading config.  It
    runs :func:`update_all_previews` and :func:`repair_bar_sequences` in one
    step, so copy-paste artifacts, deleted bars, and stale tube previews are
    all resolved before the user is prompted.

    Parameters
    ----------
    bar_radius : float
        Tube preview radius, typically ``float(config.BAR_RADIUS)``.
    caller : str
        Script name used in the startup message, e.g. ``"RSBarSnap"``.
    """
    n = update_all_previews(bar_radius)
    changed = repair_bar_sequences()
    parts = []
    if n:
        parts.append(f"{n} bar preview(s) updated")
    if changed:
        parts.append(f"{len(changed)} sequence number(s) repaired")
    if parts:
        print(f"{caller} (startup): {', '.join(parts)}.")
