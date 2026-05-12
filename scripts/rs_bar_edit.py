#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSBarEdit - Visualize bar lengths and batch resize bars.

On entry: scans every registered bar, groups bars by length (1mm bins),
paints each bar centerline+tube preview a distinct color per length group,
and adds a temporary text-dot at the bar midpoint showing
``"<bar_id>\\n<length>mm"``.

Interactive options (looped):
  - SelectByLength : pick a length group; selects every bar (curve+tube)
    in that group.  Selection is preserved on Exit.
  - ResizeSelected : prompt for a new length; every currently-selected bar
    is shortened/elongated about its midpoint, the tube preview is
    regenerated, and the color/label scheme is refreshed.
  - Refresh        : recompute color groups and dots (after manual edits).
  - Exit           : remove dots, restore by-layer colors, KEEP current
    Rhino selection.

Only straight-line bars are resized in place (LineCurve replacement).
Curved bars are skipped with a warning.
"""

import colorsys
import importlib
import math
import os
import sys
from collections import defaultdict

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
import System.Drawing as sd

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    TUBE_BAR_ID_KEY,
    TUBE_LAYER,
    ensure_bar_preview,
    get_all_bars,
    paint_bar,
    repair_on_entry,
    reset_bar_color,
)
from core.rhino_helpers import curve_endpoints

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_LENGTH_BIN_MM = 1.0  # round bar lengths to nearest 1 mm for grouping
_DOT_PREFIX = "rsbaredit_dot"  # ObjectName prefix for our temporary dots


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _bar_length(curve_id):
    s, e = curve_endpoints(curve_id)
    return float(((e - s) ** 2).sum() ** 0.5)


def _bar_midpoint(curve_id):
    s, e = curve_endpoints(curve_id)
    return (s + e) * 0.5


def _bar_unit_dir(curve_id):
    s, e = curve_endpoints(curve_id)
    v = e - s
    n = float((v * v).sum() ** 0.5)
    if n < 1e-9:
        return None
    return v / n


def _bin_length(L):
    return round(L / _LENGTH_BIN_MM) * _LENGTH_BIN_MM


def _color_for_index(i, n):
    """Distinct RGB color via evenly-spaced HSV hue."""
    if n <= 0:
        return sd.Color.FromArgb(200, 200, 200)
    h = (i / float(n)) % 1.0
    r, g, b = colorsys.hsv_to_rgb(h, 0.65, 0.95)
    return sd.Color.FromArgb(int(r * 255), int(g * 255), int(b * 255))


def _build_length_groups(bar_map):
    """Return (groups, color_by_bin, length_per_bar).

    groups          : ordered list of (length_bin, [bar_id, ...])
    color_by_bin    : {length_bin: System.Drawing.Color}
    length_per_bar  : {bar_id: actual_length_mm}
    """
    length_per_bar = {}
    for bar_id, oid in bar_map.items():
        length_per_bar[bar_id] = _bar_length(oid)

    bin_to_bars = defaultdict(list)
    for bar_id, L in length_per_bar.items():
        bin_to_bars[_bin_length(L)].append(bar_id)

    sorted_bins = sorted(bin_to_bars.keys())
    groups = [(b, sorted(bin_to_bars[b])) for b in sorted_bins]
    n = len(groups)
    color_by_bin = {b: _color_for_index(i, n) for i, b in enumerate(sorted_bins)}
    return groups, color_by_bin, length_per_bar


# ---------------------------------------------------------------------------
# Paint / dot helpers
# ---------------------------------------------------------------------------


def _paint_all(bar_map, color_by_bin, length_per_bar):
    rs.EnableRedraw(False)
    try:
        for bar_id, oid in bar_map.items():
            color = color_by_bin[_bin_length(length_per_bar[bar_id])]
            paint_bar(oid, color)
    finally:
        rs.EnableRedraw(True)


def _reset_all_colors(bar_map):
    rs.EnableRedraw(False)
    try:
        for oid in bar_map.values():
            reset_bar_color(oid)
    finally:
        rs.EnableRedraw(True)


def _add_length_dots(bar_map, length_per_bar):
    """Add a text-dot at each bar midpoint.  Returns the list of dot GUIDs."""
    dot_ids = []
    rs.EnableRedraw(False)
    try:
        for bar_id, oid in bar_map.items():
            mid = _bar_midpoint(oid)
            label = f"{bar_id}\n{length_per_bar[bar_id]:.0f}mm"
            dot_id = rs.AddTextDot(label, (float(mid[0]), float(mid[1]), float(mid[2])))
            if dot_id:
                rs.ObjectName(dot_id, f"{_DOT_PREFIX}_{bar_id}")
                dot_ids.append(dot_id)
    finally:
        rs.EnableRedraw(True)
    return dot_ids


def _clear_dots(dot_ids):
    if not dot_ids:
        return
    alive = [d for d in dot_ids if rs.IsObject(d)]
    if alive:
        rs.DeleteObjects(alive)


# ---------------------------------------------------------------------------
# Selection helpers
# ---------------------------------------------------------------------------


def _find_tube_for_bar(bar_id):
    """Find the tube object on TUBE_LAYER whose tube_bar_id matches *bar_id*."""
    if not rs.IsLayer(TUBE_LAYER):
        return None
    for oid in rs.ObjectsByLayer(TUBE_LAYER) or []:
        if rs.GetUserText(oid, TUBE_BAR_ID_KEY) == bar_id:
            return oid
    return None


def _select_bars(bar_map, bar_ids):
    """Replace current selection with curve+tube of every bar in *bar_ids*."""
    to_select = []
    for bid in bar_ids:
        oid = bar_map.get(bid)
        if oid is None:
            continue
        to_select.append(oid)
        tube = _find_tube_for_bar(bid)
        if tube is not None:
            to_select.append(tube)
    rs.UnselectAllObjects()
    if to_select:
        rs.SelectObjects(to_select)
    return to_select


def _selected_bar_ids(bar_map):
    """Return the set of bar IDs corresponding to the user's current selection.

    A bar is considered selected if either its centerline curve OR its tube
    preview is selected.
    """
    sel = rs.SelectedObjects() or []
    if not sel:
        return []
    selected_curve_ids = set(sel)
    bar_ids = []
    for bar_id, oid in bar_map.items():
        if oid in selected_curve_ids:
            bar_ids.append(bar_id)
            continue
        tube = _find_tube_for_bar(bar_id)
        if tube is not None and tube in selected_curve_ids:
            bar_ids.append(bar_id)
    return bar_ids


# ---------------------------------------------------------------------------
# Resize
# ---------------------------------------------------------------------------


def _resize_bar(curve_id, new_length_mm):
    """Replace *curve_id* with a straight LineCurve of *new_length_mm*,
    centered on the existing midpoint and aligned with the existing
    start->end direction.  Preserves the Rhino object GUID and all UserText.

    Returns True on success, False if the bar is too short to determine a
    direction or the geometry replacement failed.
    """
    direction = _bar_unit_dir(curve_id)
    if direction is None:
        return False
    mid = _bar_midpoint(curve_id)
    half = float(new_length_mm) * 0.5
    new_start = mid - direction * half
    new_end = mid + direction * half
    p0 = Rhino.Geometry.Point3d(float(new_start[0]), float(new_start[1]), float(new_start[2]))
    p1 = Rhino.Geometry.Point3d(float(new_end[0]), float(new_end[1]), float(new_end[2]))
    new_curve = Rhino.Geometry.LineCurve(p0, p1)
    return bool(sc.doc.Objects.Replace(rs.coerceguid(curve_id), new_curve))


def _do_resize_selected(bar_map, selected_bar_ids):
    """Prompt for new length; resize each selected bar; regenerate tubes.

    Returns True if any bar was resized.
    """
    if not selected_bar_ids:
        print("RSBarEdit: No bars selected. Use SelectByLength first or pick bars manually.")
        return False

    # Suggest the average current length as default.
    cur_lengths = [_bar_length(bar_map[b]) for b in selected_bar_ids]
    default_L = sum(cur_lengths) / len(cur_lengths)

    new_L = rs.GetReal(
        f"New length (mm) for {len(selected_bar_ids)} bar(s)",
        number=round(default_L, 1),
        minimum=1.0,
    )
    if new_L is None:
        print("RSBarEdit: resize cancelled.")
        return False

    n_ok = 0
    n_skip = 0
    for bar_id in selected_bar_ids:
        oid = bar_map[bar_id]
        if not _resize_bar(oid, new_L):
            print(f"  RSBarEdit: skipped {bar_id} (degenerate or non-line geometry).")
            n_skip += 1
            continue
        ensure_bar_preview(oid, float(config.BAR_RADIUS), bar_id=bar_id)
        n_ok += 1
    print(f"RSBarEdit: resized {n_ok} bar(s) to {new_L:.1f} mm ({n_skip} skipped).")
    return n_ok > 0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _format_group_label(length_bin, bar_ids):
    if len(bar_ids) <= 4:
        ids = ",".join(bar_ids)
    else:
        ids = ",".join(bar_ids[:3]) + f",...(+{len(bar_ids) - 3})"
    return f"{length_bin:.0f}mm_x{len(bar_ids)}_[{ids}]"


def _print_summary(groups):
    print("\n--- Bar Length Groups ---")
    total = 0
    for L, bids in groups:
        total += len(bids)
        print(f"  {L:.0f} mm  x{len(bids)}  : {','.join(bids)}")
    print(f"  Total bars: {total}")
    print("--- End ---\n")


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), "RSBarEdit")

    bar_map = get_all_bars()
    if not bar_map:
        print("RSBarEdit: No registered bars in the document.")
        return

    groups, color_by_bin, length_per_bar = _build_length_groups(bar_map)
    _paint_all(bar_map, color_by_bin, length_per_bar)
    dot_ids = _add_length_dots(bar_map, length_per_bar)
    _print_summary(groups)

    selected_length_index = 0  # OptionList default

    try:
        while True:
            # Recompute group label list every loop so it reflects edits.
            group_labels = [_format_group_label(L, bids) for L, bids in groups]
            if selected_length_index >= len(group_labels):
                selected_length_index = 0

            go = Rhino.Input.Custom.GetOption()
            go.SetCommandPrompt("RSBarEdit (Esc to exit)")
            go.AcceptNothing(True)

            sel_idx = (
                go.AddOptionList("SelectByLength", group_labels, selected_length_index)
                if group_labels else -1
            )
            resize_idx = go.AddOption("ResizeSelected")
            refresh_idx = go.AddOption("Refresh")
            exit_idx = go.AddOption("Exit")

            res = go.Get()
            if res == Rhino.Input.GetResult.Cancel:
                break
            if res == Rhino.Input.GetResult.Nothing:
                # Bare Enter -> exit
                break
            if res != Rhino.Input.GetResult.Option:
                continue

            opt = go.Option()
            if opt is None:
                continue

            if sel_idx != -1 and opt.Index == sel_idx:
                selected_length_index = int(opt.CurrentListOptionIndex)
                L_bin, bar_ids = groups[selected_length_index]
                _select_bars(bar_map, bar_ids)
                print(f"RSBarEdit: selected {len(bar_ids)} bar(s) at {L_bin:.0f} mm.")
                continue

            if opt.Index == resize_idx:
                sel_bar_ids = _selected_bar_ids(bar_map)
                if _do_resize_selected(bar_map, sel_bar_ids):
                    # Recompute everything after geometry changes.
                    bar_map = get_all_bars()
                    groups, color_by_bin, length_per_bar = _build_length_groups(bar_map)
                    _clear_dots(dot_ids)
                    dot_ids = _add_length_dots(bar_map, length_per_bar)
                    _paint_all(bar_map, color_by_bin, length_per_bar)
                    _print_summary(groups)
                    # Re-select the just-resized bars so the user can iterate.
                    _select_bars(bar_map, [b for b in sel_bar_ids if b in bar_map])
                continue

            if opt.Index == refresh_idx:
                bar_map = get_all_bars()
                groups, color_by_bin, length_per_bar = _build_length_groups(bar_map)
                _clear_dots(dot_ids)
                dot_ids = _add_length_dots(bar_map, length_per_bar)
                _paint_all(bar_map, color_by_bin, length_per_bar)
                _print_summary(groups)
                continue

            if opt.Index == exit_idx:
                break
    finally:
        # Always restore display state, but preserve the user's selection.
        preserved_selection = list(rs.SelectedObjects() or [])
        _clear_dots(dot_ids)
        _reset_all_colors(bar_map)
        rs.UnselectAllObjects()
        if preserved_selection:
            alive = [oid for oid in preserved_selection if rs.IsObject(oid)]
            if alive:
                rs.SelectObjects(alive)
        print("RSBarEdit: Done. Display restored; selection preserved.")


if __name__ == "__main__":
    main()
