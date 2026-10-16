#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSBarEdit - Visualize bar lengths, batch resize bars, mark staging bars.

Two modes, chosen at the first prompt:

**BarLength** (the default, and everything described below) - the length
grouping, batch resize and refresh tools.

**FakeBar** - Add / Delete the "fake bar" mark.  A fake bar is real geometry
that will NOT be fabricated: temporary staging put up by hand so the robot has
something to mate a real bar against.  It stays a full registered bar - id,
assembly step, joints and all - because IK derives "what is already standing"
from exactly those, and a support the robot cannot see is one it will drive
through.  Only fabrication output and the sequence display treat it differently.
Entering the mode paints every fake bar pink, each Add/Delete repaints the
picked bar at once, and bars are picked by centerline or tube preview.

On entry: scans every registered bar, groups bars by length (1mm bins, shared
with RSBarSelect > SelectByLength through ``core.bar_length_groups``), paints
each bar centerline+tube preview a distinct color per length group, and adds a
temporary text-dot at the bar midpoint showing ``"<bar_id>\\n<length>mm"``.

Interactive options (looped):
  - ResizeSelected : prompt for a new length; every currently-selected bar
    is shortened/elongated about its midpoint, the tube preview is
    regenerated, and the color/label scheme is refreshed.  Select the bars
    before running the command -- RSBarSelect > SelectByLength selects a whole
    length group.
  - Refresh        : recompute color groups and dots (after manual edits).
  - Exit           : remove dots, restore by-layer colors, KEEP current
    Rhino selection.

Only straight-line bars are resized in place (LineCurve replacement).
Curved bars are skipped with a warning.
"""

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import bar_length_groups as length_groups
from core import config
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    SEQ_COLOR_FAKE,
    TUBE_BAR_ID_KEY,
    TUBE_LAYER,
    ensure_bar_preview,
    get_all_bars,
    is_fake_bar,
    paint_bar,
    repair_on_entry,
    reset_bar_color,
    set_fake_bar,
)
from core.rhino_bar_pick import bar_or_tube_filter, resolve_picked_to_bar_curve
from core.rhino_helpers import ask_option, curve_endpoints, delete_objects, objects_on_layers

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DOT_PREFIX = "rsbaredit_dot"  # ObjectName prefix for our temporary dots


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


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


def _reset_all_colors(bar_map):
    rs.EnableRedraw(False)
    try:
        for oid in bar_map.values():
            reset_bar_color(oid)
    finally:
        rs.EnableRedraw(True)


# ---------------------------------------------------------------------------
# Selection helpers
# ---------------------------------------------------------------------------


def _find_tube_for_bar(bar_id):
    """Find the tube object on TUBE_LAYER whose tube_bar_id matches *bar_id*."""
    for oid in objects_on_layers(TUBE_LAYER):
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
        print(
            "RSBarEdit: No bars selected.  Select them before running RSBarEdit "
            "(RSBarSelect > SelectByLength selects a whole length group)."
        )
        return False

    # Suggest the average current length as default.
    cur_lengths = [length_groups.bar_length(bar_map[b]) for b in selected_bar_ids]
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


def _ask_mode():
    """Ask for BarLength or FakeBar.  Returns ``"length"`` / ``"fake"`` / None."""
    choice = ask_option(
        "Edit bar lengths, or mark bars as non-fabricated staging",
        ("BarLength", "FakeBar"),
        default="BarLength",
    )
    return {"BarLength": "length", "FakeBar": "fake"}.get(choice)


def _pick_bar_curve(prompt):
    """Pick a registered bar by its centerline **or** its tube preview.

    Returns the centerline curve id, or ``None`` on Enter/Esc (which is how the
    caller's toggle loop ends).  Picking the tube matters here: with the tube
    preview on, the centerline is buried inside it and effectively unclickable.
    """
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    go.AcceptNothing(True)
    go.EnablePreSelect(False, True)
    go.SetCustomGeometryFilter(bar_or_tube_filter)
    if go.Get() != Rhino.Input.GetResult.Object:
        return None
    picked_id = go.Object(0).ObjectId
    rs.UnselectObject(picked_id)
    return resolve_picked_to_bar_curve(picked_id)


def _show_fake_color(curve_id, fake):
    """Paint one bar to match its fake state: pink when fake, by-layer when not."""
    if fake:
        paint_bar(curve_id, SEQ_COLOR_FAKE)
    else:
        reset_bar_color(curve_id)
    rs.Redraw()


def _paint_fake_bars(bar_map):
    """Paint every currently-fake bar pink.  Returns their sorted bar ids.

    Only fake bars are touched -- a non-fake bar is left with whatever overlay
    it already carries, so entering FakeBar mode never wipes an IK or sequence
    preview the user was reading.
    """
    fake_ids = [b for b, oid in sorted(bar_map.items()) if is_fake_bar(oid)]
    rs.EnableRedraw(False)
    try:
        for bar_id in fake_ids:
            paint_bar(bar_map[bar_id], SEQ_COLOR_FAKE)
    finally:
        rs.EnableRedraw(True)
    return fake_ids


def _pick_bars_for_fake(bar_map, want_fake):
    """Toggle the fake mark on picked bars until Enter/Esc.  Returns the count.

    *want_fake* True = Add, False = Delete.  Bars already in the wanted state are
    reported rather than silently re-written, so a mis-pick is visible.  Each
    change repaints that bar immediately, so the pink set on screen always
    matches the marks on the model.
    """
    verb = "mark as fake" if want_fake else "unmark"
    n_changed = 0
    while True:
        picked = _pick_bar_curve(f"Select a bar to {verb}  (Enter when done)")
        if picked is None:
            return n_changed
        bar_id = rs.GetUserText(picked, BAR_ID_KEY)
        if not bar_id or bar_id not in bar_map:
            print("RSBarEdit: that curve is not a registered bar.")
            continue
        curve_id = bar_map[bar_id]
        if is_fake_bar(curve_id) == want_fake:
            state = "already fake" if want_fake else "not fake"
            print(f"RSBarEdit: {bar_id} is {state}; nothing to do.")
            continue
        set_fake_bar(curve_id, want_fake)
        _show_fake_color(curve_id, want_fake)
        n_changed += 1
        print(
            f"RSBarEdit: {bar_id} -> "
            + ("FAKE (will not be fabricated)" if want_fake else "real")
        )


def _run_fake_bar():
    """Add / Delete the fake-bar mark.  No geometry is touched either way.

    The pink highlight is painted on entry and *left on* at exit: which bars are
    staging is a property of the model, not a diagnostic overlay, so it survives
    the command the same way it survives RSClearColorPreview.
    """
    bar_map = get_all_bars()
    if not bar_map:
        print("RSBarEdit: No registered bars in the document.")
        return

    already = _paint_fake_bars(bar_map)
    print(
        f"RSBarEdit (FakeBar): {len(already)} of {len(bar_map)} bar(s) marked fake"
        + (f": {', '.join(already)}." if already else ".")
        + "  Fake bars are shown in pink."
    )

    while True:
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt("FakeBar (Esc to exit)")
        go.AcceptNothing(True)
        add_idx = go.AddOption("Add")
        del_idx = go.AddOption("Delete")
        exit_idx = go.AddOption("Exit")

        res = go.Get()
        if res != Rhino.Input.GetResult.Option:
            return  # Enter or Esc
        opt = go.Option()
        if opt is None:
            continue
        if opt.Index == exit_idx:
            return
        if opt.Index in (add_idx, del_idx):
            n = _pick_bars_for_fake(bar_map, want_fake=opt.Index == add_idx)
            if n:
                print(f"RSBarEdit: {n} bar(s) changed.")


def _run_bar_length():
    bar_map = get_all_bars()
    if not bar_map:
        print("RSBarEdit: No registered bars in the document.")
        return

    def _show_groups():
        groups, color_by_bin, length_per_bar = length_groups.build_length_groups(bar_map)
        length_groups.paint_length_groups(bar_map, color_by_bin, length_per_bar)
        length_groups.print_length_summary(groups)
        return length_groups.add_length_dots(bar_map, length_per_bar, _DOT_PREFIX)

    dot_ids = _show_groups()
    try:
        while True:
            go = Rhino.Input.Custom.GetOption()
            go.SetCommandPrompt("RSBarEdit (Esc to exit)")
            go.AcceptNothing(True)

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

            if opt.Index == resize_idx:
                sel_bar_ids = _selected_bar_ids(bar_map)
                if _do_resize_selected(bar_map, sel_bar_ids):
                    # Recompute everything after geometry changes.
                    bar_map = get_all_bars()
                    delete_objects(dot_ids)
                    dot_ids = _show_groups()
                    # Re-select the just-resized bars so the user can iterate.
                    _select_bars(bar_map, [b for b in sel_bar_ids if b in bar_map])
                continue

            if opt.Index == refresh_idx:
                bar_map = get_all_bars()
                delete_objects(dot_ids)
                dot_ids = _show_groups()
                continue

            if opt.Index == exit_idx:
                break
    finally:
        # Always restore display state, but preserve the user's selection.
        preserved_selection = list(rs.SelectedObjects() or [])
        delete_objects(dot_ids)
        _reset_all_colors(bar_map)
        _paint_fake_bars(bar_map)  # re-assert: the fake mark outlives the overlay
        rs.UnselectAllObjects()
        if preserved_selection:
            alive = [oid for oid in preserved_selection if rs.IsObject(oid)]
            if alive:
                rs.SelectObjects(alive)
        print("RSBarEdit: Done. Display restored; selection preserved.")


def main():
    importlib.reload(config)
    importlib.reload(length_groups)
    repair_on_entry(float(config.BAR_RADIUS), "RSBarEdit")

    mode = _ask_mode()
    if mode is None:
        return
    if mode == "fake":
        _run_fake_bar()
        return
    _run_bar_length()


if __name__ == "__main__":
    main()
