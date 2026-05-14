#! python 3
# venv: scaffolding_env
"""RSSequenceEdit - Interactive assembly sequence viewer and editor.

Displays all registered bars colour-coded by assembly step and lets the user:

  - **Click a bar** to make it the active (current) step.
  - **Type a step number** (e.g. ``3``) to jump to that assembly step.
  - **Type a bar id** (e.g. ``B5`` / ``b5``) to jump to that specific bar.
  - **Next / Previous** to walk through the sequence one step at a time.
  - **MoveEarlier / MoveLater** to swap the active bar one step back or forward
    in the assembly order without leaving the tool.
  - **PickBarAfterThis** to pick any other bar and insert it immediately after
    the current bar in the sequence.
  - **EditSupports** to declare which other bars must be assembled before the
    current bar is considered stable.  Inside this mode all unbuilt bars are
    forced visible, the existing supports are highlighted in **dark purple**,
    and clicking a bar toggles its membership in the support set.  Press
    Enter to save, Esc to cancel; the previous unbuilt-visibility setting is
    restored on exit.  Built bars whose supports are still unbuilt are
    recoloured **teal** to flag them as unstable.
  - **ShowUnbuilt / HideUnbuilt** to toggle the visibility of bars (and their
    child joints) that come later than the current step, giving an instant
    "assembly stage" preview.  The active step's robotic tool is always shown;
    all other tools are hidden while the command is active.
  - **Finish** (or Escape) to restore normal display and exit.

Colour legend
-------------
- Green - already assembled (earlier in sequence)
- Teal  - already assembled but still needs temporary support
- Blue  - current step (active bar)
- Grey  - not yet assembled (later in sequence, when shown)
"""

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    SEQ_COLOR_SUPPORT_PICK,
    get_bar_seq_map,
    get_supported_until,
    set_supported_until,
    move_bar_earlier,
    move_bar_later,
    insert_bar_after,
    paint_bar,
    repair_on_entry,
    show_sequence_colors,
    reset_sequence_colors,
)
from core.rhino_helpers import curve_endpoints

# Reuse the tube-aware bar filter and tube->centerline resolver so clicking
# the bar's tube preview works the same way it does in
# RSJointPlace / RSBarSnap / etc.
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    make_bar_or_tube_filter as _make_bar_or_tube_filter,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)


# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------


class _SequenceSession:
    """Holds mutable UI state for one RSSequenceEdit run."""

    def __init__(self):
        self.active_bar_id = None
        self.show_unbuilt = True
        # GUID of the temporary text-dot label drawn at the active bar's
        # end-point.  Recreated on every set_active(), cleaned up on exit.
        self._active_dot_id = None

    # ------------------------------------------------------------------
    # Active-bar end-point label

    def _clear_active_dot(self):
        if self._active_dot_id is not None and rs.IsObject(self._active_dot_id):
            rs.DeleteObject(self._active_dot_id)
        self._active_dot_id = None

    def _refresh_active_dot(self):
        """Place a text-dot at the active bar's end-point showing
        ``bar_id`` and assembly sequence number.  This is the same end of
        the bar where the physical label will be applied."""
        self._clear_active_dot()
        if self.active_bar_id is None:
            return
        bar_map = get_bar_seq_map()
        entry = bar_map.get(self.active_bar_id)
        if entry is None:
            return
        oid, seq = entry
        try:
            _start, end = curve_endpoints(oid)
        except Exception:
            return
        label = f"{self.active_bar_id}\nstep {seq}"
        dot_id = rs.AddTextDot(
            label, (float(end[0]), float(end[1]), float(end[2]))
        )
        if dot_id:
            self._active_dot_id = dot_id

    # ------------------------------------------------------------------
    # Helpers

    def _sorted_bars(self):
        """Return [(bar_id, oid, seq)] in ascending sequence order."""
        bar_map = get_bar_seq_map()
        return sorted(
            [(bid, oid, seq) for bid, (oid, seq) in bar_map.items()],
            key=lambda x: x[2],
        )

    def _bar_count(self):
        return len(get_bar_seq_map())

    def _active_seq(self):
        if self.active_bar_id is None:
            return None
        bar_map = get_bar_seq_map()
        entry = bar_map.get(self.active_bar_id)
        return entry[1] if entry else None

    def _print_status(self):
        seq = self._active_seq()
        total = self._bar_count()
        unbuilt_note = "shown" if self.show_unbuilt else "hidden"
        print(
            f"Step {seq} of {total}  ({self.active_bar_id})"
            f"  |  unbuilt bars: {unbuilt_note}"
        )

    # ------------------------------------------------------------------
    # State transitions

    def set_active(self, bar_id):
        self.active_bar_id = bar_id
        show_sequence_colors(bar_id, self.show_unbuilt)
        self._refresh_active_dot()
        self._print_status()

    def select_next(self):
        bars = self._sorted_bars()
        if not bars:
            return
        seq = self._active_seq() or 0
        for bar_id, oid, s in bars:
            if s > seq:
                self.set_active(bar_id)
                return

    def select_previous(self):
        bars = self._sorted_bars()
        if not bars:
            return
        seq = self._active_seq()
        if seq is None:
            seq = bars[-1][2] + 1
        for bar_id, oid, s in reversed(bars):
            if s < seq:
                self.set_active(bar_id)
                return

    def select_by_step(self, step):
        """Make the bar at sequence number *step* (1-based) active."""
        for bar_id, _oid, seq in self._sorted_bars():
            if seq == step:
                self.set_active(bar_id)
                return True
        print(f"RSSequenceEdit: No bar at step {step}.")
        return False

    def select_by_bar_id(self, bar_id):
        """Make the bar with id *bar_id* (e.g. ``"B5"``) active."""
        if bar_id in get_bar_seq_map():
            self.set_active(bar_id)
            return True
        print(f"RSSequenceEdit: Bar {bar_id} is not registered.")
        return False

    def do_move_earlier(self):
        if self.active_bar_id is None:
            return
        move_bar_earlier(self.active_bar_id)
        show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()

    def do_move_later(self):
        if self.active_bar_id is None:
            return
        move_bar_later(self.active_bar_id)
        show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()

    def do_pick_bar_after_this(self):
        """Ask user to pick a bar and insert it immediately after the active bar."""
        if self.active_bar_id is None:
            print("RSSequenceEdit: Select an active bar first.")
            return
        go2 = Rhino.Input.Custom.GetObject()
        go2.SetCommandPrompt(
            f"Pick a bar to insert AFTER {self.active_bar_id} in the sequence"
        )
        go2.EnablePreSelect(False, False)
        go2.SetCustomGeometryFilter(_bar_or_tube_filter)
        result2 = go2.Get()
        if result2 != Rhino.Input.GetResult.Object:
            return
        picked_id = go2.Object(0).ObjectId
        bar_curve_id = _resolve_picked_to_bar_curve(picked_id)
        if bar_curve_id is None:
            return
        picked_bar_id = rs.GetUserText(bar_curve_id, BAR_ID_KEY)
        if not picked_bar_id:
            return
        if picked_bar_id == self.active_bar_id:
            print("RSSequenceEdit: Cannot move a bar after itself.")
            return
        insert_bar_after(picked_bar_id, self.active_bar_id)
        # show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()
        self.select_next()  # move active to the bar we just inserted, which is now next in the sequence

    def do_edit_supports(self):
        """Toggle-pick the bars that must be assembled before the active bar
        is considered stable, and persist the list as ``supported_until``
        UserText on the active bar's centerline curve.

        Behaviour
        ---------
        * Forces all unbuilt bars visible while the picker is open so any
          bar in the model can be toggled, regardless of the current
          ``ShowUnbuilt`` setting.  The setting is restored on exit.
        * Pre-existing supports are highlighted in dark purple
          (``SEQ_COLOR_SUPPORT_PICK``); clicking a bar toggles its
          membership in the support set and re-tints accordingly.
        * Enter saves the new set; Esc cancels and leaves the persisted
          list unchanged.
        * The active bar itself is masked out by the pick filter.
        """
        if self.active_bar_id is None:
            print("RSSequenceEdit: Select an active bar first.")
            return
        bar_map = get_bar_seq_map()
        active_oid, _ = bar_map[self.active_bar_id]
        # Drop any deps that don't resolve in the current bar map.  The
        # entry-point already ran ``cleanup_stale_supports`` via
        # ``repair_on_entry``, but the file may have been edited mid-session.
        selected = [b for b in get_supported_until(active_oid) if b in bar_map]

        original_show_unbuilt = self.show_unbuilt

        def _repaint():
            # Reset to the standard sequence palette with EVERYTHING visible,
            # then overlay the support-pick purple on currently-selected bars.
            show_sequence_colors(self.active_bar_id, True)
            for bid in selected:
                entry = bar_map.get(bid)
                if entry:
                    paint_bar(entry[0], SEQ_COLOR_SUPPORT_PICK)

        _repaint()
        print(
            f"RSSequenceEdit: editing supports for {self.active_bar_id}.  "
            "Click bars to toggle (purple = selected).  Enter = save, Esc = cancel."
        )

        committed = False
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                f"Toggle supports for {self.active_bar_id} "
                f"({len(selected)} selected)  [Enter = save, Esc = cancel]"
            )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(
                _make_bar_or_tube_filter(exclude_bar_ids=[self.active_bar_id])
            )
            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                print("RSSequenceEdit: cancelled, supports unchanged.")
                break

            if result == Rhino.Input.GetResult.Nothing:
                set_supported_until(active_oid, selected)
                committed = True
                if selected:
                    print(
                        f"RSSequenceEdit: {self.active_bar_id} supported until: "
                        f"{', '.join(selected)}."
                    )
                else:
                    print(
                        f"RSSequenceEdit: cleared supports for {self.active_bar_id}."
                    )
                break

            if result == Rhino.Input.GetResult.Object:
                picked_id = go.Object(0).ObjectId
                rs.UnselectObject(picked_id)
                bar_curve_id = _resolve_picked_to_bar_curve(picked_id)
                if bar_curve_id is None:
                    continue
                bid = rs.GetUserText(bar_curve_id, BAR_ID_KEY)
                if not bid or bid == self.active_bar_id:
                    continue
                if bid in selected:
                    selected.remove(bid)
                else:
                    selected.append(bid)
                _repaint()
                continue

            # Anything else (unexpected) -> bail without saving.
            break

        # Restore the user's preferred unbuilt-visibility setting and refresh
        # the standard sequence display so unstable bars repaint correctly.
        self.show_unbuilt = original_show_unbuilt
        show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        if committed:
            self._print_status()

    def toggle_unbuilt(self):
        self.show_unbuilt = not self.show_unbuilt
        if self.active_bar_id is not None:
            show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _parse_typed_selector(text, total_steps):
    """Classify a typed input as a bar id, an assembly step number, or invalid.

    A leading ``b`` / ``B`` followed by digits -> bar id (e.g. ``b5`` -> ``"B5"``).
    A bare positive integer in ``[1, total_steps]`` -> assembly step number.
    Anything else returns ``("error", message)``.
    """
    text = (text or "").strip()
    if not text:
        return ("error", "Empty input.")
    if text[0] in ("b", "B"):
        suffix = text[1:]
        if not suffix.isdigit():
            return ("error", f"Invalid bar id: {text!r}")
        return ("bar", "B" + suffix)
    if text.lstrip("+").isdigit():
        n = int(text)
        if 1 <= n <= total_steps:
            return ("step", n)
        return ("error", f"Step {n} out of range (1..{total_steps}).")
    return ("error", f"Unrecognised input: {text!r}")


def _build_get_option(session, last_action):
    """Construct the multi-input ``GetObject`` used by the main loop.

    Accepts: bar object click | command-line option | bare Enter (repeat) |
    typed string (bar id like ``B5`` / ``b5`` or assembly step number).
    """
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(
        f"Select bar, type step # or bar id (e.g. B5)  (Enter = {last_action})"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)   # bare Enter -> repeat last action
    go.AcceptString(True)    # accept typed numbers / bar ids
    go.SetCustomGeometryFilter(_bar_or_tube_filter)

    go.AddOption("Next")
    go.AddOption("Previous")
    go.AddOption("MoveEarlier")
    go.AddOption("MoveLater")
    go.AddOption("PickBarAfterThis")
    go.AddOption("EditSupports")
    if session.show_unbuilt:
        go.AddOption("HideUnbuilt")
    else:
        go.AddOption("ShowUnbuilt")
    go.AddOption("Finish")
    return go


def _run_action(name, session):
    """Execute a named action on *session*.  Returns False only for Finish."""
    if name == "Finish":
        return False
    elif name == "Next":
        session.select_next()
    elif name == "Previous":
        session.select_previous()
    elif name == "MoveEarlier":
        session.do_move_earlier()
    elif name == "MoveLater":
        session.do_move_later()
    elif name == "PickBarAfterThis":
        session.do_pick_bar_after_this()
    elif name == "EditSupports":
        session.do_edit_supports()
    elif name in ("ShowUnbuilt", "HideUnbuilt"):
        session.toggle_unbuilt()
    return True


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), "RSSequenceEdit")

    bar_map = get_bar_seq_map()
    if not bar_map:
        print("RSSequenceEdit: No registered bars found. Run RSCreateBar first.")
        return

    session = _SequenceSession()
    # Default Enter action - walks forward through the sequence.
    last_action = "Next"

    try:
        while True:
            # Create a fresh GetObject each iteration so a previous pick doesn't
            # re-trigger via pre-selection (which would cause an infinite loop).
            go = _build_get_option(session, last_action)
            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                break

            if result == Rhino.Input.GetResult.Object:
                picked_id = go.Object(0).ObjectId
                bar_curve_id = _resolve_picked_to_bar_curve(picked_id)
                if bar_curve_id is not None:
                    bar_id = rs.GetUserText(bar_curve_id, BAR_ID_KEY)
                    if bar_id:
                        session.set_active(bar_id)
                # Picking a bar doesn't change the repeat action.
                continue

            if result == Rhino.Input.GetResult.String:
                kind, value = _parse_typed_selector(
                    go.StringResult(), len(get_bar_seq_map())
                )
                if kind == "bar":
                    session.select_by_bar_id(value)
                elif kind == "step":
                    session.select_by_step(value)
                else:
                    print(f"RSSequenceEdit: {value}")
                # Typed input doesn't change the repeat action either.
                continue

            if result == Rhino.Input.GetResult.Nothing:
                # Bare Enter - repeat last action.
                action_name = last_action
            elif result == Rhino.Input.GetResult.Option:
                action_name = go.Option().EnglishName
            else:
                continue

            if not _run_action(action_name, session):
                break

            # Update repeat target for all actions except the visibility toggle
            # (toggling twice in a row would be confusing).
            if action_name not in ("ShowUnbuilt", "HideUnbuilt"):
                last_action = action_name

    finally:
        # Restore normal display on exit and remove the active-bar dot.
        session._clear_active_dot()
        reset_sequence_colors()
        print("RSSequenceEdit: Finished. Display restored.")


if __name__ == "__main__":
    main()
