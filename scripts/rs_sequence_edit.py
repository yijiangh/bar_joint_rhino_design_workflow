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

    HideUnbuilt **persists**: it saves the current step to the document as
    ``scaffolding.build_stage`` and every other RSScaffolding command re-applies
    it on entry, so later bars, joints and tools stay hidden -- and therefore
    unpickable -- after this command exits, across save and reload.  Only
    ShowUnbuilt clears it.
  - **Finish** (or Escape) to exit.  Colours are restored; a latched build stage
    is not -- that is the point of it.

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
    clear_build_stage,
    get_bar_seq_map,
    get_build_stage,
    set_build_stage,
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
from core.rhino_helpers import curve_endpoints, ensure_layer

# Reuse the tube-aware bar filter and tube->centerline resolver so clicking
# the bar's tube preview works the same way it does in
# RSJointPlace / RSBarSnap / etc.
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    make_bar_or_tube_filter as _make_bar_or_tube_filter,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)


# ---------------------------------------------------------------------------
# Active-bar label dots
# ---------------------------------------------------------------------------
#
# The label dot is tagged and parked on the shared diagnostic-marks layer so it
# can always be found again.  It used to be tracked ONLY by an in-memory GUID,
# which meant any path that lost that reference -- an exception between baking
# the dot and the cleanup, a second session opened over a first, a Ctrl+Z that
# resurrected a deleted dot -- left a dot in the document that nothing could
# find or remove.  With the tag, cleanup is a sweep of everything carrying it
# rather than a delete of one remembered id, so strays from earlier runs get
# collected too.
#
# Distinct from ``rhino_joint_refresh._MARK_KEY`` on purpose: those dots mark
# broken links, are deliberate output of ShowColorsPreview, and are cleared by
# RSClearColorPreview.  Neither sweep may eat the other's dots.
_SEQ_DOT_KEY = "rs_sequence_label_dot"


def _sweep_sequence_dots():
    """Delete every label dot this command has ever left behind.

    Safe to call when none exist, and safe to call on entry: the dots are pure
    UI, so there is nothing to preserve between runs.
    """
    layer = config.LAYER_DIAGNOSTIC_MARKS
    if not rs.IsLayer(layer):
        return 0
    strays = [
        oid for oid in (rs.ObjectsByLayer(layer) or [])
        if rs.GetUserText(oid, _SEQ_DOT_KEY)
    ]
    if strays:
        rs.DeleteObjects(strays)
    return len(strays)


# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------


class _SequenceSession:
    """Holds mutable UI state for one RSSequenceEdit run."""

    def __init__(self):
        self.active_bar_id = None
        self.show_unbuilt = True
        # Resume a build stage latched in a previous run.  The document, not
        # this object, is the source of truth: repair_on_entry has already
        # hidden the unbuilt bars by the time we get here, so starting with
        # show_unbuilt = True would offer HideUnbuilt when the user needs
        # ShowUnbuilt.  Pre-setting active_bar_id is what stops a browse from
        # moving the stage -- set_active() writes the latch while show_unbuilt
        # is off, so without this, latching at B7 and then merely clicking B3
        # to look around would silently make B3 the new stage.
        stage = get_build_stage()
        if stage is not None:
            self.active_bar_id = stage[0]
            self.show_unbuilt = False
        # GUID of the temporary text-dot label drawn at the active bar's
        # end-point.  Recreated on every set_active(), cleaned up on exit.
        self._active_dot_id = None

    # ------------------------------------------------------------------
    # Active-bar end-point label

    def _clear_active_dot(self):
        if self._active_dot_id is not None and rs.IsObject(self._active_dot_id):
            rs.DeleteObject(self._active_dot_id)
        self._active_dot_id = None

    def _refresh_active_dot(self, bar_map=None):
        """Place a text-dot at the active bar's end-point showing
        ``bar_id`` and assembly sequence number.  This is the same end of
        the bar where the physical label will be applied."""
        self._clear_active_dot()
        if self.active_bar_id is None:
            return
        if bar_map is None:
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
            # Tag + park before storing the id: if anything below throws, the
            # sweep can still find this dot.  A dot born on the current layer
            # with no tag is the leak this exists to prevent.
            ensure_layer(config.LAYER_DIAGNOSTIC_MARKS)
            rs.ObjectLayer(dot_id, config.LAYER_DIAGNOSTIC_MARKS)
            rs.SetUserText(dot_id, _SEQ_DOT_KEY, self.active_bar_id)
            self._active_dot_id = dot_id

    # ------------------------------------------------------------------
    # Helpers

    # Every helper below takes an optional *bar_map* -- a
    # ``get_bar_seq_map()`` snapshot of "which bar sits at which step".  That
    # call walks every object in the document, so one user click used to pay
    # for it five times over.  A caller that already holds a snapshot passes it
    # down; the default ``None`` means "fetch a fresh one".
    #
    # The default MUST stay None: do_move_earlier / do_move_later /
    # insert_bar_after CHANGE the step numbers, so a snapshot taken before them
    # is stale and would paint and label bars with their old steps.  Forgetting
    # to pass one costs a scan; passing a stale one shows the wrong thing.

    def _sorted_bars(self, bar_map=None):
        """Return [(bar_id, oid, seq)] in ascending sequence order."""
        if bar_map is None:
            bar_map = get_bar_seq_map()
        return sorted(
            [(bid, oid, seq) for bid, (oid, seq) in bar_map.items()],
            key=lambda x: x[2],
        )

    def _bar_count(self, bar_map=None):
        if bar_map is None:
            bar_map = get_bar_seq_map()
        return len(bar_map)

    def _active_seq(self, bar_map=None):
        if self.active_bar_id is None:
            return None
        if bar_map is None:
            bar_map = get_bar_seq_map()
        entry = bar_map.get(self.active_bar_id)
        return entry[1] if entry else None

    def _print_status(self, bar_map=None):
        if bar_map is None:
            bar_map = get_bar_seq_map()
        seq = self._active_seq(bar_map)
        total = self._bar_count(bar_map)
        unbuilt_note = (
            "shown" if self.show_unbuilt else "hidden (persists after exit)"
        )
        print(
            f"Step {seq} of {total}  ({self.active_bar_id})"
            f"  |  unbuilt bars: {unbuilt_note}"
        )

    # ------------------------------------------------------------------
    # State transitions

    def set_active(self, bar_id, bar_map=None):
        """Make *bar_id* the active step: repaint, latch, relabel, report.

        Takes one ``get_bar_seq_map()`` snapshot and hands it to all four, so a
        click costs one document scan instead of five.  Callers that have just
        reordered the sequence must NOT pass a snapshot -- see the note above
        the helpers.
        """
        if bar_map is None:
            bar_map = get_bar_seq_map()
        self.active_bar_id = bar_id
        show_sequence_colors(bar_id, self.show_unbuilt, bar_map=bar_map)
        # While unbuilt bars are hidden, the active step IS the build stage, so
        # keep the document latch in step with the screen -- Next / Previous /
        # typed jumps all land here.  When unbuilt bars are shown there is no
        # latch to keep in step, and writing one would create it behind the
        # user's back.
        if not self.show_unbuilt:
            seq = self._active_seq(bar_map)
            if seq is not None:
                set_build_stage(bar_id, seq)
        self._refresh_active_dot(bar_map)
        self._print_status(bar_map)

    def select_next(self):
        bar_map = get_bar_seq_map()
        bars = self._sorted_bars(bar_map)
        if not bars:
            return
        seq = self._active_seq(bar_map) or 0
        for bar_id, oid, s in bars:
            if s > seq:
                self.set_active(bar_id, bar_map)
                return

    def select_previous(self):
        bar_map = get_bar_seq_map()
        bars = self._sorted_bars(bar_map)
        if not bars:
            return
        seq = self._active_seq(bar_map)
        if seq is None:
            seq = bars[-1][2] + 1
        for bar_id, oid, s in reversed(bars):
            if s < seq:
                self.set_active(bar_id, bar_map)
                return

    def select_by_step(self, step):
        """Make the bar at sequence number *step* (1-based) active."""
        bar_map = get_bar_seq_map()
        for bar_id, _oid, seq in self._sorted_bars(bar_map):
            if seq == step:
                self.set_active(bar_id, bar_map)
                return True
        print(f"RSSequenceEdit: No bar at step {step}.")
        return False

    def select_by_bar_id(self, bar_id):
        """Make the bar with id *bar_id* (e.g. ``"B5"``) active."""
        bar_map = get_bar_seq_map()
        if bar_id in bar_map:
            self.set_active(bar_id, bar_map)
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
            # highlight_supports=False so the SAVED set is not painted too --
            # `selected` is the set being edited, and a bar just toggled off
            # would otherwise stay purple and look as if it were still in.
            show_sequence_colors(self.active_bar_id, True, highlight_supports=False)
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
        """Toggle unbuilt visibility AND the persistent build-stage latch.

        This is the only place the latch is created or destroyed by user
        intent.  Everything else either follows it (``set_active``) or merely
        re-asserts it (``apply_build_stage_visibility``).
        """
        # Toggling never reorders, so one snapshot serves the whole call.
        bar_map = get_bar_seq_map()
        self.show_unbuilt = not self.show_unbuilt
        if self.show_unbuilt:
            # ShowUnbuilt -- un-latch.  clear_build_stage() only drops the key;
            # the show_sequence_colors() below does the repainting.
            clear_build_stage()
        elif self.active_bar_id is not None:
            seq = self._active_seq(bar_map)
            if seq is not None:
                set_build_stage(self.active_bar_id, seq)
        if self.active_bar_id is not None:
            show_sequence_colors(
                self.active_bar_id, self.show_unbuilt, bar_map=bar_map
            )
        self._print_status(bar_map)


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

    # Collect any label dots a previous run left behind before adding our own.
    n_stray = _sweep_sequence_dots()
    if n_stray:
        print(f"RSSequenceEdit (startup): removed {n_stray} stray label dot(s).")

    bar_map = get_bar_seq_map()
    if not bar_map:
        print("RSSequenceEdit: No registered bars found. Run RSCreateBar first.")
        return

    session = _SequenceSession()
    # __init__ may have resumed a latched build stage.  Paint it now so the
    # stage view and status line are up before the first prompt.  The guard
    # matters: the staged bar may have been deleted since it was latched, in
    # which case repair_on_entry has already fallen back or cleared the key,
    # and set_active() on a missing bar would write a bogus stage.
    if session.active_bar_id is not None:
        if session.active_bar_id in bar_map:
            session.set_active(session.active_bar_id, bar_map)
        else:
            # Belt and braces: repair_on_entry above should already have made
            # the key resolve (or dropped it), so this is unreachable in
            # practice.  Fall back to a clean unlatched session rather than
            # carrying a bar id that no longer exists.
            session.active_bar_id = None
            session.show_unbuilt = True

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
        # reset_sequence_colors() re-applies a latched build stage as its last
        # step, so "restored" means colours only when one is set.
        #
        # Sweep as well as clear: _clear_active_dot only knows the one id it is
        # still holding, and this block also runs on the exception path, where
        # that id is exactly what may have been lost.
        session._clear_active_dot()
        _sweep_sequence_dots()
        reset_sequence_colors()
        if session.show_unbuilt:
            print("RSSequenceEdit: Finished. Display restored.")
        else:
            print(
                f"RSSequenceEdit: Finished. Colours restored; bars later than "
                f"{session.active_bar_id} stay HIDDEN until ShowUnbuilt."
            )


if __name__ == "__main__":
    main()
