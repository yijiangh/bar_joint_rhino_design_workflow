#! python 3
# venv: scaffolding_env
"""RSSequenceEdit - Interactive assembly sequence viewer and editor.

Displays all registered bars colour-coded by assembly step and lets the user:

  - **Click a bar** to make it the active (current) step.
  - **Next / Previous** to walk through the sequence one step at a time.
  - **MoveEarlier / MoveLater** to swap the active bar one step back or forward
    in the assembly order without leaving the tool.
  - **PickBarAfterThis** to pick any other bar and insert it immediately after
    the current bar in the sequence.
  - **ShowUnbuilt / HideUnbuilt** to toggle the visibility of bars that come
    later than the current step, giving an instant "assembly stage" preview.
  - **Finish** (or Escape) to restore normal display and exit.

Colour legend
-------------
- Green — already assembled (earlier in sequence)
- Blue  — current step (active bar)
- Grey  — not yet assembled (later in sequence, when shown)
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
    BAR_TYPE_KEY,
    BAR_TYPE_VALUE,
    get_bar_seq_map,
    move_bar_earlier,
    move_bar_later,
    insert_bar_after,
    repair_on_entry,
    show_sequence_colors,
    reset_sequence_colors,
)


# ---------------------------------------------------------------------------
# Geometry filter — only accept registered scaffolding bars
# ---------------------------------------------------------------------------


def _bar_filter(rhino_object, geometry, component_index):
    return rs.GetUserText(rhino_object.Id, BAR_TYPE_KEY) == BAR_TYPE_VALUE


# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------


class _SequenceSession:
    """Holds mutable UI state for one RSSequenceEdit run."""

    def __init__(self):
        self.active_bar_id = None
        self.show_unbuilt = True

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
        go2.SetCustomGeometryFilter(_bar_filter)
        result2 = go2.Get()
        if result2 != Rhino.Input.GetResult.Object:
            return
        picked_id = go2.Object(0).ObjectId
        picked_bar_id = rs.GetUserText(picked_id, BAR_ID_KEY)
        if not picked_bar_id:
            return
        if picked_bar_id == self.active_bar_id:
            print("RSSequenceEdit: Cannot move a bar after itself.")
            return
        insert_bar_after(picked_bar_id, self.active_bar_id)
        show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()

    def toggle_unbuilt(self):
        self.show_unbuilt = not self.show_unbuilt
        if self.active_bar_id is not None:
            show_sequence_colors(self.active_bar_id, self.show_unbuilt)
        self._print_status()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


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
    # Default Enter action — walks forward through the sequence.
    last_action = "Next"

    while True:
        # Create a fresh GetObject each iteration to avoid pre-selection
        # re-triggering from the previous pick (which would cause an infinite loop).
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(f"Select a bar  (Enter = {last_action})")
        go.EnablePreSelect(False, False)
        go.AcceptNothing(True)  # allow bare Enter to repeat last action
        go.SetCustomGeometryFilter(_bar_filter)

        go.AddOption("Next")
        go.AddOption("Previous")
        go.AddOption("MoveEarlier")
        go.AddOption("MoveLater")
        go.AddOption("PickBarAfterThis")
        if session.show_unbuilt:
            go.AddOption("HideUnbuilt")
        else:
            go.AddOption("ShowUnbuilt")
        go.AddOption("Finish")

        result = go.Get()

        if result == Rhino.Input.GetResult.Cancel:
            break

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            bar_id = rs.GetUserText(picked_id, BAR_ID_KEY)
            if bar_id:
                session.set_active(bar_id)
            # Picking a bar doesn't change the repeat action.
            continue

        if result == Rhino.Input.GetResult.Nothing:
            # Bare Enter — repeat last action.
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

    # Restore normal display on exit
    reset_sequence_colors()
    print("RSSequenceEdit: Finished. Display restored.")


if __name__ == "__main__":
    main()
