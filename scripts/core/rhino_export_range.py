"""Command-line prompt for the bar range of a partial export (From .. Until).

Used by the right-click of the RSExportBarAction button
(``rs_export_all_bar_actions.py``). Pressing Enter right away exports every
bar, as before. The ``FromBar`` / ``UntilBar`` options let you pick the first
and the last bar to export, in two steps each:

1. click a bar in the viewport, or type its id (``B5``) or its step number
   (``5``, the number RSSequenceEdit shows);
2. the viewport shows the scene built up to and including that bar (earlier
   bars green, the picked bar blue, later bars hidden); Accept it, or Repick.

What a range means for the exported files is decided in
``core.export_subset`` and the exporter; this module only asks.
"""

from __future__ import annotations

import Rhino
import rhinoscriptsyntax as rs

from core.rhino_bar_pick import bar_or_tube_filter, resolve_picked_to_bar_curve
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    get_build_stage,
    reset_sequence_colors,
    show_sequence_colors,
)


# * ---------------------------------------------------------------- typed input


def parse_bar_selector(text: str, seq_map: dict) -> tuple:
    """Turn a typed bar id or step number into a bar id of ``seq_map``.

    Accepted: the bar id itself (``B5``; the case of the letter does not
    matter) or a bare step number (``5``, the ``bar_seq`` number RSSequenceEdit
    shows). Same rules as ``rs_sequence_edit._parse_typed_selector``, which is
    private to that script.

    Args:
        text (str): what the user typed.
        seq_map (dict): ``{bar_id: (oid, seq)}`` of the bars that may be picked
            (the real bars).

    Returns:
        tuple: ``(bar_id, None)`` on success, ``(None, message)`` otherwise.
    """
    text = (text or "").strip()
    if not text:
        return None, "Nothing typed."
    # An exact id first (also covers ids that do not follow the B<n> pattern).
    if text in seq_map:
        return text, None
    if text[0] in ("b", "B") and text[1:].isdigit():
        bar_id = "B" + text[1:]
        if bar_id in seq_map:
            return bar_id, None
        return None, f"{bar_id} is not a real registered bar (deleted, or a fake staging bar)."
    if text.lstrip("+").isdigit():
        step = int(text)
        for bar_id, (_oid, seq) in seq_map.items():
            if int(seq) == step:
                return bar_id, None
        return None, f"No real bar at step {step} (a fake staging bar, or out of range)."
    return None, f"Could not read {text!r}: type a bar id like B5 or a step number like 5."


# * ---------------------------------------------------------------- one end of the range


def _bar_label(bar_id: str, seq_map: dict) -> str:
    """A bar's id with its step number, for prompts and messages.

    Args:
        bar_id (str): the bar.
        seq_map (dict): ``{bar_id: (oid, seq)}``.

    Returns:
        str: e.g. ``"B5 (step 5)"``.
    """
    return f"{bar_id} (step {int(seq_map[bar_id][1])})"


def _get_bar_click_or_text(prompt: str, seq_map: dict):
    """One pick: a clicked bar or a typed id / step number.

    Args:
        prompt (str): the command-line prompt.
        seq_map (dict): ``{bar_id: (oid, seq)}`` of the bars that may be picked.

    Returns:
        str | None: the picked bar id, ``""`` when the input was not a usable
        bar (the reason is printed; ask again), or ``None`` on Esc.
    """
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    # No pre-selection: a bar still selected from earlier would be returned
    # at once without the user clicking anything.
    go.EnablePreSelect(False, False)
    go.AcceptString(True)
    go.SetCustomGeometryFilter(bar_or_tube_filter)
    result = go.Get()
    if result == Rhino.Input.GetResult.Cancel:
        return None
    if result == Rhino.Input.GetResult.Object:
        curve_id = resolve_picked_to_bar_curve(go.Object(0).ObjectId)
        rs.UnselectAllObjects()
        bar_id = rs.GetUserText(curve_id, BAR_ID_KEY) if curve_id is not None else None
        if not bar_id:
            print("  That object is not a registered bar.")
            return ""
        if bar_id not in seq_map:
            print(f"  {bar_id} is a fake staging bar; the robot never builds it. Pick a real bar.")
            return ""
        return bar_id
    if result == Rhino.Input.GetResult.String:
        bar_id, message = parse_bar_selector(go.StringResult(), seq_map)
        if bar_id is None:
            print(f"  {message}")
            return ""
        return bar_id
    return ""


def _ask_accept(prompt: str):
    """Ask Accept / Repick on the command line (Enter = Accept).

    Args:
        prompt (str): the command-line prompt.

    Returns:
        bool | None: True to accept, False to pick again, None on Esc.
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(prompt)
    idx_accept = go.AddOption("Accept")
    go.AddOption("Repick")
    go.SetCommandPromptDefault("Accept")
    go.AcceptNothing(True)
    result = go.Get()
    if result == Rhino.Input.GetResult.Cancel:
        return None
    if result == Rhino.Input.GetResult.Nothing:
        return True
    if result == Rhino.Input.GetResult.Option:
        return go.OptionIndex() == idx_accept
    return None


def pick_range_bar(label: str, seq_map: dict):
    """Pick one end of the range, preview the scene built up to it, confirm.

    Args:
        label (str): "From" or "Until", for the prompts.
        seq_map (dict): ``{bar_id: (oid, seq)}`` of the real bars.

    Returns:
        str | None: the accepted bar id, or ``None`` on Esc (the caller keeps
        its previous value).
    """
    try:
        while True:
            # Everything visible again before each pick: a bar hidden by the
            # previous preview cannot be clicked.
            reset_sequence_colors()
            rs.Redraw()
            bar_id = _get_bar_click_or_text(
                f"{label} bar: click a bar, or type its id (B5) or step number", seq_map,
            )
            if bar_id is None:
                return None
            if bar_id == "":
                continue
            # The built scene at the end of this bar's step: earlier bars
            # green, this bar blue, later bars and their joint halves hidden.
            # Supports are not forced visible (they may be later bars).
            show_sequence_colors(bar_id, show_unbuilt=False, highlight_supports=False)
            rs.Redraw()
            answer = _ask_accept(
                f"{label} = {_bar_label(bar_id, seq_map)}: the scene built up to it is shown"
            )
            if answer is None:
                return None
            if answer:
                return bar_id
            # Repick: loop round.
    finally:
        # Back to the normal view (re-applies a latched build stage, if any).
        reset_sequence_colors()
        rs.Redraw()


# * ---------------------------------------------------------------- the whole range


def prompt_export_range(seq_map: dict, caller: str):
    """Ask which bars to export. Enter right away = every bar.

    Args:
        seq_map (dict): ``{bar_id: (oid, seq)}`` of the real bars
            (``get_real_bar_seq_map``); must not be empty.
        caller (str): the command name, for the printed lines.

    Returns:
        tuple | None: ``(from_bar_id, until_bar_id)``, or ``None`` on Esc.
    """
    ordered = [bar_id for bar_id, _v in sorted(seq_map.items(), key=lambda kv: kv[1][1])]
    from_bar_id, until_bar_id = ordered[0], ordered[-1]

    # Bars hidden by a latched build stage (RSSequenceEdit > HideUnbuilt)
    # cannot be clicked; say how to reach them anyway.
    if get_build_stage() is not None:
        print(
            f"{caller}: a build stage is latched (HideUnbuilt), so later bars are "
            "hidden. Type their id or step number to pick them."
        )

    while True:
        is_all = (from_bar_id, until_bar_id) == (ordered[0], ordered[-1])
        n_bars = sum(
            1 for bar_id in ordered
            if seq_map[from_bar_id][1] <= seq_map[bar_id][1] <= seq_map[until_bar_id][1]
        )
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(
            f"Export {'ALL ' if is_all else ''}bars {_bar_label(from_bar_id, seq_map)} .. "
            f"{_bar_label(until_bar_id, seq_map)}: {n_bars} of {len(ordered)}"
        )
        idx_from = go.AddOption("FromBar")
        idx_until = go.AddOption("UntilBar")
        idx_all = go.AddOption("All")
        go.SetCommandPromptDefault("Export")
        go.AcceptNothing(True)
        result = go.Get()

        if result == Rhino.Input.GetResult.Cancel:
            return None
        if result == Rhino.Input.GetResult.Nothing:
            # Enter: export this range, if it runs forwards.
            if seq_map[from_bar_id][1] > seq_map[until_bar_id][1]:
                print(
                    f"{caller}: From {_bar_label(from_bar_id, seq_map)} comes after "
                    f"Until {_bar_label(until_bar_id, seq_map)}; pick them again."
                )
                continue
            return from_bar_id, until_bar_id
        if result != Rhino.Input.GetResult.Option:
            continue

        chosen = go.OptionIndex()
        if chosen == idx_from:
            picked = pick_range_bar("From", seq_map)
            if picked is not None:
                from_bar_id = picked
        elif chosen == idx_until:
            picked = pick_range_bar("Until", seq_map)
            if picked is not None:
                until_bar_id = picked
        elif chosen == idx_all:
            from_bar_id, until_bar_id = ordered[0], ordered[-1]
