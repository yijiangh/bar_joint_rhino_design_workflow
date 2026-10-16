#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSBarSelect - Select bars by id, or a whole length group at once.

Two modes, chosen at the first prompt:

**SelectByName** (the default) - type a bar id like ``B4`` (case-insensitive; a
bare number like ``4`` also works) and this selects that bar's centerline curve +
tube preview and zooms to it, so you can locate one bar in a large model without
hunting for it by eye. Enter several ids separated by commas (``B4,B7``) to select
more than one at once. The prompt loops so you can jump from bar to bar -- each
entry REPLACES the previous selection; press Enter on an empty prompt (or Esc) to
finish.

**SelectByLength** - every bar is colored by its length group and tagged with
its length while you choose; the summary printed on entry lists each length, its
bar count and its bar ids.  Type a length in mm (``1050``) to select every bar of
that length.  Then, optionally, also select their joints:

  - ``BearingJoints``: the tool-bearing halves on those bars (Male and Ground).
  - ``PairedJoints``: both halves of every joint pair touching those bars -- the
    Male and its Female / MoCap receiver, whichever bar each sits on.

Type another length to switch groups; Enter or Esc finishes and keeps the
selection.  The grouping is shared with RSBarEdit (``core.bar_length_groups``),
so a length means the same set of bars in both commands.

The model is never edited: bar ids are read as stored (nothing is healed or
renumbered), and the length colors and tags are removed on exit, with every
bar's previous color restored.  No PyBullet needed.
"""

from __future__ import annotations

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import bar_length_groups as _length_groups_module
from core import joint_name_conventions as jnc
from core import rhino_bar_registry as _registry_module
from core.rhino_bar_registry import BAR_ID_KEY, BAR_TYPE_KEY, BAR_TYPE_VALUE
from core.rhino_helpers import ask_option, objects_on_layers


# Command name used in every command-line message + dialog title.
CMD = "RSBarSelect"


def _reload():
    """Re-import the bar registry + length groups so ScriptEditor picks up changes.

    Rebinds the module-level ``registry`` / ``length_groups`` globals to the
    freshly reloaded modules.  Matches the reload pattern of the other commands.
    """
    global registry, length_groups
    registry = importlib.reload(_registry_module)
    length_groups = importlib.reload(_length_groups_module)


_reload()


def _scan_bars() -> dict:
    """Read-only scan of the document for every registered bar, keyed by id.

    Reads each bar's stored ``bar_id`` as-is -- unlike
    ``rhino_bar_registry.get_all_bars`` it does NOT call ``ensure_bar_id``, so
    running this command never renumbers or moves anything. A copy-pasted bar that
    still carries a duplicate id therefore shows up as two oids under the same id,
    and both get selected.

    Returns:
        dict: ``{bar_id: [oid, ...]}`` for every registered bar centerline.
    """
    out = {}
    for oid in rs.AllObjects() or []:
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE:
            continue
        bar_id = rs.GetUserText(oid, BAR_ID_KEY)
        if bar_id:
            out.setdefault(bar_id, []).append(oid)
    return out


def _oids_to_select(bar_oids) -> list:
    """Expand each bar centerline oid into itself plus its tube preview (if any).

    The centerline curve is thin and easy to miss; adding the tube makes the
    selection highlight obvious in the viewport.

    Args:
        bar_oids (list): bar centerline object ids to expand.

    Returns:
        list: centerline ids plus any matching tube-preview ids.
    """
    ids = []
    for oid in bar_oids:
        ids.append(oid)
        tube = registry._find_existing_tube(oid)
        if tube is not None and rs.IsObject(tube):
            ids.append(tube)
    return ids


def _bearing_joint_oids(bar_ids) -> list:
    """Male and Ground blocks whose ``parent_bar_id`` is in *bar_ids*.

    The tool-bearing halves: they belong to the bar they sit on.
    """
    wanted = set(bar_ids)
    return [
        oid
        for oid in objects_on_layers(*jnc.TOOL_BEARING_LAYERS)
        if rs.GetUserText(oid, jnc.UT_PARENT_BAR) in wanted
    ]


def _paired_joint_oids(bar_ids) -> list:
    """Both halves of every joint pair with a half on one of *bar_ids*.

    The Male and its Female / MoCap receiver, whichever bar each sits on.
    Ground and standalone MoCap joints are not pairs and are left out.
    """
    wanted = set(bar_ids)
    halves = []  # (oid, joint_id) of every paired half in the document
    joint_ids = set()
    for layer in jnc.PAIRED_LAYERS:
        subtype = jnc.subtype_of_layer(layer)
        for oid in objects_on_layers(layer):
            jid = rs.GetUserText(oid, jnc.UT_JOINT_ID)
            if not jid or not jnc.is_paired_half(subtype, jid):
                continue
            halves.append((oid, jid))
            if rs.GetUserText(oid, jnc.UT_PARENT_BAR) in wanted:
                joint_ids.add(jid)
    return [oid for oid, jid in halves if jid in joint_ids]


def _apply_selection(to_select, zoom=True) -> int:
    """Replace the current selection with *to_select* (and zoom to it).

    Returns:
        int: how many objects ended up selected.
    """
    rs.UnselectAllObjects()
    if to_select:
        rs.SelectObjects(to_select)
        if zoom:
            try:
                rs.ZoomSelected()
            except Exception:
                pass  # zoom is a convenience -- never let it break the selection
    return len(to_select)


def _select_bars(tokens, bars) -> int:
    """Select every bar named by ``tokens``; report matches + misses.

    Replaces the current selection (unselect-all first), selects the matched
    bars + their tubes, and zooms to them.

    Args:
        tokens (list[str]): raw id tokens typed by the user (already comma-split).
        bars (dict): ``{bar_id: [oid, ...]}`` from :func:`_scan_bars`.

    Returns:
        int: how many distinct bar ids were found and selected.
    """
    to_select = []
    found_ids = []
    missing = []
    for token in tokens:
        bar_id = jnc.parse_bar_id(token)  # B4 / b4 / 4 -> B4; None if not a bar id
        if bar_id is None:
            missing.append(token.strip())
            continue
        oids = bars.get(bar_id)
        if not oids:
            missing.append(bar_id)
            continue
        found_ids.append(bar_id)
        to_select.extend(_oids_to_select(oids))

    _apply_selection(to_select)
    if found_ids:
        print(f"{CMD}: selected {', '.join(found_ids)} ({len(to_select)} object(s)).")
    if missing:
        print(f"{CMD}: no bar found for: {', '.join(missing)}.")
    return len(found_ids)


def _ask_mode():
    """Ask for SelectByName or SelectByLength.

    Returns:
        str | None: ``"name"`` / ``"length"``, or ``None`` if the user cancelled.
    """
    choice = ask_option(
        "Select bars by typed id, or by length group",
        ("SelectByName", "SelectByLength"),
        default="SelectByName",
    )
    return {"SelectByName": "name", "SelectByLength": "length"}.get(choice)


def _run_select_by_name(bars) -> None:
    """Prompt for bar id(s) and select the matching bar(s), in a loop."""
    # Loop so the user can jump from bar to bar. Each entry replaces the previous
    # selection; an empty entry / Esc ends the command.
    while True:
        raw = rs.GetString(
            "Bar id to select (e.g. B4; comma-separated for several; Enter to finish)"
        )
        if not raw or not raw.strip():
            return
        tokens = [tok for tok in raw.split(",") if tok.strip()]
        _select_bars(tokens, bars)
        # A bar may have been added / deleted between iterations; refresh cheaply.
        bars = _scan_bars()


def _ask_length(groups, has_selection):
    """Typed length, ``BearingJoints`` / ``PairedJoints``, or ``None`` when done.

    Returns ``("length", group_index)``, ``("joints", option_name)`` or
    ``None`` (Enter or Esc).  A length with no group is reported with the
    lengths that exist and asked again.
    """
    while True:
        gn = Rhino.Input.Custom.GetNumber()
        gn.SetCommandPrompt(
            "Length (mm) of the bars to select"
            + (", or also select their joints" if has_selection else "")
            + " (Enter when done)"
        )
        gn.AcceptNothing(True)
        if has_selection:
            gn.AddOption("BearingJoints")
            gn.AddOption("PairedJoints")
        result = gn.Get()
        if result == Rhino.Input.GetResult.Option:
            return "joints", gn.Option().EnglishName
        if result != Rhino.Input.GetResult.Number:
            return None
        index = length_groups.find_length_group(groups, gn.Number())
        if index is not None:
            return "length", index
        print(
            f"{CMD}: no bars at {gn.Number():.0f} mm.  Available lengths: "
            f"{length_groups.available_lengths(groups)}."
        )


def _run_select_by_length(bars) -> None:
    """Color and tag bars by length; select a group, optionally with its joints.

    Args:
        bars (dict): ``{bar_id: [oid, ...]}`` from :func:`_scan_bars`.
    """
    # Grouped by each id's first oid; every oid under that id is selected below.
    bar_map = {bar_id: oids[0] for bar_id, oids in bars.items()}
    groups, color_by_bin, length_per_bar = length_groups.build_length_groups(bar_map)
    if not groups:
        print(f"{CMD}: no bars to group by length.")
        return
    length_groups.print_length_summary(groups)

    preview = length_groups.LengthPreview(
        bar_map, color_by_bin, length_per_bar, name_prefix="rsbarselect_dot"
    )
    bar_ids, bar_oids = [], []
    try:
        while True:
            answer = _ask_length(groups, has_selection=bool(bar_ids))
            if answer is None:
                return
            kind, value = answer
            if kind == "length":
                length_bin, bar_ids = groups[value]
                bar_oids = [o for b in bar_ids for o in _oids_to_select(bars[b])]
                n = _apply_selection(bar_oids)
                print(
                    f"{CMD}: selected {len(bar_ids)} bar(s) at {length_bin:.0f} mm "
                    f"({n} object(s)): {', '.join(bar_ids)}."
                )
                continue
            joint_oids = (
                _bearing_joint_oids(bar_ids) if value == "BearingJoints"
                else _paired_joint_oids(bar_ids)
            )
            _apply_selection(bar_oids + joint_oids, zoom=False)
            print(
                f"{CMD}: {len(bar_ids)} bar(s) + {len(joint_oids)} "
                f"{'bearing' if value == 'BearingJoints' else 'paired'} joint "
                "block(s) selected."
            )
    finally:
        preview.close()


def main() -> None:
    """Ask for the mode, then run SelectByName or SelectByLength."""
    _reload()

    bars = _scan_bars()
    if not bars:
        rs.MessageBox("No registered bars found in this document.", 0, CMD)
        return

    mode = _ask_mode()
    if mode is None:
        return
    if mode == "length":
        _run_select_by_length(bars)
        return
    _run_select_by_name(bars)


if __name__ == "__main__":
    main()
