#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSSelectBar - Select bars by id, or a whole length group at once.

Two modes, chosen at the first prompt:

**SelectByName** (the default) - type a bar id like ``B4`` (case-insensitive; a
bare number like ``4`` also works) and this selects that bar's centerline curve +
tube preview and zooms to it, so you can locate one bar in a large model without
hunting for it by eye. Enter several ids separated by commas (``B4,B7``) to select
more than one at once. The prompt loops so you can jump from bar to bar -- each
entry REPLACES the previous selection; press Enter on an empty prompt (or Esc) to
finish.

**SelectByLength** - type a length in mm (``1050``) and select every bar of that
length, together with the **male and ground joints** on those bars. The summary printed
on entry lists each length, its bar count and its bar ids. The grouping and the
prompt are RSBarEdit's, so a length means the same set of bars in both commands.
The prompt loops until Enter/Esc.

Read-only: it never edits the document (it reads each bar's stored ``bar_id`` as
is, and does not heal / renumber anything), so it is safe to run any time. No
PyBullet needed.
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

import rs_bar_edit as _bar_edit_module
from core import config
from core import rhino_bar_registry as _registry_module
from core.rhino_bar_registry import BAR_ID_KEY, BAR_TYPE_KEY, BAR_TYPE_VALUE


# Command name used in every command-line message + dialog title.
CMD = "RSSelectBar"


def _reload():
    """Re-import the bar registry + RSBarEdit so ScriptEditor picks up changes.

    Rebinds the module-level ``registry`` / ``bar_edit`` globals to the freshly
    reloaded modules.  Matches the reload pattern used by the other RS* commands.
    """
    global registry, bar_edit
    registry = importlib.reload(_registry_module)
    bar_edit = importlib.reload(_bar_edit_module)


_reload()


def _normalize_bar_id(token: str):
    """Turn one raw user token into a canonical bar id like ``B4``.

    Accepts ``B4`` / ``b4`` / a bare number ``4``. Returns ``None`` when the token
    is empty or not a bar-id shape (so the caller can report it as unmatched).

    Args:
        token (str): one raw id typed by the user.

    Returns:
        str | None: the canonical ``B<n>`` id, or ``None`` if it can't be parsed.
    """
    text = (token or "").strip().upper()
    if not text:
        return None
    if text.startswith("B"):
        text = text[1:]
    if not text.isdigit():
        return None
    return f"B{int(text)}"


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


def _bar_joint_oids(bar_ids) -> list:
    """Male + ground joint instances whose ``parent_bar_id`` is in *bar_ids*.

    Those two halves belong to the bar they are placed on, so they are part of
    "that bar" the way its tube preview is.  The **female** half of a joint
    belongs to the *other* bar of the pair and is deliberately left alone.

    Args:
        bar_ids (list[str]): bar ids of the group being selected.

    Returns:
        list: joint block instance ids; layers that do not exist are skipped.
    """
    wanted = set(bar_ids)
    out = []
    for layer in (
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        out.extend(
            oid
            for oid in rs.ObjectsByLayer(layer) or []
            if rs.GetUserText(oid, "parent_bar_id") in wanted
        )
    return out


def _apply_selection(to_select) -> int:
    """Replace the current selection with *to_select* and zoom to it.

    Returns:
        int: how many objects ended up selected.
    """
    rs.UnselectAllObjects()
    if to_select:
        rs.SelectObjects(to_select)
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
        bar_id = _normalize_bar_id(token)
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
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("Select bars by typed id, or by length group")
    name_idx = go.AddOption("SelectByName")
    length_idx = go.AddOption("SelectByLength")
    go.SetCommandPromptDefault("SelectByName")
    go.AcceptNothing(True)
    while True:
        res = go.Get()
        if res == Rhino.Input.GetResult.Nothing:
            return "name"
        if res == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == name_idx:
                return "name"
            if chosen == length_idx:
                return "length"
            continue
        return None


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


def _run_select_by_length(bars) -> None:
    """Type a length in mm; select those bars + their male/ground joints, in a loop.

    The grouping, the prompt and the printed summary come straight from RSBarEdit
    (``build_length_groups`` / ``pick_length_group`` / ``print_length_summary``),
    so ``1050`` means the same set of bars in both commands.  Only the selection
    is this command's own, which keeps the read-only promise: RSBarEdit reaches
    the same groups through ``get_all_bars``, which heals bar ids as it goes.

    Args:
        bars (dict): ``{bar_id: [oid, ...]}`` from :func:`_scan_bars`.
    """
    # build_length_groups keys by a single oid per bar; a duplicate id is grouped
    # by its first oid, but every oid under that id is selected below.
    bar_map = {bar_id: oids[0] for bar_id, oids in bars.items()}
    groups, _colors, _lengths = bar_edit.build_length_groups(bar_map)
    if not groups:
        print(f"{CMD}: no bars to group by length.")
        return
    bar_edit.print_length_summary(groups)

    last_length_mm = None  # remembered as the default of the next prompt
    while True:
        group_idx = bar_edit.pick_length_group(
            groups, default_mm=last_length_mm, command=CMD
        )
        if group_idx is None:
            return  # Enter / Esc

        length_bin, bar_ids = groups[group_idx]
        last_length_mm = length_bin
        to_select = []
        for bar_id in bar_ids:
            to_select.extend(_oids_to_select(bars[bar_id]))
        joint_oids = _bar_joint_oids(bar_ids)
        to_select.extend(joint_oids)
        n = _apply_selection(to_select)
        print(
            f"{CMD}: selected {len(bar_ids)} bar(s) at {length_bin:.0f} mm "
            f"+ {len(joint_oids)} male/ground joint(s) ({n} object(s)): "
            f"{', '.join(bar_ids)}."
        )


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
