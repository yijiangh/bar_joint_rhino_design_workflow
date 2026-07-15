#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSSelectBar - Select a bar by typing its id.

Type a bar id like ``B4`` (case-insensitive; a bare number like ``4`` also works)
and this selects that bar's centerline curve + tube preview and zooms to it, so
you can locate one bar in a large model without hunting for it by eye. Enter
several ids separated by commas (``B4,B7``) to select more than one at once. The
prompt loops so you can jump from bar to bar -- each entry REPLACES the previous
selection; press Enter on an empty prompt (or Esc) to finish.

Read-only: it never edits the document (it reads each bar's stored ``bar_id`` as
is, and does not heal / renumber anything), so it is safe to run any time. No
PyBullet needed.
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import rhino_bar_registry as _registry_module
from core.rhino_bar_registry import BAR_ID_KEY, BAR_TYPE_KEY, BAR_TYPE_VALUE


# Command name used in every command-line message + dialog title.
CMD = "RSSelectBar"


def _reload():
    """Re-import the bar registry so ScriptEditor picks up code changes.

    Rebinds the module-level ``registry`` global to the freshly reloaded module.
    Matches the reload pattern used by the other RS* commands.
    """
    global registry
    registry = importlib.reload(_registry_module)


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

    rs.UnselectAllObjects()
    if to_select:
        rs.SelectObjects(to_select)
        try:
            rs.ZoomSelected()
        except Exception:
            pass  # zoom is a convenience -- never let it break the selection
    if found_ids:
        print(f"{CMD}: selected {', '.join(found_ids)} ({len(to_select)} object(s)).")
    if missing:
        print(f"{CMD}: no bar found for: {', '.join(missing)}.")
    return len(found_ids)


def main() -> None:
    """Prompt for bar id(s) and select the matching bar(s), in a loop."""
    _reload()

    bars = _scan_bars()
    if not bars:
        rs.MessageBox("No registered bars found in this document.", 0, CMD)
        return

    # Loop so the user can jump from bar to bar. Each entry replaces the previous
    # selection; an empty entry / Esc ends the command.
    while True:
        raw = rs.GetString(
            "Bar id to select (e.g. B4; comma-separated for several; Enter to finish)"
        )
        if not raw or not raw.strip():
            break
        tokens = [tok for tok in raw.split(",") if tok.strip()]
        _select_bars(tokens, bars)
        # A bar may have been added / deleted between iterations; refresh cheaply.
        bars = _scan_bars()


if __name__ == "__main__":
    main()
