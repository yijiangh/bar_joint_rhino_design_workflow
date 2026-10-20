"""Sort the buttons of every tab in ``scaffolding_toolbar.rui`` by command name.

A tab is a ``<tool_bar>``; its buttons are ``<tool_bar_item>`` blocks, listed in
display order.  A button does not hold its command: it points at a
``<macro_item>`` by guid (``<left_macro_id>``, and ``<right_macro_id>`` for the
right-click), and the macro's ``<text>`` is the command name, e.g. ``RSBarSnap``
(docs/note.md section 8).  This tool reorders the ``<tool_bar_item>`` blocks
inside each tab alphabetically by their **left-click** command name, ignoring
case, so each family sits together: ``RSBar*``, ``RSExport*``, ``RSJoint*`` ...

Only whole ``<tool_bar_item>`` blocks move.  A right-click command stays on its
button, and no guid, macro, icon or tooltip changes.  Every byte outside the
moved blocks, including the BOM and the CRLF line endings, is kept, so the diff
is a pure move.  Running it twice gives the same file.

**Run it with Rhino closed.**  Rhino keeps the toolbar file in memory and writes
it back when it closes, so a sort made while Rhino is open is overwritten.  For
the same reason, re-run it whenever Rhino has rewritten the file in the old
order.

Usage (from the repo root, any Python 3.8+, no third-party packages)::

    python scripts/sort_toolbar_buttons.py            # sort in place
    python scripts/sort_toolbar_buttons.py --check    # report only; exit 1 if unsorted
"""

from __future__ import annotations

import argparse
import os
import re
import sys


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUI_PATH = os.path.join(REPO_ROOT, "scaffolding_toolbar.rui")

_LOCALE_TAG = "locale_1033"

#: A macro's guid and its command name (the first ``<text>`` inside it).
_MACRO_RE = re.compile(
    r'<macro_item\s+guid="(?P<guid>[^"]+)"[^>]*>\s*<text>\s*'
    r"<" + _LOCALE_TAG + r">(?P<name>[^<]*)</" + _LOCALE_TAG + r">",
    re.S,
)
#: One whole tab, opening tag to closing tag.
_TOOLBAR_RE = re.compile(r"<tool_bar\s.*?</tool_bar>", re.S)
#: The tab's own name, the first ``<text>`` inside the tab.
_TOOLBAR_NAME_RE = re.compile(
    r"<text>\s*<" + _LOCALE_TAG + r">(?P<name>[^<]*)</" + _LOCALE_TAG + r">", re.S
)
#: One button, opening tag to closing tag.
_ITEM_RE = re.compile(r"<tool_bar_item\s.*?</tool_bar_item>", re.S)
_LEFT_RE = re.compile(r"<left_macro_id>\s*([^<\s]+)\s*</left_macro_id>")

#: Sorts buttons whose left-click macro is missing after all the named ones.
_UNKNOWN = "￿"


def macro_names(text):
    """Map every macro guid to its command name.

    Args:
        text (str): the whole .rui file.

    Returns:
        dict[str, str]: ``{macro guid: command name}``.
    """
    return {m.group("guid"): m.group("name").strip() for m in _MACRO_RE.finditer(text)}


def button_name(item, names):
    """The left-click command name of one ``<tool_bar_item>`` block, or ``None``."""
    left = _LEFT_RE.search(item)
    return names.get(left.group(1)) if left else None


def sort_tab(block, names):
    """Reorder the buttons of one ``<tool_bar>`` block.

    Args:
        block (str): the tab's text, ``<tool_bar ...>`` to ``</tool_bar>``.
        names (dict[str, str]): from :func:`macro_names`.

    Returns:
        tuple[str, list[str], list[str]]: the new block text, and the button
        names before and after (a missing macro shows as ``"?"``).

    Raises:
        ValueError: if anything other than whitespace sits between two
            buttons, since reordering would then move it too.
    """
    items = list(_ITEM_RE.finditer(block))
    before = [button_name(m.group(0), names) or "?" for m in items]
    if len(items) < 2:
        return block, before, before
    gaps = [block[a.end():b.start()] for a, b in zip(items, items[1:])]
    if any(gap.strip() for gap in gaps):
        raise ValueError("something other than whitespace sits between two buttons")

    def key(item_text):
        name = button_name(item_text, names)
        return name.lower() if name else _UNKNOWN

    ordered = sorted((m.group(0) for m in items), key=key)  # stable on ties
    pieces = [block[:items[0].start()]]
    for index, item_text in enumerate(ordered):
        pieces.append(item_text)
        if index < len(gaps):
            pieces.append(gaps[index])
    pieces.append(block[items[-1].end():])
    after = [button_name(t, names) or "?" for t in ordered]
    return "".join(pieces), before, after


def sort_rui(text):
    """Sort the buttons of every tab in *text*.

    Args:
        text (str): the whole .rui file.

    Returns:
        tuple[str, list[tuple[str, list[str], list[str]]]]: the new file text,
        and per tab ``(tab name, names before, names after)``.
    """
    names = macro_names(text)
    report = []

    def _replace(match):
        block = match.group(0)
        title = _TOOLBAR_NAME_RE.search(block)
        tab = title.group("name").strip() if title else "?"
        try:
            new_block, before, after = sort_tab(block, names)
        except ValueError as exc:
            raise ValueError(f"tab {tab}: {exc}") from None
        report.append((tab, before, after))
        return new_block

    return _TOOLBAR_RE.sub(_replace, text), report


def main(argv=None):
    """Sort the toolbar file in place, or report with ``--check``.

    Returns:
        int: 0 when the file is sorted (or was just sorted); 1 with
        ``--check`` when it is not, or when the file cannot be sorted safely.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="report only, write nothing")
    parser.add_argument("--path", default=RUI_PATH, help="the .rui file (default: the repo's)")
    args = parser.parse_args(argv)

    # newline="" keeps the CRLF line endings; utf-8 keeps the BOM as a character.
    with open(args.path, "r", encoding="utf-8", newline="") as stream:
        text = stream.read()
    try:
        updated, report = sort_rui(text)
    except ValueError as exc:
        print(f"sort_toolbar_buttons: cannot sort safely ({exc}); nothing written.")
        return 1

    for tab, before, after in report:
        state = "already sorted" if before == after else "re-ordered"
        print(f"  [{tab}] {state}: {', '.join(after)}")
        if "?" in after:
            print(f"    WARNING: a button in [{tab}] points at a macro that does not exist.")

    if updated == text:
        print(f"sort_toolbar_buttons: sorted ({args.path}).")
        return 0
    if args.check:
        print(f"sort_toolbar_buttons: NOT sorted ({args.path}); run without --check.")
        return 1
    with open(args.path, "w", encoding="utf-8", newline="") as stream:
        stream.write(updated)
    print(f"sort_toolbar_buttons: wrote {args.path}. Close and reopen the toolbar in Rhino.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
