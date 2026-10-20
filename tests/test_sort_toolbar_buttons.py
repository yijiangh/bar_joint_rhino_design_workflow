"""scripts/sort_toolbar_buttons.py on the repo's own toolbar file (read only)."""

import collections
import re

import pytest

import sort_toolbar_buttons as stb


@pytest.fixture(scope="module")
def rui_text():
    with open(stb.RUI_PATH, "r", encoding="utf-8", newline="") as stream:
        return stream.read()


def _items_by_tab(text):
    """``{tab name: [whole <tool_bar_item> text, ...]}`` in file order."""
    out = {}
    for block in stb._TOOLBAR_RE.findall(text):
        tab = stb._TOOLBAR_NAME_RE.search(block).group("name").strip()
        out[tab] = stb._ITEM_RE.findall(block)
    return out


def test_every_tab_ends_sorted_by_left_click_name(rui_text):
    updated, report = stb.sort_rui(rui_text)
    names = stb.macro_names(updated)
    for tab, items in _items_by_tab(updated).items():
        order = [stb.button_name(item, names) for item in items]
        assert None not in order, tab
        assert order == sorted(order, key=str.lower), tab
    assert {tab for tab, _, _ in report} == set(_items_by_tab(rui_text))


def test_buttons_are_moved_whole_never_changed(rui_text):
    updated, _ = stb.sort_rui(rui_text)
    before, after = _items_by_tab(rui_text), _items_by_tab(updated)
    assert before.keys() == after.keys()
    for tab in before:
        # Same blocks, byte for byte: right-click pairs and guids travel with their button.
        assert collections.Counter(before[tab]) == collections.Counter(after[tab]), tab


def test_nothing_outside_the_buttons_changes(rui_text):
    updated, _ = stb.sort_rui(rui_text)
    assert len(updated) == len(rui_text)
    strip = lambda t: stb._ITEM_RE.sub("@", t)  # noqa: E731
    assert strip(updated) == strip(rui_text)
    assert updated.count("\r\n") == rui_text.count("\r\n")
    assert updated[:1] == rui_text[:1]  # BOM kept when present


def test_sorting_twice_changes_nothing(rui_text):
    once, _ = stb.sort_rui(rui_text)
    twice, _ = stb.sort_rui(once)
    assert twice == once


def test_refuses_when_something_sits_between_buttons():
    text = (
        '<macro_item guid="m1"><text><locale_1033>RSB</locale_1033></text></macro_item>'
        '<macro_item guid="m2"><text><locale_1033>RSA</locale_1033></text></macro_item>'
        '<tool_bar guid="t"><text><locale_1033>Tab</locale_1033></text>'
        '<tool_bar_item guid="i1"><left_macro_id>m1</left_macro_id></tool_bar_item>'
        "<!-- a comment that would move -->"
        '<tool_bar_item guid="i2"><left_macro_id>m2</left_macro_id></tool_bar_item>'
        "</tool_bar>"
    )
    with pytest.raises(ValueError, match="Tab"):
        stb.sort_rui(text)


def test_small_tab_is_sorted_case_insensitively():
    text = (
        '<macro_item guid="m1"><text><locale_1033>RSbeta</locale_1033></text></macro_item>'
        '<macro_item guid="m2"><text><locale_1033>RSAlpha</locale_1033></text></macro_item>'
        '<tool_bar guid="t"><text><locale_1033>Tab</locale_1033></text>\n'
        '  <tool_bar_item guid="i1"><left_macro_id>m1</left_macro_id></tool_bar_item>\n'
        '  <tool_bar_item guid="i2"><left_macro_id>m2</left_macro_id>'
        "<right_macro_id>m1</right_macro_id></tool_bar_item>\n"
        "</tool_bar>"
    )
    updated, report = stb.sort_rui(text)
    assert report == [("Tab", ["RSbeta", "RSAlpha"], ["RSAlpha", "RSbeta"])]
    first = re.search(r'<tool_bar_item guid="(\w+)"', updated).group(1)
    assert first == "i2"
    assert "<right_macro_id>m1</right_macro_id></tool_bar_item>" in updated
