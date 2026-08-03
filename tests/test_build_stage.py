"""Tests for the pure build-stage resolution behind the persistent HideUnbuilt filter.

Run with ``python -m pytest tests/test_build_stage.py -v``.  See ``docs/Su_note.md``
section 19 for what pytest, ``assert`` and ``@pytest.mark.parametrize`` actually do.

``core.build_stage`` imports nothing from Rhino, so these run in a plain terminal with
no stubbing -- that is the whole reason the resolver lives outside
``core.rhino_bar_registry``, which does ``import Rhino`` at module level and therefore
cannot be imported here at all.
"""

import pytest

from core.build_stage import (
    STATUS_CLEAR,
    STATUS_FALLBACK,
    STATUS_OFF,
    STATUS_OK,
    format_build_stage,
    parse_build_stage,
    resolve_build_stage_seq,
    stage_filter_note,
)


def _bar_map(pairs):
    """Build a fake ``{bar_id: (oid, seq)}`` map from a list of ``(bar_id, seq)``.

    That is the shape ``rhino_bar_registry.get_bar_seq_map()`` returns in Rhino.  The
    oid is a throwaway string here: the resolver only ever reads ``entry[1]``, the
    sequence number, so it never notices these are not real Rhino GUIDs.
    """
    return {bar_id: (f"oid-{bar_id}", seq) for bar_id, seq in pairs}


#: A healthy five-bar document, numbered tight B1..B5 as repair_bar_sequences leaves it.
FIVE_BARS = _bar_map([("B1", 1), ("B2", 2), ("B3", 3), ("B4", 4), ("B5", 5)])


# ---------------------------------------------------------------------------
# format / parse round trip
# ---------------------------------------------------------------------------


def test_format_build_stage():
    assert format_build_stage("B7", 7) == "B7|7"


def test_round_trip():
    """Whatever format_build_stage writes, parse_build_stage must read back."""
    assert parse_build_stage(format_build_stage("B12", 3)) == ("B12", 3)


@pytest.mark.parametrize("raw", ["B7", "|", "B7|x", "", None, "B7|", "|7", "B7|7|7"])
def test_parse_rejects_garbage(raw):
    """A hand-edited or truncated value comes back as None, never as an exception."""
    assert parse_build_stage(raw) is None


# ---------------------------------------------------------------------------
# resolve_build_stage_seq -- the rule the whole filter rests on
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("raw", ["", None])
def test_not_latched(raw):
    status, seq, bar_id, message = resolve_build_stage_seq(raw, FIVE_BARS)
    assert status == STATUS_OFF
    assert (seq, bar_id, message) == (None, None, None)


def test_stage_bar_present():
    status, seq, bar_id, message = resolve_build_stage_seq("B3|3", FIVE_BARS)
    assert status == STATUS_OK
    assert (seq, bar_id) == (3, "B3")
    assert message is None


def test_renumbered_bar_uses_its_current_seq():
    """The id is the primary key: follow the physical bar, not the saved step.

    RSReorderBarID can move B3 to step 5 and put a different bar at step 3.  The
    filter must then hide past step 5 -- otherwise the stage silently jumps to
    whichever bar inherited the old number.
    """
    renumbered = _bar_map([("B1", 1), ("B2", 2), ("B3", 5), ("B4", 3), ("B5", 4)])
    status, seq, bar_id, _message = resolve_build_stage_seq("B3|3", renumbered)
    assert status == STATUS_OK
    assert (seq, bar_id) == (5, "B3")


def test_deleted_bar_falls_back_to_nearest_earlier_step():
    """RSRemoveBar on the staged bar: keep the view where it was, minus one step."""
    without_b4 = _bar_map([("B1", 1), ("B2", 2), ("B3", 3), ("B5", 5)])
    status, seq, bar_id, message = resolve_build_stage_seq("B4|4", without_b4)
    assert status == STATUS_FALLBACK
    assert (seq, bar_id) == (3, "B3")
    assert "B4" in message and "B3" in message


def test_fallback_never_jumps_forward():
    """Only bars at or before the saved step are candidates.

    Falling forward would reveal work the user had deliberately hidden, which is the
    one outcome the fallback exists to avoid.
    """
    only_later = _bar_map([("B8", 8), ("B9", 9)])
    status, seq, bar_id, message = resolve_build_stage_seq("B4|4", only_later)
    assert status == STATUS_CLEAR
    assert (seq, bar_id) == (None, None)
    assert "no bar remains at or before" in message


def test_stage_bar_deleted_from_empty_document():
    status, seq, bar_id, message = resolve_build_stage_seq("B4|4", {})
    assert status == STATUS_CLEAR
    assert (seq, bar_id) == (None, None)
    assert message


@pytest.mark.parametrize("raw", ["B7", "|", "B7|x", "nonsense"])
def test_unreadable_value_clears_without_raising(raw):
    status, seq, bar_id, message = resolve_build_stage_seq(raw, FIVE_BARS)
    assert status == STATUS_CLEAR
    assert (seq, bar_id) == (None, None)
    assert "unreadable" in message


def test_fallback_accepts_a_bar_sitting_exactly_on_the_saved_step():
    renamed = _bar_map([("B1", 1), ("BX", 4), ("B5", 5)])
    status, seq, bar_id, _message = resolve_build_stage_seq("B4|4", renamed)
    assert status == STATUS_FALLBACK
    assert (seq, bar_id) == (4, "BX")


# ---------------------------------------------------------------------------
# The shared reminder
# ---------------------------------------------------------------------------


def test_stage_filter_note_names_the_stage_and_the_way_out():
    note = stage_filter_note("RSJointPlace", "B7", 7)
    assert "RSJointPlace" in note
    assert "B7" in note and "step 7" in note
    assert "ShowUnbuilt" in note


def test_stage_filter_note_without_a_caller():
    assert stage_filter_note(None, "B7", 7).startswith("RSScaffolding")
