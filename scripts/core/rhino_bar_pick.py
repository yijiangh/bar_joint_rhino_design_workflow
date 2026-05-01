"""Tube-aware bar picking primitives shared across RS* entry-points.

Scope is intentionally narrow: this module owns *only* the pieces that
multiple ``rs_*.py`` scripts genuinely share.  Command-specific pickers
that compose extra options (length, dual-pair, etc.) live in their own
entry-point file so the user-interaction code stays next to the command
that owns it.

Public API
----------

Geometry filter + resolver  (pick a bar centerline OR its tube preview)
    * :func:`bar_or_tube_filter`
    * :func:`resolve_picked_to_bar_curve`

Plain bar pick
    * :func:`pick_bar`
        Used by: ``rs_bar_snap``, ``rs_bar_brace``, ``rs_bar_subfloor``,
        ``rs_joint_place``.

Bar pick + inline ``Pair`` option
    * :func:`pick_bar_with_pair_option`
        Used by: ``rs_bar_snap``, ``rs_joint_place``.

Doc-userText backed defaults  (single owner of the persisted state)
    * :func:`get_default_pair_name` / :func:`set_default_pair_name`
    * :func:`get_default_brace_length` / :func:`set_default_brace_length`
    * :func:`get_default_subfloor_left_pair_name`  / setter
    * :func:`get_default_subfloor_right_pair_name` / setter

Building blocks for command-specific composite pickers
    * :func:`require_pair_names`           -- guard + MessageBox if empty
    * :func:`resolve_default_pair_index`   -- name -> index against ``names``

Command-specific composite pickers (length / dual-pair / etc.) live in
their owning entry-point file, e.g. ``rs_bar_brace.py``,
``rs_bar_subfloor.py``.
"""

from __future__ import annotations

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

from core.joint_pair import (
    JointPairDef,
    get_joint_pair,
    list_joint_pair_names,
)
from core.rhino_bar_registry import (
    BAR_TYPE_KEY,
    BAR_TYPE_VALUE,
    BAR_ID_KEY,
    TUBE_AXIS_GUID_KEY,
    TUBE_BAR_ID_KEY,
)


# ---------------------------------------------------------------------------
# Doc-userText persistence keys + accessors
# ---------------------------------------------------------------------------

_DOC_USERTEXT_PAIR_KEY = "scaffolding.last_joint_pair"
_DOC_USERTEXT_BRACE_LENGTH_KEY = "scaffolding.last_brace_length"
_DOC_USERTEXT_SUBFLOOR_LEFT_KEY = "scaffolding.last_subfloor_left_pair"
_DOC_USERTEXT_SUBFLOOR_RIGHT_KEY = "scaffolding.last_subfloor_right_pair"
_DEFAULT_BRACE_LENGTH = 500.0


def _get_doc_string(key: str) -> str | None:
    try:
        value = sc.doc.Strings.GetValue(key)
    except Exception:
        value = None
    return value or None


def _set_doc_string(key: str, value: str) -> None:
    try:
        sc.doc.Strings.SetString(key, str(value))
    except Exception:
        pass


def get_default_pair_name() -> str | None:
    """Return the document-stored last-used pair name, or ``None``."""
    return _get_doc_string(_DOC_USERTEXT_PAIR_KEY)


def set_default_pair_name(name: str) -> None:
    _set_doc_string(_DOC_USERTEXT_PAIR_KEY, name)


def get_default_brace_length() -> float:
    """Return the document-stored last-used brace length (mm)."""
    raw = _get_doc_string(_DOC_USERTEXT_BRACE_LENGTH_KEY)
    if not raw:
        return _DEFAULT_BRACE_LENGTH
    try:
        return float(raw)
    except (TypeError, ValueError):
        return _DEFAULT_BRACE_LENGTH


def set_default_brace_length(length_mm: float) -> None:
    _set_doc_string(_DOC_USERTEXT_BRACE_LENGTH_KEY, f"{float(length_mm):.6f}")


def get_default_subfloor_left_pair_name() -> str | None:
    return _get_doc_string(_DOC_USERTEXT_SUBFLOOR_LEFT_KEY)


def set_default_subfloor_left_pair_name(name: str) -> None:
    _set_doc_string(_DOC_USERTEXT_SUBFLOOR_LEFT_KEY, name)


def get_default_subfloor_right_pair_name() -> str | None:
    return _get_doc_string(_DOC_USERTEXT_SUBFLOOR_RIGHT_KEY)


def set_default_subfloor_right_pair_name(name: str) -> None:
    _set_doc_string(_DOC_USERTEXT_SUBFLOOR_RIGHT_KEY, name)


# ---------------------------------------------------------------------------
# Geometry filter + tube resolver
# ---------------------------------------------------------------------------


def bar_or_tube_filter(rhino_object, geometry, component_index):
    """``Rhino.Input.Custom.GetObject`` filter accepting bar curves OR tubes.

    The filter accepts:

    * the thin **bar centerline curve** tagged with
      ``UserText[BAR_TYPE_KEY] == BAR_TYPE_VALUE``, and
    * the surrounding **tube preview** Brep tagged with ``TUBE_BAR_ID_KEY``.

    Callers pass the picked id through :func:`resolve_picked_to_bar_curve`
    so they always get the underlying curve regardless of which geometry
    the user actually clicked.
    """
    oid = rhino_object.Id
    return (
        rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE
        or bool(rs.GetUserText(oid, TUBE_BAR_ID_KEY))
    )


def resolve_picked_to_bar_curve(picked_id):
    """If *picked_id* is a tube preview, return its underlying bar curve.

    Returns the bar centerline curve id, the original id when the user
    clicked the curve directly, or ``None`` for a stale tube whose bar
    was deleted.
    """
    axis_guid_str = rs.GetUserText(picked_id, TUBE_AXIS_GUID_KEY)
    if not axis_guid_str:
        return picked_id
    for oid in rs.AllObjects():
        if (
            rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE
            and str(rs.coerceguid(oid)) == axis_guid_str
        ):
            return oid
    return None


def _picked_bar_id(rhino_object):
    """Return the bar id of *rhino_object* (a curve or its tube), or None."""
    oid = rhino_object.Id
    bar_id = rs.GetUserText(oid, BAR_ID_KEY)
    if bar_id:
        return bar_id
    return rs.GetUserText(oid, TUBE_BAR_ID_KEY) or None


def make_bar_or_tube_filter(exclude_bar_ids=None):
    """Return a ``GetObject``-compatible filter that accepts bar curves /
    tubes the same way :func:`bar_or_tube_filter` does, but rejects any
    object whose bar id is in *exclude_bar_ids*.

    Pass ``None`` or an empty iterable to get the unrestricted filter
    (equivalent to :func:`bar_or_tube_filter`).
    """
    excluded = frozenset(exclude_bar_ids or ())
    if not excluded:
        return bar_or_tube_filter

    def _filter(rhino_object, geometry, component_index):
        if not bar_or_tube_filter(rhino_object, geometry, component_index):
            return False
        bar_id = _picked_bar_id(rhino_object)
        return bar_id not in excluded

    return _filter


# ---------------------------------------------------------------------------
# Joint-pair list helpers (public so command-specific composite pickers in
# rs_bar_brace.py / rs_bar_subfloor.py can reuse them)
# ---------------------------------------------------------------------------


def require_pair_names(command_name: str) -> list[str] | None:
    """Return the registered joint-pair names, or ``None`` after popping a
    MessageBox when the registry is empty.  Use as the first guard in any
    composite picker that needs a pair selection."""
    names = list_joint_pair_names()
    if not names:
        rs.MessageBox(
            "No joint pairs are registered yet.  Run RSDefineJointPair first.",
            0,
            command_name,
        )
        return None
    return names


def resolve_default_pair_index(names: list[str], preferred: str | None) -> int:
    """Pick a sensible starting index inside *names*.

    Order of preference: explicit *preferred* arg, then doc-userText
    default (``scaffolding.last_joint_pair``), then 0."""
    if preferred and preferred in names:
        return names.index(preferred)
    fallback = get_default_pair_name()
    if fallback in names:
        return names.index(fallback)
    return 0


# ---------------------------------------------------------------------------
# Pickers
# ---------------------------------------------------------------------------


def pick_bar(prompt: str):
    """Prompt the user to pick a registered bar -- centerline OR tube preview.

    Always returns the bar **curve** id (never the tube), or ``None`` on cancel.

    Callers: ``rs_bar_snap``, ``rs_bar_brace``, ``rs_bar_subfloor``,
    ``rs_joint_place``.
    """
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    go.EnablePreSelect(True, True)
    go.SetCustomGeometryFilter(bar_or_tube_filter)
    result = go.Get()
    if result != Rhino.Input.GetResult.Object:
        return None
    picked_id = go.Object(0).ObjectId
    # Deselect so a subsequent pick_bar() with pre-select doesn't immediately
    # re-return the same object without waiting for user input.
    rs.UnselectObject(picked_id)
    return resolve_picked_to_bar_curve(picked_id)


def pick_bar_with_pair_option(
    bar_prompt: str,
    command_name: str = "JointPair",
) -> tuple[object, JointPairDef] | tuple[None, None]:
    """Pick a registered bar while exposing a ``Pair`` option to switch
    the active joint pair without leaving the prompt.

    Returns ``(bar_curve_id, JointPairDef)`` on success, ``(None, None)``
    on cancel / empty pair registry.

    Callers: ``rs_bar_snap`` (first bar of a snap pair),
    ``rs_joint_place`` (first bar of a joint placement).
    """
    names = require_pair_names(command_name)
    if names is None:
        return None, None

    selected_index = resolve_default_pair_index(names, None)

    if len(names) == 1:
        set_default_pair_name(names[0])
        return pick_bar(bar_prompt), get_joint_pair(names[0])

    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(bar_prompt)
        go.EnablePreSelect(True, True)
        go.SetCustomGeometryFilter(bar_or_tube_filter)
        list_opt_index = go.AddOptionList("Pair", names, selected_index)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None, None

        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None and opt.Index == list_opt_index:
                selected_index = int(opt.CurrentListOptionIndex)
            continue

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            rs.UnselectObject(picked_id)
            bar_id = resolve_picked_to_bar_curve(picked_id)
            if bar_id is None:
                continue  # stale tube
            chosen = names[selected_index]
            set_default_pair_name(chosen)
            return bar_id, get_joint_pair(chosen)

        return None, None
