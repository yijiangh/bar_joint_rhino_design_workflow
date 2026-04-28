"""Interactive joint-pair selector for Rhino entry-point scripts.

Provides:

* :func:`prompt_joint_pair` -- a standalone GetOption prompt that lets
  the user confirm or change the active joint pair.
* :func:`pick_bar_with_pair_option` -- a combined pick-a-bar prompt that
  also exposes a ``Pair`` option, so the user can change the active pair
  inline without a separate Enter step.

Both persist the current selection to document user-text under
``scaffolding.last_joint_pair`` so it remains the default across
subsequent invocations.
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
    TUBE_AXIS_GUID_KEY,
    TUBE_BAR_ID_KEY,
)


_DOC_USERTEXT_KEY = "scaffolding.last_joint_pair"


def get_default_pair_name() -> str | None:
    """Return the document-stored last-used pair name, or ``None``."""

    try:
        value = sc.doc.Strings.GetValue(_DOC_USERTEXT_KEY)
    except Exception:
        value = None
    return value or None


def set_default_pair_name(name: str) -> None:
    try:
        sc.doc.Strings.SetString(_DOC_USERTEXT_KEY, str(name))
    except Exception:
        pass


def prompt_joint_pair(
    command_name: str = "JointPair",
    *,
    prompt: str = "Joint pair (Enter to accept)",
) -> JointPairDef | None:
    """Prompt the user to confirm or change the active joint pair.

    The current default (last-used pair, or the first registered pair if
    no document default exists) is shown as a list option.  The user can
    either:

    * Press Enter / right-click to accept the default, or
    * Click the list option and pick a different pair from the popup.

    Returns the resolved :class:`~core.joint_pair.JointPairDef`, or
    ``None`` if the user cancelled or the registry is empty.
    """

    names = list_joint_pair_names()
    if not names:
        rs.MessageBox(
            "No joint pairs are registered yet.  Run RSDefineJointPair first.",
            0,
            command_name,
        )
        return None

    default_name = get_default_pair_name()
    if default_name not in names:
        default_name = names[0]
    default_index = names.index(default_name)

    # Single registered pair: skip the prompt entirely.
    if len(names) == 1:
        set_default_pair_name(names[0])
        return get_joint_pair(names[0])

    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(prompt)
    go.AcceptNothing(True)
    list_opt_index = go.AddOptionList("Pair", names, default_index)

    selected_index = default_index
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None
        if result == Rhino.Input.GetResult.Nothing:
            break
        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None and opt.Index == list_opt_index:
                selected_index = int(opt.CurrentListOptionIndex)
                # Loop again so the user can confirm with Enter.
                continue
        # Any other result type: treat as accept.
        break

    chosen_name = names[selected_index]
    set_default_pair_name(chosen_name)
    return get_joint_pair(chosen_name)


def _bar_filter(rhino_object, geometry, component_index):
    oid = rhino_object.Id
    return rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE or bool(
        rs.GetUserText(oid, TUBE_BAR_ID_KEY)
    )


def _resolve_picked_to_bar_curve(picked_id):
    """If the user clicked a tube preview, resolve to the underlying bar curve."""
    axis_guid_str = rs.GetUserText(picked_id, TUBE_AXIS_GUID_KEY)
    if axis_guid_str:
        for oid in rs.AllObjects():
            if (
                rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE
                and str(rs.coerceguid(oid)) == axis_guid_str
            ):
                return oid
        return None
    return picked_id


def pick_bar_with_pair_option(
    bar_prompt: str,
    command_name: str = "JointPair",
) -> tuple[object, JointPairDef] | tuple[None, None]:
    """Prompt the user to pick a registered bar AND simultaneously expose a
    ``Pair`` option for switching the active joint pair.

    The user can either:

    * Click a registered bar directly -- the current default pair is
      used and returned.
    * Click the ``Pair`` option to choose a different joint pair, then
      pick a bar.

    Returns ``(bar_object_id, JointPairDef)`` on success or
    ``(None, None)`` if the user cancelled or no pairs are registered.
    """

    names = list_joint_pair_names()
    if not names:
        rs.MessageBox(
            "No joint pairs are registered yet.  Run RSDefineJointPair first.",
            0,
            command_name,
        )
        return None, None

    default_name = get_default_pair_name()
    if default_name not in names:
        default_name = names[0]
    selected_index = names.index(default_name)

    # Single registered pair: skip the option entirely and behave like
    # a plain bar pick.
    if len(names) == 1:
        set_default_pair_name(names[0])
        pair = get_joint_pair(names[0])
        return _pick_bar_only(bar_prompt), pair

    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(bar_prompt)
        go.EnablePreSelect(True, True)
        go.SetCustomGeometryFilter(_bar_filter)
        list_opt_index = go.AddOptionList("Pair", names, selected_index)

        result = go.Get()
        if result == Rhino.Input.GetResult.Cancel:
            return None, None

        if result == Rhino.Input.GetResult.Option:
            opt = go.Option()
            if opt is not None and opt.Index == list_opt_index:
                selected_index = int(opt.CurrentListOptionIndex)
                # Loop so the user can pick a bar (or change pair again).
                continue
            # Unknown option -- loop again.
            continue

        if result == Rhino.Input.GetResult.Object:
            picked_id = go.Object(0).ObjectId
            rs.UnselectObject(picked_id)
            bar_id = _resolve_picked_to_bar_curve(picked_id)
            if bar_id is None:
                # Stale tube preview: try again.
                continue
            chosen_name = names[selected_index]
            set_default_pair_name(chosen_name)
            return bar_id, get_joint_pair(chosen_name)

        # Anything else (Nothing, etc.): cancel.
        return None, None


def _pick_bar_only(prompt: str):
    """Plain bar pick without any pair option (used when only one pair exists)."""
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(prompt)
    go.EnablePreSelect(True, True)
    go.SetCustomGeometryFilter(_bar_filter)
    result = go.Get()
    if result != Rhino.Input.GetResult.Object:
        return None
    picked_id = go.Object(0).ObjectId
    rs.UnselectObject(picked_id)
    return _resolve_picked_to_bar_curve(picked_id)
