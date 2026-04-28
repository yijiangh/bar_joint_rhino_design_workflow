"""Robotic-tool placement in Rhino.

Given a male joint block instance already placed in the document, this
module computes the world TCP frame (the screw-axis frame on the male
block) and inserts the chosen robotic-tool block so that the tool's TCP
coincides with that screw frame.

Tagging convention on the inserted tool block (Rhino UserText):
    tool_id     : "T<joint_id>"            (e.g. "TJ1-2")
    tool_name   : the registry name        (e.g. "AT3L")
    joint_id    : the joint it's attached to
    block_name  : the underlying InstanceDefinition

Object name (for picking by name):  "T<joint_id>"  -> e.g. "TJ1-2".

This module is Rhino-runtime-only at call time but safe to import outside
Rhino (it imports ``rhinoscriptsyntax`` lazily inside its functions).
"""

from __future__ import annotations

import numpy as np

from core import config
from core import robotic_tool as _robotic_tool
from core.rhino_block_import import (
    has_block_definition,
    require_block_definition,
)


# Doc-userText key for the session-wide default tool.
_DOC_USERTEXT_DEFAULT_TOOL_KEY = "scaffolding.last_robotic_tool"


# ---------------------------------------------------------------------------
# Default-tool persistence (doc-userText)
# ---------------------------------------------------------------------------


def get_default_tool_name() -> str | None:
    """Return the doc-stored default tool name, or ``None`` if unset/missing."""
    import scriptcontext as sc  # noqa: PLC0415  (Rhino runtime)

    name = sc.doc.Strings.GetValue(_DOC_USERTEXT_DEFAULT_TOOL_KEY)
    if not name:
        return None
    if name not in _robotic_tool.load_robotic_tools():
        return None
    return name


def set_default_tool_name(name: str) -> None:
    import scriptcontext as sc  # noqa: PLC0415  (Rhino runtime)

    sc.doc.Strings.SetString(_DOC_USERTEXT_DEFAULT_TOOL_KEY, str(name))


# ---------------------------------------------------------------------------
# Block import + placement
# ---------------------------------------------------------------------------


def _numpy_to_rhino_transform(matrix: np.ndarray):
    import Rhino  # noqa: PLC0415

    xform = Rhino.Geometry.Transform(1.0)
    for row in range(4):
        for col in range(4):
            xform[row, col] = float(matrix[row, col])
    return xform


def _has_block_definition(name: str) -> bool:
    return has_block_definition(name)


def _import_tool_block_definition(tool: _robotic_tool.RoboticToolDef) -> bool:
    """Make sure the tool's block definition is loaded in the active doc.

    Imports from ``asset/<asset_filename>`` via the same helper used for
    joint blocks (RhinoCommon ``File3dm.Read`` with a ``_-Insert`` fallback).
    """
    asset_path = tool.asset_path() if tool.asset_filename else None
    try:
        require_block_definition(tool.block_name, asset_path=asset_path)
        return True
    except RuntimeError as exc:
        print(f"  [tool] {exc}")
        return False


def _male_world_frame_from_object(male_id) -> np.ndarray:
    """Return the world transform of an inserted male block instance."""
    import Rhino  # noqa: PLC0415
    import scriptcontext as sc  # noqa: PLC0415

    rh_obj = sc.doc.Objects.FindId(male_id)
    if rh_obj is None or not isinstance(rh_obj, Rhino.DocObjects.InstanceObject):
        raise ValueError(f"Object {male_id} is not a block instance.")
    xform = rh_obj.InstanceXform
    return np.array(
        [[xform[r, c] for c in range(4)] for r in range(4)], dtype=float
    )


def remove_tool_for_joint(joint_id: str) -> int:
    """Delete any existing tool instance(s) tagged with ``joint_id``.

    Returns the number of objects removed.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return 0
    removed = 0
    for oid in rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []:
        if rs.GetUserText(oid, "joint_id") == joint_id:
            rs.DeleteObject(oid)
            removed += 1
    return removed


def place_tool_at_male_joint(
    male_id,
    joint_id: str,
    pair,
    tool: _robotic_tool.RoboticToolDef,
):
    """Insert the chosen tool block aligned to the male joint's screw frame.

    Parameters
    ----------
    male_id : Rhino object id
        The already-placed male joint block instance.
    joint_id : str
        The joint identifier (e.g. ``"J1-2"``); used in the tool object's
        name (``"T<joint_id>"``) and ``joint_id`` user-text.
    pair : core.joint_pair.JointPairDef
        Used to look up ``M_screw_from_block`` for the male half.
    tool : RoboticToolDef
        The tool to place.

    Returns
    -------
    Rhino object id of the inserted tool block, or ``None`` on failure.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not _import_tool_block_definition(tool):
        print(
            f"  WARNING: tool block '{tool.block_name}' for tool '{tool.name}' "
            f"could not be loaded (asset missing?); skipping tool placement."
        )
        return None

    # Remove any prior tool tagged with this joint_id (idempotent placement).
    remove_tool_for_joint(joint_id)

    male_world = _male_world_frame_from_object(male_id)
    M_tcp_from_block = np.asarray(tool.M_tcp_from_block, dtype=float)

    # The tool's TCP should be coincident with the male joint block's
    # origin frame (not the screw frame).  We want
    #   world_tool_block @ M_tcp_from_block == male_world
    #   => world_tool_block = male_world @ inv(M_tcp_from_block)
    world_tool_block = male_world @ np.linalg.inv(M_tcp_from_block)

    tool_oid = rs.InsertBlock(tool.block_name, [0, 0, 0])
    if tool_oid is None:
        print(f"  WARNING: failed to insert tool block '{tool.block_name}'.")
        return None
    rs.TransformObject(tool_oid, _numpy_to_rhino_transform(world_tool_block))
    rs.ObjectLayer(tool_oid, config.LAYER_TOOL_INSTANCES)

    tool_id = f"T{joint_id}"
    rs.ObjectName(tool_oid, tool_id)
    rs.SetUserText(tool_oid, "tool_id", tool_id)
    rs.SetUserText(tool_oid, "tool_name", tool.name)
    rs.SetUserText(tool_oid, "joint_id", joint_id)
    rs.SetUserText(tool_oid, "block_name", tool.block_name)
    return tool_oid


def auto_place_tool_at_male_joint(male_id, joint_id: str, pair):
    """Place the appropriate tool at a newly-created male joint.

    Resolution order:
      1. ``pair.male.preferred_robotic_tool_name`` if set on the pair
         definition (and the named tool exists in the registry).
      2. The doc-stored default ``scaffolding.last_robotic_tool``.
      3. The first tool in the registry (alphabetical).

    Designers don't have to configure anything.  The only no-op case (with
    a console message) is when the registry itself is empty.
    """
    all_tools = _robotic_tool.load_robotic_tools()
    if not all_tools:
        print("  [tool] no robotic tools registered; skipping tool placement.")
        return None

    preferred = getattr(pair.male, "preferred_robotic_tool_name", "") or ""
    if preferred:
        if preferred in all_tools:
            print(
                f"  [tool] using pair-preferred tool '{preferred}' "
                f"for joint {joint_id}."
            )
            return place_tool_at_male_joint(
                male_id, joint_id, pair, all_tools[preferred]
            )
        print(
            f"  [tool] pair-preferred tool '{preferred}' not in registry; "
            f"falling back."
        )

    name = get_default_tool_name()
    if not name or name not in all_tools:
        name = sorted(all_tools.keys())[0]
        print(f"  [tool] no default set; falling back to first tool '{name}'.")
    return place_tool_at_male_joint(male_id, joint_id, pair, all_tools[name])


# ---------------------------------------------------------------------------
# Cycling between available tools
# ---------------------------------------------------------------------------


def find_tool_for_joint(joint_id: str):
    """Return the Rhino object id of the tool instance tagged with *joint_id*,
    or ``None`` if no such tool is currently placed."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return None
    for oid in rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []:
        if rs.GetUserText(oid, "joint_id") == joint_id:
            return oid
    return None


def get_tool_name_for_joint(joint_id: str) -> str | None:
    """Return the ``tool_name`` user-text on the tool instance tagged with
    *joint_id*, or ``None`` if no tool is placed for that joint."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    oid = find_tool_for_joint(joint_id)
    if oid is None:
        return None
    name = rs.GetUserText(oid, "tool_name")
    return name or None


def place_tool_by_name_at_male_joint(
    male_id, joint_id: str, pair, tool_name: str | None
):
    """Place a specific named tool at the male joint, falling back to the
    auto-place behavior when *tool_name* is ``None`` or unknown."""
    if tool_name:
        try:
            tool = _robotic_tool.get_robotic_tool(tool_name)
        except KeyError:
            print(
                f"  [tool] requested tool '{tool_name}' not found in registry; "
                f"falling back to default."
            )
        else:
            return place_tool_at_male_joint(male_id, joint_id, pair, tool)
    return auto_place_tool_at_male_joint(male_id, joint_id, pair)


def find_male_block_for_joint(joint_id: str):
    """Return the Rhino object id of the male joint block tagged with
    *joint_id*, or ``None``.  Looks by the conventional object name
    ``{joint_id}_male`` first, then by user-text scan as a fallback."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    ids = rs.ObjectsByName(f"{joint_id}_male") or []
    if ids:
        return ids[0]
    if not rs.IsLayer(config.LAYER_JOINT_MALE_INSTANCES):
        return None
    for oid in rs.ObjectsByLayer(config.LAYER_JOINT_MALE_INSTANCES) or []:
        if rs.GetUserText(oid, "joint_id") == joint_id:
            return oid
    return None


def cycle_tool_at_tool_instance(tool_oid, *, pair=None) -> str | None:
    """Replace the clicked tool instance with the next tool in the registry.

    The ordering is the alphabetical order of tool names, cycled.  If the
    registry has only one entry, this no-ops.  Returns the new tool name
    on success (or the unchanged name if cycling was a no-op), ``None``
    on failure.

    *pair* is forwarded to :func:`place_tool_at_male_joint` for API
    parity; it is currently unused by the placement math (TCP is matched
    to the male block origin) but kept for forward compatibility.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    joint_id = rs.GetUserText(tool_oid, "joint_id")
    current_name = rs.GetUserText(tool_oid, "tool_name")
    if not joint_id:
        print("  [tool] clicked tool has no 'joint_id' user-text; cannot cycle.")
        return None

    names = sorted(_robotic_tool.load_robotic_tools().keys())
    if not names:
        print("  [tool] registry is empty; nothing to cycle to.")
        return None
    if len(names) == 1:
        print(f"  [tool] only one tool registered ('{names[0]}'); cycle is a no-op.")
        next_name = names[0]
    else:
        try:
            idx = names.index(current_name)
        except ValueError:
            idx = -1
        next_name = names[(idx + 1) % len(names)]

    male_id = find_male_block_for_joint(joint_id)
    if male_id is None:
        print(f"  [tool] could not locate male joint block for {joint_id}.")
        return None

    try:
        tool = _robotic_tool.get_robotic_tool(next_name)
    except KeyError:
        print(f"  [tool] tool '{next_name}' not found in registry.")
        return None

    new_oid = place_tool_at_male_joint(male_id, joint_id, pair, tool)
    if new_oid is None:
        return None
    set_default_tool_name(next_name)
    print(f"  [tool] {joint_id}: tool cycled '{current_name}' -> '{next_name}'.")
    return next_name
