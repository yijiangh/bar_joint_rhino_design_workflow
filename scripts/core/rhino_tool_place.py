"""Robotic-tool placement in Rhino.

Given a joint block instance already placed in the document, this module
computes the joint's **tool-attach frame** and inserts the chosen
robotic-tool block so that the tool's TCP coincides with it.

:func:`tool_attach_frame` is that frame, and it is the single function both
the write side (:func:`place_tool_at_block_instance`) and the read side
(:func:`is_tool_on_joint`) go through, so the two can never disagree:

* male / female joint blocks -> the block instance's own world frame (the
  historical convention: the block origin IS the TCP).
* ground joint blocks -> that frame post-multiplied by the definition's
  constant block-local ``M_tool_from_block``, so a ground joint's tool can
  be rolled relative to the block WITHOUT moving the block.  The ground
  block's own orientation is pinned by ``auto_jr_y_down`` (its local +Y must
  point at the floor), which is why the arm's approach needs a frame of its
  own.  ``M_tool_from_block`` is identity for every ground joint defined
  before that frame existed, so their behaviour is unchanged.

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

import os

import numpy as np

from core import config
from core import robotic_tool as _robotic_tool
from core.rhino_block_import import refresh_block_definition, require_block_definition


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


def _import_tool_block_definition(tool: _robotic_tool.RoboticToolDef) -> bool:
    """Make sure the tool's block definition is loaded in the active doc.

    Imports from ``asset/<asset_filename>`` via the same helper used for
    joint blocks (RhinoCommon ``File3dm.Read`` with a ``_-Insert`` fallback).
    """
    asset_path = tool.asset_path() if tool.asset_filename else None
    try:
        # Pin the tool's parts onto the (visible) tool-instances layer: the
        # source .3dm's own layers are not imported, so without this the block's
        # geometry lands on a hidden/unrelated layer and the inserted tool shows
        # nothing (see rhino_block_import.import_block_definition_from_3dm).
        require_block_definition(
            tool.block_name,
            asset_path=asset_path,
            layer_name=config.LAYER_TOOL_INSTANCES,
        )
        return True
    except RuntimeError as exc:
        print(f"  [tool] {exc}")
        return False


def _block_instance_world_xform(block_id) -> np.ndarray:
    """Return the world transform of any inserted block instance."""
    import Rhino  # noqa: PLC0415
    import scriptcontext as sc  # noqa: PLC0415

    rh_obj = sc.doc.Objects.FindId(block_id)
    if rh_obj is None or not isinstance(rh_obj, Rhino.DocObjects.InstanceObject):
        raise ValueError(f"Object {block_id} is not a block instance.")
    xform = rh_obj.InstanceXform
    return np.array(
        [[xform[r, c] for c in range(4)] for r in range(4)], dtype=float
    )


# Back-compat alias.
_male_world_frame_from_object = _block_instance_world_xform


# ---------------------------------------------------------------------------
# Tool-attach frame (block frame + optional block-local offset)
# ---------------------------------------------------------------------------

#: Cache of ``ground_joint_name -> 4x4 block-local tool-attach offset``, keyed by
#: the registry file's (path, mtime, size).  `resync_tools_to_joints`,
#: `restore_missing_tools_at_joints` and `enforce_bar_tool_sides` each walk EVERY
#: tool in the document on EVERY RSUpdatePreview run, so the registry must not be
#: re-parsed per tool.  Keying on the file stamp means a hand-edit of
#: joint_pairs.json is still picked up on the next call, with no Rhino restart.
_GROUND_OFFSET_CACHE: dict = {}
_GROUND_OFFSET_STAMP = None


def clear_tool_attach_cache() -> None:
    """Drop the cached ground tool-attach offsets.

    Call after writing the registry (RSDefineJointHalf) and from tests; the
    file-stamp key below already covers ordinary edits.
    """
    global _GROUND_OFFSET_STAMP

    _GROUND_OFFSET_CACHE.clear()
    _GROUND_OFFSET_STAMP = None


def _ground_offsets(path: str | None = None) -> dict:
    """``{ground_joint_name: 4x4}`` for the whole registry, cached by file stamp."""
    global _GROUND_OFFSET_STAMP

    from core import joint_pair as _joint_pair  # noqa: PLC0415

    path = path or _joint_pair.DEFAULT_REGISTRY_PATH
    try:
        stat = os.stat(path)
        stamp = (path, stat.st_mtime_ns, stat.st_size)
    except OSError:
        stamp = (path, None, None)
    if stamp == _GROUND_OFFSET_STAMP:
        return _GROUND_OFFSET_CACHE

    offsets: dict = {}
    try:
        registry = _joint_pair.load_joint_registry(path)
    except (OSError, ValueError, KeyError) as exc:
        # ! A malformed registry must not abort a whole RSUpdatePreview repair
        # ! pass -- fall back to identity (the historical behaviour) and say so.
        print(
            f"  [tool] cannot read ground tool-attach offsets ({exc}); "
            "attaching tools on the raw block frames."
        )
    else:
        for name, ground in registry.ground_joints.items():
            offsets[name] = np.asarray(
                getattr(ground, "M_tool_from_block", np.eye(4)), dtype=float
            )
    _GROUND_OFFSET_CACHE.clear()
    _GROUND_OFFSET_CACHE.update(offsets)
    _GROUND_OFFSET_STAMP = stamp
    return _GROUND_OFFSET_CACHE


def ground_tool_attach_offset(
    ground_joint_name: str, path: str | None = None
) -> np.ndarray:
    """Block-local tool-attach offset for a ground joint DEFINITION (4x4).

    Identity for an empty or unregistered name.  Always a pure rotation about
    the block origin -- ``GroundJointDef`` zeroes the translation -- so the TCP
    probe point in :mod:`core.joint_relink` is unaffected.
    """
    if not ground_joint_name:
        return np.eye(4)
    return _ground_offsets(path).get(str(ground_joint_name), np.eye(4))


def tool_attach_offset(block_id) -> np.ndarray:
    """Block-local offset between a joint block's frame and its tool's TCP frame.

    Ground blocks carry a ``ground_joint_name`` user-text (written by
    :func:`core.ground_placement.place_ground_block`); male/female blocks do
    not, so they resolve to identity and their behaviour is unchanged.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    name = rs.GetUserText(block_id, "ground_joint_name") or ""
    return ground_tool_attach_offset(name)


def tool_attach_frame(block_id) -> np.ndarray:
    """THE world frame a tool's TCP must coincide with for *block_id*.

    Single source of truth shared by :func:`place_tool_at_block_instance`
    (which poses a tool onto it) and :func:`is_tool_on_joint` (which checks a
    tool against it), so a placement can never be reported as drift.
    """
    return _block_instance_world_xform(block_id) @ tool_attach_offset(block_id)


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


def place_tool_at_block_instance(
    block_id,
    joint_id: str,
    tool: _robotic_tool.RoboticToolDef,
):
    """Insert *tool*'s block aligned to *block_id*'s frame.

    Generic core: works for ANY joint block instance -- male and ground
    alike -- via :func:`tool_attach_frame`.

    Computes ``world_tool_block = tool_attach_frame(block_id) @
    inv(M_tcp_from_block)`` so the tool's TCP coincides with the joint's
    tool-attach frame.  That frame is the block's own world frame for male
    joints and for any ground joint whose ``M_tool_from_block`` is identity,
    which is the historical behaviour.

    Returns the inserted tool's Rhino object id, or ``None`` on failure.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not _import_tool_block_definition(tool):
        print(
            f"  WARNING: tool block '{tool.block_name}' for tool '{tool.name}' "
            f"could not be loaded (asset missing?); skipping tool placement."
        )
        return None

    # Idempotent placement: drop any existing tool tagged with this joint_id.
    remove_tool_for_joint(joint_id)

    attach_world = tool_attach_frame(block_id)
    M_tcp_from_block = np.asarray(tool.M_tcp_from_block, dtype=float)
    world_tool_block = attach_world @ np.linalg.inv(M_tcp_from_block)

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


def replace_all_tool_instances(pair: dict) -> dict:
    """Re-place EVERY placed tool instance with the side-matching tool from *pair*.

    The engine of RSSwapRoboticTool.  Each joint keeps the arm side it
    already had (the L/R suffix of its instance's ``tool_name`` user-text);
    only the tool itself changes.

    Two passes, validate-then-mutate:

    Pass 1 (read-only): for every object on ``config.LAYER_TOOL_INSTANCES``,
    read ``joint_id`` + ``tool_name``, classify the side, and resolve the
    joint block it is attached to.  ANY failure aborts with a RuntimeError
    listing every offending instance -- nothing has been deleted yet.

    Pass 2 (mutate): delete all tool instances, force-refresh/import both pair
    blocks from their asset .3dm files (even when no tools are currently
    placed), then re-place one tool per joint.

    Args:
        pair (dict): ``{"left": RoboticToolDef, "right": RoboticToolDef}``,
            e.g. from ``core.robotic_tool.get_active_pair()``.

    Returns:
        dict: ``{"replaced": total, "left": n_left, "right": n_right}``.

    Raises:
        RuntimeError: on any Pass-1 validation failure, block-refresh
            failure, or per-joint placement failure.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415
    # Rhino-runtime import: rhino_helpers pulls in rhinoscriptsyntax at its
    # top, so importing it lazily keeps this module importable outside Rhino.
    from core.rhino_helpers import suspend_redraw  # noqa: PLC0415

    # * Pass 1: validate every placed tool instance before touching anything.
    jobs: dict = {}       # joint_id -> (side, joint_block_id)
    tool_oids: list = []  # everything on the tool layer (deleted in pass 2)
    problems: list = []
    if rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        existing_tool_oids = rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []
    else:
        existing_tool_oids = []
    for oid in existing_tool_oids:
        tool_oids.append(oid)
        joint_id = rs.GetUserText(oid, "joint_id")
        tool_name = rs.GetUserText(oid, "tool_name") or ""
        if not joint_id:
            problems.append(f"  - object {oid}: missing 'joint_id' user-text")
            continue
        side = _robotic_tool.arm_side_from_tool_name(tool_name)
        if side is None:
            problems.append(
                f"  - joint {joint_id}: tool_name {tool_name!r} has no L/R "
                "suffix; cannot keep its arm side"
            )
            continue
        block_id = find_attached_block_for_joint(joint_id)
        if block_id is None:
            problems.append(
                f"  - joint {joint_id}: no male/ground joint block found in doc"
            )
            continue
        if joint_id in jobs and jobs[joint_id][0] != side:
            problems.append(
                f"  - joint {joint_id}: two tool instances with conflicting "
                "L/R sides"
            )
            continue
        jobs[joint_id] = (side, block_id)
    if problems:
        raise RuntimeError(
            "replace_all_tool_instances: aborting BEFORE any change; fix "
            "these tool instances first:\n" + "\n".join(problems)
        )

    # * Pass 2: delete everything, refresh/import definitions, re-place.
    # ! Always refresh both definitions, even with zero existing tool
    # ! instances. This supports swapping from an IK document that never had
    # ! the candidate source blocks inserted into it.
    counts = {"left": 0, "right": 0}
    with suspend_redraw():
        for oid in tool_oids:
            if rs.IsObject(oid):
                rs.DeleteObject(oid)
        for side in ("left", "right"):
            tool = pair[side]
            # Re-import onto the visible tool-instances layer so the swapped
            # tool's parts are not left on a hidden source-file layer.
            refresh_block_definition(
                tool.block_name,
                tool.asset_path(),
                layer_name=config.LAYER_TOOL_INSTANCES,
            )
        for joint_id, (side, block_id) in jobs.items():
            new_oid = place_tool_at_block_instance(block_id, joint_id, pair[side])
            if new_oid is None:
                raise RuntimeError(
                    f"replace_all_tool_instances: failed to place tool "
                    f"'{pair[side].name}' at joint {joint_id}."
                )
            counts[side] += 1
    return {"replaced": counts["left"] + counts["right"], **counts}


def place_tool_at_male_joint(
    male_id,
    joint_id: str,
    pair,
    tool: _robotic_tool.RoboticToolDef,
):
    """Back-compat wrapper around :func:`place_tool_at_block_instance`
    for the male-joint workflow.  ``pair`` is currently unused (the math
    only needs the male block's world frame) but kept for API parity.
    """
    del pair  # unused
    return place_tool_at_block_instance(male_id, joint_id, tool)


def _get_active_pair_or_none():
    """Load the active tool pair, printing (not raising) when unresolvable.

    Auto-placement runs in the middle of joint placement; a missing tool
    setup should not abort the whole joint command, so the error is printed
    loudly and ``None`` is returned instead of raising.
    """
    try:
        return _robotic_tool.get_active_pair()
    except (RuntimeError, ValueError) as exc:
        print(f"  [tool] no active tool pair; skipping tool placement: {exc}")
        return None


def _resolve_default_active_tool(active: dict) -> _robotic_tool.RoboticToolDef:
    """Pick the doc-default tool if it is in the active pair, else active left.

    Any doc default that is NOT in the active pair is loudly skipped -- the
    active pair (set by RSSwapRoboticTool) always wins over stale defaults.
    """
    name = get_default_tool_name()
    if name == active["left"].name:
        return active["left"]
    if name == active["right"].name:
        return active["right"]
    if name:
        print(
            f"  [tool] doc default '{name}' is not in the active pair; using "
            f"active left tool '{active['left'].name}' instead "
            "(run RSSwapRoboticTool to change pairs)."
        )
    return active["left"]


def auto_place_tool_at_male_joint(male_id, joint_id: str, pair):
    """Place the appropriate tool at a newly-created male joint.

    Resolution order (active pair only -- no silent fallback outside it):
      1. ``pair.male.preferred_robotic_tool_name`` if it is one of the
         ACTIVE pair's tools.
      2. The doc-stored default ``scaffolding.last_robotic_tool`` if it is
         one of the active pair's tools.
      3. The active LEFT tool.

    Designers don't have to configure anything.  The only no-op case (with
    a console message) is when the active pair cannot be resolved.
    """
    active = _get_active_pair_or_none()
    if active is None:
        return None

    active_by_name = {active["left"].name: active["left"], active["right"].name: active["right"]}
    preferred = getattr(pair.male, "preferred_robotic_tool_name", "") or ""
    if preferred:
        if preferred in active_by_name:
            print(
                f"  [tool] using pair-preferred tool '{preferred}' "
                f"for joint {joint_id}."
            )
            return place_tool_at_male_joint(
                male_id, joint_id, pair, active_by_name[preferred]
            )
        print(
            f"  [tool] pair-preferred tool '{preferred}' is not in the active "
            f"pair ({sorted(active_by_name)}); ignoring it "
            "(run RSSwapRoboticTool to change pairs)."
        )

    tool = _resolve_default_active_tool(active)
    return place_tool_at_male_joint(male_id, joint_id, pair, tool)


def auto_place_tool_at_ground_block(ground_id, joint_id: str):
    """Place the appropriate tool at a newly-created ground joint block.

    Same resolution order as the male variant minus the per-pair preference
    (ground joints have no `pair.male.preferred_robotic_tool_name`).
    """
    active = _get_active_pair_or_none()
    if active is None:
        return None
    tool = _resolve_default_active_tool(active)
    return place_tool_at_block_instance(ground_id, joint_id, tool)


def place_tool_by_name_at_ground_block(
    ground_id, joint_id: str, tool_name: str | None
):
    """Re-place a specific named tool at a ground block (used by RSJointEdit
    flip path to preserve whichever tool the user previously chose).
    Falls back to :func:`auto_place_tool_at_ground_block` when ``tool_name``
    is None or unknown.
    """
    if tool_name:
        try:
            tool = _robotic_tool.get_robotic_tool(tool_name)
        except KeyError:
            print(
                f"  [tool] requested tool '{tool_name}' not found in registry; "
                f"falling back to default."
            )
        else:
            return place_tool_at_block_instance(ground_id, joint_id, tool)
    return auto_place_tool_at_ground_block(ground_id, joint_id)


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


def find_ground_block_for_joint(joint_id: str):
    """Return the Rhino object id of the ground joint block tagged with
    *joint_id*, or ``None``."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    ids = rs.ObjectsByName(f"{joint_id}_ground") or []
    if ids:
        return ids[0]
    if not rs.IsLayer(config.LAYER_JOINT_GROUND_INSTANCES):
        return None
    for oid in rs.ObjectsByLayer(config.LAYER_JOINT_GROUND_INSTANCES) or []:
        if rs.GetUserText(oid, "joint_id") == joint_id:
            return oid
    return None


def find_attached_block_for_joint(joint_id: str):
    """Return the Rhino object id of the joint block (male OR ground) the
    tool is attached to.  Dispatches by the ``joint_id`` prefix (``J*`` =
    male pair, ``G*`` = ground), with a search of the other layer as
    fallback so renamed ids still resolve."""
    if joint_id.startswith("G"):
        oid = find_ground_block_for_joint(joint_id)
        if oid is not None:
            return oid
        return find_male_block_for_joint(joint_id)
    oid = find_male_block_for_joint(joint_id)
    if oid is not None:
        return oid
    return find_ground_block_for_joint(joint_id)


# ---------------------------------------------------------------------------
# Re-snapping drifted tools back onto their joint blocks
# ---------------------------------------------------------------------------

#: Element-wise tolerance (mm / unitless) for the tool-on-joint check below.
#: A correctly-placed tool satisfies ``tool_world @ M_tcp_from_block ==
#: tool_attach_frame(block_id)`` to floating-point precision, so any real drift
#: (user nudged the tool, or the joint was moved after the tool was placed) sits
#: far above this.  Mirrors the TCP-coincidence invariant used by
#: ``core.joint_relink``.
_TOOL_ON_JOINT_TOL = 1e-3


def is_tool_on_joint(tool_oid, block_id, tool) -> bool:
    """True when *tool_oid* is placed on the joint block *block_id*.

    THE definition of "this tool sits on that joint", inverted straight from
    :func:`place_tool_at_block_instance`, which poses a tool as
    ``world_tool_block = tool_attach_frame(block_id) @ inv(M_tcp_from_block)``.
    So a correctly placed tool satisfies ``tool_world @ M_tcp_from_block ==
    tool_attach_frame(block_id)``.  Both :func:`resync_tools_to_joints` (which
    repairs) and :func:`find_detached_tools` (which reports) go through here,
    and both sides read the attach frame from the same function, so a ground
    joint's rolled tool is never mistaken for drift and re-snapped away.

    Args:
        tool_oid: the robotic-tool block instance.
        block_id: the male/ground joint block it claims to be on.
        tool (RoboticToolDef): supplies ``M_tcp_from_block``.

    Returns:
        bool: True when the tool's TCP reproduces the joint's attach frame.

    Raises:
        ValueError: if either object is not a readable block instance.
    """
    tool_world = _block_instance_world_xform(tool_oid)
    attach_world = tool_attach_frame(block_id)
    tcp_world = tool_world @ np.asarray(tool.M_tcp_from_block, dtype=float)
    return bool(
        np.allclose(tcp_world, attach_world, rtol=0.0, atol=_TOOL_ON_JOINT_TOL)
    )


def find_detached_tools() -> list:
    """Return tool instances that are not sitting on a joint block.  Read-only.

    Same criterion as :func:`resync_tools_to_joints` (both call
    :func:`is_tool_on_joint`), but reporting instead of repairing.  Run it AFTER
    the resync pass: anything still detached is one the resync could not fix,
    i.e. a tool genuinely flying free of the model.  That is the case a
    ``joint_id``-only check misses -- the metadata can still resolve while the
    geometry has come adrift.

    Returns:
        list[tuple]: ``(tool_oid, joint_id, reason)`` per detached tool.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return []

    all_tools = _robotic_tool.load_robotic_tools()
    detached = []
    for tool_oid in list(rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []):
        if not rs.IsObject(tool_oid):
            continue
        joint_id = rs.GetUserText(tool_oid, "joint_id") or ""
        tool_name = rs.GetUserText(tool_oid, "tool_name") or ""
        if not joint_id:
            detached.append((tool_oid, "", "no 'joint_id' user-text"))
            continue
        tool = all_tools.get(tool_name)
        if tool is None:
            detached.append(
                (tool_oid, joint_id, f"tool_name {tool_name!r} is not registered")
            )
            continue
        block_id = find_attached_block_for_joint(joint_id)
        if block_id is None:
            detached.append((tool_oid, joint_id, "its joint block is gone"))
            continue
        try:
            if not is_tool_on_joint(tool_oid, block_id, tool):
                detached.append(
                    (tool_oid, joint_id, "not on its joint (could not be re-snapped)")
                )
        except ValueError as exc:
            detached.append((tool_oid, joint_id, f"unreadable block frame ({exc})"))
    return detached


def resync_tools_to_joints(verbose: bool = False) -> int:
    """Snap any tool that has drifted away from its joint back onto it.

    For every robotic-tool instance this checks whether its TCP frame still
    coincides with the joint block it is tagged to (``joint_id`` user text) --
    the same ``tool_world @ M_tcp_from_block == block_world`` invariant that
    :mod:`core.joint_relink` relies on.  When a tool has drifted (the user
    nudged it, or the joint block moved after the tool was placed) it is put
    back by re-running the canonical placement,
    :func:`place_tool_at_block_instance` -- the very routine RSBarSnap /
    RSBarBrace auto-placement and the tool-cycle command already use -- so the
    same tool lands exactly on the joint's current frame.

    Tools already sitting on their joint are left untouched.  Tools whose
    joint block is gone (joint deleted) or whose ``tool_name`` is no longer in
    the registry are skipped with a note when *verbose*.  Returns the number
    of tools actually re-snapped.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    if not rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        return 0

    all_tools = _robotic_tool.load_robotic_tools()
    n_moved = 0
    for tool_oid in list(rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []):
        if not rs.IsObject(tool_oid):
            continue  # already removed (e.g. a duplicate cleared this pass)
        joint_id = rs.GetUserText(tool_oid, "joint_id")
        tool_name = rs.GetUserText(tool_oid, "tool_name")
        if not joint_id or not tool_name:
            continue
        tool = all_tools.get(tool_name)
        if tool is None:
            if verbose:
                print(
                    f"  [tool] {joint_id}: tool_name '{tool_name}' not in "
                    f"registry; cannot resync, skipping."
                )
            continue
        block_id = find_attached_block_for_joint(joint_id)
        if block_id is None:
            if verbose:
                print(f"  [tool] {joint_id}: no joint block found; skipping.")
            continue

        # Is the tool still on its joint?  Skip (rather than abort the whole
        # pass) if either object isn't a readable block instance -- e.g. it was
        # exploded.
        try:
            on_joint = is_tool_on_joint(tool_oid, block_id, tool)
        except ValueError as exc:
            if verbose:
                print(f"  [tool] {joint_id}: cannot read block frame ({exc}); skipping.")
            continue
        if on_joint:
            continue  # already on the joint -- leave it alone

        # Drifted -> re-place the same tool on the joint's current frame.
        if place_tool_at_block_instance(block_id, joint_id, tool) is not None:
            n_moved += 1
            if verbose:
                print(f"  [tool] {joint_id}: drifted tool re-snapped onto joint.")
    return n_moved


def _restore_side_tool(block_id, joint_id, active, default_tool):
    """Pick the correct-side tool for a joint whose tool went missing.

    A bar carries two tool-bearing joints, one LEFT-suffix tool and one
    RIGHT-suffix tool (this is the L/R layout RSIKKeyframe requires).  So if the
    SIBLING joint on the same bar (matched by ``parent_bar_id``) still holds a
    tool with a resolvable side, the missing joint must be the OPPOSITE side --
    restore that so the bar's L/R layout is preserved.  When the side cannot be
    inferred (no ``parent_bar_id``, no sibling tool, or an ambiguous layout)
    fall back to *default_tool* (the doc default active tool).
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    bar_id = rs.GetUserText(block_id, "parent_bar_id")
    if not bar_id:
        return default_tool

    sibling_sides: set = set()
    for layer in (config.LAYER_JOINT_MALE_INSTANCES, config.LAYER_JOINT_GROUND_INSTANCES):
        if not rs.IsLayer(layer):
            continue
        for other_id in rs.ObjectsByLayer(layer) or []:
            other_jid = rs.GetUserText(other_id, "joint_id")
            if not other_jid or other_jid == joint_id:
                continue
            if rs.GetUserText(other_id, "parent_bar_id") != bar_id:
                continue
            other_toid = find_tool_for_joint(other_jid)
            if other_toid is None:
                continue
            side = _robotic_tool.arm_side_from_tool_name(
                rs.GetUserText(other_toid, "tool_name") or ""
            )
            if side is not None:
                sibling_sides.add(side)

    if len(sibling_sides) == 1:
        opposite = "right" if next(iter(sibling_sides)) == "left" else "left"
        return active[opposite]
    return default_tool


def restore_missing_tools_at_joints(verbose: bool = False) -> int:
    """Re-place the active-pair tool on any joint block that lost its tool.

    Every male (``J*``) and ground (``G*``) joint block is auto-tooled the
    moment it is created (RSBarSnap / RSBarBrace / RSGroundPlace / RSJointEdit
    all call the ``auto_place_tool_*`` helpers), so the document convention is
    "every joint block carries exactly one tool instance".  A tool can still go
    missing -- most visibly after :func:`replace_all_tool_instances`
    (RSSwapRoboticTool) swaps to a tool of a DIFFERENT type, which deletes the
    old instances before re-inserting the new ones -- leaving the joint with no
    visible tool.

    For every joint block that currently has NO tool instance tagged to its
    ``joint_id``, this re-places the correct-side active-pair tool (inferred from
    the bar's surviving sibling tool by :func:`_restore_side_tool`, falling back
    to the doc default) via :func:`place_tool_at_block_instance`, so the tool
    re-appears on the joint's current frame with the L/R layout preserved.
    Joints that already carry a tool are left untouched.

    Returns the number of tools restored.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    active = _get_active_pair_or_none()
    if active is None:
        return 0  # no resolvable pair -> nothing to restore with (already noted)
    default_tool = _resolve_default_active_tool(active)

    n_placed = 0
    seen_joint_ids: set = set()
    for layer in (config.LAYER_JOINT_MALE_INSTANCES, config.LAYER_JOINT_GROUND_INSTANCES):
        if not rs.IsLayer(layer):
            continue
        for block_id in list(rs.ObjectsByLayer(layer) or []):
            joint_id = rs.GetUserText(block_id, "joint_id")
            if not joint_id or joint_id in seen_joint_ids:
                continue
            seen_joint_ids.add(joint_id)
            if find_tool_for_joint(joint_id) is not None:
                continue  # already tooled -- leave it alone
            tool = _restore_side_tool(block_id, joint_id, active, default_tool)
            if place_tool_at_block_instance(block_id, joint_id, tool) is not None:
                n_placed += 1
                if verbose:
                    print(f"  [tool] {joint_id}: restored missing tool '{tool.name}'.")
    return n_placed


def cycle_tool_at_tool_instance(tool_oid, *, pair=None) -> str | None:
    """Toggle the clicked tool instance between the ACTIVE pair's L/R tools.

    With candidate pairs in the registry, "cycling" is a side toggle within
    the active pair only: a joint holding the active LEFT tool gets the
    active RIGHT tool and vice versa.  Use RSSwapRoboticTool to change WHICH
    pair is active.  Returns the new tool name on success, ``None`` on
    failure.

    *pair* is kept for API parity with older callers; it is unused by the
    placement math (TCP is matched to the joint block origin).
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    joint_id = rs.GetUserText(tool_oid, "joint_id")
    current_name = rs.GetUserText(tool_oid, "tool_name") or ""
    if not joint_id:
        print("  [tool] clicked tool has no 'joint_id' user-text; cannot toggle.")
        return None

    try:
        active = _robotic_tool.get_active_pair()
    except (RuntimeError, ValueError) as exc:
        print(f"  [tool] cannot resolve the active tool pair: {exc}")
        return None

    side = _robotic_tool.arm_side_from_tool_name(current_name)
    if side is None:
        print(
            f"  [tool] clicked tool {current_name!r} has no L/R suffix; cannot "
            "decide which side to toggle. Run RSSwapRoboticTool to re-place "
            "all tools from the active pair."
        )
        return None
    if current_name not in (active["left"].name, active["right"].name):
        # Stale doc (tool placed before the last pair swap): still toggle the
        # side, but land on the active pair -- with a note, never silently.
        print(
            f"  [tool] clicked tool {current_name!r} is not in the active pair "
            f"({active['left'].name}/{active['right'].name}); toggling onto "
            "the active pair."
        )

    other_side = "right" if side == "left" else "left"
    tool = active[other_side]

    male_id = find_attached_block_for_joint(joint_id)
    if male_id is None:
        print(f"  [tool] could not locate joint block for {joint_id}.")
        return None

    new_oid = place_tool_at_block_instance(male_id, joint_id, tool)
    if new_oid is None:
        return None
    set_default_tool_name(tool.name)
    print(f"  [tool] {joint_id}: tool toggled '{current_name}' -> '{tool.name}'.")
    return tool.name
