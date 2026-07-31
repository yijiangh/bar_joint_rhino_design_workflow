"""Reload edited joint blocks and report joints/tools that no longer hold together.

Motivation
----------
Every joint placement path goes through
``core.rhino_block_import.require_block_definition``, which short-circuits when a
definition of that name already exists in the document.  So after re-running
RSDefineJointHalf and re-exporting a joint's ``asset/*.3dm``, Rhino keeps drawing
the OLD geometry forever -- there was no joint-side equivalent of what
RSSwapRoboticTool does for robotic tools.  :func:`refresh_stale_joint_blocks`
closes that gap.

What this module does NOT do is move geometry.  The checks here reuse the
definitions the *creation* side already owns, and report:

* whether a placed pair still mates -- via ``screw_alignment_diagnostics`` and
  the ``VARIANT_OK_*`` tolerances from :mod:`core.joint_placement`, i.e. the
  exact test the solver applies when it accepts a variant;
* whether a tool still sits on its joint -- via
  ``core.rhino_tool_place.is_tool_on_joint``, inverted from the routine that
  places tools in the first place;
* whether a joint/tool still has a bar, and whether a bar has any joint.

Anything that fails is surfaced (selected + listed) for the user to fix with
RSJointEdit / RSJointPlace / RSGroundPlace.  Re-deriving a solved placement from
stored ``(jp, jr)`` is deliberately not attempted: the reconstruction is not a
trustworthy authority on where a solved joint belongs, and acting on it moved
correct joints.

Rhino-runtime only (Rhino 8), like :mod:`core.joint_relink`.
"""

from __future__ import annotations

import numpy as np
import rhinoscriptsyntax as rs

from core import config
from core.joint_pair import load_joint_registry
from core.joint_pair_solver import screw_alignment_diagnostics
from core.joint_pick_helpers import block_instance_frame
from core.joint_placement import (
    VARIANT_OK_ORIGIN_TOL_MM,
    VARIANT_OK_Z_AXIS_TOL_RAD,
)
from core.rhino_bar_registry import (
    _bar_curve_and_tube,
    get_bar_seq_map,
    paint_bar,
    reset_bar_color,
)
from core.rhino_block_import import (
    asset_stamp,
    block_asset_stamp,
    update_block_definition_geometry,
)
from core.rhino_helpers import ensure_layer, set_object_color, suspend_redraw


# ---------------------------------------------------------------------------
# Layers
# ---------------------------------------------------------------------------

#: Layers holding baked joint block instances, in the order they are scanned.
JOINT_LAYERS = (
    config.LAYER_JOINT_FEMALE_INSTANCES,
    config.LAYER_JOINT_MALE_INSTANCES,
    config.LAYER_JOINT_GROUND_INSTANCES,
)


# ---------------------------------------------------------------------------
# Small Rhino helpers
# ---------------------------------------------------------------------------


def _layer_oids(layer):
    """Return every object id on *layer* (empty when the layer does not exist)."""
    if not rs.IsLayer(layer):
        return []
    return list(rs.ObjectsByLayer(layer) or [])


def _reset_color(oid) -> None:
    """Revert *oid*'s color override back to by-layer."""
    if rs.IsObject(oid) and hasattr(rs, "ObjectColorSource"):
        rs.ObjectColorSource(oid, 0)  # 0 == by layer


def _joint_block_instances():
    """Yield ``(oid, layer, joint_id, block_name)`` per baked joint block."""
    for layer in JOINT_LAYERS:
        for oid in _layer_oids(layer):
            # Stray non-block objects on a managed layer are not our business
            # (repair_on_entry evicts them).
            if not rs.IsBlockInstance(oid):
                continue
            yield (
                oid,
                layer,
                rs.GetUserText(oid, "joint_id") or "",
                rs.BlockInstanceName(oid) or "",
            )


# ---------------------------------------------------------------------------
# Reload edited joint block definitions
# ---------------------------------------------------------------------------


def refresh_stale_joint_blocks(verbose: bool = False) -> int:
    """Reload every joint block whose asset .3dm changed since it was imported.

    Change detection is the asset stamp (source file mtime + size) recorded in
    the definition's description by :mod:`core.rhino_block_import`.  A block with
    no stamp -- a document saved before stamping existed -- counts as changed and
    is reloaded once; the stamp is written on the way in, so the next run is a
    no-op.

    The reload itself is :func:`~core.rhino_block_import.update_block_definition_geometry`,
    which swaps the definition's geometry IN PLACE.  Nothing is deleted and
    nothing is re-created, so every instance keeps its id, world transform,
    object name and user text.

    Args:
        verbose (bool): print a line per block considered.

    Returns:
        int: the number of block definitions reloaded.
    """
    registry = load_joint_registry()

    # block_name -> (asset_path, layer to pin imported sub-objects onto)
    candidates: dict = {}
    for _oid, layer, _joint_id, block_name in _joint_block_instances():
        if not block_name or block_name in candidates:
            continue
        half = registry.halves.get(block_name)
        ground = next(
            (g for g in registry.ground_joints.values() if g.block_name == block_name),
            None,
        )
        definition = half or ground
        if definition is None:
            if verbose:
                print(
                    f"  [joint] block {block_name!r} is not registered in "
                    "joint_pairs.json; skipping."
                )
            continue
        candidates[block_name] = (definition.asset_path(), layer)

    n_reloaded = 0
    with suspend_redraw():
        for block_name, (asset_path, layer) in sorted(candidates.items()):
            if not asset_path:
                print(
                    f"  [joint] {block_name}: no asset_filename in joint_pairs.json; "
                    "cannot check whether it is up to date."
                )
                continue
            disk_stamp = asset_stamp(asset_path)
            if not disk_stamp:
                print(
                    f"  [joint] {block_name}: asset file missing at {asset_path}; "
                    "skipping reload (re-run RSDefineJointHalf to export it)."
                )
                continue
            if block_asset_stamp(block_name) == disk_stamp:
                continue  # unchanged since import -- leave the definition alone
            if update_block_definition_geometry(
                block_name, asset_path, layer_name=layer
            ):
                n_reloaded += 1
    return n_reloaded


# ---------------------------------------------------------------------------
# Does a placed pair still mate?  (same test the solver accepts a variant with)
# ---------------------------------------------------------------------------


def _screw_frame(oid, half) -> np.ndarray:
    """World screw frame of a placed half: ``block_world @ M_screw_from_block``.

    The same composition ``core.joint_pair.fk_half_from_bar_frame`` uses, but
    driven by the block's ACTUAL transform in the document rather than a
    recomputed one -- so this reports where the joint really is.
    """
    block_world, _name = block_instance_frame(oid)
    return np.asarray(block_world, dtype=float) @ np.asarray(
        half.M_screw_from_block, dtype=float
    )


def report_unmated_joints(verbose: bool = False) -> list:
    """List placed female/male pairs whose halves no longer mate.  Read-only.

    Reuses the creation-side definition of a good interface: the female and male
    screw frames must coincide within ``VARIANT_OK_ORIGIN_TOL_MM`` /
    ``VARIANT_OK_Z_AXIS_TOL_RAD``, measured by
    ``core.joint_pair_solver.screw_alignment_diagnostics`` -- exactly the check
    ``core.joint_placement.is_variant_acceptable`` applies when the solver picks
    a variant.  So "mated" means the same thing here as it did at placement time.

    Ground joints have no partner and are not checked (a ground joint can only be
    broken by losing its bar, which :func:`find_broken_links` covers).

    Args:
        verbose (bool): print each offending joint with its measured errors.

    Returns:
        list[tuple]: ``(joint_id, origin_error_mm, z_axis_error_rad)`` per pair
        that is out of tolerance.
    """
    registry = load_joint_registry()

    # joint_id -> {"female": (oid, half), "male": (oid, half)}
    pairs: dict = {}
    for oid, layer, joint_id, block_name in _joint_block_instances():
        if not joint_id or layer == config.LAYER_JOINT_GROUND_INSTANCES:
            continue
        half = registry.halves.get(block_name)
        if half is None:
            continue
        role = (
            "female" if layer == config.LAYER_JOINT_FEMALE_INSTANCES else "male"
        )
        pairs.setdefault(joint_id, {})[role] = (oid, half)

    unmated = []
    for joint_id in sorted(pairs):
        sides = pairs[joint_id]
        if "female" not in sides or "male" not in sides:
            continue  # half a pair -- find_broken_links reports the survivor
        try:
            female = _screw_frame(*sides["female"])
            male = _screw_frame(*sides["male"])
        except (ValueError, AttributeError) as exc:
            if verbose:
                print(f"  [joint] {joint_id}: cannot read block frame ({exc}).")
            continue
        diag = screw_alignment_diagnostics(female, male)
        origin_err = diag["origin_error_mm"]
        z_err = diag["z_axis_error_rad"]
        if (
            origin_err <= VARIANT_OK_ORIGIN_TOL_MM
            and abs(z_err) <= VARIANT_OK_Z_AXIS_TOL_RAD
        ):
            continue
        unmated.append((joint_id, origin_err, z_err))
        if verbose:
            print(
                f"  [joint] {joint_id}: halves do not mate "
                f"(origin {origin_err:.3f} mm, z-axis {np.degrees(z_err):.2f} deg)."
            )
    return unmated


# ---------------------------------------------------------------------------
# Broken links: joints/tools with no bar, bars with no joint, detached tools
# ---------------------------------------------------------------------------


def find_broken_links() -> dict:
    """Report model links that no longer resolve.  Read-only.

    Returns:
        dict: ``{"orphans": [(oid, kind, label), ...],
        "bare_bars": [(oid, bar_id), ...]}`` where *kind* is ``"joint"`` or
        ``"tool"``.

        * *orphans* -- joint instances whose ``parent_bar_id`` is empty or names a
          bar that is not in the document; female/male halves whose partner half
          is missing (a joint is only a joint with both sides); plus every tool
          ``core.rhino_tool_place.find_detached_tools`` reports (no joint, joint
          gone, or geometrically off its joint).  A tool whose joint survives but
          whose joint's bar is gone counts too, otherwise only the joint under it
          would be flagged and the tool would look fine.
        * *bare_bars* -- registered bars carrying no joint instance at all.

    None of these can be repaired automatically: the bar is gone, or the joint
    needs RSJointPlace / RSGroundPlace.
    """
    from core.rhino_tool_place import find_detached_tools  # noqa: PLC0415

    bar_map = get_bar_seq_map()
    orphans = []
    bars_with_joints: set = set()
    orphan_joint_ids: set = set()

    # Pass 1: which joint_ids have which halves?  A female with no male (or the
    # reverse) is as broken as one with no bar -- there is nothing for it to mate
    # with.  Ground joints are single-sided by design and never counted here.
    halves_by_joint: dict = {}
    for oid, layer, joint_id, _block_name in _joint_block_instances():
        if not joint_id or layer == config.LAYER_JOINT_GROUND_INSTANCES:
            continue
        halves_by_joint.setdefault(joint_id, set()).add(layer)

    for oid, layer, joint_id, _block_name in _joint_block_instances():
        bar_id = rs.GetUserText(oid, "parent_bar_id") or ""
        reason = ""
        if not bar_id or bar_id not in bar_map:
            reason = f"parent bar {bar_id or '<none>'} is gone"
        elif (
            joint_id
            and layer != config.LAYER_JOINT_GROUND_INSTANCES
            and len(halves_by_joint.get(joint_id, ())) < 2
        ):
            missing = (
                "male"
                if layer == config.LAYER_JOINT_FEMALE_INSTANCES
                else "female"
            )
            reason = f"its {missing} half is missing"

        if not reason:
            bars_with_joints.add(bar_id)
            continue
        if joint_id:
            orphan_joint_ids.add(joint_id)
        # A joint that still has its bar keeps that bar off the "bare" list --
        # the bar is not jointless, its joint is just incomplete.
        if bar_id and bar_id in bar_map:
            bars_with_joints.add(bar_id)
        orphans.append(
            (oid, "joint", f"joint {joint_id or '<no joint_id>'}: {reason}")
        )

    reported_tools: set = set()
    for tool_oid, joint_id, reason in find_detached_tools():
        reported_tools.add(tool_oid)
        label = rs.ObjectName(tool_oid) or str(tool_oid)
        orphans.append(
            (tool_oid, "tool", f"tool {label} ({joint_id or '<no joint>'}): {reason}")
        )

    for tool_oid in _layer_oids(config.LAYER_TOOL_INSTANCES):
        if tool_oid in reported_tools:
            continue
        joint_id = rs.GetUserText(tool_oid, "joint_id") or ""
        if joint_id not in orphan_joint_ids:
            continue
        label = rs.ObjectName(tool_oid) or str(tool_oid)
        orphans.append(
            (tool_oid, "tool", f"tool {label} ({joint_id}): its joint is broken")
        )

    bare_bars = [
        (oid, bar_id)
        for bar_id, (oid, _seq) in sorted(bar_map.items())
        if bar_id not in bars_with_joints
    ]
    return {"orphans": orphans, "bare_bars": bare_bars}


#: UserText key tagging the throwaway marker dots, so the cleanup can find them
#: even if the layer was renamed.
_MARK_KEY = "rs_diagnostic_mark"


def _drop_tool_marker(oid) -> None:
    """Put a colored text dot carrying the tool's id (``T<joint_id>``) on *oid*.

    Tools get a dot because they are the one object type that cannot be
    recolored: a block instance's color only reaches sub-objects whose color
    source is "by parent", and the robotic-tool assets carry baked colors (that
    is what makes them read as red/green).  Joints take the color override fine,
    so they get no dot -- the label would only clutter the model.
    """
    box = rs.BoundingBox(oid)
    if not box:
        return
    centre = [
        sum(corner[i] for corner in box) / float(len(box)) for i in range(3)
    ]
    dot = rs.AddTextDot(rs.ObjectName(oid) or str(oid), centre)
    if dot is None:
        return
    ensure_layer(config.LAYER_DIAGNOSTIC_MARKS)
    rs.ObjectLayer(dot, config.LAYER_DIAGNOSTIC_MARKS)
    set_object_color(dot, config.ORPHAN_LINK_COLOR)
    rs.SetUserText(dot, _MARK_KEY, "orphan")


def mark_broken_links(links: dict) -> int:
    """Highlight the objects reported by :func:`find_broken_links`.

    Three channels, because no single one covers every object type:

    * **color** -- joint blocks take an ``ObjectColor`` override; bars go through
      ``core.rhino_bar_registry.paint_bar``, which colors the centre-line AND its
      tube preview (coloring the curve alone is invisible, the tube covers it).
    * **marker dot** -- tools only, labelled with the tool id (``T<joint_id>``).
      They are the one type the color override cannot reach; see
      :func:`_drop_tool_marker`.
    * **selection** -- everything flagged is left selected, ready to inspect or
      delete.  A bare bar contributes BOTH its centre-line curve and its tube
      preview (``_bar_curve_and_tube``), so deleting the selection removes the
      whole bar rather than leaving the tube behind.  Nothing is deleted here --
      that is the user's call.

    Returns:
        int: the number of flagged items (bars count once, not once per object).
    """
    orphans = links.get("orphans", [])
    bare_bars = links.get("bare_bars", [])

    with suspend_redraw():
        for oid, kind, _label in orphans:
            set_object_color(oid, config.ORPHAN_LINK_COLOR)
            if kind == "tool":
                _drop_tool_marker(oid)
        selection = [oid for oid, _kind, _label in orphans]
        for oid, _bar_id in bare_bars:
            paint_bar(oid, config.BARE_BAR_COLOR)
            selection.extend(_bar_curve_and_tube(oid))
        if selection:
            rs.SelectObjects(selection)
    return len(orphans) + len(bare_bars)


def clear_broken_link_marks() -> int:
    """Undo :func:`mark_broken_links`: by-layer color, no dots, nothing selected.

    Baked joints and tools are always inserted without a color override, and bars
    without one outside a preview, so resetting them is safe -- it only removes
    what a previous run put there.

    Returns:
        int: the number of objects reset or deleted.
    """
    n = 0
    with suspend_redraw():
        # Sweep by LAYER, not via get_bar_seq_map(): a bar only appears in that
        # map when it has both a bar_id and a parseable sequence number, and a
        # tube is only reachable from its curve while the tube_axis_id link
        # holds. Either gap would strand a colored bar that nothing then clears.
        # The layers themselves are the reliable inventory. Tubes are baked
        # without a color override (`ensure_bar_preview` passes color=None), so
        # reverting them to by-layer restores their normal appearance -- and
        # `ensure_bar_preview` REUSES a geometrically-current tube without
        # repainting it, which is why clearing here is the only thing that can
        # remove a preview color.
        for layer in JOINT_LAYERS + (
            config.LAYER_TOOL_INSTANCES,
            config.LAYER_BAR_CENTERLINES,
            config.LAYER_BAR_TUBE_PREVIEWS,
        ):
            for oid in _layer_oids(layer):
                _reset_color(oid)
                n += 1
        # Belt and braces: a registered bar that somehow sits off the managed
        # centre-line layer still gets reverted, tube included.
        for _bar_id, (oid, _seq) in get_bar_seq_map().items():
            reset_bar_color(oid)
        for oid in _layer_oids(config.LAYER_DIAGNOSTIC_MARKS):
            if rs.GetUserText(oid, _MARK_KEY):
                rs.DeleteObject(oid)
                n += 1
        rs.UnselectAllObjects()
    return n


def broken_link_legend_lines() -> list:
    """Return the legend for the broken-link report."""
    return [
        "Broken links -- everything listed below is SELECTED in the viewport:",
        "  orange = joint that lost its bar or its other half, or a tool that "
        "lost its joint / is no longer on it. Tools also get an orange dot "
        "labelled with their tool id, because a tool block cannot be recolored "
        "(its asset has baked colors).",
        "  dark indigo = registered bar carrying no joint. Its centre-line AND "
        "its tube preview are both selected, so Delete removes the whole bar; "
        "or give it a joint with RSJointPlace / RSGroundPlace.",
        "  RSUpdatePreview right-click (RSClearColorPreview) clears all of this.",
    ]


def print_broken_links(links: dict, cap: int = 20) -> None:
    """Print the broken-link report, capped so a large model stays readable."""
    orphans = links.get("orphans", [])
    bare_bars = links.get("bare_bars", [])
    if not orphans and not bare_bars:
        return
    for line in broken_link_legend_lines():
        print(line)
    for _oid, _kind, label in orphans[:cap]:
        print(f"  [orphan] {label}")
    if len(orphans) > cap:
        print(f"  [orphan] ... and {len(orphans) - cap} more")
    for _oid, bar_id in bare_bars[:cap]:
        print(f"  [bare bar] {bar_id} has no joint")
    if len(bare_bars) > cap:
        print(f"  [bare bar] ... and {len(bare_bars) - cap} more")


def show_colors_preview() -> dict:
    """Paint every diagnostic overlay at once: IK status + broken links.

    The read-only counterpart of the RSUpdatePreview repair pass, reached through
    that command's ``ShowColorsPreview`` option.  RSClearColorPreview (right-click)
    removes everything this paints.

    Bars are colored by IK status (``show_all_ik_preview`` reverts stale bar
    colors itself), then the broken links are painted and selected on top --
    broken beats IK, since a bar with no joint is the more urgent problem.

    Returns:
        dict: the :func:`find_broken_links` result, for the caller to summarise.
    """
    from core.rhino_bar_registry import (  # noqa: PLC0415
        print_ik_preview_legend,
        show_all_ik_preview,
    )

    clear_broken_link_marks()

    n_has_ik, n_total = show_all_ik_preview()
    print_ik_preview_legend()
    print(f"  {n_has_ik}/{n_total} bar(s) have a solved IK keyframe.")

    links = find_broken_links()
    if mark_broken_links(links):
        print_broken_links(links)
    else:
        print("  no broken bar/joint/tool links.")
    return links


__all__ = [
    "JOINT_LAYERS",
    "broken_link_legend_lines",
    "clear_broken_link_marks",
    "find_broken_links",
    "mark_broken_links",
    "print_broken_links",
    "refresh_stale_joint_blocks",
    "report_unmated_joints",
    "show_colors_preview",
]