#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSUpdatePreview - bring the document back in line with the registries.

Left-click offers two jobs on the command line:

* **UpdatePreview** (default) -- the repair pass described below. It clears every
  color overlay, so it leaves a clean document.
* **ShowColorsPreview** -- read-only: color bars by IK status and mark the broken
  links (orphaned joints/tools, bars with no joint).

Right-click is RSClearColorPreview, which removes both overlays again.

The repair pass is idempotent: running it twice in a row reports the same and
changes nothing.

1. Bar tube previews -- regenerate any that are missing or geometrically stale.
2. Joint block definitions -- reload the ones whose ``asset/*.3dm`` changed since
   they were imported (edit a joint with RSDefineJointHalf, re-export it, click
   here and the document picks up the new geometry). The geometry is swapped IN
   PLACE, so every placed instance keeps its id, transform, name and user text.
3. Robotic tools -- restore tools that went missing entirely, give every bar one
   LEFT and one RIGHT tool (deterministically, by position along the bar), and
   re-snap any tool that drifted off its joint.
4. Report what it will not touch -- pairs whose halves no longer mate, joints and
   tools that lost their bar, tools no longer on their joint, and bars carrying
   no joint. Run ShowColorsPreview (or right-click) to see them highlighted.

This command never moves a joint. Re-deriving a solved placement is not reliable
enough to do silently; fix a reported joint with RSJointEdit / RSJointPlace /
RSGroundPlace instead. One consequence: editing ``M_block_from_bar`` in
``joint_pairs.json`` affects only NEWLY placed joints -- re-place an existing one
to adopt a changed transform.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_bar_registry import clear_ik_preview, repair_on_entry, update_all_previews
from core.rhino_joint_refresh import (
    clear_broken_link_marks,
    find_broken_links,
    refresh_stale_joint_blocks,
    report_unmated_joints,
    show_colors_preview,
)
from core.rhino_tool_place import (
    enforce_bar_tool_sides,
    resync_tools_to_joints,
    restore_missing_tools_at_joints,
)


def _run_update_preview():
    """The repair pass. Rebuilds previews, reloads blocks, fixes tools."""
    # Run the standard entry-point repair first: this purges orphan tube
    # previews left behind by user copy/paste (axis_id pointing at another
    # bar OR self_guid != actual GUID) before we regenerate the canonical
    # tubes. Without this pass, copy-pasting a bar+tube to a new spot
    # leaves the duplicate tube on the layer forever.
    repair_on_entry(float(config.BAR_RADIUS), caller="RSUpdatePreview")
    # repair_on_entry already invokes update_all_previews internally; the
    # second verbose pass below is purely diagnostic so the user sees the
    # per-bar reused/regenerated/created tally.
    n_changed = update_all_previews(float(config.BAR_RADIUS), verbose=False)
    if n_changed:
        print(f"RSUpdatePreview: regenerated/created {n_changed} bar preview(s).")
    else:
        print("RSUpdatePreview: all bar previews already up to date.")

    # Clear every overlay: this is the repair pass, so it leaves a clean document.
    # The ShowColorsPreview option paints them again on demand.
    n_cleared = clear_ik_preview()
    n_marks = clear_broken_link_marks()
    print(
        f"RSUpdatePreview: cleared the color preview "
        f"({n_cleared} bar(s), {n_marks} other object(s) reverted)."
    )

    # Reload joint blocks whose asset .3dm changed since it was imported. This is
    # the joint-side counterpart of what RSSwapRoboticTool does for tools: every
    # joint placement path uses require_block_definition, which skips the import
    # when a definition of that name already exists, so an edited block would
    # otherwise never reach an open document.
    n_blocks = refresh_stale_joint_blocks(verbose=False)
    if n_blocks:
        print(
            f"RSUpdatePreview: reloaded {n_blocks} joint block definition(s) from "
            "asset/ (instances kept in place)."
        )

    # Restore tools that went missing entirely -- e.g. after RSSwapRoboticTool
    # swaps to a tool of a different type, a joint can be left with no visible
    # tool. Re-place the active pair's default tool on any joint block that has
    # lost its tool (every joint block is meant to carry one). Do this BEFORE the
    # drift re-snap: restored tools land exactly on the joint, so the re-snap
    # below then only has to touch genuinely drifted ones.
    n_restored = restore_missing_tools_at_joints(verbose=False)
    if n_restored:
        print(f"RSUpdatePreview: restored {n_restored} missing tool(s) onto their joints.")

    # Every bar carries one LEFT and one RIGHT tool. The restore pass above infers
    # the side from the bar's sibling joint, which makes the outcome depend on
    # document order and leaves both joints on the same side when the sibling
    # lookup fails. Re-decide it geometrically so the layout is reproducible.
    n_sides = enforce_bar_tool_sides(verbose=False)
    if n_sides:
        print(f"RSUpdatePreview: corrected the L/R side of {n_sides} tool(s).")

    # Snap any tool that drifted away from its joint back onto it (reuses the
    # canonical RSBarSnap/RSBarBrace placement path). Tools already on their
    # joints are left untouched.
    n_tools = resync_tools_to_joints(verbose=False)
    if n_tools:
        print(f"RSUpdatePreview: re-snapped {n_tools} drifted tool(s) onto their joints.")

    # Report-only from here on. Pairs whose halves no longer mate, judged by the
    # same interface tolerances the solver accepts a variant with.
    unmated = report_unmated_joints(verbose=True)
    if unmated:
        print(
            f"RSUpdatePreview: {len(unmated)} joint(s) whose halves no longer mate "
            "-- re-run RSJointEdit on them (nothing was moved)."
        )

    # Finally, count what cannot be repaired automatically: joints/tools whose bar
    # is gone, tools no longer on their joint, and bars carrying no joint. The
    # repair pass only counts them -- ShowColorsPreview paints and lists them.
    links = find_broken_links()
    n_broken = len(links["orphans"]) + len(links["bare_bars"])
    if n_broken:
        print(
            f"RSUpdatePreview: {len(links['orphans'])} orphaned joint/tool(s) and "
            f"{len(links['bare_bars'])} bar(s) with no joint -- run the "
            "ShowColorsPreview option (or right-click this button) to see them."
        )
    else:
        print("RSUpdatePreview: no broken bar/joint/tool links.")

    rs.Redraw()


def main():
    """Ask which of the two jobs to run, then run it.

    Right-click (RSClearColorPreview) removes whatever ShowColorsPreview painted.
    """
    importlib.reload(config)

    choice = rs.GetString(
        "RSUpdatePreview",
        "UpdatePreview",
        ["UpdatePreview", "ShowColorsPreview"],
    )
    if choice is None:
        print("RSUpdatePreview: Cancelled.")
        return
    if choice.strip().lower().startswith("s"):
        show_colors_preview()
        return
    _run_update_preview()


if __name__ == "__main__":
    main()