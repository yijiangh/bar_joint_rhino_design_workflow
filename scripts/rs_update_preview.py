#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSUpdatePreview - bring the document back in line with the registries.

**Left-click** runs the repair pass below, then paints the diagnostic overlay
(bars by IK status, broken links marked and selected) and pops up one tally of
everything repaired, everything still broken, and everything checked.

**Right-click** is RSClearColorPreview, which removes the overlay again -- except
the fake-bar tint, which is a property of the model rather than a diagnostic.

There is no job prompt: seeing what a repair pass could not fix is part of
running it, so what used to be the separate ``ShowColorsPreview`` option now
always runs.

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
   no joint. These are painted, selected and counted in the popup.

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
from core.rhino_bar_registry import repair_on_entry, update_all_previews
from core.rhino_joint_refresh import (
    refresh_stale_joint_blocks,
    report_joint_usertext_issues,
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

    # No overlay clear here: show_colors_preview() at the end of this pass clears
    # and repaints it anyway (via clear_broken_link_marks + show_all_ik_preview),
    # so clearing up front would just be a wasted document-wide pass.

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
    # Always report, even when nothing was restored: a pass that examined every
    # joint and changed nothing used to be indistinguishable from a healthy
    # model, which is the one case you actually need to debug.
    tool_counts = restore_missing_tools_at_joints(verbose=False)
    print(
        "RSUpdatePreview: tools -- {checked} joint(s) checked, "
        "{already_tooled} already tooled, {restored} restored.".format(**tool_counts)
    )
    _broken = {k: v for k, v in tool_counts.items() if v and k in (
        "no_joint_id", "duplicate_joint_id", "place_failed"
    )}
    if _broken:
        print(
            "RSUpdatePreview: tools -- NOT restorable: "
            + ", ".join(f"{v} x {k}" for k, v in _broken.items())
            + ".\n  no_joint_id / duplicate_joint_id: the joint block's user text is "
            "wrong, so no tool can be tagged to it.\n"
            "  If the checked count is lower than the number of joints you expect, "
            "a joint block is on the wrong layer and this pass never saw it."
        )

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

    # Stale / copied joint user text. The IK collision scene names bodies from
    # the joint_id USER TEXT (not the visible object name), so a Rhino-copied
    # block with donor user text makes IK report phantom joint names and lose
    # its whitelists. Detect it here; the repair is RSReorderBarID -> Relink.
    usertext_issues = report_joint_usertext_issues(verbose=True)
    if usertext_issues:
        print(
            f"RSUpdatePreview: {len(usertext_issues)} joint user-text issue(s) -- "
            "run RSReorderBarID -> Relink (review its plan before applying), then "
            "RSRebuildRobotCell."
        )

    # Paint the diagnostic overlay and count what could NOT be repaired
    # automatically: joints/tools whose bar is gone, tools no longer on their
    # joint, bars carrying no joint. This used to be a separate
    # ShowColorsPreview option -- repairing and then seeing what is left over is
    # one job, so it always runs.
    links = show_colors_preview()

    _show_summary(tool_counts, n_sides, n_tools, unmated, usertext_issues, links)
    rs.Redraw()


def _show_summary(tool_counts, n_sides, n_tools, unmated, usertext_issues, links):
    """Pop up one report covering every pass, in one format.

    A popup rather than command-line output because the colour legend and the
    per-object lists scroll the history away exactly when you need to compare
    them against what is painted in the viewport.  The full detail still goes to
    the command line; this is the tally.
    """
    n_orphans = len(links["orphans"])
    n_bare = len(links["bare_bars"])
    lines = [
        "Repaired",
        f"  {tool_counts['restored']} missing tool(s) re-placed",
        f"  {n_sides} tool(s) had their L/R side corrected",
        f"  {n_tools} drifted tool(s) re-snapped onto their joint",
        "",
        "Needs your attention",
        f"  {n_orphans} orphaned joint/tool(s) -- parent bar is gone",
        f"  {n_bare} bar(s) carrying no joint",
        f"  {len(unmated)} joint(s) whose halves no longer mate -- re-run RSJointEdit",
        f"  {len(usertext_issues)} joint user-text issue(s) (stale/copied ids) -- "
        "RSReorderBarID > Relink",
    ]

    broken_text = [
        (tool_counts["no_joint_id"], "joint block(s) with no joint_id user text"),
        (tool_counts["duplicate_joint_id"], "joint block(s) sharing a joint_id"),
        (tool_counts["place_failed"], "tool(s) that failed to place"),
    ]
    for count, label in broken_text:
        if count:
            lines.append(f"  {count} {label}")

    lines += [
        "",
        "Checked",
        f"  {tool_counts['checked']} joint block(s), "
        f"{tool_counts['already_tooled']} already tooled",
    ]
    if tool_counts["checked"] < n_orphans + tool_counts["already_tooled"]:
        lines.append(
            "  NOTE: fewer joint blocks were checked than exist -- one is on the "
            "wrong layer"
        )
    lines += [
        "",
        "Problems are painted and selected in the viewport.",
        "Right-click this button (RSClearColorPreview) to remove the overlay.",
    ]
    rs.MessageBox("\n".join(lines), 0, "RSUpdatePreview")


def main():
    """Repair the model, paint what is still broken, and report the tally.

    One job, not two: the old ``ShowColorsPreview`` prompt is gone because
    seeing what a repair pass could not fix is part of running it.  Right-click
    (RSClearColorPreview) removes the overlay this paints.
    """
    importlib.reload(config)
    _run_update_preview()


if __name__ == "__main__":
    main()