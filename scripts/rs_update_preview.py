#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSUpdatePreview - Refresh tube previews and re-snap drifted tools.

Scans the document for every bar tagged with a bar_id, checks whether its
tube preview is present and geometrically current, and regenerates any that
are missing or stale.

It then re-snaps any robotic-tool instance that has drifted away from the
joint it belongs to: a tool whose TCP frame no longer coincides with its
joint block (because the user nudged the tool, or moved/rebuilt the joint
after placement) is moved back onto the joint's current frame, reusing the
same placement routine that RSBarSnap / RSBarBrace auto-placement use. Tools
already sitting on their joints are left untouched.
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
from core.rhino_tool_place import resync_tools_to_joints, restore_missing_tools_at_joints


def main():
    importlib.reload(config)
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

    # Restore tools that went missing entirely -- e.g. after RSSwapRoboticTool
    # swaps to a tool of a different type, a joint can be left with no visible
    # tool. Re-place the active pair's default tool on any joint block that has
    # lost its tool (every joint block is meant to carry one). Do this BEFORE the
    # drift re-snap: restored tools land exactly on the joint, so the re-snap
    # below then only has to touch genuinely drifted ones.
    n_restored = restore_missing_tools_at_joints(verbose=False)
    if n_restored:
        print(f"RSUpdatePreview: restored {n_restored} missing tool(s) onto their joints.")

    # Snap any tool that drifted away from its joint back onto it (reuses the
    # canonical RSBarSnap/RSBarBrace placement path). Tools already on their
    # joints are left untouched.
    n_tools = resync_tools_to_joints(verbose=False)
    if n_tools:
        print(f"RSUpdatePreview: re-snapped {n_tools} drifted tool(s) onto their joints.")

    # Clean up the IK color preview: revert any orange/magenta bar overrides left
    # by RSIKKeyframeAll back to by-layer. Right-click (RSShowIKPreview) re-shows
    # them. Shares the color helpers in core.rhino_bar_registry.
    n_cleared = clear_ik_preview()
    if n_cleared:
        print(f"RSUpdatePreview: cleared IK color preview on {n_cleared} bar(s).")


if __name__ == "__main__":
    main()
