#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSUpdatePreview - Refresh tube previews for all registered bars.

Scans the document for every bar tagged with a bar_id, checks whether its
tube preview is present and geometrically current, and regenerates any that
are missing or stale.
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


if __name__ == "__main__":
    main()
