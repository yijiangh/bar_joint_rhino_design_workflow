#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
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
from core.rhino_bar_registry import update_all_previews


def main():
    importlib.reload(config)
    count = update_all_previews(float(config.BAR_RADIUS))
    print(f"RSUpdatePreview: Updated {count} bar preview(s).")


if __name__ == "__main__":
    main()
