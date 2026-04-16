#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSCreateBar - Register selected curves as scaffolding bars.

Select one or more curves. Each gets a unique bar ID (B1, B2, …) and a tube
preview at the configured BAR_RADIUS.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_bar_registry import ensure_bar_id, ensure_bar_preview


def main():
    importlib.reload(config)
    curve_ids = rs.GetObjects("Select curves to register as bars", rs.filter.curve)
    if not curve_ids:
        return
    for cid in curve_ids:
        bar_id = ensure_bar_id(cid)
        ensure_bar_preview(cid, float(config.BAR_RADIUS), bar_id=bar_id)
        print(f"RSCreateBar: Registered {bar_id}")
    print(f"RSCreateBar: {len(curve_ids)} bar(s) registered.")


if __name__ == "__main__":
    main()
