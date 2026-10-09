#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSShowBarActionPlanMotion - Right-click companion of RSShowBarActionPlan.

Pick a design folder (written by RSExportDesign, planned by
``scripts/plan_design.py``) and a bar, then step through the bar's planned
trajectories from the command line. Delegates to
``rs_show_bar_action_plan.main_motion`` (the left-click script owns the viewer).
"""

from __future__ import annotations

import importlib
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import rs_show_bar_action_plan


def main() -> None:
    importlib.reload(rs_show_bar_action_plan).main_motion()


if __name__ == "__main__":
    main()
