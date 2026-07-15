#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowBarActionPlanMotion - Right-click companion of RSShowBarActionPlan.

Pick a bar, load its planned motion (`<bar>.solved_motion.json`) if it isn't
cached yet, and open a slider pop-up to scrub the M1..M4 trajectory. Delegates to
``rs_show_bar_action_plan.main_motion`` (the left-click script owns the viewer).
"""

from __future__ import annotations

import importlib
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rs_show_bar_action_plan


def main() -> None:
    importlib.reload(rs_show_bar_action_plan).main_motion()


if __name__ == "__main__":
    main()
