#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.core_lib`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
# r: trimesh==4.12.2
"""RSLoadSolvedBarActionAll - Right-click companion of RSLoadSolvedBarAction.

Reads EVERY bar's solved plan from a design folder's ``solutions/`` (instead of one
picked bar), writes each bar's solved keyframes to its user text, and launches
RSShowBarActionPlan -- which then draws every loaded bar's base frame at once (the
assembly-wide map) and steps through them with NextBar/PrevBar. Shares
``load_and_view`` with the left-click script.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rs_load_solved_bar_action import load_and_view


def main() -> None:
    # Right-click: all bars in the folder.
    load_and_view(batch=True)


if __name__ == "__main__":
    main()
