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
# r: txaio
"""RSPBStart - Warm up planning: build every robot's planning world now (optional).

No command needs this any more: the IK commands and viewers build each robot's
planning mirror (its own PyBullet world, owned by the shared core) on first use
and keep it for the session. Clicking it pays that load up front and runs the
same WalkableGround auto-assign + seed-base passes as RSRebuildRobotCell (it IS
RSRebuildRobotCell). The RSPBStartGUI right-click entry also opens the assembly
robot's planning world in PyBullet's window, for debugging. RSPBStop closes
them.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rs_rebuild_robot_cell


def main(use_gui: bool = False) -> None:
    rs_rebuild_robot_cell.refresh("RSPBStartGUI" if use_gui else "RSPBStart", gui=use_gui)


if __name__ == "__main__":
    main(use_gui=False)
