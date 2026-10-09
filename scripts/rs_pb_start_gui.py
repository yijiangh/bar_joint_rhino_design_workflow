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
"""RSPBStartGUI - Right-click entry for RSPBStart: warm up planning and show the assembly robot's world.

Thin wrapper around `rs_pb_start.main`: the assembly robot's planning mirror
opens in PyBullet's own window (one per process), for debugging what the
planner checks.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rs_pb_start import main


if __name__ == "__main__":
    main(use_gui=True)
