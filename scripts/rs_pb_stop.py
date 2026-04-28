#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSPBStop - Disconnect the shared PyBullet client."""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.robot_cell import is_pb_running, stop_pb_client


def main() -> None:
    if not is_pb_running():
        print("RSPBStop: No PyBullet client is currently running.")
        return
    stop_pb_client()
    print("RSPBStop: Disconnected.")


if __name__ == "__main__":
    main()
