#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSPBStart - Start the shared PyBullet client for IK / FK workflows.

Left-click starts a Direct (headless) connection; the RSPBStartGUI right-click
entry starts a GUI connection. Either way the dual-arm Husky robot cell is
loaded into a cached PyBullet planner. Subsequent IK scripts (RSIKKeyframe,
RSShowIK) reuse this same client via `sc.sticky`.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.robot_cell import is_pb_running, start_pb_client


def main(use_gui: bool = False) -> None:
    if is_pb_running():
        print("RSPBStart: PyBullet client already running. Run RSPBStop first if you want to restart.")
        return

    print(f"RSPBStart: Starting PyBullet ({'GUI' if use_gui else 'Direct'}) and loading robot cell...")
    client, _planner = start_pb_client(use_gui=use_gui)
    print(f"RSPBStart: Robot cell loaded. Client id={client.client_id}.")


if __name__ == "__main__":
    main(use_gui=False)
