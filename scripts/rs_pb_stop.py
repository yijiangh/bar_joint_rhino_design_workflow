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
"""RSPBStop - Close every planning world and drop the cached cells; stop the ssik sidecar.

The robots' planning mirrors (PyBullet worlds owned by the shared core), the
drawing cells and the preview meshes are closed or forgotten; the next IK
command or viewer rebuilds what it needs. The ssik IK sidecar (a Python 3.11
process, started by the "ssik" IK backend) is shut down too.
"""

from __future__ import annotations

import os
import sys

import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config  # noqa: F401  (puts the tamp submodule on sys.path)
import rs_rebuild_robot_cell


def main() -> None:
    rs_rebuild_robot_cell.close_all(sc.sticky)
    try:
        from husky_assembly_tamp.keyframe import ssik_client
        ssik_client.shutdown()
    except Exception as exc:  # noqa: BLE001 -- nothing to stop, or tamp missing
        print(f"RSPBStop: ssik sidecar not stopped ({exc}).")
    print("RSPBStop: closed the planning worlds and dropped the cached cells.")


if __name__ == "__main__":
    main()
