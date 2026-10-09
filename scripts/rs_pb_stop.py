#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

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
