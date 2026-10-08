#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSPBStop - Disconnect ALL per-robot PyBullet sessions."""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core.robot_cell import _STICKY, _STICKY_PB_SESSIONS, stop_pb_client


def main() -> None:
    # Check the raw session registry, NOT is_pb_running(): that helper only
    # answers for Cindy, and we must still clean up when Cindy's GUI died but
    # the headless support sessions are alive. stop_pb_client() itself is
    # robust to sessions in any state.
    if not _STICKY.get(_STICKY_PB_SESSIONS):
        print("RSPBStop: No PyBullet sessions are currently running.")
        return
    stop_pb_client()
    print("RSPBStop: Disconnected all sessions.")


if __name__ == "__main__":
    main()
