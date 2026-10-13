#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

import rs_rebuild_robot_cell


def main(use_gui: bool = False) -> None:
    rs_rebuild_robot_cell.refresh("RSPBStartGUI" if use_gui else "RSPBStart", gui=use_gui)


if __name__ == "__main__":
    session_log.run(__file__, main, use_gui=False)
