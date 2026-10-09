#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

from rs_pb_start import main


if __name__ == "__main__":
    main(use_gui=True)
