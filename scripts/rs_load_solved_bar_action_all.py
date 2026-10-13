#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

from rs_load_solved_bar_action import load_and_view


def main() -> None:
    # Right-click: all bars in the folder.
    load_and_view(batch=True)


if __name__ == "__main__":
    session_log.run(__file__, main)
