#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSClearIKKeyframeAll - Erase saved dual-arm IK data on EVERY bar.

The every-bar companion of ``RSClearIKKeyframe`` (one picked bar). Both have no
toolbar button of their own any more: RSIKMaintenance > ClearAllBars runs this,
and the command can still be typed. It applies the clear to every registered
bar, then asks the same three-toggle choice of WHAT to erase (``Keyframe`` /
``BasePosition`` / ``Support``, all default Erase). Shares the scope + prompt +
clear + report logic via ``clear_ik``.
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

from rs_clear_ik_keyframe import clear_ik


def main() -> None:
    # Right-click: EVERY registered bar, then choose what to erase.
    clear_ik(all_bars=True)


if __name__ == "__main__":
    session_log.run(__file__, main)
