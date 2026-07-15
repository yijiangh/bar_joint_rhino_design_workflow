#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSClearIKKeyframeAndBase - Remove a bar's IK config AND its mobile base.

Right-click companion to ``RSClearIKKeyframe`` (left-click = clear the arm IK
config only, keeping the mobile base position). This one also deletes the robot
base frame, so the bar reverts to fully "no IK" (base is re-sampled/re-picked on
the next solve). Shares the pick + clear + report logic with the left-click
script via ``clear_ik``.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rs_clear_ik_keyframe import clear_ik


def main() -> None:
    # Right-click: clear the arm IK config AND the mobile base position.
    clear_ik(clear_base_frame=True)


if __name__ == "__main__":
    main()
