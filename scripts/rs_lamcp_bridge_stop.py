#! python 3
# venv: scaffolding_env
# ! Keep the venv line identical to rs_lamcp_bridge_start.py: stop must run in
# ! the SAME interpreter as start, otherwise it cannot see the server handle
# ! stored in `sc.sticky`.
"""RSLamcpBridgeStop - Right-click entry for RSLamcpBridgeStart.

Thin wrapper around `rs_lamcp_bridge_start.stop`: shuts down the LAMCP bridge
server and frees port 8765, so Claude Code can no longer reach this Rhino
session until the bridge is started again.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rs_lamcp_bridge_start import stop


if __name__ == "__main__":
    stop()
