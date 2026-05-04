#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
# NOTE: do NOT add `# r: compas_fab` -- compas_fab is loaded from the in-repo
# submodule via sys.path injection inside scripts/core/robot_cell.py.
"""C1: PyBullet lifecycle for the dual-arm IK GH workflow.

Inputs (GH params):
    trigger : bool       -- rising-edge starts the PyBullet client
    use_gui : bool       -- True for PyBullet debug GUI window
    stop    : bool       -- rising-edge stops the client (optional)

Outputs:
    pb_running : bool

Idempotent: re-running with `trigger=True` while a client is already up is a
no-op. Sticky cache is shared with the rs_* Rhino scripts (same keys), so
RSPBStart from the toolbar and this component are interchangeable.
"""

import os
import sys

# --- bootstrap: put `scripts/` on sys.path so `core.robot_cell` is importable.
# Importing `core.robot_cell` in turn prepends `external/compas_fab/src` to
# sys.path before anything imports `compas_fab` (see
# `core.robot_cell._ensure_submodule_compas_fab_loaded`).
REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from core import robot_cell  # noqa: E402


pb_running = robot_cell.is_pb_running()

if 'stop' in dir() and stop and pb_running:
    robot_cell.stop_pb_client()
    pb_running = False
elif trigger and not pb_running:
    robot_cell.start_pb_client(use_gui=bool(use_gui), verbose=True)
    pb_running = True

print(f"gh_init_pb: pb_running={pb_running}")
