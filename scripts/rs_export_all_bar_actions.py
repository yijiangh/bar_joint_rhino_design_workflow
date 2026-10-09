#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: trimesh==4.12.2
"""RSExportAllBarActions - Export the whole document as a schema 2 design folder (= RSExportDesign).

Right-click companion of ``RSExportBarAction``. The design folder written
through the shared core (``rs_export_design.export``: ``design.json``,
``actions/``, ``meshes/``, ``robots/``, ``source/``) holds every bar's actions,
the holds, the schedule, the robots and every body; the planner
(``scripts/plan_design.py``) and the monitor read it. It replaces the old
``BarActions/`` + ``RobotCell.json`` + ``WalkableGround.json`` +
``ActionSchedule.json`` export. No PyBullet, compas_fab or tamp needed.
"""

from __future__ import annotations

import importlib
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def main(command: str = "RSExportAllBarActions") -> None:
    import rs_export_design
    importlib.reload(rs_export_design).export(command)


if __name__ == "__main__":
    main()
