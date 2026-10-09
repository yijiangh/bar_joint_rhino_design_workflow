#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: trimesh==4.12.2
"""RSExportRobotCell - Export the document as a schema 2 design folder (= RSExportDesign).

There is no ``RobotCell.json`` any more: the design folder written through the
shared core holds the robots (URDF/SRDF under ``robots/``), their tools and
every body, and each consumer builds its compas_fab cell from it with the core
(``bar_assembly_core.mirrors``). This button runs that export.
"""

from __future__ import annotations

import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def main() -> None:
    print("RSExportRobotCell: there is no RobotCell.json any more; writing the design folder (RSExportDesign).")
    import rs_export_all_bar_actions
    rs_export_all_bar_actions.main("RSExportRobotCell")


if __name__ == "__main__":
    main()
