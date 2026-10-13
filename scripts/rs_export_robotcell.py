#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
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

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402


def main() -> None:
    print("RSExportRobotCell: there is no RobotCell.json any more; writing the design folder (RSExportDesign).")
    import rs_export_all_bar_actions
    rs_export_all_bar_actions.main("RSExportRobotCell")


if __name__ == "__main__":
    session_log.run(__file__, main)
