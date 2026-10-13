#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSExportBarAction - Export the document as a schema 2 design folder (= RSExportDesign).

The old single-bar ``BarActions/<bar>__J/__R.json`` export is gone: a schema 2
design folder holds the whole plan (every bar's actions, holds and schedule),
written through the shared core by RSExportDesign, and the planner
(``scripts/plan_design.py --bars B3``) can plan one bar of it. This button runs
that export.
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
    print("RSExportBarAction: the export writes the whole design now (RSExportDesign); "
          "plan one bar with scripts/plan_design.py --bars <id>.")
    import rs_export_all_bar_actions
    rs_export_all_bar_actions.main("RSExportBarAction")


if __name__ == "__main__":
    session_log.run(__file__, main)
