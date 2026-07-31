#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSClearColorPreview - remove every diagnostic overlay (right-click of RSUpdatePreview).

Puts the document back to its plain appearance:

* bar colors (the IK-status overlay and the bare-bar color) revert to by-layer,
  centre-lines and their tube previews alike;
* joint and tool instances revert to by-layer;
* the orange marker dots dropped on broken links are deleted;
* nothing is left selected.

Paint them again with the ``ShowColorsPreview`` option on left-click
(RSUpdatePreview). Nothing here touches geometry or metadata -- only appearance.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import rhino_bar_registry as _rhino_bar_registry_module
from core import rhino_joint_refresh as _rhino_joint_refresh_module


def main():
    importlib.reload(config)
    reg = importlib.reload(_rhino_bar_registry_module)
    joint_refresh = importlib.reload(_rhino_joint_refresh_module)

    # clear_broken_link_marks already reverts bars, joints, tools and dots; the
    # bar-registry call is kept because it owns the IK overlay's bookkeeping and
    # reports how many bars it touched.
    n_bars = reg.clear_ik_preview()
    n_marks = joint_refresh.clear_broken_link_marks()

    print(
        f"RSClearColorPreview: reverted {n_bars} bar(s) and cleared {n_marks} "
        "joint/tool/marker object(s)."
    )
    rs.Redraw()


if __name__ == "__main__":
    main()
