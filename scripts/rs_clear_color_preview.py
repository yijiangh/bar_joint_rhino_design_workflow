#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSClearColorPreview - remove every diagnostic overlay (right-click of RSUpdatePreview).

Puts the document back to its plain appearance:

* bar colors (the IK-status overlay and the bare-bar color) revert to by-layer,
  centre-lines and their tube previews alike;
* joint and tool instances revert to by-layer;
* the orange marker dots dropped on broken links are deleted;
* nothing is left selected.

**Fake bars keep their pink tint.**  Which bars are staging that will not be
fabricated is a property of the model, not a diagnostic overlay, so clearing the
overlay must not clear it -- otherwise the one marker you want on screen
permanently is the one that vanishes every time you tidy up.

Paint the overlay again by left-clicking RSUpdatePreview, which now always ends
by painting it. Nothing here touches geometry or metadata -- only appearance.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

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
    session_log.run(__file__, main)
