#! python 3
# venv: scaffolding_env
"""RSShowIKPreview - Color every bar by its IK status (right-click of RSUpdatePreview).

Right-click companion of RSUpdatePreview (left-click clears the IK color preview).
This command paints every bar that already carries a solved IK keyframe in
``COLOR_HAS_IK`` and prints a legend explaining the colors to the command line.

Only the persisted "has IK" state can be shown -- the transient "failed" color is
never stored on the bar (see the "IK color preview" section in
``core.rhino_bar_registry``). All color logic is shared with RSIKKeyframeAll and
RSUpdatePreview via that module (single source of truth).
"""

import importlib
import os
import sys

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import rhino_bar_registry as _rhino_bar_registry_module


def main():
    importlib.reload(config)
    reg = importlib.reload(_rhino_bar_registry_module)

    reg.repair_on_entry(float(config.BAR_RADIUS), caller="RSShowIKPreview")

    n_has_ik, n_total = reg.show_all_ik_preview()
    reg.print_ik_preview_legend()
    print(f"RSShowIKPreview: {n_has_ik}/{n_total} bar(s) have a solved IK keyframe.")


if __name__ == "__main__":
    main()
