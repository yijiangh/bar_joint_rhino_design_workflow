#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via the
# sys.path injection in `core.robot_cell`. Do NOT list it under `# r:` (the pip
# cache would ignore submodule SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSGHPreviewWarmup -- one-shot robot pre-loader for RSGHSequencePreview.

Click the button ONCE per Rhino session: it loads the three robot models
(Cindy, Alice, Belle) and bakes their display meshes into the session cache --
the only heavy work in the preview pipeline.  Clicking again is harmless;
skipping it just means the preview pays each load on first need.  No PyBullet.

Setup and troubleshooting live in docs/grasshopper_animation.md.

INPUTS
    run   bool   Button -- fires the warm-up once per press

OUTPUTS
    info
"""

import importlib
import sys

import Grasshopper  # noqa: F401
import System  # noqa: F401  (type hints are converted to .NET types)

# A GH component has no reliable ``__file__``, so the repo's ``scripts`` folder
# is named outright. Change this one line if the repo moves.
SCRIPTS = r"c:\Users\SU\Documents\GitHub\bar_joint_rhino_design_workflow\scripts"
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

from core import gh_bridge as _gh_bridge_module
from core import gh_warmup as _gh_warmup_module


class RSGHPreviewWarmup(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, run: bool):
        importlib.reload(_gh_bridge_module)
        module = importlib.reload(_gh_warmup_module)
        result = module.run(ghenv, run=run)
        return result["info"]