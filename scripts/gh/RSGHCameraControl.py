#! python 3
# venv: scaffolding_env
"""RSGHCameraControl -- drive the Rhino viewport camera from Grasshopper.

No numpy / compas / pybullet here: this component only touches the viewport,
so it carries no ``# r:`` requirements and loads instantly.

Setup and troubleshooting live in docs/grasshopper_animation.md -- this
docstring is deliberately short: the paste-time parameter sync reads the whole
pasted text and long prose here has broken it before.

INPUTS
    camera    Point3d   camera position, document units
    target    Point3d   what the camera looks at, document units
    lens      float     35mm-equivalent focal length (50 = Rhino default)
    up        Vector3d  camera up; empty = world Z
    active    bool      False = no-op, the viewport is left alone
    viewport  str       named viewport; empty = the active view

OUTPUTS
    ok, info

NOTES
    * Feed ``camera`` from ``Curve | Evaluate`` driven by the same slider that
      drives RSGHSequencePreview.step (normalised: ``step / frame_count``) and
      the camera flies along the curve as the structure assembles.
    * Document units, passed straight through.  The repo's millimetre
      convention covers robot/IK data, not viewport geometry.
"""

import importlib
import sys

import Grasshopper  # noqa: F401
import Rhino
import System  # noqa: F401  (type hints are converted to .NET types)

# A GH component has no reliable ``__file__``, so the repo's ``scripts`` folder
# is named outright. Change this one line if the repo moves.
SCRIPTS = r"c:\Users\SU\Documents\GitHub\bar_joint_rhino_design_workflow\scripts"
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

from core import gh_bridge as _gh_bridge_module
from core import gh_camera as _gh_camera_module


class RSGHCameraControl(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, camera: Rhino.Geometry.Point3d, target: Rhino.Geometry.Point3d, lens: float, up: Rhino.Geometry.Vector3d, active: bool, viewport: str):
        importlib.reload(_gh_bridge_module)
        module = importlib.reload(_gh_camera_module)
        result = module.run(camera=camera, target=target, lens=lens if lens else 50.0, up=up, active=active, viewport=viewport)
        return result["ok"], result["info"]
