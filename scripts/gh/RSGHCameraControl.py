#! python 3
# venv: scaffolding_env
"""RSGHCameraControl -- drive the Rhino viewport camera from Grasshopper.

No numpy / compas / pybullet here: this component only touches the viewport, so
it carries no ``# r:`` requirements and loads instantly.

HOW TO USE THIS FILE
====================
1. Drop a **Python 3 Script** component on the canvas and open the Script Editor.
2. Click **Convert To GH_ScriptInstance** (SDK mode).
3. Select all (Ctrl+A), **delete**, then paste this file into the empty editor
   and close it -- the six inputs and two outputs are generated from the
   ``RunScript`` signature.  Pasting over a selection lets the editor re-indent
   the block and you get ``unindent does not match any outer indentation
   level``; that is also why the code below has no bracket continuations.
4. Feed ``camera`` and ``target`` points, set ``active`` True.

See ``RSGHSequencePreview.py``'s header for the full SDK-mode notes; the two
that matter here:

* the argument *name* becomes the param NickName, and the *annotation* becomes
  the type hint -- ``camera: Rhino.Geometry.Point3d`` gives you a proper Point
  param rather than a generic one, so a GH point plugs straight in;
* the class is re-instantiated every solve, so nothing may be cached on ``self``.

INPUTS (generated from the signature)
    camera    Point3d   camera position, document units
    target    Point3d   what it looks at, document units
    lens      float     35mm-equivalent focal length (50 = Rhino default)
    up        Vector3d  camera up; empty = world Z
    active    bool      False = no-op, the viewport is left alone
    viewport  str       named viewport; empty = the active view

OUTPUTS
    ok, info

ANIMATING
    Feed ``camera`` from ``Curve | Evaluate`` driven by the same slider that
    drives ``RSGHSequencePreview.step`` (normalise it: ``step / frame_count``)
    and the camera flies along the curve as the structure assembles.

UNITS
    Document units, passed straight through.  The repo's millimetre convention
    covers robot/IK data, not viewport geometry.
"""

import importlib
import sys

import Grasshopper  # noqa: F401  (base class for SDK mode)
import Rhino
import System  # noqa: F401  (the editor rewrites type hints to .NET types)

# A GH component has no reliable ``__file__``, so the repo's ``scripts`` folder
# is named outright. Change this one line if the repo moves.
SCRIPTS = r"c:\Users\SU\Documents\GitHub\bar_joint_rhino_design_workflow\scripts"
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

from core import gh_bridge as _gh_bridge_module
from core import gh_camera as _gh_camera_module


# THE EDITOR OWNS THE RunScript SIGNATURE. In SDK mode the Script Editor keeps
# the component's params and this signature in sync by rewriting it in place --
# converting type hints to .NET ones (hence `import System` above), and
# sometimes dedenting the def out of the class, which shows up as "unindent does
# not match any outer indentation level". If that happens, re-indent the def to
# 4 spaces under the class and leave the signature text as the editor wrote it.
# Never delete the `class` line. See RSGHSequencePreview.py for the long version.


class RSGHCameraControl(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, camera: Rhino.Geometry.Point3d, target: Rhino.Geometry.Point3d, lens: float, up: Rhino.Geometry.Vector3d, active: bool, viewport: str):
        importlib.reload(_gh_bridge_module)
        module = importlib.reload(_gh_camera_module)
        result = module.run(camera=camera, target=target, lens=lens if lens else 50.0, up=up, active=active, viewport=viewport)
        return result["ok"], result["info"]
