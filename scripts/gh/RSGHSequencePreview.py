#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSGHSequencePreview -- scrub the WHOLE assembly plan from a Grasshopper slider.

One slider frame = one step of the global assembly timeline -- the same list
RSShowAssemblyPlan walks (88 steps in the current study file): per bar
approach / assembled / (hold) / retreat / home, plus the support robots'
release steps.  All robots render by forward kinematics from the keyframes
saved on the bar curves in THIS document; no PyBullet.

Setup and troubleshooting live in docs/grasshopper_animation.md -- this
docstring is deliberately short: the paste-time parameter sync reads the whole
pasted text and long prose here has broken it before.

INPUTS
    enable          bool   master switch; False restores the document
    reload          bool   Button -- rebuilds the timeline + `step` slider
    step            int    frame index (auto-created slider; right-click to Animate)
    show_unbuilt    bool   gray guide lines on not-yet-built bars
    show_current    bool   blue guide line on the active bar
    show_support    bool   purple guide lines on the supporting bars
    line_thickness  float  guide-line width in screen PIXELS (e.g. 1..6)
    line_style      str    "continuous" or "dashed"
    dash_pattern    str    dash/gap in mm along the bar, e.g. "40,20"

OUTPUTS
    bar_id, seq, pose, frame_count, step_label, info

NOTES
    * The guide lines are a screen OVERLAY drawn on top of the geometry in
      exact colours -- they work in any display mode and never touch
      PrintDisplay or the display settings.
    * ``enable=False`` restores everything: visibility, the overlay, and the
      two touched layers (robotic tools + bar centerlines, both hidden while
      enabled).  Other layers are never touched, so manual toggles survive.
    * The assembly tools are auto-registered from robotic_tools.json; if an
      OBJ export is missing the arms render flange-only and ``info`` says so.
    * Read ``info`` whenever something looks wrong -- skipped bars, missing
      keyframes and undrawable robots are all reported there.
"""

import importlib
import os
import sys

import Grasshopper  # noqa: F401
import Rhino
import System  # noqa: F401  (type hints are converted to .NET types)

# A GH component has no reliable ``__file__``: the repo's ``scripts`` folder is the
# Rhino Search Path that holds rhino_env.py, the one the toolbar commands use (README).
SCRIPTS = next((p for p in Rhino.ApplicationSettings.FileSettings.GetSearchPaths() if os.path.isfile(os.path.join(p, "rhino_env.py"))), None)
if SCRIPTS is None: raise ImportError("Add <repo>/scripts to Tools > Options > Files > Search Paths (README).")
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

from core import gh_bridge as _gh_bridge_module
from core import gh_seq_preview as _gh_seq_preview_module


class RSGHSequencePreview(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, enable: bool, reload: bool, step: int, show_unbuilt: bool, show_current: bool, show_support: bool, line_thickness: float, line_style: str, dash_pattern: str):
        # Reloaded every solve so edits to scripts/core/gh_seq_preview.py take
        # effect without re-pasting this file.
        importlib.reload(_gh_bridge_module)
        module = importlib.reload(_gh_seq_preview_module)
        # `ghenv` is injected by the Grasshopper host -- it has no import line.
        result = module.run(ghenv, enable=enable, reload=reload, step=step, show_unbuilt=show_unbuilt, show_current=show_current, show_support=show_support, line_thickness=line_thickness, line_style=line_style, dash_pattern=dash_pattern)
        return result["bar_id"], result["seq"], result["pose"], result["frame_count"], result["step_label"], result["info"]