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
"""RSGHSequencePreview -- scrub the WHOLE assembly plan from a Grasshopper slider.

One slider frame = one step of the global assembly timeline -- the same list
RSShowAssemblyPlan walks (88 steps in the current study file): per bar
approach / assembled / (hold) / retreat / home, plus the support robots'
release steps.  All robots render by forward kinematics from the keyframes
saved on the bar curves in THIS document; no PyBullet, no RSPBStart.

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
import sys

import Grasshopper  # noqa: F401
import System  # noqa: F401  (type hints are converted to .NET types)

# A GH component has no reliable ``__file__``, so the repo's ``scripts`` folder
# is named outright. Change this one line if the repo moves.
SCRIPTS = r"c:\Users\SU\Documents\GitHub\bar_joint_rhino_design_workflow\scripts"
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

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