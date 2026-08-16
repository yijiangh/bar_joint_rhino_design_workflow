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
"""RSGHSequencePreview -- scrub the assembly sequence from a Grasshopper slider.

HOW TO USE THIS FILE
====================
1. Drop a **Python 3 Script** component on the canvas and open the Script Editor.
2. Click **Convert To GH_ScriptInstance** on the editor dashboard (this is what
   "SDK mode" means -- see the notes below).
3. Select all (Ctrl+A), delete, **then** paste this file into the now-empty
   editor, and close it.  The component's inputs and outputs are generated from
   the ``RunScript`` signature: 12 inputs, 5 outputs, no clicking ``+`` and no
   typing nicknames.

   Delete first, paste second.  Pasting *over* a selection lets the editor
   re-indent the incoming block against whatever the cursor was sitting in, and
   you get ``unindent does not match any outer indentation level``.  The code
   below is also written with no bracket continuations at all -- hence the long
   lines -- because those are what a re-indent mangles.
4. Wire a **Button** to ``reload`` and click it once.  A slider appears wired to
   ``step``, its range already the frame count of this document.
5. Set ``enable`` to True and drag the slider.

Only step 3 needs repeating when the logic changes -- and usually not even
that: the real code lives in ``scripts/core/gh_*.py`` and is re-imported on
every solve by the ``importlib.reload`` calls below, so editing those files and
nudging the component picks up the change.

WHAT SDK MODE CHANGES (worth knowing)
=====================================
* **Parameters follow the signature, both ways.**  Edit ``RunScript``'s
  arguments and the component rebuilds its params; add a param on the canvas and
  the editor rewrites the signature.  The argument *name* becomes the param
  NickName -- so ``ensure_int_slider(ghenv, "step", ...)`` finds the ``step``
  input purely because the argument is called ``step``.  Renaming an argument
  silently renames a param.
* **The annotation is the type hint.**  ``enable: bool`` gives the param a
  boolean type hint; ``poses: List[str]`` additionally sets **List Access**,
  which is otherwise a right-click chore.
* **Outputs come from the return tuple**, in order.  The names are read from the
  signature's return annotation / the editor's output params -- keep the
  ``return`` order matching the documented output list below.
* **``ghenv`` still exists.**  SDK mode wraps your code in a class, but the host
  still injects ``ghenv`` into the module scope, so ``ghenv.Component`` works
  the same as in script mode (as it does in
  ``support_materials/gh_keyframe_demos/python/GH_init_pb.py``).  ``self`` also
  carries ``.Component``; ``core.gh_bridge.component_of`` accepts either.
* **The class is instantiated fresh per solve.**  Do not keep state on ``self``
  -- it will not survive.  Anything that must persist goes in ``sc.sticky``,
  which is what ``gh_bridge.state()`` manages (keyed per component, so two
  copies of this file do not corrupt each other).
* **A Button is a source component, not a param setting.**  SDK mode cannot
  generate it; wire one to ``reload`` yourself.

INPUTS (generated from the signature)
    enable                bool   master switch; False restores the document
    reload                bool   Button -- rebuilds the `step` slider
    step                  int    frame index (auto-created slider)
    poses                 List[str]  M1/M2/M3/M4, in play order; default M3
    show_unbuilt          bool   show bars later than the active step
    show_bars_and_joints  bool   document bar/joint layer visibility
    show_assemble_robot   bool   draw the dual-arm robot
    show_support_robot    bool   accepted but inert (pipeline not wired)
    preview_color_unbuilt bool   grey tint on / off
    preview_color_built   bool   green tint on / off
    preview_color_current bool   blue tint on / off
    preview_color_support bool   purple tint on / off

OUTPUTS
    bar_id, seq, pose, frame_count, info

NOTES
    * No ``RSPBStart`` required -- the robot is drawn by FK only.
    * Every registered bar is always in the frame list; the persistent
      ``HideUnbuilt`` latch is neither read nor written.
    * ``enable=False`` restores colours, layers and the IK cache layer.
"""

import importlib
import sys
from typing import List  # noqa: F401  (the editor may rewrite it away -- see below)

import Grasshopper  # noqa: F401  (base class for SDK mode)
import System  # noqa: F401  (REQUIRED -- see the signature note below)

# A GH component has no reliable ``__file__``, so the repo's ``scripts`` folder
# is named outright. Change this one line if the repo moves.
SCRIPTS = r"c:\Users\SU\Documents\GitHub\bar_joint_rhino_design_workflow\scripts"
if SCRIPTS not in sys.path: sys.path.insert(0, SCRIPTS)

from core import gh_bridge as _gh_bridge_module
from core import gh_seq_preview as _gh_seq_preview_module


# THE EDITOR OWNS THE RunScript SIGNATURE -- read this before editing it.
#
# In SDK mode the Script Editor keeps the component's parameters and the
# RunScript signature in sync, and it does that by REWRITING the signature in
# place. Two consequences, both of which have already bitten:
#
#   1. It converts Python type hints to .NET ones, so `poses: List[str]` comes
#      back as `poses: System.Collections.Generic.List[object]`. That is why
#      `import System` above is mandatory -- without it the component fails with
#      "undefined name 'System'". The rewrite is fine; `normalize_poses` unwraps
#      GH_String items via `.Value`.
#   2. The rewrite can dedent `def RunScript` out of the class, which surfaces as
#      "unindent does not match any outer indentation level" pointing at the def.
#      If that happens: re-indent the def to 4 spaces under the class line and
#      leave the signature text exactly as the editor wrote it. Do NOT delete the
#      `class` line -- SDK mode needs it, and a module-level `def RunScript(self,
#      ...)` is not a valid component.
#
# The lines below are also deliberately long with no bracket continuations: one
# statement per line gives the rewriter and the paste re-indenter less to mangle.


class RSGHSequencePreview(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, enable: bool, reload: bool, step: int, poses: List[str], show_unbuilt: bool, show_bars_and_joints: bool, show_assemble_robot: bool, show_support_robot: bool, preview_color_unbuilt: bool, preview_color_built: bool, preview_color_current: bool, preview_color_support: bool):
        # Reloaded every solve so edits to scripts/core/gh_*.py take effect
        # without re-pasting this file.
        importlib.reload(_gh_bridge_module)
        module = importlib.reload(_gh_seq_preview_module)
        colors = {"preview_color_unbuilt": preview_color_unbuilt, "preview_color_built": preview_color_built, "preview_color_current": preview_color_current, "preview_color_support": preview_color_support}
        shows = {"show_unbuilt": show_unbuilt, "show_bars_and_joints": show_bars_and_joints, "show_assemble_robot": show_assemble_robot, "show_support_robot": show_support_robot}
        # `ghenv` is injected by the Grasshopper host -- it has no import line.
        result = module.run(ghenv, enable=enable, reload=reload, step=step, poses=poses, **shows, **colors)
        return result["bar_id"], result["seq"], result["pose"], result["frame_count"], result["info"]
