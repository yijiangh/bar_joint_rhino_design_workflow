"""Logic behind the ``RSGHPreviewWarmup`` Grasshopper component.

A one-shot cache warmer for ``RSGHSequencePreview``: reading the document into a
process, reading the robot models (Cindy the dual-arm, Alice and Belle the
support Huskies) and their meshes, and baking them into the document is
the ONLY heavy work in the whole preview -- and it is one-time, cached for the
Rhino session.  Without this component the preview pays it on its first frame
(a visible stall); with it, the user clicks one button while setting up the
canvas and every later scrub is mesh-transform only.

No PyBullet anywhere -- "warm-up" here means model loading + mesh baking,
nothing else.  The bake targets the SAME cache entries the preview uses (every
robot placed, on the ``Assembly`` layer), which is what makes it count.
Everything baked is left hidden; the preview decides visibility per frame.

Clicking the button again is harmless: the baked parts are kept while the
robots' and tools' geometry is unchanged, so a second click only re-poses.
"""

from __future__ import annotations

import scriptcontext as sc
import session_log

from core import config
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import process_display
from core import process_source
from core import gh_bridge
from core import ik_viz


def run(ghenv, run=False):
    """Read the robots and bake their meshes on the button's rising edge.

    Args:
        ghenv: the component's ``ghenv`` (or ``self`` in SDK mode).
        run (bool): Button; the warm-up fires once per press.

    Returns:
        dict: ``{"info": str}`` -- what was loaded, or why it failed.
    """
    # A script-mode input with no Type hint arrives as a GH wrapper object, and
    # bool(GH_Boolean(False)) is True -- unwrap to the Python value first.
    run = getattr(run, "Value", run)
    store = gh_bridge.state(ghenv)
    if not gh_bridge.rising_edge(store, "run", bool(run)):
        return {"info": store.get("last", "idle -- click the button to pre-load the robots")}

    notes = []
    with gh_bridge.rhino_doc():
        try:
            result = process_source.RhinoProcessSource(
                "RSGHPreviewWarmup", process_source.geometry_cache(sc.sticky)
            ).build()
            process = result.process
            robot_id = process_source.robot_id(config.ASSEMBLY_ROBOT_NAME)
            movement = next((m for _action, m in process.movements()), None)
            if movement is None:
                raise RuntimeError("the process has no actions yet (no bar with a left and a right tool)")
            # Every robot placed (at its process base, else the origin) so each one's meshes are baked.
            scene = process.scene_at(movement)
            for other, robot in scene.robots.items():
                process_display.place(scene, other, robot.base, update=False)
            process_display.refresh(scene)
            ik_viz.update_state(scene, robot_id=robot_id, layer_key=ik_viz.LAYER_KEY_ASSEMBLY,
                                mesh_mode=ik_viz.MESH_MODE_VISUAL, hide_bodies=True)
            ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
            notes.append(f"{', '.join(sorted(process.robots))}: loaded + tools + meshes baked")
        except Exception as exc:  # noqa: BLE001 -- report, never raise into Grasshopper
            session_log.caught(exc)
            notes.append(f"FAILED ({exc})")

    info = " | ".join(notes)
    store["last"] = f"last warm-up: {info}"
    return {"info": info}
