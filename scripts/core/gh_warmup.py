"""Logic behind the ``RSGHPreviewWarmup`` Grasshopper component.

A one-shot cache warmer for ``RSGHSequencePreview``: loading the three robot
models (Cindy the dual-arm, Alice and Belle the support Huskies) and baking
their display meshes is the ONLY heavy work in the whole preview -- and it is
one-time, cached for the Rhino session.  Without this component the preview
pays each load on the first frame that needs that robot (a visible stall);
with it, the user clicks one button while setting up the canvas and every
later scrub is mesh-transform only.

No PyBullet anywhere -- "warm-up" here means model loading + mesh baking,
nothing else.  The bake targets the SAME cache entries the preview uses
(same robot-cell objects, same layer keys), which is what makes it count.
Everything baked is left hidden; the preview decides visibility per frame.

Clicking the button again is harmless: the loaders and the mesh cache are
idempotent, so a second click just re-reports "already loaded".
"""

from __future__ import annotations

from core import config
from core import gh_bridge
from core import ik_viz
from core import robot_cell


def run(ghenv, run=False):
    """Warm the robot caches on the button's rising edge.

    Args:
        ghenv: the component's ``ghenv`` (or ``self`` in SDK mode).
        run (bool): Button; the warm-up fires once per press.

    Returns:
        dict: ``{"info": str}`` -- per-robot success/failure report.
    """
    # A script-mode input with no Type hint arrives as a GH wrapper object, and
    # bool(GH_Boolean(False)) is True -- unwrap to the Python value first.
    run = getattr(run, "Value", run)
    store = gh_bridge.state(ghenv)
    if not gh_bridge.rising_edge(store, "run", bool(run)):
        return {"info": store.get("last", "idle -- click the button to pre-load the robots")}

    notes = []
    with gh_bridge.rhino_doc():
        # Cindy: the dual-arm assembly cell + its meshes, baked into the same
        # "Assembly" bundle the preview renders on.
        try:
            rcell = robot_cell.get_or_load_robot_cell()
            # Register the arm tools BEFORE the bake: the mesh cache is built
            # once per cell, so a tool-less first bake would stay tool-less.
            try:
                robot_cell.ensure_arm_tool_models(rcell)
            except RuntimeError as exc:
                notes.append(f"assembly tools unavailable ({exc})")
            if getattr(rcell, "tool_models", None):
                state = robot_cell.base_assembly_cell_state()
            else:
                state = robot_cell.default_cell_state()
            ik_viz.update_state(
                state, robot_cell=rcell,
                mesh_modes=(ik_viz.MESH_MODE_VISUAL,),
                layer_key=ik_viz.LAYER_KEY_ASSEMBLY,
            )
            ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
            notes.append("Cindy (assembly): loaded + tools + meshes baked")
        except Exception as exc:  # a broken cell must not kill the other two
            notes.append(f"Cindy (assembly): FAILED ({exc})")

        # Alice + Belle: each support cell + its meshes, on the same
        # "Support <name>" layers the preview uses.
        from core import robot_cell_support  # lazy: pulls the support URDF stack
        for name in getattr(config, "SUPPORT_ROBOT_NAMES", ("Alice", "Belle")):
            try:
                cell = robot_cell_support.get_or_load_support_cell(name)
                state = robot_cell_support.default_support_cell_state(name)
                key = f"Support {name}"
                ik_viz.update_state(
                    state, robot_cell=cell,
                    mesh_modes=(ik_viz.MESH_MODE_VISUAL,),
                    layer_key=key,
                )
                ik_viz.set_layer_visible(key, False)
                notes.append(f"{name} (support): loaded + meshes baked")
            except Exception as exc:
                notes.append(f"{name} (support): FAILED ({exc})")

    info = " | ".join(notes)
    store["last"] = f"last warm-up: {info}"
    return {"info": info}