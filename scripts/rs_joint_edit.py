#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSJointEdit - Re-edit the orientation of a previously placed joint pair.

Click any placed female or male joint block instance on the canvas.  The
script reads the stored joint metadata, removes the existing blocks, and
re-enters the same interactive click-to-toggle session used by RSJointPlace
so you can flip the female or male orientation without re-picking the bars.

  - **Click the female block** to toggle female orientation (le_rev).
  - **Click the male block** to toggle male orientation (ln_rev).
  - Press **Enter** or click **Accept** to confirm and write the new blocks.
  - Press **Escape** to cancel (the original placement is restored).

Requires that the joint was placed by a recent version of RSJointPlace that
stores ``le_rev``, ``ln_rev``, ``variant_index``, ``female_parent_bar``, and
``male_parent_bar`` as user-text keys on the block instances.
"""

import os
import sys

import Rhino
import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Import rs_joint_place as a module to reuse the solver, _JointSession, and
# _interactive_click_loop without duplicating code.  The module-level
# _reload_runtime_modules() call runs on first import, which is intentional.
import rs_joint_place as _rjp

from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry
from core.rhino_helpers import curve_endpoints


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _placed_joint_filter(rhino_object, geometry, component_index):
    """Geometry filter — accept any placed female or male joint block instance."""
    layer = rs.ObjectLayer(rhino_object.Id)
    return layer in (_rjp._FEMALE_INSTANCES_LAYER, _rjp._MALE_INSTANCES_LAYER)


def _find_bar_curve(bar_id):
    """Return the Rhino object ID of the bar curve with the given bar_id, or None."""
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_ID_KEY) == bar_id:
            return oid
    return None


def _remove_placed_joint(joint_id):
    """Delete all placed female and male block instances for *joint_id*.

    Looks up blocks by their object name (``{joint_id}_female`` /
    ``{joint_id}_male``) which is set by :func:`_place_joint_blocks`.
    """
    to_delete = []
    for suffix in ("_female", "_male"):
        ids = rs.ObjectsByName(f"{joint_id}{suffix}")
        if ids:
            to_delete.extend(ids)
    if to_delete:
        rs.DeleteObjects(to_delete)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _rjp._reload_runtime_modules()
    repair_on_entry(float(_rjp.config.BAR_RADIUS), "RSJointEdit")

    # --- Step 1: let user click any placed joint block ----------------------
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Click any placed joint block to re-edit its orientation")
    go.EnablePreSelect(True, True)
    go.SetCustomGeometryFilter(_placed_joint_filter)
    result = go.Get()
    if result != Rhino.Input.GetResult.Object:
        print("RSJointEdit: No joint block selected.")
        return

    clicked_id = go.Object(0).ObjectId
    # Read the layer now, before _remove_placed_joint deletes the object.
    clicked_layer = rs.ObjectLayer(clicked_id)

    # --- Step 2: read stored metadata --------------------------------------
    joint_id = rs.GetUserText(clicked_id, "joint_id")
    le_bar_id = rs.GetUserText(clicked_id, "female_parent_bar")
    ln_bar_id = rs.GetUserText(clicked_id, "male_parent_bar")
    le_rev_str = rs.GetUserText(clicked_id, "le_rev")
    ln_rev_str = rs.GetUserText(clicked_id, "ln_rev")

    if not joint_id or not le_bar_id or not ln_bar_id:
        print(
            "RSJointEdit: Could not read joint metadata from the selected block.\n"
            "  This joint may have been placed by an older version of RSJointPlace.\n"
            "  Re-place it with RSJointPlace to enable re-editing."
        )
        return

    # le_rev / ln_rev default to False if missing (older placements without
    # this key stored will start from variant 0, i.e. both forward).
    le_rev = le_rev_str == "True"
    ln_rev = ln_rev_str == "True"

    # --- Step 3: find the bar curves in the document -----------------------
    le_id = _find_bar_curve(le_bar_id)
    ln_id = _find_bar_curve(ln_bar_id)
    if le_id is None or ln_id is None:
        missing = [b for b, i in [(le_bar_id, le_id), (ln_bar_id, ln_id)] if i is None]
        print(f"RSJointEdit: Could not find bar curve(s): {', '.join(missing)}.")
        return

    # --- Step 4: validate block definitions --------------------------------
    try:
        female_block_name = _rjp._require_block_definition(_rjp._FEMALE_BLOCK_NAME)
        male_block_name = _rjp._require_block_definition(_rjp._MALE_BLOCK_NAME)
    except RuntimeError as exc:
        print(f"RSJointEdit: {exc}")
        return

    # --- Step 5: get bar geometry (variants computed on demand in the session) ----
    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)

    # --- Step 6: remove placed blocks and enter interactive session --------
    _remove_placed_joint(joint_id)

    # The opening click acts as the first toggle, so pre-flip the appropriate side.
    init_le_rev = (
        not le_rev if clicked_layer == _rjp._FEMALE_INSTANCES_LAYER else le_rev
    )
    init_ln_rev = not ln_rev if clicked_layer == _rjp._MALE_INSTANCES_LAYER else ln_rev

    session = _rjp._JointSession(
        le_start,
        le_end,
        ln_start,
        ln_end,
        female_block_name,
        male_block_name,
        le_rev=init_le_rev,
        ln_rev=init_ln_rev,
    )

    chosen = _rjp._interactive_click_loop(session)

    if chosen is None:
        # Cancelled — restore the original placement (one solver call).
        print("RSJointEdit: Cancelled — restoring original placement.")
        orig_variant = _rjp._compute_variant(
            le_start, le_end, ln_start, ln_end, le_rev, ln_rev
        )
        _rjp._place_joint_blocks(orig_variant, le_id, ln_id, le_bar_id, ln_bar_id)
        return

    _rjp._place_joint_blocks(chosen, le_id, ln_id, le_bar_id, ln_bar_id)
    print(f"RSJointEdit: {joint_id} updated.")


if __name__ == "__main__":
    main()
