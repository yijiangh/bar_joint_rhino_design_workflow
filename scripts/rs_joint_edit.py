#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSJointEdit - Re-edit the orientation of previously placed joint pairs.

Click any placed female or male joint block anywhere in the document.  The
clicked block's side (female = le_rev, male = ln_rev) is toggled instantly
and the joint pair is re-placed — no confirmation step required.

  - **Click the female block** to toggle female joint orientation (le_rev).
  - **Click the male block** to toggle male joint orientation (ln_rev).
  - Click repeatedly to cycle back and forth.
  - Press **Escape** to exit the command.

Requires that the joint was placed by a recent version of RSJointPlace that
stores ``le_rev``, ``ln_rev``, ``variant_index``, ``joint_pair_name``,
``female_parent_bar``, and ``male_parent_bar`` as user-text keys on the
block instances.
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

from core.joint_pair import get_joint_pair
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

    # Continuous pick loop — no accept/confirm step.  Each click on a joint
    # block immediately flips that block's side and re-places the pair.
    # Press Escape to exit the command.
    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(
            "Click any placed joint block to flip its orientation  (Escape to exit)"
        )
        go.EnablePreSelect(False, False)
        go.SetCustomGeometryFilter(_placed_joint_filter)
        result = go.Get()
        if result != Rhino.Input.GetResult.Object:
            print("RSJointEdit: Done.")
            return

        clicked_id = go.Object(0).ObjectId
        clicked_layer = rs.ObjectLayer(clicked_id)

        # Read stored metadata before the block is deleted.
        joint_id = rs.GetUserText(clicked_id, "joint_id")
        joint_pair_name = rs.GetUserText(clicked_id, "joint_pair_name")
        le_bar_id = rs.GetUserText(clicked_id, "female_parent_bar")
        ln_bar_id = rs.GetUserText(clicked_id, "male_parent_bar")
        le_rev = rs.GetUserText(clicked_id, "le_rev") == "True"
        ln_rev = rs.GetUserText(clicked_id, "ln_rev") == "True"

        if not joint_id or not le_bar_id or not ln_bar_id or not joint_pair_name:
            print(
                "RSJointEdit: Could not read joint metadata from the selected block.\n"
                "  This joint may have been placed by an older version of RSJointPlace.\n"
                "  Re-place it with RSJointPlace to enable re-editing."
            )
            continue

        try:
            pair = get_joint_pair(joint_pair_name)
        except KeyError:
            print(
                f"RSJointEdit: Joint pair '{joint_pair_name}' is no longer registered."
            )
            continue

        # Validate block definitions for this pair.
        try:
            _rjp._require_block_definition(
                pair.female.block_name, asset_path=pair.female.asset_path()
            )
            _rjp._require_block_definition(
                pair.male.block_name, asset_path=pair.male.asset_path()
            )
        except RuntimeError as exc:
            print(f"RSJointEdit: {exc}")
            continue

        # Toggle the side that was clicked.
        if clicked_layer == _rjp._FEMALE_INSTANCES_LAYER:
            le_rev = not le_rev
        elif clicked_layer == _rjp._MALE_INSTANCES_LAYER:
            ln_rev = not ln_rev

        # Find the underlying bar curves.
        le_id = _find_bar_curve(le_bar_id)
        ln_id = _find_bar_curve(ln_bar_id)
        if le_id is None or ln_id is None:
            missing = [
                b for b, i in [(le_bar_id, le_id), (ln_bar_id, ln_id)] if i is None
            ]
            print(f"RSJointEdit: Could not find bar curve(s): {', '.join(missing)}.")
            continue

        # Compute only the one variant we need, then swap the blocks.
        le_start, le_end = curve_endpoints(le_id)
        ln_start, ln_end = curve_endpoints(ln_id)
        new_variant = _rjp._compute_variant(
            le_start, le_end, ln_start, ln_end, le_rev, ln_rev, pair=pair
        )
        _remove_placed_joint(joint_id)
        _rjp._place_joint_blocks(
            new_variant, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
        )
        print(f"RSJointEdit: {joint_id} flipped (pair '{pair.name}').")


if __name__ == "__main__":
    main()
