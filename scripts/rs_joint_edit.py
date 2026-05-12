#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
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

# Shared placement primitives live in core.joint_placement; this command
# only owns the click loop + per-block-flip orchestration.
import rs_joint_place as _rjp  # for _reload_runtime_modules() side-effects

from core import config
from core.ground_placement import (
    GROUND_INSTANCES_LAYER,
    fk_ground_block_frame,
    place_ground_block,
    remove_placed_ground,
)
from core.joint_pair import get_joint_pair, load_joint_registry
from core.joint_placement import (
    FEMALE_INSTANCES_LAYER,
    MALE_INSTANCES_LAYER,
    compute_variant_with_recovery,
    place_joint_blocks,
)
from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry
from core.rhino_block_import import require_block_definition
from core.rhino_helpers import curve_endpoints
from core.rhino_tool_place import (
    cycle_tool_at_tool_instance,
    get_tool_name_for_joint,
    place_tool_by_name_at_male_joint,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _placed_joint_filter(rhino_object, geometry, component_index):
    """Geometry filter -- accept any placed joint block (female/male/ground)
    OR robotic tool instance."""
    layer = rs.ObjectLayer(rhino_object.Id)
    return layer in (
        FEMALE_INSTANCES_LAYER,
        MALE_INSTANCES_LAYER,
        GROUND_INSTANCES_LAYER,
        config.LAYER_TOOL_INSTANCES,
    )


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


def _flip_ground_block(clicked_id):
    """Flip a placed ground-joint block's X axis along the bar.

    Reads ``ground_joint_name`` / ``parent_bar_id`` / ``position_mm`` /
    ``rotation_deg`` / ``flipped`` from the clicked instance, deletes it,
    and re-bakes via :func:`core.ground_placement.place_ground_block`
    with ``flipped`` toggled.  ``jr`` is preserved (the flip post-multi-
    plies ``M_block_from_bar`` by ``R_y(pi)``, which keeps block-local
    +Y -- so world-up alignment is preserved and ``jr`` does not change).
    """
    import math
    import numpy as np

    joint_id = rs.GetUserText(clicked_id, "joint_id")
    ground_name = rs.GetUserText(clicked_id, "ground_joint_name")
    bar_id = rs.GetUserText(clicked_id, "parent_bar_id")
    jp_text = rs.GetUserText(clicked_id, "position_mm")
    jr_text = rs.GetUserText(clicked_id, "rotation_deg")
    flipped_text = rs.GetUserText(clicked_id, "flipped")
    if not joint_id or not ground_name or not bar_id or not jp_text or not jr_text:
        print(
            "RSJointEdit: Could not read ground-joint metadata from the selected block.\n"
            "  This block may have been placed by an older RSGroundPlace.\n"
            "  Re-place it with RSGroundPlace to enable re-editing."
        )
        return

    try:
        registry = load_joint_registry()
        ground = registry.ground_joints[ground_name]
    except KeyError:
        print(
            f"RSJointEdit: Ground joint '{ground_name}' is no longer registered."
        )
        return

    bar_curve_id = _find_bar_curve(bar_id)
    if bar_curve_id is None:
        print(f"RSJointEdit: Could not find bar curve {bar_id}.")
        return

    bar_start, bar_end = curve_endpoints(bar_curve_id)
    bar_start = np.asarray(bar_start, dtype=float)
    bar_end = np.asarray(bar_end, dtype=float)
    jp = float(jp_text)
    jr = math.radians(float(jr_text))
    # Older bakes (pre-flipped-flag) have no UserText -> default False.
    flipped = (flipped_text == "True") if flipped_text else False
    flipped = not flipped

    remove_placed_ground(joint_id)
    ground_oid, _ = place_ground_block(
        ground=ground,
        bar_id=bar_id,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=jp,
        jr=jr,
        flipped=flipped,
        joint_id=joint_id,
    )

    # Re-place the robotic tool so it follows the flipped ground frame.
    # Preserve whichever tool the user previously had attached to this joint.
    from core.rhino_tool_place import (  # noqa: PLC0415
        get_tool_name_for_joint,
        place_tool_by_name_at_ground_block,
    )
    prev_tool = get_tool_name_for_joint(joint_id)
    place_tool_by_name_at_ground_block(ground_oid, joint_id, prev_tool)

    print(f"RSJointEdit: ground {joint_id} flipped (flipped now {flipped}).")


def main():
    _rjp._reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSJointEdit")

    # Continuous pick loop — no accept/confirm step.  Each click on a joint
    # block immediately flips that block's side and re-places the pair.
    # Press Escape to exit the command.
    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(
            "Click a joint block to flip it, or a tool to cycle it  (Escape to exit)"
        )
        go.EnablePreSelect(False, False)
        go.SetCustomGeometryFilter(_placed_joint_filter)
        result = go.Get()
        if result != Rhino.Input.GetResult.Object:
            print("RSJointEdit: Done.")
            return

        clicked_id = go.Object(0).ObjectId
        clicked_layer = rs.ObjectLayer(clicked_id)

        # Tool instance: cycle to next tool in registry, no joint changes.
        if clicked_layer == config.LAYER_TOOL_INSTANCES:
            cycle_tool_at_tool_instance(clicked_id)
            continue

        # Ground-joint instance: flip jr by 180 deg and re-bake at the
        # same (jp) along the same bar.  No mate-side recovery needed.
        if clicked_layer == GROUND_INSTANCES_LAYER:
            _flip_ground_block(clicked_id)
            continue

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
            require_block_definition(
                pair.female.block_name, asset_path=pair.female.asset_path()
            )
            require_block_definition(
                pair.male.block_name, asset_path=pair.male.asset_path()
            )
        except RuntimeError as exc:
            print(f"RSJointEdit: {exc}")
            continue

        # Toggle the side that was clicked, and remember which side it was
        # so the auto-recovery (when the chosen orientation lands on a bad
        # local minimum) can flip the OTHER side.
        clicked_side = None
        if clicked_layer == FEMALE_INSTANCES_LAYER:
            le_rev = not le_rev
            clicked_side = "female"
        elif clicked_layer == MALE_INSTANCES_LAYER:
            ln_rev = not ln_rev
            clicked_side = "male"

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
        # If recovery is needed (bad local minimum on this orientation),
        # flip the OPPOSITE side from whichever the user just clicked.
        recover_side = "male" if clicked_side == "female" else "female"
        le_start, le_end = curve_endpoints(le_id)
        ln_start, ln_end = curve_endpoints(ln_id)
        new_variant, _recovered, le_rev, ln_rev = compute_variant_with_recovery(
            le_start, le_end, ln_start, ln_end, le_rev, ln_rev,
            pair=pair, recover_side=recover_side,
            log_prefix="RSJointEdit",
        )
        _remove_placed_joint(joint_id)
        # Preserve the previously-chosen tool for this joint, so flipping
        # the male/female block doesn't reset the tool back to the doc default.
        prev_tool_name = get_tool_name_for_joint(joint_id)
        _, male_id, new_joint_id = place_joint_blocks(
            new_variant, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
        )
        place_tool_by_name_at_male_joint(male_id, new_joint_id, pair, prev_tool_name)
        print(f"RSJointEdit: {joint_id} flipped (pair '{pair.name}').")


if __name__ == "__main__":
    main()
