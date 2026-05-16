#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineJointHalf - Define ONE joint half (Male / Female / Ground).

Workflow:

    Step 0  Choose kind: Male, Female, or Ground.
    Step 1  Pick the block instance.
    Step 2  Pick the bar axis line.
    Step 3  (Male/Female only) Pick the screw axis line.
    Step 4  (Male/Female only) Pick the screw center point.
    Step 5  Enter the half name.  For Male/Female the name MUST equal the
            block-definition name (so a mate can resolve the half by
            block_name).  For Ground the name is the ground-joint key.

The script:
  - Computes the constant transforms `M_block_from_bar` and (for non-ground
    halves) `M_screw_from_block`, both in millimetres.
  - Exports the block definition to `asset/<block_name>.3dm` (overwrite).
  - Exports a single-mesh collision OBJ to `asset/<block_name>.obj` in
    millimetres (overwrite).
  - Persists the half/ground entry into the normalized
    `scripts/core/joint_pairs.json` registry.

Stacked picks: each pick auto-hides the previous selection, then everything
is restored at the end (same UX as the legacy RSDefineJointPair command).
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import joint_pair as _joint_pair_module
from core import joint_pick_helpers as _picks_module
from core.rhino_block_export import export_block_definition_to_3dm
from core.rhino_block_obj_export import export_picked_meshes_to_obj_mm
from core.rhino_helpers import suspend_redraw


def _reload():
    global jp_mod, picks
    jp_mod = importlib.reload(_joint_pair_module)
    picks = importlib.reload(_picks_module)


_reload()


_DIALOG = "RSDefineJointHalf"


def _pick_kind() -> str | None:
    answer = rs.GetString(
        "Joint half kind", "Male", ["Male", "Female", "Ground"]
    )
    if answer is None:
        return None
    a = answer.strip().lower()
    if a.startswith("ma"):
        return "male"
    if a.startswith("fe"):
        return "female"
    if a.startswith("gr"):
        return "ground"
    return None


def _project_screw_origin(screw_point, screw_axis_start, screw_axis_end):
    from core.transforms import project_point_to_line

    screw_dir = screw_axis_end - screw_axis_start
    proj, _ = project_point_to_line(screw_point, screw_axis_start, screw_dir)
    return proj, screw_dir


def main() -> None:
    _reload()
    rs.UnselectAllObjects()
    scale_to_mm = picks.doc_unit_scale_to_mm()
    print(f"{_DIALOG}: scale_to_mm = {scale_to_mm:g}")

    kind = _pick_kind()
    if kind is None:
        print(f"{_DIALOG}: cancelled at kind selection.")
        return
    print(f"{_DIALOG}: kind = {kind}")

    selected: list = []

    block_id = picks.pick_block_instance(f"Pick {kind.upper()} block instance", _DIALOG)
    if block_id is None:
        print(f"{_DIALOG}: cancelled at block pick.")
        return
    selected.append(block_id)

    with picks.temporarily_hidden(selected):
        bar_id = picks.pick_line(f"Pick {kind.upper()} bar axis line", _DIALOG)
    if bar_id is None:
        print(f"{_DIALOG}: cancelled at bar axis pick.")
        return
    selected.append(bar_id)

    screw_axis_id = None
    screw_point_id = None
    if kind in ("male", "female"):
        with picks.temporarily_hidden(selected):
            screw_axis_id = picks.pick_line("Pick SCREW axis line", _DIALOG)
        if screw_axis_id is None:
            print(f"{_DIALOG}: cancelled at screw axis pick.")
            return
        selected.append(screw_axis_id)

        with picks.temporarily_hidden(selected):
            screw_point_id = picks.pick_point("Pick SCREW center point")
        if screw_point_id is None:
            print(f"{_DIALOG}: cancelled at screw center pick.")
            return
        selected.append(screw_point_id)

    # Collision meshes -- pick AFTER bar/screw geometry so we can hide
    # everything else and pick stacked meshes cleanly. The user must
    # hand-model the low-poly collision mesh; we never auto-generate it
    # from the block render geometry.
    with picks.temporarily_hidden(selected):
        mesh_ids = picks.pick_meshes(
            "Pick collision MESH object(s) (one or more) and press Enter"
        )
    if not mesh_ids:
        print(f"{_DIALOG}: cancelled at collision-mesh pick (none selected).")
        return
    selected.extend(mesh_ids)
    print(f"{_DIALOG}: collision meshes picked = {len(mesh_ids)}")

    # Resolve geometry / block name
    block_xform_doc, block_def_name = picks.block_instance_frame(block_id)
    if not block_def_name:
        rs.MessageBox(
            "Block instance must reference a named block definition.", 0, _DIALOG
        )
        return
    print(f"{_DIALOG}: block definition name = '{block_def_name}'")

    # Half name
    if kind in ("male", "female"):
        default_name = block_def_name
        prompt = f"Half name (block_name; default '{default_name}')"
    else:
        default_name = ""
        prompt = "Ground joint name (required)"
    half_name = rs.GetString(prompt, default_name)
    if half_name is None:
        print(f"{_DIALOG}: cancelled at name input.")
        return
    half_name = half_name.strip()
    if not half_name:
        rs.MessageBox("Name is required.", 0, _DIALOG)
        return

    if kind in ("male", "female") and half_name != block_def_name:
        rs.MessageBox(
            f"For male/female halves, the half name MUST equal the block "
            f"definition name. Got name='{half_name}', block='{block_def_name}'.",
            0,
            _DIALOG,
        )
        return

    bar_start_doc, bar_end_doc = picks.line_endpoints(bar_id)
    block_frame_mm = picks.frame_to_mm(block_xform_doc, scale_to_mm)
    bar_start_mm = picks.vec_to_mm(bar_start_doc, scale_to_mm)
    bar_end_mm = picks.vec_to_mm(bar_end_doc, scale_to_mm)

    M_block_from_bar = picks.compute_M_block_from_bar(
        block_frame_mm, bar_start_mm, bar_end_mm
    )
    print(
        f"{_DIALOG}: M_block_from_bar translation (mm) = "
        f"({M_block_from_bar[0,3]:.4f}, {M_block_from_bar[1,3]:.4f}, {M_block_from_bar[2,3]:.4f})"
    )

    if kind in ("male", "female"):
        screw_axis_start_doc, screw_axis_end_doc = picks.line_endpoints(screw_axis_id)
        screw_point_doc = picks.point_xyz(screw_point_id)
        screw_origin_doc, screw_dir_doc = _project_screw_origin(
            screw_point_doc, screw_axis_start_doc, screw_axis_end_doc
        )
        screw_origin_mm = picks.vec_to_mm(screw_origin_doc, scale_to_mm)
        # Direction is unitless w.r.t. doc-unit scale (only direction matters).
        M_screw_from_block = picks.compute_M_screw_from_block(
            block_frame_mm, screw_origin_mm, screw_dir_doc
        )
        screw_z_world = M_screw_from_block[:3, 2]
        screw_origin_local = M_screw_from_block[:3, 3]
        print(
            f"{_DIALOG}: M_screw_from_block translation (mm in block frame) = "
            f"({screw_origin_local[0]:.4f}, {screw_origin_local[1]:.4f}, {screw_origin_local[2]:.4f})"
        )
        print(
            f"{_DIALOG}: screw Z axis (in block frame) = "
            f"({screw_z_world[0]:.4f}, {screw_z_world[1]:.4f}, {screw_z_world[2]:.4f})"
        )

    # Asset filenames
    asset_filename = f"{block_def_name}.3dm"
    obj_filename = f"{block_def_name}.obj"
    asset_dir = jp_mod.DEFAULT_ASSET_DIR
    asset_path = os.path.join(asset_dir, asset_filename)
    obj_path = os.path.join(asset_dir, obj_filename)
    print(f"{_DIALOG}: exporting block .3dm -> {asset_path}")
    print(f"{_DIALOG}: exporting collision OBJ -> {obj_path}")

    with suspend_redraw():
        ok_3dm = export_block_definition_to_3dm(block_def_name, asset_path)
        ok_obj = export_picked_meshes_to_obj_mm(
            mesh_ids, block_xform_doc, obj_path, label=block_def_name
        )

    if not ok_3dm:
        rs.MessageBox(
            f"Failed to export block .3dm to {asset_path}. See console.", 0, _DIALOG
        )
        return
    if not ok_obj:
        print(f"{_DIALOG}: WARNING: collision OBJ export failed; persisting half without collision_filename.")
        obj_filename = ""

    # Build dataclass and save
    if kind in ("male", "female"):
        half = jp_mod.JointHalfDef(
            block_name=block_def_name,
            M_block_from_bar=M_block_from_bar,
            M_screw_from_block=M_screw_from_block,
            kind=kind,
            asset_filename=asset_filename,
            collision_filename=obj_filename,
        )
        jp_mod.save_joint_half(half)
        print(f"{_DIALOG}: saved half '{block_def_name}' (kind={kind}) to registry.")
    else:
        ground = jp_mod.GroundJointDef(
            name=half_name,
            block_name=block_def_name,
            M_block_from_bar=M_block_from_bar,
            asset_filename=asset_filename,
            collision_filename=obj_filename,
        )
        jp_mod.save_ground_joint(ground)
        print(f"{_DIALOG}: saved ground joint '{half_name}' (block={block_def_name}) to registry.")

    print(f"{_DIALOG}: done.")


if __name__ == "__main__":
    main()
