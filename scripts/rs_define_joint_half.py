#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineJointHalf - Define ONE joint block (Female / Male / Ground / MoCap).

The block's NAME says what it is: every joint block is named
``<Type>_<Subtype>`` (``T20_Female``, ``T20_Male``, ``T20_Ground``,
``T20_MoCap``), see ``core.joint_name_conventions``.  There is no kind prompt.

Workflow:

    Step 1  Pick the block instance.  Its name gives the Subtype; confirm it.
    Step 2  Pick the bar axis line.  GROUND instead picks a +X point and a +Y
            point (see below), then shows a ghost preview of the tool and an
            Accept / Repick prompt.  A MoCap block whose Type already has a
            registered Female may copy that Female's two matrices instead.
    Step 3  (Female / Male / MoCap) Pick the screw axis line.
    Step 4  (Female / Male / MoCap) Pick the screw center point.
    Step 5  Pick the collision mesh(es).
    Step 6  (MoCap only) Pick the marker sphere objects and type each one's
            Motive label (default M1, M2, ...).  Their centres are stored in
            the block's own frame as ``marker_points_mm``.  Enter without a
            pick keeps the markers already recorded.

The registry entry is keyed by the block name.  Re-defining an existing half
keeps its ``bar_cradle``, preferred tool and marker points.

The script:
  - Computes the constant transforms `M_block_from_bar` and (for non-ground
    halves) `M_screw_from_block`, both in millimetres.
  - Exports the block definition to `asset/<block_name>.3dm` (overwrite).
  - Exports a single-mesh collision OBJ to `asset/<block_name>.obj` in
    millimetres (overwrite).
  - Persists the half/ground entry into the normalized
    `scripts/core/joint_pairs.json` registry.

Ground: the tool-attach frame
-----------------------------
A ground block's own orientation is not free: `core.single_sided_placement.
auto_jr_y_down` rotates it about the bar until its local +Y points at the
floor, so the foot lands on the ground.  The arm, however, may have to
approach with its TCP rolled relative to that.  The two frames therefore
cannot be the same frame, and the Ground branch picks the second one
explicitly -- the same way RSDefineRoboticTool picks a TCP frame, except the
ORIGIN is not picked: it is the ground block's own insertion point, and both
picked points are read as directions from it.

    * +X direction point -- where the tool's +X should point.  Together with
      the origin it also IS the bar axis, feeding `compute_M_block_from_bar`
      in place of the old bar-axis line pick.
    * +Y direction point -- direction only, and only to fix the roll (the
      frame is re-orthonormalized from X, so it may be picked loosely).

The rotation between the picked frame and the block frame is saved as
`GroundJointDef.M_tool_from_block`, and `core.rhino_tool_place.
tool_attach_frame` applies it when a tool is placed.  The ground block
itself is NEVER moved by it.

! Anchoring the bar axis at the block origin makes `M_block_from_bar` come
! out with a ZERO translation, so RSJointPlace > JointOnly lands the block origin
! exactly on the point clicked on the bar.  Ground joints defined with the
! older bar-axis-line pick carry whatever offset that line's start point had
! (T20_Ground: 25 mm), so re-defining one shifts where a given `jp` puts it.
! Its DIRECTION also sets both the tool's +X and the sense of the bar axis,
! so re-defining can flip which way new placements face along the bar --
! JointOnly's Flip covers that, and the old pick had the same
! sensitivity.  Already-baked instances never move either way.

Stacked picks: each pick auto-hides the previous selection, then everything
is restored at the end (same UX as the legacy RSDefineJointPair command).
"""

from __future__ import annotations

import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import joint_name_conventions as jnc
from core import joint_pair as _joint_pair_module
from core import joint_pick_helpers as _picks_module
from core import marker_points
from core.rhino_block_export import export_block_definition_to_3dm
from core.rhino_block_obj_export import export_picked_meshes_to_obj_mm
from core.rhino_helpers import delete_objects, doc_unit_scale_to_mm, suspend_redraw
from core.transforms import (
    frame_from_x_and_y_hint,
    invert_transform,
    make_transform,
    orthonormalize_rotation,
)


def _reload():
    global jp_mod, picks
    jp_mod = importlib.reload(_joint_pair_module)
    picks = importlib.reload(_picks_module)


_reload()


_DIALOG = "RSDefineJointHalf"


def _subtype_of_picked_block(block_name: str) -> str | None:
    """The Subtype the picked block's name declares, after a confirm prompt.

    ``None`` (with a message) when the name does not follow
    ``<Type>_<Subtype>`` or the user cancels.
    """
    try:
        type_, subtype = jnc.split_block_name(block_name)
    except ValueError:
        examples = ", ".join(jnc.block_name("T20", s) for s in jnc.SUBTYPES)
        rs.MessageBox(
            f"Block '{block_name}' is not named <Type>_<Subtype>.\n\n"
            f"The part after the last '_' must be one of {', '.join(jnc.SUBTYPES)} "
            f"-- e.g. {examples}.  Rename the block definition and run again.",
            0,
            _DIALOG,
        )
        return None
    answer = rs.GetString(
        f"Define '{block_name}' as a {subtype} block (Type {type_})",
        "Accept",
        ["Accept", "Cancel"],
    )
    if answer is None or answer.strip().lower().startswith("c"):
        return None
    return subtype


def _ask_copy_from_female(female_name: str) -> bool | None:
    """Offer to reuse the Female's matrices for a MoCap block of the same Type.

    Right only when the MoCap block was modelled on the Female's block origin
    (same part, plate added), which is why picking stays the default.
    ``None`` on cancel.
    """
    answer = rs.GetString(
        f"Bar/screw frames: pick them, or copy {female_name}'s "
        "(only if this block shares its origin)",
        "Pick",
        ["Pick", "CopyFromFemale"],
    )
    if answer is None:
        return None
    return answer.strip().lower().startswith("c")


def _pick_marker_points(block_frame_mm, scale_to_mm, selected):
    """Pick a MoCap block's marker spheres; ``{label: block-local mm}``.

    Each sphere's centre is the centre of its bounding box.  The sphere being
    labelled is selected so you can see which one the prompt means.  Returns
    ``None`` when nothing is picked (or a label prompt is cancelled), which
    keeps the markers already recorded.
    """
    with picks.temporarily_hidden(selected):
        sphere_ids = rs.GetObjects(
            "Pick the marker sphere objects (Enter to keep the recorded markers)",
            filter=rs.filter.surface | rs.filter.polysurface | rs.filter.mesh,
            preselect=False,
            select=False,
        )
    if not sphere_ids:
        return None
    points: dict = {}
    for sphere_id in sphere_ids:
        corners = rs.BoundingBox(sphere_id)
        if not corners:
            print(f"{_DIALOG}: skipped an object with no bounding box.")
            continue
        centre_doc = marker_points.bounding_box_centre([tuple(c) for c in corners])
        centre_mm = picks.vec_to_mm(centre_doc, scale_to_mm)
        default = marker_points.next_default_label(points)
        rs.SelectObject(sphere_id)
        try:
            while True:
                label = rs.GetString("Motive label for the selected sphere", default)
                if label is None:
                    return None
                label = label.strip() or default
                if label not in points:
                    break
                rs.MessageBox(f"Label '{label}' is already used.", 0, _DIALOG)
        finally:
            rs.UnselectObject(sphere_id)
        points[label] = marker_points.to_block_local_mm(block_frame_mm, centre_mm)
        print(
            f"{_DIALOG}: marker {label} at block-local "
            f"({points[label][0]:.2f}, {points[label][1]:.2f}, {points[label][2]:.2f}) mm"
        )
    selected.extend(sphere_ids)
    return points


def _project_screw_origin(screw_point, screw_axis_start, screw_axis_end):
    from core.transforms import project_point_to_line

    screw_dir = screw_axis_end - screw_axis_start
    proj, _ = project_point_to_line(screw_point, screw_axis_start, screw_dir)
    return proj, screw_dir


# ---------------------------------------------------------------------------
# Ground: tool-attach frame picks + ghost preview
# ---------------------------------------------------------------------------

def _resolve_preview_tool():
    """The tool to ghost: the doc default when it is in the active pair, else
    the active LEFT tool.  ``None`` when no active pair can be resolved (the
    preview is then skipped -- it must never block a definition)."""
    from core import rhino_tool_place as tool_place  # noqa: PLC0415

    active = tool_place._get_active_pair_or_none()
    if active is None:
        return None
    return tool_place._resolve_default_active_tool(active)


def _show_ghost_tool(tool, attach_doc, scale_to_mm):
    """Insert a preview instance of *tool* with its TCP on *attach_doc*.

    ``attach_doc`` is in DOCUMENT units while ``M_tcp_from_block`` is stored in
    millimetres, so the latter's translation is scaled back before use (the
    inverse of ``picks.frame_to_mm``).

    Deliberately NOT placed on ``LAYER_TOOL_INSTANCES`` and given no
    ``joint_id`` / ``tool_name`` user-text: a ghost that leaked onto that layer
    would be reported as an orphan by ``find_detached_tools``.  The caller
    deletes it in a ``finally``.

    No preview color is set, deliberately.  An object color on a BLOCK INSTANCE
    only reaches sub-objects whose color source is "by parent", and the robotic
    tool assets carry baked colors -- so the instance renders in its layer's
    color whatever we ask for.  (Same limitation that makes RSUpdatePreview mark
    flagged tools with a text dot instead of recoloring them.)  The ghost's job
    is to show the ORIENTATION, which reads fine either way.
    """
    from core import rhino_tool_place as tool_place  # noqa: PLC0415
    from core.joint_placement import insert_block_instance  # noqa: PLC0415

    if not tool_place._import_tool_block_definition(tool):
        return None
    m_tcp_doc = np.array(tool.M_tcp_from_block, dtype=float, copy=True)
    m_tcp_doc[:3, 3] /= float(scale_to_mm)
    frame = attach_doc @ invert_transform(m_tcp_doc)
    try:
        return insert_block_instance(tool.block_name, frame)
    except RuntimeError as exc:
        print(f"{_DIALOG}: ghost preview unavailable ({exc}).")
        return None


def _print_attach_diagnostics(M_tool_from_block) -> None:
    """Dump the picked attach rotation so a wrong pick is visible in the log."""
    print(f"{_DIALOG}: M_tool_from_block (block-local rotation), rows:")
    for row_index in range(3):
        row = M_tool_from_block[row_index]
        print(f"    [{row[0]:8.4f} {row[1]:8.4f} {row[2]:8.4f}]")
    cos_angle = (float(np.trace(M_tool_from_block[:3, :3])) - 1.0) / 2.0
    angle_deg = float(np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))))
    print(
        f"{_DIALOG}: -> the tool attaches rotated {angle_deg:.1f} deg from the "
        "block frame (0 deg = the historical 'tool sits on the block frame')."
    )


def _pick_ground_tool_frame(block_id, selected, scale_to_mm):
    """Pick the ground +X / +Y direction points and confirm the tool frame.

    Same shape as RSDefineRoboticTool's TCP picks -- pick a point giving the +X
    direction, then one giving +Y -- with one difference: the origin is NOT
    picked.  It is the ground block's own insertion point, so both vectors are
    measured from there.

    That origin doubles as the bar-axis anchor, so the bar line handed back is
    ``block origin -> +X point``.  ``M_block_from_bar`` therefore comes out with
    a zero translation, and RSJointPlace > JointOnly lands the block origin on the
    point clicked on the bar (rather than a fixed offset past it).

    Loops until accepted: pick +X -> pick +Y -> ghost tool -> Accept / Repick.
    Returns ``(x_point_id, y_point_id, bar_start, bar_end, M_tool_from_block)``
    with the points in DOCUMENT units, or ``None`` when cancelled.
    """
    tool = _resolve_preview_tool()
    if tool is None:
        print(
            f"{_DIALOG}: no active robotic tool pair; continuing WITHOUT the "
            "ghost preview (run RSSwapRoboticTool to get one)."
        )

    block_xform_doc, _block_name = picks.block_instance_frame(block_id)
    origin_doc = np.asarray(block_xform_doc[:3, 3], dtype=float)
    # SVD-orthonormalized so a scaled block instance still yields a rotation.
    block_rotation = orthonormalize_rotation(block_xform_doc[:3, :3])
    print(
        f"{_DIALOG}: frame origin = block insertion point "
        f"({origin_doc[0]:.4f}, {origin_doc[1]:.4f}, {origin_doc[2]:.4f}) [doc units]; "
        "both picked points are read as directions FROM it."
    )

    while True:
        with picks.temporarily_hidden(selected):
            x_point_id = picks.pick_point(
                "Pick the +X direction point (tool +X; also sets the bar axis)"
            )
        if x_point_id is None:
            return None
        with picks.temporarily_hidden(selected + [x_point_id]):
            y_point_id = picks.pick_point(
                "Pick the +Y direction point (direction only -- fixes the roll)"
            )
        if y_point_id is None:
            return None

        x_dir = picks.point_xyz(x_point_id) - origin_doc
        y_hint = picks.point_xyz(y_point_id) - origin_doc
        try:
            # X is exact; Y is only a hint, so the frame is re-orthonormalized
            # from it and the +Y point may be picked loosely.
            attach_frame = frame_from_x_and_y_hint(origin_doc, x_dir, y_hint)
        except ValueError:
            rs.MessageBox(
                "The +X and +Y points must not be collinear with the block "
                "origin, and neither may sit on it: +X sets the direction and "
                "+Y is what fixes the roll about it.",
                0,
                _DIALOG,
            )
            continue

        attach_rotation = attach_frame[:3, :3]
        M_tool_from_block = make_transform(rotation=block_rotation.T @ attach_rotation)
        _print_attach_diagnostics(M_tool_from_block)

        ghost_id = None
        answer = None
        try:
            if tool is not None:
                # `attach_frame` already sits on the block origin -- only the
                # rotation is being chosen here, never the position.
                ghost_id = _show_ghost_tool(tool, attach_frame, scale_to_mm)
                if ghost_id is not None:
                    sc.doc.Views.Redraw()
                    print(f"{_DIALOG}: ghost preview showing tool '{tool.name}'.")
            answer = rs.GetString("Tool orientation", "Accept", ["Accept", "Repick"])
        finally:
            if ghost_id is not None:
                delete_objects([ghost_id])
                sc.doc.Views.Redraw()

        if answer is None:
            return None
        if answer.strip().lower().startswith("r"):
            continue
        return (
            x_point_id,
            y_point_id,
            origin_doc,
            origin_doc + x_dir,
            M_tool_from_block,
        )


def main() -> None:
    _reload()
    rs.UnselectAllObjects()
    scale_to_mm = doc_unit_scale_to_mm()
    print(f"{_DIALOG}: scale_to_mm = {scale_to_mm:g}")

    selected: list = []

    block_id = picks.pick_block_instance(
        "Pick the joint block instance (named <Type>_<Subtype>)", _DIALOG
    )
    if block_id is None:
        print(f"{_DIALOG}: cancelled at block pick.")
        return
    selected.append(block_id)

    block_xform_doc, block_def_name = picks.block_instance_frame(block_id)
    if not block_def_name:
        rs.MessageBox(
            "Block instance must reference a named block definition.", 0, _DIALOG
        )
        return
    subtype = _subtype_of_picked_block(block_def_name)
    if subtype is None:
        print(f"{_DIALOG}: cancelled at block name check.")
        return
    is_half = subtype in jnc.HALF_SUBTYPES  # has a screw bore
    print(f"{_DIALOG}: block '{block_def_name}' -> {subtype}")

    registry = jp_mod.load_joint_registry()
    block_frame_mm = picks.frame_to_mm(block_xform_doc, scale_to_mm)

    # A MoCap block is a Female with a marker plate: when it was modelled on the
    # Female's origin, the Female's two matrices are already right for it.
    copied_from = None
    if subtype == jnc.MOCAP:
        female_name = jnc.block_name(jnc.block_type(block_def_name), jnc.FEMALE)
        if female_name in registry.halves:
            copy = _ask_copy_from_female(female_name)
            if copy is None:
                print(f"{_DIALOG}: cancelled at frame source choice.")
                return
            if copy:
                copied_from = registry.halves[female_name]

    # Ground blocks pick TWO POINTS -- a +X direction and a +Y direction, both
    # measured from the block's own origin (RSDefineRoboticTool's TCP picks,
    # minus the origin pick) -- which fix the tool-attach frame AND the bar
    # axis.  Halves keep the single bar-axis line pick: their tool attaches on
    # the block frame, so there is no second frame to choose.
    M_tool_from_block = np.eye(4)
    if copied_from is not None:
        M_block_from_bar = np.array(copied_from.M_block_from_bar, dtype=float)
        M_screw_from_block = np.array(copied_from.M_screw_from_block, dtype=float)
        print(f"{_DIALOG}: frames copied from '{copied_from.block_name}'.")
    else:
        if subtype == jnc.GROUND:
            picked = _pick_ground_tool_frame(block_id, selected, scale_to_mm)
            if picked is None:
                print(f"{_DIALOG}: cancelled at ground +X / +Y pick.")
                return
            x_point_id, y_point_id, bar_start_doc, bar_end_doc, M_tool_from_block = picked
            selected.extend([x_point_id, y_point_id])
        else:
            with picks.temporarily_hidden(selected):
                bar_id = picks.pick_line(f"Pick {subtype.upper()} bar axis line", _DIALOG)
            if bar_id is None:
                print(f"{_DIALOG}: cancelled at bar axis pick.")
                return
            selected.append(bar_id)
            bar_start_doc, bar_end_doc = picks.line_endpoints(bar_id)

        screw_axis_id = None
        screw_point_id = None
        if is_half:
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

        bar_start_mm = picks.vec_to_mm(bar_start_doc, scale_to_mm)
        bar_end_mm = picks.vec_to_mm(bar_end_doc, scale_to_mm)
        M_block_from_bar = picks.compute_M_block_from_bar(
            block_frame_mm, bar_start_mm, bar_end_mm
        )
        if is_half:
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

    print(
        f"{_DIALOG}: M_block_from_bar translation (mm) = "
        f"({M_block_from_bar[0,3]:.4f}, {M_block_from_bar[1,3]:.4f}, {M_block_from_bar[2,3]:.4f})"
    )
    if is_half:
        screw_z = M_screw_from_block[:3, 2]
        screw_origin_local = M_screw_from_block[:3, 3]
        print(
            f"{_DIALOG}: M_screw_from_block translation (mm in block frame) = "
            f"({screw_origin_local[0]:.4f}, {screw_origin_local[1]:.4f}, {screw_origin_local[2]:.4f})"
        )
        print(
            f"{_DIALOG}: screw Z axis (in block frame) = "
            f"({screw_z[0]:.4f}, {screw_z[1]:.4f}, {screw_z[2]:.4f})"
        )

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

    markers = None
    if subtype == jnc.MOCAP:
        markers = _pick_marker_points(block_frame_mm, scale_to_mm, selected)
        if markers is None:
            print(f"{_DIALOG}: no markers picked; keeping the recorded ones.")

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
    if is_half:
        # Re-defining must not silently drop what only the JSON holds.
        previous = registry.halves.get(block_def_name)
        half = jp_mod.JointHalfDef(
            block_name=block_def_name,
            M_block_from_bar=M_block_from_bar,
            M_screw_from_block=M_screw_from_block,
            asset_filename=asset_filename,
            collision_filename=obj_filename,
            preferred_robotic_tool_name=(
                previous.preferred_robotic_tool_name if previous else ""
            ),
            bar_cradle=previous.bar_cradle if previous else False,
            marker_points_mm=(
                markers if markers is not None
                else (previous.marker_points_mm if previous else {})
            ),
        )
        jp_mod.save_joint_half(half)
        print(f"{_DIALOG}: saved half '{block_def_name}' (kind={half.kind}) to registry.")
    else:
        ground = jp_mod.GroundJointDef(
            name=block_def_name,
            block_name=block_def_name,
            M_block_from_bar=M_block_from_bar,
            asset_filename=asset_filename,
            collision_filename=obj_filename,
            M_tool_from_block=M_tool_from_block,
        )
        jp_mod.save_ground_joint(ground)
        # The tool-attach offsets are cached per registry file stamp; drop the
        # cache so the very next tool placement in this session uses the frame
        # just picked, without waiting on a file-stat comparison.
        from core.rhino_tool_place import clear_tool_attach_cache  # noqa: PLC0415

        clear_tool_attach_cache()
        print(f"{_DIALOG}: saved ground joint '{block_def_name}' to registry.")

    # Either path just wrote a collision OBJ the session's collision cache has
    # never seen -- without this the new block is skipped from collision until
    # Rhino restarts.
    from core.env_collision import clear_joint_obj_path_cache  # noqa: PLC0415

    clear_joint_obj_path_cache()
    print(f"{_DIALOG}: done.")


if __name__ == "__main__":
    main()
