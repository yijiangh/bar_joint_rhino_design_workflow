#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineRoboticTool - ONE button for everything tool-definition related.

Two modes, chosen at the first command-line prompt:

**AssemblyTool** (default) -- define/update one assembly-tool candidate
(the tools that hold the male joint, mounted on an arm's tool0 flange).
Each pick auto-hides previous selections so stacked geometry is easier to
grab; everything is shown again at the end:

    1. pick the tool block instance (geometry baked at the robot flange
       frame; the block-local origin IS tool0)
    2. pick the TCP origin point (where the male joint will be held)
    3. pick the TCP +X axis tip point (defines TCP X direction)
    4. pick the TCP +Y axis tip point (only used to disambiguate Z
       handedness; the frame is re-orthonormalized from X and Y)
    5. pick the collision MESH object(s) -- hand-modeled low-poly Mesh
       objects positioned on the tool instance. Block instances are NOT
       accepted here (mesh objects only); we never auto-mesh breps/render
       geometry (too dense/unreliable for collision + planning). Every
       previous selection (the tool block AND the TCP points) is hidden
       for this pick, so only the collision meshes are left to grab.
    6. enter the tool name -- it MUST end in L (left arm) or R (right
       arm); the two sides of one candidate share a prefix (AT4L / AT4R)

    The script then exports ``asset/<tool_name>_SimpleVis.3dm`` (overwrite), writes
    the picked meshes to ``asset/<tool_name>.obj`` (overwrite; expressed in the
    source block's local frame, in millimeters), and saves the entry (including
    ``M_tcp_from_block`` and ``collision_filename``) into
    ``scripts/core/robotic_tools.json``.

    Reusing an existing tool name replaces that registry entry; it never
    creates a second entry with the same name. Defining a candidate does NOT
    activate it -- run RSSwapRoboticTool and type either member's name to make
    its L/R pair the active pair.

**SupportGripper** -- record the support-arm bar-grasp -> tool0 transform:
pick a baked bar-grasp frame group (origin on the bar centerline, Z along
the bar) and a baked tool0 frame group; writes
``BAR_GRASP_TO_TOOL0[gripper_kind] = inverse(bar_grasp) @ tool0`` into
``scripts/core/config_generated_ik.py`` (other gripper kinds preserved).

The TCP frame is recorded **relative to the block instance's local frame**
(i.e. ``M_tcp_from_block``), so it is invariant under any rigid motion of
the tool in world space.  At placement time the tool block is inserted at
``world_tcp @ inv(M_tcp_from_block)``.
"""

from __future__ import annotations

import contextlib
import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config_generated_ik as _generated_ik
from core import robotic_tool as _robotic_tool_module
from core.rhino_block_export import export_block_definition_to_3dm
from core.rhino_block_obj_export import export_picked_meshes_to_obj_mm
from core.rhino_frame_io import (
    doc_unit_scale_to_mm,
    reconstruct_frame,
    resolve_frame_group,
)
from core.rhino_helpers import point_to_array, suspend_redraw
from core.rhino_tool_place import get_default_tool_name, set_default_tool_name


DEFAULT_GRIPPER_KIND = "Robotiq"


def _reload():
    global robotic_tool, RoboticToolDef, save_robotic_tool, DEFAULT_ASSET_DIR
    global arm_side_from_tool_name, resolve_pair_for_tool, get_active_pair_names
    global frame_from_x_and_y_hint, invert_transform
    importlib.reload(_robotic_tool_module)
    robotic_tool = _robotic_tool_module
    RoboticToolDef = robotic_tool.RoboticToolDef
    save_robotic_tool = robotic_tool.save_robotic_tool
    DEFAULT_ASSET_DIR = robotic_tool.DEFAULT_ASSET_DIR
    arm_side_from_tool_name = robotic_tool.arm_side_from_tool_name
    resolve_pair_for_tool = robotic_tool.resolve_pair_for_tool
    get_active_pair_names = robotic_tool.get_active_pair_names

    from core.transforms import (
        frame_from_x_and_y_hint as _frame_from_x_and_y_hint,
        invert_transform as _invert_transform,
    )
    frame_from_x_and_y_hint = _frame_from_x_and_y_hint
    invert_transform = _invert_transform


_reload()


# ---------------------------------------------------------------------------
# Doc-unit scaling
# ---------------------------------------------------------------------------


def _frame_to_mm(matrix: np.ndarray, scale_to_mm: float) -> np.ndarray:
    out = np.array(matrix, dtype=float, copy=True)
    out[:3, 3] *= scale_to_mm
    return out


def _vec_to_mm(vector: np.ndarray, scale_to_mm: float) -> np.ndarray:
    return np.asarray(vector, dtype=float) * scale_to_mm


# ---------------------------------------------------------------------------
# Hide / show helpers
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def _temporarily_hidden(object_ids):
    hidden: list = []
    try:
        for oid in object_ids:
            if oid is not None and rs.IsObject(oid):
                if rs.HideObject(oid):
                    hidden.append(oid)
        yield
    finally:
        for oid in hidden:
            if rs.IsObject(oid):
                rs.ShowObject(oid)


# ---------------------------------------------------------------------------
# Pickers
# ---------------------------------------------------------------------------


def _pick_block_instance(prompt: str):
    return rs.GetObject(prompt, filter=4096, preselect=False, select=False)


def _pick_point(prompt: str):
    return rs.GetObject(prompt, filter=1, preselect=False, select=False)


def _pick_collision_sources(prompt: str):
    """Pick one or more collision-mesh objects (filter 32, mesh only).

    Block instances are intentionally NOT accepted here: collision meshes must
    be hand-modeled low-poly Mesh objects, never auto-derived from a block
    definition. Returns a list of guids or None.
    """
    return rs.GetObjects(
        prompt, filter=32, preselect=False, select=False, minimum_count=1
    )


def _block_instance_frame(block_instance_id) -> tuple[np.ndarray, str]:
    rh_obj = sc.doc.Objects.FindId(block_instance_id)
    if rh_obj is None or not isinstance(rh_obj, Rhino.DocObjects.InstanceObject):
        raise ValueError("Selected object is not a block instance.")
    xform = rh_obj.InstanceXform
    matrix = np.array(
        [[xform[r, c] for c in range(4)] for r in range(4)],
        dtype=float,
    )
    instance_def = rh_obj.InstanceDefinition
    block_name = instance_def.Name if instance_def is not None else ""
    return matrix, block_name


def _point_xyz(point_id) -> np.ndarray:
    point_obj = sc.doc.Objects.FindId(point_id)
    if point_obj is None:
        raise ValueError("Failed to resolve selected point.")
    return point_to_array(point_obj.Geometry.Location)


# ---------------------------------------------------------------------------
# TCP frame computation
# ---------------------------------------------------------------------------


def _world_tcp_frame(
    tcp_origin: np.ndarray, x_tip: np.ndarray, y_tip: np.ndarray
) -> np.ndarray:
    """Build a right-handed orthonormal TCP frame from three picked points.

    X = unit(x_tip - origin).  Z = unit(X x (y_tip - origin)).  Y = Z x X.
    The y_tip serves only to choose Z's sign; any non-collinear point on
    the +Y side will work.

    Thin wrapper over ``core.transforms.frame_from_x_and_y_hint`` -- the same
    recipe is used by RSDefineJointHalf's ground tool-attach picks, so the two
    commands can never drift apart on what "pick X, pick Y" means.
    """
    return frame_from_x_and_y_hint(
        np.asarray(tcp_origin, dtype=float),
        np.asarray(x_tip - tcp_origin, dtype=float),
        np.asarray(y_tip - tcp_origin, dtype=float),
    )


# ---------------------------------------------------------------------------
# AssemblyTool mode
# ---------------------------------------------------------------------------


def _run_assembly_tool_mode() -> None:
    """Define/update one assembly-tool candidate (block + TCP + registry)."""
    rs.UnselectAllObjects()
    scale_to_mm = doc_unit_scale_to_mm()

    selected: list = []

    block_id = _pick_block_instance("Pick TOOL block instance (baked at robot flange)")
    if block_id is None:
        return
    selected.append(block_id)

    with _temporarily_hidden(selected):
        tcp_origin_id = _pick_point("Pick TCP origin point")
    if tcp_origin_id is None:
        return
    selected.append(tcp_origin_id)

    with _temporarily_hidden(selected):
        x_tip_id = _pick_point("Pick TCP +X axis tip point")
    if x_tip_id is None:
        return
    selected.append(x_tip_id)

    with _temporarily_hidden(selected):
        y_tip_id = _pick_point("Pick TCP +Y axis tip point")
    if y_tip_id is None:
        return
    selected.append(y_tip_id)

    # Collision meshes -- hide EVERY previous selection (the tool block instance
    # AND the TCP points) so only the hand-modeled low-poly mesh objects are left
    # to pick. Block instances are not accepted in this step (mesh objects only);
    # we never auto-derive a collision mesh from a block definition.
    with _temporarily_hidden(selected):
        mesh_ids = _pick_collision_sources(
            "Pick collision MESH object(s), then press Enter"
        )
    if not mesh_ids:
        print("RSDefineRoboticTool: cancelled at collision-mesh pick (none selected).")
        return
    selected.extend(oid for oid in mesh_ids if oid not in selected)
    print(f"RSDefineRoboticTool: collision-mesh sources picked = {len(mesh_ids)}")

    tool_name = rs.GetString("Robotic tool name (must end in L or R, e.g. AT4L)")
    if tool_name is None:
        return
    tool_name = tool_name.strip()
    if not tool_name:
        rs.MessageBox("Tool name is required.", 0, "RSDefineRoboticTool")
        return
    # ! Naming rule: the L/R suffix assigns the arm side and pairs the two
    # ! tools of one candidate. Refuse anything else.
    if arm_side_from_tool_name(tool_name) is None:
        rs.MessageBox(
            "Assembly tools must be named with an L/R suffix (e.g. AT4L / "
            "AT4R) -- the suffix selects the arm side and pairs the two "
            "tools of one candidate.",
            0,
            "RSDefineRoboticTool",
        )
        return
    invalid_filename_chars = sorted(
        {char for char in tool_name if char in '<>:"/\\|?*' or ord(char) < 32}
    )
    if invalid_filename_chars:
        rs.MessageBox(
            "Tool name cannot be used as an asset filename because it contains "
            f"invalid character(s): {' '.join(repr(c) for c in invalid_filename_chars)}",
            0,
            "RSDefineRoboticTool",
        )
        return

    block_xform_doc, source_block_name = _block_instance_frame(block_id)
    if not source_block_name:
        rs.MessageBox(
            "Block instance must reference a named block definition.",
            0,
            "RSDefineRoboticTool",
        )
        return

    tcp_origin_doc = _point_xyz(tcp_origin_id)
    x_tip_doc = _point_xyz(x_tip_id)
    y_tip_doc = _point_xyz(y_tip_id)

    # Validate non-collinearity of the three TCP-defining points.
    x_vec = x_tip_doc - tcp_origin_doc
    y_vec = y_tip_doc - tcp_origin_doc
    if (
        float(np.linalg.norm(x_vec)) < 1e-9
        or float(np.linalg.norm(y_vec)) < 1e-9
        or float(np.linalg.norm(np.cross(x_vec, y_vec))) < 1e-9
    ):
        rs.MessageBox(
            "TCP origin, +X tip, and +Y tip must be three non-collinear points.",
            0,
            "RSDefineRoboticTool",
        )
        return

    # Convert to mm and compute world TCP frame, then take its expression
    # in the block's local frame.
    block_frame_mm = _frame_to_mm(block_xform_doc, scale_to_mm)
    tcp_origin_mm = _vec_to_mm(tcp_origin_doc, scale_to_mm)
    x_tip_mm = _vec_to_mm(x_tip_doc, scale_to_mm)
    y_tip_mm = _vec_to_mm(y_tip_doc, scale_to_mm)
    tcp_frame_world_mm = _world_tcp_frame(tcp_origin_mm, x_tip_mm, y_tip_mm)
    M_tcp_from_block = invert_transform(block_frame_mm) @ tcp_frame_world_mm

    # ! ---- Diagnostics ------------------------------------------------------
    # ! M_tcp_from_block is the flange->TCP transform, computed RELATIVE to the
    # ! picked block instance's frame. It changes ONLY when the three TCP points
    # ! move relative to THAT block -- a differently-SHAPED tool that still holds
    # ! the joint at the same offset from its flange yields the SAME M_tcp (the
    # ! shape lives in the .obj collision mesh, not here). These prints dump
    # ! every input and the full result so a "the TCP didn't change" report can
    # ! be checked against the real numbers: if the printed M_tcp already equals
    # ! the old value, the picks/block genuinely produced it; if it is new but
    # ! the saved JSON still shows the old value, the problem is the save step.
    print(f"RSDefineRoboticTool[AssemblyTool]: tool '{tool_name}'")
    print(f"  source block name        : {source_block_name}")
    print("  picked block world frame rows (mm):")
    for _r in range(4):
        _row = block_frame_mm[_r]
        print(f"    [{_row[0]:10.4f} {_row[1]:10.4f} {_row[2]:10.4f} {_row[3]:10.4f}]")
    print(
        f"  picked TCP points (world mm) : origin "
        f"({tcp_origin_mm[0]:.4f}, {tcp_origin_mm[1]:.4f}, {tcp_origin_mm[2]:.4f}) | "
        f"+X ({x_tip_mm[0]:.4f}, {x_tip_mm[1]:.4f}, {x_tip_mm[2]:.4f}) | "
        f"+Y ({y_tip_mm[0]:.4f}, {y_tip_mm[1]:.4f}, {y_tip_mm[2]:.4f})"
    )
    print("  computed M_tcp_from_block rows (block-local mm):")
    for _r in range(4):
        _row = M_tcp_from_block[_r]
        print(f"    [{_row[0]:10.4f} {_row[1]:10.4f} {_row[2]:10.4f} {_row[3]:10.4f}]")
    tcp_local_origin = M_tcp_from_block[:3, 3]
    tcp_local_z = M_tcp_from_block[:3, 2]
    print(
        f"  -> TCP origin (block-local mm): "
        f"({tcp_local_origin[0]:.4f}, {tcp_local_origin[1]:.4f}, {tcp_local_origin[2]:.4f})"
    )
    print(
        f"  -> TCP +Z axis (block-local)  : "
        f"({tcp_local_z[0]:.4f}, {tcp_local_z[1]:.4f}, {tcp_local_z[2]:.4f})"
    )

    # Export the block geometry (.3dm) and picked collision meshes (.obj) under
    # the registry TOOL NAME, not the source Rhino block name -- this keeps
    # different tool definitions independent even when their source blocks share
    # a generic name such as RightHandTool_SimpleVis. The .3dm carries a
    # `_SimpleVis` suffix to mark it as the visualization block; the swap/import
    # helper reads this exact filename back from the registry, so the suffix is
    # transparent to placement (the imported definition is still named <tool_name>).
    asset_filename = f"{tool_name}_SimpleVis.3dm"
    asset_path = os.path.join(DEFAULT_ASSET_DIR, asset_filename)
    obj_filename = f"{tool_name}.obj"
    obj_path = os.path.join(DEFAULT_ASSET_DIR, obj_filename)
    with suspend_redraw():
        if os.path.isfile(asset_path):
            print(f"  asset already exists, overwriting export -> {asset_path}")
        ok_3dm = export_block_definition_to_3dm(source_block_name, asset_path)
        if ok_3dm:
            print(f"  exported tool block     -> {asset_path}")
        else:
            print(f"  WARNING: failed to export tool block to {asset_path}")
        ok_obj = export_picked_meshes_to_obj_mm(
            mesh_ids, block_xform_doc, obj_path, label=tool_name
        )
        if ok_obj:
            print(f"  exported collision OBJ  -> {obj_path}")

    tool = RoboticToolDef(
        name=tool_name,
        # The imported definition is named after the tool as well. The
        # standalone .3dm contains loose geometry, so the swap/import helper
        # can assign this unique destination definition name automatically.
        block_name=tool_name,
        M_tcp_from_block=M_tcp_from_block,
        asset_filename=asset_filename,
        collision_filename=obj_filename if ok_obj else "",
    )
    was_registered = tool_name in robotic_tool.load_robotic_tools()
    save_robotic_tool(tool)
    registry_action = "replaced existing definition" if was_registered else "added"
    print(
        f"  registry {registry_action} for '{tool_name}': "
        f"{os.path.join(SCRIPT_DIR, 'core', 'robotic_tools.json')}"
    )
    if not ok_obj:
        # ! Saved WITHOUT a collision mesh so the TCP picks are not lost, but
        # ! the kinematic side will refuse this tool until the OBJ exists.
        print(
            "  ERROR: collision OBJ export FAILED for block "
            f"'{source_block_name}' (see console lines above). The registry entry "
            "was saved WITHOUT a collision mesh, and RSRebuildRobotCell will "
            "REFUSE this tool until it exists. Re-run RSDefineRoboticTool "
            "(AssemblyTool mode) and pick valid mesh objects."
        )

    # Pair status: is the other side of this candidate defined yet?
    try:
        pair = resolve_pair_for_tool(tool_name, robotic_tool.load_robotic_tools())
    except ValueError as exc:
        print(f"  pair status: {exc}")
    else:
        print(
            f"  pair complete: {pair['left'].name} (left) + "
            f"{pair['right'].name} (right)."
        )

    # Doc-default bookkeeping: only tools of the ACTIVE pair may become the
    # session default; otherwise remind the user how to activate this one.
    try:
        active_names = get_active_pair_names()
    except (RuntimeError, ValueError):
        active_names = None
    if active_names and tool_name in active_names.values():
        if get_default_tool_name() is None:
            set_default_tool_name(tool_name)
            print(f"  default robotic tool for this document set to: {tool_name}")
    else:
        print(
            "  note: this tool is NOT in the active pair; kinematics and "
            "placements keep using the current pair until you run "
            "RSSwapRoboticTool and type this tool name."
        )


# ---------------------------------------------------------------------------
# SupportGripper mode (bar-grasp -> tool0 export)
# ---------------------------------------------------------------------------


def _format_gripper_dict(entries: dict) -> str:
    """Emit ``BAR_GRASP_TO_TOOL0`` as python source (sorted, 9 sig digits)."""
    if not entries:
        return "BAR_GRASP_TO_TOOL0 = {}\n"
    lines = ["BAR_GRASP_TO_TOOL0 = {"]
    for kind in sorted(entries.keys()):
        lines.append(f'    "{kind}": (')
        for row in np.asarray(entries[kind], dtype=float):
            lines.append(
                "        (" + ", ".join(f"{float(v):.9g}" for v in row) + "),"
            )
        lines.append("    ),")
    lines.append("}\n")
    return "\n".join(lines)


def _write_generated_ik(path: str, gripper_entries: dict) -> None:
    """Rewrite ``config_generated_ik.py`` holding only ``BAR_GRASP_TO_TOOL0``."""
    header = (
        '"""Auto-generated IK-related CAD transforms.\n\n'
        "Generated by `scripts/rs_define_robotic_tool.py` (SupportGripper mode).\n"
        "Re-run that mode after the CAD bar-grasp frame or tool0 frame changes.\n\n"
        "All translations are stored in millimeters.\n\n"
        "`BAR_GRASP_TO_TOOL0[gripper_kind]` is a 4x4 that takes the bar-grasp\n"
        "frame (origin on bar centerline, Z along bar) to the gripper's tool0\n"
        "frame:\n\n"
        "    tool0_world = bar_grasp_world @ BAR_GRASP_TO_TOOL0[gripper_kind]\n"
        '"""\n\n'
    )
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(header + _format_gripper_dict(gripper_entries))


def _load_existing_gripper() -> dict:
    """Current ``BAR_GRASP_TO_TOOL0`` entries (so re-runs keep other kinds)."""
    importlib.reload(_generated_ik)
    raw = getattr(_generated_ik, "BAR_GRASP_TO_TOOL0", {}) or {}
    return {str(k): np.asarray(v, dtype=float) for k, v in raw.items()}


def _prompt_gripper_kind() -> str | None:
    value = rs.GetString("Gripper kind", DEFAULT_GRIPPER_KIND)
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    return value


def _run_support_gripper_mode() -> None:
    """Record ``inverse(bar_grasp) @ tool0`` for one gripper kind."""
    scale_to_mm = doc_unit_scale_to_mm()
    output_path = os.path.join(SCRIPT_DIR, "core", "config_generated_ik.py")

    gripper_kind = _prompt_gripper_kind()
    if gripper_kind is None:
        print("RSDefineRoboticTool[SupportGripper]: Cancelled.")
        return

    grasp_group = resolve_frame_group(
        "Select the baked BAR-GRASP frame group (origin on bar centerline, Z along bar)"
    )
    if grasp_group is None:
        print("RSDefineRoboticTool[SupportGripper]: Cancelled.")
        return

    tool0_group = resolve_frame_group("Select the baked tool0 frame group")
    if tool0_group is None:
        print("RSDefineRoboticTool[SupportGripper]: Cancelled.")
        return

    try:
        grasp_frame, grasp_label = reconstruct_frame(grasp_group[1], scale_to_mm)
        tool0_frame, tool0_label = reconstruct_frame(tool0_group[1], scale_to_mm)
    except ValueError as exc:
        rs.MessageBox(str(exc), 0, "RSDefineRoboticTool")
        return

    tf = invert_transform(grasp_frame) @ tool0_frame

    gripper_entries = _load_existing_gripper()
    gripper_entries[gripper_kind] = tf
    _write_generated_ik(output_path, gripper_entries)

    translation = tf[:3, 3]
    print(
        f"RSDefineRoboticTool[SupportGripper]: '{gripper_kind}' written to {output_path}"
    )
    print(
        f"  bar_grasp group='{grasp_label}', tool0 group='{tool0_label}', "
        f"translation (mm) = [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}]"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    _reload()

    mode = rs.GetString(
        "Define mode", "AssemblyTool", ["AssemblyTool", "SupportGripper"]
    )
    if mode is None:
        return
    normalized = mode.strip().lower()
    if normalized.startswith("a"):
        _run_assembly_tool_mode()
    elif normalized.startswith("s"):
        _run_support_gripper_mode()
    else:
        print(f"RSDefineRoboticTool: unknown mode {mode!r}; cancelled.")


if __name__ == "__main__":
    main()
