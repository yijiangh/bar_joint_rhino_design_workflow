#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineRoboticTool - Define a new robotic-tool entry from baked Rhino geometry.

Workflow (each pick auto-hides previous selections so stacked geometry is
easier to grab; everything is shown again at the end):

    1. tool block instance (collision mesh baked at the robot flange frame)
    2. TCP origin point (where the male joint will be held)
    3. TCP +X axis tip point  (defines TCP X direction)
    4. TCP +Y axis tip point  (only used to disambiguate Z handedness;
       the recorded TCP frame is re-orthonormalized from X and Y)
    5. enter the tool name in the command console

The TCP frame is recorded **relative to the block instance's local frame**
(i.e. ``M_tcp_from_block``), so it is invariant under any rigid motion of
the tool in world space.  At placement time the tool block is inserted at
``world_tcp @ inv(M_tcp_from_block)``.

The block definition is exported to ``asset/<block_name>.3dm`` (skipped if
the file already exists), and the entry is saved to
``scripts/core/robotic_tools.json``.
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

from core import robotic_tool as _robotic_tool_module
from core.rhino_block_export import export_block_definition_to_3dm
from core.rhino_helpers import point_to_array, suspend_redraw


def _reload():
    global robotic_tool, RoboticToolDef, save_robotic_tool, DEFAULT_ASSET_DIR
    global frame_from_axes, invert_transform, unit
    importlib.reload(_robotic_tool_module)
    robotic_tool = _robotic_tool_module
    RoboticToolDef = robotic_tool.RoboticToolDef
    save_robotic_tool = robotic_tool.save_robotic_tool
    DEFAULT_ASSET_DIR = robotic_tool.DEFAULT_ASSET_DIR

    from core.transforms import (
        frame_from_axes as _frame_from_axes,
        invert_transform as _invert_transform,
        unit as _unit,
    )
    frame_from_axes = _frame_from_axes
    invert_transform = _invert_transform
    unit = _unit


_reload()


# ---------------------------------------------------------------------------
# Doc-unit scaling
# ---------------------------------------------------------------------------


def _doc_unit_scale_to_mm() -> float:
    return float(
        Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters)
    )


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
# Asset export (shared with rs_define_joint_pair via core.rhino_block_export)
# ---------------------------------------------------------------------------


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
    """
    x_dir = unit(np.asarray(x_tip - tcp_origin, dtype=float))
    y_hint = np.asarray(y_tip - tcp_origin, dtype=float)
    z_dir = unit(np.cross(x_dir, y_hint))
    y_dir = unit(np.cross(z_dir, x_dir))
    return frame_from_axes(np.asarray(tcp_origin, dtype=float), x_dir, y_dir, z_dir)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    _reload()
    rs.UnselectAllObjects()
    scale_to_mm = _doc_unit_scale_to_mm()

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

    tool_name = rs.GetString("Robotic tool name (required)")
    if tool_name is None:
        return
    tool_name = tool_name.strip()
    if not tool_name:
        rs.MessageBox("Tool name is required.", 0, "RSDefineRoboticTool")
        return

    block_xform_doc, block_name = _block_instance_frame(block_id)
    if not block_name:
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

    asset_filename = f"{block_name}.3dm"
    tool = RoboticToolDef(
        name=tool_name,
        block_name=block_name,
        M_tcp_from_block=M_tcp_from_block,
        asset_filename=asset_filename,
    )

    # Diagnostics.
    print(f"RSDefineRoboticTool: tool '{tool_name}'")
    print(f"  block name              : {block_name}")
    tcp_local_origin = M_tcp_from_block[:3, 3]
    tcp_local_z = M_tcp_from_block[:3, 2]
    print(
        f"  TCP origin (block-local mm)  : "
        f"({tcp_local_origin[0]:.4f}, {tcp_local_origin[1]:.4f}, {tcp_local_origin[2]:.4f})"
    )
    print(
        f"  TCP +Z axis (block-local)    : "
        f"({tcp_local_z[0]:.4f}, {tcp_local_z[1]:.4f}, {tcp_local_z[2]:.4f})"
    )

    # Export asset (overwrite any existing file).
    asset_path = os.path.join(DEFAULT_ASSET_DIR, asset_filename)
    with suspend_redraw():
        if os.path.isfile(asset_path):
            print(f"  asset already exists, overwriting export -> {asset_path}")
        ok = export_block_definition_to_3dm(block_name, asset_path)
        if ok:
            print(f"  exported tool block   -> {asset_path}")
        else:
            print(f"  WARNING: failed to export tool block to {asset_path}")

    save_robotic_tool(tool)
    print(
        f"  registry updated: {os.path.join(SCRIPT_DIR, 'core', 'robotic_tools.json')}"
    )

    # Set as session default if no default has been set yet, so the next
    # joint placement immediately exercises the new tool.
    from core.rhino_tool_place import get_default_tool_name, set_default_tool_name
    if get_default_tool_name() is None:
        set_default_tool_name(tool_name)
        print(f"  default robotic tool for this document set to: {tool_name}")
    else:
        print(
            f"  default robotic tool unchanged "
            f"(still '{get_default_tool_name()}'); "
            f"set scaffolding.last_robotic_tool to '{tool_name}' to switch."
        )


if __name__ == "__main__":
    main()
