#! python 3
# venv: scaffolding_env
# r: numpy
"""RSDefineJointPair - Define a new joint pair from baked Rhino geometry.

Workflow (each pick auto-hides the previous selection so stacked geometry
is easier to grab; everything is shown again at the end):

    1. male block instance
    2. male bar axis line
    3. female block instance
    4. female bar axis line
    5. screw axis line
    6. screw center point
    7. enter the pair name in the command console

The script computes the constant transforms needed by the simplified
`JointPairDef` representation, exports each block definition to
`asset/<block_name>.3dm` (overwrite if the file already exists, becareful
when the same block to be reused across multiple pairs), and updates
`scripts/core/joint_pairs.json`.

Block sharing across pairs is supported: if a previously-registered pair
already references a male/female block with the same name and matching
constant transforms (within tolerance), the existing asset is reused and
no re-export is performed.
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

from core import joint_pair as _joint_pair_module
from core.rhino_block_export import export_block_definition_to_3dm
from core.rhino_helpers import point_to_array, suspend_redraw


def _reload():
    global joint_pair, JointHalfDef, JointPairDef, canonical_bar_frame_from_line
    global save_joint_pair, load_joint_pairs, DEFAULT_ASSET_DIR
    global invert_transform
    importlib.reload(_joint_pair_module)
    joint_pair = _joint_pair_module
    JointHalfDef = joint_pair.JointHalfDef
    JointPairDef = joint_pair.JointPairDef
    canonical_bar_frame_from_line = joint_pair.canonical_bar_frame_from_line
    save_joint_pair = joint_pair.save_joint_pair
    load_joint_pairs = joint_pair.load_joint_pairs
    DEFAULT_ASSET_DIR = joint_pair.DEFAULT_ASSET_DIR

    from core.transforms import invert_transform as _invert_transform
    invert_transform = _invert_transform


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
    """Hide given object ids on entry; restore visibility on exit."""

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
    object_id = rs.GetObject(prompt, filter=4096, preselect=False, select=False)
    return object_id


def _pick_line(prompt: str):
    object_id = rs.GetObject(prompt, filter=4, preselect=False, select=False)
    if object_id is None:
        return None
    curve = rs.coercecurve(object_id)
    if curve is None or not curve.IsLinear():
        rs.MessageBox("Selected curve must be a single straight line segment.", 0, "RSDefineJointPair")
        return None
    return object_id


def _pick_point(prompt: str):
    object_id = rs.GetObject(prompt, filter=1, preselect=False, select=False)
    return object_id


# ---------------------------------------------------------------------------
# Block instance -> 4x4 numpy frame
# ---------------------------------------------------------------------------


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


def _line_endpoints(line_id) -> tuple[np.ndarray, np.ndarray]:
    curve = rs.coercecurve(line_id)
    return point_to_array(curve.PointAtStart), point_to_array(curve.PointAtEnd)


def _point_xyz(point_id) -> np.ndarray:
    point_obj = sc.doc.Objects.FindId(point_id)
    if point_obj is None:
        raise ValueError("Failed to resolve selected point.")
    return point_to_array(point_obj.Geometry.Location)


# ---------------------------------------------------------------------------
# Asset export (shared with rs_define_robotic_tool via core.rhino_block_export)
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Block-sharing check
# ---------------------------------------------------------------------------
# A block name may appear in multiple joint pairs ONLY when its constant
# transforms (M_block_from_bar and M_screw_from_block) match the existing
# definition within tolerance.  This means the same physical part is being
# reused with the same bar-axis and screw-axis relationships.

_SHARED_BLOCK_TRANSLATION_TOL_MM = 0.05
_SHARED_BLOCK_ROTATION_TOL = 1e-3  # Frobenius norm on rotation delta


def _frames_match(
    a: np.ndarray, b: np.ndarray
) -> tuple[bool, float, float]:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    t_err = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    r_err = float(np.linalg.norm(a[:3, :3] - b[:3, :3]))
    return (
        t_err <= _SHARED_BLOCK_TRANSLATION_TOL_MM
        and r_err <= _SHARED_BLOCK_ROTATION_TOL,
        t_err,
        r_err,
    )


def _resolve_shared_half(
    pair_name: str,
    role: str,
    half: JointHalfDef,
) -> tuple[JointHalfDef, str | None]:
    """Look up any existing half in another pair using the same block name.

    Returns ``(half_to_store, error_message)``.  On match, the returned
    half adopts the existing asset filename so the asset is not re-exported
    under a new name.  On mismatch, an error message is returned.
    """

    pairs = load_joint_pairs()
    for other_name, other_pair in pairs.items():
        if other_name == pair_name:
            continue
        for other_role, other_half in (("female", other_pair.female), ("male", other_pair.male)):
            if other_half.block_name != half.block_name:
                continue
            block_match, t_err_b, r_err_b = _frames_match(
                half.M_block_from_bar, other_half.M_block_from_bar
            )
            screw_match, t_err_s, r_err_s = _frames_match(
                half.M_screw_from_block, other_half.M_screw_from_block
            )
            if block_match and screw_match:
                shared = JointHalfDef(
                    block_name=half.block_name,
                    M_block_from_bar=half.M_block_from_bar,
                    M_screw_from_block=half.M_screw_from_block,
                    asset_filename=other_half.asset_filename or half.asset_filename,
                    mesh_filename=other_half.mesh_filename or half.mesh_filename,
                    mesh_scale=other_half.mesh_scale,
                    preferred_robotic_tool_name=(
                        other_half.preferred_robotic_tool_name
                        or half.preferred_robotic_tool_name
                    ),
                )
                print(
                    f"  reusing block '{half.block_name}' from pair '{other_name}' "
                    f"({other_role}); transforms match (t_block={t_err_b:.4f} mm, "
                    f"r_block={r_err_b:.2e}, t_screw={t_err_s:.4f} mm, r_screw={r_err_s:.2e})."
                )
                return shared, None
            return half, (
                f"Block name '{half.block_name}' is already used by joint pair "
                f"'{other_name}' ({other_role}) but with different geometry "
                f"(M_block_from_bar t-err={t_err_b:.4f} mm, r-err={r_err_b:.2e}; "
                f"M_screw_from_block t-err={t_err_s:.4f} mm, r-err={r_err_s:.2e}). "
                f"A block can only be shared across pairs if its bar-axis and "
                f"screw-axis relationships are identical. Either re-pick using "
                f"the same bar/screw geometry, or rename one of the block "
                f"definitions in Rhino."
            )
    return half, None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _half_from_picks(
    block_xform_doc_units: np.ndarray,
    bar_start_doc: np.ndarray,
    bar_end_doc: np.ndarray,
    screw_origin_doc: np.ndarray,
    screw_dir_doc: np.ndarray,
    block_name: str,
    asset_filename: str,
    scale_to_mm: float,
) -> JointHalfDef:
    block_frame = _frame_to_mm(block_xform_doc_units, scale_to_mm)
    bar_start = _vec_to_mm(bar_start_doc, scale_to_mm)
    bar_end = _vec_to_mm(bar_end_doc, scale_to_mm)
    screw_origin = _vec_to_mm(screw_origin_doc, scale_to_mm)

    from core.transforms import frame_from_axes, unit

    # Block X-axis in world coords; used to anchor the bar/screw X axes so
    # that M_block_from_bar and M_screw_from_block are invariant under any
    # rigid rotation of the whole rig in world space (block + bar + screw
    # rotated together).  Without this, the X axis would be derived via
    # `orthogonal_to(z)`, which snaps to a world reference axis and is
    # therefore NOT equivariant under world rotation -- breaking block
    # sharing across pairs whose blocks are placed in different orientations.
    block_x_world = unit(block_frame[:3, 0])
    block_y_world = unit(block_frame[:3, 1])

    def _x_anchored_to_block(z_axis: np.ndarray) -> np.ndarray:
        """Return a unit X axis perpendicular to ``z_axis`` and anchored to
        the block's local X (or Y if X is parallel to Z)."""
        for ref in (block_x_world, block_y_world):
            x_proj = ref - float(np.dot(ref, z_axis)) * z_axis
            n = float(np.linalg.norm(x_proj))
            if n > 1e-9:
                return x_proj / n
        # Degenerate fallback (should not happen with a well-formed block).
        from core.transforms import orthogonal_to
        return orthogonal_to(z_axis)

    # Bar frame: origin at bar_start, Z along bar, X anchored to block X.
    bar_z = unit(np.asarray(bar_end - bar_start, dtype=float))
    bar_x = _x_anchored_to_block(bar_z)
    bar_y = unit(np.cross(bar_z, bar_x))
    bar_frame = frame_from_axes(bar_start, bar_x, bar_y, bar_z)
    M_block_from_bar = invert_transform(bar_frame) @ block_frame

    # Screw frame: origin at screw_origin, Z along screw axis, X anchored to
    # block X (same rationale as above).
    z_axis = unit(np.asarray(screw_dir_doc, dtype=float))
    x_axis = _x_anchored_to_block(z_axis)
    y_axis = unit(np.cross(z_axis, x_axis))
    screw_frame = frame_from_axes(screw_origin, x_axis, y_axis, z_axis)
    M_screw_from_block = invert_transform(block_frame) @ screw_frame

    return JointHalfDef(
        block_name=block_name,
        M_block_from_bar=M_block_from_bar,
        M_screw_from_block=M_screw_from_block,
        asset_filename=asset_filename,
    )


def main() -> None:
    _reload()
    rs.UnselectAllObjects()
    scale_to_mm = _doc_unit_scale_to_mm()

    selected: list = []

    # 1. male block
    male_block_id = _pick_block_instance("Pick MALE block instance")
    if male_block_id is None:
        return
    selected.append(male_block_id)

    # 2. male bar axis
    with _temporarily_hidden(selected):
        male_bar_id = _pick_line("Pick MALE bar axis line")
    if male_bar_id is None:
        return
    selected.append(male_bar_id)

    # 3. female block
    with _temporarily_hidden(selected):
        female_block_id = _pick_block_instance("Pick FEMALE block instance")
    if female_block_id is None:
        return
    selected.append(female_block_id)

    # 4. female bar axis
    with _temporarily_hidden(selected):
        female_bar_id = _pick_line("Pick FEMALE bar axis line")
    if female_bar_id is None:
        return
    selected.append(female_bar_id)

    # 5. screw axis line
    with _temporarily_hidden(selected):
        screw_axis_id = _pick_line("Pick SCREW axis line")
    if screw_axis_id is None:
        return
    selected.append(screw_axis_id)

    # 6. screw center point
    with _temporarily_hidden(selected):
        screw_point_id = _pick_point("Pick SCREW center point")
    if screw_point_id is None:
        return
    selected.append(screw_point_id)

    # 7. pair name from command line (no popup, no default)
    pair_name = rs.GetString("Joint pair name (required)")
    if pair_name is None:
        return
    pair_name = pair_name.strip()
    if not pair_name:
        rs.MessageBox("Joint pair name is required.", 0, "RSDefineJointPair")
        return

    # Collect data
    male_xform, male_block_name = _block_instance_frame(male_block_id)
    female_xform, female_block_name = _block_instance_frame(female_block_id)
    male_bar_start, male_bar_end = _line_endpoints(male_bar_id)
    female_bar_start, female_bar_end = _line_endpoints(female_bar_id)
    screw_axis_start, screw_axis_end = _line_endpoints(screw_axis_id)
    screw_point = _point_xyz(screw_point_id)

    if not male_block_name or not female_block_name:
        rs.MessageBox("Block instances must reference named block definitions.", 0, "RSDefineJointPair")
        return

    if male_block_name == female_block_name:
        rs.MessageBox("Male and female block definitions must be different.", 0, "RSDefineJointPair")
        return

    screw_dir = screw_axis_end - screw_axis_start

    # Use the picked screw center as the screw origin (project onto the axis
    # for safety even if the user picked a near-axis point).
    from core.transforms import project_point_to_line
    screw_origin_proj, _ = project_point_to_line(screw_point, screw_axis_start, screw_dir)

    # Asset filename mirrors the block definition name so the same male/female
    # block can be reused across multiple joint pairs without duplication.
    asset_filename_male = f"{male_block_name}.3dm"
    asset_filename_female = f"{female_block_name}.3dm"

    male_half = _half_from_picks(
        male_xform,
        male_bar_start,
        male_bar_end,
        screw_origin_proj,
        screw_dir,
        male_block_name,
        asset_filename_male,
        scale_to_mm,
    )
    female_half = _half_from_picks(
        female_xform,
        female_bar_start,
        female_bar_end,
        screw_origin_proj,
        screw_dir,
        female_block_name,
        asset_filename_female,
        scale_to_mm,
    )

    # Contact distance = length of the common perpendicular between the two
    # picked (infinite) bar center lines.  This is the user's intuitive
    # bar-to-bar gap.  The two bars MUST not be parallel: parallel picks
    # are ambiguous (the male's lateral offset gets folded into the
    # perpendicular distance and corrupts the value).  Pick bars at a
    # non-zero angle (orthogonal is ideal).
    from core.geometry import (
        closest_params_infinite_lines,
        distance_infinite_lines,
    )

    fbar_s_mm = _vec_to_mm(female_bar_start, scale_to_mm)
    fbar_d_mm = _vec_to_mm(female_bar_end - female_bar_start, scale_to_mm)
    mbar_s_mm = _vec_to_mm(male_bar_start, scale_to_mm)
    mbar_d_mm = _vec_to_mm(male_bar_end - male_bar_start, scale_to_mm)

    fbar_unit = fbar_d_mm / float(np.linalg.norm(fbar_d_mm))
    mbar_unit = mbar_d_mm / float(np.linalg.norm(mbar_d_mm))
    parallel_sin = float(np.linalg.norm(np.cross(fbar_unit, mbar_unit)))
    if parallel_sin < 1e-3:
        rs.MessageBox(
            "The two picked bars are (nearly) parallel.  "
            "Contact distance is only well-defined for non-parallel bars; "
            "please pick bars at a clear angle (orthogonal is ideal) and try again.",
            0,
            "RSDefineJointPair",
        )
        return

    contact_distance_mm = abs(
        float(distance_infinite_lines(fbar_s_mm, fbar_d_mm, mbar_s_mm, mbar_d_mm))
    )

    pair = JointPairDef(
        name=pair_name,
        female=female_half,
        male=male_half,
        contact_distance_mm=contact_distance_mm,
    )

    # Diagnostics — print before exporting assets so the user can sanity-check.
    print(f"RSDefineJointPair: pair '{pair_name}'")
    print(f"  contact distance        : {contact_distance_mm:.4f} mm")
    print(f"  female block            : {female_block_name}")
    print(f"  male   block            : {male_block_name}")
    fz = female_half.M_screw_from_block @ np.array([0, 0, 1, 0])
    mz = male_half.M_screw_from_block @ np.array([0, 0, 1, 0])
    print(f"  screw axis (mm) from female block : ({fz[0]:.4f}, {fz[1]:.4f}, {fz[2]:.4f})")
    print(f"  screw axis (mm) from male   block : ({mz[0]:.4f}, {mz[1]:.4f}, {mz[2]:.4f})")
    fo = female_half.M_screw_from_block[:3, 3]
    mo = male_half.M_screw_from_block[:3, 3]
    print(f"  screw origin offset female (mm) : |t|={np.linalg.norm(fo):.4f}")
    print(f"  screw origin offset male   (mm) : |t|={np.linalg.norm(mo):.4f}")

    # Resolve possible block sharing with previously-registered pairs.
    male_resolved, male_err = _resolve_shared_half(pair_name, "male", male_half)
    if male_err:
        rs.MessageBox(male_err, 0, "RSDefineJointPair")
        return
    female_resolved, female_err = _resolve_shared_half(pair_name, "female", female_half)
    if female_err:
        rs.MessageBox(female_err, 0, "RSDefineJointPair")
        return

    pair = JointPairDef(
        name=pair_name,
        female=female_resolved,
        male=male_resolved,
        contact_distance_mm=contact_distance_mm,
    )

    male_is_shared = male_resolved.asset_filename != asset_filename_male
    female_is_shared = female_resolved.asset_filename != asset_filename_female

    # Export block definitions to asset/ (overwrite any existing file; skip
    # only when this pair reuses a block already exported by another pair).
    asset_dir = DEFAULT_ASSET_DIR
    male_path = os.path.join(asset_dir, male_resolved.asset_filename)
    female_path = os.path.join(asset_dir, female_resolved.asset_filename)

    with suspend_redraw():
        if male_is_shared:
            print(f"  reusing male asset    -> {male_path}")
            ok_male = True
        else:
            if os.path.isfile(male_path):
                print(f"  asset already exists, overwriting male export -> {male_path}")
            ok_male = export_block_definition_to_3dm(male_block_name, male_path)
        if female_is_shared:
            print(f"  reusing female asset  -> {female_path}")
            ok_female = True
        else:
            if os.path.isfile(female_path):
                print(f"  asset already exists, overwriting female export -> {female_path}")
            ok_female = export_block_definition_to_3dm(female_block_name, female_path)

    if not male_is_shared:
        if not ok_male:
            print(f"  WARNING: failed to export male block to {male_path}")
        else:
            print(f"  exported male block   -> {male_path}")
    if not female_is_shared:
        if not ok_female:
            print(f"  WARNING: failed to export female block to {female_path}")
        else:
            print(f"  exported female block -> {female_path}")

    save_joint_pair(pair)
    print(f"  registry updated: {os.path.join(SCRIPT_DIR, 'core', 'joint_pairs.json')}")


if __name__ == "__main__":
    main()
