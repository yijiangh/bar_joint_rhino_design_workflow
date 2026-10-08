"""Shared joint-placement primitives.

This module hosts the headless / non-interactive parts of the
joint-placement workflow that are reused by:

* :mod:`rs_joint_place`        -- the interactive ``RSJointPlace`` command,
* :mod:`rs_joint_edit`         -- the ``RSJointEdit`` flip command,
* :mod:`core.joint_auto_place` -- auto-placement after ``RSBarSnap`` /
  ``RSBarBrace`` / ``RSBarSubfloor`` create a new bar.

It owns:

* the preview colours (layer and user-text names live in
  :mod:`core.joint_name_conventions`),
* the variant-index encoding (``le_rev`` + ``ln_rev`` -> 0..3),
* the solver wrapper :func:`compute_variant`,
* interface-error tolerances and the auto-recovery helper
  :func:`compute_variant_with_recovery`,
* the block-insertion / metadata-tagging routine :func:`place_joint_blocks`.

All Rhino-runtime imports are deferred into function bodies so this
module is safe to import from non-Rhino test contexts.
"""

from __future__ import annotations

import numpy as np

from core import joint_name_conventions as jnc
from core.joint_pair_solver import (
    optimize_pair_placement,
    screw_alignment_diagnostics,
)
from core.rhino_block_import import require_block_definition
from core.rhino_helpers import (
    curve_endpoints,
    set_object_color,
    set_objects_layer,
    suspend_redraw,
)


# ---------------------------------------------------------------------------
# Preview constants
# ---------------------------------------------------------------------------

# Per-variant preview colors (cycled by variant index).
PREVIEW_COLORS = [
    (230, 80, 80),
    (80, 80, 230),
    (80, 200, 80),
    (200, 160, 50),
]

# A solved variant is "acceptable" when the female/male screw frames
# coincide within these tolerances.  Special joint geometries sometimes
# have only two valid variants (out of the nominal four); the other two
# land on local minima with large interface errors.  The auto-recovery
# helper detects this and flips one side once.
VARIANT_OK_ORIGIN_TOL_MM = 0.05
VARIANT_OK_Z_AXIS_TOL_RAD = 0.01  # ~0.57 deg


# ---------------------------------------------------------------------------
# Variant index encoding
# ---------------------------------------------------------------------------
#
# The 4 assembly variants come from reversing the bar endpoint order.
# Swapping start/end of a bar reverses its frame Z-axis, which reorients
# the connector along the bar.  Each combination is a full 4-DOF re-opt.
#
#     variant_index = (2 if ln_rev else 0) + (1 if le_rev else 0)
#
# bit 0 (le_rev) is the RECEIVER side (female or mocap); bit 1 (ln_rev) is the
# MALE side.


def variant_index(le_rev: bool, ln_rev: bool) -> int:
    return (2 if ln_rev else 0) + (1 if le_rev else 0)


def variant_flags(idx: int) -> tuple[bool, bool]:
    """Inverse of :func:`variant_index` -- returns ``(le_rev, ln_rev)``."""
    return bool(idx & 1), bool((idx >> 1) & 1)


# ---------------------------------------------------------------------------
# Block insertion (Rhino-runtime)
# ---------------------------------------------------------------------------


def _numpy_to_rhino_transform(matrix):
    import Rhino  # noqa: PLC0415

    xform = Rhino.Geometry.Transform(1.0)
    for row in range(4):
        for col in range(4):
            xform[row, col] = float(matrix[row, col])
    return xform


def insert_block_instance(
    block_name, frame, *, layer_name=None, color=None, subtype=None
):
    """Insert one block instance at world identity, then transform it.

    ``subtype``, when given, tags a PREVIEW block (user text
    ``UT_PREVIEW_SUBTYPE``) so the interactive pickers in :mod:`rs_joint_place`
    can tell receiver, male, ground and MoCap previews apart.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    oid = rs.InsertBlock(block_name, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(oid, _numpy_to_rhino_transform(frame))
    if layer_name:
        set_objects_layer(oid, layer_name)
    if color is not None:
        set_object_color(oid, color)
    if subtype is not None:
        rs.SetUserText(oid, jnc.UT_PREVIEW_SUBTYPE, subtype)
    return oid


# ---------------------------------------------------------------------------
# Variant computation + interface diagnostics
# ---------------------------------------------------------------------------


def compute_variant(le_start, le_end, ln_start, ln_end, le_rev, ln_rev, *, pair):
    """Run the full pair-placement optimization for one orientation choice.

    ``le_rev`` reverses the female-bar endpoint order; ``ln_rev`` reverses
    the male-bar endpoint order.  Returns the solver's result dict
    augmented with ``variant_index``, ``variant_label``,
    ``female_flip_rad`` / ``male_flip_rad`` (for diagnostics), and the
    cached ``origin_error_mm`` / ``z_axis_error_rad`` interface metrics.
    """
    les = le_end if le_rev else le_start
    lee = le_start if le_rev else le_end
    lns = ln_end if ln_rev else ln_start
    lne = ln_start if ln_rev else ln_end
    idx = variant_index(le_rev, ln_rev)
    label = (
        f"{idx + 1}: le={'rev' if le_rev else 'fwd'}, "
        f"ln={'rev' if ln_rev else 'fwd'}"
    )
    res = optimize_pair_placement(les, lee, lns, lne, pair, return_debug=True)
    diag = screw_alignment_diagnostics(
        res["female_screw_frame"],
        res["male_screw_frame"],
    )
    res.update(
        {
            "variant_index": idx,
            "variant_label": label,
            "female_flip_rad": float(np.pi) if le_rev else 0.0,
            "male_flip_rad": float(np.pi) if ln_rev else 0.0,
            "origin_error_mm": diag["origin_error_mm"],
            "z_axis_error_rad": diag["z_axis_error_rad"],
        }
    )
    return res


def interface_metrics(result) -> tuple[float, float]:
    """Return ``(origin_error_mm, z_axis_error_rad)`` for a variant dict.

    Uses cached values when present; otherwise recomputes from the
    female/male screw frames.
    """
    if "origin_error_mm" in result and "z_axis_error_rad" in result:
        return float(result["origin_error_mm"]), float(result["z_axis_error_rad"])
    diag = screw_alignment_diagnostics(
        result["female_screw_frame"],
        result["male_screw_frame"],
    )
    return float(diag["origin_error_mm"]), float(diag["z_axis_error_rad"])


def is_variant_acceptable(variant) -> bool:
    """True iff the variant's interface error is within tolerance."""
    origin_err, z_err = interface_metrics(variant)
    return (
        origin_err <= VARIANT_OK_ORIGIN_TOL_MM
        and abs(z_err) <= VARIANT_OK_Z_AXIS_TOL_RAD
    )


#: The two sides ``compute_variant_with_recovery`` can flip.
RECOVER_SIDES = ("receiver", "male")


def compute_variant_with_recovery(
    le_start,
    le_end,
    ln_start,
    ln_end,
    le_rev,
    ln_rev,
    *,
    pair,
    recover_side: str = "receiver",
    log_prefix: str = "RSJointPlace",
):
    """Compute the requested variant; if its interface error exceeds
    tolerance (a sign that this orientation lands on a degenerate local
    minimum), flip ``recover_side`` once and use that variant instead.

    Recovery never propagates more than once -- if the recovered variant
    is still bad, it is placed as-is with a warning.

    Returns ``(variant, recovered_bool, new_le_rev, new_ln_rev)``.
    """
    if recover_side not in RECOVER_SIDES:
        raise ValueError(
            f"recover_side must be one of {RECOVER_SIDES}, got {recover_side!r}"
        )

    variant = compute_variant(
        le_start, le_end, ln_start, ln_end, le_rev, ln_rev, pair=pair
    )
    if is_variant_acceptable(variant):
        return variant, False, le_rev, ln_rev

    origin_err, z_err = interface_metrics(variant)
    print(
        f"{log_prefix}: variant {variant['variant_index'] + 1} interface error "
        f"too large (origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
        f"auto-flipping {recover_side} side."
    )
    if recover_side == "receiver":
        new_le, new_ln = (not le_rev), ln_rev
    else:
        new_le, new_ln = le_rev, (not ln_rev)
    recovered = compute_variant(
        le_start, le_end, ln_start, ln_end, new_le, new_ln, pair=pair
    )
    rec_o, rec_z = interface_metrics(recovered)
    if is_variant_acceptable(recovered):
        print(
            f"  recovery succeeded -> variant {recovered['variant_index'] + 1} "
            f"(origin={rec_o:.4f} mm, z={np.degrees(rec_z):.4f}deg)."
        )
    else:
        print(
            f"  WARNING: recovery did not improve interface error "
            f"(origin={rec_o:.4f} mm, z={np.degrees(rec_z):.4f}deg); "
            f"placing the recovered variant as-is."
        )
    return recovered, True, new_le, new_ln


# ---------------------------------------------------------------------------
# Place final blocks
# ---------------------------------------------------------------------------


def _bar_unit_direction(curve_id):
    start, end = curve_endpoints(curve_id)
    direction = np.array(end) - np.array(start)
    norm = float(np.linalg.norm(direction))
    if norm <= 0.0:
        raise ValueError(f"Bar curve {curve_id} has zero length.")
    return direction / norm


def block_orientation_tag(block_x_world, bar_dir):
    """``"P"`` if the block's local +X axis points toward the bar end, ``"N"`` otherwise."""
    return "P" if float(np.dot(block_x_world, bar_dir)) > 0 else "N"


def write_joint_user_text(
    obj_id,
    *,
    joint_id,
    block_name,
    pair_name,
    parent_bar,
    connected_bar,
    le_bar_id,
    ln_bar_id,
    pos_mm,
    rot_rad,
    ori,
    le_rev,
    ln_rev,
    var_idx,
):
    """Write the user text RSJointEdit needs to re-open a placed pair half.

    ``joint_type`` / ``joint_subtype`` come from the block name
    (``T20_MoCap`` -> ``T20`` / ``MoCap``).  ``female_parent_bar`` keeps its
    on-disk name although the receiver may be a MoCap half.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    type_, subtype = jnc.split_block_name(block_name)
    text = {
        jnc.UT_JOINT_ID: joint_id,
        jnc.UT_JOINT_TYPE: type_,
        jnc.UT_JOINT_SUBTYPE: subtype,
        jnc.UT_PAIR_NAME: pair_name,
        jnc.UT_PARENT_BAR: parent_bar,
        jnc.UT_CONNECTED_BAR: connected_bar,
        jnc.UT_RECEIVER_BAR: le_bar_id,
        jnc.UT_MALE_BAR: ln_bar_id,
        jnc.UT_POSITION: f"{float(pos_mm):.4f}",
        jnc.UT_ROTATION: f"{float(np.degrees(rot_rad)):.4f}",
        jnc.UT_ORI: ori,
        jnc.UT_LE_REV: str(le_rev),
        jnc.UT_LN_REV: str(ln_rev),
        jnc.UT_VARIANT: str(var_idx),
    }
    for key, value in text.items():
        rs.SetUserText(obj_id, key, value)


def place_joint_blocks(result, le_id, ln_id, le_bar_id, ln_bar_id, *, pair):
    """Insert the receiver + male block instances for one solved variant.

    The receiver is whatever ``pair.receiver`` holds -- ``T20_Female`` or, via
    ``joint_pair.with_receiver``, ``T20_MoCap`` -- and its Subtype picks the
    layer and the object name, so ``J40-53_female`` on Joint Female Instances
    and ``J40-53_mocap`` on Joint MoCap Instances come out of the same code.

    Writes all required UserText so that :mod:`rs_joint_edit` can later
    re-open this joint without re-solving.  Returns
    ``(receiver_id, male_id, joint_id)``.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    female_frame = result["female_frame"]
    male_frame = result["male_frame"]
    origin_err, z_err = interface_metrics(result)

    receiver_block = require_block_definition(
        pair.receiver.block_name, asset_path=pair.receiver.asset_path()
    )
    male_block = require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    joint_id = jnc.pair_joint_id(le_bar_id, ln_bar_id)
    receiver_subtype = pair.receiver_subtype

    le_dir = _bar_unit_direction(le_id)
    ln_dir = _bar_unit_direction(ln_id)
    female_ori = block_orientation_tag(female_frame[:3, 0], le_dir)
    male_ori = block_orientation_tag(male_frame[:3, 0], ln_dir)

    var_idx = int(result.get("variant_index", 0))
    le_rev_val, ln_rev_val = variant_flags(var_idx)

    with suspend_redraw():
        receiver_id = insert_block_instance(
            receiver_block, female_frame,
            layer_name=jnc.joint_layer(receiver_subtype),
        )
        male_id = insert_block_instance(
            male_block, male_frame, layer_name=jnc.LAYER_MALE
        )
        # Object names are how RSJointEdit looks blocks back up by joint_id.
        rs.ObjectName(receiver_id, jnc.object_name(joint_id, receiver_subtype))
        rs.ObjectName(male_id, jnc.object_name(joint_id, jnc.MALE))

        write_joint_user_text(
            receiver_id,
            joint_id=joint_id,
            block_name=receiver_block,
            pair_name=pair.name,
            parent_bar=le_bar_id,
            connected_bar=ln_bar_id,
            le_bar_id=le_bar_id,
            ln_bar_id=ln_bar_id,
            pos_mm=result["fjp"],
            rot_rad=result["fjr"],
            ori=female_ori,
            le_rev=le_rev_val,
            ln_rev=ln_rev_val,
            var_idx=var_idx,
        )
        write_joint_user_text(
            male_id,
            joint_id=joint_id,
            block_name=male_block,
            pair_name=pair.name,
            parent_bar=ln_bar_id,
            connected_bar=le_bar_id,
            le_bar_id=le_bar_id,
            ln_bar_id=ln_bar_id,
            pos_mm=result["mjp"],
            rot_rad=result["mjr"],
            ori=male_ori,
            le_rev=le_rev_val,
            ln_rev=ln_rev_val,
            var_idx=var_idx,
        )

    print(f"RSJointPlace: Joints placed. Residual: {result['residual']:.6f}")
    print(
        f"  Variant {var_idx + 1}: "
        f"{jnc.role(receiver_subtype)} flip="
        f"{np.degrees(float(result.get('female_flip_rad', 0.0))):.1f}deg, "
        f"male flip={np.degrees(float(result.get('male_flip_rad', 0.0))):.1f}deg"
    )
    print(
        f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
        f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg"
    )
    print(
        f"  Interface origin err={origin_err:.4f} mm, "
        f"z-axis err={np.degrees(z_err):.4f}deg"
    )
    return receiver_id, male_id, joint_id


# Re-export what callers most commonly need.
__all__ = [
    "PREVIEW_COLORS",
    "RECOVER_SIDES",
    "VARIANT_OK_ORIGIN_TOL_MM",
    "VARIANT_OK_Z_AXIS_TOL_RAD",
    "variant_index",
    "variant_flags",
    "insert_block_instance",
    "block_orientation_tag",
    "write_joint_user_text",
    "compute_variant",
    "interface_metrics",
    "is_variant_acceptable",
    "compute_variant_with_recovery",
    "place_joint_blocks",
]
