"""Shared ground-joint placement primitives.

Hosts the headless logic that the interactive ``RSGroundPlace`` command
(and its re-edit hook in ``RSJointEdit``) build on.

A *ground joint* anchors one bar to the world via a `GroundJointDef`
(see ``core.joint_pair.GroundJointDef``).  Its DOFs are:

* ``jp`` - signed distance along the bar's Z axis from ``bar_start`` (mm).
* ``jr`` - rotation about the bar's local Z axis (rad).

Forward kinematics::

    block_frame_world = bar_frame @ T_z(jp) @ R_z(jr) @ M_block_from_bar

There is NO mating partner and NO screw frame.

Names (all from :mod:`core.joint_name_conventions`):

* layer ``Joint Ground Instances``;
* joint id ``G<bar>-<Type>-<i>`` -- ``G4-T20-0`` for ``T20_Ground`` on ``B4``;
* object name ``<joint id>_ground``, so :func:`remove_placed_ground` can find it.

All Rhino-runtime imports are deferred so this module is safe to import
from non-Rhino test contexts.
"""

from __future__ import annotations

import numpy as np

from core import joint_name_conventions as jnc
from core.joint_pair import (
    GroundJointDef,
    canonical_bar_frame_from_line,
)
from core.transforms import (
    rotation_about_local_z,
    rotation_matrix,
    make_transform,
    translation_transform,
)


# 180 deg rotation about block-local +Y.  Post-multiplied into
# `M_block_from_bar` to implement the user-facing ``flip`` operation:
# block local +X and +Z reverse, +Y is preserved.  Because the block's
# +Y stays the same, the auto-jr-world-up heuristic returns the same
# ``jr``, so flipping does NOT change the bar-axial rotation -- the only
# visible change is that the block's local X axis now points the other
# way along the bar (and local +Z is mirrored).
_FLIP_Y_PI = make_transform(rotation=rotation_matrix((0.0, 1.0, 0.0), np.pi))


def effective_M_block_from_bar(ground: GroundJointDef, *, flipped: bool) -> np.ndarray:
    """Return the active ``M_block_from_bar`` for this placement.

    When ``flipped`` is True we post-multiply by ``R_y(pi)``.  Composition
    order matters: post-multiplication means the rotation is applied in
    the BLOCK-LOCAL frame, so block-local +Y is preserved (and block-local
    +X / +Z are reversed).
    """
    if not flipped:
        return ground.M_block_from_bar
    return ground.M_block_from_bar @ _FLIP_Y_PI


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Single preview color (no variant cycling for ground joints).
GROUND_PREVIEW_COLOR = (180, 120, 60)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------


def fk_ground_block_frame(
    bar_start: np.ndarray,
    bar_end: np.ndarray,
    jp: float,
    jr: float,
    ground: GroundJointDef,
    *,
    flipped: bool = False,
) -> np.ndarray:
    """Compute the world-frame 4x4 of the ground block at ``(jp, jr)``."""
    bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
    M = effective_M_block_from_bar(ground, flipped=flipped)
    return (
        bar_frame
        @ translation_transform((0.0, 0.0, float(jp)))
        @ rotation_about_local_z(float(jr))
        @ M
    )


# ---------------------------------------------------------------------------
# Auto-jr heuristic
# ---------------------------------------------------------------------------


def auto_jr_y_down(
    bar_start: np.ndarray,
    bar_end: np.ndarray,
    ground: GroundJointDef,
    *,
    flipped: bool = False,
    world_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> float:
    """Return ``jr`` (rad) that maximizes alignment of the block's local
    +Y axis with world DOWN (i.e. ``-world_up``).

    Ground-joint convention: the block is authored so that the foot/base
    points along its local +Y axis, and we want that to point at the
    ground -- i.e. block-local +Y should point along world -Z.

    Note: the ``flipped`` post-multiplication ``R_y(pi)`` preserves the
    block-local +Y column, so this function returns the same value
    whether ``flipped`` is True or False.  The argument exists for API
    symmetry with :func:`fk_ground_block_frame`.

    Closed form: with ``v = bar_R^T @ (-world_up)`` (world-down expressed
    in bar coords) and ``b = M[:3, 1]`` (block local +Y in bar coords
    before the jr rotation), we need to maximize::

        f(jr) = A cos(jr) + B sin(jr) + C

    where ``A = v_x*b_x + v_y*b_y``, ``B = v_y*b_x - v_x*b_y``,
    ``C = v_z*b_z``.  Maximum at ``jr = atan2(B, A)``.

    If the bar is colinear with world up (degenerate -- block-Y can never
    align in the bar's XY plane) the function still returns a finite
    value (``atan2(0, 0) == 0``).
    """
    bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
    bar_R = bar_frame[:3, :3]
    down = -np.asarray(world_up, dtype=float)
    v = bar_R.T @ down
    M = effective_M_block_from_bar(ground, flipped=flipped)
    b = np.asarray(M[:3, 1], dtype=float)
    A = float(v[0] * b[0] + v[1] * b[1])
    B = float(v[1] * b[0] - v[0] * b[1])
    return float(np.arctan2(B, A))


# ---------------------------------------------------------------------------
# Block insertion (Rhino-runtime)
# ---------------------------------------------------------------------------


def next_ground_joint_index(bar_id: str, type_: str) -> int:
    """Return the smallest ``i`` such that ``G<bar>-<Type>-<i>`` is not already
    used by a baked ground block."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    base = jnc.single_joint_id_base(jnc.GROUND, bar_id, type_)
    used = set()
    if rs.IsLayer(jnc.LAYER_GROUND):
        for oid in rs.ObjectsByLayer(jnc.LAYER_GROUND) or []:
            parts = jnc.split_single_joint_id(rs.GetUserText(oid, jnc.UT_JOINT_ID))
            if parts and jnc.single_joint_id_base(*parts[:3]) == base:
                used.add(parts[3])
    i = 0
    while i in used:
        i += 1
    return i


def insert_ground_block_preview(block_name: str, frame: np.ndarray):
    """Insert a colored preview block instance, tagged as a Ground preview."""
    from core.joint_placement import insert_block_instance  # noqa: PLC0415

    return insert_block_instance(
        block_name,
        frame,
        color=GROUND_PREVIEW_COLOR,
        subtype=jnc.GROUND,
    )


def place_ground_block(
    *,
    ground: GroundJointDef,
    bar_id: str,
    bar_start: np.ndarray,
    bar_end: np.ndarray,
    jp: float,
    jr: float,
    flipped: bool = False,
    joint_id: str | None = None,
):
    """Bake the final ground block instance with persistent UserText.

    When ``joint_id`` is None, a fresh id is allocated via
    :func:`next_ground_joint_index` so several ground placements of one Type
    on the same bar coexist.  Pass an explicit ``joint_id`` from
    re-edit code paths so the existing id is preserved across a flip.

    Returns ``(object_id, joint_id)``.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    from core.joint_placement import insert_block_instance  # noqa: PLC0415
    from core.rhino_block_import import require_block_definition  # noqa: PLC0415
    from core.rhino_helpers import suspend_redraw  # noqa: PLC0415

    block_name = require_block_definition(
        ground.block_name, asset_path=ground.asset_path()
    )
    frame = fk_ground_block_frame(
        bar_start, bar_end, jp, jr, ground, flipped=flipped
    )
    if joint_id is None:
        idx = next_ground_joint_index(bar_id, ground.type)
        joint_id = jnc.single_joint_id(jnc.GROUND, bar_id, ground.type, idx)

    type_, subtype = jnc.split_block_name(block_name)
    with suspend_redraw():
        oid = insert_block_instance(block_name, frame, layer_name=jnc.LAYER_GROUND)
        rs.ObjectName(oid, jnc.object_name(joint_id, jnc.GROUND))
        rs.SetUserText(oid, jnc.UT_JOINT_ID, joint_id)
        rs.SetUserText(oid, jnc.UT_JOINT_TYPE, type_)
        rs.SetUserText(oid, jnc.UT_JOINT_SUBTYPE, subtype)
        rs.SetUserText(oid, jnc.UT_BLOCK_NAME, block_name)
        rs.SetUserText(oid, jnc.UT_PARENT_BAR, str(bar_id))
        rs.SetUserText(oid, jnc.UT_POSITION, f"{float(jp):.4f}")
        rs.SetUserText(oid, jnc.UT_ROTATION, f"{float(np.degrees(jr)):.4f}")
        rs.SetUserText(oid, jnc.UT_FLIPPED, "True" if flipped else "False")

    print(
        f"RSGroundPlace: placed {joint_id} on {bar_id} "
        f"(jp={jp:.2f} mm, jr={np.degrees(jr):.1f} deg, flipped={flipped})."
    )
    return oid, joint_id


def remove_placed_ground(joint_id: str) -> None:
    """Delete the baked ground block instance for ``joint_id`` (if any)."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    ids = rs.ObjectsByName(jnc.object_name(joint_id, jnc.GROUND))
    if ids:
        rs.DeleteObjects(ids)


__all__ = [
    "GROUND_PREVIEW_COLOR",
    "effective_M_block_from_bar",
    "fk_ground_block_frame",
    "auto_jr_y_down",
    "next_ground_joint_index",
    "insert_ground_block_preview",
    "place_ground_block",
    "remove_placed_ground",
]
