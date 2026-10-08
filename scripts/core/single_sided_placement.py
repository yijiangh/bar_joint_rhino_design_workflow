"""Placement of SINGLE-SIDED joints: one bar, no partner.

Two Subtypes are single-sided (``joint_name_conventions.SINGLE_SIDED_SUBTYPES``):

* **Ground** -- anchors a bar to the floor (``T20_Ground``, a
  ``joint_pair.GroundJointDef``).  It carries a robot tool.
* **MoCap**, standalone -- a marker carrier alone on a bar (``T20_MoCap``, a
  ``joint_pair.JointHalfDef``).  Fitted by hand: no tool.

Both are placed the same way, by RSJointPlace > JointOnly (and re-edited by
RSJointEdit).  Their DOFs:

* ``jp`` - signed distance along the bar's Z axis from ``bar_start`` (mm).
* ``jr`` - rotation about the bar's local Z axis (rad).

Forward kinematics, the same as every joint half::

    block_frame_world = bar_frame @ T_z(jp) @ R_z(jr) @ M_block_from_bar

``definition`` below is either kind of definition: both expose
``block_name``, ``asset_path()``, ``M_block_from_bar`` and ``type``.

Names (all from :mod:`core.joint_name_conventions`): layer
``Joint <Subtype> Instances``; joint id ``G<bar>-<Type>-<i>`` /
``M<bar>-<Type>-<i>`` (``G4-T20-0``, ``M7-T20-0``); object name
``<joint id>_<role>``.

All Rhino-runtime imports are deferred so this module is safe to import from
non-Rhino test contexts.
"""

from __future__ import annotations

import numpy as np

from core import joint_name_conventions as jnc
from core.joint_pair import canonical_bar_frame_from_line
from core.transforms import (
    rotation_about_local_z,
    rotation_matrix,
    make_transform,
    translation_transform,
)


# 180 deg rotation about block-local +Y.  Post-multiplied into
# `M_block_from_bar` to implement the user-facing ``flip`` operation (Ground
# only): block local +X and +Z reverse, +Y is preserved.  Because the block's
# +Y stays the same, the floor-facing ``jr`` does not change -- the only
# visible change is that the block's local X axis now points the other way
# along the bar (and local +Z is mirrored).
_FLIP_Y_PI = make_transform(rotation=rotation_matrix((0.0, 1.0, 0.0), np.pi))

#: Preview colour per Subtype (no variant cycling for single-sided joints).
PREVIEW_COLORS = {
    jnc.GROUND: (180, 120, 60),
    jnc.MOCAP: (60, 150, 200),
}
GROUND_PREVIEW_COLOR = PREVIEW_COLORS[jnc.GROUND]

#: A MoCap joint's plate faces along its block-local +Z (the back of the
#: Female it is built from -- measured: a placed Female's +Z points away from
#: its male).
MOCAP_PLATE_AXIS = 2
#: The plate faces this way by default ...
MOCAP_DEFAULT_FACING = (0.0, 0.0, 1.0)
#: ... unless the bar is within this angle of that direction, where "up" is
#: out of reach (the plate can only turn around the bar) and it faces this
#: instead.
MOCAP_VERTICAL_BAR_DEG = 15.0
MOCAP_FALLBACK_FACING = (1.0, 0.0, 0.0)


def subtype_of(definition) -> str:
    """``GROUND`` / ``MOCAP`` for a single-sided definition; raises otherwise."""
    subtype = jnc.block_subtype(definition.block_name)
    if subtype not in jnc.SINGLE_SIDED_SUBTYPES:
        raise ValueError(
            f"{definition.block_name!r} is a {subtype} block; only "
            f"{jnc.SINGLE_SIDED_SUBTYPES} are placed on a single bar"
        )
    return subtype


def effective_M_block_from_bar(definition, *, flipped: bool = False) -> np.ndarray:
    """The ``M_block_from_bar`` for this placement.

    When ``flipped`` is True we post-multiply by ``R_y(pi)``.  Composition
    order matters: post-multiplication applies the rotation in the
    BLOCK-LOCAL frame, so block-local +Y is preserved (and +X / +Z reversed).
    """
    if not flipped:
        return definition.M_block_from_bar
    return definition.M_block_from_bar @ _FLIP_Y_PI


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------


def fk_single_block_frame(
    bar_start, bar_end, jp: float, jr: float, definition, *, flipped: bool = False
) -> np.ndarray:
    """World-frame 4x4 of a single-sided block at ``(jp, jr)``."""
    bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
    M = effective_M_block_from_bar(definition, flipped=flipped)
    return (
        bar_frame
        @ translation_transform((0.0, 0.0, float(jp)))
        @ rotation_about_local_z(float(jr))
        @ M
    )


# ---------------------------------------------------------------------------
# Default angle about the bar
# ---------------------------------------------------------------------------


def auto_jr_facing(bar_start, bar_end, M_block_from_bar, local_axis: int, target) -> float:
    """``jr`` (rad) that turns block-local axis *local_axis* toward *target*.

    The block can only turn about the bar, so this is the best achievable
    alignment.  Closed form: with ``v = bar_R^T @ target`` (target in bar
    coords) and ``b = M[:3, local_axis]`` (the block axis in bar coords before
    the jr rotation), maximise::

        f(jr) = A cos(jr) + B sin(jr) + C

    where ``A = v_x*b_x + v_y*b_y``, ``B = v_y*b_x - v_x*b_y``,
    ``C = v_z*b_z``.  Maximum at ``jr = atan2(B, A)``; ``atan2(0, 0) == 0``
    when the target is along the bar.
    """
    bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
    v = bar_frame[:3, :3].T @ np.asarray(target, dtype=float)
    b = np.asarray(M_block_from_bar[:3, local_axis], dtype=float)
    A = float(v[0] * b[0] + v[1] * b[1])
    B = float(v[1] * b[0] - v[0] * b[1])
    return float(np.arctan2(B, A))


def auto_jr_y_down(
    bar_start,
    bar_end,
    ground,
    *,
    flipped: bool = False,
    world_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> float:
    """Ground: ``jr`` that turns the block's local +Y (its foot) toward the floor.

    ``flipped`` preserves block-local +Y, so the result does not depend on it;
    the argument exists for symmetry with :func:`fk_single_block_frame`.
    """
    M = effective_M_block_from_bar(ground, flipped=flipped)
    down = -np.asarray(world_up, dtype=float)
    return auto_jr_facing(bar_start, bar_end, M, 1, down)


def mocap_default_facing(bar_start, bar_end) -> tuple:
    """World +Z, or world +X when the bar is within 15 deg of vertical."""
    axis = np.asarray(bar_end, dtype=float) - np.asarray(bar_start, dtype=float)
    axis = axis / float(np.linalg.norm(axis))
    up = np.asarray(MOCAP_DEFAULT_FACING, dtype=float)
    if abs(float(axis @ up)) > np.cos(np.radians(MOCAP_VERTICAL_BAR_DEG)):
        return MOCAP_FALLBACK_FACING
    return MOCAP_DEFAULT_FACING


def auto_jr_mocap(bar_start, bar_end, half) -> float:
    """MoCap: ``jr`` that turns the marker plate up (or +X on a vertical bar)."""
    return auto_jr_facing(
        bar_start,
        bar_end,
        half.M_block_from_bar,
        MOCAP_PLATE_AXIS,
        mocap_default_facing(bar_start, bar_end),
    )


def default_jr(bar_start, bar_end, definition) -> float:
    """The starting ``jr`` for a new placement of *definition*."""
    if subtype_of(definition) == jnc.GROUND:
        return auto_jr_y_down(bar_start, bar_end, definition)
    return auto_jr_mocap(bar_start, bar_end, definition)


# ---------------------------------------------------------------------------
# Block insertion (Rhino-runtime)
# ---------------------------------------------------------------------------


def next_single_joint_index(subtype: str, bar_id: str, type_: str) -> int:
    """Smallest ``i`` such that ``G<bar>-<Type>-<i>`` (or ``M…``) is not
    already used by a baked block of *subtype*."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    base = jnc.single_joint_id_base(subtype, bar_id, type_)
    layer = jnc.joint_layer(subtype)
    used = set()
    if rs.IsLayer(layer):
        for oid in rs.ObjectsByLayer(layer) or []:
            parts = jnc.split_single_joint_id(rs.GetUserText(oid, jnc.UT_JOINT_ID))
            if parts and jnc.single_joint_id_base(*parts[:3]) == base:
                used.add(parts[3])
    i = 0
    while i in used:
        i += 1
    return i


def insert_single_sided_preview(definition, frame: np.ndarray):
    """Insert a coloured preview block, tagged with its Subtype."""
    from core.joint_placement import insert_block_instance  # noqa: PLC0415

    subtype = subtype_of(definition)
    return insert_block_instance(
        definition.block_name,
        frame,
        color=PREVIEW_COLORS[subtype],
        subtype=subtype,
    )


def place_single_sided_block(
    *,
    definition,
    bar_id: str,
    bar_start,
    bar_end,
    jp: float,
    jr: float,
    flipped: bool = False,
    joint_id: str | None = None,
    log_prefix: str = "RSJointPlace",
):
    """Bake a single-sided block with the user text RSJointEdit re-reads.

    When ``joint_id`` is None a fresh id is allocated, so several joints of
    one Type on one bar coexist; re-edit paths pass the existing id.

    Returns ``(object_id, joint_id)``.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    from core.joint_placement import insert_block_instance  # noqa: PLC0415
    from core.rhino_block_import import require_block_definition  # noqa: PLC0415
    from core.rhino_helpers import suspend_redraw  # noqa: PLC0415

    subtype = subtype_of(definition)
    block_name = require_block_definition(
        definition.block_name, asset_path=definition.asset_path()
    )
    frame = fk_single_block_frame(
        bar_start, bar_end, jp, jr, definition, flipped=flipped
    )
    if joint_id is None:
        idx = next_single_joint_index(subtype, bar_id, definition.type)
        joint_id = jnc.single_joint_id(subtype, bar_id, definition.type, idx)

    with suspend_redraw():
        oid = insert_block_instance(
            block_name, frame, layer_name=jnc.joint_layer(subtype)
        )
        rs.ObjectName(oid, jnc.object_name(joint_id, subtype))
        rs.SetUserText(oid, jnc.UT_JOINT_ID, joint_id)
        rs.SetUserText(oid, jnc.UT_JOINT_TYPE, definition.type)
        rs.SetUserText(oid, jnc.UT_JOINT_SUBTYPE, subtype)
        rs.SetUserText(oid, jnc.UT_BLOCK_NAME, block_name)
        rs.SetUserText(oid, jnc.UT_PARENT_BAR, str(bar_id))
        rs.SetUserText(oid, jnc.UT_POSITION, f"{float(jp):.4f}")
        rs.SetUserText(oid, jnc.UT_ROTATION, f"{float(np.degrees(jr)):.4f}")
        rs.SetUserText(oid, jnc.UT_FLIPPED, "True" if flipped else "False")

    print(
        f"{log_prefix}: placed {joint_id} on {bar_id} "
        f"(jp={jp:.2f} mm, jr={np.degrees(jr):.1f} deg, flipped={flipped})."
    )
    return oid, joint_id


def remove_placed_single(joint_id: str, subtype: str) -> None:
    """Delete the baked single-sided block for ``joint_id`` (if any)."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    ids = rs.ObjectsByName(jnc.object_name(joint_id, subtype))
    if ids:
        rs.DeleteObjects(ids)


def find_definition(block_name: str, registry):
    """The registered single-sided definition for *block_name*, or ``None``.

    Ground blocks live in ``registry.ground_joints``, MoCap blocks in
    ``registry.halves``.
    """
    if not jnc.is_block_name(block_name):
        return None
    subtype = jnc.block_subtype(block_name)
    if subtype == jnc.GROUND:
        return next(
            (g for g in registry.ground_joints.values() if g.block_name == block_name),
            None,
        )
    if subtype == jnc.MOCAP:
        return registry.halves.get(block_name)
    return None


def single_sided_definitions(registry) -> list:
    """Every registered single-sided definition, Ground first, by block name."""
    grounds = sorted(registry.ground_joints.values(), key=lambda g: g.block_name)
    mocaps = sorted(
        (h for h in registry.halves.values() if h.subtype == jnc.MOCAP),
        key=lambda h: h.block_name,
    )
    return [*grounds, *mocaps]


__all__ = [
    "PREVIEW_COLORS",
    "GROUND_PREVIEW_COLOR",
    "MOCAP_PLATE_AXIS",
    "subtype_of",
    "effective_M_block_from_bar",
    "fk_single_block_frame",
    "auto_jr_facing",
    "auto_jr_y_down",
    "auto_jr_mocap",
    "mocap_default_facing",
    "default_jr",
    "next_single_joint_index",
    "insert_single_sided_preview",
    "place_single_sided_block",
    "remove_placed_single",
    "find_definition",
    "single_sided_definitions",
]
