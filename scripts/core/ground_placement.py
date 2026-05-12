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

Layer + UserText conventions:

* All baked instances live on ``LAYER_JOINT_GROUND_INSTANCES``.
* Object name pattern: ``{joint_id}_ground`` so :func:`_remove_placed_ground`
  can find them later (mirrors the female/male naming in
  :mod:`core.joint_placement`).
* ``joint_id`` pattern: ``G{bar_num}-{ground_name}``.

All Rhino-runtime imports are deferred so this module is safe to import
from non-Rhino test contexts.
"""

from __future__ import annotations

import numpy as np

from core import config
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

GROUND_INSTANCES_LAYER = config.LAYER_JOINT_GROUND_INSTANCES

# UserText role tag set on interactive preview blocks (RSGroundPlace).
JOINT_ROLE_GROUND = "ground"

# UserText `joint_type` value for ground joints (parallels female/male types
# written by `core.joint_placement._write_joint_user_text`).
GROUND_JOINT_TYPE = "ground"

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


def _bar_id_to_num(bar_id: str) -> str:
    """``"B7"`` -> ``"7"``; safe on already-numeric or empty input."""
    return str(bar_id).lstrip("B") if bar_id else "?"


def _ground_id_base(bar_id: str, ground_name: str) -> str:
    return f"G{_bar_id_to_num(bar_id)}-{ground_name}"


def make_ground_joint_id(bar_id: str, ground_name: str, *, index: int = 0) -> str:
    """Return ``G{bar_num}-{ground_name}-{index}``.

    Multiple ground joints with the same (bar, ground_name) coexist via
    distinct ``index`` values.  Use :func:`next_ground_joint_index` to
    pick the next free one when baking a NEW placement.
    """
    return f"{_ground_id_base(bar_id, ground_name)}-{int(index)}"


def next_ground_joint_index(bar_id: str, ground_name: str) -> int:
    """Return the smallest non-negative integer ``i`` such that
    ``G{bar_num}-{ground_name}-{i}`` is not already used by a baked ground
    block on ``GROUND_INSTANCES_LAYER``.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    base = _ground_id_base(bar_id, ground_name)
    used = set()
    if rs.IsLayer(GROUND_INSTANCES_LAYER):
        for oid in rs.ObjectsByLayer(GROUND_INSTANCES_LAYER) or []:
            jid = rs.GetUserText(oid, "joint_id") or ""
            if jid.startswith(base + "-"):
                tail = jid[len(base) + 1:]
                if tail.isdigit():
                    used.add(int(tail))
    i = 0
    while i in used:
        i += 1
    return i


def insert_ground_block_preview(block_name: str, frame: np.ndarray):
    """Insert a colored preview block instance, tagged with role=ground."""
    from core.joint_placement import insert_block_instance  # noqa: PLC0415

    return insert_block_instance(
        block_name,
        frame,
        color=GROUND_PREVIEW_COLOR,
        role=JOINT_ROLE_GROUND,
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
    :func:`next_ground_joint_index` so multiple ground placements on the
    same bar+ground_def coexist.  Pass an explicit ``joint_id`` from
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
        idx = next_ground_joint_index(bar_id, ground.name)
        joint_id = make_ground_joint_id(bar_id, ground.name, index=idx)

    with suspend_redraw():
        oid = insert_block_instance(
            block_name, frame, layer_name=GROUND_INSTANCES_LAYER
        )
        rs.ObjectName(oid, f"{joint_id}_ground")
        rs.SetUserText(oid, "joint_id", joint_id)
        rs.SetUserText(oid, "joint_type", GROUND_JOINT_TYPE)
        rs.SetUserText(oid, "ground_joint_name", ground.name)
        rs.SetUserText(oid, "block_name", block_name)
        rs.SetUserText(oid, "parent_bar_id", str(bar_id))
        rs.SetUserText(oid, "position_mm", f"{float(jp):.4f}")
        rs.SetUserText(oid, "rotation_deg", f"{float(np.degrees(jr)):.4f}")
        rs.SetUserText(oid, "flipped", "True" if flipped else "False")

    print(
        f"RSGroundPlace: placed {joint_id} on {bar_id} "
        f"(jp={jp:.2f} mm, jr={np.degrees(jr):.1f} deg, flipped={flipped})."
    )
    return oid, joint_id


def remove_placed_ground(joint_id: str) -> None:
    """Delete the baked ground block instance for ``joint_id`` (if any)."""
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    ids = rs.ObjectsByName(f"{joint_id}_ground")
    if ids:
        rs.DeleteObjects(ids)


__all__ = [
    "GROUND_INSTANCES_LAYER",
    "JOINT_ROLE_GROUND",
    "GROUND_JOINT_TYPE",
    "GROUND_PREVIEW_COLOR",
    "effective_M_block_from_bar",
    "fk_ground_block_frame",
    "auto_jr_y_down",
    "make_ground_joint_id",
    "next_ground_joint_index",
    "insert_ground_block_preview",
    "place_ground_block",
    "remove_placed_ground",
]
