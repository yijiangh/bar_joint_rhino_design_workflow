"""Joint pair data model, registry, and forward kinematics.

A joint pair is described by two halves (female and male).  Each half maps
a bar line and two scalar DOFs (`jp`, `jr`) to a block pose and a screw
frame via two constant 4x4 transforms:

    bar_frame   = canonical_bar_frame_from_line(bar_start, bar_end)
    block_frame = bar_frame @ T_z(jp) @ R_z(jr) @ half.M_block_from_bar
    screw_frame = block_frame @ half.M_screw_from_block

The optimizer aligns the female and male `screw_frame` origins and local
Z axes; roll about Z is unobservable and arbitrary.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from core import joint_name_conventions as jnc
from core.transforms import (
    frame_from_axes,
    orthogonal_to,
    orthonormalize_rotation,
    rotation_about_local_z,
    translation_transform,
    unit,
)


_CORE_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(_CORE_DIR)
REPO_DIR = os.path.dirname(SCRIPTS_DIR)
DEFAULT_REGISTRY_PATH = os.path.join(_CORE_DIR, "joint_pairs.json")
DEFAULT_ASSET_DIR = os.path.join(REPO_DIR, "asset")

DEFAULT_JP_RANGE = (-500.0, 500.0)
DEFAULT_JR_RANGE = (-math.pi, math.pi)


def _as_4x4(value: Iterable[Iterable[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")
    out = np.array(matrix, dtype=float, copy=True)
    out[:3, :3] = orthonormalize_rotation(out[:3, :3])
    out[3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return out


# ---------------------------------------------------------------------------
# Half kinds
# ---------------------------------------------------------------------------
# A half's ``kind`` is its Subtype's role ("T20_MoCap" -> "mocap"); see
# ``core.joint_name_conventions``.  It is stored in the registry for a readable
# JSON file and checked against the block name on load, so the two can never
# disagree.
#
# A MoCap half is used TWO ways with the same registry entry -- only the
# PLACEMENT differs:
#
#   paired      the receiver of its Type's mate (``with_receiver(T20, MOCAP)``):
#               a male screws into it exactly as into a Female.  Gets a
#               ``J<le>-<ln>`` id and carries ``joint_pair_name`` /
#               ``male_parent_bar``.
#   standalone  placed on one bar with no mate, purely to carry markers.  Gets
#               an ``M<bar>-<Type>-<i>`` id and carries none of those keys.
#
# ``marker_points_mm`` matters in BOTH cases, so the MoCap LAYER is exactly the
# set of marker-bearing joints, while ``male_parent_bar`` says whether one is
# also structural.

#: What ``JointHalfDef.kind`` may be: every role except ground.
VALID_HALF_KINDS = tuple(jnc.role(s) for s in jnc.HALF_SUBTYPES)


@dataclass(frozen=True)
class JointHalfDef:
    """Constant geometry for one half of a joint pair."""

    block_name: str
    M_block_from_bar: np.ndarray
    M_screw_from_block: np.ndarray
    # The Subtype's role ("female" / "male" / "mocap").  Empty -> derived
    # from the block name; anything else must agree with it.
    kind: str = ""
    asset_filename: str = ""           # e.g. "typical_female.3dm" under asset/
    mesh_filename: str = ""            # URDF mesh, optional
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    preferred_robotic_tool_name: str = ""  # used at first-place time only
    # OBJ filename (under DEFAULT_ASSET_DIR) used as the low-poly collision
    # mesh attached as a `compas_fab.robots.RigidBody` for env collision in
    # the IK keyframe workflow. The OBJ origin must coincide with the block
    # definition's local frame. Empty string -> fallback to slow Rhino
    # block-def render-mesh path.
    collision_filename: str = ""
    # True for cradle-style female halves (the subfloor receivers): the OTHER
    # bar physically rests INSIDE this block at the assembled pose, so the IK
    # allowed-contact policy must let the incoming bar (and its male) touch
    # this female during the approach + insert movements. Normal clamp-style
    # females keep the default False.
    bar_cradle: bool = False
    # Centres of the OptiTrack marker spheres this half carries: {Motive label ->
    # (x, y, z)} in MILLIMETRES, in the BLOCK DEFINITION's own frame.  POINTS, not
    # frames -- a sphere is rotationally symmetric and Motive reports a position
    # only, so unlike `M_screw_from_block` there is no axis to record.
    #
    # Block-local so the numbers describe the PART, not wherever the block sat
    # when it was defined: a placed instance's predicted world positions are
    # `block_world @ point`.  Labelled because pairing a measured marker with the
    # modelled one needs its identity, not just a nearby position.
    #
    # NOTE those predictions are in DOCUMENT coordinates, while Motive reports in
    # its own calibrated lab frame -- the two have different origins, so a
    # predicted and a measured position cannot be compared until something
    # registers one frame onto the other (today `rs_align_model_three_bars` does
    # that from three bar axes).  Only the leftover after that registration is
    # build error.
    #
    # `default_factory` rather than `= {}` because a mutable default would be
    # SHARED by every instance of this class.  Empty for every half carrying no
    # markers; registries written before this field existed omit it -> {}.
    marker_points_mm: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "M_block_from_bar", _as_4x4(self.M_block_from_bar))
        object.__setattr__(self, "M_screw_from_block", _as_4x4(self.M_screw_from_block))
        subtype = jnc.block_subtype(self.block_name)
        if subtype not in jnc.HALF_SUBTYPES:
            raise ValueError(
                f"{self.block_name!r} is a {subtype} block; only "
                f"{jnc.HALF_SUBTYPES} are joint halves (ground joints are "
                "GroundJointDef)."
            )
        if not self.kind:
            object.__setattr__(self, "kind", jnc.role(subtype))
        elif self.kind != jnc.role(subtype):
            raise ValueError(
                f"JointHalfDef {self.block_name!r} has kind={self.kind!r}, but its "
                f"block name says {jnc.role(subtype)!r} (<Type>_<Subtype>)."
            )
        # Same normalize-on-construction job `_as_4x4` does for the matrices
        # above.  Callers hand in numpy arrays (every Rhino pick path produces
        # them), lists, or ints -- and numpy values do NOT survive `json.dump`,
        # which would fail at save time with a traceback pointing at json rather
        # than at the bad input.  A wrong-length point raises here, naming the
        # label, instead of much later inside `block_world @ point`.  Keys are
        # forced to str because JSON object keys always come back as strings.
        points = {}
        for label, point in dict(self.marker_points_mm).items():
            coords = tuple(float(c) for c in point)
            if len(coords) != 3:
                raise ValueError(
                    f"marker_points_mm[{label!r}] must have 3 coordinates, "
                    f"got {len(coords)}"
                )
            points[str(label)] = coords
        object.__setattr__(self, "marker_points_mm", points)

    @property
    def subtype(self) -> str:
        """``"Female"`` / ``"Male"`` / ``"MoCap"`` -- from the block name."""
        return jnc.block_subtype(self.block_name)

    @property
    def type(self) -> str:
        """``"T20"`` -- the product family, from the block name."""
        return jnc.block_type(self.block_name)

    def asset_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.asset_filename) if self.asset_filename else ""

    def collision_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.collision_filename) if self.collision_filename else ""

    def to_dict(self) -> dict:
        return {
            "block_name": self.block_name,
            "kind": self.kind,
            "asset_filename": self.asset_filename,
            "mesh_filename": self.mesh_filename,
            "mesh_scale": list(self.mesh_scale),
            "preferred_robotic_tool_name": self.preferred_robotic_tool_name,
            "collision_filename": self.collision_filename,
            "bar_cradle": self.bar_cradle,
            # A JSON object: {label: [x, y, z]}.  Lists, not tuples -- `json`
            # writes both as arrays, but reading back always yields lists, and
            # `__post_init__` re-tuples them on the way in.
            "marker_points_mm": {
                label: list(point) for label, point in self.marker_points_mm.items()
            },
            "M_block_from_bar": self.M_block_from_bar.tolist(),
            "M_screw_from_block": self.M_screw_from_block.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "JointHalfDef":
        return cls(
            block_name=str(data["block_name"]),
            M_block_from_bar=np.asarray(data["M_block_from_bar"], dtype=float),
            M_screw_from_block=np.asarray(data["M_screw_from_block"], dtype=float),
            kind=str(data.get("kind", "")),
            asset_filename=str(data.get("asset_filename", "")),
            mesh_filename=str(data.get("mesh_filename", "")),
            mesh_scale=tuple(float(v) for v in data.get("mesh_scale", (1.0, 1.0, 1.0))),
            preferred_robotic_tool_name=str(data.get("preferred_robotic_tool_name", "")),
            collision_filename=str(data.get("collision_filename", "")),
            # Registries written before this flag existed simply omit it -> False.
            bar_cradle=bool(data.get("bar_cradle", False)),
            # Likewise absent from registries written before markers existed -> {}.
            # `__post_init__` does the float/length normalizing, so the raw dict
            # is handed straight through.
            marker_points_mm=dict(data.get("marker_points_mm", {})),
        )


@dataclass(frozen=True)
class GroundJointDef:
    """A joint half rigidly anchored to the world (no mate, no screw frame).

    A ground joint is the structural anchor for one end of a bar to the
    environment.  It exposes the same `(jp, jr)` DOFs as a regular joint
    half (slide along the bar, rotate about it), but has no `M_screw_from_block`
    because there is no mating partner.

    `M_tool_from_block` decouples the robotic tool's attach frame from the
    block frame.  It is needed because the two are governed by different
    constraints: `core.single_sided_placement.auto_jr_y_down` requires the block's
    local +Y to point at the ground (the foot must sit down), while the arm
    may have to approach with its TCP rolled relative to that.  Identity --
    the default, and what every male/female half does implicitly -- means the
    tool attaches on the block frame itself.
    """

    name: str
    block_name: str
    M_block_from_bar: np.ndarray
    asset_filename: str = ""
    collision_filename: str = ""
    jp_range: tuple[float, float] = DEFAULT_JP_RANGE
    jr_range: tuple[float, float] = DEFAULT_JR_RANGE
    # Constant BLOCK-LOCAL rotation from the ground block's frame to the frame
    # the robotic tool's TCP is attached at.  ROTATION ONLY: `__post_init__`
    # forces the translation to zero, so the TCP always stays on the block
    # origin and the 50 mm TCP probe in `core.joint_relink._tool_edit` is
    # unaffected.  Being block-local, it rides along with the `flip` operation
    # (`core.single_sided_placement.effective_M_block_from_bar`) automatically; note
    # that flip reverses block-local +Z, so a 180 deg roll about Z is exactly
    # invariant under a flip while a general angle reverses sense.
    M_tool_from_block: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self) -> None:
        if jnc.block_subtype(self.block_name) != jnc.GROUND:
            raise ValueError(
                f"ground joint block {self.block_name!r} must be named <Type>_Ground"
            )
        object.__setattr__(self, "M_block_from_bar", _as_4x4(self.M_block_from_bar))
        tool_from_block = _as_4x4(self.M_tool_from_block)
        tool_from_block[:3, 3] = 0.0
        object.__setattr__(self, "M_tool_from_block", tool_from_block)

    @property
    def type(self) -> str:
        """``"T20"`` -- the product family, which also goes into the joint id."""
        return jnc.block_type(self.block_name)

    def asset_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.asset_filename) if self.asset_filename else ""

    def collision_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.collision_filename) if self.collision_filename else ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "block_name": self.block_name,
            "asset_filename": self.asset_filename,
            "collision_filename": self.collision_filename,
            "jp_range": list(self.jp_range),
            "jr_range": list(self.jr_range),
            "M_block_from_bar": self.M_block_from_bar.tolist(),
            # Emitted even when identity so the knob is discoverable in the file.
            "M_tool_from_block": self.M_tool_from_block.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "GroundJointDef":
        return cls(
            name=str(data["name"]),
            block_name=str(data["block_name"]),
            M_block_from_bar=np.asarray(data["M_block_from_bar"], dtype=float),
            asset_filename=str(data.get("asset_filename", "")),
            collision_filename=str(data.get("collision_filename", "")),
            jp_range=tuple(float(v) for v in data.get("jp_range", DEFAULT_JP_RANGE)),
            jr_range=tuple(float(v) for v in data.get("jr_range", DEFAULT_JR_RANGE)),
            # Absent from registries written before the tool-attach frame existed
            # -> identity -> the historical "the tool attaches on the block frame".
            M_tool_from_block=np.asarray(
                data.get("M_tool_from_block", np.eye(4)), dtype=float
            ),
        )


@dataclass(frozen=True)
class JointPairDef:
    """A mate: a receiver half and a male half that screw together.

    Example -- the mate ``T20`` is ``T20_Female`` + ``T20_Male`` with
    ``contact_distance_mm = 36``: when the two halves are screwed together, the
    two bar axes are exactly 36 mm apart.  RSBarSnap / RSBarBrace use that
    distance to place a new bar; RSJointPlace solves both halves onto two bars.

    The receiving slot is named ``female`` because that is its name on disk
    (``female_block_name`` in the ``mates`` table).  It holds a Female, or --
    via :func:`with_receiver` -- the MoCap half of the same Type.  New code
    reads :attr:`receiver`.
    """

    name: str
    female: JointHalfDef
    male: JointHalfDef
    contact_distance_mm: float
    jp_range: tuple[float, float] = DEFAULT_JP_RANGE
    jr_range: tuple[float, float] = DEFAULT_JR_RANGE

    @property
    def receiver(self) -> JointHalfDef:
        """The half the male seats INTO -- female or mocap.

        The same object as :attr:`female`; this name is the one that stays true
        whichever kind of receiving half the mate carries.
        """
        return self.female

    @property
    def receiver_subtype(self) -> str:
        """``"Female"`` or ``"MoCap"`` -- decides the placed receiver's layer,
        object name and collision key."""
        return self.female.subtype

    def __post_init__(self) -> None:
        if self.female.subtype not in jnc.RECEIVER_SUBTYPES:
            raise ValueError(
                f"mate {self.name!r}: receiver {self.female.block_name!r} is not one "
                f"of {jnc.RECEIVER_SUBTYPES}"
            )
        if self.male.subtype != jnc.MALE:
            raise ValueError(
                f"mate {self.name!r}: {self.male.block_name!r} is not a Male half"
            )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "contact_distance_mm": float(self.contact_distance_mm),
            "jp_range": list(self.jp_range),
            "jr_range": list(self.jr_range),
            "female": self.female.to_dict(),
            "male": self.male.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "JointPairDef":
        return cls(
            name=str(data["name"]),
            female=JointHalfDef.from_dict(data["female"]),
            male=JointHalfDef.from_dict(data["male"]),
            contact_distance_mm=float(data["contact_distance_mm"]),
            jp_range=tuple(float(v) for v in data.get("jp_range", DEFAULT_JP_RANGE)),
            jr_range=tuple(float(v) for v in data.get("jr_range", DEFAULT_JR_RANGE)),
        )


# ---------------------------------------------------------------------------
# Canonical bar frame
# ---------------------------------------------------------------------------


def canonical_bar_frame_from_line(
    bar_start: Iterable[float], bar_end: Iterable[float]
) -> np.ndarray:
    """Return the canonical 4x4 bar frame.

    Origin = ``bar_start``, Z = unit(``bar_end - bar_start``),
    X = ``orthogonal_to(Z)`` (deterministic), Y = Z x X.
    """

    start = np.asarray(bar_start, dtype=float)
    end = np.asarray(bar_end, dtype=float)
    z_axis = unit(end - start)
    x_axis = orthogonal_to(z_axis)
    y_axis = unit(np.cross(z_axis, x_axis))
    return frame_from_axes(start, x_axis, y_axis, z_axis)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------


def fk_half_from_bar_frame(
    bar_frame: np.ndarray, jp: float, jr: float, half: JointHalfDef
) -> dict[str, np.ndarray]:
    bar_frame = np.asarray(bar_frame, dtype=float)
    block_frame = (
        bar_frame
        @ translation_transform((0.0, 0.0, float(jp)))
        @ rotation_about_local_z(float(jr))
        @ half.M_block_from_bar
    )
    screw_frame = block_frame @ half.M_screw_from_block
    return {
        "bar_frame": bar_frame,
        "block_frame": block_frame,
        "screw_frame": screw_frame,
    }


# ---------------------------------------------------------------------------
# Registry I/O
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Normalized registry on disk
# ---------------------------------------------------------------------------
# The on-disk schema is a 3-table normalized registry:
#
#   {
#     "halves":        [JointHalfDef.to_dict() ...],   # keyed by block_name
#     "mates":         [{name, contact_distance_mm, jp_range, jr_range,
#                        female_block_name, male_block_name}, ...],
#     "ground_joints": [GroundJointDef.to_dict() ...],  # keyed by name
#   }
#
# Each `JointHalfDef` appears exactly once in `halves` (deduplicated by
# `block_name`).  A `JointPairDef` is reconstructed by joining the two
# referenced halves into the mate entry.


@dataclass(frozen=True)
class JointRegistry:
    halves: dict[str, JointHalfDef] = field(default_factory=dict)
    mates: dict[str, JointPairDef] = field(default_factory=dict)
    ground_joints: dict[str, GroundJointDef] = field(default_factory=dict)


def _mate_to_dict(pair: JointPairDef) -> dict:
    return {
        "name": pair.name,
        "contact_distance_mm": float(pair.contact_distance_mm),
        "jp_range": list(pair.jp_range),
        "jr_range": list(pair.jr_range),
        "female_block_name": pair.female.block_name,
        "male_block_name": pair.male.block_name,
    }


def load_joint_registry(path: str = DEFAULT_REGISTRY_PATH) -> JointRegistry:
    if not os.path.exists(path):
        return JointRegistry()
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)

    halves: dict[str, JointHalfDef] = {}
    for entry in data.get("halves", []):
        half = JointHalfDef.from_dict(entry)
        halves[half.block_name] = half

    mates: dict[str, JointPairDef] = {}
    for entry in data.get("mates", []):
        name = str(entry["name"])
        fname = str(entry["female_block_name"])
        mname = str(entry["male_block_name"])
        if fname not in halves:
            raise KeyError(
                f"Mate {name!r} references unknown female half block_name={fname!r}"
            )
        if mname not in halves:
            raise KeyError(
                f"Mate {name!r} references unknown male half block_name={mname!r}"
            )
        mates[name] = JointPairDef(
            name=name,
            female=halves[fname],
            male=halves[mname],
            contact_distance_mm=float(entry["contact_distance_mm"]),
            jp_range=tuple(float(v) for v in entry.get("jp_range", DEFAULT_JP_RANGE)),
            jr_range=tuple(float(v) for v in entry.get("jr_range", DEFAULT_JR_RANGE)),
        )

    ground_joints: dict[str, GroundJointDef] = {}
    for entry in data.get("ground_joints", []):
        gj = GroundJointDef.from_dict(entry)
        ground_joints[gj.name] = gj

    return JointRegistry(halves=halves, mates=mates, ground_joints=ground_joints)


def save_joint_registry(
    registry: JointRegistry, path: str = DEFAULT_REGISTRY_PATH
) -> None:
    payload = {
        "halves": [registry.halves[k].to_dict() for k in sorted(registry.halves)],
        "mates": [_mate_to_dict(registry.mates[k]) for k in sorted(registry.mates)],
        "ground_joints": [
            registry.ground_joints[k].to_dict() for k in sorted(registry.ground_joints)
        ],
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def save_joint_half(
    half: JointHalfDef, path: str = DEFAULT_REGISTRY_PATH
) -> None:
    reg = load_joint_registry(path)
    reg.halves[half.block_name] = half
    save_joint_registry(reg, path)


def save_ground_joint(
    ground: GroundJointDef, path: str = DEFAULT_REGISTRY_PATH
) -> None:
    reg = load_joint_registry(path)
    reg.ground_joints[ground.name] = ground
    save_joint_registry(reg, path)


# ---------------------------------------------------------------------------
# Mate-centric API (back-compat: same callers as before the split)
# ---------------------------------------------------------------------------


def load_joint_pairs(path: str = DEFAULT_REGISTRY_PATH) -> dict[str, JointPairDef]:
    return load_joint_registry(path).mates


def save_joint_pair(
    pair: JointPairDef,
    path: str = DEFAULT_REGISTRY_PATH,
    *,
    overwrite_halves: bool = True,
) -> None:
    """Insert/overwrite a mate entry; also upsert its two halves.

    When `overwrite_halves` is True (default), the female/male halves carried
    by `pair` overwrite any existing halves with the same `block_name`.  Set
    to False to preserve the existing half geometry (useful when the caller
    only wants to update mate-level fields like `contact_distance_mm`).
    """
    reg = load_joint_registry(path)
    for half in (pair.female, pair.male):
        if overwrite_halves or half.block_name not in reg.halves:
            reg.halves[half.block_name] = half
    reg.mates[pair.name] = pair
    save_joint_registry(reg, path)


def get_joint_pair(
    name: str, *, path: str = DEFAULT_REGISTRY_PATH
) -> JointPairDef:
    pairs = load_joint_pairs(path)
    if name not in pairs:
        raise KeyError(f"Joint pair {name!r} not found in {path}.")
    return pairs[name]


def list_joint_pair_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_joint_pairs(path).keys())


# ---------------------------------------------------------------------------
# Receiver variants: a MoCap half is placed through its Type's Female mate
# ---------------------------------------------------------------------------
# ``T20_MoCap`` is ``T20_Female`` with a marker plate on its back, so it has no
# mate of its own: placing a T20 joint with a MoCap receiver uses mate ``T20``
# with ``T20_MoCap`` swapped into the receiving slot.  The solver then uses the
# MoCap block's own matrices, so this is right even if the plate changes how
# the block sits on the bar.


def receiver_subtypes(pair: JointPairDef, halves: dict) -> list:
    """Receivers registered for *pair*'s Type, Female first.

    ``["Female"]`` today; ``["Female", "MoCap"]`` once ``T20_MoCap`` is
    registered.  RSJointPlace asks which one only when there is a choice.
    """
    type_ = pair.receiver.type
    return [s for s in jnc.RECEIVER_SUBTYPES if jnc.block_name(type_, s) in halves]


def with_receiver(pair: JointPairDef, subtype: str, halves: dict) -> JointPairDef:
    """*pair* with its Type's *subtype* half in the receiving slot.

    ``with_receiver(T20, "MoCap", halves)`` -> mate ``T20`` holding
    ``T20_MoCap`` + ``T20_Male``.  Returns *pair* unchanged when it already
    holds that receiver.  Raises ``KeyError`` when the block is not registered.
    """
    if pair.receiver_subtype == subtype:
        return pair
    name = jnc.block_name(pair.receiver.type, subtype)
    if name not in halves:
        raise KeyError(
            f"mate {pair.name!r} has no registered {subtype} receiver {name!r}"
        )
    return JointPairDef(
        name=pair.name,
        female=halves[name],
        male=pair.male,
        contact_distance_mm=pair.contact_distance_mm,
        jp_range=pair.jp_range,
        jr_range=pair.jr_range,
    )


def get_joint_pair_variant(
    name: str, receiver_subtype: str | None = None, *, path: str = DEFAULT_REGISTRY_PATH
) -> JointPairDef:
    """Mate *name*, with its *receiver_subtype* half swapped in when given.

    What RSJointEdit uses to rebuild a placed pair: the mate name comes from
    the block's ``joint_pair_name`` user text, the receiver Subtype from the
    layer its receiver block sits on.
    """
    registry = load_joint_registry(path)
    if name not in registry.mates:
        raise KeyError(f"Joint pair {name!r} not found in {path}.")
    pair = registry.mates[name]
    if receiver_subtype is None:
        return pair
    return with_receiver(pair, receiver_subtype, registry.halves)


def list_joint_half_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_joint_registry(path).halves.keys())


def list_ground_joint_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_joint_registry(path).ground_joints.keys())
