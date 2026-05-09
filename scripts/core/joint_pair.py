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


VALID_HALF_KINDS = ("male", "female")


@dataclass(frozen=True)
class JointHalfDef:
    """Constant geometry for one half of a joint pair."""

    block_name: str
    M_block_from_bar: np.ndarray
    M_screw_from_block: np.ndarray
    kind: str = "male"                 # "male" or "female"
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

    def __post_init__(self) -> None:
        object.__setattr__(self, "M_block_from_bar", _as_4x4(self.M_block_from_bar))
        object.__setattr__(self, "M_screw_from_block", _as_4x4(self.M_screw_from_block))
        if self.kind not in VALID_HALF_KINDS:
            raise ValueError(
                f"JointHalfDef.kind must be one of {VALID_HALF_KINDS}, got {self.kind!r}"
            )

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
            "M_block_from_bar": self.M_block_from_bar.tolist(),
            "M_screw_from_block": self.M_screw_from_block.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "JointHalfDef":
        return cls(
            block_name=str(data["block_name"]),
            M_block_from_bar=np.asarray(data["M_block_from_bar"], dtype=float),
            M_screw_from_block=np.asarray(data["M_screw_from_block"], dtype=float),
            kind=str(data.get("kind", "male")),
            asset_filename=str(data.get("asset_filename", "")),
            mesh_filename=str(data.get("mesh_filename", "")),
            mesh_scale=tuple(float(v) for v in data.get("mesh_scale", (1.0, 1.0, 1.0))),
            preferred_robotic_tool_name=str(data.get("preferred_robotic_tool_name", "")),
            collision_filename=str(data.get("collision_filename", "")),
        )


@dataclass(frozen=True)
class GroundJointDef:
    """A joint half rigidly anchored to the world (no mate, no screw frame).

    A ground joint is the structural anchor for one end of a bar to the
    environment.  It exposes the same `(jp, jr)` DOFs as a regular joint
    half (slide along the bar, rotate about it), but has no `M_screw_from_block`
    because there is no mating partner.
    """

    name: str
    block_name: str
    M_block_from_bar: np.ndarray
    asset_filename: str = ""
    collision_filename: str = ""
    jp_range: tuple[float, float] = DEFAULT_JP_RANGE
    jr_range: tuple[float, float] = DEFAULT_JR_RANGE

    def __post_init__(self) -> None:
        object.__setattr__(self, "M_block_from_bar", _as_4x4(self.M_block_from_bar))

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
        )


@dataclass(frozen=True)
class JointPairDef:
    """A pair of joint halves (female + male) with shared screw axis at mate."""

    name: str
    female: JointHalfDef
    male: JointHalfDef
    contact_distance_mm: float
    jp_range: tuple[float, float] = DEFAULT_JP_RANGE
    jr_range: tuple[float, float] = DEFAULT_JR_RANGE

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


def list_joint_half_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_joint_registry(path).halves.keys())


def list_ground_joint_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_joint_registry(path).ground_joints.keys())
