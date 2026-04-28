"""Robotic-tool definitions: data model + JSON registry.

Mirrors :mod:`core.joint_pair` but for the robotic end-effectors that
tighten the male joint onto the female joint.

A robotic tool has:
  - ``block_name``        : the Rhino InstanceDefinition name of its
                            collision-mesh geometry, baked at the robot
                            **flange (tool-zero) frame**.  Inserting the
                            block at the FK result of the robot's flange
                            puts the tool in the right pose automatically.
  - ``M_tcp_from_block``  : 4x4 transform from the block's local frame to
                            the **TCP frame** (the frame of the male joint
                            once held by the tool).  This is the offset the
                            motion planner uses: given a desired world
                            ``M_tcp`` they will pose the block so that the
                            block-local TCP coincides with ``M_tcp``.

All distances are in millimetres, all angles in radians.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass

import numpy as np

from core.transforms import orthonormalize_rotation


_CORE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(_CORE_DIR))
DEFAULT_REGISTRY_PATH = os.path.join(_CORE_DIR, "robotic_tools.json")
DEFAULT_ASSET_DIR = os.path.join(REPO_DIR, "asset")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _as_4x4(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")
    out = np.array(matrix, dtype=float, copy=True)
    out[:3, :3] = orthonormalize_rotation(out[:3, :3])
    out[3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return out


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RoboticToolDef:
    """One robotic tool definition.

    Attributes
    ----------
    name : str
        Unique tool name (registry key).
    block_name : str
        Rhino InstanceDefinition name to insert.
    M_tcp_from_block : (4, 4) ndarray
        Transform expressing the TCP frame in the block's local frame.
        ``world_tcp = world_block @ M_tcp_from_block``.
    asset_filename : str
        Filename of the exported .3dm under :data:`DEFAULT_ASSET_DIR`.
    mesh_filename, mesh_scale : str, tuple
        Optional URDF/visualization mesh metadata, mirroring
        :class:`core.joint_pair.JointHalfDef`.
    """

    name: str
    block_name: str
    M_tcp_from_block: np.ndarray
    asset_filename: str = ""
    mesh_filename: str = ""
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "M_tcp_from_block", _as_4x4(self.M_tcp_from_block))

    def asset_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.asset_filename) if self.asset_filename else ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "block_name": self.block_name,
            "asset_filename": self.asset_filename,
            "mesh_filename": self.mesh_filename,
            "mesh_scale": list(self.mesh_scale),
            "M_tcp_from_block": self.M_tcp_from_block.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "RoboticToolDef":
        return cls(
            name=str(data["name"]),
            block_name=str(data["block_name"]),
            M_tcp_from_block=np.asarray(data["M_tcp_from_block"], dtype=float),
            asset_filename=str(data.get("asset_filename", "")),
            mesh_filename=str(data.get("mesh_filename", "")),
            mesh_scale=tuple(float(v) for v in data.get("mesh_scale", (1.0, 1.0, 1.0))),
        )


# ---------------------------------------------------------------------------
# JSON registry
# ---------------------------------------------------------------------------


def load_robotic_tools(path: str = DEFAULT_REGISTRY_PATH) -> dict[str, RoboticToolDef]:
    """Return ``{tool_name: RoboticToolDef}`` from the JSON registry.

    Returns an empty dict if the file does not exist yet.
    """
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    tools: dict[str, RoboticToolDef] = {}
    for entry in data.get("tools", []):
        tool = RoboticToolDef.from_dict(entry)
        tools[tool.name] = tool
    return tools


def save_robotic_tool(tool: RoboticToolDef, path: str = DEFAULT_REGISTRY_PATH) -> None:
    """Insert/overwrite a single tool entry in the JSON registry."""
    tools = load_robotic_tools(path)
    tools[tool.name] = tool
    payload = {"tools": [tools[name].to_dict() for name in sorted(tools)]}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def get_robotic_tool(
    name: str, *, path: str = DEFAULT_REGISTRY_PATH
) -> RoboticToolDef:
    tools = load_robotic_tools(path)
    if name not in tools:
        raise KeyError(f"Robotic tool {name!r} not found in {path}.")
    return tools[name]


def list_robotic_tool_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_robotic_tools(path).keys())
