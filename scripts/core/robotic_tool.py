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

Naming rule: an assembly tool name always ends in ``L`` (left arm) or ``R``
(right arm), and the two sides of one physical candidate share a prefix
(``AT4L`` / ``AT4R``).  :func:`arm_side_from_tool_name` is the single home
of that rule.

The registry JSON also stores which candidate pair is currently *active*:

    {"active": {"left": "AT3L", "right": "AT3R"}, "tools": [...]}

Everything kinematic (collision cell, placement defaults, tool cycling)
resolves tools through :func:`get_active_pair`; RSSwapRoboticTool is the
button that changes it.

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


def arm_side_from_tool_name(tool_name: str) -> str | None:
    """Classify an arm side from a tool name's L/R suffix.

    This is THE single home of the naming rule (previously copy-pasted in
    ``robot_cell`` / ``ik_collision_setup`` / ``rs_ik_keyframe`` /
    ``bar_action``; they all import from here now).

    Args:
        tool_name (str): registry tool name, e.g. ``"AT3L"`` / ``"AT3R"``.

    Returns:
        str | None: ``"left"`` for an 'L' suffix, ``"right"`` for 'R',
        ``None`` when the name is empty or has no L/R suffix.
    """
    if not tool_name:
        return None
    stripped = str(tool_name).strip()
    if not stripped:
        return None
    last = stripped[-1].upper()
    if last == "L":
        return "left"
    if last == "R":
        return "right"
    return None


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
    # OBJ filename (under DEFAULT_ASSET_DIR) used as the per-tool collision
    # mesh attached as a `compas_fab.robots.RigidBody` to the arm's tool0
    # link in the IK keyframe workflow. Must be exported with the block's
    # local origin coincident with the robot flange (tool0). Empty string
    # disables the rigid-body attach for that tool.
    collision_filename: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "M_tcp_from_block", _as_4x4(self.M_tcp_from_block))

    def asset_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.asset_filename) if self.asset_filename else ""

    def collision_path(self, asset_dir: str = DEFAULT_ASSET_DIR) -> str:
        return os.path.join(asset_dir, self.collision_filename) if self.collision_filename else ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "block_name": self.block_name,
            "asset_filename": self.asset_filename,
            "mesh_filename": self.mesh_filename,
            "mesh_scale": list(self.mesh_scale),
            "collision_filename": self.collision_filename,
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
            collision_filename=str(data.get("collision_filename", "")),
        )


# ---------------------------------------------------------------------------
# JSON registry
# ---------------------------------------------------------------------------


def _load_registry_payload(path: str = DEFAULT_REGISTRY_PATH) -> dict:
    """Read the raw registry JSON payload (``{}`` if the file is missing)."""
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _save_registry_payload(payload: dict, path: str = DEFAULT_REGISTRY_PATH) -> None:
    """Write the raw registry JSON payload with stable formatting."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def load_robotic_tools(path: str = DEFAULT_REGISTRY_PATH) -> dict[str, RoboticToolDef]:
    """Return ``{tool_name: RoboticToolDef}`` from the JSON registry.

    Returns an empty dict if the file does not exist yet.
    """
    payload = _load_registry_payload(path)
    tools: dict[str, RoboticToolDef] = {}
    for entry in payload.get("tools", []):
        tool = RoboticToolDef.from_dict(entry)
        tools[tool.name] = tool
    return tools


def save_robotic_tool(tool: RoboticToolDef, path: str = DEFAULT_REGISTRY_PATH) -> None:
    """Insert/overwrite a single tool entry in the JSON registry.

    Tool names are unique registry keys. Saving an existing exact name replaces
    its definition, and rebuilding from the name-keyed mapping also collapses
    duplicate-name entries that may exist in an older hand-edited file.

    Only the ``tools`` list is rebuilt; every other top-level key in the
    payload (most importantly ``active``, the active L/R pair) is kept.
    """
    payload = _load_registry_payload(path)
    tools = load_robotic_tools(path)
    tools[tool.name] = tool
    payload["tools"] = [tools[name].to_dict() for name in sorted(tools)]
    _save_registry_payload(payload, path)


def get_robotic_tool(
    name: str, *, path: str = DEFAULT_REGISTRY_PATH
) -> RoboticToolDef:
    tools = load_robotic_tools(path)
    if name not in tools:
        raise KeyError(f"Robotic tool {name!r} not found in {path}.")
    return tools[name]


def list_robotic_tool_names(path: str = DEFAULT_REGISTRY_PATH) -> list[str]:
    return sorted(load_robotic_tools(path).keys())


# ---------------------------------------------------------------------------
# Active tool pair (which L/R candidate the whole workflow currently uses)
# ---------------------------------------------------------------------------


def resolve_pair_for_tool(
    tool_name: str, tools: dict[str, RoboticToolDef]
) -> dict[str, RoboticToolDef]:
    """Resolve the full left/right pair from either member's name.

    The pair convention is a shared prefix plus an L/R suffix: given
    ``"AT4L"`` this returns ``{"left": AT4L_def, "right": AT4R_def}``
    (and the same result for ``"AT4R"``).

    Args:
        tool_name (str): name of either pair member, e.g. ``"AT4L"``.
        tools (dict): registry contents from :func:`load_robotic_tools`.

    Returns:
        dict: ``{"left": RoboticToolDef, "right": RoboticToolDef}``.

    Raises:
        ValueError: if the name has no L/R suffix, is not registered, or
            its partner is not registered -- with instructions on the fix.
    """
    side = arm_side_from_tool_name(tool_name)
    if side is None:
        raise ValueError(
            f"Tool name {tool_name!r} has no L/R suffix. Assembly tools must "
            "be named <prefix>L / <prefix>R (e.g. AT4L / AT4R); the suffix "
            "selects the arm side."
        )
    if tool_name not in tools:
        raise ValueError(
            f"Tool {tool_name!r} is not in the registry. Define it first with "
            "RSDefineRoboticTool (AssemblyTool mode)."
        )
    # Build the partner's name with the opposite suffix, keeping the letter
    # case of the original suffix (AT4L -> AT4R, at4l -> at4r).
    partner_letter = "R" if side == "left" else "L"
    if tool_name[-1].islower():
        partner_letter = partner_letter.lower()
    partner_name = tool_name[:-1] + partner_letter
    if partner_name not in tools:
        raise ValueError(
            f"Pair partner {partner_name!r} for tool {tool_name!r} is not in "
            f"the registry. Model + define {partner_name!r} first with "
            "RSDefineRoboticTool (AssemblyTool mode) -- both sides of a "
            "candidate pair must exist before it can be activated."
        )
    other_side = "right" if side == "left" else "left"
    return {side: tools[tool_name], other_side: tools[partner_name]}


def get_active_pair_names(path: str = DEFAULT_REGISTRY_PATH) -> dict[str, str]:
    """Return the active pair's tool names: ``{"left": name, "right": name}``.

    The ``active`` key in ``robotic_tools.json`` is the source of truth.
    If the key is absent (registry file from before the active-pair schema)
    and the registry holds exactly one L-suffix and one R-suffix tool, that
    only possible pair is used (with an info print).  Anything else raises.

    Raises:
        RuntimeError: empty registry, missing/invalid ``active`` entry, or
            an ambiguous registry with no ``active`` entry -- each message
            names the command to run to fix it.
    """
    payload = _load_registry_payload(path)
    tools = load_robotic_tools(path)
    if not tools:
        raise RuntimeError(
            f"No robotic tools registered in {path}. Define a left+right tool "
            "pair with RSDefineRoboticTool (AssemblyTool mode) first."
        )

    active = payload.get("active")
    if active is not None:
        names: dict[str, str] = {}
        for side in ("left", "right"):
            name = active.get(side) if isinstance(active, dict) else None
            if not name or name not in tools:
                raise RuntimeError(
                    f"robotic_tools.json 'active' entry is invalid for side "
                    f"'{side}' (got {name!r}). Run RSSwapRoboticTool and type "
                    "a registered tool name to pick a valid active pair."
                )
            if arm_side_from_tool_name(name) != side:
                raise RuntimeError(
                    f"robotic_tools.json 'active' {side} tool {name!r} does "
                    f"not carry the matching "
                    f"'{'L' if side == 'left' else 'R'}' name suffix. Run "
                    "RSSwapRoboticTool to pick a valid active pair."
                )
            names[side] = str(name)
        return names

    # ! No 'active' key: only acceptable when the registry has exactly one
    # ! possible pair (a file from before this schema). Anything ambiguous
    # ! must raise -- we never silently guess which candidate the user means.
    by_side: dict[str, list[str]] = {"left": [], "right": []}
    for name in tools:
        side = arm_side_from_tool_name(name)
        if side in by_side:
            by_side[side].append(name)
    if len(by_side["left"]) == 1 and len(by_side["right"]) == 1:
        names = {"left": by_side["left"][0], "right": by_side["right"][0]}
        print(
            "robotic_tools.json has no 'active' entry; using the only "
            f"registered pair {names['left']!r} + {names['right']!r}. "
            "Run RSSwapRoboticTool once to record this explicitly."
        )
        return names
    raise RuntimeError(
        "robotic_tools.json has no 'active' entry and more than one candidate "
        f"per side (left={sorted(by_side['left'])}, "
        f"right={sorted(by_side['right'])}). Run RSSwapRoboticTool and type "
        "a registered tool name to pick the active pair."
    )


def get_active_pair(path: str = DEFAULT_REGISTRY_PATH) -> dict[str, RoboticToolDef]:
    """Return the active pair's definitions: ``{"left": def, "right": def}``.

    Same resolution rules (and errors) as :func:`get_active_pair_names`.
    """
    names = get_active_pair_names(path)
    tools = load_robotic_tools(path)
    return {side: tools[name] for side, name in names.items()}


def set_active_pair(
    left_name: str, right_name: str, path: str = DEFAULT_REGISTRY_PATH
) -> None:
    """Persist the active pair into the registry's ``active`` key.

    Args:
        left_name (str): registry name of the left-arm tool (must end in L).
        right_name (str): registry name of the right-arm tool (must end in R).
        path (str): registry path (tests override this).

    Raises:
        ValueError: unknown names, or a name whose L/R suffix does not match
            the side it is assigned to.
    """
    tools = load_robotic_tools(path)
    for side, name in (("left", left_name), ("right", right_name)):
        if name not in tools:
            raise ValueError(
                f"Cannot activate unknown tool {name!r} (registered: "
                f"{sorted(tools)}). Define it with RSDefineRoboticTool "
                "(AssemblyTool mode) first."
            )
        if arm_side_from_tool_name(name) != side:
            raise ValueError(
                f"Tool {name!r} cannot be the {side}-arm tool: its name "
                f"suffix must be '{'L' if side == 'left' else 'R'}'."
            )
    payload = _load_registry_payload(path)
    payload["active"] = {"left": str(left_name), "right": str(right_name)}
    _save_registry_payload(payload, path)
