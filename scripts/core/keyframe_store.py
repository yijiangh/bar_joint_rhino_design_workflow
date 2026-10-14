"""The keyframes Rhino keeps on bar curves as user text: the one persistent store of solved IK.

Cindy's assembly keyframe (`config.KEY_ASSEMBLY_*`: base frame, approach / assembled / retreat joints per arm) and a
support robot's hold (`config.KEY_SUPPORT_*`: robot, base frame, grasp frame, approach / held joints) live on the bar
curve. `core.process_source` reads them into the process on every command; the IK commands write them back.

- ! Frames are 4x4 world matrices in mm; joint records are `{"joint_names": [...], "joint_values": [...]}` (per arm
  `{"left": ..., "right": ...}` for Cindy).
- Needs numpy and Rhino only (rhinoscriptsyntax is imported where used).
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np

from core import config


def _rs():
    import rhinoscriptsyntax as rs
    return rs


# --- --- --- --- --- CINDY: ASSEMBLY KEYFRAME --- --- --- --- ---

def read_assembly_keyframe(bar_oid):
    """The bar's assembly keyframe, or None if the base, approach or assembled record is missing or broken.

    Returns:
        dict | None: `{"base_frame_world_mm": (4, 4) array, "approach": {...}, "assembled": {...},
        "retreat": {...} | None}`; `retreat` is None for bars solved before it was saved.
    """
    rs = _rs()
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    retreat_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT)
    if not base_raw or not approach_raw or not assembled_raw:
        return None
    try:
        base_mm = np.asarray(json.loads(base_raw), dtype=float)
        if base_mm.shape != (4, 4):
            return None
        approach = json.loads(approach_raw)
        assembled = json.loads(assembled_raw)
        retreat = json.loads(retreat_raw) if retreat_raw else None
    except ValueError:
        return None
    return {"base_frame_world_mm": base_mm, "approach": approach, "assembled": assembled, "retreat": retreat}


def has_ik_keyframe(bar_oid) -> bool:
    """Whether the bar carries a complete, well-formed assembly keyframe: the one test of "is this bar IK-solved"."""
    return read_assembly_keyframe(bar_oid) is not None


def load_assembly_payload(bar_oid) -> dict:
    """The assembled keyframe as the flat left/right payload the support flow anchors to.

    Returns:
        dict: `{"base_frame_world_mm": (4, 4) array, "joint_values_left", "joint_values_right", "joint_names_left",
        "joint_names_right"}`.

    Raises:
        RuntimeError: When the bar has no solved assembly keyframe: solve it first.
    """
    rs = _rs()
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not assembled_raw:
        bar_id = rs.GetUserText(bar_oid, "bar_id") or str(bar_oid)
        raise RuntimeError(
            f"Bar {bar_id!r} has no solved assembly keyframe "
            f"({config.KEY_ASSEMBLY_BASE_FRAME} / {config.KEY_ASSEMBLY_IK_ASSEMBLED}). "
            "Run RSIKKeyframe's assembly flow on it first.")
    assembled = json.loads(assembled_raw)
    return {
        "base_frame_world_mm": np.asarray(json.loads(base_raw), dtype=float),
        "joint_values_left": assembled["left"]["joint_values"],
        "joint_values_right": assembled["right"]["joint_values"],
        "joint_names_left": assembled["left"]["joint_names"],
        "joint_names_right": assembled["right"]["joint_names"],
    }


def write_assembly_base_frame(bar_oid, base_frame_world_mm) -> None:
    """Save Cindy's base frame for a bar (written as soon as it is picked, before the solve)."""
    _rs().SetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME,
                      json.dumps(np.asarray(base_frame_world_mm, dtype=float).tolist()))


def write_assembly_keyframes(bar_oid, approach: dict, assembled: dict, retreat: Optional[dict]) -> None:
    """Save the approach, assembled and retreat joints, each `{"left": {...}, "right": {...}}`; retreat None keeps
    the stored one."""
    rs = _rs()
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH, json.dumps(approach))
    rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED, json.dumps(assembled))
    if retreat is not None:
        rs.SetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT, json.dumps(retreat))


def read_assembly_base_frame(bar_oid):
    """Cindy's saved base frame for a bar as a (4, 4) mm array, or None if missing or broken."""
    raw = _rs().GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    if not raw:
        return None
    try:
        matrix = np.asarray(json.loads(raw), dtype=float)
    except ValueError:
        return None
    return matrix if matrix.shape == (4, 4) else None


# --- --- --- --- --- SUPPORT ROBOTS: HOLD KEYFRAME (all or nothing) --- --- --- --- ---

#: The config names of the keys that together form one complete support keyframe.
SUPPORT_KEYS = ("KEY_SUPPORT_ROBOT", "KEY_SUPPORT_BASE_FRAME", "KEY_SUPPORT_GRASP_FRAME", "KEY_SUPPORT_IK_APPROACH",
                "KEY_SUPPORT_IK_HELD")


def read_bar_support_keyframe(bar_oid):
    """The held bar's complete support keyframe, or None when no support key is present.

    Returns:
        dict | None: `{"robot_name", "base_frame_world_mm", "grasp_frame_world_mm", "approach": {...},
        "held": {...}}`.

    Raises:
        RuntimeError: When only some keys are present (the hold must be re-solved), naming the missing ones.
    """
    rs = _rs()
    raw = {name: rs.GetUserText(bar_oid, getattr(config, name)) for name in SUPPORT_KEYS}
    if not any(raw.values()):
        return None
    missing = [getattr(config, name) for name, value in raw.items() if not value]
    if missing:
        bar_id = rs.GetUserText(bar_oid, "bar_id") or str(bar_oid)
        raise RuntimeError(f"Bar {bar_id!r} has a PARTIAL support keyframe (missing {missing}). "
                           "Re-run RSIKKeyframe's support flow on it to write a complete one.")
    return {
        "robot_name": raw["KEY_SUPPORT_ROBOT"],
        "base_frame_world_mm": np.asarray(json.loads(raw["KEY_SUPPORT_BASE_FRAME"]), dtype=float),
        "grasp_frame_world_mm": np.asarray(json.loads(raw["KEY_SUPPORT_GRASP_FRAME"]), dtype=float),
        "approach": json.loads(raw["KEY_SUPPORT_IK_APPROACH"]),
        "held": json.loads(raw["KEY_SUPPORT_IK_HELD"]),
    }


def write_support_grasp_frame(bar_oid, grasp_frame_world_mm) -> None:
    """Save the picked grasp frame (written as soon as it is picked, before the solve)."""
    _rs().SetUserText(bar_oid, config.KEY_SUPPORT_GRASP_FRAME,
                      json.dumps(np.asarray(grasp_frame_world_mm, dtype=float).tolist()))


def write_support_base_frame(bar_oid, base_frame_world_mm) -> None:
    """Save the support robot's base frame (written as soon as it is picked, before the solve)."""
    _rs().SetUserText(bar_oid, config.KEY_SUPPORT_BASE_FRAME,
                      json.dumps(np.asarray(base_frame_world_mm, dtype=float).tolist()))


def write_bar_support_keyframe(bar_oid, robot_name: str, base_frame_world_mm, grasp_frame_world_mm,
                               approach_cfg: dict, held_cfg: dict) -> None:
    """Save a complete support keyframe on the held bar.

    Args:
        bar_oid: The held bar's curve.
        robot_name: "Alice" or "Belle": whose URDF solved it.
        base_frame_world_mm, grasp_frame_world_mm: 4x4 world frames, mm.
        approach_cfg, held_cfg: `{"joint_names", "joint_values"}`.
    """
    rs = _rs()
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_ROBOT, robot_name)
    write_support_base_frame(bar_oid, base_frame_world_mm)
    write_support_grasp_frame(bar_oid, grasp_frame_world_mm)
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_IK_APPROACH, json.dumps(approach_cfg))
    rs.SetUserText(bar_oid, config.KEY_SUPPORT_IK_HELD, json.dumps(held_cfg))


def approach_tool0_from_grasp(tool0_world_mm) -> np.ndarray:
    """The support flange's approach pose: the grasp pose backed off along its own −z by `SUPPORT_LM_DISTANCE_MM`.

    The hold's linear approach starts here and the release's retreat ends here (`process_builder` uses the same line).
    """
    out = np.array(tool0_world_mm, dtype=float, copy=True)
    out[:3, 3] -= float(config.SUPPORT_LM_DISTANCE_MM) * out[:3, 2]
    return out
