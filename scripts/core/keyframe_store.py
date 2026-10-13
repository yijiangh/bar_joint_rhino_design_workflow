"""The keyframes Rhino keeps on bar curves as user text: the one persistent store of solved IK.

Cindy's assembly keyframe (`config.KEY_ASSEMBLY_*`: base frame, approach / assembled / retreat joints per arm) and a
support robot's hold (`config.KEY_SUPPORT_*`: robot, base frame, grasp frame, approach / held joints) live on the bar
curve. `core.design_source` reads them into the design on every command; the IK commands write them back.

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


#: Farthest a solved base can stand from its bar, measured in the base's ground plane: the arm's reach plus the
#: arm mount's offset from the base origin, the tool and the joint (generous: solved bars measure under 1 m).
STALE_BASE_DISTANCE_MM = config.ARM_REACH_RADIUS_MM + 650.0


def base_to_bar_distance_mm(base_frame_world_mm, bar_start_mm, bar_end_mm) -> float:
    """How far the bar is from the base origin, in the base's ground plane (its xy): mm to the nearest point of the
    bar's segment. Rhino-free."""
    base = np.asarray(base_frame_world_mm, dtype=float)
    a, b = ((base[:3, :3].T @ (np.asarray(p, dtype=float) - base[:3, 3]))[:2] for p in (bar_start_mm, bar_end_mm))
    along = b - a
    length_sq = float(along @ along)
    t = 0.0 if length_sq <= 1e-12 else min(1.0, max(0.0, float(-a @ along) / length_sq))
    return float(np.linalg.norm(a + t * along))


def stale_ik_reason(bar_oid) -> Optional[str]:
    """Why the bar's assembly keyframe cannot belong to where the bar is now, or None (also None without one).

    A bar moved, rotated or copied after its solve keeps its keyframe: the base frame and the joints are world
    values in user text, and nothing moves them along. The test is the base standing out of reach of the bar.

    - ? It catches large moves only (metres, as after copying a module). A bar nudged within reach passes; the
      plan check's B14 at export measures every keyframe exactly.
    """
    keyframe = read_assembly_keyframe(bar_oid)
    if keyframe is None:
        return None
    from core.rhino_helpers import curve_endpoints, doc_unit_scale_to_mm  # noqa: PLC0415

    scale = doc_unit_scale_to_mm()
    start, end = (p * scale for p in curve_endpoints(bar_oid))
    distance = base_to_bar_distance_mm(keyframe["base_frame_world_mm"], start, end)
    if distance <= STALE_BASE_DISTANCE_MM:
        return None
    return (f"the saved robot base stands {distance / 1000.0:.1f} m from the bar (the arms reach about "
            f"{STALE_BASE_DISTANCE_MM / 1000.0:.1f} m): the bar was moved or copied after its IK was solved")


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

    The hold's linear approach starts here and the release's retreat ends here (`design_builder` uses the same line).
    """
    out = np.array(tool0_world_mm, dtype=float, copy=True)
    out[:3, 3] -= float(config.SUPPORT_LM_DISTANCE_MM) * out[:3, 2]
    return out
