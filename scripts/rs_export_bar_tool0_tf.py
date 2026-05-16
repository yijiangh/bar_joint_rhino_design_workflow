#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSExportBarTool0TF - Export bar-OCF -> tool0 transforms for one bar.

Pick a bar curve. If the bar has both male joints placed AND both arm
tools placed (i.e. ``resolve_arm_tools_on_bar`` succeeds), this writes a
JSON file containing the world frames and the bar-local transforms to
each arm's ``tool0``:

    T_bar_to_tool0_<side>_mm = inv(bar_world_mm) @ tool0_<side>_world_mm

Bar OCF: origin at bar start, Z along bar axis (see
``core.env_collision._bar_world_frame_mm``). Tool0 world: block instance
world xform of the placed arm tool (see
``core.env_collision._block_instance_xform_mm``). All translations are
in millimeters.

Output: ``<root>/BarTool0TF/<bar_id>.json``. Root is the same sticky
folder used by ``RSExportBarAction``.
"""

from __future__ import annotations

import importlib
import json
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import env_collision as _env_collision_module
from core import ik_collision_setup as _ik_collision_setup_module
from core.rhino_bar_pick import pick_bar
from core.rhino_bar_registry import BAR_ID_KEY, repair_on_entry
from core.transforms import invert_transform


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"
TITLE = "RSExportBarTool0TF"


def _prompt_export_root() -> str | None:
    last = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
    chosen = rs.BrowseForFolder(
        folder=last if last and os.path.isdir(last) else None,
        message="Select export root folder",
        title=TITLE,
    )
    if not chosen:
        return None
    sc.sticky[EXPORT_ROOT_STICKY_KEY] = chosen
    return chosen


def _mat_to_list(m) -> list:
    return [[float(v) for v in row] for row in np.asarray(m, dtype=float)]


def main() -> None:
    config = importlib.reload(_config_module)
    env_collision = importlib.reload(_env_collision_module)
    ik_collision_setup = importlib.reload(_ik_collision_setup_module)

    repair_on_entry(float(config.BAR_RADIUS), TITLE)

    rs.UnselectAllObjects()
    bar_oid = pick_bar(
        "Pick a bar to export its OCF->tool0 transforms (Esc to cancel)"
    )
    if bar_oid is None:
        return
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)
    if not bar_id:
        rs.MessageBox(
            "Picked curve is not a registered bar (no 'bar_id' user-text).",
            0,
            TITLE,
        )
        return

    arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(bar_id)
    if err is not None:
        rs.MessageBox(
            f"Bar '{bar_id}' is not fully placed: {err}",
            0,
            TITLE,
        )
        return

    length_mm, bar_world_mm = env_collision._bar_world_frame_mm(bar_oid)
    if length_mm <= 0.0:
        rs.MessageBox(
            f"Bar '{bar_id}' has zero length; cannot build OCF.", 0, TITLE,
        )
        return

    tool0_left_world_mm = env_collision._block_instance_xform_mm(arm_tools["left"])
    tool0_right_world_mm = env_collision._block_instance_xform_mm(arm_tools["right"])

    inv_bar = invert_transform(bar_world_mm)
    bar_to_tool0_left_mm = inv_bar @ tool0_left_world_mm
    bar_to_tool0_right_mm = inv_bar @ tool0_right_world_mm

    left_tool_name = rs.GetUserText(arm_tools["left"], "tool_name") or ""
    right_tool_name = rs.GetUserText(arm_tools["right"], "tool_name") or ""

    payload = {
        "bar_id": bar_id,
        "units": "mm",
        "bar_length_mm": float(length_mm),
        "bar_ocf": {
            "origin": "bar_start",
            "z_axis": "bar_direction",
            "world_mm": _mat_to_list(bar_world_mm),
        },
        "tool0_left": {
            "tool_name": left_tool_name,
            "world_mm": _mat_to_list(tool0_left_world_mm),
            "bar_to_tool0_mm": _mat_to_list(bar_to_tool0_left_mm),
        },
        "tool0_right": {
            "tool_name": right_tool_name,
            "world_mm": _mat_to_list(tool0_right_world_mm),
            "bar_to_tool0_mm": _mat_to_list(bar_to_tool0_right_mm),
        },
    }

    root = _prompt_export_root()
    if not root:
        print(f"{TITLE}: cancelled.")
        return

    out_dir = os.path.join(root, "BarTool0TF")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{bar_id}.json")

    if os.path.exists(out):
        ans = rs.MessageBox(
            f"'{out}' exists. Overwrite?", 4 | 32, TITLE,
        )
        if ans != 6:
            print(f"{TITLE}: cancelled (kept existing file).")
            return

    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    tl = bar_to_tool0_left_mm[:3, 3]
    tr = bar_to_tool0_right_mm[:3, 3]
    print(f"{TITLE}: saved {out}")
    print(
        f"  bar '{bar_id}' (L={length_mm:.1f}mm) -> "
        f"left tool0 translation [{tl[0]:.2f}, {tl[1]:.2f}, {tl[2]:.2f}] mm, "
        f"right tool0 translation [{tr[0]:.2f}, {tr[1]:.2f}, {tr[2]:.2f}] mm"
    )


if __name__ == "__main__":
    main()
