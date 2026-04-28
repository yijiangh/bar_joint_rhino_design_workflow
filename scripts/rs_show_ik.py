#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowIK - Replay a saved `ik_assembly` keyframe.

Pick a bar that has an `ik_assembly` user-text record (written by
RSIKKeyframe). The script rebuilds the dual-arm robot cell state (base
frame + left/right group configurations) and shows it in Rhino via
`core.ik_viz`.

Use `mode` option to toggle between the `final` and `approach` sub-
records. Default is `final`.
"""

from __future__ import annotations

import importlib
import json
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core.rhino_bar_registry import pick_bar


IK_ASSEMBLY_KEY = "ik_assembly"


def _reload():
    global config, ik_viz, robot_cell
    config = importlib.reload(_config_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)


_reload()


def _prompt_mode() -> str:
    answer = rs.GetString("Show which keyframe", "final", ["final", "approach"])
    if answer is None:
        return "final"
    answer = answer.strip().lower()
    if answer not in ("final", "approach"):
        return "final"
    return answer


def _apply_payload(state, payload_group):
    """Merge `{left,right}` group configs from payload into `state.robot_configuration`."""
    for _side, cfg in payload_group.items():
        names = cfg["joint_names"]
        values = cfg["joint_values"]
        for name, value in zip(names, values):
            state.robot_configuration[name] = float(value)


def main() -> None:
    _reload()

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, "RSShowIK")
        return
    _client, planner = robot_cell.get_planner()

    rs.UnselectAllObjects()
    bar_id = pick_bar("Pick a bar with a saved ik_assembly record")
    if bar_id is None:
        return

    raw = rs.GetUserText(bar_id, IK_ASSEMBLY_KEY)
    if not raw:
        rs.MessageBox(
            f"No '{IK_ASSEMBLY_KEY}' user-text on the selected bar. "
            "Run RSIKKeyframe first.",
            0,
            "RSShowIK",
        )
        return

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        rs.MessageBox(f"Stored ik_assembly is not valid JSON: {exc}", 0, "RSShowIK")
        return

    mode = _prompt_mode()
    if mode not in payload:
        rs.MessageBox(f"'{mode}' keyframe missing in the saved record.", 0, "RSShowIK")
        return

    rcell = robot_cell.get_or_load_robot_cell()
    state = robot_cell.default_cell_state()

    base_mm = np.array(payload["base_frame_world_mm"], dtype=float)
    # Use the private mm -> m Frame helper via a public call path.
    deps = robot_cell.import_compas_stack()
    origin_m = base_mm[:3, 3] / 1000.0
    x_axis = base_mm[:3, 0]
    y_axis = base_mm[:3, 1]
    state.robot_base_frame = deps["Frame"](
        list(map(float, origin_m)), list(map(float, x_axis)), list(map(float, y_axis))
    )
    _apply_payload(state, payload[mode])

    robot_cell.set_cell_state(planner, state)
    ik_viz.show_state(state)

    stored_robot = payload.get("robot_id", "<unknown>")
    print(f"RSShowIK: showing '{mode}' keyframe for bar {bar_id}  (robot_id={stored_robot})")


if __name__ == "__main__":
    main()
