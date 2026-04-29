#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowIK - Replay a saved `ik_assembly` keyframe (and optional
`ik_support` keyframe on the same bar).

Pick a bar that has an `ik_assembly` user-text record (written by
RSIKKeyframe). The script rebuilds the dual-arm robot cell state (base
frame + left/right group configurations) and shows it in Rhino via
`core.ik_viz`. Pineapple (wrist + tool) blocks are inserted at the
tool0 frames derived from FK on the saved configuration so the tool
geometry is visible alongside the robot.

If the same bar ALSO carries an `ik_support` user-text record (written
by RSIKSupportKeyframe), the support arm is baked alongside the dual-
arm using the same mesh-mode, and a Robotiq gripper block is inserted
at the saved support tool0 frame.

Use `mode` option to toggle between the `final` and `approach` sub-
records on the assembly side. Default is `final`. The support side
always shows its single saved pose (it has no approach phase).

The preview is non-baked: when the user dismisses the prompt, both
robots' meshes, pineapples, and the Robotiq block are deleted (mirrors
the RSIKKeyframe try/finally cleanup).
"""

from __future__ import annotations

import importlib
import json
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core import robot_cell_support as _robot_cell_support_module
from core.rhino_bar_pick import pick_bar
from core.rhino_block_import import has_block_definition
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import set_objects_layer, suspend_redraw


IK_ASSEMBLY_KEY = "ik_assembly"
IK_SUPPORT_KEY = "ik_support"
PINEAPPLE_ROLE_KEY = "_ik_pineapple_role"
PINEAPPLE_LAYER = "IKPineapplePreview"
SUPPORT_GRIPPER_PREVIEW_ROLE = "support_gripper_preview"
LEFT_TOOL0_LINK = "left_ur_arm_tool0"
RIGHT_TOOL0_LINK = "right_ur_arm_tool0"


def _reload():
    global config, ik_viz, robot_cell, robot_cell_support
    config = importlib.reload(_config_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    robot_cell_support = importlib.reload(_robot_cell_support_module)


_reload()


def _prompt_mode() -> str:
    answer = rs.GetString("Show which keyframe", "final", ["final", "approach"])
    if answer is None:
        return "final"
    answer = answer.strip().lower()
    if answer not in ("final", "approach"):
        return "final"
    return answer


def _prompt_mesh_mode() -> str:
    """Ask which robot mesh to display; default = previously cached choice."""
    current = ik_viz.get_mesh_mode()
    answer = rs.GetString(
        "Robot mesh display",
        current,
        [ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION],
    )
    if answer is None:
        return current
    answer = answer.strip().lower()
    if answer not in (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION):
        return current
    ik_viz.set_mesh_mode(answer)
    return answer


def _apply_payload(state, payload_group):
    """Merge `{left,right}` group configs from payload into `state.robot_configuration`."""
    for _side, cfg in payload_group.items():
        names = cfg["joint_names"]
        values = cfg["joint_values"]
        for name, value in zip(names, values):
            state.robot_configuration[name] = float(value)


def _np_mm_to_rhino_xform(matrix: np.ndarray):
    scale_from_mm = 1.0 / doc_unit_scale_to_mm()
    doc_matrix = np.array(matrix, dtype=float, copy=True)
    doc_matrix[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for i in range(4):
        for j in range(4):
            xform[i, j] = float(doc_matrix[i, j])
    return xform


def _insert_pineapple(block_name, frame_mm, role):
    oid = rs.InsertBlock(block_name, [0, 0, 0])
    if oid is None:
        raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
    rs.TransformObject(oid, _np_mm_to_rhino_xform(frame_mm))
    rs.SetUserText(oid, PINEAPPLE_ROLE_KEY, role)
    set_objects_layer(oid, PINEAPPLE_LAYER)
    return oid


def _insert_pineapples(tool0_left_mm, tool0_right_mm):
    for block_name in (config.LEFT_PINEAPPLE_BLOCK, config.RIGHT_PINEAPPLE_BLOCK):
        if not has_block_definition(block_name):
            raise RuntimeError(f"Missing required Rhino block definition '{block_name}'.")
    with suspend_redraw():
        left = _insert_pineapple(config.LEFT_PINEAPPLE_BLOCK, tool0_left_mm, "left")
        right = _insert_pineapple(config.RIGHT_PINEAPPLE_BLOCK, tool0_right_mm, "right")
    return [left, right]


def _cleanup_ids(oids):
    if not oids:
        return
    with suspend_redraw():
        for oid in oids:
            try:
                rs.DeleteObject(oid)
            except Exception:
                pass


def _detach_drawn_ids():
    """Steal currently-tracked ik_viz GUIDs out of the sticky dict so a
    subsequent ``ik_viz.show_state`` call doesn't delete them. Returns the
    snapshot — caller is responsible for cleanup. Mirrors the pattern used
    in `rs_ik_support_keyframe.py::_bake_dual_arm_at_captured_pose`.
    """
    guids = list(sc.sticky.get(ik_viz._STICKY_DRAWN_IDS, []) or [])
    sc.sticky[ik_viz._STICKY_DRAWN_IDS] = []
    sc.sticky.pop(ik_viz._STICKY_SCENE_OBJECT, None)
    return guids


def _show_support_state(planner, assembly_payload, support_payload, mesh_mode, deps):
    """Bake the support arm meshes via ik_viz at the saved support pose.
    Swaps the planner's cell to the support cell so PyBullet matches; the
    dual-arm meshes are already in the doc and survive the swap because
    the caller detached their GUIDs from ik_viz tracking first.
    """
    cell = robot_cell_support.get_or_load_support_cell()
    state = robot_cell_support.default_support_cell_state()

    # DualArm tool obstacle: configure at the assembly's FINAL pose (the
    # support keyframe was solved against final, regardless of which mode
    # the user is currently viewing for the assembly side).
    da_payload = assembly_payload["final"]
    state = robot_cell_support.configure_dual_arm_obstacle(
        state,
        base_frame_world_mm=np.asarray(assembly_payload["base_frame_world_mm"], dtype=float),
        joint_values_left=da_payload["left"]["joint_values"],
        joint_values_right=da_payload["right"]["joint_values"],
        joint_names_left=da_payload["left"]["joint_names"],
        joint_names_right=da_payload["right"]["joint_names"],
    )

    support_base_mm = np.asarray(support_payload["base_frame_world_mm"], dtype=float)
    state.robot_base_frame = robot_cell._mm_matrix_to_m_frame(deps["Frame"], support_base_mm)
    final_support = support_payload["final"]
    for name, value in zip(final_support["joint_names"], final_support["joint_values"]):
        state.robot_configuration[name] = float(value)

    robot_cell_support.set_cell_state(planner, state)
    ik_viz.show_state(state, mesh_mode=mesh_mode, robot_model=cell.robot_model)


def _insert_support_gripper(tool0_mm: np.ndarray):
    block_name = config.ROBOTIQ_GRIPPER_BLOCK
    if not has_block_definition(block_name):
        raise RuntimeError(f"Missing required Rhino block definition '{block_name}'.")
    with suspend_redraw():
        oid = rs.InsertBlock(block_name, [0, 0, 0])
        if oid is None:
            raise RuntimeError(f"Failed to insert Rhino block '{block_name}'.")
        rs.TransformObject(oid, _np_mm_to_rhino_xform(tool0_mm))
        rs.SetUserText(oid, PINEAPPLE_ROLE_KEY, SUPPORT_GRIPPER_PREVIEW_ROLE)
        set_objects_layer(oid, config.SUPPORT_PREVIEW_LAYER)
    return [oid]


def _tool0_world_mm(planner, link_name: str) -> np.ndarray:
    """Query PyBullet for the link's world pose (meters), return 4x4 in mm.

    Bypasses `compas_robots.RobotModel.forward_kinematics` because
    `ik_viz.show_state` calls `scene_object.scale(...)` which mutates the
    cached `RobotModel`'s joint origins; subsequent compas-side FK then
    returns translations in doc units rather than meters. PyBullet keeps
    its own URDF-native (meters) state, so the link pose query here is
    immune to that scaling.
    """
    deps = robot_cell.import_compas_stack()
    pp = deps["pp"]
    robot_puid = planner.client.robot_puid
    link_id = pp.link_from_name(robot_puid, link_name)
    pose = pp.get_link_pose(robot_puid, link_id)
    matrix = np.asarray(pp.tform_from_pose(pose), dtype=float)
    matrix[:3, 3] *= 1000.0  # m -> mm
    return matrix


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
    mesh_mode = _prompt_mesh_mode()

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

    pineapple_ids = []
    da_doc_oids = []
    support_block_ids = []
    try:
        robot_cell.set_cell_state(planner, state)
        ik_viz.show_state(state, mesh_mode=mesh_mode)

        try:
            tool0_left_mm = _tool0_world_mm(planner, LEFT_TOOL0_LINK)
            tool0_right_mm = _tool0_world_mm(planner, RIGHT_TOOL0_LINK)
            pineapple_ids = _insert_pineapples(tool0_left_mm, tool0_right_mm)
        except Exception as exc:
            print(f"RSShowIK: pineapple preview skipped ({type(exc).__name__}: {exc}).")

        ik_support_raw = rs.GetUserText(bar_id, IK_SUPPORT_KEY)
        if ik_support_raw:
            try:
                support_payload = json.loads(ik_support_raw)
                # Detach dual-arm bake from ik_viz tracking so the support
                # show_state doesn't wipe it.
                da_doc_oids = _detach_drawn_ids()
                _show_support_state(planner, payload, support_payload, mesh_mode, deps)
                tool0_support_mm = np.asarray(
                    support_payload["tool0_frame_world_mm"], dtype=float
                )
                try:
                    support_block_ids = _insert_support_gripper(tool0_support_mm)
                except Exception as exc:
                    print(f"RSShowIK: SupportGripper preview skipped "
                          f"({type(exc).__name__}: {exc}).")
                stored_support = support_payload.get("robot_id", "<unknown>")
                print(f"RSShowIK: also showing 'ik_support' for bar {bar_id} "
                      f"(robot_id={stored_support}).")
            except Exception as exc:
                print(f"RSShowIK: ik_support display failed "
                      f"({type(exc).__name__}: {exc}).")

        stored_robot = payload.get("robot_id", "<unknown>")
        print(f"RSShowIK: showing '{mode}' keyframe for bar {bar_id}  "
              f"(robot_id={stored_robot}, mesh_mode={mesh_mode})")

        # Block until user dismisses; cleanup runs in `finally` so the
        # robot + pineapple meshes never stay baked in the document.
        rs.GetString("Press Enter to dismiss the IK preview", "OK", ["OK"])
    finally:
        _cleanup_ids(pineapple_ids)
        _cleanup_ids(support_block_ids)
        _cleanup_ids(da_doc_oids)
        ik_viz.clear_scene()


if __name__ == "__main__":
    main()
