#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowIK - Interactive viewer for saved `assembly_ik_*` keyframes
(and optional legacy `ik_support` keyframe on the same bar).

The user picks a bar to start; the script enters an interactive loop
that runs until Esc. Inside the loop the user can:

  * click any other bar to switch the active bar;
  * press Enter (or `TogglePose`) to cycle through saved poses
    (currently `assembled` <-> `approach`);
  * use ShowUnbuilt / HideUnbuilt to toggle the visibility of unbuilt bars.

The active bar is highlighted via the same sequence-color overlay used
by RSIKKeyframe / RSSequenceEdit. Bars with no saved IK data still
become active (handy as a navigation aid) but show no robot preview.
The robot is always rendered with the visual mesh.

If the active bar ALSO carries an `ik_support` legacy user-text record,
the support arm + Robotiq gripper preview is baked alongside the dual-
arm, using the saved `assembled` assembly pose as collision context.

The preview is non-baked: on Esc all robot meshes, pineapples, and the
Robotiq block are cleaned up and the sequence display is restored.
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
from core import env_collision as _env_collision_module
from core import ik_collision_setup as _ik_collision_setup_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core import robot_cell_support as _robot_cell_support_module
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    pick_bar,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    get_bar_seq_map,
    reset_sequence_colors,
    show_sequence_colors,
)
from core.rhino_block_import import has_block_definition
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import set_objects_layer, suspend_redraw


# Legacy support-side single-blob key (KEY_SUPPORT_* split is not yet wired
# in `rs_ik_support_keyframe.py`).
IK_SUPPORT_KEY = "ik_support"
PINEAPPLE_ROLE_KEY = "_ik_pineapple_role"
PINEAPPLE_LAYER = "IKPineapplePreview"
SUPPORT_GRIPPER_PREVIEW_ROLE = "support_gripper_preview"
LEFT_TOOL0_LINK = "left_ur_arm_tool0"
RIGHT_TOOL0_LINK = "right_ur_arm_tool0"

# Pose cycle order. Future poses can be appended; cycling is index-modulo.
POSES = ("assembled", "approach")


def _reload():
    global config, env_collision, ik_collision_setup, ik_viz, robot_cell, robot_cell_support
    config = importlib.reload(_config_module)
    env_collision = importlib.reload(_env_collision_module)
    ik_collision_setup = importlib.reload(_ik_collision_setup_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    robot_cell_support = importlib.reload(_robot_cell_support_module)


_reload()


def _apply_groups(state, groups):
    """Merge `{left,right}` group configs into `state.robot_configuration`."""
    for _side, cfg in groups.items():
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


def _cleanup_ids(oids):
    if not oids:
        return
    with suspend_redraw():
        for oid in oids:
            try:
                rs.DeleteObject(oid)
            except Exception:
                pass


# Sub-layer keys under ``config.LAYER_IK_CACHE`` for the two coexisting cells.
IK_LAYER_KEY_ASSEMBLY = "Assembly"
IK_LAYER_KEY_SUPPORT = "Support"


def _show_support_state(
    planner,
    assembly_base_mm,
    assembled_groups,
    support_payload,
    mesh_modes,
    deps,
):
    """Update the support-cell preview at the saved support pose.

    Routed through the new cached ``ik_viz.update_state``: the support cell's
    meshes live on their own sub-layer (``IK_LAYER_KEY_SUPPORT``) so they
    coexist with the dual-arm bake without either side wiping the other.
    PyBullet is also swapped to the support cell so collision queries match.
    """
    cell = robot_cell_support.get_or_load_support_cell()
    state = robot_cell_support.default_support_cell_state()

    # DualArm tool obstacle: always configure at the assembled pose - the
    # support keyframe was solved against assembled regardless of which
    # pose the user is currently viewing on the assembly side.
    state = robot_cell_support.configure_dual_arm_obstacle(
        state,
        base_frame_world_mm=np.asarray(assembly_base_mm, dtype=float),
        joint_values_left=assembled_groups["left"]["joint_values"],
        joint_values_right=assembled_groups["right"]["joint_values"],
        joint_names_left=assembled_groups["left"]["joint_names"],
        joint_names_right=assembled_groups["right"]["joint_names"],
    )

    support_base_mm = np.asarray(support_payload["base_frame_world_mm"], dtype=float)
    state.robot_base_frame = robot_cell._mm_matrix_to_m_frame(deps["Frame"], support_base_mm)
    final_support = support_payload["final"]
    for name, value in zip(final_support["joint_names"], final_support["joint_values"]):
        state.robot_configuration[name] = float(value)

    robot_cell_support.set_cell_state(planner, state)
    ik_viz.update_state(
        state,
        robot_cell=cell,
        mesh_modes=mesh_modes,
        layer_key=IK_LAYER_KEY_SUPPORT,
    )


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


# ---------------------------------------------------------------------------
# Payload loading
# ---------------------------------------------------------------------------


def _load_assembly_payload(bar_oid):
    """Read the new split assembly keys off ``bar_oid``.

    Returns ``{"base_frame_world_mm", "assembled", "approach"}`` (any of
    "assembled" / "approach" may be ``None`` if missing) or ``None`` if
    the bar has no assembled keyframe at all.
    """
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not assembled_raw:
        return None
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    try:
        return {
            "base_frame_world_mm": np.asarray(json.loads(base_raw), dtype=float),
            "assembled": json.loads(assembled_raw),
            "approach": json.loads(approach_raw) if approach_raw else None,
        }
    except json.JSONDecodeError as exc:
        print(f"RSShowIK: malformed user-text on bar ({exc}); skipping IK preview.")
        return None


def _load_support_payload(bar_oid):
    raw = rs.GetUserText(bar_oid, IK_SUPPORT_KEY)
    if not raw:
        return None
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"RSShowIK: malformed '{IK_SUPPORT_KEY}' on bar ({exc}); skipping support preview.")
        return None


def _build_assembly_state(base_mm, groups, deps):
    state = robot_cell.default_cell_state()
    origin_m = base_mm[:3, 3] / 1000.0
    x_axis = base_mm[:3, 0]
    y_axis = base_mm[:3, 1]
    state.robot_base_frame = deps["Frame"](
        list(map(float, origin_m)),
        list(map(float, x_axis)),
        list(map(float, y_axis)),
    )
    _apply_groups(state, groups)
    return state


# ---------------------------------------------------------------------------
# Interactive session
# ---------------------------------------------------------------------------


class _PreviewSession:
    """Holds mutable UI state for one RSShowIK run."""

    def __init__(self, planner):
        self.planner = planner
        self.deps = robot_cell.import_compas_stack()
        self.active_bar_id = None
        self.active_bar_oid = None
        self.pose = POSES[0]
        self.show_unbuilt = False
        self.mesh_mode = ik_viz.MESH_MODE_VISUAL
        # Doc oids the session owns directly (Robotiq gripper block etc).
        self._support_block_ids = []
        self._session_started = False
        # State produced by the most recent _render(); reused by check_collision.
        self._last_assembly_state = None
        self._last_assembly_payload = None
        self._last_assembly_groups = None
        self._last_support_payload = None
        # Highlight bookkeeping: {oid: prev_color_source_or_None}
        self._highlight_oids = []

    # ---- mutations -----------------------------------------------------

    def set_active_bar(self, bar_id, bar_oid):
        # Switching bars changes the env_geom (different built-before set) and
        # may also change which arm tools are attached. Both mutate
        # ``rcell.rigid_body_models`` whose keys are baked into the cached
        # ``RobotCellObject._rigid_body_scene_objects`` dict at first draw -- so
        # the cache must be torn down when the bar changes.
        if self.active_bar_id is not None and bar_id != self.active_bar_id and self._session_started:
            ik_viz.discard_cache()
            self._session_started = False
        self.active_bar_id = bar_id
        self.active_bar_oid = bar_oid
        # Reset pose on bar switch so users always start from "assembled".
        self.pose = POSES[0]
        self.refresh()

    def cycle_pose(self):
        if self.active_bar_id is None:
            return
        idx = POSES.index(self.pose) if self.pose in POSES else -1
        self.pose = POSES[(idx + 1) % len(POSES)]
        self.refresh()

    def toggle_unbuilt(self):
        self.show_unbuilt = not self.show_unbuilt
        if self.active_bar_id is not None:
            show_sequence_colors(self.active_bar_id, self.show_unbuilt)

    def cycle_mesh_mode(self):
        """Flip visual<->collision (cheap layer-visibility toggle, no rebake)."""
        self.mesh_mode = (
            ik_viz.MESH_MODE_COLLISION
            if self.mesh_mode == ik_viz.MESH_MODE_VISUAL
            else ik_viz.MESH_MODE_VISUAL
        )
        # Clear any stale red highlight (it lives on a particular mode's geometry).
        self._revert_highlight()
        if self._session_started:
            ik_viz.set_active_mesh_mode(IK_LAYER_KEY_ASSEMBLY, self.mesh_mode)
            ik_viz.set_active_mesh_mode(IK_LAYER_KEY_SUPPORT, self.mesh_mode)
        print(f"RSShowIK: mesh_mode={self.mesh_mode}")

    # ---- rendering -----------------------------------------------------

    def refresh(self):
        self._revert_highlight()
        self._clear_preview()
        self._last_assembly_state = None
        self._last_assembly_payload = None
        self._last_assembly_groups = None
        self._last_support_payload = None
        if self.active_bar_id is None:
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)
            return

        show_sequence_colors(self.active_bar_id, self.show_unbuilt)

        payload = _load_assembly_payload(self.active_bar_oid)
        if payload is None:
            print(
                f"RSShowIK: bar {self.active_bar_id} has no "
                f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}' record; showing geometry only."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)
            return

        groups = payload.get(self.pose)
        if groups is None:
            # Requested pose missing -> fall back to assembled.
            print(
                f"RSShowIK: bar {self.active_bar_id} has no '{self.pose}' pose; "
                f"showing 'assembled' instead."
            )
            self.pose = "assembled"
            groups = payload["assembled"]

        self._render(payload, groups)

    def _render(self, payload, groups):
        modes = (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION)
        ik_viz.set_mesh_mode(self.mesh_mode)

        rcell = robot_cell.get_or_load_robot_cell()
        # Build the IK-pose state, then mirror IK keyframe's collision setup
        # (per-arm tool RBs + env_* bars/joints) so the cached scene object
        # has the FULL set of rigid bodies to draw -- robot, tools, arm-tool
        # collision meshes, AND every built bar / joint earlier in sequence.
        state = _build_assembly_state(
            payload["base_frame_world_mm"], groups, self.deps
        )
        try:
            state, _env_geom = ik_collision_setup.prepare_assembly_collision_state(
                rcell, self.planner, state, self.active_bar_id
            )
        except Exception as exc:
            print(
                f"RSShowIK: prepare_assembly_collision_state failed "
                f"({type(exc).__name__}: {exc}); rendering robot only."
            )

        rs.EnableRedraw(False)
        try:
            # Open the IK preview session on first render of this _PreviewSession.
            # Pre-bake BOTH visual and collision so MeshMode toggle is a cheap
            # layer-visibility flip rather than a rebake.
            if not self._session_started:
                ik_viz.begin_session(
                    robot_cell=rcell,
                    mesh_modes=modes,
                    active_mesh_mode=self.mesh_mode,
                    layer_key=IK_LAYER_KEY_ASSEMBLY,
                )
                self._session_started = True

            robot_cell.set_cell_state(self.planner, state)
            # Update both modes so toggling shows the latest pose immediately.
            ik_viz.update_state(
                state, mesh_modes=modes, layer_key=IK_LAYER_KEY_ASSEMBLY
            )
            # Make sure the active mode's sub-layer is the visible one.
            ik_viz.set_active_mesh_mode(IK_LAYER_KEY_ASSEMBLY, self.mesh_mode)

            self._last_assembly_state = state
            self._last_assembly_payload = payload
            self._last_assembly_groups = groups

            support_payload = _load_support_payload(self.active_bar_oid)
            self._last_support_payload = support_payload
            if support_payload is not None:
                try:
                    _show_support_state(
                        self.planner,
                        payload["base_frame_world_mm"],
                        payload["assembled"],
                        support_payload,
                        modes,
                        self.deps,
                    )
                    ik_viz.set_active_mesh_mode(IK_LAYER_KEY_SUPPORT, self.mesh_mode)
                    tool0_support_mm = np.asarray(
                        support_payload["tool0_frame_world_mm"], dtype=float
                    )
                    try:
                        self._support_block_ids = _insert_support_gripper(tool0_support_mm)
                    except Exception as exc:
                        print(
                            f"RSShowIK: SupportGripper preview skipped "
                            f"({type(exc).__name__}: {exc})."
                        )
                    stored_support = support_payload.get("robot_id", "<unknown>")
                    print(
                        f"RSShowIK: also showing 'ik_support' for bar "
                        f"{self.active_bar_id} (robot_id={stored_support})."
                    )
                except Exception as exc:
                    print(
                        f"RSShowIK: ik_support display failed "
                        f"({type(exc).__name__}: {exc})."
                    )
            else:
                # Active bar has no support payload; hide stale support arm
                # left over from a previously-active bar.
                ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)

            print(
                f"RSShowIK: showing '{self.pose}' keyframe for bar "
                f"{self.active_bar_id} (mesh_mode={self.mesh_mode})"
            )
        finally:
            rs.EnableRedraw(True)

    # ---- cleanup -------------------------------------------------------

    def _clear_preview(self):
        # Clean up the inserted Robotiq gripper block (not part of the cached
        # cell scene).  The cached robot/tool meshes stay in place; the next
        # _render() call will delta-transform them to the new pose, or
        # refresh() will hide their sub-layers if no payload exists.
        _cleanup_ids(self._support_block_ids)
        self._support_block_ids = []

    # ---- collision diagnostic ------------------------------------------

    def check_collision(self):
        """Run a full-report collision check on the current pose; red-highlight offenders."""
        if self._last_assembly_state is None or self.active_bar_id is None:
            print("RSShowIK: no active IK pose to check; pick a bar with an IK record first.")
            return
        self._revert_highlight()

        rcell = robot_cell.get_or_load_robot_cell()
        # Mirror IK keyframe's full collision setup: per-arm tool RBs
        # (AssemblyLeftArmToolBody/RightArmToolBody, attached_to_link=tool0)
        # AND env_* bars/joints. Without the arm-tool RBs the check would
        # silently drop CC.2 / CC.3 / CC.5 pairs involving tool meshes,
        # which is exactly what was causing IK keyframe to fail while
        # ShowIK reported "no collisions".
        try:
            state, env_geom = ik_collision_setup.prepare_assembly_collision_state(
                rcell, self.planner, self._last_assembly_state, self.active_bar_id
            )
        except Exception as exc:
            print(
                f"RSShowIK: prepare_assembly_collision_state failed "
                f"({type(exc).__name__}: {exc}); aborting."
            )
            return
        try:
            robot_cell.set_cell_state(self.planner, state)
        except Exception as exc:
            print(f"RSShowIK: set_cell_state failed ({exc}); aborting.")
            return

        from compas_fab.backends import CollisionCheckError

        collision_pairs = []
        try:
            self.planner.check_collision(
                state, options={"full_report": True, "verbose": False}
            )
            print("RSShowIK: CheckCollision -- no collisions detected.")
            return
        except CollisionCheckError as exc:
            collision_pairs = list(getattr(exc, "collision_pairs", []) or [])
            for line in str(exc).splitlines():
                print(f"RSShowIK: COLLISION -- {line}")
        except Exception as exc:
            print(f"RSShowIK: check_collision raised {type(exc).__name__}: {exc}")
            return

        # Resolve names from the (Link/Tool/RigidBody) pairs.
        link_names = set()
        tool_names = set()
        rb_names = set()
        for a, b in collision_pairs:
            for item in (a, b):
                cls_name = type(item).__name__
                name = getattr(item, "name", None)
                if name is None:
                    continue
                if cls_name == "Link":
                    link_names.add(name)
                elif cls_name == "ToolModel":
                    tool_names.add(name)
                else:  # RigidBody
                    rb_names.add(name)

        red = (255, 40, 40)
        oids_to_highlight = []

        # Robot links on the currently-visible mode.
        link_geom = ik_viz.get_link_native_geometry(
            rcell, IK_LAYER_KEY_ASSEMBLY, self.mesh_mode
        )
        for ln in link_names:
            for guid in link_geom.get(ln, []):
                oids_to_highlight.append(guid)
        # Tools.
        tool_geom = ik_viz.get_tool_native_geometry(
            rcell, IK_LAYER_KEY_ASSEMBLY, self.mesh_mode
        )
        for tn in tool_names:
            for guid in tool_geom.get(tn, []):
                oids_to_highlight.append(guid)
        # Env rigid bodies (bars / joints) -> doc oid via env_geom.
        for rb in rb_names:
            payload = env_geom.get(rb)
            if payload and payload.get("source_oid"):
                oids_to_highlight.append(payload["source_oid"])

        self._apply_highlight(oids_to_highlight, red)
        print(
            f"RSShowIK: CheckCollision -- {len(collision_pairs)} colliding pair(s); "
            f"highlighted {len(self._highlight_oids)} object(s)."
        )

    def _apply_highlight(self, oids, rgb):
        if not oids:
            return
        with suspend_redraw():
            for oid in oids:
                try:
                    rs.ObjectColor(oid, rgb)
                except Exception:
                    continue
                self._highlight_oids.append(oid)

    def _revert_highlight(self):
        if not self._highlight_oids:
            return
        with suspend_redraw():
            for oid in self._highlight_oids:
                try:
                    # ColorSource 0 = ByLayer (restore default)
                    rs.ObjectColorSource(oid, 0)
                except Exception:
                    continue
        self._highlight_oids = []

    def cleanup(self):
        self._revert_highlight()
        self._clear_preview()
        if self._session_started:
            ik_viz.end_session()
            self._session_started = False
        try:
            reset_sequence_colors()
        except Exception as exc:  # noqa: BLE001 -- never let cleanup mask the real outcome
            print(f"RSShowIK: failed to restore sequence colors ({exc}); continuing.")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def _resolve_pick_to_bar(picked_obj_id):
    """Resolve a clicked oid (centerline curve or tube preview) to
    ``(bar_id, bar_curve_oid)`` or ``(None, None)``."""
    bar_curve_id = _resolve_picked_to_bar_curve(picked_obj_id)
    if bar_curve_id is None:
        return None, None
    bar_id = rs.GetUserText(bar_curve_id, BAR_ID_KEY)
    if not bar_id:
        return None, None
    return bar_id, bar_curve_id


def _build_get_option(session):
    go = Rhino.Input.Custom.GetObject()
    pose_label = session.pose if session.active_bar_id else "-"
    mesh_label = session.mesh_mode
    go.SetCommandPrompt(
        f"Pick a bar to view, Enter to cycle pose [now: {pose_label}, mesh: {mesh_label}], Esc to exit"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)
    go.SetCustomGeometryFilter(_bar_or_tube_filter)
    go.AddOption("TogglePose")
    go.AddOption("MeshMode")
    go.AddOption("CheckCollision")
    if session.show_unbuilt:
        go.AddOption("HideUnbuilt")
    else:
        go.AddOption("ShowUnbuilt")
    return go


def main() -> None:
    _reload()

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, "RSShowIK")
        return
    _client, planner = robot_cell.get_planner()

    rs.UnselectAllObjects()
    initial_oid = pick_bar("Pick a bar to view its IK keyframe (Esc to cancel)")
    if initial_oid is None:
        return
    initial_bar_id = rs.GetUserText(initial_oid, BAR_ID_KEY)
    if not initial_bar_id:
        rs.MessageBox(
            "Picked curve is not a registered bar (no 'bar_id' user-text).",
            0,
            "RSShowIK",
        )
        return

    session = _PreviewSession(planner)
    try:
        session.set_active_bar(initial_bar_id, initial_oid)

        while True:
            go = _build_get_option(session)
            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                break

            if result == Rhino.Input.GetResult.Object:
                bar_id, bar_oid = _resolve_pick_to_bar(go.Object(0).ObjectId)
                if bar_id is None:
                    print("RSShowIK: picked object is not a registered bar; ignoring.")
                    continue
                if bar_id == session.active_bar_id:
                    # Click on already-active bar = cheap refresh (no pose reset).
                    session.refresh()
                else:
                    session.set_active_bar(bar_id, bar_oid)
                continue

            if result == Rhino.Input.GetResult.Nothing:
                session.cycle_pose()
                continue

            if result == Rhino.Input.GetResult.Option:
                name = go.Option().EnglishName
                if name == "TogglePose":
                    session.cycle_pose()
                elif name == "MeshMode":
                    session.cycle_mesh_mode()
                elif name == "CheckCollision":
                    session.check_collision()
                elif name in ("ShowUnbuilt", "HideUnbuilt"):
                    session.toggle_unbuilt()
                continue
    finally:
        session.cleanup()


if __name__ == "__main__":
    main()
