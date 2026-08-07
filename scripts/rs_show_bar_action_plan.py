#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
"""RSShowBarActionPlan - View a bar's solved assembly plan (IK keyframes or motion).

Left-click (``main``): the IK KEYFRAME viewer. Pick a bar, then Enter (or
``TogglePose``) cycles the four assembly movements' start states (M1 home -> M2
approach -> M3 assembled -> M4 retreat), each with its solved IK config + per-
movement collision context, plus the M4-home target. The active bar's base frame
is drawn (axis triad + footprint). When ``RSLoadSolvedBarAction`` has loaded solved
bars, it auto-starts on them, draws every base frame, and offers NextBar/PrevBar.

Right-click (``main_motion``): the MOTION viewer. Pick a bar; if its planned
trajectory isn't cached yet it loads ``<bar>.solved_motion.json`` from the export
root, then steps through the concatenated M1..M4 trajectory from the COMMAND LINE
(Enter/Next/Prev/Jump) -- a prompt, not a modal dialog, so the viewport stays free
to zoom/orbit. The held bar follows the arm through M1/M2; FK-only rendering.

Both run until Esc / close. Legacy `ik_support` records are still shown alongside
the dual-arm when present. The preview is non-baked: everything is cleaned up on
exit and the sequence display is restored.
"""

from __future__ import annotations

import importlib
import json
import math
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import bar_action as _bar_action_module
from core import base_frame_viz as _base_frame_viz_module
from core import config as _config_module
from core import env_collision as _env_collision_module
from core import ik_collision_setup as _ik_collision_setup_module
from core import ik_viz as _ik_viz_module
from core import robot_cell as _robot_cell_module
from core import robot_cell_support as _robot_cell_support_module
from core import rhino_walkable_ground as _walkable_ground_module
from core import solved_action_cache as _solved_action_cache_module
from core.rhino_bar_pick import (
    bar_or_tube_filter as _bar_or_tube_filter,
    pick_bar,
    resolve_picked_to_bar_curve as _resolve_picked_to_bar_curve,
)
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    clear_build_stage,
    get_bar_seq_map,
    reset_sequence_colors,
    set_build_stage,
    show_sequence_colors,
)
from core.rhino_block_import import has_block_definition
from core.rhino_frame_io import doc_unit_scale_to_mm
from core.rhino_helpers import set_objects_layer, suspend_redraw


# Legacy support-side single-blob key (KEY_SUPPORT_* split is not yet wired
# in `rs_ik_support_keyframe.py`).
IK_SUPPORT_KEY = "ik_support"
LEFT_TOOL0_LINK = "left_ur_arm_tool0"
RIGHT_TOOL0_LINK = "right_ur_arm_tool0"

# Temporary color painted on the active bar's assigned WalkableGround brep(s) so
# the user can see which ground surface the robot base is allowed to stand on.
# Reverted to ByLayer on bar switch / session cleanup (green, distinct from the
# blue bar-selection color and the red collision highlight).
WALKABLE_GROUND_HIGHLIGHT_COLOR = (60, 200, 90)

# Cycle order -- the four assembly movements plus a final "M4 target" preview.
# TogglePose steps through each movement's START state with its solved config +
# per-movement collision context:
#   M1 = home/gripped, M2 = approach/gripped, M3 = assembled/released,
#   M4 = retreat (bar released; M4's own start config is None, so the saved
#        retreat keyframe is applied to view it).
# The extra final step "M4-home" is not a movement start -- it shows M4's TARGET
# configuration (the home pose the arms return to at the end of the free motion).
M4_TARGET_POSE = "M4-home"
POSES = ("M1", "M2", "M3", "M4", M4_TARGET_POSE)


def _reload():
    global bar_action, base_frame_viz, config, env_collision, ik_collision_setup
    global ik_viz, robot_cell, robot_cell_support, solved_action_cache, walkable_ground
    config = importlib.reload(_config_module)
    env_collision = importlib.reload(_env_collision_module)
    ik_collision_setup = importlib.reload(_ik_collision_setup_module)
    # bar_action.build_assembly_movements is the single source of the per-pose
    # collision state (M2 start = approach, M3 start = assembled).
    bar_action = importlib.reload(_bar_action_module)
    ik_viz = importlib.reload(_ik_viz_module)
    robot_cell = importlib.reload(_robot_cell_module)
    robot_cell_support = importlib.reload(_robot_cell_support_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    solved_action_cache = importlib.reload(_solved_action_cache_module)
    walkable_ground = importlib.reload(_walkable_ground_module)


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
    """Read the split assembly keys off ``bar_oid``.

    Returns ``{"base_frame_world_mm", "approach", "assembled", "retreat"}`` (any of
    "approach" / "retreat" may be ``None`` if missing -- e.g. bars keyframed before
    the three-movement IK) or ``None`` if the bar has no assembled keyframe at all.
    """
    base_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_BASE_FRAME)
    assembled_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_ASSEMBLED)
    if not base_raw or not assembled_raw:
        return None
    approach_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_APPROACH)
    retreat_raw = rs.GetUserText(bar_oid, config.KEY_ASSEMBLY_IK_RETREAT)
    try:
        return {
            "base_frame_world_mm": np.asarray(json.loads(base_raw), dtype=float),
            "approach": json.loads(approach_raw) if approach_raw else None,
            "assembled": json.loads(assembled_raw),
            "retreat": json.loads(retreat_raw) if retreat_raw else None,
        }
    except json.JSONDecodeError as exc:
        print(f"RSShowBarActionPlan: malformed user-text on bar ({exc}); skipping IK preview.")
        return None


def _load_support_payload(bar_oid):
    raw = rs.GetUserText(bar_oid, IK_SUPPORT_KEY)
    if not raw:
        return None
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"RSShowBarActionPlan: malformed '{IK_SUPPORT_KEY}' on bar ({exc}); skipping support preview.")
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
    """Holds mutable UI state for one RSShowBarActionPlan run."""

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
        self._last_env_geom = {}
        self._last_assembly_payload = None
        self._last_support_payload = None
        # Highlight bookkeeping: {oid: prev_color_source_or_None}
        self._highlight_oids = []
        # WalkableGround brep(s) temporarily colored to show the active bar's
        # assigned ground(s). Tracked separately from the collision-red
        # `_highlight_oids` so it survives pose cycling and is only cleared on a
        # bar switch / session cleanup.
        self._wg_highlight_oids = []
        # Bars available to step through (NextBar/PrevBar). Populated by main()
        # from the loaded-solved cache; empty in the classic single-pick flow.
        self.bar_ids = []
        # Motion view: when a `solved_motion` action is loaded, step through the
        # active bar's trajectory waypoints instead of the discrete keyframes.
        self.motion_mode = False
        self.motion_wps = []     # [(role, movement, np.array(wp12)), ...]
        self.motion_idx = 0
        self._jn12 = None        # cached 12 arm-joint names (left 6 + right 6)

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
        # Start from the first movement (M1) on a bar switch, back in keyframe view.
        self.pose = POSES[0]
        self.motion_mode = False
        self.motion_wps = []
        self.motion_idx = 0
        self.refresh()
        # Show which WalkableGround brep(s) this bar's robot base stands on.
        self._show_walkable_grounds()

    def cycle_pose(self):
        """Advance to the next preview step (M1 -> M2 -> M3 -> M4 -> M4-home -> M1)."""
        if self.active_bar_id is None:
            return
        idx = POSES.index(self.pose) if self.pose in POSES else -1
        self.pose = POSES[(idx + 1) % len(POSES)]
        self.refresh()

    def toggle_unbuilt(self):
        """Toggle unbuilt visibility AND the persistent build-stage latch.

        Only this explicit user toggle writes the latch.  ``__init__``'s
        ``show_unbuilt = False`` default and ``refresh()``'s re-assert are
        deliberately left alone: both run with no user involvement, so if
        either wrote the latch, merely opening this viewer would hide the
        model permanently.
        """
        self.show_unbuilt = not self.show_unbuilt
        if self.show_unbuilt:
            clear_build_stage()
        elif self.active_bar_id is not None:
            entry = get_bar_seq_map().get(self.active_bar_id)
            if entry is not None:
                set_build_stage(self.active_bar_id, entry[1])
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
        print(f"RSShowBarActionPlan: mesh_mode={self.mesh_mode}")

    # ---- motion view ---------------------------------------------------

    def _joint_names_12(self, rcell):
        """Return the 12 arm-joint names (left 6 then right 6) a trajectory maps onto.

        Matches the order the headless planner writes ``movement.trajectory`` in
        (``left_names + right_names``). Cached on the session.
        """
        if self._jn12 is None:
            self._jn12 = (
                list(rcell.get_configurable_joint_names(config.LEFT_GROUP))
                + list(rcell.get_configurable_joint_names(config.RIGHT_GROUP))
            )
        return self._jn12

    def _active_motion_action(self):
        """Return the cached loaded action for the active bar IF it is a motion load."""
        loaded, kind = solved_action_cache.get_loaded()
        if kind != "motion":
            return None
        return loaded.get(self.active_bar_id)

    def _build_motion_waypoints(self):
        """Flatten the active bar's per-movement trajectories into one waypoint list.

        Each entry is ``(role, movement, wp12)`` -- the movement supplies that
        phase's attachments (bar held in M1/M2, released in M3/M4) so the held bar
        follows the arm as it scrubs. M0 (unplanned) and movements with no
        trajectory are skipped.
        """
        self.motion_wps = []
        self.motion_idx = 0
        action = self._active_motion_action()
        if action is None:
            return
        for role in ("M1", "M2", "M3", "M4"):
            mv = bar_action._movement_by_role(action, role)
            traj = getattr(mv, "trajectory", None) if mv is not None else None
            if not traj:
                continue
            for wp in traj:
                self.motion_wps.append((role, mv, np.asarray(wp, dtype=float)))

    def render_motion_index(self, idx):
        """Set the scrub position to waypoint ``idx`` and render it (for the slider)."""
        if not self.motion_wps:
            return
        self.motion_idx = max(0, min(int(idx), len(self.motion_wps) - 1))
        self._render_motion_waypoint()

    def _render_motion_waypoint(self):
        """Render the current trajectory waypoint on the active bar."""
        if not self.motion_wps:
            return
        role, movement, wp12 = self.motion_wps[self.motion_idx]
        rcell = robot_cell.get_or_load_robot_cell()
        modes = (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION)

        # Clone the movement's start_state (carries this phase's attachments +
        # base frame) and stamp the waypoint's arm config onto it.
        state = movement.start_state.copy()
        if state.robot_configuration is None:
            state.robot_configuration = rcell.zero_full_configuration()
        for name, val in zip(self._joint_names_12(rcell), wp12):
            state.robot_configuration[name] = float(val)

        rs.EnableRedraw(False)
        try:
            if not self._session_started:
                ik_viz.begin_session(
                    robot_cell=rcell, mesh_modes=modes,
                    active_mesh_mode=self.mesh_mode, layer_key=IK_LAYER_KEY_ASSEMBLY,
                )
                self._session_started = True
            # Visualization only (FK) -- no set_cell_state, so scrubbing stays fast.
            ik_viz.update_state(state, mesh_modes=modes, layer_key=IK_LAYER_KEY_ASSEMBLY)
            ik_viz.set_active_mesh_mode(IK_LAYER_KEY_ASSEMBLY, self.mesh_mode)
            self._last_assembly_state = state
        finally:
            rs.EnableRedraw(True)
        print(f"RSShowBarActionPlan[motion]: {role} waypoint "
              f"{self.motion_idx + 1}/{len(self.motion_wps)} ({movement.movement_id})")

    # ---- rendering -----------------------------------------------------

    def refresh(self):
        self._revert_highlight()
        self._clear_preview()
        self._last_assembly_state = None
        self._last_env_geom = {}
        self._last_assembly_payload = None
        self._last_support_payload = None
        if self.active_bar_id is None:
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)
            return

        show_sequence_colors(self.active_bar_id, self.show_unbuilt)

        payload = _load_assembly_payload(self.active_bar_oid)
        if payload is None:
            print(
                f"RSShowBarActionPlan: bar {self.active_bar_id} has no "
                f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}' record; showing geometry only."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)
            return

        self._render(payload)

    def _build_movements(self, payload):
        """Build the M1-M4 assembly movements for the active bar from its keyframes.

        The single source of the viewer's collision context, shared with
        RSIKKeyframe (both call ``bar_action.build_assembly_movements``). Each
        movement's ``start_state`` carries that movement's solved config +
        per-movement attachments: M1 = home/gripped, M2 = approach/gripped,
        M3 = assembled/released.

        Args:
            payload (dict): the bar's keyframe record -- ``base_frame_world_mm``
                plus the ``approach`` / ``assembled`` per-arm configs.

        Returns:
            tuple: ``(movements, env_geom)`` -- ``{"M1".."M4": Movement}`` and the
            collision-body dict.

        Raises:
            RuntimeError: if the bar's two arm tools can't be resolved.
        """
        rcell = robot_cell.get_or_load_robot_cell()
        arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(self.active_bar_id)
        if err is not None:
            raise RuntimeError(err)
        tool0_left = env_collision._block_instance_xform_mm(arm_tools["left"])
        tool0_right = env_collision._block_instance_xform_mm(arm_tools["right"])
        movements, env_geom = bar_action.build_assembly_movements(
            rcell, self.planner, self.active_bar_id,
            payload["base_frame_world_mm"],
            tool0_left, tool0_right,
            approach_groups=payload.get("approach"),
            assembled_groups=payload.get("assembled"),
        )
        return movements, env_geom

    def _render(self, payload):
        modes = (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION)
        ik_viz.set_mesh_mode(self.mesh_mode)

        rcell = robot_cell.get_or_load_robot_cell()
        # Build the movements and show the CURRENT movement's start state (its solved
        # config + per-movement collision context -- exactly what RSIKKeyframe used).
        # On failure (e.g. the bar's tools can't be resolved), hide the IK preview.
        env_geom = {}
        try:
            movements, env_geom = self._build_movements(payload)
            # The final "M4-home" step is not a movement start; it reuses the M4
            # movement but renders its TARGET configuration instead of the start.
            movement_key = "M4" if self.pose == M4_TARGET_POSE else self.pose
            movement = movements[movement_key]
            state = movement.start_state
            if self.pose == "M4":
                # M4 (free home) has start_state.robot_configuration = None -- the
                # planner fills it from M3's end (the retreat keyframe). Apply the
                # saved retreat config so M4 shows the retreat pose (bar released).
                retreat = payload.get("retreat")
                if retreat is None:
                    raise RuntimeError("bar has no saved retreat keyframe (M4)")
                state = movement.start_state.copy()
                if state.robot_configuration is None:
                    state.robot_configuration = movements["M3"].start_state.robot_configuration.copy()
                _apply_groups(state, retreat)
            elif self.pose == M4_TARGET_POSE:
                # M4's target is the HOME configuration the arms return to at the
                # end of the free motion. The rigid bodies are already the released
                # (bar-detached) M4 layout, so only swap in the target config.
                target_cfg = movement.target_configuration
                if target_cfg is None:
                    raise RuntimeError("M4 has no target configuration (home)")
                state = movement.start_state.copy()
                state.robot_configuration = target_cfg.copy()
            print(
                f"RSShowBarActionPlan: {self.pose} state -- {movement.movement_id} | {movement.tag}"
            )
        except Exception as exc:
            print(
                f"RSShowBarActionPlan: movement build failed "
                f"({type(exc).__name__}: {exc}); hiding IK preview."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            return

        # Some movements carry no start configuration (e.g. M1, whose start config
        # is planner-computed and left None). There is no robot pose to draw, so
        # warn and hide the robot preview -- but keep the movement in the cycle so
        # the user can still step past it. `refresh()` already reset the
        # _last_* fields, so leave them None (check_collision then reports "no
        # active IK pose" instead of acting on a config-less state).
        if getattr(state, "robot_configuration", None) is None:
            print(
                f"RSShowBarActionPlan: {self.pose} has no start configuration "
                f"(planner-computed); showing geometry only, no robot preview."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)
            return

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
            self._last_env_geom = env_geom
            self._last_assembly_payload = payload

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
                            f"RSShowBarActionPlan: SupportGripper preview skipped "
                            f"({type(exc).__name__}: {exc})."
                        )
                    stored_support = support_payload.get("robot_id", "<unknown>")
                    print(
                        f"RSShowBarActionPlan: also showing 'ik_support' for bar "
                        f"{self.active_bar_id} (robot_id={stored_support})."
                    )
                except Exception as exc:
                    print(
                        f"RSShowBarActionPlan: ik_support display failed "
                        f"({type(exc).__name__}: {exc})."
                    )
            else:
                # Active bar has no support payload; hide stale support arm
                # left over from a previously-active bar.
                ik_viz.set_layer_visible(IK_LAYER_KEY_SUPPORT, False)

            print(
                f"RSShowBarActionPlan: showing {self.pose} state for bar "
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
            print("RSShowBarActionPlan: no active IK pose to check; pick a bar with an IK record first.")
            return
        self._revert_highlight()

        # The rendered state is already the movement start state from
        # `_build_movements` (per-arm tool RBs attached at tool0 + every built
        # bar/joint), so check exactly what is on screen -- no rebuild.
        self._run_collision_check_on_state(
            self._last_assembly_state, self._last_env_geom, verbose=True,
        )

    def _run_collision_check_on_state(self, state, env_geom, *, verbose: bool) -> int:
        """Push ``state`` to the planner, run a full-report collision check,
        and red-highlight the offending links / tools / rigid bodies.

        Returns the number of colliding pairs (0 if none). Used by both the
        one-shot ``check_collision`` command and the InteractiveCollisionCheck
        jog dialog (which calls this on every slider tick).
        """
        # Debug: show every body's touch_bodies whitelist (canonical names).
        rb_states = getattr(state, "rigid_body_states", None) or {}
        for key in sorted(rb_states.keys()):
            touch_list = getattr(rb_states[key], "touch_bodies", None) or []
            if touch_list:
                print(
                    f"RSShowBarActionPlan._run_collision_check_on_state: {key} whitelist={sorted(touch_list)}"
                )
        
        try:
            robot_cell.set_cell_state(self.planner, state)
        except Exception as exc:
            print(f"RSShowBarActionPlan: set_cell_state failed ({exc}); aborting.")
            return 0

        from compas_fab.backends import CollisionCheckError

        collision_pairs = []
        try:
            self.planner.check_collision(
                state, options={"full_report": True, "verbose": False}
            )
            if verbose:
                print("RSShowBarActionPlan: CheckCollision -- no collisions detected.")
            return 0
        except CollisionCheckError as exc:
            collision_pairs = list(getattr(exc, "collision_pairs", []) or [])
            if verbose:
                for line in str(exc).splitlines():
                    print(f"RSShowBarActionPlan: COLLISION -- {line}")
        except Exception as exc:
            print(f"RSShowBarActionPlan: check_collision raised {type(exc).__name__}: {exc}")
            return 0

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

        rcell = robot_cell.get_or_load_robot_cell()
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
        if verbose:
            print(
                f"RSShowBarActionPlan: CheckCollision -- {len(collision_pairs)} colliding pair(s); "
                f"highlighted {len(self._highlight_oids)} object(s)."
            )
        return len(collision_pairs)

    # ---- interactive jog dialog ---------------------------------------

    def interactive_collision_check(self):
        """Open a modal Eto dialog with one slider per configurable joint.

        Every slider tick mutates the prepared collision state's
        ``robot_configuration``, pushes it to the planner + Rhino preview,
        and re-runs the same collision check used by ``CheckCollision`` so
        the user can intentionally jog the robot into known bad poses and
        confirm that offenders light up red.

        On dialog close the highlights are cleared but the last jogged
        configuration is left in place. Use ``TogglePose`` (Enter) to
        snap back to a saved IK pose.
        """
        if self._last_assembly_state is None or self.active_bar_id is None:
            print(
                "RSShowBarActionPlan: pick a bar with an IK record and let it render first "
                "(no _last_assembly_state); InteractiveCollisionCheck aborted."
            )
            return
        self._revert_highlight()

        rcell = robot_cell.get_or_load_robot_cell()
        # Jog on top of the rendered movement start state (already built by
        # `_build_movements`); only the configuration changes as the user drags
        # the sliders.
        state = self._last_assembly_state
        env_geom = self._last_env_geom

        joint_specs = []  # list of (group, name, lower_rad, upper_rad)
        for group in (config.LEFT_GROUP, config.RIGHT_GROUP):
            try:
                names = list(rcell.get_configurable_joint_names(group))
            except Exception as exc:
                print(f"RSShowBarActionPlan: get_configurable_joint_names({group!r}) failed ({exc}); skipping group.")
                continue
            for name in names:
                joint = None
                try:
                    joint = rcell.robot_model.get_joint_by_name(name)
                except Exception:
                    joint = None
                lim = getattr(joint, "limit", None) if joint is not None else None
                lo = float(getattr(lim, "lower", -math.pi)) if lim is not None else -math.pi
                hi = float(getattr(lim, "upper",  math.pi)) if lim is not None else  math.pi
                if hi <= lo:
                    lo, hi = -math.pi, math.pi
                joint_specs.append((group, name, lo, hi))

        if not joint_specs:
            print("RSShowBarActionPlan: no configurable joints found; aborting InteractiveCollisionCheck.")
            return

        _run_collision_jog_dialog(self, state, env_geom, joint_specs)
        # Dialog closed: drop highlights, leave the jogged configuration as-is.
        self._revert_highlight()
        print("RSShowBarActionPlan: InteractiveCollisionCheck closed.")

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

    # ---- walkable-ground highlight -------------------------------------

    def _show_walkable_grounds(self):
        """Temporarily color the active bar's assigned WalkableGround brep(s).

        Reads the bar's saved ground-id list (``KEY_BAR_WALKABLE_GROUND_IDS``),
        paints each matching brep green, and prints the ground id + Rhino object
        id on the command line so the user can see (and check) which ground
        surface the robot base is allowed to stand on. Called on every bar
        switch (both the left-click keyframe view and the right-click motion
        view route through ``set_active_bar``).

        Returns:
            None: mutates the document's object colors and ``_wg_highlight_oids``.
        """
        self._revert_walkable_grounds()
        if self.active_bar_oid is None:
            return
        ground_ids = walkable_ground.get_bar_ground_ids(self.active_bar_oid)
        if not ground_ids:
            print(
                f"RSShowBarActionPlan: bar {self.active_bar_id} has no assigned "
                "WalkableGround (run RSAssignAndShowWalkableGround or RSRebuildRobotCell)."
            )
            return
        # {ground_id: oid} for every WalkableGround brep, so we can map the bar's
        # saved id list back to the actual Rhino objects.
        all_grounds = walkable_ground.get_all_walkable_grounds()
        with suspend_redraw():
            for gid in ground_ids:
                oid = all_grounds.get(gid)
                if oid is None:
                    print(
                        f"RSShowBarActionPlan: bar {self.active_bar_id} references "
                        f"WalkableGround '{gid}' but no such brep is in the document."
                    )
                    continue
                try:
                    rs.ObjectColor(oid, WALKABLE_GROUND_HIGHLIGHT_COLOR)
                except Exception:
                    continue
                self._wg_highlight_oids.append(oid)
                print(f"RSShowBarActionPlan: WalkableGround {gid} -> object {oid}")
        print(
            f"RSShowBarActionPlan: bar {self.active_bar_id} stands on "
            f"{ground_ids} (highlighted green)."
        )

    def _revert_walkable_grounds(self):
        """Restore the highlighted WalkableGround brep(s) to their ByLayer color."""
        if not self._wg_highlight_oids:
            return
        with suspend_redraw():
            for oid in self._wg_highlight_oids:
                try:
                    # ColorSource 0 = ByLayer (restore default)
                    rs.ObjectColorSource(oid, 0)
                except Exception:
                    continue
        self._wg_highlight_oids = []

    def cleanup(self):
        self._revert_highlight()
        self._revert_walkable_grounds()
        self._clear_preview()
        if self._session_started:
            ik_viz.end_session()
            self._session_started = False
        try:
            reset_sequence_colors()
        except Exception as exc:  # noqa: BLE001 -- never let cleanup mask the real outcome
            print(f"RSShowBarActionPlan: failed to restore sequence colors ({exc}); continuing.")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def _run_collision_jog_dialog(session, state, env_geom, joint_specs):
    """Open a modal Eto dialog of per-joint sliders for `state.robot_configuration`.

    Each slider tick:
      1. writes the slider's joint value (radians) into ``state.robot_configuration``;
      2. pushes the state to the planner;
      3. updates the cached Rhino preview via ``ik_viz.update_state``;
      4. runs ``session._run_collision_check_on_state`` to re-highlight offenders.

    ``joint_specs`` is a list of ``(group, joint_name, lower_rad, upper_rad)``.
    Sliders are integer 0..1000 mapped linearly onto each joint's [lower, upper]
    range; the value label shows degrees with one decimal.
    """
    import Eto.Forms as forms
    import Eto.Drawing as drawing

    # Eto control constructors don't accept keyword args under Rhino's CPython
    # PythonNet host (TypeError "No overload ... takes '0' arguments"). Use
    # tiny helpers that construct then assign.
    def _label(text):
        lbl = forms.Label()
        lbl.Text = text
        return lbl

    def _button(text):
        btn = forms.Button()
        btn.Text = text
        return btn

    SLIDER_STEPS = 1000

    def slider_value_to_rad(value, lo, hi):
        return lo + (hi - lo) * (float(value) / SLIDER_STEPS)

    def rad_to_slider_value(rad, lo, hi):
        if hi <= lo:
            return 0
        v = (float(rad) - lo) / (hi - lo)
        return int(round(max(0.0, min(1.0, v)) * SLIDER_STEPS))

    dlg = forms.Dialog[bool]()
    dlg.Title = f"Interactive Collision Check - bar {session.active_bar_id}"
    dlg.Padding = drawing.Padding(8)
    dlg.Resizable = True
    dlg.MinimumSize = drawing.Size(520, 360)

    status_label = _label("Move a slider to jog the robot.")
    pair_count_label = _label("Collisions: -")

    layout = forms.DynamicLayout()
    layout.Spacing = drawing.Size(6, 4)
    layout.BeginVertical()

    # Header row.
    layout.AddRow(_label("Joint"), _label("Jog"), _label("Value (deg)"))

    # Track widgets to allow a "Reset" button to snap sliders back to the
    # starting configuration (the IK pose currently rendered).
    initial_values = {}
    sliders_by_name = {}
    value_labels = {}

    # Reusable mode list for ik_viz updates.
    modes = (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION)

    def make_handler(name, lo, hi, val_label):
        def handler(_sender, _args):
            slider = sliders_by_name[name]
            rad = slider_value_to_rad(slider.Value, lo, hi)
            try:
                state.robot_configuration[name] = float(rad)
            except Exception as exc:  # noqa: BLE001
                status_label.Text = f"set {name}={rad:.3f} failed: {exc}"
                return
            val_label.Text = f"{math.degrees(rad):+7.1f}"

            rs.EnableRedraw(False)
            try:
                try:
                    ik_viz.update_state(
                        state, mesh_modes=modes, layer_key=IK_LAYER_KEY_ASSEMBLY
                    )
                except Exception as exc:  # noqa: BLE001
                    status_label.Text = f"ik_viz.update_state failed: {exc}"
                    return
                # Drop previous highlights before re-checking so cleared bodies
                # don't stay red across ticks.
                session._revert_highlight()
                n = session._run_collision_check_on_state(
                    state, env_geom, verbose=False
                )
            finally:
                rs.EnableRedraw(True)
                try:
                    sc.doc.Views.Redraw()
                except Exception:
                    pass

            pair_count_label.Text = (
                "Collisions: 0 (clear)" if n == 0 else f"Collisions: {n} colliding pair(s)"
            )
            status_label.Text = f"{name} = {math.degrees(rad):+.1f} deg ({rad:+.3f} rad)"
        return handler

    cur_group = None
    for (group, name, lo, hi) in joint_specs:
        if group != cur_group:
            layout.AddRow(_label(f"-- {group} --"))
            cur_group = group

        try:
            cur_rad = float(state.robot_configuration[name])
        except Exception:
            cur_rad = 0.0
        cur_rad = max(lo, min(hi, cur_rad))
        initial_values[name] = cur_rad

        slider = forms.Slider()
        slider.MinValue = 0
        slider.MaxValue = SLIDER_STEPS
        slider.Value = rad_to_slider_value(cur_rad, lo, hi)
        slider.Width = 260
        sliders_by_name[name] = slider

        val_label = _label(f"{math.degrees(cur_rad):+7.1f}")
        val_label.Width = 70
        value_labels[name] = val_label

        slider.ValueChanged += make_handler(name, lo, hi, val_label)

        # Compact joint label: drop the long group prefix where possible.
        short = name.split("_", 2)[-1] if name.startswith(("left_", "right_")) else name
        side = "L" if name.startswith("left_") else ("R" if name.startswith("right_") else "?")
        joint_label = _label(f"[{side}] {short}")
        joint_label.Width = 170

        layout.AddRow(joint_label, slider, val_label)

    layout.AddRow(None)  # spacer
    layout.AddRow(status_label)
    layout.AddRow(pair_count_label)

    reset_btn = _button("Reset to current pose")
    verbose_btn = _button("Verbose check (dump pairs)")
    close_btn = _button("Close")

    def on_reset(_sender, _args):
        for name, rad in initial_values.items():
            slider = sliders_by_name[name]
            # Find this joint's lo/hi so we can map back; small linear search.
            spec = next(s for s in joint_specs if s[1] == name)
            _, _, lo, hi = spec
            slider.Value = rad_to_slider_value(rad, lo, hi)
            # ValueChanged fires from the assignment above and updates state.

    def on_verbose(_sender, _args):
        # One-shot full CC dump: enumerates every CC.1..CC.5 pair with
        # PASS / COLLISION / SKIPPED reason. Use to verify that
        # AssemblyLeftArmToolBody / AssemblyRightArmToolBody actually get
        # paired against env_bar_* / env_joint_* in CC.4.
        print("=" * 78)
        print("RSShowBarActionPlan: InteractiveCollisionCheck -- VERBOSE DUMP")
        print("Look for CC.4 lines mentioning 'AssemblyLeftArmToolBody' / "
              "'AssemblyRightArmToolBody'.")
        print("=" * 78)
        try:
            n = session._run_collision_check_on_state(
                state, env_geom, verbose=True
            )
            pair_count_label.Text = (
                "Collisions: 0 (clear)" if n == 0 else f"Collisions: {n} colliding pair(s)"
            )
            status_label.Text = f"verbose dump complete -- see Rhino command log"
        except Exception as exc:  # noqa: BLE001
            status_label.Text = f"verbose check failed: {exc}"

    def on_close(_sender, _args):
        dlg.Close(True)

    reset_btn.Click += on_reset
    verbose_btn.Click += on_verbose
    close_btn.Click += on_close
    layout.AddRow(reset_btn, verbose_btn, close_btn)
    layout.EndVertical()
    dlg.Content = layout
    dlg.AbortButton = close_btn

    # Run an initial check so the pair-count label is meaningful even before
    # any slider is moved.
    try:
        n0 = session._run_collision_check_on_state(state, env_geom, verbose=False)
        pair_count_label.Text = (
            "Collisions: 0 (clear)" if n0 == 0 else f"Collisions: {n0} colliding pair(s)"
        )
    except Exception as exc:  # noqa: BLE001
        pair_count_label.Text = f"initial check failed: {exc}"

    try:
        parent = Rhino.UI.RhinoEtoApp.MainWindow
    except Exception:
        parent = None
    if parent is None:
        dlg.ShowModal()
    else:
        dlg.ShowModal(parent)


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
    now_label = session.pose if session.active_bar_id else "-"
    go.SetCommandPrompt(
        f"Pick a bar, Enter=cycle pose [now: {now_label}, mesh: {session.mesh_mode}], "
        "Esc to exit (right-click the button for the motion scrub)"
    )
    go.EnablePreSelect(False, False)
    go.AcceptNothing(True)
    go.SetCustomGeometryFilter(_bar_or_tube_filter)
    go.AddOption("TogglePose")
    go.AddOption("MeshMode")
    go.AddOption("CheckCollision")
    go.AddOption("InteractiveCollisionCheck")
    if session.show_unbuilt:
        go.AddOption("HideUnbuilt")
    else:
        go.AddOption("ShowUnbuilt")
    # Step through the loaded bars (all-bars map) without having to click each one.
    if len(session.bar_ids) > 1:
        go.AddOption("NextBar")
        go.AddOption("PrevBar")
    return go


def _draw_loaded_base_frames(bar_oids_by_id):
    """Bake a base-frame marker for every loaded bar that has a saved base.

    Reads each bar's base frame from its (just-synced) user-text and draws the
    axis triad + footprint via ``core.base_frame_viz``.

    Args:
        bar_oids_by_id (dict): ``{bar_id: bar_oid}`` for the bars to draw.

    Returns:
        None.
    """
    frames = {}
    for bar_id, oid in bar_oids_by_id.items():
        payload = _load_assembly_payload(oid)
        if payload is not None:
            frames[bar_id] = payload["base_frame_world_mm"]
    if frames:
        base_frame_viz.draw_base_frames(frames)


def _step_bar(session, seq_map, delta):
    """Activate the next/previous bar in ``session.bar_ids`` (the all-bars map)."""
    if len(session.bar_ids) < 2 or session.active_bar_id not in session.bar_ids:
        return
    idx = session.bar_ids.index(session.active_bar_id)
    nxt = session.bar_ids[(idx + delta) % len(session.bar_ids)]
    oid_seq = seq_map.get(nxt)
    if oid_seq is not None:
        session.set_active_bar(nxt, oid_seq[0])


def main() -> None:
    _reload()

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, "RSShowBarActionPlan")
        return
    _client, planner = robot_cell.get_planner()

    rcell = robot_cell.get_or_load_robot_cell()
    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSShowBarActionPlan: aborted (stale collision cell).")
        return

    seq_map = get_bar_seq_map()  # {bar_id: (oid, seq)}

    # If RSLoadSolvedBarAction just loaded solved BarAction(s), start on those bars
    # (no pick) and draw every loaded bar's base frame. Otherwise, classic flow:
    # pick a bar and read its live user-text.
    loaded_actions, _loaded_kind = solved_action_cache.get_loaded()
    loaded_here = {
        bid: seq_map[bid][0] for bid in loaded_actions if bid in seq_map
    }
    if loaded_actions and not loaded_here:
        rs.MessageBox(
            "The loaded solved BarAction(s) are not bars in this document.",
            0, "RSShowBarActionPlan",
        )
        return

    if loaded_here:
        # Assembly-sequence order so NextBar/PrevBar steps in build order.
        bar_ids = sorted(loaded_here, key=lambda b: seq_map[b][1])
        initial_bar_id = bar_ids[0]
        initial_oid = loaded_here[initial_bar_id]
        _draw_loaded_base_frames(loaded_here)
    else:
        rs.UnselectAllObjects()
        initial_oid = pick_bar("Pick a bar to view its IK keyframe (Esc to cancel)")
        if initial_oid is None:
            return
        initial_bar_id = rs.GetUserText(initial_oid, BAR_ID_KEY)
        if not initial_bar_id:
            rs.MessageBox(
                "Picked curve is not a registered bar (no 'bar_id' user-text).",
                0,
                "RSShowBarActionPlan",
            )
            return
        bar_ids = []
        _draw_loaded_base_frames({initial_bar_id: initial_oid})

    session = _PreviewSession(planner)
    session.bar_ids = bar_ids
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
                    print("RSShowBarActionPlan: picked object is not a registered bar; ignoring.")
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
                elif name == "InteractiveCollisionCheck":
                    session.interactive_collision_check()
                elif name in ("ShowUnbuilt", "HideUnbuilt"):
                    session.toggle_unbuilt()
                elif name == "NextBar":
                    _step_bar(session, seq_map, +1)
                elif name == "PrevBar":
                    _step_bar(session, seq_map, -1)
                continue
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()
        solved_action_cache.clear_loaded()


EXPORT_ROOT_STICKY_KEY = "bar_joint:export_root_path"


def _run_motion_command_loop(session):
    """Step through the active bar's trajectory from the Rhino COMMAND LINE.

    Uses a GetOption prompt rather than a modal dialog, so the viewport stays free
    to zoom / orbit / pan while scrubbing. Enter or ``Next`` advances a waypoint,
    ``Prev`` goes back (both wrap around), ``Jump`` asks for a 1-based index, and
    ``Close`` / Esc exits. Each step renders that waypoint (FK only).

    Args:
        session (_PreviewSession): a session whose ``motion_wps`` is already built.

    Returns:
        None.
    """
    n = len(session.motion_wps)
    session.render_motion_index(0)  # show the first waypoint immediately
    while True:
        role, movement, _wp = session.motion_wps[session.motion_idx]
        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(
            f"Motion scrub [{role} {session.motion_idx + 1}/{n}: {movement.movement_id}] "
            "-- Enter/Next=forward, Prev=back, Jump=index, Esc to exit"
        )
        go.AcceptNothing(True)
        go.AddOption("Next")
        go.AddOption("Prev")
        go.AddOption("Jump")
        go.AddOption("Close")
        res = go.Get()

        if res == Rhino.Input.GetResult.Cancel:
            break
        if res == Rhino.Input.GetResult.Nothing:
            session.render_motion_index((session.motion_idx + 1) % n)
            continue
        if res == Rhino.Input.GetResult.Option:
            name = go.Option().EnglishName
            if name == "Next":
                session.render_motion_index((session.motion_idx + 1) % n)
            elif name == "Prev":
                session.render_motion_index((session.motion_idx - 1) % n)
            elif name == "Jump":
                gi = Rhino.Input.Custom.GetInteger()
                gi.SetCommandPrompt(f"Waypoint index (1..{n})")
                gi.SetLowerLimit(1, False)
                gi.SetUpperLimit(n, False)
                gi.SetDefaultInteger(session.motion_idx + 1)
                if gi.Get() == Rhino.Input.GetResult.Number:
                    session.render_motion_index(gi.Number() - 1)
            elif name == "Close":
                break
            continue


def main_motion() -> None:
    """Right-click entry: pick a bar, ensure its motion is loaded, scrub it.

    Loads ``<bar>.solved_motion.json`` from the export root when this bar's motion
    isn't cached yet (RSLoadSolvedBarAction may have loaded it already), syncs the
    condensed IK to user-text, draws the base frame, and steps through the
    trajectory from the command line (viewport stays free to zoom/orbit).
    """
    _reload()

    if not robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0, "RSShowBarActionPlan")
        return
    _client, planner = robot_cell.get_planner()
    rcell = robot_cell.get_or_load_robot_cell()
    if not robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSShowBarActionPlan: aborted (stale collision cell).")
        return

    rs.UnselectAllObjects()
    oid = pick_bar("Pick a bar to scrub its planned motion (Esc to cancel)")
    if oid is None:
        return
    bar_id = rs.GetUserText(oid, BAR_ID_KEY)
    if not bar_id:
        rs.MessageBox("Picked curve is not a registered bar.", 0, "RSShowBarActionPlan")
        return

    # Ensure this bar's MOTION action is cached; else load it from disk now.
    loaded, kind = solved_action_cache.get_loaded()
    action = loaded.get(bar_id) if kind == "motion" else None
    if action is None:
        root = sc.sticky.get(EXPORT_ROOT_STICKY_KEY)
        if not root or not os.path.isdir(os.path.join(root, "BarActions")):
            root = rs.BrowseForFolder(
                folder=root if root and os.path.isdir(root) else None,
                message="Select export root (folder that contains BarActions/)",
                title="RSShowBarActionPlan",
            )
            if not root:
                return
            sc.sticky[EXPORT_ROOT_STICKY_KEY] = root
        actions_dir = os.path.join(root, "BarActions")
        action = solved_action_cache.load_solved_action_file(actions_dir, bar_id, "motion")
        if action is None:
            rs.MessageBox(
                f"No {solved_action_cache.solved_action_filename(bar_id, 'motion')} "
                f"under:\n{actions_dir}\n\nRun the headless motion planner first.",
                0, "RSShowBarActionPlan",
            )
            return
        # Merge into the (motion) cache + sync the condensed IK to user-text.
        loaded = loaded if kind == "motion" else {}
        loaded[bar_id] = action
        solved_action_cache.set_loaded(loaded, "motion")
        bar_action.write_bar_keyframe_from_action(oid, action, rcell)

    session = _PreviewSession(planner)
    try:
        session.set_active_bar(bar_id, oid)
        _draw_loaded_base_frames({bar_id: oid})
        session._build_motion_waypoints()
        if not session.motion_wps:
            rs.MessageBox(
                f"Bar {bar_id} has no planned trajectory in its solved_motion "
                "(run the headless planner with --movement all).",
                0, "RSShowBarActionPlan",
            )
            return
        _run_motion_command_loop(session)
    finally:
        session.cleanup()
        base_frame_viz.clear_base_frames()


if __name__ == "__main__":
    main()
