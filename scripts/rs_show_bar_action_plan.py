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
``TogglePose``) cycles the bar's timeline: approach -> assembled -> hold (held
bars only: the support robot grips while the arms still hold) -> retreat ->
home, each with its solved IK config + per-movement collision context. Support
robots appear per pose exactly as the schedule says: holders from earlier bars
stand frozen in every pose, the bar's own holder joins from the hold pose
onward, and when the viewed bar is the LAST stabilizer of a hold the cycle
continues into that hold's release pose (its robot retreated, the assembly
robot parked far away). The active bar's base frame is drawn (axis triad +
footprint).
When ``RSLoadSolvedBarAction`` has loaded solved bars, it auto-starts on them,
draws every base frame, and offers NextBar/PrevBar.

Right-click (``main_motion``): the MOTION viewer. Pick a bar; if its planned
trajectory isn't cached yet it loads ``<bar>.solved_motion.json`` from the export
root, then steps through the concatenated planned trajectory (transfer -> insert
-> retreat -> home) from the COMMAND LINE (Enter/Next/Prev/Jump) -- a prompt, not
a modal dialog, so the viewport stays free to zoom/orbit. The held bar follows
the arm while gripped; FK-only rendering.

Both run until Esc / close. A bar with a solved support keyframe (the split
KEY_SUPPORT_* keys) also shows the holding robot alongside the dual-arm. The
preview is non-baked: everything is cleaned up on exit and the sequence display
is restored.
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
from core.rhino_helpers import suspend_redraw


LEFT_TOOL0_LINK = "left_ur_arm_tool0"
RIGHT_TOOL0_LINK = "right_ur_arm_tool0"

# Temporary color painted on the active bar's assigned WalkableGround brep(s) so
# the user can see which ground surface the robot base is allowed to stand on.
# Reverted to ByLayer on bar switch / session cleanup (green, distinct from the
# blue bar-selection color and the red collision highlight).
WALKABLE_GROUND_HIGHLIGHT_COLOR = (60, 200, 90)

# Pose taxonomy -- TogglePose steps through the bar's timeline. Which poses a
# bar gets depends on the hold plan (see _PreviewSession._rebuild_hold_context):
#
#   approach  = arms at the approach keyframe, bar gripped         (J_M5 start)
#   assembled = bar inserted, arms still gripping                  (R_M0 start)
#   hold      = the bar's own support robot grips too (handover)   [held bars]
#   retreat   = arms pulled back, bar released                     (R_M3 start)
#   home      = arms at the fixed home pose                        (R_M3 target)
#   release X = a hold whose LAST stabilizing bar is THIS bar lets go: its
#               robot shown retreated at the approach conf, the assembly robot
#               parked far away                         [last-stabilizer bars]
#
# Support-robot presence per pose: holders that arrived at earlier steps are
# frozen in EVERY pose; the bar's own holder is absent through approach +
# assembled (it only arrives after the insert) and present from hold onward.
HOLD_POSE = "hold"
RELEASE_POSE_PREFIX = "release"
BASE_POSES = ("approach", "assembled", "retreat", "home")


def _pose_label(pose) -> str:
    """Human-readable name of a pose (release poses are tuples)."""
    if isinstance(pose, tuple):
        return f"{pose[0]} {pose[1]}"
    return str(pose)


def derive_hold_plan_safe(label: str = "RSShowBarActionPlan") -> dict:
    """The derived hold plan, or ``{}`` with a printed note if it cannot derive.

    A viewer must never die because the sequence data is momentarily
    inconsistent -- without a plan it simply shows the assembly poses only.

    Args:
        label (str): command name used in the printed note.

    Returns:
        dict: a ``core.hold_schedule.derive_hold_plan`` result (may be empty).
    """
    from core.hold_schedule import derive_hold_plan
    from core.rhino_bar_registry import collect_hold_inputs

    try:
        bar_seq, supported = collect_hold_inputs()
        return derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    except RuntimeError as exc:
        print(f"{label}: hold plan unavailable ({exc}); assembly poses only.")
        return {}


def poses_for_bar(bar_id, bar_oid, hold_plan, quiet: bool = False) -> list:
    """The ordered pose cycle for one bar (see the pose taxonomy above).

    The single definition of "what steps does this bar have", shared by the
    per-bar viewer and the whole-assembly stepper.

    Args:
        bar_id (str): the bar.
        bar_oid: its Rhino object id.
        hold_plan (dict): a ``derive_hold_plan`` result.
        quiet (bool): suppress the "needs holding but unsolved" note (the
            whole-assembly stepper reports those once, up front).

    Returns:
        list: poses -- strings, plus ``(RELEASE_POSE_PREFIX, held_bar_id)``
        tuples for every hold that releases right after this bar.
    """
    poses = ["approach", "assembled"]
    if bar_id in hold_plan:
        if _load_support_payload(bar_oid) is not None:
            poses.append(HOLD_POSE)
        elif not quiet:
            print(
                f"RSShowBarActionPlan: bar {bar_id} needs holding but has no solved "
                "support keyframe — the hold pose is skipped."
            )
    poses += ["retreat", "home"]
    releasing_here = sorted(
        (b for b, e in hold_plan.items() if e["release_after_bar_id"] == bar_id),
        key=lambda b: hold_plan[b]["hold_start_seq"],
    )
    poses += [(RELEASE_POSE_PREFIX, b) for b in releasing_here]
    return poses


def build_global_timeline(hold_plan=None, bar_map=None):
    """Every step of the WHOLE assembly, in sequence order.

    Concatenates each bar's pose cycle (:func:`poses_for_bar`) following the
    assembly sequence, so stepping the result walks the build from the first
    bar to the last -- the same order ``ActionSchedule.json`` records.

    Bars that are fake (never assembled) or have no solved assembly keyframe
    contribute no ASSEMBLY steps; they are returned separately so the caller
    can say so out loud rather than silently skipping them. Their attached
    RELEASE steps are kept, though — a hold whose last stabilizing bar
    happens to be unsolved must still play its release (the release scene
    needs only the held bars' support keyframes, not that bar's assembly
    keyframe), otherwise the holding robots would silently vanish between
    two steps.

    Args:
        hold_plan (dict | None): a ``derive_hold_plan`` result; derived when omitted.
        bar_map (dict | None): a ``get_bar_seq_map`` result; fetched when omitted.

    Returns:
        tuple: ``(steps, skipped)`` where ``steps`` is a list of
        ``(bar_id, bar_oid, pose)`` and ``skipped`` is ``[(bar_id, reason)]``.
    """
    from core.rhino_bar_registry import get_fake_bar_ids

    if bar_map is None:
        bar_map = get_bar_seq_map()
    if hold_plan is None:
        hold_plan = derive_hold_plan_safe()
    fake_ids = get_fake_bar_ids(bar_map)

    steps = []
    skipped = []
    for bar_id in sorted(bar_map, key=lambda b: bar_map[b][1]):
        bar_oid = bar_map[bar_id][0]
        reason = None
        if bar_id in fake_ids:
            reason = "fake bar (never assembled)"
        elif not bar_action.has_ik_keyframe(bar_oid):
            reason = "no solved IK keyframe"
        if reason is None:
            for pose in poses_for_bar(bar_id, bar_oid, hold_plan, quiet=True):
                steps.append((bar_id, bar_oid, pose))
            continue
        # Skipped bar: keep only its attached release steps (if any).
        releases = [
            pose for pose in poses_for_bar(bar_id, bar_oid, hold_plan, quiet=True)
            if isinstance(pose, tuple)
        ]
        for pose in releases:
            steps.append((bar_id, bar_oid, pose))
        if releases:
            held = ", ".join(pose[1] for pose in releases)
            reason += f" — its release step(s) for {held} are kept"
        skipped.append((bar_id, reason))
    return steps, skipped


def support_presence_for_step(bar_map, active_bar_id, pose, pose_cycle, hold_plan,
                              label: str = "RSShowBarActionPlan"):
    """Which support robots stand where for ONE timeline step.

    Answers ``{robot_name: (base_frame_mm, cfg)}`` -- e.g. at bar B37's
    ``("release", "B21")`` step: Alice at her approach (pulled-back) config,
    Belle still at her held config; at ``("release", "B35")``: Alice absent,
    Belle pulled back.

    Module-level (not on ``_PreviewSession``) so the Grasshopper preview can
    reuse it without a planner: everything here reads bar user-text only.

    Presence rules (mirrors the schedule semantics):
    - holders that arrived at EARLIER steps are frozen in every pose;
    - the bar's OWN holder is absent through approach + assembled (it
      only arrives after the insert) and present from hold onward;
    - in a release pose (assembly robot parked far away) the releasing
      robot shows retreated at its approach config, holds released just
      before it are gone, and other spanning holds stay at their held
      configs.

    Args:
        bar_map (dict): a ``get_bar_seq_map`` result.
        active_bar_id (str): the step's host bar.
        pose: the step's pose (string, or a ``(RELEASE_POSE_PREFIX, bar)`` tuple).
        pose_cycle (list): the host bar's full pose cycle (:func:`poses_for_bar`)
            -- needed only to order same-bar releases.
        hold_plan (dict): a ``derive_hold_plan`` result.
        label (str): command name used in printed notes.

    Returns:
        dict: ``{robot_name: (base_frame_mm, cfg)}``.
    """
    entries = {}
    active = bar_map.get(active_bar_id)
    if active is None or not hold_plan:
        return entries

    def _payload_for(held_bar_id):
        held = bar_map.get(held_bar_id)
        p = _load_support_payload(held[0]) if held is not None else None
        if p is None:
            print(
                f"{label}: hold on {held_bar_id} has no solved "
                "support keyframe; its robot is not drawn."
            )
        return p

    if isinstance(pose, tuple):
        releasing_bar = pose[1]
        release_seq = hold_plan[releasing_bar]["release_after_seq"]
        # Releases attached to this bar fire in hold-start order; holds
        # earlier in that order have already let go and driven away.
        release_order = [p[1] for p in pose_cycle if isinstance(p, tuple)]
        for held_bar_id, e in hold_plan.items():
            if not (e["hold_start_seq"] <= release_seq <= e["release_after_seq"]):
                continue
            if (held_bar_id in release_order
                    and release_order.index(held_bar_id) < release_order.index(releasing_bar)):
                continue
            p = _payload_for(held_bar_id)
            if p is None:
                continue
            cfg = p["approach"] if held_bar_id == releasing_bar else p["held"]
            entries[p["robot_name"]] = (p["base_frame_world_mm"], cfg)
        return entries

    from core.hold_schedule import robots_holding_at_step
    for _robot_name, held_bar_id in robots_holding_at_step(hold_plan, active[1]).items():
        p = _payload_for(held_bar_id)
        if p is not None:
            entries[p["robot_name"]] = (p["base_frame_world_mm"], p["held"])
    if pose in (HOLD_POSE, "retreat", "home") and active_bar_id in hold_plan:
        p = _load_support_payload(active[0])
        if p is not None:
            entries[p["robot_name"]] = (p["base_frame_world_mm"], p["held"])
    return entries


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


# Sub-layer keys under ``config.LAYER_IK_CACHE``. The assembly robot has one;
# each support robot gets its OWN sub-layer bundle (two of them can be on
# screen at once, e.g. the viewed bar's holder plus an earlier bar's holder).
IK_LAYER_KEY_ASSEMBLY = "Assembly"


def _support_layer_key(robot_name: str) -> str:
    """The ik_viz sub-layer key for one support robot's preview bundle."""
    return f"Support {robot_name}"


def _render_support_robot(robot_name, base_frame_mm, cfg, mesh_modes, mesh_mode, deps):
    """Draw one support robot at a base + arm config on its own sub-layer.

    Routed through the cached ``ik_viz.update_state``; the robot's attached
    SupportGripper tool rides along, so the gripper shows at the grasp
    without inserting any block. Other robots inside this robot's cell state
    stay PARKED (default), so nothing is drawn twice. Pure mesh preview —
    nothing is pushed into any PyBullet session.

    Args:
        robot_name (str): "Alice" or "Belle".
        base_frame_mm: the hold's 4x4 base frame, world mm.
        cfg (dict): ``{"joint_names", "joint_values"}`` arm config to show.
        mesh_modes: both ik_viz mesh modes (pre-baked for cheap toggling).
        mesh_mode (str): the currently visible mode.
        deps (dict): the imported compas stack.
    """
    cell = robot_cell_support.get_or_load_support_cell(robot_name)
    state = robot_cell_support.default_support_cell_state(robot_name)
    state.robot_base_frame = robot_cell._mm_matrix_to_m_frame(
        deps["Frame"], np.asarray(base_frame_mm, dtype=float)
    )
    for name, value in zip(cfg["joint_names"], cfg["joint_values"]):
        state.robot_configuration[name] = float(value)

    key = _support_layer_key(robot_name)
    ik_viz.update_state(state, robot_cell=cell, mesh_modes=mesh_modes, layer_key=key)
    ik_viz.set_active_mesh_mode(key, mesh_mode)


def _tool0_world_mm(planner, link_name: str) -> np.ndarray:
    """Query PyBullet for the link's world pose (meters), return 4x4 in mm.

    Bypasses `compas_robots.RobotModel.forward_kinematics` because
    `ik_viz.show_state` calls `scene_object.scale(...)` which mutates the
    cached `RobotModel`'s joint origins; subsequent compas-side FK then
    returns translations in doc units rather than meters. PyBullet keeps
    its own URDF-native (meters) state, so the link pose query here is
    immune to that scaling.

    Raw ``pybullet`` calls scoped by ``physicsClientId`` on purpose: the
    ``pp.*`` helpers read a module-global client id and would silently talk
    to the wrong session now that one PyBullet client runs per robot.
    """
    import pybullet

    client_id = planner.client.client_id
    robot_puid = planner.client.robot_puid

    # Find the link index by scanning each joint's child-link name (link i is
    # joint i's child in pybullet).
    link_id = None
    for j in range(pybullet.getNumJoints(robot_puid, physicsClientId=client_id)):
        info = pybullet.getJointInfo(robot_puid, j, physicsClientId=client_id)
        if info[12].decode("utf-8") == link_name:
            link_id = j
            break
    if link_id is None:
        raise RuntimeError(f"Link {link_name!r} not found on the planner's robot.")

    # Indices 4/5 = the URDF link frame in world (what pp.get_link_pose read).
    link_state = pybullet.getLinkState(robot_puid, link_id, physicsClientId=client_id)
    pos, quat = link_state[4], link_state[5]
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(
        pybullet.getMatrixFromQuaternion(quat), dtype=float
    ).reshape(3, 3)
    matrix[:3, 3] = np.asarray(pos, dtype=float) * 1000.0  # m -> mm
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
    """Read the split KEY_SUPPORT_* keys into the viewer's display shape.

    Returns:
        dict | None: ``{"robot_name", "base_frame_world_mm", "held",
        "approach"}`` (held/approach = arm configs), or None when the bar has
        no support keyframe (partial keys are reported + skipped).
    """
    from core import hold_action_builder
    try:
        payload = hold_action_builder.read_bar_support_keyframe(bar_oid)
    except RuntimeError as exc:
        print(f"RSShowBarActionPlan: {exc} Skipping support preview.")
        return None
    if payload is None:
        return None
    return {
        "robot_name": payload["robot_name"],
        "base_frame_world_mm": payload["base_frame_world_mm"],
        "held": payload["held"],
        "approach": payload["approach"],
    }


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
        # Pose cycle for the ACTIVE bar; rebuilt per bar by
        # _rebuild_hold_context (held bars gain "hold", last-stabilizer bars
        # gain the release poses).
        self.poses = list(BASE_POSES)
        self.pose = self.poses[0]
        # The derived hold plan (who holds which bar until when).
        self.hold_plan = {}
        # Support sub-layer keys currently visible, so pose changes can hide
        # robots that left the scene.
        self._shown_support_keys = set()
        # Per-bar movement cache: {bar_id: (token, jointing, release, env_geom)}
        # where token = the bar's raw keyframe user-text strings, so a bar
        # re-solved mid-session is rebuilt instead of served stale. Building
        # the movements forks the full template state ten times — far too
        # heavy to redo on every pose step.
        self._movements_cache = {}
        # {bar_id: (oid, seq)}, refreshed on bar switch (presence lookups
        # would otherwise re-scan the whole document every render).
        self._bar_map = {}
        # What show_sequence_colors last painted, so pose steps skip the
        # full-document recolor when nothing changed.
        self._seq_colored_for = None
        self.show_unbuilt = False
        self.mesh_mode = ik_viz.MESH_MODE_VISUAL
        self._session_started = False
        # State produced by the most recent _render(); reused by check_collision.
        self._last_assembly_state = None
        self._last_env_geom = {}
        self._last_assembly_payload = None
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

    def set_active_bar(self, bar_id, bar_oid, hold_plan=None, pose=None):
        # ! No cache teardown on a bar switch. The cell is a STATIC canonical
        # registry now (every bar/joint registered once, per-step visibility
        # via is_hidden), so the baked meshes stay valid across bars — and
        # ``ik_viz.update_state`` reconciles any body-set drift itself
        # (_sync_rb_keyset). The old ``discard_cache()`` here re-baked the
        # whole cell on every switch, which made the whole-assembly stepper
        # (RSShowAssemblyPlan) unusably slow.
        self.active_bar_id = bar_id
        self.active_bar_oid = bar_oid
        # Cached per-bar data (bar map for presence lookups; movements are
        # cached separately, keyed by bar + keyframe token).
        self._bar_map = get_bar_seq_map()
        # This bar's pose cycle depends on the hold plan (own hold? releases
        # attached here?); start from its first pose, back in keyframe view.
        # ``pose`` lets a caller land on a specific step in one render (the
        # whole-assembly stepper walking backwards into a bar's last pose).
        self._rebuild_hold_context(hold_plan)
        self.pose = pose if pose in self.poses else self.poses[0]
        self.motion_mode = False
        self.motion_wps = []
        self.motion_idx = 0
        self.refresh()
        # Show which WalkableGround brep(s) this bar's robot base stands on.
        self._show_walkable_grounds()

    def _rebuild_hold_context(self, hold_plan=None):
        """Derive the hold plan and this bar's pose cycle.

        The "hold" pose is added only when the bar itself needs holding AND
        its support keyframe is solved; a release pose is appended for every
        hold whose LAST stabilizing bar is this bar (that is when the release
        physically happens — right after this bar's assembly).

        Args:
            hold_plan (dict | None): a pre-derived plan to reuse (the
                whole-assembly stepper derives it once for the run); ``None``
                derives it fresh from the document.
        """
        self.hold_plan = derive_hold_plan_safe() if hold_plan is None else hold_plan
        self.poses = poses_for_bar(self.active_bar_id, self.active_bar_oid, self.hold_plan)

    def cycle_pose(self):
        """Advance to the next pose in this bar's cycle (wraps around)."""
        if self.active_bar_id is None:
            return
        idx = self.poses.index(self.pose) if self.pose in self.poses else -1
        self.pose = self.poses[(idx + 1) % len(self.poses)]
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
            self._seq_colored_for = (self.active_bar_id, self.show_unbuilt)

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
        for key in self._shown_support_keys:
            ik_viz.set_active_mesh_mode(key, self.mesh_mode)
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
        phase's attachments (bar held through the transfer + insert, released
        from the retreat on) so the held bar follows the arm as it scrubs.
        The unplanned lead-in and movements with no trajectory are skipped.
        """
        self.motion_wps = []
        self.motion_idx = 0
        action = self._active_motion_action()
        if action is None:
            return
        # The planned arm movements in timeline order (tool/manual events and
        # the unplanned J_M0 lead-in carry no trajectory).
        for role in ("J_M3", "J_M5", "R_M2", "R_M3"):
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
        self._last_assembly_state = None
        self._last_env_geom = {}
        self._last_assembly_payload = None
        if self.active_bar_id is None:
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            self._hide_support_layers()
            return

        # Recoloring walks every bar/joint/tool in the document — only worth
        # it when the active bar or the unbuilt toggle actually changed
        # (pose steps within one bar repaint nothing).
        wanted_coloring = (self.active_bar_id, self.show_unbuilt)
        if self._seq_colored_for != wanted_coloring:
            show_sequence_colors(self.active_bar_id, self.show_unbuilt)
            self._seq_colored_for = wanted_coloring

        payload = _load_assembly_payload(self.active_bar_oid)
        if payload is None and not isinstance(self.pose, tuple):
            # No assembly keyframe -> no arm pose to draw. Release poses are
            # the exception: they need only the held bars' SUPPORT keyframes
            # (the assembly robot is parked far away), so they render even on
            # an unsolved host bar.
            print(
                f"RSShowBarActionPlan: bar {self.active_bar_id} has no "
                f"'{config.KEY_ASSEMBLY_IK_ASSEMBLED}' record; showing geometry only."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            self._hide_support_layers()
            return

        self._render(payload)

    def _keyframe_token(self):
        """The active bar's raw keyframe user-text — the movement-cache key.

        Cheap (four user-text reads); changes exactly when the bar is
        re-solved, so the cache below can never serve a stale build.
        """
        return tuple(
            rs.GetUserText(self.active_bar_oid, key) or ""
            for key in (
                config.KEY_ASSEMBLY_BASE_FRAME,
                config.KEY_ASSEMBLY_IK_APPROACH,
                config.KEY_ASSEMBLY_IK_ASSEMBLED,
                config.KEY_ASSEMBLY_IK_RETREAT,
            )
        )

    def _build_movements(self, payload):
        """Build (or reuse) the split jointing/release movements for the active bar.

        The single source of the viewer's collision context, shared with
        RSIKKeyframe (both call ``bar_action.build_split_assembly_movements``).
        Each movement's ``start_state`` carries that movement's solved config
        + per-movement attachments. Cached per bar for the session (invalidated
        by the keyframe token when a bar is re-solved), because the build forks
        the full template state per movement — far too heavy per pose step.

        Args:
            payload (dict | None): the bar's keyframe record --
                ``base_frame_world_mm`` plus the ``approach`` / ``assembled``
                / ``retreat`` per-arm configs. ``None`` builds with a
                placeholder base and no configs — enough for the release
                poses of an unsolved host bar (they only need the body
                layout; the robot is parked far away anyway).

        Returns:
            tuple: ``(jointing_mvts, release_mvts, env_geom)``.

        Raises:
            RuntimeError: if the bar's two arm tools can't be resolved.
        """
        token = self._keyframe_token()
        cached = self._movements_cache.get(self.active_bar_id)
        if cached is not None and cached[0] == token:
            return cached[1], cached[2], cached[3]

        rcell = robot_cell.get_or_load_robot_cell()
        arm_tools, err = ik_collision_setup.resolve_arm_tools_on_bar(self.active_bar_id)
        if err is not None:
            raise RuntimeError(err)
        tool0_left = env_collision._block_instance_xform_mm(arm_tools["left"])
        tool0_right = env_collision._block_instance_xform_mm(arm_tools["right"])
        base_mm = (
            payload["base_frame_world_mm"] if payload is not None
            else np.eye(4, dtype=float)
        )
        jointing_mvts, release_mvts, env_geom = bar_action.build_split_assembly_movements(
            rcell, self.planner, self.active_bar_id,
            base_mm,
            tool0_left, tool0_right,
            approach_groups=payload.get("approach") if payload else None,
            assembled_groups=payload.get("assembled") if payload else None,
            retreat_groups=payload.get("retreat") if payload else None,
        )
        self._movements_cache[self.active_bar_id] = (token, jointing_mvts, release_mvts, env_geom)
        return jointing_mvts, release_mvts, env_geom

    def _assembly_state_for_pose(self, payload, jointing_mvts, release_mvts):
        """The assembly robot's cell state for the current pose.

        Args:
            payload (dict): the bar's keyframe record.
            jointing_mvts (dict): ``{"M0".."M5"}`` jointing movements.
            release_mvts (dict): ``{"M0".."M3"}`` release movements.

        Returns:
            RobotCellState: the state to draw.

        Raises:
            RuntimeError: when the pose needs a keyframe the bar lacks.
        """
        # ! Always hand out COPIES: the movements are cached for the session,
        # and the collision-jog dialog mutates the rendered state in place —
        # a shared object would quietly corrupt the cached movement.
        if isinstance(self.pose, tuple):
            # Release pose: the assembly robot has finished this bar and
            # driven away. The structure on screen IS the assembly bundle's
            # baked rigid bodies, so hiding that sub-layer would take every
            # bar with it — instead the robot is drawn at its PARKED base far
            # away, the same convention the collision scenes use. Arms at
            # home; the released-bar body layout comes from the free-home
            # movement's snapshot.
            movement = release_mvts["M3"]
            state = movement.start_state.copy()
            home_cfg = movement.target_configuration
            if home_cfg is not None:
                state.robot_configuration = home_cfg.copy()
            elif state.robot_configuration is None:
                state.robot_configuration = (
                    release_mvts["M2"].start_state.robot_configuration.copy()
                )
            state.robot_base_frame = robot_cell._mm_matrix_to_m_frame(
                self.deps["Frame"],
                np.asarray(config.ROBOT_PARKED_BASE_FRAME_MM, dtype=float),
            )
            print(
                f"RSShowBarActionPlan: {_pose_label(self.pose)} -- assembly robot "
                "parked far away; the support robot lets go and retreats."
            )
            return state
        if self.pose == "approach":
            # Arms at the approach keyframe, bar gripped.
            movement = jointing_mvts["M5"]
            state = movement.start_state.copy()
        elif self.pose in ("assembled", HOLD_POSE):
            # Bar inserted, arms still gripping (the release action's first
            # tool event carries exactly this snapshot).
            movement = release_mvts["M0"]
            state = movement.start_state.copy()
        elif self.pose == "retreat":
            # Arms pulled back, bar released (the free-home movement starts
            # at the retreat keyframe when it is saved).
            movement = release_mvts["M3"]
            state = movement.start_state.copy()
            if state.robot_configuration is None:
                retreat = payload.get("retreat")
                if retreat is None:
                    raise RuntimeError("bar has no saved retreat keyframe")
                state.robot_configuration = (
                    release_mvts["M2"].start_state.robot_configuration.copy()
                )
                _apply_groups(state, retreat)
        elif self.pose == "home":
            # The fixed home pose the arms return to; bar released.
            movement = release_mvts["M3"]
            target_cfg = movement.target_configuration
            if target_cfg is None:
                raise RuntimeError("release action has no home target configuration")
            state = movement.start_state.copy()
            state.robot_configuration = target_cfg.copy()
        else:
            raise RuntimeError(f"unknown pose {self.pose!r}")
        print(
            f"RSShowBarActionPlan: {_pose_label(self.pose)} -- "
            f"{movement.movement_id} | {movement.tag}"
        )
        return state

    def _support_presence(self):
        """Which support robots stand where for the CURRENT pose.

        Thin forwarder: the logic lives in the module-level
        :func:`support_presence_for_step` so the Grasshopper preview can share
        it without a planner.

        Returns:
            dict: ``{robot_name: (base_frame_mm, cfg)}``.
        """
        # Cached on bar switch: a full-document re-scan per pose step is
        # wasted work (the registry cannot change while the viewer runs).
        bar_map = self._bar_map or get_bar_seq_map()
        return support_presence_for_step(
            bar_map, self.active_bar_id, self.pose, self.poses, self.hold_plan,
        )

    def _render(self, payload):
        modes = (ik_viz.MESH_MODE_VISUAL, ik_viz.MESH_MODE_COLLISION)
        ik_viz.set_mesh_mode(self.mesh_mode)

        rcell = robot_cell.get_or_load_robot_cell()
        # Build the movements and the current pose's assembly state (solved
        # config + per-movement collision context -- exactly what RSIKKeyframe
        # used). On failure (e.g. the bar's tools can't be resolved), hide the
        # IK preview.
        env_geom = {}
        try:
            jointing_mvts, release_mvts, env_geom = self._build_movements(payload)
            state = self._assembly_state_for_pose(payload, jointing_mvts, release_mvts)
        except Exception as exc:
            print(
                f"RSShowBarActionPlan: movement build failed "
                f"({type(exc).__name__}: {exc}); hiding IK preview."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            self._hide_support_layers()
            return

        # A pose whose start configuration is planner-computed (left None) has
        # no robot to draw -- warn and show geometry only, but keep the pose in
        # the cycle so the user can still step past it.
        if getattr(state, "robot_configuration", None) is None:
            print(
                f"RSShowBarActionPlan: {_pose_label(self.pose)} has no start "
                "configuration (planner-computed); showing geometry only."
            )
            ik_viz.set_layer_visible(IK_LAYER_KEY_ASSEMBLY, False)
            self._hide_support_layers()
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

            # Support robots, per the presence rules for this pose. Each robot
            # draws on its own sub-layer; robots that left the scene since the
            # last pose are hidden.
            entries = self._support_presence()
            shown = set()
            for robot_name in sorted(entries):
                base_mm, cfg = entries[robot_name]
                try:
                    _render_support_robot(
                        robot_name, base_mm, cfg, modes, self.mesh_mode, self.deps
                    )
                    shown.add(_support_layer_key(robot_name))
                except Exception as exc:
                    print(
                        f"RSShowBarActionPlan: support preview for {robot_name} "
                        f"failed ({type(exc).__name__}: {exc})."
                    )
            for key in self._shown_support_keys - shown:
                ik_viz.set_layer_visible(key, False)
            self._shown_support_keys = shown
            if entries:
                print(
                    "RSShowBarActionPlan: support robot(s) in scene: "
                    + ", ".join(sorted(entries)) + "."
                )

            print(
                f"RSShowBarActionPlan: showing {_pose_label(self.pose)} state for "
                f"bar {self.active_bar_id} (mesh_mode={self.mesh_mode})"
            )
        finally:
            rs.EnableRedraw(True)

    # ---- cleanup -------------------------------------------------------

    def _hide_support_layers(self):
        """Hide every currently shown support robot's preview sub-layer."""
        for key in self._shown_support_keys:
            ik_viz.set_layer_visible(key, False)
        self._shown_support_keys = set()

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
        self._hide_support_layers()
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
