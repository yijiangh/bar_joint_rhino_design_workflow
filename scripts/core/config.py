"""Generic, joint-pair-agnostic constants used across the toolchain.

Per-pair geometry (block-from-bar transforms, contact distances, screw
offsets, asset filenames) lives in :mod:`core.joint_pair` and the
``joint_pairs.json`` registry.  Anything that is *not* tied to a specific
joint pair belongs here.

All distances are in millimetres; all angles are in radians.
"""

from __future__ import annotations

import os
import sys

import numpy as np

from core.transforms import orthonormalize_rotation


try:
    from core import config_generated_ik as _generated_ik
except ImportError:  # Optional: only required for IK workflows.
    _generated_ik = None


_CORE_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(_CORE_DIR)
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)

# ---------------------------------------------------------------------------
# husky_assembly_tamp submodule (the offline planner package)
# ---------------------------------------------------------------------------
# ! The dual-arm IK solvers, the keyframe chain solve, and the base search live
# ! in the husky_assembly_tamp submodule now (its `keyframe` package), together
# ! with their tuning constants. Rhino has no pip install, so the submodule is
# ! put on sys.path here -- config.py is imported first by every entry script,
# ! which wires the path for all later `husky_assembly_tamp.*` imports too.
_TAMP_ROOT = os.path.join(REPO_ROOT, "external", "husky_assembly_tamp")
if not os.path.isdir(os.path.join(_TAMP_ROOT, "husky_assembly_tamp")):
    raise RuntimeError(
        f"husky_assembly_tamp submodule missing at {_TAMP_ROOT}. "
        "Run `git submodule update --init --recursive`."
    )
if _TAMP_ROOT not in sys.path:
    sys.path.insert(0, _TAMP_ROOT)

# Solver tuning is owned by the solver package; re-imported here so Rhino-side
# call sites keep reading `config.IK_*` with ONE definition project-wide. This
# import is cheap (os/sys only -- no compas / pybullet).
from husky_assembly_tamp.keyframe.config import (  # noqa: E402
    IK_BACKEND,
    IK_BASE_SAMPLE_RADIUS,
    IK_BASE_SAMPLE_MAX_ITER,
    IK_BASE_STANDOFF_MM,
    IK_MAX_DESCEND_ITERATIONS,
    IK_MAX_RESTART_ITER,
    IK_TOLERANCE_POSITION,
    IK_TOLERANCE_ORIENTATION,
)


def _as_matrix(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 transform matrix.")
    matrix = np.array(matrix, dtype=float, copy=True)
    matrix[:3, :3] = orthonormalize_rotation(matrix[:3, :3])
    matrix[3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return matrix


def _as_triplet(value) -> tuple[float, float, float]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Expected a 3-vector.")
    return tuple(float(component) for component in vector)


# Bar parameters
# Bar parameters (shared across every joint pair)
BAR_RADIUS = 10.0

# UI: color used to highlight bars the user has just picked, across all
# interactive scripts that show selection feedback (snap, brace, …).
SELECTED_BAR_COLOR = (30, 100, 220)  # blue

# UI: RSUpdatePreview paints broken model links so they are visible without
# reading the command history. Neither case can be auto-repaired (the bar is
# gone / the joint needs RSJointPlace or RSGroundPlace) -- see
# `core.rhino_joint_refresh.find_broken_links`.
ORPHAN_LINK_COLOR = (175, 55, 10)   # orange -- joint/tool whose parent bar is gone
# Muted indigo, deliberately: a green bare-bar color is hard to tell apart from
# the green R-side robotic tools, and a bright purple competes with the orange
# orphan color for attention. This one reads as "flagged" without shouting.
BARE_BAR_COLOR = (75, 55, 110)      # dark indigo -- registered bar carrying no joint

# Output / optimizer tuning
DEFAULT_NEW_BAR_LENGTH = 500.0
OPTIMIZER_RANDOM_RESTARTS = 12
OPTIMIZER_TRANSLATION_PERTURBATION = 50.0


# ---------------------------------------------------------------------------
# Managed Rhino layers
# ---------------------------------------------------------------------------
# All Rhino layers our scripts manage live as sublayers of one root layer.
# The all-caps ``MANAGED`` prefix signals to the user that these layers are
# owned by the toolchain and should not be edited manually.  (Rhino rejects
# layer names that start with a bracket like ``[`` or ``{``.)  On every
# command entry we (1) ensure the root + all sublayers exist and are
# visible, and (2) evict any stray objects on those sublayers back to
# ``DEFAULT_LAYER``.


# ---------------------------------------------------------------------------
# IK keyframe workflow
# ---------------------------------------------------------------------------

# Robot identity and planning groups (compas_fab / PyBullet)
ROBOT_ID = "dual-arm_husky_Cindy"
LEFT_GROUP = "base_left_arm_manipulator"
RIGHT_GROUP = "base_right_arm_manipulator"
LEFT_TOOL_NAME = "AssemblyLeftTool"
RIGHT_TOOL_NAME = "AssemblyRightTool"

# URDF/SRDF locations (relative to repo root `asset/husky_urdf/`)
# HUSKY_URDF_FILENAME is just the filename; LocalPackageMeshLoader.load_urdf()
# implicitly prepends the package's `urdf/` subfolder (see compas_robots
# resources/basic.py). HUSKY_SRDF_REL_PATH keeps the `config/` subfolder
# because SRDF uses `build_path(dirname, basename)` which does not.
HUSKY_PKG_PATH = os.path.join(REPO_ROOT, "asset", "husky_urdf")
HUSKY_URDF_PKG_NAME = "mt_husky_dual_ur5_e_moveit_config"
HUSKY_URDF_FILENAME = "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
HUSKY_SRDF_REL_PATH = os.path.join("config", "dual_arm_husky.srdf")

# Linear-mate (LM) offsets, in mm. Two knobs rather than one, because approach
# and retreat want different clearances: the approach is a short, well-conditioned
# straight push into the mate, while the retreat has to pull the tools fully clear
# of the just-mated joint before the free move home.
#
# Approach (M1 target = M2 start): BOTH tool0 frames translated by
# ``-unit(avg(male z)) * LM_APPROACH_DISTANCE`` off the assembled pose -- one
# shared axis, so the bar stays rigid between the arms
# (`core.bar_action._compute_approach_targets_mm`).
LM_APPROACH_DISTANCE = 15.0  # mm
# Retreat (M3 target): bar released, so each arm pulls back INDEPENDENTLY along
# its OWN male joint's world -Z by this distance
# (`core.bar_action._retreat_tool0_target_mm`).
LM_RETREAT_DISTANCE = 50.0  # mm

# Fixed dual-arm "home" configuration used by Plan M4 (return-to-home after
# bar placement). Order matches the canonical left-then-right arm joint order
# (HUSKY_DUAL_UR5e_JOINT_NAMES[0] + [1]): 6 left arm joints, then 6 right.
HUSKY_DUAL_ARM_HOME_CONF_12 = np.array([
    -1.381079037103113, -0.08674286382411818, -2.8050931738052864,
    -1.7444565873683324, 0.23963370629882144, 1.4217452086745808,
     1.3946926052686688, -3.0267499888085663,  2.8043950421044888,
    -1.727003294848389, -0.40561451816348215, -1.2402309664671707,
])

# The 12-DOF home split into its two 6-DOF arm halves, keeping the authored
# left-then-right order (first 6 -> left arm, last 6 -> right arm). These feed
# Plan M4's return-to-home `target_configuration` in
# `core.bar_action._build_m4`, so the exported action and the RSShowIK preview
# both show the same home pose.
HOME_CONF_LEFT_6 = HUSKY_DUAL_ARM_HOME_CONF_12[:6].tolist()
HOME_CONF_RIGHT_6 = HUSKY_DUAL_ARM_HOME_CONF_12[6:].tolist()

# IK base sampling fallback
IK_BASE_SAMPLE_RADIUS = 1000.0  # mm
IK_BASE_SAMPLE_MAX_ITER = 10
# How far BEHIND the bar the SEED mobile base stands (mm). The seed is offset
# from the bar's ground projection by this distance, OPPOSITE the average
# male-joint insertion direction (the way the bar is pushed to mate), so the base
# faces along the insertion direction and moving forward carries the bar into the
# assembly. Only a starting point: the expanding-radius search samples around this
# seed and IK validates each. Tunable.
IK_BASE_STANDOFF_MM = 1000.0  # mm
# Standoff used by the MULTI-bar IK command (RSIKKeyframeAll): the mobile base is
# placed this far perpendicular from each bar, facing the center between the two
# held joints. Shorter than the single-bar seed above by design (the user places
# these deliberately, closer to the work). Tunable.
#
# Kept equal to the MIDDLE entry of ``BASE_GUIDE_OFFSETS_MM`` below, so the
# auto-placed base lands exactly on the middle base-guide line that
# RSIKKeyframe / RSIKKeyframeAll draw on the walkable ground.
IK_BASE_STANDOFF_MULTIBAR_MM = 500.0  # mm

# ---------------------------------------------------------------------------
# Base-placement guide lines (core.base_guide_geom / core.base_guide_viz)
#
# Both IK keyframe commands draw a set of ground lines that show WHERE the
# mobile base may stand for a bar:
#   - the line joining the bar's two assembly-joint centers, projected onto the
#     walkable ground (offset 0);
#   - that line offset AGAINST the assembly direction by each distance below;
#   - the "extension line" joining all those lines' midpoints -- the line the
#     placed base origin sits on.
# The middle offset is the multi-bar default standoff (see above).
# ---------------------------------------------------------------------------
BASE_GUIDE_OFFSETS_MM = (375.0, 500.0, 625.0)

# ---------------------------------------------------------------------------
# Base heading degeneracy thresholds (core.rhino_walkable_ground.resolve_bar_heading)
#
# The base heading is the average of the bar's anchor-joint insertion axes (each
# joint block's local +Z). A male half's +Z is -bar_X spun about the bar axis by
# its `jr`, i.e. HORIZONTAL and PERPENDICULAR to the bar -- so two anchors on one
# bar can point in opposite directions and cancel, and `jr` can tip an axis
# near-vertical. Either way the naive average degenerates into numerical noise
# and the base lands on an arbitrary side. These two knobs detect that:
#   - an axis whose ground-plane (horizontal) component is shorter than
#     MIN_HORIZONTAL carries no usable azimuth and is discarded;
#   - a set of axes whose mean resultant length (|sum of units| / count) is below
#     CANCEL_TOL is treated as cancelling -> the open-side fallback is used.
# ---------------------------------------------------------------------------
INSERTION_DIR_MIN_HORIZONTAL = 0.20  # ~78 deg from horizontal
INSERTION_DIR_CANCEL_TOL = 0.30      # |sum(unit axes)| / n

# User-text key stamped on a JOINT BLOCK when the user picks that joint's tool
# side BY HAND (RSJointEdit's tool cycle). Value = the chosen side, "left" or
# "right". The automatic heading rule
# (`core.rhino_tool_place.assign_tool_sides_from_heading`) never overwrites a
# joint carrying this, so a deliberate manual edit is not silently undone on the
# next RSUpdatePreview / IK run. It lives on the joint block rather than the tool
# instance because re-placing a tool deletes and recreates the tool object.
# Cleared by an explicit base Flip (that IS the user re-deciding the side).
KEY_TOOL_SIDE_MANUAL = "tool_side_manual"
# NOTE: the IK/base-search tuning constants (IK_BASE_SAMPLE_*, IK_BASE_STANDOFF_MM,
# IK_MAX_DESCEND_ITERATIONS, IK_MAX_RESTART_ITER, IK_TOLERANCE_*, IK_BACKEND) and
# the ssik sidecar wiring moved to `husky_assembly_tamp.keyframe.config` with the
# solvers; they are re-imported near the top of this file so Rhino call sites keep
# reading `config.IK_*`.

# IK solver tuning (compas_fab PyBullet planner)
# How many candidate IK solutions the PyBullet planner generates in ONE
# inverse_kinematics() call: it descends from up to this many internal random
# seeds and returns the first collision-free one (raises if none of them are
# collision-free). Higher = more chances to find a collision-free pose, slower.
# NOTE: only the single-arm SUPPORT robot uses this (see
# core.robot_cell_support). The dual-arm path forces max_results=1 (deterministic
# single descent) and owns its own random restarts in solve_dual_arm_ik so the
# two arms are resampled together -- see IK_MAX_RESTART_ITER (now in the tamp
# keyframe config).
# TODO This seems unnecessary for me and should be using the same IK_MAX_RESTART_ITER as the dual arm
IK_MAX_RESULTS = 20

ARM_SIDES = ("left", "right")


def _sanitize_bar_grasp_to_tool0(raw):
    """{gripper_kind: 4x4 mm} -> orthonormal float matrices."""
    if raw is None:
        return {}
    return {str(k): _as_matrix(v) for k, v in raw.items()}


BAR_GRASP_TO_TOOL0 = _sanitize_bar_grasp_to_tool0(
    getattr(_generated_ik, "BAR_GRASP_TO_TOOL0", None) if _generated_ik is not None else None
)


# ---------------------------------------------------------------------------
# Support (single-arm) robot
# ---------------------------------------------------------------------------

SUPPORT_ROBOT_ID = "single-arm_husky_Alice"
SUPPORT_URDF_PKG_NAME = "mt_husky_moveit_config"
SUPPORT_URDF_FILENAME = "husky_ur5_e_no_base_joint_Alice_Calibrated.urdf"
SUPPORT_SRDF_REL_PATH = os.path.join("config", "husky.srdf")
SUPPORT_GROUP = "manipulator"  # arm-only chain (ur_arm_base_link -> ur_arm_tool0)
SUPPORT_TOOL_NAME = "SupportGripper"
SUPPORT_TOOL_TOUCH_LINKS = ["ur_arm_wrist_3_link"]

# Robotiq gripper Rhino block (origin = tool0). Block must be pre-baked in doc.
ROBOTIQ_GRIPPER_BLOCK = "Robotiq_Gripper"
ROBOTIQ_GRIPPER_TOOL_MESH = os.path.join(REPO_ROOT, "asset", "Robotiq_Gripper_m.obj")

# Dual-arm robot loaded as a static articulated tool (collision obstacle) on the support cell
DUAL_ARM_OBSTACLE_TOOL_NAME = "DualArm"

# IK persistence on supported bar (legacy single-key blob written by
# rs_ik_support_keyframe; will be split into the KEY_SUPPORT_* keys below
# in a follow-up). Keep for backward compat.
IK_SUPPORT_KEY = "ik_support"

# ---------------------------------------------------------------------------
# IK keyframe user-text keys (written on the bar curve).
#
# Splits the legacy `ik_assembly` / `ik_support` JSON blobs into one key per
# logical concept so callers can read/edit them individually and the registry
# is human-greppable. All values are JSON-encoded strings:
#   - *_BASE_FRAME       -> 4x4 list-of-lists, world mm
#   - *_IK_APPROACH/...  -> {"left": {"joint_names": [...], "joint_values": [...]},
#                            "right": {...}}
# ASSEMBLY = dual-arm Husky (rs_ik_keyframe). SUPPORT = single-arm support
# robot (rs_ik_support_keyframe).
KEY_ASSEMBLY_BASE_FRAME = "assembly_robot_base_frame_world_mm"
KEY_ASSEMBLY_IK_APPROACH = "assembly_ik_approach"
KEY_ASSEMBLY_IK_ASSEMBLED = "assembly_ik_assembled"
# Retreat = the M3 target pose (bar released, both arms pull back along the
# joint -Z by LM_RETREAT_DISTANCE). rs_ik_keyframe now solves this third keyframe and
# saves its per-arm config here, next to approach/assembled.
KEY_ASSEMBLY_IK_RETREAT = "assembly_ik_retreat"
KEY_SUPPORT_BASE_FRAME = "support_robot_base_frame_world_mm"
KEY_SUPPORT_IK_APPROACH = "support_ik_approach"
KEY_SUPPORT_IK_HELD = "support_ik_held"

# Dynamic preview / committed preview layer
SUPPORT_PREVIEW_LAYER = "IKSupportPreview"


LAYER_PATH_SEP = "::"  # Rhino's layer-path separator
MANAGED_LAYER_ROOT = "MANAGED Scaffolding"

LAYER_BAR_CENTERLINES = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Bar Centerlines"
LAYER_BAR_TUBE_PREVIEWS = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Bar Tube Previews"
LAYER_JOINT_FEMALE_INSTANCES = (
    MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Joint Female Instances"
)
LAYER_JOINT_MALE_INSTANCES = (
    MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Joint Male Instances"
)
LAYER_JOINT_GROUND_INSTANCES = (
    MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Joint Ground Instances"
)
LAYER_TOOL_INSTANCES = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Robotic Tool Instances"
LAYER_WALKABLE_GROUND = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Walkable Ground"
# Static environment obstacle meshes (tables, walls, scaffolding, etc.). Any
# mesh placed on this layer is registered as a static `obstacle_<name>` rigid
# body by `core.env_collision.collect_environment_geometry` and checked for
# collision against the robot, tools, and grasped workpieces.
LAYER_ENVIRONMENT = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Environment Obstacles"
# Cached IK preview meshes (robot links + tool models). Baked once on first
# `ik_viz.update_state(...)`; toggled visible only inside an IK preview session.
LAYER_IK_CACHE = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "IK Cache"

# Back-compat alias used by IK keyframe scripts.
WALKABLE_GROUND_LAYER = LAYER_WALKABLE_GROUND

# ---------------------------------------------------------------------------
# WalkableGround association (bar -> which ground surface(s) the base samples on)
#
# Each WalkableGround brep gets a stable id (e.g. "WG0") stored as user-text so
# it survives copy/paste and matches the id used in the exported
# ``WalkableGround.json``. Each bar curve stores the comma-separated list of the
# ground ids it may sample its robot base on. ``rs_assign_and_show_walkable_ground`` fills
# these in (auto by distance, then hand-editable); the batch export copies the
# bar's list into its BarAction JSON.
# ---------------------------------------------------------------------------

# User-text key on a WalkableGround brep holding its stable id (e.g. "WG0").
KEY_WALKABLE_GROUND_ID = "walkable_ground_id"
# Prefix used when minting a new ground id.
WALKABLE_GROUND_ID_PREFIX = "WG"
# User-text key on a bar curve: comma-separated ground ids (e.g. "WG0,WG1").
KEY_BAR_WALKABLE_GROUND_IDS = "walkable_ground_ids"

# Auto-association heuristic knobs. A bar is associated with any ground whose
# closest point to the bar centerline is within this distance, capped at
# ``WALKABLE_ASSOC_MAX_COUNT`` grounds (nearest first). The single nearest ground
# is always kept even if it sits beyond the distance cutoff, so every bar gets at
# least one ground to stand on.
WALKABLE_ASSOC_MAX_DIST_MM = 2500.0
WALKABLE_ASSOC_MAX_COUNT = 2

# ---------------------------------------------------------------------------
# Robot base-frame visualization (RSShowBarActionPlan)
#
# The mobile-base "footprint" rectangle drawn on the ground at each solved robot
# base frame, so the assembly-wide map shows where the robot stands for each bar.
# Dimensions are the base extents in mm: length along the base +X (heading),
# width along +Y. Approximate Husky footprint -- adjust to taste; purely cosmetic.
# ---------------------------------------------------------------------------
BASE_FOOTPRINT_LENGTH_MM = 990.0   # along base +X (heading)
BASE_FOOTPRINT_WIDTH_MM = 670.0    # along base +Y
# Axis-triad arm length (mm) for the X/Y/Z lines at the base origin.
BASE_FRAME_AXIS_LEN_MM = 400.0
# Layer the base-frame markers are baked onto (cleared when the viewer exits).
LAYER_BASE_FRAME_PREVIEW = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Base Frame Preview"

# Layer the base-placement GUIDE lines are baked onto (see BASE_GUIDE_OFFSETS_MM).
# Deliberately a separate layer from LAYER_BASE_FRAME_PREVIEW: `base_frame_viz.
# draw_base_frames` clears its own layer before drawing, which would otherwise
# wipe the guides every time a base marker is re-baked. Like the two layers above
# it is NOT in MANAGED_LAYERS -- the managed-layer enforcer would evict the guide
# lines as strays.
LAYER_BASE_GUIDE_PREVIEW = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Base Guide Preview"

# ---------------------------------------------------------------------------
# Arm reachability ghost (RSIKKeyframe base pick)
#
# A translucent sphere per arm, centered on that arm's mount link and sized to
# the UR5e's nominal reach, ghosted at the candidate base so the user can see
# what the robot can actually touch from there. Cosmetic: the real reachability
# test is still the IK solve.
# ---------------------------------------------------------------------------
ARM_REACH_RADIUS_MM = 850.0  # UR5e nominal reach, per arm
# URDF link each arm is bolted to (fixed joints all the way from the root link),
# used to place the reach spheres relative to the mobile-base frame.
ARM_MOUNT_LINKS = {
    "left": "left_ur_arm_base_link",
    "right": "right_ur_arm_base_link",
}

# Layer holding the text-dot markers RSUpdatePreview drops on broken links. A
# block instance's object color only reaches sub-objects whose color source is
# "by parent", and the robotic-tool assets carry baked colors, so recoloring the
# instance is invisible on them -- a colored dot beside the object is what
# actually shows. Deliberately NOT in MANAGED_LAYERS: the managed-layer enforcer
# would evict these markers as strays.
LAYER_DIAGNOSTIC_MARKS = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Diagnostic Marks"

MANAGED_LAYERS = (
    LAYER_BAR_CENTERLINES,
    LAYER_BAR_TUBE_PREVIEWS,
    LAYER_JOINT_FEMALE_INSTANCES,
    LAYER_JOINT_MALE_INSTANCES,
    LAYER_JOINT_GROUND_INSTANCES,
    LAYER_TOOL_INSTANCES,
    LAYER_WALKABLE_GROUND,
    LAYER_ENVIRONMENT,
)
# NOTE: ``LAYER_IK_CACHE`` is intentionally NOT in ``MANAGED_LAYERS``: the
# managed-layer enforcer makes every listed layer visible on every entry-point
# call, but the IK preview cache should stay hidden outside an active session.
DEFAULT_LAYER = "Default"
