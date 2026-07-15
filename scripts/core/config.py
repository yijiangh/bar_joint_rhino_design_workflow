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

# Approach distance: tool0 translated by -avg(male z) * LM_DISTANCE before final
LM_DISTANCE = 15.0  # mm

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
IK_BASE_STANDOFF_MM = 500.0  # mm

# IK solver tuning (compas_fab PyBullet planner)
# How many candidate IK solutions the PyBullet planner generates in ONE
# inverse_kinematics() call: it descends from up to this many internal random
# seeds and returns the first collision-free one (raises if none of them are
# collision-free). Higher = more chances to find a collision-free pose, slower.
# NOTE: only the single-arm SUPPORT robot uses this (see
# core.robot_cell_support). The dual-arm path forces max_results=1 (deterministic
# single descent) and owns its own random restarts in solve_dual_arm_ik so the
# two arms are resampled together -- see IK_MAX_RESTART_ITER below.
# TODO This seems unnecessary for me and should be using the same IK_MAX_RESTART_ITER as the dual arm
IK_MAX_RESULTS = 20
# Max gradient-descent steps the solver takes toward the target within a single
# seed before giving up on that seed. Used by both single and dual arm
IK_MAX_DESCEND_ITERATIONS = 200

# Dual-arm IK random-restart budget for a COLD solve (no good warm-start, e.g. the
# first movement M1). Each restart resamples a random dual-arm config to descend
# from. Warm-started solves (M2/M3, seeded from the previous keyframe) force this to 1.
# It is an early-exit CEILING -- easy poses solve in a few restarts; only a tight pose
# (or a genuinely unreachable one) spends the whole budget. Measured M1 success on the
# tight double-kissing-jig B6 at base (0,0,0): 50 -> ~1/8, 150 -> ~4/8, 400 -> 8/8.
IK_MAX_RESTART_ITER = 100
IK_TOLERANCE_POSITION = 1e-5  # m (compas_fab uses SI; values converted at the call site if needed)
IK_TOLERANCE_ORIENTATION = 1e-5  # rad

ARM_SIDES = ("left", "right")

# ---------------------------------------------------------------------------
# IK backend selection
# ---------------------------------------------------------------------------
# * Which inverse-kinematics engine ``core.robot_cell.solve_dual_arm_ik`` uses:
# *   "ssik"     -> DEFAULT. Analytical IK from ssik (github.com/personalrobotics/ssik).
# *                 ssik builds a closed-form solver straight from our CALIBRATED
# *                 URDF, so there is nothing to tune. ssik needs Python 3.11+ and
# *                 Rhino runs 3.9, so it lives in a sidecar process we talk to over
# *                 stdio. One-time venv setup: see the repo README ("Analytical IK
# *                 solver (ssik)") or scripts/ssik_sidecar/README.md. Code:
# *                 ``core.ssik_client`` + ``core.robot_cell.solve_dual_arm_ik_ssik``.
# *   "gradient" -> the original PyBullet damped-least-squares descent with random
# *                 restarts. Kept only for benchmarking against ssik and as an
# *                 archival fallback; not used in the normal workflow.
IK_BACKEND = "ssik"
# IK_BACKEND = "gradient"  # benchmark / archival fallback only

# --- ssik sidecar wiring (paths only; there are no tuning knobs) -----------
# The Python 3.11 venv that has ssik installed. Created one-time by the user (see
# scripts/ssik_sidecar/README.md).
SSIK_VENV_DIR = os.path.join(REPO_ROOT, "external", "ssik_env")
# A venv stores its interpreter in a different place per OS: Windows puts it under
# Scripts\python.exe, macOS/Linux under bin/python. Pick the right one so the
# sidecar launches whether Rhino runs on Windows or Mac.
if sys.platform == "win32":
    SSIK_VENV_PYTHON = os.path.join(SSIK_VENV_DIR, "Scripts", "python.exe")
else:
    SSIK_VENV_PYTHON = os.path.join(SSIK_VENV_DIR, "bin", "python")
# The stdio sidecar server, launched BY that 3.11 interpreter.
SSIK_SIDECAR_SCRIPT = os.path.join(SCRIPTS_DIR, "ssik_sidecar", "serve.py")
# Where the per-arm solver modules generated by `ssik build` live.
SSIK_ARTIFACT_DIR = os.path.join(REPO_ROOT, "asset", "ssik")
# Per-arm build recipe: (base_link, ee_link, generated module name). Documents the
# one-time `ssik build <urdf> --base <base_link> --ee <ee_link>` for each arm and
# tells the client/sidecar which module to import. Each arm is a plain 6R chain.
SSIK_ARM_BUILD = {
    "left": ("left_ur_arm_base_link", "left_ur_arm_tool0", "left_ur_arm_ik"),
    "right": ("right_ur_arm_base_link", "right_ur_arm_tool0", "right_ur_arm_ik"),
}


def _sanitize_ocf_to_tool0_dict(raw):
    """Sanitize `{joint_type: {arm_side: 4x4}}` into orthonormal float matrices.

    Accepts only the nested-per-arm schema. Left and right UR arms each carry
    a distinct physical tool, so the OCF -> tool0 transform differs per side.
    """
    if raw is None:
        return {}
    sanitized = {}
    for joint_type, per_side in raw.items():
        if not isinstance(per_side, dict):
            raise ValueError(
                f"MALE_JOINT_OCF_TO_TOOL0['{joint_type}'] must be a "
                "{'left': <4x4>, 'right': <4x4>} dict. Re-export via RSExportGraspTool0TF (Joint mode)."
            )
        sanitized_sides = {}
        for side, matrix in per_side.items():
            if side not in ARM_SIDES:
                raise ValueError(
                    f"MALE_JOINT_OCF_TO_TOOL0['{joint_type}']['{side}']: side must be one of {ARM_SIDES}."
                )
            sanitized_sides[str(side)] = _as_matrix(matrix)
        sanitized[str(joint_type)] = sanitized_sides
    return sanitized


MALE_JOINT_OCF_TO_TOOL0 = _sanitize_ocf_to_tool0_dict(
    getattr(_generated_ik, "MALE_JOINT_OCF_TO_TOOL0", None) if _generated_ik is not None else None
)


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
# joint -Z by LM_DISTANCE). rs_ik_keyframe now solves this third keyframe and
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
