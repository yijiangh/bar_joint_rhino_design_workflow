"""Physical parameters and CAD-derived fixed transforms for the connector system.

All dimensions are in millimeters. All angles are in radians.

`config_generated.py` is the Rhino-exported source of truth for CAD geometry.
This module keeps the stable public API used by the rest of the repo.
"""

from __future__ import annotations

import math
import os

import numpy as np

from core.transforms import invert_transform, orthonormalize_rotation


try:
    from core import config_generated as _generated
except ImportError as exc:  # pragma: no cover - generated file is committed in the repo.
    raise RuntimeError("Missing core.config_generated; run the Rhino CAD export workflow.") from exc

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
BAR_RADIUS = 10.0
BAR_CONTACT_DISTANCE = float(_generated.BAR_CONTACT_DISTANCE)
BAR_GAP = BAR_CONTACT_DISTANCE - 2.0 * BAR_RADIUS

# CAD reference frames
LE_BAR_REFERENCE_FRAME = _as_matrix(_generated.LE_BAR_REFERENCE_FRAME)
LN_BAR_REFERENCE_FRAME = _as_matrix(_generated.LN_BAR_REFERENCE_FRAME)

# Fixed transforms (all translations in mm)
FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM = _as_matrix(_generated.FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM)
FEMALE_MALE_GAP_OFFSET_TRANSFORM = _as_matrix(_generated.FEMALE_MALE_GAP_OFFSET_TRANSFORM)
JJR_ZERO_TRANSFORM = _as_matrix(_generated.JJR_ZERO_TRANSFORM)
MALE_SCREW_HOLE_OFFSET_TRANSFORM = _as_matrix(_generated.MALE_SCREW_HOLE_OFFSET_TRANSFORM)
MALE_FIXED_ROT_TO_BAR_TRANSFORM = _as_matrix(_generated.MALE_FIXED_ROT_TO_BAR_TRANSFORM)

# Visual assets
FEMALE_MESH_FILENAME = str(_generated.FEMALE_MESH_FILENAME)
MALE_MESH_FILENAME = str(_generated.MALE_MESH_FILENAME)
FEMALE_MESH_PATH = os.path.join(SCRIPTS_DIR, FEMALE_MESH_FILENAME)
MALE_MESH_PATH = os.path.join(SCRIPTS_DIR, MALE_MESH_FILENAME)
FEMALE_MESH_SCALE = _as_triplet(_generated.FEMALE_MESH_SCALE)
MALE_MESH_SCALE = _as_triplet(_generated.MALE_MESH_SCALE)

SOURCE_FRAME_NAMES = tuple(str(name) for name in getattr(_generated, "SOURCE_FRAME_NAMES", ()))

# Derived compatibility values from the CAD transforms.
FEMALE_AXIAL_OFFSET = float(FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM[2, 3])
FEMALE_RADIAL_OFFSET = float(
    np.linalg.norm(FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM[:3, 3] - np.array([0.0, 0.0, FEMALE_AXIAL_OFFSET]))
)
MALE_AXIAL_OFFSET = float(MALE_FIXED_ROT_TO_BAR_TRANSFORM[0, 3])
MALE_RADIAL_OFFSET = float(np.linalg.norm(MALE_FIXED_ROT_TO_BAR_TRANSFORM[:3, 3] - np.array([MALE_AXIAL_OFFSET, 0.0, 0.0])))

# Joint DOF bounds
FJP_RANGE = (-500.0, 500.0)
MJP_RANGE = (-500.0, 500.0)
FJR_RANGE = (-math.pi, math.pi)
MJR_RANGE = (-math.pi, math.pi)
JJR_RANGE = (-math.pi, math.pi)

# Output / optimizer tuning
DEFAULT_NEW_BAR_LENGTH = 500.0
OPTIMIZER_RANDOM_RESTARTS = 12
OPTIMIZER_TRANSLATION_PERTURBATION = 50.0


def inverse_fixed_transform(transform: np.ndarray) -> np.ndarray:
    return invert_transform(transform)


# ---------------------------------------------------------------------------
# IK keyframe workflow
# ---------------------------------------------------------------------------

# Robot identity and planning groups (compas_fab / PyBullet)
ROBOT_ID = "dual-arm_husky_Cindy"
LEFT_GROUP = "base_left_arm_manipulator"
RIGHT_GROUP = "base_right_arm_manipulator"
LEFT_TOOL_NAME = "AL"
RIGHT_TOOL_NAME = "AR"

# URDF/SRDF locations (relative to repo root `asset/husky_urdf/`)
# HUSKY_URDF_FILENAME is just the filename; LocalPackageMeshLoader.load_urdf()
# implicitly prepends the package's `urdf/` subfolder (see compas_robots
# resources/basic.py). HUSKY_SRDF_REL_PATH keeps the `config/` subfolder
# because SRDF uses `build_path(dirname, basename)` which does not.
HUSKY_PKG_PATH = os.path.join(REPO_ROOT, "asset", "husky_urdf")
HUSKY_URDF_PKG_NAME = "mt_husky_dual_ur5_e_moveit_config"
HUSKY_URDF_FILENAME = "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
HUSKY_SRDF_REL_PATH = os.path.join("config", "dual_arm_husky.srdf")

# Pineapple (wrist + tool) proxy block names, pre-baked by user
LEFT_PINEAPPLE_BLOCK = "AssemblyLeft_Pineapple"
RIGHT_PINEAPPLE_BLOCK = "AssemblyRight_Pineapple"

# Tool meshes exported from the pineapple Rhino block definitions, in METERS
# (compas convention). Written by `RSExportPineappleOBJ`; consumed by
# `core.robot_cell.get_or_load_robot_cell` when attaching tool models for
# IK collision checking.
LEFT_PINEAPPLE_TOOL_MESH = os.path.join(REPO_ROOT, "asset", "AssemblyLeft_Pineapple_m.obj")
RIGHT_PINEAPPLE_TOOL_MESH = os.path.join(REPO_ROOT, "asset", "AssemblyRight_Pineapple_m.obj")

# Layer holding ground Breps for base-point snapping
WALKABLE_GROUND_LAYER = "WalkableGround"

# Approach distance: tool0 translated by -avg(male z) * LM_DISTANCE before final
LM_DISTANCE = 15.0  # mm

# IK base sampling fallback
IK_BASE_SAMPLE_RADIUS = 150.0  # mm
IK_BASE_SAMPLE_MAX_ITER = 20

# IK solver tuning (compas_fab PyBullet planner)
IK_MAX_RESULTS = 20
IK_MAX_DESCEND_ITERATIONS = 200
IK_TOLERANCE_POSITION = 1e-3  # m (compas_fab uses SI; values converted at the call site if needed)
IK_TOLERANCE_ORIENTATION = 1e-3  # rad


ARM_SIDES = ("left", "right")


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
                "{'left': <4x4>, 'right': <4x4>} dict. Re-export via RSExportJointTool0TF."
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
