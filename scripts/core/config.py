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


_CORE_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(_CORE_DIR)


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
