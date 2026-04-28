"""Generic, joint-pair-agnostic constants used across the toolchain.

Per-pair geometry (block-from-bar transforms, contact distances, screw
offsets, asset filenames) lives in :mod:`core.joint_pair` and the
``joint_pairs.json`` registry.  Anything that is *not* tied to a specific
joint pair belongs here.

All distances are in millimetres; all angles are in radians.
"""

from __future__ import annotations

import math


# Bar parameters (shared across every joint pair)
BAR_RADIUS = 10.0

# UI: color used to highlight bars the user has just picked, across all
# interactive scripts that show selection feedback (snap, brace, …).
SELECTED_BAR_COLOR = (30, 100, 220)  # blue

# Joint DOF bounds (used by Rhino UI prompts, optimizer bounds, etc.)
FJP_RANGE = (-500.0, 500.0)
MJP_RANGE = (-500.0, 500.0)
FJR_RANGE = (-math.pi, math.pi)
MJR_RANGE = (-math.pi, math.pi)
JJR_RANGE = (-math.pi, math.pi)

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
LAYER_TOOL_INSTANCES = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Robotic Tool Instances"

MANAGED_LAYERS = (
    LAYER_BAR_CENTERLINES,
    LAYER_BAR_TUBE_PREVIEWS,
    LAYER_JOINT_FEMALE_INSTANCES,
    LAYER_JOINT_MALE_INSTANCES,
    LAYER_TOOL_INSTANCES,
)

DEFAULT_LAYER = "Default"
