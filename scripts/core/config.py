"""Physical parameters for the T20-5 connector system.

All dimensions are in millimeters. All angles are in radians.
Replace the provisional defaults with measured CAD values before production use.
"""

# Bar parameters
BAR_RADIUS = 10.0
BAR_CONTACT_DISTANCE = 20.0

# Female joint OCF offsets (relative to the supporting bar axis)
# Provisional symmetric defaults keep the FK/optimization problem solvable until
# measured connector values are filled in.
FEMALE_RADIAL_OFFSET = BAR_CONTACT_DISTANCE / 2.0
FEMALE_AXIAL_OFFSET = 0.0

# Male joint OCF offsets (relative to the supporting bar axis)
MALE_RADIAL_OFFSET = BAR_CONTACT_DISTANCE / 2.0
MALE_AXIAL_OFFSET = 0.0

# DOF bounds
FJP_RANGE = (-500.0, 500.0)
MJP_RANGE = (-500.0, 500.0)
FJR_RANGE = (-3.141592653589793, 3.141592653589793)
MJR_RANGE = (-3.141592653589793, 3.141592653589793)
JJR_RANGE = (-3.141592653589793, 3.141592653589793)

# Output
DEFAULT_NEW_BAR_LENGTH = 500.0
OPTIMIZER_RANDOM_RESTARTS = 12
OPTIMIZER_TRANSLATION_PERTURBATION = 50.0
