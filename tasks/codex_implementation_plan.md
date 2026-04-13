# Plan: Bar Joint Rhino Design Workflow

## Context

We have a robot-friendly scaffolding connector system (T20-5) consisting of male and female joint halves that attach to cylindrical bars. Designers need scripts in Rhino 8 to:
1. **T1 - Bar Axis Generation**: Given existing bars, position a new bar at the correct contact distance D
2. **T2 - Joint Placement**: Given locked bar positions, optimize joint DOF values and place joint block instances

There are two scenarios:
- **S1**: New bar `Ln` joins ONE existing bar `Le`
- **S2**: New bar `Ln` joins TWO existing bars `Le1`, `Le2`

Target: Rhino 8 (CPython), using numpy + scipy. No Gurobi dependency.

---

## File Structure

```
scripts/
├── t1_bar_axis.py              # Entry point for T1 (bar axis line generation)
├── t1_bar_axis_rerun.py        # Thin wrapper: reruns T1 with cached inputs
├── t2_joint_placement.py       # Entry point for T2 (joint placement)
├── t2_joint_placement_rerun.py # Thin wrapper: reruns T2 with cached inputs
└── core/
    ├── __init__.py
    ├── geometry.py             # Line-line distance math (pure numpy)
    ├── kinematics.py           # FK chains, joint optimization (numpy + scipy)
    └── config.py               # Connector geometry parameters (user fills in)
tests/
├── test_geometry.py            # Unit tests for line-line distance math
├── test_kinematics.py          # Unit tests for FK chains and joint optimization
└── test_s2_t1.py               # Unit tests for S2-T1 bar axis generation
```

---

## Module 1: `core/config.py` - Connector Parameters

Define all physical connector dimensions as named constants. The user will fill in exact values from CAD measurements.

```python
"""Physical parameters for the T20-5 connector system.
All dimensions in millimeters, angles in radians.
Users: fill in exact values from CAD measurements of your connector.
"""

# Bar parameters
BAR_RADIUS = 10.0  # radius of 20mm tubes
BAR_CONTACT_DISTANCE = 20.0  # center-to-center distance D between two touching bars (= 2 * BAR_RADIUS for tangent contact, or larger if there's a gap)

# Female joint OCF offsets (relative to bar axis)
# Origin: projection of male joint screw hole center onto the bar axis line
# The radial offset from bar center to the female joint OCF origin
FEMALE_RADIAL_OFFSET = 0.0  # TODO: measure from CAD (distance from bar axis to female OCF origin, along the contact normal direction)
FEMALE_AXIAL_OFFSET = 0.0   # TODO: if OCF origin is shifted along the bar axis from the contact point

# Male joint OCF offsets (relative to bar axis)
# Origin: midpoint between M_P1 and M_P2
MALE_RADIAL_OFFSET = 0.0    # TODO: measure from CAD
MALE_AXIAL_OFFSET = 0.0     # TODO: if OCF origin is shifted along the bar axis

# DOF bounds
FJP_RANGE = (-500.0, 500.0)  # Female prismatic sliding range along bar (mm)
MJP_RANGE = (-500.0, 500.0)  # Male prismatic sliding range along bar (mm)
FJR_RANGE = (-3.14159, 3.14159)  # Female revolute around bar axis (rad), full rotation
MJR_RANGE = (-3.14159, 3.14159)  # Male revolute around bar axis (rad)
JJR_RANGE = (-3.14159, 3.14159)  # Joint-joint rotation (rad)

# Output
DEFAULT_NEW_BAR_LENGTH = 500.0  # Default length for generated Ln line (mm)
```

---

## Module 2: `core/geometry.py` - Line Geometry Math

This module contains pure-numpy functions for line-line distance computation. Port directly from the FrameX codebase (see `support_materials/FrameX/python/multi_tangent/contact.py:39-81` and `support_materials/FrameX/python/multi_tangent/smilp/util.py:35-73`).

### Function 1: `closest_params_infinite_lines(x_i, n_i, x_j, n_j)`

Computes the arc-length parameters `t_i, t_j` at the closest points between two **infinite** lines.

- **Line representation**: point `x` + parameter `t` * direction `n` (direction need not be unit)
- **Math** (Equation 17 from the CADJ paper, Section 3.5):
  ```
  T = [t_i, t_j]^T = Mat^{-1} @ b
  where:
    Mat = [[n_i . n_i,   -n_i . n_j],
           [-n_i . n_j,   n_j . n_j]]    (note: n_j^T n_i = n_i^T n_j)
    b   = [(x_j - x_i) . n_i,
           (x_i - x_j) . n_j]
  ```
- **Returns**: `(t_i, t_j)` such that closest points are `x_i + t_i * n_i` and `x_j + t_j * n_j`
- **Precondition**: lines must not be parallel. Check `|det(Mat)| > tol` before calling. If parallel, raise or return None.

**Reference implementation**: `support_materials/FrameX/python/multi_tangent/smilp/util.py` lines 47-73 (`parameter_tA`, `parameter_tB`).

### Function 2: `distance_infinite_lines(x_i, n_i, x_j, n_j)`

Signed distance between two infinite lines (for non-parallel case).

- **Math** (Equation 3 from the CADJ paper):
  ```
  n_perp = (n_i / ||n_i||) x (n_j / ||n_j||)     # cross product
  d = (x_i - x_j) . (n_perp / ||n_perp||)         # signed distance
  ```
- **Returns**: signed scalar distance

**Reference**: `support_materials/FrameX/python/multi_tangent/smilp/util.py` lines 35-45.

### Function 3: `closest_params_finite_segments(p1_start, p1_end, p2_start, p2_end)`

Computes closest-point parameters for two **finite-length** line segments, with `t in [0, 1]`.

- **Algorithm** (from `contact.py:39-81`):
  1. Set `d1 = p1_end - p1_start`, `d2 = p2_end - p2_start`
  2. Build matrix `A = [[d1.d1, -d1.d2], [d2.d1, -d2.d2]]`
  3. If `|det(A)| > tol`: solve `A @ [t1, t2] = [(p2_start - p1_start).d1, (p2_start - p1_start).d2]`
     - If both `t1, t2 in [0,1]`: return `(t1, t2)` (interior solution)
  4. Boundary search: try all 4 candidates `(0, ?), (1, ?), (?, 0), (?, 1)`, project the fixed endpoint onto the other segment, clip to `[0,1]`, pick pair with minimum distance.
- **Returns**: `(t1, t2)` where closest points are `p1_start + t1 * d1` and `p2_start + t2 * d2`

**Reference**: `support_materials/FrameX/python/multi_tangent/contact.py` lines 39-81. Port this function directly, replacing `convex_combination` and `compute_closest_point_to_line` as inline helpers.

### Function 4: `are_lines_parallel(n_i, n_j, tol=1e-6)`

- **Math**: `||n_i x n_j|| / (||n_i|| * ||n_j||) < tol`
- **Returns**: bool

---

## Module 3: `core/kinematics.py` - Joint FK and Optimization

### Overview of the kinematic chain

Each connector pair between bars L_e and L_n has 5 DOFs:
- **FJP** (float): Female joint prismatic - slide along L_e axis
- **FJR** (float): Female joint revolute - rotate around L_e axis
- **MJP** (float): Male joint prismatic - slide along L_n axis
- **MJR** (float): Male joint revolute - rotate around L_n axis
- **JJR** (float): Joint-joint rotation - rotation at the male-female interface

### OCF Conventions (from specs page 3)

**Bar frame**: For a bar line defined by two endpoints, the frame is:
- Origin: midpoint of the line
- Z-axis: along the bar longitudinal axis (unit direction vector)
- X-axis, Y-axis: perpendicular to Z, initially arbitrary (set by a reference direction, e.g., global Z crossed with bar direction, with fallback for vertical bars)

**Female joint OCF**:
- Origin: projection of screw hole center onto bar axis
- X: along bar axis
- Y: parallel to brep edge
- Z: same direction as male joint screwing direction (points from female bar towards male bar)

**Male joint OCF**:
- Origin: midpoint between M_P1 and M_P2
- X: along bar axis
- Y: parallel to brep edge
- Z: screw insertion direction (points from male bar towards female bar)

### Helper: `make_bar_frame(line_start, line_end)`

Given a bar's two endpoints, compute the bar's 4x4 homogeneous frame:
- Origin = midpoint
- Z = unit direction from start to end
- X = perpendicular to Z (use `cross(world_Z, bar_Z)` normalized; if bar is nearly vertical, use `cross(world_X, bar_Z)` instead)
- Y = cross(Z, X)
- Return 4x4 matrix `[X Y Z origin; 0 0 0 1]`

### Helper: `translate_along_axis(frame, axis, distance)`

Returns a 4x4 transform that translates along `axis` by `distance`, applied to `frame`.

### Helper: `rotate_around_axis(frame, axis, angle)`

Returns a 4x4 transform that rotates around `axis` by `angle` (Rodrigues' formula), applied to `frame`.

### Function: `fk_female(bar_frame_e, fjp, fjr, jjr, contact_normal_dir)`

Forward kinematics for the female joint chain:
1. Start from `bar_frame_e` (the frame of bar L_e, but at the contact point, not the bar midpoint)
   - Actually: start from L_e's axis. The FJP parameter gives the position along L_e's axis relative to L_e's anchor/start point.
2. Translate along L_e's Z-axis by `fjp` → position on L_e
3. Rotate around L_e's Z-axis by `fjr` → orient the female joint around the bar
4. Apply female connector offset: translate by `FEMALE_RADIAL_OFFSET` along the contact normal direction (perpendicular to bar, towards the other bar)
5. Rotate around the female OCF's Z-axis by `jjr` → joint-joint rotation
6. Return the resulting 4x4 frame

**Implementation detail**: At step 2, after translating to position on L_e, we have a frame on the bar axis. At step 3, FJR rotates the "exit direction" around the bar. The contact normal direction (pointing towards Ln) is determined by this rotation. At step 4, we offset along that direction.

More precisely:
```python
def fk_female(Le_start, Le_end, fjp, fjr, jjr, config):
    # Bar direction
    Le_dir = (Le_end - Le_start)
    Le_len = norm(Le_dir)
    Le_unit = Le_dir / Le_len

    # Position on Le at parameter fjp (measured from Le_start)
    pos_on_Le = Le_start + fjp * Le_unit

    # Build local frame at pos_on_Le
    Z = Le_unit  # along bar
    X_ref = perpendicular_to(Z)  # arbitrary perpendicular
    X = X_ref
    Y = cross(Z, X)

    # Rotate X, Y around Z by fjr
    X_rot = cos(fjr) * X + sin(fjr) * Y
    Y_rot = -sin(fjr) * X + cos(fjr) * Y

    # Female OCF: offset along the rotated X direction (radial offset towards other bar)
    female_origin = pos_on_Le + config.FEMALE_RADIAL_OFFSET * X_rot

    # Female OCF axes:
    # X_f = Z (along bar)
    # Z_f = X_rot (towards male joint / screw direction)
    # Y_f = Y_rot
    # Note: match the OCF convention from specs - X along bar, Z along screw direction
    X_f = Z
    Z_f = X_rot
    Y_f = cross(Z_f, X_f)

    # Apply JJR: rotate around Z_f by jjr
    X_jjr = cos(jjr) * X_f + sin(jjr) * Y_f
    Y_jjr = -sin(jjr) * X_f + cos(jjr) * Y_f

    # Build 4x4 frame
    frame = np.eye(4)
    frame[:3, 0] = X_jjr
    frame[:3, 1] = Y_jjr
    frame[:3, 2] = Z_f
    frame[:3, 3] = female_origin
    return frame
```

### Function: `fk_male(Ln_start, Ln_end, mjp, mjr, config)`

Forward kinematics for the male joint chain:
```python
def fk_male(Ln_start, Ln_end, mjp, mjr, config):
    Ln_dir = (Ln_end - Ln_start)
    Ln_unit = Ln_dir / norm(Ln_dir)

    pos_on_Ln = Ln_start + mjp * Ln_unit

    Z = Ln_unit
    X_ref = perpendicular_to(Z)  # same method as female
    X = X_ref
    Y = cross(Z, X)

    # Rotate X, Y around Z by mjr
    X_rot = cos(mjr) * X + sin(mjr) * Y
    Y_rot = -sin(mjr) * X + cos(mjr) * Y

    # Male OCF: offset along the rotated X direction
    male_origin = pos_on_Ln + config.MALE_RADIAL_OFFSET * X_rot

    # Male OCF axes:
    X_m = Z
    Z_m = X_rot  # screw insertion direction (towards female)
    Y_m = cross(Z_m, X_m)

    frame = np.eye(4)
    frame[:3, 0] = X_m
    frame[:3, 1] = Y_m
    frame[:3, 2] = Z_m
    frame[:3, 3] = male_origin
    return frame
```

### Function: `frame_distance(frame_a, frame_b)`

Compute a combined position + orientation error between two 4x4 frames.

```python
def frame_distance(frame_a, frame_b):
    # Position error (L2)
    pos_err = np.sum((frame_a[:3, 3] - frame_b[:3, 3]) ** 2)

    # Orientation error: sum of squared distances between corresponding axis vectors
    orient_err = 0.0
    for i in range(3):  # X, Y, Z axes
        orient_err += np.sum((frame_a[:3, i] - frame_b[:3, i]) ** 2)

    return pos_err + orient_err
```

**Note from specs (page 4)**: "I suggest we do a sum of origin L2 distance and vector-vector distance for each pair of x,y,z axes." This is exactly what the function above computes.

### Function: `optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)`

Main optimization for S1-T2. Finds optimal DOF values.

```python
from scipy.optimize import minimize

def optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config):
    def objective(params):
        fjp, fjr, mjp, mjr, jjr = params
        f_female = fk_female(Le_start, Le_end, fjp, fjr, jjr, config)
        f_male = fk_male(Ln_start, Ln_end, mjp, mjr, config)
        return frame_distance(f_female, f_male)

    # Initial guess: closest points between the two lines
    t_e, t_n = closest_params_infinite_lines(Le_start, Le_end - Le_start, Ln_start, Ln_end - Ln_start)
    fjp0 = t_e * norm(Le_end - Le_start)
    mjp0 = t_n * norm(Ln_end - Ln_start)

    x0 = [fjp0, 0.0, mjp0, 0.0, 0.0]
    bounds = [config.FJP_RANGE, config.FJR_RANGE, config.MJP_RANGE, config.MJR_RANGE, config.JJR_RANGE]

    result = minimize(objective, x0, method='L-BFGS-B', bounds=bounds)

    if result.fun > 1e-3:
        # Try multiple random restarts
        best = result
        for _ in range(10):
            x0_rand = [fjp0 + np.random.uniform(-50, 50),
                        np.random.uniform(-np.pi, np.pi),
                        mjp0 + np.random.uniform(-50, 50),
                        np.random.uniform(-np.pi, np.pi),
                        np.random.uniform(-np.pi, np.pi)]
            res = minimize(objective, x0_rand, method='L-BFGS-B', bounds=bounds)
            if res.fun < best.fun:
                best = res
        result = best

    fjp, fjr, mjp, mjr, jjr = result.x
    return {
        'fjp': fjp, 'fjr': fjr, 'mjp': mjp, 'mjr': mjr, 'jjr': jjr,
        'residual': result.fun,
        'female_frame': fk_female(Le_start, Le_end, fjp, fjr, jjr, config),
        'male_frame': fk_male(Ln_start, Ln_end, mjp, mjr, config),
    }
```

### Important note on `perpendicular_to(Z)`

This helper must produce a **consistent** perpendicular vector for a given bar direction. Use:
```python
def perpendicular_to(z_axis):
    """Return a unit vector perpendicular to z_axis."""
    z = z_axis / np.linalg.norm(z_axis)
    # Use world Z as reference, fall back to world X for near-vertical bars
    if abs(np.dot(z, [0, 0, 1])) < 0.95:
        x = np.cross([0, 0, 1], z)
    else:
        x = np.cross([1, 0, 0], z)
    x = x / np.linalg.norm(x)
    return x
```

---

## Script 1: `t1_bar_axis.py` - Bar Axis Line Generation

### Entry point logic

```python
import rhinoscriptsyntax as rs
import scriptcontext as sc
import sys, os

# Add core to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'core'))
from geometry import closest_params_infinite_lines, distance_infinite_lines, are_lines_parallel
from config import BAR_CONTACT_DISTANCE, DEFAULT_NEW_BAR_LENGTH

def main(rerun=False):
    if rerun and "t1_inputs" in sc.sticky:
        inputs = sc.sticky["t1_inputs"]
    else:
        inputs = get_inputs()
        if inputs is None:
            return
        sc.sticky["t1_inputs"] = inputs

    if inputs["mode"] == "S1":
        run_s1_t1(inputs)
    elif inputs["mode"] == "S2":
        run_s2_t1(inputs)

main(rerun=False)  # or rerun=True in the rerun wrapper script
```

### Input collection: `get_inputs()`

```python
def get_inputs():
    rs.UnselectAllObjects()

    # Ask the user which mode
    mode = rs.ListBox(["S1: Join to ONE bar", "S2: Join to TWO bars"],
                       "Select connection mode:", "T1 - Bar Axis Generation")
    if mode is None:
        return None

    if "ONE" in mode:
        # S1: select existing bar Le, then new bar Ln
        le_id = rs.GetObject("Select existing bar (Le)", rs.filter.curve)
        if le_id is None: return None
        ln_id = rs.GetObject("Select new bar (Ln) - will be repositioned", rs.filter.curve)
        if ln_id is None: return None
        return {"mode": "S1", "le_id": le_id, "ln_id": ln_id}

    else:
        # S2: select Le1, Le2, then pick contact points Ce1, Ce2
        le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
        if le1_id is None: return None
        le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
        if le2_id is None: return None
        ce1 = rs.GetPointOnCurve(le1_id, "Pick contact point on Le1 (Ce1)")
        if ce1 is None: return None
        ce2 = rs.GetPointOnCurve(le2_id, "Pick contact point on Le2 (Ce2)")
        if ce2 is None: return None
        return {"mode": "S2", "le1_id": le1_id, "le2_id": le2_id, "ce1": ce1, "ce2": ce2}
```

### S1-T1 Algorithm: `run_s1_t1(inputs)`

**Goal**: Move Ln so that the shortest distance between infinite lines Ln and Le equals `D = BAR_CONTACT_DISTANCE`.

```python
def run_s1_t1(inputs):
    D = BAR_CONTACT_DISTANCE

    # Extract line geometry
    Le_start = np.array(rs.CurveStartPoint(inputs["le_id"]))
    Le_end = np.array(rs.CurveEndPoint(inputs["le_id"]))
    Ln_start = np.array(rs.CurveStartPoint(inputs["ln_id"]))
    Ln_end = np.array(rs.CurveEndPoint(inputs["ln_id"]))

    n_e = Le_end - Le_start  # direction of Le
    n_n = Ln_end - Ln_start  # direction of Ln

    # Check parallel
    if are_lines_parallel(n_e, n_n):
        rs.MessageBox("Error: bars are parallel. Skew bars required.")
        return

    # Compute closest point parameters on infinite lines (Eq 17)
    t_e, t_n = closest_params_infinite_lines(Le_start, n_e, Ln_start, n_n)

    # Closest points
    p_e = Le_start + t_e * n_e  # point on Le
    p_n = Ln_start + t_n * n_n  # point on Ln

    # Shortest segment vector (from Ln towards Le)
    seg = p_e - p_n
    current_dist = np.linalg.norm(seg)
    seg_unit = seg / current_dist

    # Translation needed: move Ln so distance becomes D
    # New p_n should be at: p_e - D * seg_unit
    # So translation = (p_e - D * seg_unit) - p_n
    translation = seg_unit * (current_dist - D)

    # Move Ln
    new_Ln_start = Ln_start + translation
    new_Ln_end = Ln_end + translation

    # Add new line to Rhino (centered, with default length)
    mid = (new_Ln_start + new_Ln_end) / 2
    n_n_unit = n_n / np.linalg.norm(n_n)
    half_len = DEFAULT_NEW_BAR_LENGTH / 2
    line_id = rs.AddLine(mid - half_len * n_n_unit, mid + half_len * n_n_unit)

    # Optionally delete old Ln
    rs.DeleteObject(inputs["ln_id"])

    rs.SelectObject(line_id)
    print(f"S1-T1: Bar placed at distance {D:.2f} from Le")
```

**Key math summary for S1-T1**:
1. Use `closest_params_infinite_lines` (Eq 17) to find closest approach points
2. Compute the perpendicular segment between the two lines
3. Translate Ln along this segment so the distance equals D
4. Output a new line at the translated position with default length

### S2-T1 Algorithm: `run_s2_t1(inputs)`

**Goal**: Find direction `nn` and position for Ln such that:
- Shortest segment from Ln to Le1 passes through Ce1 (on Le1 side), length = D
- Shortest segment from Ln to Le2 passes through Ce2 (on Le2 side), length = D

**Mathematical formulation**:

Given `nn` (unit direction of Ln):
1. Contact normal from Ln to Le1: `cn1 = nn x n1 / ||nn x n1||`
2. Contact normal from Ln to Le2: `cn2 = nn x n2 / ||nn x n2||`
3. Point on Ln near Le1: `P1 = Ce1 + D * cn1` (sign TBD)
4. Point on Ln near Le2: `P2 = Ce2 + D * cn2` (sign TBD)
5. Constraint: `(P2 - P1)` must be parallel to `nn`, i.e., `(P2 - P1) x nn = 0`

**Optimization variables**: `theta, phi` parameterizing `nn = (sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta))`

**Objective**: `|| (P2 - P1) x nn ||^2`

**Sign resolution**: There are 4 sign combinations (`+D` or `-D` for each contact normal). Try all 4, pick the one that converges to the smallest residual.

```python
from scipy.optimize import minimize

def run_s2_t1(inputs):
    D = BAR_CONTACT_DISTANCE

    # Extract geometry
    Le1_start = np.array(rs.CurveStartPoint(inputs["le1_id"]))
    Le1_end = np.array(rs.CurveEndPoint(inputs["le1_id"]))
    Le2_start = np.array(rs.CurveStartPoint(inputs["le2_id"]))
    Le2_end = np.array(rs.CurveEndPoint(inputs["le2_id"]))
    Ce1 = np.array(inputs["ce1"])
    Ce2 = np.array(inputs["ce2"])

    n1 = Le1_end - Le1_start
    n1 = n1 / np.linalg.norm(n1)
    n2 = Le2_end - Le2_start
    n2 = n2 / np.linalg.norm(n2)

    # Initial guess for nn: direction from Ce1 to Ce2 (rough approximation)
    init_dir = Ce2 - Ce1
    init_dir = init_dir / np.linalg.norm(init_dir)
    theta0 = np.arccos(np.clip(init_dir[2], -1, 1))
    phi0 = np.arctan2(init_dir[1], init_dir[0])

    best_result = None
    best_cost = np.inf

    for s1 in [+1, -1]:
        for s2 in [+1, -1]:
            def objective(params):
                theta, phi = params
                nn = np.array([np.sin(theta)*np.cos(phi),
                               np.sin(theta)*np.sin(phi),
                               np.cos(theta)])

                cn1 = np.cross(nn, n1)
                cn1_norm = np.linalg.norm(cn1)
                if cn1_norm < 1e-10:
                    return 1e10  # nn parallel to n1, infeasible
                cn1 = cn1 / cn1_norm

                cn2 = np.cross(nn, n2)
                cn2_norm = np.linalg.norm(cn2)
                if cn2_norm < 1e-10:
                    return 1e10
                cn2 = cn2 / cn2_norm

                P1 = Ce1 + s1 * D * cn1
                P2 = Ce2 + s2 * D * cn2

                diff = P2 - P1
                cross_val = np.cross(diff, nn)
                return np.dot(cross_val, cross_val)

            res = minimize(objective, [theta0, phi0], method='Nelder-Mead',
                          options={'xatol': 1e-12, 'fatol': 1e-12, 'maxiter': 10000})
            if res.fun < best_cost:
                best_cost = res.fun
                best_result = (res.x, s1, s2)

    if best_cost > 1e-4:
        rs.MessageBox("Warning: S2-T1 optimization did not converge well. "
                       f"Residual: {best_cost:.6f}")

    theta, phi = best_result[0]
    s1, s2 = best_result[1], best_result[2]
    nn = np.array([np.sin(theta)*np.cos(phi),
                   np.sin(theta)*np.sin(phi),
                   np.cos(theta)])

    cn1 = np.cross(nn, n1)
    cn1 = cn1 / np.linalg.norm(cn1)
    cn2 = np.cross(nn, n2)
    cn2 = cn2 / np.linalg.norm(cn2)
    P1 = Ce1 + s1 * D * cn1
    P2 = Ce2 + s2 * D * cn2

    # Ln passes through P1 and P2 with direction nn
    mid = (P1 + P2) / 2
    half_len = DEFAULT_NEW_BAR_LENGTH / 2
    line_id = rs.AddLine(mid - half_len * nn, mid + half_len * nn)

    rs.SelectObject(line_id)
    print(f"S2-T1: Bar placed with direction ({nn[0]:.3f}, {nn[1]:.3f}, {nn[2]:.3f})")
```

---

## Script 2: `t2_joint_placement.py` - Joint Placement

### Entry point logic

```python
def main(rerun=False):
    if rerun and "t2_inputs" in sc.sticky:
        inputs = sc.sticky["t2_inputs"]
    else:
        inputs = get_inputs_t2()
        if inputs is None:
            return
        sc.sticky["t2_inputs"] = inputs

    if inputs["mode"] == "S1":
        result = run_s1_t2(inputs)
        place_joint_blocks(result, inputs)
    elif inputs["mode"] == "S2":
        result1 = run_s1_t2({"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
        result2 = run_s1_t2({"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})
        place_joint_blocks(result1, {"le_id": inputs["le1_id"], "ln_id": inputs["ln_id"]})
        place_joint_blocks(result2, {"le_id": inputs["le2_id"], "ln_id": inputs["ln_id"]})
```

### Input collection: `get_inputs_t2()`

```python
def get_inputs_t2():
    mode = rs.ListBox(["S1: ONE connection", "S2: TWO connections"],
                       "Select connection mode:", "T2 - Joint Placement")
    if mode is None:
        return None

    if "ONE" in mode:
        le_id = rs.GetObject("Select existing bar (Le) - gets MALE joint", rs.filter.curve)
        ln_id = rs.GetObject("Select new bar (Ln) - gets FEMALE joint", rs.filter.curve)
        return {"mode": "S1", "le_id": le_id, "ln_id": ln_id}
    else:
        ln_id = rs.GetObject("Select new bar (Ln)", rs.filter.curve)
        le1_id = rs.GetObject("Select first existing bar (Le1)", rs.filter.curve)
        le2_id = rs.GetObject("Select second existing bar (Le2)", rs.filter.curve)
        return {"mode": "S2", "ln_id": ln_id, "le1_id": le1_id, "le2_id": le2_id}
```

### `run_s1_t2(inputs)` - calls `optimize_joint_placement` from `kinematics.py`

Extracts line geometry from Rhino objects, calls the optimizer, returns the result dict.

### `place_joint_blocks(result, inputs)` - Block instance placement

```python
def place_joint_blocks(result, inputs):
    """Place FemaleJoint and MaleJoint block instances at the optimized positions."""
    import Rhino
    import json

    female_frame = result['female_frame']  # 4x4 numpy array
    male_frame = result['male_frame']      # 4x4 numpy array

    # Convert numpy 4x4 to Rhino Transform
    def numpy_to_rhino_transform(m):
        xform = Rhino.Geometry.Transform(1.0)
        for i in range(4):
            for j in range(4):
                xform[i, j] = float(m[i, j])
        return xform

    # Check if block definitions exist, otherwise create placeholder
    def ensure_block_def(name, color):
        if not rs.IsBlock(name):
            # Create a simple placeholder box
            box = rs.AddBox([
                [-10, -10, -10], [10, -10, -10], [10, 10, -10], [-10, 10, -10],
                [-10, -10, 10], [10, -10, 10], [10, 10, 10], [-10, 10, 10]
            ])
            rs.ObjectColor(box, color)
            rs.AddBlock([box], [0, 0, 0], name, delete_input=True)

    ensure_block_def("FemaleJoint", (255, 100, 100))  # red-ish
    ensure_block_def("MaleJoint", (100, 255, 100))     # green-ish

    # Place female joint block instance
    xform_f = numpy_to_rhino_transform(female_frame)
    female_id = rs.InsertBlock("FemaleJoint", [0, 0, 0])
    rs.TransformObject(female_id, xform_f)

    # Place male joint block instance
    xform_m = numpy_to_rhino_transform(male_frame)
    male_id = rs.InsertBlock("MaleJoint", [0, 0, 0])
    rs.TransformObject(male_id, xform_m)

    # Store JSON attribute user text
    joint_data_female = {
        "type": "FemaleJoint",
        "dof": {"fjp": result['fjp'], "fjr": result['fjr'], "jjr": result['jjr']},
        "bar_id": str(inputs["le_id"]),
        "connected_bar_id": str(inputs["ln_id"]),
        "transform": female_frame.tolist(),
        "residual": result['residual']
    }
    joint_data_male = {
        "type": "MaleJoint",
        "dof": {"mjp": result['mjp'], "mjr": result['mjr']},
        "bar_id": str(inputs["ln_id"]),
        "connected_bar_id": str(inputs["le_id"]),
        "transform": male_frame.tolist(),
        "residual": result['residual']
    }

    rs.SetUserText(female_id, "joint_data", json.dumps(joint_data_female, indent=2))
    rs.SetUserText(male_id, "joint_data", json.dumps(joint_data_male, indent=2))

    print(f"T2: Joints placed. Residual: {result['residual']:.6f}")
    print(f"  FJP={result['fjp']:.2f}, FJR={np.degrees(result['fjr']):.1f}deg, "
          f"MJP={result['mjp']:.2f}, MJR={np.degrees(result['mjr']):.1f}deg, "
          f"JJR={np.degrees(result['jjr']):.1f}deg")
```

### Rerun wrapper scripts

**`t1_bar_axis_rerun.py`**:
```python
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "t1_bar_axis.py")).read().replace(
    "main(rerun=False)", "main(rerun=True)"))
```

Better approach: define a flag variable that the main script checks:
```python
# t1_bar_axis_rerun.py
import scriptcontext as sc
sc.sticky["t1_rerun_flag"] = True
exec(open(r"<same_dir>/t1_bar_axis.py").read())
```

And in t1_bar_axis.py, at the bottom:
```python
rerun = sc.sticky.pop("t1_rerun_flag", False)
main(rerun=rerun)
```

---

## Rhino Toolbar Button Setup

Two buttons on the Rhino toolbar:

**Button 1: "T1 Bar Axis"**
- Left-click macro: `! _-RunPythonScript "C:\<path>\scripts\t1_bar_axis.py"`
- Right-click macro: `! _-RunPythonScript "C:\<path>\scripts\t1_bar_axis_rerun.py"`

**Button 2: "T2 Joint Place"**
- Left-click macro: `! _-RunPythonScript "C:\<path>\scripts\t2_joint_placement.py"`
- Right-click macro: `! _-RunPythonScript "C:\<path>\scripts\t2_joint_placement_rerun.py"`

---

## Key Reference Files (for implementation)

| What | File | Lines |
|------|------|-------|
| Closest t between finite segments | `support_materials/FrameX/python/multi_tangent/contact.py` | 39-81 |
| Infinite-line signed distance | `support_materials/FrameX/python/multi_tangent/smilp/util.py` | 35-45 |
| Infinite-line closest t params | `support_materials/FrameX/python/multi_tangent/smilp/util.py` | 47-73 |
| Contact point projection | `support_materials/FrameX/python/multi_tangent/contact.py` | 29-36 |

---

## Self-Contained Tests (pytest, no Rhino needed)

All tests below use only numpy and scipy. Run with `pytest tests/` from the project root.
The tests directory must be able to import `scripts/core/` (add it to `sys.path` or use a conftest.py).

**IMPORTANT**: These tests validate the **math** independently of Rhino. The FrameX codebase tests are NOT applicable because their joints use swivel couplers (rotation axis = contact normal), whereas our T20-5 connectors have a 5-DOF kinematic chain (FJP, FJR, MJP, MJR, JJR) as described in the specs.

---

### `tests/conftest.py`

```python
import sys, os
# Allow tests to import scripts/core/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
```

---

### `tests/test_geometry.py` - Line geometry math

All expected values below are hand-computed. Each test documents the derivation.

```python
import numpy as np
import pytest
from core.geometry import (
    closest_params_infinite_lines,
    distance_infinite_lines,
    closest_params_finite_segments,
    are_lines_parallel,
)

TOL = 1e-10

# ============================================================
# closest_params_infinite_lines
# ============================================================

class TestClosestParamsInfiniteLines:
    def test_perpendicular_at_origins(self):
        """Two perpendicular lines through the origin, offset along Z.
        L1: (0,0,0) + t*(1,0,0)   (X-axis)
        L2: (0,0,5) + t*(0,1,0)   (Y-axis at Z=5)
        Closest points: (0,0,0) and (0,0,5), so tA=0, tB=0.
        """
        tA, tB = closest_params_infinite_lines(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([0.,0,5]), np.array([0.,1,0]))
        assert abs(tA) < TOL
        assert abs(tB) < TOL

    def test_skew_lines_offset(self):
        """Skew lines with known analytical solution.
        L1: x=(0,0,0), n=(1,0,0)
        L2: x=(1,2,3), n=(0,1,0)

        Derivation (minimize ||P1-P2||^2):
          P1=(t,0,0), P2=(1,2+s,3)
          d/dt: 2(t-1)=0 -> t=1
          d/ds: 2(2+s)=0 -> s=-2

        Using the formula:
          Mat = [[1, 0],[0, -1]], b = [(1,2,3).(1,0,0), (1,2,3).(0,1,0)] = [1, 2]
          T = Mat^{-1} @ b = [1, -2]
        """
        tA, tB = closest_params_infinite_lines(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([1.,2,3]), np.array([0.,1,0]))
        assert abs(tA - 1.0) < TOL
        assert abs(tB - (-2.0)) < TOL
        # Verify closest points
        pA = np.array([0,0,0]) + tA * np.array([1,0,0])  # (1,0,0)
        pB = np.array([1,2,3]) + tB * np.array([0,1,0])  # (1,0,3)
        np.testing.assert_allclose(pA, [1, 0, 0], atol=TOL)
        np.testing.assert_allclose(pB, [1, 0, 3], atol=TOL)
        assert abs(np.linalg.norm(pA - pB) - 3.0) < TOL

    def test_45_degree_lines(self):
        """Two lines at 45 degrees.
        L1: x=(0,0,0), n=(1,0,0)
        L2: x=(5,3,4), n=(1,1,0) (not unit - should still work)

        Mat = [[1, -1], [-1, 2]], b = [(5,3,4).(1,0,0), (5,3,4).(1,1,0)] = [5, 8]
        det = 2-1 = 1
        Mat^{-1} = [[2, 1],[1, 1]]
        T = [2*5+1*8, 1*5+1*8] = [18, 13]
        pA = (18, 0, 0), pB = (5+13, 3+13, 4) = (18, 16, 4)
        dist = ||(0, -16, -4)|| = sqrt(272)
        """
        tA, tB = closest_params_infinite_lines(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([5.,3,4]), np.array([1.,1,0]))
        assert abs(tA - 18.0) < TOL
        assert abs(tB - 13.0) < TOL

    def test_symmetric_config(self):
        """Two lines symmetric about the origin.
        L1: x=(0,0,-D/2), n=(1,0,0)
        L2: x=(0,0,+D/2), n=(0,1,0)
        With D=20, closest points at origins of each line: tA=0, tB=0, dist=20.
        """
        D = 20.0
        tA, tB = closest_params_infinite_lines(
            np.array([0,0,-D/2]), np.array([1.,0,0]),
            np.array([0,0,+D/2]), np.array([0.,1,0]))
        assert abs(tA) < TOL
        assert abs(tB) < TOL


# ============================================================
# distance_infinite_lines
# ============================================================

class TestDistanceInfiniteLines:
    def test_perpendicular_z_separated(self):
        """Same as closest_params test 1. Distance should be 5."""
        d = distance_infinite_lines(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([0.,0,5]), np.array([0.,1,0]))
        assert abs(abs(d) - 5.0) < TOL

    def test_skew_offset(self):
        """L1: x=(0,0,0) n=(1,0,0), L2: x=(1,2,3) n=(0,1,0).
        Perpendicular direction: (1,0,0)x(0,1,0) = (0,0,1).
        Signed dist = (x1-x2).(0,0,1) = (0-3) = -3. |d|=3.
        """
        d = distance_infinite_lines(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([1.,2,3]), np.array([0.,1,0]))
        assert abs(abs(d) - 3.0) < TOL

    def test_non_unit_directions(self):
        """Same geometry but with scaled direction vectors. Result should be identical."""
        d = distance_infinite_lines(
            np.array([0.,0,0]), np.array([5.,0,0]),   # scaled by 5
            np.array([0.,0,7]), np.array([0.,3,0]))   # scaled by 3
        assert abs(abs(d) - 7.0) < TOL


# ============================================================
# closest_params_finite_segments
# ============================================================

class TestClosestParamsFiniteSegments:
    def test_interior_solution(self):
        """Two segments whose infinite-line closest point is within both segments.
        Seg1: (0,0,0)-(4,0,0)   (along X, length 4)
        Seg2: (2,0,3)-(2,4,3)   (along Y at x=2, z=3)

        d1=(4,0,0), d2=(0,4,0)
        A = [[16, 0],[0, -16]], b = [(2,0,3).(4,0,0), (2,0,3).(0,4,0)] = [8, 0]
        t = [0.5, 0.0]  -> both in [0,1]
        Closest pts: (2,0,0) and (2,0,3). Dist=3.
        """
        t1, t2 = closest_params_finite_segments(
            np.array([0.,0,0]), np.array([4.,0,0]),
            np.array([2.,0,3]), np.array([2.,4,3]))
        assert abs(t1 - 0.5) < TOL
        assert abs(t2 - 0.0) < TOL

    def test_boundary_clipping(self):
        """Infinite-line solution is outside segment bounds -> boundary case.
        Seg1: (0,0,0)-(1,0,0)   (short segment along X)
        Seg2: (5,0,2)-(5,4,2)   (along Y, far away at x=5)

        Infinite solution: tA=5 (out of [0,1]).
        Boundary search should find t1=1 (closest end of seg1 to seg2).
        Project (1,0,0) onto seg2: closest at (5,0,2), t2=0.
        Distance = ||(1,0,0)-(5,0,2)|| = sqrt(16+4) = sqrt(20).
        """
        t1, t2 = closest_params_finite_segments(
            np.array([0.,0,0]), np.array([1.,0,0]),
            np.array([5.,0,2]), np.array([5.,4,2]))
        assert abs(t1 - 1.0) < TOL
        assert abs(t2 - 0.0) < TOL
        p1 = np.array([0,0,0]) + t1 * np.array([1,0,0])
        p2 = np.array([5,0,2]) + t2 * np.array([0,4,2])
        # p2 should be (5,0,2) since t2=0
        np.testing.assert_allclose(p2, [5, 0, 2], atol=TOL)
        assert abs(np.linalg.norm(p1 - p2) - np.sqrt(20)) < 1e-6

    def test_coincides_with_infinite_when_inside(self):
        """When infinite-line solution is inside both segments,
        finite and infinite should agree."""
        x1, n1 = np.array([0.,0,0]), np.array([1.,0,0])
        x2, n2 = np.array([0.,0,5]), np.array([0.,1,0])
        # Infinite solution
        tA_inf, tB_inf = closest_params_infinite_lines(x1, n1, x2, n2)
        # Finite with segments long enough to contain tA=0, tB=0
        t1, t2 = closest_params_finite_segments(
            x1 - 10*n1, x1 + 10*n1,
            x2 - 10*n2, x2 + 10*n2)
        # Convert finite t in [0,1] to the same parameterization
        # Seg1 goes from x1-10*n1 to x1+10*n1, so t=0.5 corresponds to x1
        assert abs(t1 - 0.5) < TOL
        assert abs(t2 - 0.5) < TOL


# ============================================================
# are_lines_parallel
# ============================================================

class TestAreLinesParallel:
    def test_parallel(self):
        assert are_lines_parallel(np.array([1,0,0.]), np.array([2,0,0.]))
        assert are_lines_parallel(np.array([1,0,0.]), np.array([-3,0,0.]))

    def test_not_parallel(self):
        assert not are_lines_parallel(np.array([1,0,0.]), np.array([0,1,0.]))
        assert not are_lines_parallel(np.array([1,0,0.]), np.array([1,1,0.]))

    def test_near_parallel(self):
        """Slightly off-parallel should still be detected as parallel."""
        assert are_lines_parallel(np.array([1,0,0.]), np.array([1, 1e-8, 0.]))
```

---

### `tests/test_kinematics.py` - FK chains and joint optimization

These tests validate the FK formulation and the optimizer independently of any Rhino geometry. They use the connector parameters directly.

```python
import numpy as np
import pytest
from core.kinematics import (
    fk_female,
    fk_male,
    frame_distance,
    optimize_joint_placement,
    perpendicular_to,
)
from core import config

TOL_POSITION = 1e-4    # mm
TOL_ORIENTATION = 1e-4 # unitless (axis vector difference)
TOL_RESIDUAL = 1e-3    # combined frame_distance

# ============================================================
# perpendicular_to
# ============================================================

class TestPerpendicularTo:
    def test_result_is_unit(self):
        for z in [[1,0,0], [0,1,0], [0,0,1], [1,1,0], [1,1,1], [0.1, 0.2, 0.95]]:
            x = perpendicular_to(np.array(z, dtype=float))
            assert abs(np.linalg.norm(x) - 1.0) < 1e-12

    def test_result_is_perpendicular(self):
        for z in [[1,0,0], [0,1,0], [0,0,1], [1,1,0], [1,1,1]]:
            z = np.array(z, dtype=float)
            x = perpendicular_to(z)
            assert abs(np.dot(x, z / np.linalg.norm(z))) < 1e-12

    def test_deterministic(self):
        """Same input must always produce same output."""
        z = np.array([0.3, 0.5, 0.7])
        x1 = perpendicular_to(z)
        x2 = perpendicular_to(z)
        np.testing.assert_array_equal(x1, x2)


# ============================================================
# frame_distance
# ============================================================

class TestFrameDistance:
    def test_identical_frames(self):
        f = np.eye(4)
        assert frame_distance(f, f) < 1e-15

    def test_translation_only(self):
        """Two frames differing only by a translation of (3,4,0).
        Position error = 3^2 + 4^2 = 25. Orientation error = 0.
        """
        f1 = np.eye(4)
        f2 = np.eye(4)
        f2[:3, 3] = [3, 4, 0]
        assert abs(frame_distance(f1, f2) - 25.0) < 1e-12

    def test_rotation_only(self):
        """Two frames at same position, one rotated 90 deg around Z.
        Axes after rotation: X->(0,1,0), Y->(-1,0,0), Z->(0,0,1).
        X error: ||(1,0,0)-(0,1,0)||^2 = 2
        Y error: ||(0,1,0)-(-1,0,0)||^2 = 2
        Z error: 0
        Total orientation error = 4. Position error = 0.
        """
        f1 = np.eye(4)
        f2 = np.eye(4)
        f2[:3, 0] = [0, 1, 0]   # X' = Y
        f2[:3, 1] = [-1, 0, 0]  # Y' = -X
        f2[:3, 2] = [0, 0, 1]   # Z' = Z
        assert abs(frame_distance(f1, f2) - 4.0) < 1e-12


# ============================================================
# FK sanity checks
# ============================================================

class TestFKSanity:
    """These tests verify structural properties of FK, not specific numerical values,
    because exact values depend on the perpendicular_to convention."""

    def test_fk_female_origin_on_correct_side(self):
        """Female OCF origin should be offset from bar axis by FEMALE_RADIAL_OFFSET."""
        Le_start = np.array([-100., 0, 0])
        Le_end = np.array([100., 0, 0])
        fjp, fjr, jjr = 50.0, 0.0, 0.0  # 50mm from start, no rotations
        frame = fk_female(Le_start, Le_end, fjp, fjr, jjr, config)
        # Origin should be at x = -100 + 50 = -50 along bar, plus some radial offset
        # The X component of origin should be -50 (position along bar)
        assert abs(frame[0, 3] - (-50.0)) < TOL_POSITION
        # Distance from bar axis = FEMALE_RADIAL_OFFSET
        bar_axis_pt = Le_start + (fjp / np.linalg.norm(Le_end - Le_start)) * (Le_end - Le_start)
        radial_dist = np.linalg.norm(frame[:3, 3] - bar_axis_pt)
        assert abs(radial_dist - config.FEMALE_RADIAL_OFFSET) < TOL_POSITION

    def test_fk_male_origin_on_correct_side(self):
        """Male OCF origin should be offset from bar axis by MALE_RADIAL_OFFSET."""
        Ln_start = np.array([0., -100, 20])
        Ln_end = np.array([0., 100, 20])
        mjp, mjr = 100.0, 0.0  # 100mm from start = midpoint
        frame = fk_male(Ln_start, Ln_end, mjp, mjr, config)
        bar_axis_pt = Ln_start + (mjp / np.linalg.norm(Ln_end - Ln_start)) * (Ln_end - Ln_start)
        radial_dist = np.linalg.norm(frame[:3, 3] - bar_axis_pt)
        assert abs(radial_dist - config.MALE_RADIAL_OFFSET) < TOL_POSITION

    def test_fk_female_x_axis_along_bar(self):
        """The FK female frame's X axis should be along the bar axis (OCF convention)."""
        Le_start = np.array([0., 0, 0])
        Le_end = np.array([200., 0, 0])
        frame = fk_female(Le_start, Le_end, 100.0, 0.0, 0.0, config)
        bar_unit = np.array([1, 0, 0.])
        # X axis of frame should be parallel to bar direction
        assert abs(abs(np.dot(frame[:3, 0], bar_unit)) - 1.0) < TOL_ORIENTATION

    def test_fjr_rotates_radial_direction(self):
        """Rotating FJR by pi should flip the radial offset direction."""
        Le_start = np.array([0., 0, 0])
        Le_end = np.array([200., 0, 0])
        f0 = fk_female(Le_start, Le_end, 100.0, 0.0, 0.0, config)
        f_pi = fk_female(Le_start, Le_end, 100.0, np.pi, 0.0, config)
        # Origins should be on opposite sides of the bar axis
        bar_pt = np.array([100., 0, 0])
        offset0 = f0[:3, 3] - bar_pt
        offset_pi = f_pi[:3, 3] - bar_pt
        # They should be anti-parallel (dot product ≈ -||offset||^2)
        if config.FEMALE_RADIAL_OFFSET > 0:
            cos_angle = np.dot(offset0, offset_pi) / (np.linalg.norm(offset0) * np.linalg.norm(offset_pi))
            assert cos_angle < -0.99  # approximately anti-parallel


# ============================================================
# optimize_joint_placement
# ============================================================

class TestOptimizeJointPlacement:
    def test_perpendicular_bars_at_contact_distance(self):
        """Most basic test: two perpendicular bars exactly at distance D.
        Le along X at z=0, Ln along Y at z=D.
        The optimizer should find DOF values with near-zero residual,
        meaning the male and female frames meet.

        This test verifies the optimizer converges, not specific DOF values
        (which depend on connector geometry).
        """
        D = config.BAR_CONTACT_DISTANCE
        Le_start = np.array([-200., 0, 0])
        Le_end = np.array([200., 0, 0])
        Ln_start = np.array([0., -200, D])
        Ln_end = np.array([0., 200, D])

        result = optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)
        assert result['residual'] < TOL_RESIDUAL, \
            f"Optimizer did not converge: residual={result['residual']}"
        # Female and male frames should nearly coincide
        np.testing.assert_allclose(
            result['female_frame'][:3, 3],
            result['male_frame'][:3, 3],
            atol=TOL_POSITION)

    def test_angled_bars(self):
        """Two bars at 60 degrees, distance D apart.
        Le along X at z=0. Ln at 60 deg in XZ plane at appropriate offset.
        """
        D = config.BAR_CONTACT_DISTANCE
        Le_start = np.array([-200., 0, 0])
        Le_end = np.array([200., 0, 0])
        # Ln direction: 60 deg from X in XZ plane = (cos60, 0, sin60) = (0.5, 0, 0.866)
        Ln_dir = np.array([0.5, 0, np.sqrt(3)/2])
        Ln_mid = np.array([0., 0, D])  # offset by D along Z
        Ln_start = Ln_mid - 200 * Ln_dir
        Ln_end = Ln_mid + 200 * Ln_dir

        result = optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)
        assert result['residual'] < TOL_RESIDUAL, \
            f"Optimizer did not converge for angled bars: residual={result['residual']}"

    def test_skew_bars_3d(self):
        """Fully 3D skew bars. Le along X, Ln along (0,1,1) offset.
        Verifies optimizer handles general 3D geometry.
        """
        D = config.BAR_CONTACT_DISTANCE
        Le_start = np.array([-200., 0, 0])
        Le_end = np.array([200., 0, 0])
        Ln_dir = np.array([0., 1, 1]) / np.sqrt(2)
        # Place Ln so that infinite-line distance ≈ D
        Ln_mid = np.array([30., 0, D])
        Ln_start = Ln_mid - 200 * Ln_dir
        Ln_end = Ln_mid + 200 * Ln_dir

        result = optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)
        assert result['residual'] < TOL_RESIDUAL

    def test_dof_values_within_bounds(self):
        """Verify that returned DOF values are within configured bounds."""
        D = config.BAR_CONTACT_DISTANCE
        Le_start = np.array([-200., 0, 0])
        Le_end = np.array([200., 0, 0])
        Ln_start = np.array([0., -200, D])
        Ln_end = np.array([0., 200, D])

        result = optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)
        assert config.FJP_RANGE[0] <= result['fjp'] <= config.FJP_RANGE[1]
        assert config.FJR_RANGE[0] <= result['fjr'] <= config.FJR_RANGE[1]
        assert config.MJP_RANGE[0] <= result['mjp'] <= config.MJP_RANGE[1]
        assert config.MJR_RANGE[0] <= result['mjr'] <= config.MJR_RANGE[1]
        assert config.JJR_RANGE[0] <= result['jjr'] <= config.JJR_RANGE[1]

    def test_fjp_mjp_near_contact_point(self):
        """For perpendicular bars crossing at the origin,
        FJP and MJP should place joints near the closest-approach point.

        Le: (-200,0,0)-(200,0,0), midpoint at origin.
        Contact is at x=0 on Le, y=0 on Ln.
        fjp = distance from Le_start to contact ≈ 200.
        mjp = distance from Ln_start to contact ≈ 200.
        """
        D = config.BAR_CONTACT_DISTANCE
        Le_start = np.array([-200., 0, 0])
        Le_end = np.array([200., 0, 0])
        Ln_start = np.array([0., -200, D])
        Ln_end = np.array([0., 200, D])

        result = optimize_joint_placement(Le_start, Le_end, Ln_start, Ln_end, config)
        assert abs(result['fjp'] - 200.0) < 5.0   # within 5mm of expected
        assert abs(result['mjp'] - 200.0) < 5.0
```

---

### `tests/test_s2_t1.py` - S2-T1 bar axis generation (two-bar join)

These tests validate the S2-T1 optimization that finds a new bar direction passing through two given contact points. The tests call the core math function (extracted from the Rhino script) and verify geometric constraints.

**NOTE for Codex**: Extract the S2-T1 optimization logic into a function in `core/geometry.py`:
```python
def solve_s2_t1(n1, x_Ce1, n2, x_Ce2, D, nn_init_hint=None):
    """Solve for the direction nn of a new bar Ln such that:
    - Shortest segment from Ln to Le1 passes through Ce1, length = D
    - Shortest segment from Ln to Le2 passes through Ce2, length = D
    Returns: (nn, P1, P2) where nn is unit direction, P1/P2 are points on Ln.
    """
```
This makes it testable without Rhino.

```python
import numpy as np
import pytest
from core.geometry import (
    solve_s2_t1,
    closest_params_infinite_lines,
    distance_infinite_lines,
)
from core.config import BAR_CONTACT_DISTANCE

TOL = 1e-4

class TestS2T1:
    def _verify_s2_result(self, nn, P1, P2, n1, Ce1, n2, Ce2, D):
        """Shared verification: check all geometric constraints of the S2-T1 result."""
        # 1. P1 and P2 should be collinear with direction nn
        diff = P2 - P1
        if np.linalg.norm(diff) > 1e-10:
            diff_unit = diff / np.linalg.norm(diff)
            assert abs(abs(np.dot(diff_unit, nn)) - 1.0) < TOL, \
                f"P1-P2 not parallel to nn: dot={np.dot(diff_unit, nn)}"

        # 2. Distance from Ln (through P1 with direction nn) to Le1 should be D
        d1 = abs(distance_infinite_lines(P1, nn, Ce1, n1))
        assert abs(d1 - D) < TOL, f"Distance to Le1: {d1}, expected {D}"

        # 3. Distance from Ln to Le2 should be D
        d2 = abs(distance_infinite_lines(P1, nn, Ce2, n2))
        assert abs(d2 - D) < TOL, f"Distance to Le2: {d2}, expected {D}"

        # 4. Closest point on Le1 to Ln should be at Ce1
        tA, tB = closest_params_infinite_lines(Ce1, n1, P1, nn)
        closest_on_Le1 = Ce1 + tA * n1
        assert np.linalg.norm(closest_on_Le1 - Ce1) < TOL, \
            f"Closest point on Le1 is not Ce1: off by {np.linalg.norm(closest_on_Le1 - Ce1)}"

        # 5. Closest point on Le2 to Ln should be at Ce2
        tA, tB = closest_params_infinite_lines(Ce2, n2, P1, nn)
        closest_on_Le2 = Ce2 + tA * n2
        assert np.linalg.norm(closest_on_Le2 - Ce2) < TOL, \
            f"Closest point on Le2 is not Ce2: off by {np.linalg.norm(closest_on_Le2 - Ce2)}"

    def test_perpendicular_existing_bars(self):
        """Le1 along X, Le2 along Y, both at z=0.
        Contact points at Ce1=(5,0,0), Ce2=(0,8,0).
        The resulting Ln should be above (z>0) connecting these two contact regions.
        """
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])

        nn, P1, P2 = solve_s2_t1(n1, Ce1, n2, Ce2, D)
        self._verify_s2_result(nn, P1, P2, n1, Ce1, n2, Ce2, D)

    def test_parallel_existing_bars(self):
        """Le1 and Le2 both along X but at different Y positions.
        Ce1=(10, 0, 0), Ce2=(20, 5, 0). Ln should bridge across them.
        """
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([10., 0, 0])
        n2 = np.array([1., 0, 0])
        Ce2 = np.array([20., 5, 0])

        nn, P1, P2 = solve_s2_t1(n1, Ce1, n2, Ce2, D)
        self._verify_s2_result(nn, P1, P2, n1, Ce1, n2, Ce2, D)
        # nn should NOT be parallel to n1 (Ln bridges across, not along)
        assert abs(np.dot(nn, n1/np.linalg.norm(n1))) < 0.99

    def test_3d_skew_bars(self):
        """Fully 3D case: Le1 along (1,0,0), Le2 along (0,1,1).
        Contact points at arbitrary positions on each.
        """
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([15., 0, 0])
        n2 = np.array([0., 1, 1]) / np.sqrt(2)
        Ce2 = np.array([0., 10, 10])

        nn, P1, P2 = solve_s2_t1(n1, Ce1, n2, Ce2, D)
        self._verify_s2_result(nn, P1, P2, n1, Ce1, n2, Ce2, D)

    def test_output_direction_is_unit(self):
        """nn should always be a unit vector."""
        D = BAR_CONTACT_DISTANCE
        n1 = np.array([1., 0, 0])
        Ce1 = np.array([5., 0, 0])
        n2 = np.array([0., 1, 0])
        Ce2 = np.array([0., 8, 0])

        nn, P1, P2 = solve_s2_t1(n1, Ce1, n2, Ce2, D)
        assert abs(np.linalg.norm(nn) - 1.0) < 1e-10
```

---

### Test design rationale

| Test file | What it validates | Why it matters |
|-----------|-------------------|----------------|
| `test_geometry.py` | Line-line distance formulas (Eq 3, 17 from CADJ paper), finite segment clipping | These are the foundation. If the math is wrong, everything downstream fails. All expected values are hand-derived. |
| `test_kinematics.py` | FK chain structure (radial offset, axis alignment, rotation behavior), optimizer convergence | The 5-DOF kinematic chain is unique to our T20-5 connector (not in FrameX). Tests verify structural FK properties + that the optimizer finds near-zero residual for various bar geometries. |
| `test_s2_t1.py` | S2-T1 direction-finding optimization | Uses constraint verification (distance=D, closest point=Ce) rather than expected values, since multiple valid solutions may exist. |

---

## Rhino-Level Verification (manual, after implementation)

After the pytest suite passes, test in Rhino 8:

1. **T1 S1**: Create two skew lines. Run T1 S1. Verify visually that the new line is at the correct distance. Use `Distance` command between the two lines to check.

2. **T1 S2**: Create two existing lines, pick contact points. Run T1 S2. Use `Distance` command to verify D from the new line to each existing line.

3. **T2 S1**: Run T2 on two bars. Verify block instances appear at the contact region. Check `Properties > Attribute User Text` for valid JSON.

4. **T2 S2**: Run on 3 bars. Verify two joint pairs placed.

5. **Rerun**: Right-click button should reproduce the last result without re-prompting for inputs.

---

## Implementation Order for Codex

1. **`core/config.py`** - Define all parameters with TODO placeholders
2. **`core/geometry.py`** - Port the 4 geometry functions from FrameX + add `solve_s2_t1()` (pure numpy + scipy, no Rhino dependency). The `solve_s2_t1` function must be extracted here (not inlined in the Rhino script) so tests can call it.
3. **`core/kinematics.py`** - Implement FK functions and optimizer (numpy + scipy, no Rhino dependency)
4. **`tests/conftest.py`** - Path setup
5. **`tests/test_geometry.py`** - Run and verify all pass
6. **`tests/test_kinematics.py`** - Run and verify all pass
7. **`tests/test_s2_t1.py`** - Run and verify all pass
8. **`t1_bar_axis.py`** - Main T1 script with Rhino UI integration (calls `core/geometry.py`)
9. **`t1_bar_axis_rerun.py`** - Thin rerun wrapper
10. **`t2_joint_placement.py`** - Main T2 script with block placement (calls `core/kinematics.py`)
11. **`t2_joint_placement_rerun.py`** - Thin rerun wrapper

Steps 1-7 can be developed and tested outside Rhino with just numpy+scipy+pytest. Steps 8-11 require Rhino 8.

**Run tests with**: `cd <project_root> && python -m pytest tests/ -v`
