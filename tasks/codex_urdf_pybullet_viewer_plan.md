# Plan: URDF + PyBullet FK Debugging Viewer

## Context

The current `scripts/fk_joint_viewer.py` (matplotlib-based) has incorrect kinematic chain logic. We need a ground-truth visualization to debug and validate the FK. The approach:
1. A script that generates a URDF from `config.py` parameters
2. A PyBullet interactive viewer with sliders for the 5 DOFs

## Kinematic Chain (single serial chain, Le → connector → Ln)

```
le_bar (BASE, fixed grey cylinder)
 └─ fjp_joint (PRISMATIC along Le axis)
     └─ fjp_link (invisible)
         └─ fjr_joint (REVOLUTE around Le axis)
             └─ fjr_link (invisible)
                 └─ female_offset_joint (FIXED: radial offset + frame rotation)
                     └─ female_link (red box visual)
                         └─ jjr_joint (REVOLUTE around screw/contact-normal axis)
                             └─ male_link (green box visual)
                                 └─ male_offset_joint (FIXED: radial offset + frame rotation)
                                     └─ mjr_link (invisible)
                                         └─ mjr_joint (REVOLUTE around Ln axis)
                                             └─ mjr_out_link (invisible)
                                                 └─ mjp_joint (PRISMATIC along Ln axis)
                                                     └─ ln_bar (orange cylinder)
```

**Total**: 5 actuated joints (fjp, fjr, jjr, mjr, mjp) + 2 fixed joints (geometric offsets)

---

## Frame Conventions at Each Link

All links use right-handed frames. The "primary axis" convention:
- **Bar links** (le_bar, ln_bar): **Z = bar longitudinal axis**
- **Connector links** (female_link, male_link): **Z = screw/contact-normal direction** (radial, from Le towards Ln), **X = bar axis of the "parent" bar side**

### Detailed link frames

| Link | Z axis | X axis | Origin |
|------|--------|--------|--------|
| le_bar | Le bar axis direction | perpendicular to Le (arbitrary but fixed) | Le midpoint |
| fjp_link | same as le_bar | same as le_bar | translated along Le by fjp |
| fjr_link | same as le_bar | rotated around Le axis by fjr | same position as fjp_link |
| female_link | radial direction (Le→Ln) | Le bar axis direction | offset from bar axis by FEMALE_RADIAL_OFFSET along radial |
| male_link | same Z as female (radial) | rotated X by jjr | same position as female_link |
| mjr_link | Ln bar axis direction | perpendicular to Ln | offset from female/male by MALE_RADIAL_OFFSET along radial |
| mjr_out_link | same as mjr_link | rotated around Ln axis by mjr | same position as mjr_link |
| ln_bar | Ln bar axis direction | same as mjr_out_link | translated along Ln by mjp |

---

## Joint Definitions

| Joint name | Type | Axis (in parent frame) | Limits (from config.py) |
|------------|------|------------------------|------------------------|
| fjp_joint | prismatic | (0, 0, 1) | FJP_RANGE |
| fjr_joint | revolute | (0, 0, 1) | FJR_RANGE |
| female_offset_joint | **fixed** | N/A | N/A |
| jjr_joint | revolute | (0, 0, 1) | JJR_RANGE |
| male_offset_joint | **fixed** | N/A | N/A |
| mjr_joint | revolute | (0, 0, 1) | MJR_RANGE |
| mjp_joint | prismatic | (0, 0, 1) | MJP_RANGE |

Note: every actuated joint has axis `(0, 0, 1)` because each link's Z axis is aligned with the joint's natural axis. The frame rotations in the fixed joints handle the axis transitions.

---

## Fixed Joint Transforms (the critical part)

### `female_offset_joint` (fjr_link → female_link)

This transforms from the "bar-axis" convention (Z=Le axis, X=radial after FJR) to the "connector" convention (Z=radial, X=Le axis).

**Translation** (in fjr_link frame): `(FEMALE_RADIAL_OFFSET, 0, FEMALE_AXIAL_OFFSET)`
- X of fjr_link = radial direction → offset along it

**Rotation**: swap X↔Z, negate Y to stay right-handed.

The rotation matrix R that maps fjr_link axes to female_link axes:
```
female_X = fjr_Z  → R column 0 = (0, 0, 1)
female_Y = -fjr_Y → R column 1 = (0, -1, 0)  
female_Z = fjr_X  → R column 2 = (1, 0, 0)

R = [[0,  0, 1],
     [0, -1, 0],
     [1,  0, 0]]
```

**URDF rpy**: `roll=π, pitch=-π/2, yaw=0` → `rpy="3.14159265 -1.57079633 0"`

Verification: Rz(0) @ Ry(-π/2) @ Rx(π)
```
Rx(π) = [[1,0,0],[0,-1,0],[0,0,-1]]
Ry(-π/2) = [[0,0,-1],[0,1,0],[1,0,0]]
Ry(-π/2) @ Rx(π) = [[0,0,1],[0,-1,0],[1,0,0]]  ✓
```

### `male_offset_joint` (male_link → mjr_link)

This transforms from the "connector" convention (Z=radial, X=Ln axis) back to the "bar-axis" convention (Z=Ln axis, X=radial).

**Translation** (in male_link frame): `(0, 0, MALE_RADIAL_OFFSET)`
- Z of male_link = radial direction → offset along it to reach Ln bar axis

**Rotation**: same matrix R (it is its own inverse: R@R = I).

**URDF rpy**: `rpy="3.14159265 -1.57079633 0"` (same as female_offset)

---

## File Structure

```
scripts/
├── generate_urdf.py        # Reads config.py, writes T20_5_chain.urdf
├── pybullet_viewer.py      # Loads URDF, interactive sliders
└── core/
    └── config.py           # (existing, unchanged)
```

---

## Script 1: `scripts/generate_urdf.py`

Reads `core/config.py` and generates a URDF XML file.

### Parameters read from config.py:
- `BAR_RADIUS` → cylinder radius for bar visuals
- `BAR_CONTACT_DISTANCE` → used for documentation only
- `FEMALE_RADIAL_OFFSET`, `FEMALE_AXIAL_OFFSET` → female_offset_joint translation
- `MALE_RADIAL_OFFSET`, `MALE_AXIAL_OFFSET` → male_offset_joint translation
- `FJP_RANGE`, `FJR_RANGE`, `MJR_RANGE`, `MJP_RANGE`, `JJR_RANGE` → joint limits
- `DEFAULT_NEW_BAR_LENGTH` → Ln cylinder length

### Additional constants (hardcoded in generate_urdf.py):
```python
LE_BAR_LENGTH = 440.0           # mm, length of Le bar for visualization
LN_BAR_LENGTH = 420.0           # mm, length of Ln bar  
FEMALE_BOX_SIZE = (46, 28, 18)  # mm, (along-bar, tangential, radial) in connector frame
MALE_BOX_SIZE = (40, 24, 16)    # mm
SCALE = 0.001                   # mm to meters (URDF uses meters)
```

### URDF generation logic

Use Python string formatting or `xml.etree.ElementTree` to build the URDF. Each link gets:
- An `<inertial>` block (small mass, identity inertia — needed for PyBullet but values don't matter for pure kinematics)
- A `<visual>` block with `<geometry>` (cylinder for bars, box for joints) and `<material>` (color)
- No `<collision>` needed (pure visualization)

Template structure (pseudocode):
```python
def generate_urdf():
    # Scale all mm dimensions to meters
    s = SCALE  

    urdf = f'''<?xml version="1.0"?>
<robot name="t20_5_connector_chain">

  <!-- ==================== LINKS ==================== -->

  <link name="le_bar">
    <visual>
      <geometry><cylinder length="{LE_BAR_LENGTH*s}" radius="{BAR_RADIUS*s}"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="fjp_link">
    <!-- invisible carrier -->
    <inertial><mass value="0.01"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="fjr_link">
    <!-- invisible carrier -->
    <inertial><mass value="0.01"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="female_link">
    <visual>
      <!-- Box axes in female_link frame: X=bar-axis, Y=tangential, Z=radial -->
      <geometry><box size="{FEMALE_BOX_SIZE[0]*s} {FEMALE_BOX_SIZE[1]*s} {FEMALE_BOX_SIZE[2]*s}"/></geometry>
      <material name="red"><color rgba="0.9 0.5 0.55 1"/></material>
    </visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="male_link">
    <visual>
      <geometry><box size="{MALE_BOX_SIZE[0]*s} {MALE_BOX_SIZE[1]*s} {MALE_BOX_SIZE[2]*s}"/></geometry>
      <material name="green"><color rgba="0.44 0.85 0.55 1"/></material>
    </visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="mjr_link">
    <inertial><mass value="0.01"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="mjr_out_link">
    <inertial><mass value="0.01"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="ln_bar">
    <visual>
      <geometry><cylinder length="{LN_BAR_LENGTH*s}" radius="{BAR_RADIUS*s}"/></geometry>
      <material name="orange"><color rgba="0.85 0.63 0.4 1"/></material>
    </visual>
    <inertial><mass value="0.1"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- ==================== JOINTS ==================== -->

  <!-- FJP: prismatic along Le bar axis (Z of le_bar) -->
  <joint name="fjp_joint" type="prismatic">
    <parent link="le_bar"/>
    <child link="fjp_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{FJP_RANGE[0]*s}" upper="{FJP_RANGE[1]*s}" effort="100" velocity="1"/>
  </joint>

  <!-- FJR: revolute around Le bar axis (Z) -->
  <joint name="fjr_joint" type="revolute">
    <parent link="fjp_link"/>
    <child link="fjr_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{FJR_RANGE[0]}" upper="{FJR_RANGE[1]}" effort="100" velocity="1"/>
  </joint>

  <!-- FIXED: offset from Le bar axis to female connector OCF -->
  <joint name="female_offset_joint" type="fixed">
    <parent link="fjr_link"/>
    <child link="female_link"/>
    <origin xyz="{FEMALE_RADIAL_OFFSET*s} 0 {FEMALE_AXIAL_OFFSET*s}"
            rpy="3.14159265 -1.57079633 0"/>
  </joint>

  <!-- JJR: revolute around screw/contact-normal axis (Z of female_link) -->
  <joint name="jjr_joint" type="revolute">
    <parent link="female_link"/>
    <child link="male_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{JJR_RANGE[0]}" upper="{JJR_RANGE[1]}" effort="100" velocity="1"/>
  </joint>

  <!-- FIXED: offset from male connector OCF back to Ln bar axis -->
  <joint name="male_offset_joint" type="fixed">
    <parent link="male_link"/>
    <child link="mjr_link"/>
    <origin xyz="0 0 {MALE_RADIAL_OFFSET*s}"
            rpy="3.14159265 -1.57079633 0"/>
  </joint>

  <!-- MJR: revolute around Ln bar axis (Z of mjr_link) -->
  <joint name="mjr_joint" type="revolute">
    <parent link="mjr_link"/>
    <child link="mjr_out_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{MJR_RANGE[0]}" upper="{MJR_RANGE[1]}" effort="100" velocity="1"/>
  </joint>

  <!-- MJP: prismatic along Ln bar axis (Z of mjr_out_link) -->
  <joint name="mjp_joint" type="prismatic">
    <parent link="mjr_out_link"/>
    <child link="ln_bar"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{MJP_RANGE[0]*s}" upper="{MJP_RANGE[1]*s}" effort="100" velocity="1"/>
  </joint>

</robot>'''
    
    output_path = os.path.join(os.path.dirname(__file__), "T20_5_chain.urdf")
    with open(output_path, "w") as f:
        f.write(urdf)
    print(f"URDF written to {output_path}")
```

**Important notes for Codex:**
- ALL dimensions in the URDF must be in **meters** (multiply mm values from config.py by 0.001)
- Joint limits for prismatic joints are also in meters
- Joint limits for revolute joints are in radians (no conversion needed)
- The `rpy` in URDF `<origin>` is applied as Rz(yaw) @ Ry(pitch) @ Rx(roll)
- Cylinder `length` is along Z of the link frame; cylinder origin is at its center
- Box `size` is full extents (not half-extents)

---

## Script 2: `scripts/pybullet_viewer.py`

Interactive viewer using PyBullet's GUI with debug sliders.

```python
import pybullet as p
import pybullet_data
import time
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from core import config

SCALE = 0.001  # mm to meters (joint values from sliders in mm, URDF in meters)

def main():
    # Connect to PyBullet GUI
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Optional: load ground plane for reference
    p.loadURDF("plane.urdf")

    # Load our URDF
    urdf_path = os.path.join(SCRIPT_DIR, "T20_5_chain.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.3],
                           useFixedBase=True)

    # Discover joint indices by name
    joint_map = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]  # 0=revolute, 1=prismatic, 4=fixed
        if joint_type in (0, 1):  # only movable joints
            joint_map[joint_name] = i

    # Create sliders for each DOF
    # Slider values are in human-readable units (mm for prismatic, degrees for revolute)
    # Convert to URDF units (meters, radians) when setting joint positions
    slider_specs = [
        ("fjp_joint",  "FJP (mm)",  config.FJP_RANGE[0],  config.FJP_RANGE[1],  0.0,   "prismatic"),
        ("fjr_joint",  "FJR (deg)", -180, 180, 0.0, "revolute"),
        ("jjr_joint",  "JJR (deg)", -180, 180, 0.0, "revolute"),
        ("mjr_joint",  "MJR (deg)", -180, 180, 0.0, "revolute"),
        ("mjp_joint",  "MJP (mm)",  config.MJP_RANGE[0],  config.MJP_RANGE[1],  0.0,   "prismatic"),
    ]

    sliders = {}
    for joint_name, label, lo, hi, default, jtype in slider_specs:
        slider_id = p.addUserDebugParameter(label, lo, hi, default)
        sliders[joint_name] = (slider_id, jtype)

    print("Joints found:", list(joint_map.keys()))
    print("Sliders created. Move them to see the chain update.")

    # Set camera
    p.resetDebugVisualizerCamera(
        cameraDistance=0.8,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.3]
    )

    # Main loop
    while True:
        for joint_name, (slider_id, jtype) in sliders.items():
            raw_val = p.readUserDebugParameter(slider_id)
            if jtype == "prismatic":
                # Slider in mm → URDF in meters
                joint_val = raw_val * SCALE
            else:
                # Slider in degrees → URDF in radians
                joint_val = raw_val * 3.14159265 / 180.0

            if joint_name in joint_map:
                p.resetJointState(robot_id, joint_map[joint_name], joint_val)

        time.sleep(0.02)  # ~50 fps


if __name__ == "__main__":
    main()
```

**Key implementation notes for Codex:**
- Use `p.resetJointState()` (not `p.setJointMotorControl2()`) for pure kinematic positioning
- Prismatic slider values in **mm** (user-friendly), converted to **meters** for PyBullet
- Revolute slider values in **degrees** (user-friendly), converted to **radians** for PyBullet
- `useFixedBase=True` keeps Le bar fixed in space
- The `basePosition=[0, 0, 0.3]` lifts the robot so Le bar doesn't clip the ground plane
- Use `p.addUserDebugParameter` for sliders — these are built into PyBullet's GUI, no external UI library needed

---

## Dependencies

```
pip install pybullet numpy
```
No other dependencies needed. The `pybullet_data` package is included with pybullet.

---

## Verification

1. **All joints at zero**: Le (grey) and Ln (orange) should both be along the same axis (Z) with the connector in between, separated by `FEMALE_RADIAL_OFFSET + MALE_RADIAL_OFFSET` radially. The female (red) and male (green) boxes should be at the midpoint between the bars.

2. **FJP only**: sliding FJP should translate the entire connector+Ln assembly along Le's axis. Le bar stays fixed.

3. **FJR only**: rotating FJR should spin the connector+Ln assembly around Le's axis. At FJR=0° the connector points along X; at FJR=90° it points along Y.

4. **JJR only**: rotating JJR should spin the male+Ln around the contact-normal axis (the line connecting Le and Ln axes). Only the male box and Ln bar should rotate; the female box stays fixed.

5. **MJR only**: rotating MJR should spin Ln around its own axis. Only the Ln bar visual rotates; the connector stays fixed.

6. **MJP only**: sliding MJP should translate Ln along its own axis. The connector stays at the same position.

7. **Combined**: setting FJR=90° and JJR=90° should produce a visually plausible non-planar configuration where Ln is skew to Le.

---

## Self-Contained Tests (`tests/test_urdf_chain.py`)

These tests load the generated URDF into PyBullet (headless, no GUI) and verify the FK chain using `p.getLinkState()`. Run with `pytest tests/test_urdf_chain.py -v`.

```python
"""Tests for the URDF kinematic chain.

Loads the generated URDF into PyBullet in DIRECT mode (no GUI).
Sets joint values and verifies link positions/orientations against
analytically expected values.

Prerequisites:
  - Run `python scripts/generate_urdf.py` first to produce the URDF.
  - pip install pybullet numpy pytest
"""

import os
import sys
import math
import numpy as np
import pybullet as p
import pybullet_data
import pytest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, SCRIPT_DIR)
from core import config

URDF_PATH = os.path.join(SCRIPT_DIR, "T20_5_chain.urdf")
SCALE = 0.001  # mm → meters
TOL = 1e-4     # tolerance in meters (0.1mm)
ANGLE_TOL = 1e-3  # tolerance in radians


@pytest.fixture(scope="module")
def robot():
    """Load the URDF once for all tests in this module."""
    assert os.path.exists(URDF_PATH), (
        f"URDF not found at {URDF_PATH}. Run `python scripts/generate_urdf.py` first.")
    cid = p.connect(p.DIRECT)  # headless
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0], useFixedBase=True)

    # Build joint name → index map
    joint_map = {}
    link_map = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_map[joint_name] = i
        link_map[link_name] = i

    yield {"id": robot_id, "joints": joint_map, "links": link_map, "client": cid}
    p.disconnect(cid)


def set_joints(robot, values_mm_deg):
    """Set joints from a dict of {joint_name: value_in_mm_or_deg}.
    Prismatic values in mm, revolute in degrees. Converts to URDF units.
    """
    for jname, val in values_mm_deg.items():
        idx = robot["joints"][jname]
        info = p.getJointInfo(robot["id"], idx)
        jtype = info[2]
        if jtype == 1:  # prismatic
            p.resetJointState(robot["id"], idx, val * SCALE)
        else:  # revolute
            p.resetJointState(robot["id"], idx, math.radians(val))


def get_link_pos(robot, link_name):
    """Get world position of a link (meters)."""
    idx = robot["links"][link_name]
    state = p.getLinkState(robot["id"], idx)
    return np.array(state[0])  # worldLinkFramePosition


def get_link_orn_matrix(robot, link_name):
    """Get world orientation of a link as a 3x3 rotation matrix."""
    idx = robot["links"][link_name]
    state = p.getLinkState(robot["id"], idx)
    quat = state[1]  # worldLinkFrameOrientation (quaternion)
    return np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)


# ================================================================
# Test: URDF loads correctly with expected joints
# ================================================================

class TestURDFStructure:
    def test_has_5_movable_joints(self, robot):
        movable = []
        for i in range(p.getNumJoints(robot["id"])):
            info = p.getJointInfo(robot["id"], i)
            if info[2] in (0, 1):  # revolute or prismatic
                movable.append(info[1].decode("utf-8"))
        assert set(movable) == {"fjp_joint", "fjr_joint", "jjr_joint", "mjr_joint", "mjp_joint"}

    def test_joint_types(self, robot):
        expected_types = {
            "fjp_joint": 1,    # prismatic
            "fjr_joint": 0,    # revolute
            "jjr_joint": 0,    # revolute
            "mjr_joint": 0,    # revolute
            "mjp_joint": 1,    # prismatic
        }
        for jname, expected_type in expected_types.items():
            idx = robot["joints"][jname]
            info = p.getJointInfo(robot["id"], idx)
            assert info[2] == expected_type, f"{jname}: expected type {expected_type}, got {info[2]}"

    def test_has_expected_links(self, robot):
        expected = {"fjp_link", "fjr_link", "female_link", "male_link",
                    "mjr_link", "mjr_out_link", "ln_bar"}
        assert expected.issubset(set(robot["links"].keys()))


# ================================================================
# Test: FJP translates everything along Le axis
# ================================================================

class TestFJP:
    def test_fjp_moves_female_along_le(self, robot):
        """Setting FJP=100mm should move the female link 0.1m along Le axis (Z of base)."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 100, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos1 = get_link_pos(robot, "female_link")

        delta = pos1 - pos0
        # Should move 0.1m along Z (Le axis)
        assert abs(delta[2] - 0.1) < TOL
        # No movement in X or Y
        assert abs(delta[0]) < TOL
        assert abs(delta[1]) < TOL

    def test_fjp_moves_ln_bar_equally(self, robot):
        """Ln bar should translate by the same amount as female link when FJP changes."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        ln_pos0 = get_link_pos(robot, "ln_bar").copy()

        set_joints(robot, {"fjp_joint": 50, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        ln_pos1 = get_link_pos(robot, "ln_bar")

        delta = ln_pos1 - ln_pos0
        assert abs(delta[2] - 0.05) < TOL


# ================================================================
# Test: FJR rotates around Le axis
# ================================================================

class TestFJR:
    def test_fjr_rotates_female_around_le(self, robot):
        """At FJR=0, female is offset along X. At FJR=90, offset along Y."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 90, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos90 = get_link_pos(robot, "female_link")

        # The radial offset should rotate from X to Y direction
        r = config.FEMALE_RADIAL_OFFSET * SCALE
        # At FJR=0: female is at (r, 0, 0) relative to bar axis
        assert abs(pos0[0] - r) < TOL, f"At FJR=0, X offset should be ~{r}, got {pos0[0]}"
        assert abs(pos0[1]) < TOL

        # At FJR=90: female is at (0, r, 0)
        assert abs(pos90[0]) < TOL
        assert abs(pos90[1] - r) < TOL, f"At FJR=90, Y offset should be ~{r}, got {pos90[1]}"

    def test_fjr_does_not_change_z(self, robot):
        """FJR rotation should not affect the Z position (along Le axis)."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        z0 = get_link_pos(robot, "female_link")[2]

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 45, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        z45 = get_link_pos(robot, "female_link")[2]
        assert abs(z45 - z0) < TOL


# ================================================================
# Test: JJR rotates male around screw axis
# ================================================================

class TestJJR:
    def test_jjr_does_not_move_female(self, robot):
        """JJR should NOT affect the female link position."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        f0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 90,
                           "mjr_joint": 0, "mjp_joint": 0})
        f90 = get_link_pos(robot, "female_link")
        np.testing.assert_allclose(f0, f90, atol=TOL)

    def test_jjr_changes_ln_direction(self, robot):
        """JJR=90 should rotate the Ln bar direction by 90 degrees around the contact normal."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        R0 = get_link_orn_matrix(robot, "ln_bar")
        ln_dir_0 = R0[:, 2]  # Z axis of ln_bar = bar direction

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 90,
                           "mjr_joint": 0, "mjp_joint": 0})
        R90 = get_link_orn_matrix(robot, "ln_bar")
        ln_dir_90 = R90[:, 2]

        # The angle between the two Ln directions should be ~90 degrees
        cos_angle = np.clip(np.dot(ln_dir_0, ln_dir_90), -1, 1)
        angle = math.degrees(math.acos(abs(cos_angle)))
        assert abs(angle - 90) < 2.0, f"Expected ~90 deg change, got {angle}"


# ================================================================
# Test: MJP translates Ln along its own axis
# ================================================================

class TestMJP:
    def test_mjp_moves_ln_along_its_axis(self, robot):
        """MJP should move Ln along its own bar direction, not Le's."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos0 = get_link_pos(robot, "ln_bar").copy()
        R0 = get_link_orn_matrix(robot, "ln_bar")
        ln_dir = R0[:, 2]  # Ln's Z axis = bar direction

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 100})
        pos1 = get_link_pos(robot, "ln_bar")

        delta = pos1 - pos0
        # The displacement should be along ln_dir
        displacement_along = np.dot(delta, ln_dir)
        displacement_perp = np.linalg.norm(delta - displacement_along * ln_dir)
        assert abs(displacement_along - 0.1) < TOL  # 100mm = 0.1m
        assert displacement_perp < TOL

    def test_mjp_does_not_move_female(self, robot):
        """MJP should NOT affect the female link."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        f0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 100})
        f1 = get_link_pos(robot, "female_link")
        np.testing.assert_allclose(f0, f1, atol=TOL)


# ================================================================
# Test: MJR rotates Ln around its own axis
# ================================================================

class TestMJR:
    def test_mjr_does_not_change_ln_position(self, robot):
        """MJR rotation should not change Ln bar center position."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        pos0 = get_link_pos(robot, "ln_bar").copy()

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 90, "mjp_joint": 0})
        pos90 = get_link_pos(robot, "ln_bar")
        np.testing.assert_allclose(pos0, pos90, atol=TOL)

    def test_mjr_rotates_ln_axes(self, robot):
        """MJR should rotate the X/Y axes of Ln while keeping Z (bar direction) unchanged."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        R0 = get_link_orn_matrix(robot, "ln_bar")

        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 90, "mjp_joint": 0})
        R90 = get_link_orn_matrix(robot, "ln_bar")

        # Z axis (bar direction) should be unchanged
        np.testing.assert_allclose(R0[:, 2], R90[:, 2], atol=ANGLE_TOL)
        # X axis should have rotated ~90 degrees
        cos_x = np.dot(R0[:, 0], R90[:, 0])
        assert abs(cos_x) < 0.1  # should be ~0 for 90 degree rotation


# ================================================================
# Test: Radial offset distances
# ================================================================

class TestRadialOffset:
    def test_female_offset_from_bar_axis(self, robot):
        """Female link should be FEMALE_RADIAL_OFFSET away from Le bar axis."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        female_pos = get_link_pos(robot, "female_link")
        # Le bar axis is along Z through origin, so radial distance = sqrt(x^2 + y^2)
        radial_dist = math.sqrt(female_pos[0]**2 + female_pos[1]**2)
        expected = config.FEMALE_RADIAL_OFFSET * SCALE
        assert abs(radial_dist - expected) < TOL, \
            f"Female radial offset: {radial_dist}, expected {expected}"

    def test_total_bar_separation(self, robot):
        """At all joints=0, the distance between Le axis and Ln axis should be
        FEMALE_RADIAL_OFFSET + MALE_RADIAL_OFFSET = BAR_CONTACT_DISTANCE."""
        set_joints(robot, {"fjp_joint": 0, "fjr_joint": 0, "jjr_joint": 0,
                           "mjr_joint": 0, "mjp_joint": 0})
        ln_pos = get_link_pos(robot, "ln_bar")
        # Ln bar center is on Ln axis. The perpendicular distance to Le axis (Z through origin)
        radial_dist = math.sqrt(ln_pos[0]**2 + ln_pos[1]**2)
        expected = (config.FEMALE_RADIAL_OFFSET + config.MALE_RADIAL_OFFSET) * SCALE
        assert abs(radial_dist - expected) < TOL, \
            f"Bar separation: {radial_dist*1000:.1f}mm, expected {expected*1000:.1f}mm"
```

---

## Implementation Order

1. `scripts/generate_urdf.py` — generate the URDF file
2. Run it: `python scripts/generate_urdf.py` → produces `scripts/T20_5_chain.urdf`
3. `tests/test_urdf_chain.py` — the test file above
4. Run tests: `python -m pytest tests/test_urdf_chain.py -v` → all should pass
5. `scripts/pybullet_viewer.py` — the interactive viewer
6. Run it: `python scripts/pybullet_viewer.py` → opens PyBullet GUI with sliders
7. Manually verify using the 7 visual checks in the Verification section
