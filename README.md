# Bar Joint Rhino Design Workflow

Interactive Rhino 8 scripts for placing scaffolding bars and T20-5 connectors. Includes standalone PyBullet/matplotlib tools for FK debugging.

## Setup

### Python environment (for standalone scripts and tests)

```bash
pip install -r requirements.txt
```

### Connector parameters

Edit `scripts/core/config.py` to set your connector dimensions (bar radius, joint offsets, DOF ranges). The defaults are provisional symmetric values for 20mm tubes.

---

## Standalone Python Scripts

These run outside Rhino in a regular Python environment.

### Generate URDF

```bash
python scripts/generate_urdf.py
```

Produces `scripts/T20_5_chain.urdf` — a kinematic model of the full connector chain:
`Le bar → FJP → FJR → female joint → JJR → male joint → MJR → MJP → Ln bar`

### PyBullet interactive viewer

```bash
python scripts/pybullet_viewer.py
```

Opens a PyBullet GUI with sliders for all 5 DOFs (FJP, FJR, JJR, MJR, MJP). Useful for verifying the kinematic chain behaves correctly.

Requires the URDF to be generated first.

### Matplotlib FK viewer

```bash
python scripts/fk_joint_viewer.py
```

Matplotlib-based 3D viewer with sliders. Lighter weight than PyBullet, no URDF needed.

---

## Tests

```bash
# Run all tests
python -m pytest tests/ -v

# Run with visual debugging (shows matplotlib/PyBullet plots)
python -m pytest tests/ -v --viz

# Run a specific test file
python -m pytest tests/test_geometry.py -v
python -m pytest tests/test_kinematics.py -v --viz
```

| Test file | What it covers |
|-----------|---------------|
| `test_geometry.py` | Line-line distance, closest points, finite segments |
| `test_kinematics.py` | FK chains, perpendicular_to, frame_distance, joint placement optimization |
| `test_s2_t1.py` | S2-T1 bar axis solving (single + all solutions) |
| `test_urdf_chain.py` | URDF structure, joint behavior, radial offsets (requires `generate_urdf.py` first) |

---

## Rhino 8 Scripts

### Install toolbar

Generate and import a Rhino toolbar with two buttons:

```bash
python scripts/generate_rhino_toolbar.py
```

This creates `BarJointDesign.rui` in the project root. To install:

1. **Drag and drop** the `.rui` file into the Rhino 8 window
2. Or run the Rhino command `_ToolbarOpen` and browse to the file

The toolbar provides two buttons:

| Button | Left-click | Right-click |
|--------|-----------|-------------|
| **T1 Bar** | T1 bar axis generation (prompts for inputs) | Rerun with last inputs |
| **T2 Joint** | T2 joint placement (prompts for inputs) | Rerun with last inputs |

### Running scripts manually (without toolbar)

Local path: `C:\Users\yijiangh\Dropbox\0_Projects\2025_husky_assembly\Code\bar_joint_rhino_design_workflow\scripts\t1_bar_axis.py`

In Rhino 8's command line:

```
_-ScriptEditor _R "<path to script>\t1_bar_axis.py"
_-ScriptEditor _R "<path to script>\t2_joint_placement.py"
```

Do not use `_-RunPythonScript` for these files. That command uses Rhino's legacy Python runner, while these scripts use Python 3 syntax and must be launched through Rhino 8's ScriptEditor run command.

### T1 — Bar Axis Generation

Places a new bar at the correct contact distance from existing bars.

**S1 mode** (one existing bar): Select Le, then Ln. The script translates Ln so the shortest distance between the infinite lines equals D (`BAR_CONTACT_DISTANCE` in config.py).

**S2 mode** (two existing bars): Select Le1, Le2, pick contact points Ce1, Ce2. The script solves for Ln direction such that both contact constraints are satisfied. If multiple solutions exist, all are shown as colored preview lines — click to pick one.

### T2 — Joint Placement

Optimizes 5 DOFs and places connector block instances.

**S1 mode**: Select Le and Ln. The optimizer finds (FJP, FJR, MJP, MJR, JJR) values that align the female and male joint OCFs. Places "FemaleJoint" and "MaleJoint" block instances with JSON metadata in Attribute User Text.

**S2 mode**: Select Ln, Le1, Le2. Runs S1 optimization twice (one per connection).

Block definitions "FemaleJoint" and "MaleJoint" must exist in the .3dm file. If they don't, placeholder colored boxes are created automatically.

---

## Project Structure

```
scripts/
  core/
    config.py             # T20-5 connector parameters (edit this)
    geometry.py           # Line-line distance math
    kinematics.py         # FK chains and joint optimization
  t1_bar_axis.py          # Rhino: bar axis generation
  t1_bar_axis_rerun.py    # Rhino: rerun T1 with cached inputs
  t2_joint_placement.py   # Rhino: joint placement
  t2_joint_placement_rerun.py
  generate_urdf.py        # Standalone: generate URDF
  generate_rhino_toolbar.py # Standalone: generate .rui toolbar file
  pybullet_viewer.py      # Standalone: interactive PyBullet viewer
  fk_joint_viewer.py      # Standalone: matplotlib FK viewer
tests/
  test_geometry.py
  test_kinematics.py
  test_s2_t1.py
  test_urdf_chain.py
  viz_helpers.py           # Shared visualization for --viz mode
support_materials/
  FrameX/                  # Reference codebase (git submodule)
  papers/                  # Research papers
  specs.pdf                # Design specifications
```
