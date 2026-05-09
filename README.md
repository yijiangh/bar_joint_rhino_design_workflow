# Bar Joint Rhino Design Workflow

Interactive Rhino 8 scripts for placing scaffolding bars and pluggable
connector blocks ("joint pairs"). The Rhino-facing workflow uses only the
core math stack (`numpy` + `scipy`) and is split into two stages:

- **T1 – Bar axis**: snap a new bar onto an existing bar, or add a brace
  between two bars with interactive solution selection.
- **T2 – Joint placement**: place connector blocks on a bar pair using a
  4-DOF optimizer that aligns the female and male screw holes.

Each connector family is described by a **joint pair**: a female + male
block definition with the geometry needed to drive the optimizer. Joint
pairs are authored interactively in Rhino with `RSDefineJointHalf` /
`RSDefineJointMate` and
stored in `scripts/core/joint_pairs.json` along with their `.3dm` block
assets in `asset/`.

## Setup

### Rhino 8

Each Rhino entry-point script declares its own virtual environment and
requirements via ScriptEditor directives, so no manual install is
required:

| Toolbar | Button | Script | Purpose |
|---------|--------|--------|---------|
| **RSDesign** | RSBarSnap | `rs_bar_snap.py` | Snap a new bar onto an existing bar at contact distance |
| **RSDesign** | RSBarBrace | `rs_bar_brace.py` | Add a brace bar between two bars (interactive solution picker) |
| **RSDesign** | RSSequenceEdit | `rs_sequence_edit.py` | Interactive assembly sequence viewer and editor |
| **RSDesign** | RSJointPlace | `rs_joint_place.py` | Place connector blocks; auto-assigns female/male by sequence, click to flip orientation |
| **RSDesign** | RSJointEdit | `rs_joint_edit.py` | Re-edit a placed joint pair by clicking female or male block to flip orientation |
| **RSDesign** | RSIKKeyframe | `rs_ik_keyframe.py` | Dual-arm IK keyframe (pick 2 male joints, solve IK, save on shared Ln bar) |
| **RSDesign** | RSShowIK | `rs_show_ik.py` | Replay a saved IK keyframe on a picked bar |
| **RSSetup** | RSBakeFrame | `rs_bake_frame.py` | Bake named CAD reference frames into the document |
| **RSSetup** | RSExportConfig | `rs_export_config.py` | Export CAD-derived connector geometry to `config_generated.py` |
| **RSSetup** | RSMeasureGap | `rs_measure_gap.py` | Measure closest distance between two line segments |
| **RSSetup** | RSExportCase | `rs_export_case.py` | Export a T1-S2 optimization case to JSON for testing |
| **RSSetup** | RSExportGraspTool0TF | `rs_export_grasp_tool0_tf.py` | Export male-joint OCF → tool0 and bar-grasp → tool0 transforms (IK keyframe workflow) |
| **RSSetup** | RSPBStart | `rs_pb_start.py` | Start the shared PyBullet client used by IK workflows |
| **RSSetup** | RSPBStop | `rs_pb_stop.py` | Disconnect the shared PyBullet client |
| **RSSetup** | RSExportPineappleOBJ | `rs_export_pineapple_obj.py` | Export pineapple block defs as OBJ (meters) for IK collision tool models |

These scripts do not require PyBullet. They only need `numpy` and `scipy`.

**No manual pip install is needed.** Each script declares its own virtual environment and requirements using Rhino 8 ScriptEditor directives:

```python
#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
```

The first time you run any script, Rhino's ScriptEditor creates the
shared `scaffolding_env` venv and installs the required packages. If the
install ever fails, reset via **Tools → Advanced → Reset Python 3
(CPython) Runtime** in the ScriptEditor and re-run.

### Optional (tests only)

### Submodule dependencies for the IK workflow

The IK keyframe workflow (RSPBStart, RSIKKeyframe, RSShowIK) depends on a specific development branch of [compas_fab](https://github.com/compas-dev/compas_fab/tree/wip_process). That branch is vendored as a git submodule at `external/compas_fab` rather than pulled from PyPI via `# r:` — this keeps iteration on the branch deterministic and avoids the pip-cache SHA gotcha described in [tasks/yh_lesson.md](tasks/yh_lesson.md).

After cloning this repo (or pulling changes that touch the submodule), run:

```bash
git submodule update --init --recursive
```

`scripts/core/robot_cell.py` prepends `external/compas_fab/src` onto `sys.path` before any `import compas_fab`, so Rhino scripts see the submodule version automatically. To switch to a different upstream commit, `cd external/compas_fab && git fetch && git checkout <sha>` — the next Rhino script run loads that SHA with no venv state to reset.

The IK scripts still declare the remaining transitive dependencies (`compas`, `compas_robots`, `pybullet`, `pybullet_planning`, plus `numpy` / `scipy`) via `# r:` so Rhino's ScriptEditor installs them into `scaffolding_env` on first run. Do **not** add `# r: compas_fab` — that would bypass the submodule and silently shadow it.

### Optional developer install

Use `requirements-dev.txt` when you want the optional tooling:

- PyBullet viewers
- pytest
- matplotlib-backed test visualization
- URDF/PyBullet validation work

Recommended from a normal local Python environment or virtualenv:

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

## Rhino 8 Workflow

See [docs/coordinate_conventions.md](docs/coordinate_conventions.md) for
the bar/block frame conventions and
[docs/rhino_toolbar_entrypoints.md](docs/rhino_toolbar_entrypoints.md)
for the toolbar mapping.

### Install the toolbar

1. **Tools → Options → Files → Search Paths**: add the `scripts/` folder
   so Rhino can resolve the entry-point scripts by filename.
2. **Tools → Toolbars → File → Open Toolbar File…**: open
   `scaffolding_toolbar.rui`.
3. Show the **RSDesign** and **RSSetup** toolbars and dock them.

### Toolbar entry points

| Toolbar | Button | Script | Purpose |
|---------|--------|--------|---------|
| **RSDesign** | RSCreateBar | `rs_create_bar.py` | Create a registered bar from two picked points |
| **RSDesign** | RSBarSnap | `rs_bar_snap.py` | Snap a new bar onto an existing bar at the pair's contact distance |
| **RSDesign** | RSBarBrace | `rs_bar_brace.py` | Add a brace bar between two bars (interactive solution picker) |
| **RSDesign** | RSSequenceEdit | `rs_sequence_edit.py` | Interactive assembly-sequence viewer/editor |
| **RSDesign** | RSJointPlace | `rs_joint_place.py` | Place connector blocks on a bar pair; click to flip orientation |
| **RSDesign** | RSJointEdit | `rs_joint_edit.py` | Re-edit a placed joint pair |
| **RSSetup** | RSDefineJointHalf | `rs_define_joint_half.py` | Define ONE joint half (Male / Female / Ground) from baked Rhino geometry |
| **RSSetup** | RSDefineJointMate | `rs_define_joint_mate.py` | Define a mate between two existing joint halves |
| **RSSetup** | RSMeasureGap | `rs_measure_gap.py` | Measure shortest distance between two bars |
| **RSSetup** | RSUpdatePreview | `rs_update_preview.py` | Refresh all bar tube previews |
| **RSSetup** | RSExportPrefab | `rs_export_prefab.py` | Export bar prefabrication data as JSON |

### Defining joint halves and mates

The joint registry (`scripts/core/joint_pairs.json`) is normalized into
three tables: `halves` (per block_name), `mates` (named female+male
relationships), and `ground_joints` (single-half anchors to the world).

`RSDefineJointHalf` defines ONE half:

1. Choose `Male`, `Female`, or `Ground`.
2. Pick the block instance and its representative bar axis line.
3. (Male/Female only) Pick the screw axis line and screw center point.
4. Enter the half name. For Male/Female the name MUST equal the block
   definition name; for Ground it is the ground-joint key.
5. The script exports `asset/<block_name>.3dm` and a single-mesh collision
   `asset/<block_name>.obj` (millimetres), then upserts the half into the
   registry.

`RSDefineJointMate` then defines the relationship:

1. Pick the female block + female bar axis, then the male block + male bar axis.
2. Enter the mate name.
3. The script looks up both halves by `block_name` (must already exist),
   computes `contact_distance_mm` from the two bar lines, prompts
   Accept/Edit/Cancel, and persists the mate. Half geometry is left
   untouched.

After this, the new mate name appears as a `Pair` option whenever
`RSBarSnap`, `RSBarBrace`, or `RSJointPlace` prompt for the first bar.
Pick a bar directly to use the cached default pair, or click the `Pair`
option to switch.

### Design loop

- **RSBarSnap** — pick `Le` (with optional `Pair` option), then `Ln`;
  `Ln` is translated so the line-to-line contact distance matches the
  selected pair.
- **RSBarBrace** — pick two bars and two contact points; click one of
  the colored candidate brace bars in the viewport. Use
  `SlidePointOn1` / `SlidePointOn2` to re-pick contact points.
- **RSJointPlace** — pick any two registered bars; the bar with the
  lower assembly sequence is auto-assigned female (`Le`), the later one
  male (`Ln`). The optimizer solves 4 assembly variants; click the
  female or male block to toggle its orientation. Press Enter or type
  `Accept` to bake. Missing block definitions are auto-imported from
  `asset/<block_name>.3dm`.
- **RSJointEdit** — click any previously placed female/male block to
  re-open the orientation session for that joint pair.

### Test file

- Start from `HalfJointV7_template.3dm` when testing the Rhino 8 workflow.
- It already contains the frame annotations expected by `rs_export_config.py`.
- It also contains ready-made `FemaleLinkBlock` and `MaleLinkBlock` definitions authored at the world origin in the OCF convention expected by the joint-placement workflow, so RSJointPlace works out of the box.
- If you move this workflow into another Rhino file, you only need to bring over block definitions with the same block names.
- Re-run **RSExportConfig** whenever the joint definition changes, or when adapting the workflow to a new joint family.

### Button list

The RUI file already wires these up. If you prefer manual buttons, each macro is just:

```text
! _-ScriptEditor _R "rs_<name>.py"
```

Rhino resolves the filename via Search Paths (set up above).

#### RSBakeFrame (`rs_bake_frame.py`)

- Prompts for a frame name, origin, a point on `+X`, and a point on the `+Y` side.
- Reconstructs a right-handed orthogonal frame from those picks.
- Bakes grouped RGB axis lines plus an optional text dot label.
- Use this to create the six named CAD reference frames that feed the exporter.

#### RSExportConfig (`rs_export_config.py`)

- Reads the baked Rhino frame groups and reconstructs the CAD-backed transforms.
- Computes `BAR_CONTACT_DISTANCE` from the shortest-distance segment between `le_bar_link` and `ln_bar_link`.
- Writes `scripts/core/config_generated.py` and `scripts/core/cad_frames_snapshot.json`.
- Expected frame names: `le_bar_link`, `female_link`, `female_screw_hole_link`, `male_screw_hole_link`, `male_link`, `ln_bar_link`.

#### RSBarSnap (`rs_bar_snap.py`)

- Pick an existing bar `Le` and a new bar `Ln`; the script translates `Ln` so the line-to-line contact distance matches `BAR_CONTACT_DISTANCE`.
- Deterministic, single result.

#### RSBarBrace (`rs_bar_brace.py`)

- Pick two existing bars and two contact points; the script previews candidate brace bars as colored tubes.
- Click a tube in the viewport to select that solution, or use `SlidePointOn1` / `SlidePointOn2` to re-pick contact points and re-solve.

#### RSJointPlace (`rs_joint_place.py`)

- Pick any two registered bars; the bar with the lower assembly sequence is automatically assigned female (Le), the later one male (Ln).
- The optimizer solves 4 assembly variants; the best-residual variant is shown first.
- **Click the female block** to toggle female orientation; **click the male block** to toggle male orientation.
- Press Enter or type `Accept` to bake the chosen configuration.

#### RSJointEdit (`rs_joint_edit.py`)

- Click any previously placed female or male joint block to re-open the interactive orientation session for that joint pair.
- The stored `le_rev` / `ln_rev` state is read from the block's user-text, so the session resumes exactly where it was left.
- Same click-to-flip interaction as RSJointPlace; Enter or `Accept` writes the updated blocks.

#### RSMeasureGap (`rs_measure_gap.py`)

- Computes the shortest segment between two finite Rhino line objects.
- Draws that segment (or a point if they intersect) and stores the measured distance in object user text.

#### RSExportCase (`rs_export_case.py`)

- Exports the current T1-S2 selection as a JSON debug case for replay outside Rhino.

#### RSExportGraspTool0TF (`rs_export_grasp_tool0_tf.py`)

- Top-level mode prompt: `Joint` or `Gripper`.
- **Joint** mode: prompts for a joint type string (default `T20_Male`) and arm side (`left`/`right`), then picks one baked male-joint frame group followed by one baked tool0 frame group. Computes `tf = inverse(male_ocf) @ tool0_frame` and merges into `MALE_JOINT_OCF_TO_TOOL0[joint_type][arm_side]`.
- **Gripper** mode: prompts for a gripper kind (default `Robotiq`), then picks one baked bar-grasp frame group (origin on bar centerline, Z along bar) followed by one baked tool0 frame group. Computes `tf = inverse(bar_grasp) @ tool0_frame` and merges into `BAR_GRASP_TO_TOOL0[gripper_kind]`.
- Both dicts live in `scripts/core/config_generated_ik.py`. Re-running either mode preserves the other dict and other entries within the same dict.
- Needed once per joint type/arm before running the dual-arm IK keyframe workflow, and once per gripper kind before running the support-arm IK keyframe workflow.

#### RSExportPineappleOBJ (`rs_export_pineapple_obj.py`)

- Walks the two block definitions named in `core.config` (`LEFT_PINEAPPLE_BLOCK`, `RIGHT_PINEAPPLE_BLOCK`), combines all renderable geometry into a single mesh per block, scales to meters, writes to `asset/AssemblyLeft_Pineapple_m.obj` and `asset/AssemblyRight_Pineapple_m.obj`.
- If a target file already exists, prompts per file: `Reuse` keeps the existing OBJ, `Reexport` overwrites.
- Run once per project, plus whenever the pineapple geometry changes.
- The OBJs are loaded as `compas_robots.ToolModel` instances by `core.robot_cell.get_or_load_robot_cell` so the IK self-collision check sees the wrist + tool geometry.

#### RSPBStart / RSPBStop (`rs_pb_start.py`, `rs_pb_stop.py`)

- RSPBStart launches a shared PyBullet client (GUI or Direct), loads the dual-arm Husky URDF/SRDF, and caches the client + planner in `scriptcontext.sticky`.
- Subsequent IK scripts (RSIKKeyframe, RSShowIK) reuse this same client across multiple invocations to avoid reloading geometry.
- RSPBStop disconnects it.
- Requires the `external/compas_fab` submodule checked out (see "Submodule dependencies for the IK workflow" above) plus the PyPI extras `compas`, `compas_robots`, `pybullet`, `pybullet_planning` (Rhino auto-installs these via `# r:`).

#### RSIKKeyframe (`rs_ik_keyframe.py`)

- Prerequisite: RSPBStart must have been run, and `MALE_JOINT_OCF_TO_TOOL0` must contain an entry for each picked joint's `joint_type` (populate via RSExportGraspTool0TF in Joint mode).
- Prerequisite: Rhino document contains block definitions `AssemblyLeft_Pineapple` and `AssemblyRight_Pineapple`, and at least one Brep on layer `WalkableGround`.
- Workflow:
  1. Pick the left arm male joint block, then the right arm male joint block. Both must share `male_parent_bar` (the new Ln bar being assembled).
  2. Pineapple blocks preview the wrist + tool at the derived tool0 frames so collisions can be visually inspected.
  3. Pick a base point snapped to a Brep in `WalkableGround`; the Brep face normal defines the base Z-axis. Pick a second point for the base +X heading.
  4. Prompt for collision options (self, environment).
  5. IK solves the final target; if unreachable, samples base positions in a disc of radius `IK_BASE_SAMPLE_RADIUS` around the pick (each re-snapped to the same Brep) up to `IK_BASE_SAMPLE_MAX_ITER` attempts.
  6. Repeats for the approach target, offset by `-unit(avg(male_z_L, male_z_R)) * LM_DISTANCE`.
  7. On `Accept`, serializes `ik_assembly` (robot id, base frame in world mm, `final` + `approach` per-group configs) as JSON user-text on the shared Ln bar axis line. Preview meshes and pineapples are always cleared at end of run.

#### RSShowIK (`rs_show_ik.py`)

- Prerequisite: RSPBStart must have been run.
- Pick any bar carrying an `ik_assembly` user-text record; the script rebuilds the robot cell state and displays it via `core.ik_viz`.
- Prompt: `final` (default) or `approach` to toggle which sub-record is shown.

## Standalone Developer Tools

Most of these run from a terminal, not from Rhino buttons. The `T1-S2` case exporter below is the one Rhino-side debugging exception because it captures live Rhino picks and the current solver output.

### Export T1-S2 debug case

Rhino command:

```text
! _-ScriptEditor _R "rs_export_case.py"
```

What it does:

- Exports the current `T1-S2` selection as a JSON debug case so the exact Rhino input can be replayed outside Rhino.
- Captures:
  - `Le1` and `Le2` line endpoints
  - picked contact points `Ce1` and `Ce2`
  - current `BAR_CONTACT_DISTANCE` and `BAR_RADIUS`
  - Rhino document path and model units
  - the live `solve_s2_t1_all(...)` result count and solution data, or the solver error if one is raised

Typical workflow:

1. Run `RSBarBrace` until you hit an `S2` case you want to inspect more closely, such as fewer preview candidates than expected or a suspicious candidate layout.
2. Run `rs_export_case.py` from Rhino (or click the `RSExportCase` toolbar button in `scaffolding_toolbar.rui`).
3. If the last `T1 Bar` run was already an `S2` selection, choose `Yes` to reuse the cached `Le1`, `Le2`, `Ce1`, and `Ce2`.
4. Pick a save path. The default location is `tests/debug_cases/`.
5. Commit or share the exported JSON file as a reproducible failing case.

Replay it outside Rhino:

```bash
python - <<'PY'
import json
import numpy as np
import sys

sys.path.insert(0, "scripts")
from core.geometry import solve_s2_t1_all

with open("tests/debug_cases/your_case.json", "r", encoding="utf-8") as stream:
    case = json.load(stream)

solver_input = case["solver_input"]
nn_init_hint = solver_input["nn_init_hint"]
solutions = solve_s2_t1_all(
    np.array(solver_input["n1"], dtype=float),
    np.array(solver_input["ce1"], dtype=float),
    np.array(solver_input["n2"], dtype=float),
    np.array(solver_input["ce2"], dtype=float),
    float(solver_input["distance"]),
    nn_init_hint=None if nn_init_hint is None else np.array(nn_init_hint, dtype=float),
)

print(f"{len(solutions)} solution(s)")
for index, solution in enumerate(solutions, start=1):
    print(index, solution["angles"], solution["residual"])
PY
```

### Generate URDF

```bash
python scripts/generate_urdf.py
```

What it does:

- Converts the current CAD-backed connector definition in `scripts/core/config_generated.py` into `scripts/T20_5_chain.urdf`.
- Encodes the chain:
  - `Le bar -> FJP -> FJR -> female link -> female screw hole -> JJR -> male screw hole -> male link -> MJR -> MJP -> Ln bar`

When to use it:

- Run it after `Export CAD Config` whenever the Rhino-authored reference frames change.
- Use it when you want the URDF file on disk for inspection, testing, or other downstream tooling.

Requirements:

- Core runtime only (`requirements.txt`)

### Static PyBullet URDF viewer

```bash
python scripts/pb_viz_urdf_static.py
```

What it does:

- Regenerates the URDF automatically.
- Loads the connector chain in a fixed PyBullet scene.
- Applies transparent colors and draws frame annotations so you can inspect link names, frame locations, and mesh placement.

When to use it:

- Use this right after a CAD export when you want a quick visual sanity check of the generated URDF structure before touching interactive debugging.
- It is good for checking whether the fixed transforms and mesh orientation look plausible.

Requirements:

- Developer runtime (`requirements-dev.txt`)

### Interactive PyBullet viewer

```bash
python scripts/pybullet_viewer.py
```

What it does:

- Regenerates the URDF automatically.
- Opens a PyBullet GUI with sliders for `FJP`, `FJR`, `JJR`, `MJR`, and `MJP`.
- Runs live FK sanity checks against the URDF link states.
- Runs the optimizer in the background and overlays ghost meshes for the solved female and male placements.

Typical workflow:

1. Move the sliders to create a test joint state.
2. Watch the live FK and optimization status line in the terminal.
3. Press `1` to print a detailed FK report.
4. Press `2` to print a detailed optimizer report.
5. Press `h` to print the help text again.

When to use it:

- Use this for debugging the relationship between the CAD-backed FK, the URDF, and the optimizer behavior.
- This is a developer tool, not part of the Rhino button workflow.

Requirements:

- Developer runtime (`requirements-dev.txt`)

## Tests

```bash
python -m pytest -q              # all tests
python -m pytest -q --viz        # with matplotlib visualisations
```

| Test file | What it covers |
|-----------|----------------|
| `test_geometry.py` | Line-line distance, closest points, finite segments |
| `test_s2_t1.py` | S2-T1 bar-axis solver (single + all solutions) |
| `test_joint_pair_roundtrip.py` | Joint-pair registry + canonical bar frame round-trip |
| `test_three_bar_scene.py` | End-to-end pair-placement optimizer on a 3-bar scene |

## Project Structure

```text
asset/                          # Block definition .3dm assets (per joint pair)
scripts/
  core/
    config.py                  # Public config entry point
    config_generated.py        # Auto-generated CAD geometry from Rhino
    config_generated_ik.py     # Auto-generated IK transforms (MALE_JOINT_OCF_TO_TOOL0)
    geometry.py                # Line-line distance math and S2-T1 solving
    kinematics.py              # FK and joint placement optimization
    ik_viz.py                  # Rhino-side cache for the dual-arm robot scene preview
    rhino_frame_io.py          # Shared helpers to reconstruct baked Rhino frame groups
    robot_cell.py              # Dual-arm Husky cell load + PyBullet lifecycle + IK helpers
    t1_s2_case_export.py       # Shared JSON payload builder for T1-S2 debug cases
    transforms.py              # Shared frame/transform helpers
  rs_bake_frame.py             # Rhino: bake a named frame from picked points
  rs_bar_snap.py               # Rhino: snap a new bar onto an existing bar
  rs_bar_brace.py              # Rhino: add a brace bar between two bars
  rs_joint_place.py            # Rhino: place connector blocks on a bar pair
  rs_export_config.py          # Rhino: export CAD-backed fixed transforms
  rs_export_case.py            # Rhino: export a reproducible T1-S2 solver case as JSON
  rs_export_grasp_tool0_tf.py  # Rhino: export male-joint OCF -> tool0 and bar-grasp -> tool0 transforms for IK
  rs_export_pineapple_obj.py   # Rhino: export pineapple block defs to OBJ (m) for IK tool models
  rs_pb_start.py               # Rhino: start the shared PyBullet client (GUI / Direct)
  rs_pb_stop.py                # Rhino: disconnect the shared PyBullet client
  rs_ik_keyframe.py            # Rhino: dual-arm IK keyframe workflow (main)
  rs_show_ik.py                # Rhino: replay a saved IK keyframe
  rs_measure_gap.py            # Rhino: shortest segment between two finite lines
  generate_urdf.py             # Standalone: generate URDF from CAD-backed config
  pb_viz_urdf_static.py        # Standalone: static labeled PyBullet viewer
  pybullet_viewer.py           # Standalone: interactive PyBullet viewer
tests/
  test_geometry.py
  test_s2_t1.py
  test_t1_s2_case_export.py
  test_urdf_chain.py
  viz_helpers.py               # Shared visualization helpers for --viz mode
support_materials/
  FrameX/                      # Reference codebase (git submodule)
  papers/                      # Research papers
  specs.pdf                    # Design specifications
external/
  compas_fab/                  # Pinned wip_process branch (git submodule) — loaded via sys.path in core/robot_cell.py
asset/
  husky_urdf/                  # Husky URDF/SRDF/meshes (git submodule)
```
