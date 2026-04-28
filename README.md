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
pairs are authored interactively in Rhino with `RSDefineJointPair` and
stored in `scripts/core/joint_pairs.json` along with their `.3dm` block
assets in `asset/`.

## Setup

### Rhino 8

Each Rhino entry-point script declares its own virtual environment and
requirements via ScriptEditor directives, so no manual install is
required:

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
| **RSSetup** | RSDefineJointPair | `rs_define_joint_pair.py` | Define a new joint pair from baked Rhino geometry |
| **RSSetup** | RSMeasureGap | `rs_measure_gap.py` | Measure shortest distance between two bars |
| **RSSetup** | RSUpdatePreview | `rs_update_preview.py` | Refresh all bar tube previews |
| **RSSetup** | RSExportPrefab | `rs_export_prefab.py` | Export bar prefabrication data as JSON |

### Defining a joint pair

`RSDefineJointPair` walks you through:

1. Selecting the female and male block definitions in the document.
2. Picking a representative `Le` and `Ln` bar pair that mates them.
3. Computing the per-pair contact distance and the fixed
   `M_block_from_bar` / `M_screw_from_block` transforms.
4. Saving the pair to `scripts/core/joint_pairs.json` and exporting the
   block definitions to `asset/<block_name>.3dm` so they can be auto-imported
   on other machines.

After this, the new pair name appears as a `Pair` option whenever
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
    config.py                   # Generic constants (BAR_RADIUS, DOF ranges)
    geometry.py                 # Line-line distance + S2-T1 solver
    joint_pair.py               # JointPairDef, registry I/O, FK helpers
    joint_pair_solver.py        # 4-DOF pair-placement optimizer
    joint_pairs.json            # Registered joint pairs
    rhino_bar_registry.py       # Rhino bar/tube tracking + pick helpers
    rhino_helpers.py            # Generic Rhino utility functions
    rhino_pair_selector.py      # Combined pair-selection + bar-pick prompt
    transforms.py               # Frame/transform math
  rs_create_bar.py
  rs_bar_snap.py
  rs_bar_brace.py
  rs_sequence_edit.py
  rs_joint_place.py
  rs_joint_edit.py
  rs_define_joint_pair.py
  rs_measure_gap.py
  rs_update_preview.py
  rs_export_prefab.py
tests/
  test_geometry.py
  test_s2_t1.py
  test_joint_pair_roundtrip.py
  test_three_bar_scene.py
  viz_helpers.py                # Shared matplotlib helpers for --viz mode
scaffolding_toolbar.rui          # Rhino toolbar (RSDesign + RSSetup)
docs/
  coordinate_conventions.md
  rhino_toolbar_entrypoints.md
```
