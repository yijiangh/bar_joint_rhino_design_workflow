# Bar Joint Rhino Design Workflow

Interactive Rhino 8 scripts for placing scaffolding bars and pluggable
connector blocks ("joint pairs"). The Rhino-facing workflow uses only the
core math stack (`numpy` + `scipy`) and is split into two stages:

- **T1 – Bar axis**: snap a new bar onto an existing bar, or add a brace
  between two bars with interactive solution selection.
- **T2 – Joint placement**: place connector blocks on a bar pair using a
  4-DOF optimizer that aligns the female and male screw holes.

  Each connector family is described by a **joint pair** (a *mate*): a
  receiver + male block definition with the geometry needed to drive the
  optimizer. The receiver is a Female, or the MoCap variant of the same Type
  (a Female with an OptiTrack marker plate). Every joint block is named
  `<Type>_<Subtype>` (`T20_Female`, `T20_Male`, `T20_Ground`, `T20_MoCap`), and
  every other name -- layer, joint id, object name, collision key -- is derived
  from it in `scripts/core/joint_name_conventions.py`. Joint blocks are
  authored interactively in Rhino with `RSDefineJointHalf` /
  `RSDefineJointMate` and
  stored in `scripts/core/joint_pairs.json` along with their `.3dm` block
  assets in `asset/`.

## Three-module pipeline

This repo is the first of three modules that carry a scaffolding design from
Rhino all the way to a real robot:

![Three-module pipeline](docs/three_module_pipeline.png)

1. **Design front-end (this repo, Rhino 8 / Windows).** Design the scaffolding
   (bars, joints, assembly sequence, walkable ground), run quick IK keyframe
   checks (`RSIKKeyframe`), then export everything as one **schema 2 process
   folder** (`RSExportProcess`) through the shared core
   [`scaffolding_assembly_core`](https://github.com/yijiangh/scaffolding_assembly_core)
   (submodule `external/scaffolding_assembly_core`, format spec in its
   `doc/process_format.md`). The folder carries the complete plan -- robots
   (URDF/SRDF), tools, every bar / joint / ground / obstacle body, the mates,
   every action and movement with its start state and targets -- so the
   downstream modules never read anything from Rhino or from this repo's
   Python config.
2. **Offline planner.** `scripts/plan_process.py <process folder>` plans the
   assembly robot's movements with the motion planner of the
   `external/husky_assembly_tamp` submodule, through the core, and writes
   `solutions/<action>.json` into the process folder.
3. **Live execution ([husky-assembly-teleop], ROS2 / Ubuntu).** Loads the
   process folder and its solutions on the robot. Motion capture localizes the
   mobile base; the base tracks its planned pose as closely as it can, and --
   since it never lands on it exactly -- once the base stops, the **arm**
   motion is replanned live against the **original tool0 targets** using the
   mocap-sensed base pose. Solved plans also flow back into Rhino
   (`RSLoadSolvedBarAction` writes their keyframes onto the bars;
   `RSShowBarActionPlan` right-click scrubs the trajectories).

| Artifact | Produced by | Consumed by |
|---|---|---|
| `<process>/process.json`, `actions/`, `meshes/`, `robots/` | 1 · `RSExportProcess` | 2, 3, 1 (`RSCheckProcess`) |
| `<process>/solutions/<action>.json` (start/end joints, trajectories) | 2 · `scripts/plan_process.py` | 3 (execution), 1 (viewers, `RSLoadSolvedBarAction`) |

Every compas_fab cell -- in Rhino, in the planner and on the robot -- is built
by the core from the process's scenes (`Process.scene_at` ->
`scaffolding_assembly_core.mirrors`): a `CompasFabMirror` (its own PyBullet world) to
plan and check collisions; drawing goes part by part from the core's scene
(`scene_parts`), with no cell. The plugin
never builds a cell from Rhino objects. See
[docs/schema2_integration.md](docs/schema2_integration.md).

After pulling a new version of any submodule, reload Rhino's Python engine
(**ScriptEditor → Tools → Reload Python 3 (CPython) Engine**) so no stale
module copies linger.

[husky-assembly-teleop]: https://github.com/yijiangh/husky-assembly-teleop

## Rhino 8 Workshop Install (Start Here)

### 1. Get the repository onto your machine

Use either option:

- **Clone with Git**

  ```bash
  git clone https://github.com/<org>/bar_joint_rhino_design_workflow.git
  ```

- **Download ZIP** from GitHub, then unzip it to a local folder (for example, inside Documents).

Example install location:

`C:\Users\<your-user>\Documents\bar_joint_rhino_design_workflow`

### 2. Configure Rhino 8

1. Open **Tools -> Options -> Files -> Search Paths**.
2. Add the repository's `scripts` folder.
   Example:
   `C:\Users\<your-user>\Documents\bar_joint_rhino_design_workflow\scripts`
3. Open **Tools -> Toolbars -> File -> Open Toolbar File...**.
4. Select `scaffolding_toolbar.rui` from the repository root.
5. Show and dock the **RSDesign** and **RSSetup** toolbars (plus **RSStability**
   if you exchange layouts with the stability simulation).

### 3. Run any toolbar command once

**No manual pip install is needed for Rhino usage.** The Python packages of
all commands are listed once, in [requirements-rhino.txt](requirements-rhino.txt).
The first command you run installs them, with one pip call, into
`%LOCALAPPDATA%\bar_joint_rhino\python\py39-<hash>` (`scripts/rhino_env.py`);
every command then puts that folder on `sys.path` before it imports anything.

- **First run: several minutes, and Rhino does not respond meanwhile.** pybullet
  has no ready-made build for Python 3.9 on Windows, so pip compiles it. That
  needs **Visual Studio Build Tools** with the workload *Desktop development
  with C++* ([download](https://visualstudio.microsoft.com/visual-cpp-build-tools/)).
  Without it the command stops with a message saying so; install it and run the
  command again. pip caches the build, so later installs take about a minute.
- The install log is `%LOCALAPPDATA%\bar_joint_rhino\python\py39-<hash>.log`.
- Editing `requirements-rhino.txt` makes the next command install a fresh folder
  (the hash changes); the old one can be deleted once Rhino is closed.
- The scripts have **no** `# venv:` or `# r:` lines, on purpose: Rhino installs
  each `# r:` line with its own pip call, so a later package overwrote the pins
  of an earlier one (scipy installed numpy 2.0.2 over 1.24.4, compas_robots
  compas 2.15.1 over 2.13.0), which broke fresh installs. Do not add them back;
  add the package to `requirements-rhino.txt` instead.
- Upgrading from the old setup: nothing to do. The old
  `~\.rhinocode\py39-rh8\site-envs\scaffolding_env-*` folder is no longer used
  and can be deleted. If a command warns that a package is loaded from outside
  the new folder, restart Rhino.

## Setup (Robotic workflows and tests)

### Submodule dependencies for the IK workflow

The planning workflow (RSIKKeyframe, the plan viewers, the Grasshopper
preview) builds its compas_fab cells through the shared core
[`scaffolding_assembly_core`](https://github.com/yijiangh/scaffolding_assembly_core), vendored at
`external/scaffolding_assembly_core`, which plans with a specific development branch of
[compas_fab](https://github.com/compas-dev/compas_fab/tree/wip_process),
vendored at `external/compas_fab`, rather than pulled from PyPI --
this keeps iteration on the branch deterministic and avoids the pip-cache SHA
gotcha described in [tasks/yh_lesson.md](tasks/yh_lesson.md).

After cloning this repo (or pulling changes that touch the submodules), run:

```bash
git submodule sync
git submodule update --init --recursive
```

`git submodule sync` matters after a submodule moves. On 2026-10-14 the shared core
moved from `external/bar_assembly_core` (github.com/jeliag/bar_assembly_core) to
`external/scaffolding_assembly_core` (github.com/yijiangh/scaffolding_assembly_core), and its
*design* became a *process*. An existing checkout then also needs:

- delete the leftover `external/bar_assembly_core` folder;
- restart Rhino, reload the toolbar (`scaffolding_toolbar.rui`) and run
  `scripts/register_command_aliases.py` once: it removes the old `RSExportDesign` /
  `RSCheckDesign` aliases and adds `RSExportProcess` / `RSCheckProcess`;
- export again with `RSExportProcess`: folders holding `design.json` are no longer read, and
  `solutions/` planned against them must be planned again (`scripts/plan_process.py`);
- a pip-installed copy of the core (`pip install -e`) must be reinstalled under its new name.

`scripts/core/core_lib.py` prepends `external/scaffolding_assembly_core` and
`external/compas_fab/src` onto `sys.path` before any import of them, so Rhino
scripts see the submodule versions automatically. To switch to a different
upstream commit, `cd external/<submodule> && git fetch && git checkout <sha>`,
then reload Rhino's Python engine.

The remaining dependencies (`compas`, `compas_robots`, `pybullet`,
`pybullet_planning`, `roslibpy`, `trimesh`, `yourdfpy`, plus `numpy` / `scipy`) come from
[requirements-rhino.txt](requirements-rhino.txt) (see step 3 above). Do **not**
add `compas_fab` or `scaffolding_assembly_core` to it -- that would bypass the
submodules and silently shadow them.

The old `BarAssemblyAction` schema, `rs_data_structure`, is no longer a submodule: the plugin does not import
it. Only the core's optional `legacy/` converter (old exports -> schema 2) and the equivalence scripts in
`tests/equivalence/` need it, installed with pip (see [tests/README.md](tests/README.md)).

A third submodule, [husky_assembly_tamp](https://github.com/yijiangh/husky_assembly_tamp) at `external/husky_assembly_tamp`, is the **offline planner** (module 2 of the pipeline above). It also hosts the code shared between Rhino and offline planning: the dual-arm IK solvers, the keyframe chain solve, the walkable-ground base search, and their tuning constants (`husky_assembly_tamp.keyframe`). `scripts/core/config.py` puts it on `sys.path`, so Rhino commands import the solvers from there — one implementation for both the quick in-Rhino IK check and the headless planner.

### Analytical IK solver (ssik) — one-time venv

[ssik](https://github.com/personalrobotics/ssik) is the analytical IK solver behind `RSIKKeyframe` (and the headless planner) — closed-form inverse kinematics generated straight from our calibrated URDF (no tuning constants). It needs **Python 3.10 or newer** and ships compiled extensions (ssik 4.1.0 has Windows wheels for Python 3.10, 3.11 and 3.12), so it can't load inside Rhino's CPython 3.9; instead it runs in a small Python 3.11 "sidecar" venv at `external/ssik_env`. The sidecar code and its path resolution live in the tamp submodule (`husky_assembly_tamp.keyframe.config`): the default is `<this repo>/external/ssik_env` + the per-arm solver modules in tamp's `asset/ssik` (`external/husky_assembly_tamp/asset/ssik`), overridable per machine with the `HUSKY_SSIK_VENV_DIR` / `HUSKY_SSIK_ARTIFACT_DIR` environment variables. The scripts start and talk to the sidecar automatically. (Offline planning in a Python 3.11 venv imports ssik **in-process** instead — no sidecar; the sidecar is a Rhino-only workaround.)

**Until both exist, the Rhino commands use the `gradient` backend automatically** (`scripts/rhino_env.py` says so once per session). The built solver modules (Cindy's `left_ur_arm_ik.py` and `right_ur_arm_ik.py`, Alice's `alice_ur_arm_ik.py`, Belle's `belle_ur_arm_ik.py`) are committed in tamp's `asset/ssik/`, so only the venv is missing on a fresh machine. `HUSKY_SSIK_ARTIFACT_DIR` points elsewhere if needed.

The modules are built, and rebuilt when a calibrated URDF changes, in tamp: see its [sidecar README](external/husky_assembly_tamp/husky_assembly_tamp/keyframe/ssik_sidecar/README.md), step 2. A module only loads with the ssik version that built it. Tamp's are built with ssik 4.1.0, so the venv below pins that version.

Without ssik, `RSIKKeyframe` and `RSIKKeyframeAll` say so in a message box once per session and on every run in the command history: the `gradient` fallback takes about 20 s per failed robot base and offers no `InspectCandidates`. Either backend skips a base whose arms cannot reach the flange targets by distance alone (`scripts/core/reach_check.py`) without calling the solver.

You do **not** need Python 3.11 to be your system/default interpreter — your everyday `python` can stay 3.8–3.10. The easiest way is [uv](https://docs.astral.sh/uv/), a standalone tool (not run "through" your Python). Install it once (needs no Python):

- **Windows** (PowerShell): `powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"`
- **macOS / Linux**: `curl -LsSf https://astral.sh/uv/install.sh | sh`

(Or `pip install uv` to use your existing Python instead.) Then create the venv and install ssik:

```bash
uv venv --python 3.11 external/ssik_env
uv pip install --python external/ssik_env "ssik==4.1.0"
```

That's the whole setup. Run `RSIKKeyframe` -- the sidecar spawns on the first solve and is shut down by `RSResetPlanning`.

There is also a pure-PyBullet `gradient` backend that runs inside Rhino's own Python with no venv — slower, kept as a fallback for environments that can't host the 3.11 sidecar and for benchmarking against ssik. Select it with the `HUSKY_IK_BACKEND=gradient` environment variable (default is `ssik` when it is set up, else `gradient`; see `husky_assembly_tamp.keyframe.config` and `scripts/rhino_env.py`).

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

> **Tip for developers:** The process folder's `process.json`, `actions/*.json` and `solutions/*.json` can be large and deeply nested. [Janice](https://github.com/ErikKalkoken/janice) is a handy desktop viewer for browsing and searching big JSON files without loading everything into a text editor.

## Rhino 8 Workflow

See [docs/coordinate_conventions.md](docs/coordinate_conventions.md) for
the bar/block frame conventions and
[docs/rhino_toolbar_entrypoints.md](docs/rhino_toolbar_entrypoints.md)
for the canonical Rhino entrypoint reference:

- one complete toolbar button table (toolbar/button/script/primary use)
- detailed per-entrypoint behavior and prerequisites
- manual ScriptEditor macro pattern (`! _-ScriptEditor _R "rs_<name>.py"`)

### Typical design loop

1. Create or import bars.
2. Snap/brace bars and assign sequence.
3. Place and edit joints.
4. Run IK keyframe tools when needed.
5. Export the process folder (`RSExportProcess`; right-click checks one, `RSCheckProcess`). Bars with stale IK are
   listed: solve them again and export again.
6. Export debug or prefab artifacts as needed.

### Blank-canvas quick start

- Start from a new empty Rhino file.
- Draw one simple line (for example with the `Line` command).
- Run **RSCreateBar** to register that line as your first bar and generate its preview.
- Draw a second line and repeat **RSCreateBar** if you want to test **RSBarSnap** and **RSBarBrace** next.
- Use joint-definition tools only after you author or import matching block definitions for your chosen joint family.

### Stability-simulation exchange (RSStability toolbar)

The stability simulation exchanges whole layouts as a `node_list` /
`rod_list` / `coupler_list` JSON (see `large_scaffold.json`), where each rod
is one bar (`rod_id` = the Rhino `bar_id`) and `coupler_list` holds the
rod-to-rod connectivity:

1. **RSImportScaffoldJSON** — pick the JSON; it bakes one registered bar per
   rod, named `B<rod_id>`, sequenced bottom-up by `layer_id` then `rod_id`.
   Every field of the source file is stored (per-rod fields on the bar
   curve, `coupler_list` and anything else in document user text) so nothing
   is lost by the trip through Rhino.
2. Design as usual — **RSBarSnap** to push each coupled bar out to its joint
   contact distance, then joint placement.
3. **RSExportScaffoldJSON** — writes the same schema back out with the
   node positions taken from the snapped geometry. A node whose bars have
   been pulled apart is split into one node per position (the original id
   stays with the lowest-numbered rod); connectivity is unaffected because
   it lives in `coupler_list`. Before saving it prints a sanity report:
   the actual gap for every coupled pair, plus any uncoupled bars whose
   tubes overlap.

With nothing moved between import and export, the exported file is
identical to the imported one (covered by `tests/test_scaffold_json.py`).

### Session log

Every command writes what it did to a log file, one per Rhino session, in `logs/` at the repository root: its
prints, each prompt with the answer given, errors with their tracebacks, and where the time went (working vs.
waiting for the user, by step). The first command of a session prints the file's path; `RSShowLog` opens it.
See [docs/session_log.md](docs/session_log.md).

### MoCap joints: markers on the bars for debugging the robot assembly

A **`T20_MoCap`** joint is a T20 Female with an OptiTrack marker plate: two
spheres, **M1** and **M2**, 48.3 mm apart and 25.5 mm off the bar axis. It
takes a male exactly like a Female (or sits alone on a bar), and the jig puts
it on the bar exactly like a Female. Once the bars are in the robot cell,
Motive sees where each MoCap joint really is, so you can check against the
design whether a joint **rotated** on its bar, a bar **bent**, or a bar
**moved** during insertion, retreat, the support robot's hold / release, or
while the next bar is screwed in.

What Rhino does for this today: place the MoCap joints, export where every
marker sits in the process's `process.json` (**RSExportProcess**), refuse a bar
the monitor could not fit, and label every designed marker. Fitting the
measured markers to the design — the bar's 6-DoF pose — belongs to the teleop
monitor (husky-assembly-teleop `world/marker_fit.py`).

**0. The marker plate is recorded once per joint type** (joint-library
authors). **RSDefineJointHalf** on `T20_MoCap`: `CopyFromFemale` (its frames
are the Female's), then pick each sphere and type its label, `M1`, `M2`; the
sphere diameter is measured from the picks. Already done for T20:
`scripts/core/joint_pairs.json` › `T20_MoCap` › `marker_points_mm`,
`marker_diameter_mm` (19 mm).

**1. Place MoCap joints: at least three per bar you want to track, not all
facing the same way, not mirror-symmetric about the bar's middle.** Two joints
are never enough (the set looks the same end for end), and joints that all face
the same way are never enough (M1 and M2 swap) ([why](docs/mocap_marker_export.md)).
Spread them along the bar and turn one or two about it.

- New pair: **RSJointPlace** › JointPairAndTool, and answer
  `Receiving joint: Female | MoCap` with **MoCap**.
- Existing pair: **RSJointEdit** › **ReplaceJoint**, click the receiver
  (`J20-21_female` ↔ `J20-21_mocap`; id, bars and tool are kept).
- A bar with no partner there: **RSJointPlace** › JointOnly › **MoCap**
  (`M20-T20-0_mocap`; `Accept | Rotate` to aim the plate).

**2. Check that every pair mates.** A T20 pair only nests when its two bar
axes are exactly **36 mm** apart. If they are not, the placement prints
`interface error too large … auto-flipping receiver side` and an
`Interface origin err` above 0.05 mm, and it turns the receiver 180° even when
that does not help: the pair looks loose, and **FlipJoint cannot fix it**. This
is not MoCap-specific; a Female behaves the same. Fix the bar (move it, or
**RSBarSnap** it onto its partner), then FlipJoint / MoveJoint once.
**RSUpdatePreview** lists every pair that does not mate.

**3. Export for the jig: RSExportPrefab** → `<doc>_prefab.json`. A MoCap joint
goes out as `subtype: "MoCap"` with the usual `position_mm` / `rotation_deg` /
`ori`, plus `mocap.marker_diameter_mm` and `mocap.markers_mm`: its M1 / M2
centres in the bar frame (origin at the bar start, z along the bar). The jig
ignores those extra fields.

**4. Export for the robot: RSExportProcess.** There is no separate marker file
(the retired `MoCapMarkers.json`). The markers travel in
`process.json`: every MoCap half is a body `joints/<joint id>_mocap` with its
`markers`, its `part` and the bar it is fixed to (`mount`). Positions are in
**metres**, in the **half's own body frame**; the half's `assembly_pose` places
them in the world, and since a half follows its bar, the half's offset on the
bar (the two `assembly_pose`s) places them on the bar
([process format](external/scaffolding_assembly_core/doc/process_format.md) §4.3):

| Question | Where in `process.json` | Example |
|---|---|---|
| Which marker is this? | the half's body id and the marker's label | `joints/J20-22_mocap`, `M1` |
| Which bar is it on? | the half's `mount` | `bars/B20` |
| What kind of half is it? | the half's `part` | `T20/MoCap` |
| Where is the marker on its half? | `bodies.<half>.markers.<label>` (half body frame, metres) | — |
| Where is it in the world? | not stored: the half's `assembly_pose` @ the marker | — |
| How big is the sphere? | not stored: `joint_pairs.json` › `T20_MoCap` › `marker_diameter_mm` | `19.0` |

The bar body's frame in `process.json` is at the bar **midpoint**, z along the
bar (not the bar start, as in the prefab export;
[coordinate conventions](docs/coordinate_conventions.md)). A marker's point on
its bar is `inv(bar.assembly_pose) ∘ half.assembly_pose ∘ marker`, and its name
is `<half id>/<label>`, e.g. `joints/J20-21_mocap/M1`: the monitor's
`bar_markers` and `scripts/core/process_markers.py` compute both the same way.

RSExportProcess also, from `core.process_markers`:

- **checks the layouts before anything is written.** The export stops, with
  `Export stopped, nothing was written:` and one line per fault, for a MoCap
  half without markers, a bar with fewer than 3 markers, or a bar whose
  markers another rigid motion maps onto themselves within
  `config.MOCAP_LAYOUT_TOL_MM` (2 mm), e.g.
  `bars/B20: MoCap marker layout is ambiguous -- 1 other rigid motion(s) map the
  6 markers onto themselves within 2.0 mm; ...` (the rule:
  [docs/mocap_marker_export.md](docs/mocap_marker_export.md));
- **labels the markers after a successful write**, on the layer
  **MANAGED Scaffolding › MoCap Marker Labels**: a text dot at every designed
  sphere, `B20 #3 M1` (the bar, the marker's number on it in the monitor's
  order, its label), named after the monitor's marker
  (`joints/J20-21_mocap/M1`), and a line through each half's spheres.

**5. In Motive**: make **one rigid body per instrumented bar** from its
spheres, named after the bar (`B20`). The monitor matches the markers by
geometry, not by their Motive names; naming each after its dot
(`joints/J20-21_mocap/M1`, …) only helps reading the live view.

**Not available yet for `T20_MoCap` bars:**

- **Reading the measured markers back into Rhino.** RSReadMoCapBar and
  RSAlignModelThreeBars (RSMoCap tab) are for the older 4-marker bar rig: they
  expect two marker pairs 70–90 mm apart whose centres lie on the bar axis. A
  `T20_MoCap` pair is 48.3 mm long and its centre sits 25.5 mm off the axis,
  so RSReadMoCapBar rejects the rigid body and RSAlignModelThreeBars would fit
  the bar 25.5 mm off. Do not use them for MoCap-joint bars.
- **The fit itself** — matching the live Motive markers to the designed ones by
  geometry (Motive's marker ids are not trusted), the bar's measured 6-DoF pose
  and its error against the designed pose, per-marker residuals (bending, a
  mislabelled or occluded marker), recorded after each robot movement — is not
  in Rhino: it is the new teleop monitor's experimental `mocap_bars` plugin
  ([husky-assembly-teleop] `world/marker_fit.py`), which reads the markers from
  `process.json` (the hand-over note `tasks/mocap_bar_pose_monitor_handover.md`
  was written for the retired `MoCapMarkers.json`). Without it, compare by hand: a
  joint that **rotated** shows its M1–M2 line turned about the bar axis; a bar
  that **bent** shows markers drifting off the straight centre line in the
  middle of the bar while the ends stay; a bar that **moved** shows every
  marker shifted the same way.

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
| `test_process_source.py`, `test_process_builder.py` | Rhino document (fake API, rhino3dm) -> schema 2 process |
| `test_process_planning.py` | Planning on the core's mirrors: keyframe chain, support IK, contacts, `plan_process.py` |
| `test_process_display.py`, `test_plan_steps.py` | Drawing cells, the viewers' steps, solutions read back |

## Project Structure

This is a curated map of the files that matter most when learning or extending the project.

```text
scaffolding_toolbar.rui                 # Rhino toolbar buttons/macros
docs/rhino_toolbar_entrypoints.md       # Canonical table of toolbar -> script mapping + detailed command behavior

scripts/
  core/
    config.py                           # Runtime config + sanitized access to generated values
    joint_pair.py                       # Core data model: JointHalfDef, GroundJointDef, JointPairDef, JointRegistry
    joint_pairs.json                    # Joint registry data authored from Rhino
    rhino_bar_registry.py               # Bar registry CRUD + bar metadata in Rhino user text
    geometry.py                         # Core geometry and S2-T1 solver utilities
    joint_pair_solver.py                # Pair-specific joint solving utilities
    joint_name_conventions.py            # Every joint/bar/tool/collision NAME, from <Type>_<Subtype>
    joint_name_migration.py             # One-off rename of legacy names in old .3dm files
    joint_placement.py                  # Joint pair placement computation and bake helpers
    single_sided_placement.py           # Ground / standalone MoCap placement on one bar
    marker_points.py                    # MoCap marker spheres between block / world / bar frames
    process_markers.py                  # A process's MoCap markers per bar (as the monitor), layout check, labels
    process_source.py                    # Rhino document -> schema 2 process (scaffolding_assembly_core)
    process_builder.py                   # Keyframe records -> the process's actions (Rhino-free)
    process_planning.py                  # Process scenes -> the core's CompasFabMirror -> tamp / compas_fab solvers
    process_display.py                   # Process scenes -> the parts ik_viz draws (no compas, no PyBullet)
    plan_steps.py                       # The viewers' poses, timeline and solutions as moments of the process
    keyframe_store.py                   # Solved keyframes in bar user text (the persistent store)
    support_ik.py                       # Support-arm IK on a lent planner
    core_lib.py                         # Puts scaffolding_assembly_core + compas_fab (submodules) on sys.path
    ik_viz.py                           # Rhino preview cache: a core Scene baked part by part
    scaffold_json.py                    # node/rod/coupler layout-JSON round trip, for the
                                        #   stability-simulation exchange (see RSStability below)
    robotic_tool.py                     # Robotic tool registry helpers

  # Main Rhino entry points used most often in workshops
  rs_create_bar.py
  rs_bar_snap.py
  rs_bar_brace.py
  rs_sequence_edit.py
  rs_joint_place.py
  rs_joint_edit.py
  rs_bar_edit.py
  rs_define_joint_half.py
  rs_define_joint_mate.py
  rs_update_preview.py
  rs_reorder_bar_id.py
  rs_export_prefab.py
  rs_measure_gap.py
  rs_ik_keyframe.py                     # IK keyframes (assembly + support flows), on the core
  rs_show_bar_action_plan.py            # Plan viewer (keyframes; right-click: planned motion)
  rs_show_assembly_plan.py              # Whole-assembly stepper
  rs_load_solved_bar_action.py          # Solutions -> bar keyframes

  # Planning/export and mocap entry points
  rs_export_process.py                   # Document -> schema 2 process folder (+ file and plan checks)
  rs_check_process.py                    # File and plan checks of a process folder
  plan_process.py                        # Headless: plan a process folder's movements, write solutions/
  rs_reset_planning.py                  # Close the planning worlds and caches, stop the ssik sidecar
  rs_show_planning_world.py             # Debugging: the assembly robot's planning world in PyBullet's window
  rs_read_mocap_bar.py
  rs_align_model_three_bars.py

  # RSStability toolbar: exchange with the stability simulation / layout generator
  rs_import_scaffold_json.py            # layout JSON -> registered bars (bar_id = B<rod_id>)
  rs_export_scaffold_json.py            # registered bars -> the same JSON schema

tests/
  test_geometry.py
  test_s2_t1.py
  test_joint_pair_roundtrip.py
  test_three_bar_scene.py
  test_process_*.py, test_plan_steps.py  # Process export, planning and drawing through the core

external/
  scaffolding_assembly_core/                    # Shared core: process format, scenes, compas_fab cells (git submodule)
  compas_fab/                           # IK/planning dependency of the core (git submodule)
  husky_assembly_tamp/                  # Planner (git submodule): keyframe + motion solvers

asset/
  *.3dm                                 # Joint/block assets used by placement workflows
  husky_urdf/                           # Robot description assets used by IK workflows
```

For the full list of Rhino commands and what each one does, see
[docs/rhino_toolbar_entrypoints.md](docs/rhino_toolbar_entrypoints.md).
