# Schema 2: export, planning and viewing through the shared core

Branch `claude/schema2-core`, 2026-10-08/09. The plugin writes **schema 2 design folders** through the shared core
`bar_assembly_core` (submodule `external/bar_assembly_core`), runs the core's file checks (A) and plan checks (B), and
**plans and draws on the core**. Spec: `external/bar_assembly_core/doc/design_format.md`.

**The core is the only design → compas_fab interface.** The plugin reads the Rhino document into a `Design`
(`core.design_source`); every compas_fab cell and state it uses comes from the core, built from the design's scenes
(`Design.scene_at`):

- `bar_assembly_core.mirrors.compas_fab.CompasFabMirror`: one PyBullet world per robot, to plan and check collisions;
  its planner is lent to the tamp and compas_fab solvers;
- `bar_assembly_core.mirrors.cell.CompasFabCell(visual=True)`: the same cell with visual meshes and no PyBullet, to
  draw (`core.design_display`, `core.ik_viz`).

The legacy cell (`robot_cell`, `robot_cell_support`, `robot_obstacles`, `env_collision`, `ik_collision_setup`,
`bar_action`, `hold_action_builder`), the old JSON export (`RobotCell.json`, `BarActions/`, `ActionSchedule.json`,
`WalkableGround.json`) and its solved sidecars are gone.

## What works

| Piece | Where | State |
| --- | --- | --- |
| Core as a pinned submodule | `external/bar_assembly_core` (jeliag/bar_assembly_core `de396c5`); `scripts/core/core_lib.py` puts it and `external/compas_fab/src` on `sys.path` | imported once, never reloaded |
| Document → `Design` | `core/design_source.py`, `RhinoDesignSource(command, cache).build()`; also records each body's Rhino object (`SourceResult.objects`) | tested on a fake document and on `YH_ik_test.3dm` and `examples/Example4_two_box.3dm` (rhino3dm) |
| Keyframe records → actions | `core/design_builder.py` (Rhino-free) | pytest: an unsolved three-bar design with a hold passes A and B |
| Keyframes in user text | `core/keyframe_store.py`: the one persistent store of solved IK, read into the design on every command | |
| `RSExportDesign`, `RSCheckDesign` | `rs_export_design.py`, `rs_check_design.py` | builds, writes, copies `source/`, prints A and B |
| `RSExportAllBarActions`, `RSExportBarAction`, `RSExportRobotCell` | run `RSExportDesign` (the old single-bar and cell exports have no schema 2 counterpart) | |
| `RSIKKeyframe`, `RSIKKeyframeAll` | `core/design_planning.py` (`keyframe_movements`, `hold_solve_state`, `release_check`), `core/support_ik.py`; previews on `core/design_display.py` | headless: tamp's `solve_keyframe_chain` and compas_fab support IK solve through the mirrors; **not run in Rhino** |
| `RSShowBarActionPlan`, `RSShowAssemblyPlan` | `core/plan_steps.py` (poses → design moments), drawing cell; CheckCollision and the jog dialog on the mirrors (`design_planning.scene_collisions`) | steps tested headless; **not run in Rhino** |
| Motion viewer (`RSShowBarActionPlan` right-click), `RSLoadSolvedBarAction(All)` | read `<design>/solutions/` (`plan_steps.trajectory_frames`, `solved_keyframes`) | tested headless; **not run in Rhino** |
| Grasshopper preview + warm-up | `core/gh_seq_preview.py`, `core/gh_warmup.py`: the timeline of the document's design on the drawing cell, no PyBullet | **not run in Grasshopper** |
| `RSRebuildRobotCell`, `RSPBStart(GUI)`, `RSPBStop` | walkable-ground auto-assign + seed bases, then rebuild / close the core's mirrors and drawing cells; GUI opens Cindy's mirror in PyBullet's window | **not run in Rhino** |
| Motion planning | `scripts/plan_design.py` (headless CLI) | constructed design: insert, retreat, home solved and written to `solutions/`; transfer fails (`rrt_failed`) |

The export needs numpy, scipy and trimesh only; planning adds compas, compas_robots, compas_fab (submodule), PyBullet,
pybullet_planning and the tamp submodule; drawing adds compas, compas_robots and compas_fab but no PyBullet.

**What is read** (all converted to metres once, in `design_source.py`):

- bars: `bars/<id>`, a cylinder (`tube_radius` or `config.BAR_RADIUS`) at the centerline's midpoint frame;
- halves: `joints/<joint id>_<role>`, the block's collision OBJ, `part` from the block name (`T20_Male` → `T20/Male`),
  `mount` from `parent_bar_id`, `markers` from `marker_points_mm`;
- ground: `ground/<WGn>`, the walkable ground breps meshed coarse; obstacles: `obstacles/<name>` from the environment
  layer;
- robots `robots/cindy|alice|belle` with the calibrated URDFs from `asset/husky_urdf`; tools: Cindy's pair and
  `tools/<robot>/SupportGripper` (Robotiq);
- mates: male with its receiver (female or MoCap) by joint id; each ground half with the nearest walkable ground;
- schedule: `hold_schedule.build_action_schedule` (J, H, R, HR);
- per bar: the placed tool blocks (flange poses at the assembled pose), the half each tool is on, the base and the
  approach / assembled / retreat joints; per hold: the support grasp frame, base, approach and held joints.

**Actions** (`design_builder.py`; ids are role-named):

| Action | Movements |
| --- | --- |
| `B10_J` | `load` (free, target null), `mount` (operator; bar attached to both flanges), `grasp` (grip closed), `transfer` (free, coupled), `insert` (linear, coupled, compliant, `drives: tighten`, `ends_on: tools`; bar built) |
| `B10_R` | `ungrasp` (form B: grips open, bar released), `retreat` (per-arm linear along each half's −z), `home` |
| `B3_H` | `approach` (free), `open`, `to_grasp` (linear along the flange z), `close` (Alice holds the built bar too) |
| `B3_HR` | `open` (Alice lets go), `retreat` (linear); then Alice drives away (`null`) |

## Dataflow

```
Rhino doc ──RhinoDesignSource(cache=sc.sticky)──► Design (same Geometry/Tool objects while unchanged)
   │
   ├─ RSIKKeyframe / RSIKKeyframeAll
   │     design_planning.keyframe_movements(design, sc.sticky, bar)
   │        scene_at(transfer | insert | retreat) + part contacts + robots with unknown state parked
   │        → mirror_for("robots/cindy").sync(scene) → {M1, M2, M3: start_state, target_ee_frames; M4: home}
   │     with mirror.lend() as planner: tamp solve_keyframe_chain(planner, ...)   (base sampling as before)
   │     previews: design_display.cell_and_state(scene_at(transfer), cindy) → ik_viz, drawn with the mirror's states
   │     → keyframes written to user text (keyframe_store) → the next design reads them
   │
   ├─ support flow (after the grasp pick, which puts the hold into the design)
   │     design_planning.hold_solve_state → mirror_for("robots/alice"|"robots/belle"), lent
   │     support_ik.solve_support_ik(planner, state, base, flange)  → user text
   │     release_check: scene_at(HR_open), collisions at held and approach joints (after the accept)
   │
   ├─ viewers / Grasshopper: plan_steps.bar_steps | timeline → step_scene → drawing cell (Cindy's; the support
   │     robots are tools in it) → ik_viz; CheckCollision: design_planning.scene_collisions (every placed robot's mirror)
   │
   └─ scripts/plan_design.py <design folder>   (headless)
         read → per Cindy movement: scene_at → mirror → tamp motion API → solutions/<action>.json
         → RSLoadSolvedBarAction: solutions → keyframes in user text; motion viewer: trajectories → drawing cell
```

- The mirrors live in `sc.sticky` (`bar_joint:core_mirrors`), one PyBullet "direct" world per robot; the drawing cells
  too (`bar_joint:core_display_cells`), one per acting robot. Each rebuilds its cell only when a `Geometry`, `Tool` or
  robot model object changes; the geometry cache keeps those objects while the document is unchanged. Measured
  headless on `YH_ik_test.3dm`: first mirror sync 1.5 s, next command 0.04 s; a drawing cell with visual meshes loads
  in ~3 s the first time per session.
- `ik_viz` holds one cell per layer key (a rebuilt cell replaces its meshes), hides robots the core parks (absent, or
  base unknown) and what they hold. Offenders are highlighted by design id on the drawn meshes.
- Workarounds for open decisions, in `design_planning.py`, not in the core:
  - **#1 contacts** (`allow_part_contacts`): a tool may touch its whole part; mated halves may touch the partner's
    bar; a tool on a male may touch the cradle receiver it mates (`bar_cradle` in `joint_pairs.json`, the subfloor
    `T20Sub*` females); ground halves any ground. These carry the legacy touch policy's rules.
  - **#3 unknown robots** (`plannable`): another robot with an unknown base or joints is parked, not placed at the
    origin.
  - **#4 hold scene** (`hold_solve_scene`): a hold is solved with the bars it supports until at their design pose and
    the held bar hidden; the release is checked in the release's own moment (`scene_at(HR_open)`).
- Robot files are resolved once per session (`package://` → relative copies in the temp folder): compas_fab cannot
  load the source URDFs from an in-memory design.

## Choices, and why

| Choice | Why |
| --- | --- |
| No legacy engine or export behind options | The goal: the core is the only design → compas_fab interface. tamp's `headless_bar_action_planner.py` read the old export; `plan_design.py` replaces it for motion planning |
| Previews draw a drawing cell posed with the mirror's states | Both cells come from the same scene, so tool and body keys match; the mirror's cell has no visual meshes |
| `RSPBStart` / `RSRebuildRobotCell` / `RSPBStop` kept, as core refreshes | The toolbar keeps working without a `.rui` change; no command needs them any more |
| `RSExportBarAction` / `RSExportRobotCell` run `RSExportDesign` | Same reason; schema 2 has no single-bar or cell-only file |
| The viewers keep their poses (approach … release) | The Grasshopper timeline and its docs are built on them; `plan_steps` maps each to a design moment |
| `design_source` reads user text and Rhino objects itself | Reused: `get_bar_seq_map`, `collect_hold_inputs`, `derive_hold_plan`, `build_action_schedule`, `find_tool_for_joint`, `block_instance_xform_mm`, `bar_world_frame_mm`, `coerce_brep`, the registries |
| OBJ files parsed in `design_source.read_obj`, not with the core's `read_mesh` | trimesh needs PIL for OBJ files with texture coordinates (the Robotiq mesh fails without it) |
| Cindy's tool per flange = the variant placed most often (fallback: the active pair) | Schema 2 has one tool per flange; the wrong TCP gives false B14 errors. Bars placed with another variant are listed |
| Insert direction: mean of the two flange z axes; ground bars: against the walkable ground's normal | As the legacy builder (approach = assembled − direction × 15 mm) |
| Retreat: each flange along its half's −z, 50 mm; support approach and retreat 100 mm along the flange z | The distances come from `config` |
| Cindy's grasp = flange⁻¹ · bar from the placed tool blocks; Alice's from forward kinematics of the held keyframe when solved | B14 checks TCP × seat × mount |
| Home joints from `HUSKY_DUAL_ARM_HOME_CONF_12` | As before |
| Bars without a left and a right tool are left out (J and R) and listed | They cannot be jointed |
| Duplicate ids and scaled or mirrored joint blocks refuse the export | A dict would silently drop one body |
| The untighten before the ungrasp is dropped | **Open decision** (README #5, review C1) |

## Known gaps

- **Nothing of this has run inside Rhino or Grasshopper.** Tested on Rhino's CPython 3.9.10 with Rhino's pins (numpy
  1.24.4, scipy 1.13.1, trimesh 4.12.2) and compas_fab from the submodule: pytest (fake Rhino API, rhino3dm) and an
  import of every command and core module against stand-ins for the Rhino API. Rhino-only calls to watch:
  `Brep.ClosestPoint(point, maxDistance)` (out parameters), `Mesh.CreateFromBrep`, ik_viz's baking of compas_fab
  scene objects for the drawing cell, Eto in the jog dialog.
- **No headless keyframe solving.** `plan_design.py` plans motions between solved keyframes; base sampling and the
  keyframe chain run only in Rhino (RSIKKeyframe / RSIKKeyframeAll). tamp's legacy `--solve-keyframes` and
  `replay_bar_action_plan.py` read the old export and are not ported.
- Support-robot movements are not planned (decision #11); movements without arm motion are `not_planned`
  (decision #7); the bar transfer failed on the constructed design (`rrt_failed`).
- **Open decisions, not solved** (`husky-assembly-teleop` `tasks/2026-10-09_core_integration/README.md`): allowed
  contacts (#1), `on` in targets (#2), `null` meanings and unknown robots at the origin (#3), two hold scenes (#4),
  untighten (#5), hashes and `adopt` (#6), chaining tolerance (#7), part seats catalogue (#9), who plans support moves
  (#11).
- Part seats are hard-coded in the core (`T20/Male`, `T20/Ground`): tools on `T20Deck12/Male` give B14 warnings.
- A ground half mates only the nearest walkable ground (review P4).
- Cindy stays in the scene after her last release (review C2): the release poses show her at home (the legacy viewer
  parked her).
- `writer.library` is still `design_io`; mesh file names follow dict order (review P8).
- The design folder holds copies of the robot files (~40 MB) on every export.
- The partial release check before a hold is accepted (legacy checkpoint 1) is gone: the release is checked after the
  accept, report-only.
- The core mirror names collision pairs by geometry object: two bodies sharing one `Geometry` are reported under one
  name. The geometry cache never shares; other callers of the core must not either.
- The toolbar still lists RSPBStart / RSPBStop / RSRebuildRobotCell / RSExportBarAction / RSExportRobotCell; they
  work, but could be removed or renamed.
- `tests/captures/` (365 MB of legacy capture JSONs) and the `external/rs_data_structure` submodule are unused by the
  plugin (the core's optional `legacy/` converter still needs `rs_data_structure`).

## Testing in Rhino 8

**Setup** (once):

1. `git fetch && git checkout claude/schema2-core`, then `git submodule update --init external/bar_assembly_core
   external/compas_fab external/husky_assembly_tamp asset/husky_urdf`. The core commits `962e371` and `de396c5` are
   local until pushed to jeliag/bar_assembly_core.
2. Restart Rhino, or ScriptEditor → Tools → Reload Python 3 (CPython) Engine.
3. Optional: run `scripts/register_command_aliases.py` once so `RSExportDesign` and `RSCheckDesign` can be typed
   (both are also on the RSDesign toolbar: left- and right-click of one button).
4. The first command installs `requirements-rhino.txt` (README, step 3): several minutes, pybullet is compiled and needs
   Visual Studio Build Tools. Without ssik set up, IK uses the gradient backend (one line in the command history).

**Test 1: export** (`YH_ik_test.3dm`, in the repo; 4 bars, only B3 has a solved keyframe, and it is stale):
`RSExportDesign` → `Wrote …: 13 bodies, 4 mates, 2 actions, 8 movements; 1/1 bars with IK`, `A (file checks): passed`,
B2 / B14 / B5 findings as before (B1, B2, B4 left out: no tool pair). `RSCheckDesign` on the folder: the same.

**Test 2: IK** (the study document): `RSIKKeyframe` on a bar whose tools are placed. Expected: `rebuilt the planning
cell for robots/cindy … first build`, `rebuilt the drawing cell for robots/cindy`, the built bars and holding robots
drawn while picking the base, the ghost and reach spheres, then the chain solve and the pose preview (visual meshes;
MeshMode flips to collision). Run it again on another bar: no rebuild lines unless the geometry changed. Re-click a
held bar for the support flow: Cindy holding the bar drawn before the grasp pick, `rebuilt … robots/alice`, the support
solve, then `release check for held bar …`. `RSIKKeyframeAll` on two or three bars: solved/failed colors as before.
With the ssik backend, fail a solve on purpose and `InspectCandidates`: offenders turn red.

**Test 3: viewers**: `RSShowBarActionPlan` on a solved bar: Enter cycles approach → assembled → (hold) → retreat →
home → release …; CheckCollision prints pairs and paints them red; InteractiveCollisionCheck jogs Cindy's arms.
`RSShowAssemblyPlan`: the whole timeline, GoToBar, Jump. Grasshopper: `RSGHPreviewWarmup`, then `RSGHSequencePreview`
— the same steps, robots only (the document shows the bars).

**Test 4: planning round trip** (outside Rhino, Python with the core's `[mirrors]` extra and tamp):
`HUSKY_IK_BACKEND=gradient python scripts/plan_design.py <design folder> --bars B10`; then in Rhino
`RSLoadSolvedBarAction` → pick B10 → `motion`: the trajectory scrubs; `keyframes`: the viewer opens on B10.

**Test 5: session commands**: `RSRebuildRobotCell` (walkable-ground and base summary, planning cells built),
`RSPBStartGUI` (Cindy's planning world in PyBullet's window), `RSPBStop`.

**Send back**: the command history of each run, and the time each export, solve and first draw took.
