# Schema 2: export, planning and viewing through the shared core

Branch `claude/schema2-core`, 2026-10-08 to 10-13. The plugin writes **schema 2 process folders** through the shared core
`bar_assembly_core` (submodule `external/bar_assembly_core`), runs the core's file checks (A) and plan checks (B), and
**plans and draws on the core**. Spec: `external/bar_assembly_core/doc/process_format.md`.

**The core is the only process → compas_fab interface.** The plugin reads the Rhino document into a `Process`
(`core.process_source`); every compas_fab cell and state it uses comes from the core, built from the process's scenes
(`Process.scene_at`). Drawing uses no compas_fab at all:

- `bar_assembly_core.mirrors.compas_fab.CompasFabMirror`: one PyBullet world per robot, to plan and check collisions;
  its planner is lent to the tamp and compas_fab solvers;
- the core's `Scene` and `scene_parts`: what `core.process_display` and `core.ik_viz` draw, part by part, without a
  cell.

The legacy cell (`robot_cell`, `robot_cell_support`, `robot_obstacles`, `env_collision`, `ik_collision_setup`,
`bar_action`, `hold_action_builder`), the old JSON export (`RobotCell.json`, `BarActions/`, `ActionSchedule.json`,
`WalkableGround.json`) and its solved sidecars are gone.

## What works

| Piece | Where | State |
| --- | --- | --- |
| Core as a pinned submodule | `external/bar_assembly_core` (jeliag/bar_assembly_core `03a18e1`: the decisions of 2026-10-09, URDFs read with yourdfpy, `scene_parts` for viewers, plan check B15); `scripts/core/core_lib.py` puts it and `external/compas_fab/src` on `sys.path` | imported once, never reloaded |
| Document → `Process` | `core/process_source.py`, `RhinoProcessSource(command, cache).build()`; also records each body's Rhino object (`SourceResult.objects`) | tested on a fake document and on `YH_ik_test.3dm` and `examples/Example4_two_box.3dm` (rhino3dm) |
| Keyframe records → actions | `core/process_builder.py` (Rhino-free): role-named ids, `on` in targets, `loosen` on the ungrasp, `pose_in_tcp` per kind of half (`poses_in_tcp`) | pytest: an unsolved three-bar process with a hold passes A and B, no B11 `on` errors |
| Keyframes in user text | `core/keyframe_store.py`: the one persistent store of solved IK, read into the process on every command | |
| `RSExportProcess`, `RSCheckProcess` | `rs_export_process.py`, `rs_check_process.py` | builds, writes, copies `source/`, prints A and B; run in Rhino 8 on 2026-10-13 (`261008_2ModulesDemo.3dm`, session logs): 204 bodies, 60 mates, 78 actions, 306 movements written in 2 to 4 s |
| `RSIKKeyframe`, `RSIKKeyframeAll` | `core/process_planning.py` (`keyframe_movements`, `hold_solve_state`, `release_check`, `state_scene`), `core/support_ik.py`; previews by `core/ik_viz.py` | headless: tamp's `solve_keyframe_chain` and compas_fab support IK solve through the mirrors; run in Rhino 8 on 2026-10-13 (`261008_2ModulesDemo.3dm`, session logs), including the base search, the chain solve and the support re-solve pass |
| `RSShowBarActionPlan`, `RSShowAssemblyPlan` | `core/plan_steps.py` (poses → process moments), each step's scene drawn by `core/ik_viz.py`; CheckCollision and the jog dialog on the mirrors (`process_planning.scene_collisions`) | steps tested headless; run in Rhino 8 on 2026-10-13 (`261008_2ModulesDemo.3dm`, session logs) |
| Motion viewer (`RSShowBarActionPlan` right-click), `RSLoadSolvedBarAction(All)` | read `<process>/solutions/` (`plan_steps.trajectory_frames`, `solved_keyframes`); RSLoadSolvedBarAction also says `SOLUTION STALE` for actions the open document changed since planning (`process_hashes` of the document's process vs `solved_against`) | tested headless; **not run in Rhino** |
| Grasshopper preview + warm-up | `core/gh_seq_preview.py`, `core/gh_warmup.py`: the timeline of the document's process drawn by `ik_viz`, no compas, no PyBullet | **not run in Grasshopper** |
| `RSResetPlanning`, `RSShowPlanningWorld` | close the core's mirrors and caches and stop the ssik sidecar; open Cindy's mirror in PyBullet's window. The walkable-ground auto-assign and base seeding run on entry to the IK, preview and export commands (`ensure_ground_and_base`) | |
| Motion planning | `scripts/plan_process.py` (headless CLI): the last waypoint snapped to the process's target joints; movements no arm moves in written `still` | constructed process: insert, retreat, home solved, the ungrasp still, written to `solutions/`; transfer fails (`rrt_failed`) |

The export needs numpy, scipy and trimesh only; planning adds compas, compas_robots, compas_fab (submodule), PyBullet,
pybullet_planning and the tamp submodule; drawing needs only the core (yourdfpy reads the URDFs and their meshes): no
compas, no PyBullet. Every core import needs yourdfpy (in `requirements-rhino.txt`).

**What is read** (all converted to metres once, in `process_source.py`):

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

**Actions** (`process_builder.py`; ids are role-named):

| Action | Movements |
| --- | --- |
| `B10_J` | `load` (free, target null), `place_bar` (operator; bar attached to both flanges; target puts the tools `on` their halves), `grasp` (grip closed), `transfer` (free, coupled), `insert` (linear, coupled, compliant, `drives: tighten`, `ends_on: tools`; bar built) |
| `B10_R` | `ungrasp` (form B: grips open, `drives: loosen`, bar released, stays built), `retreat` (per-arm linear along each half's −z; target clears `on`), `home`; then Cindy drives away (`null` until her next J) |
| `B3_H` | `approach` (free), `open`, `to_grasp` (linear along the flange z; target puts the gripper `on` the bar), `close` (Alice holds the built bar too) |
| `B3_HR` | `open` (Alice lets go), `retreat` (linear; target clears `on`), `leave` (free, target null: planned live); then Alice drives away (`null`) |

`on` lives in targets only, never changes between movements (B11). Every half a tool acts on carries
`pose_in_tcp`, its pose in the frame of that tool's TCP (`process_builder.poses_in_tcp`, from `joint_pairs.json`):
identity for male halves (`T20/Male`, `T20Deck12/Male`: the TCP is at the block), `M_tool_from_block`⁻¹ for ground
halves (`T20/Ground`: 180° about z). Receivers get no tool in Rhino's flow, so no `pose_in_tcp`; a tool put on one
is a B14 error.

## The plan viewers show the core's state

RSShowBarActionPlan and RSShowAssemblyPlan draw every body of the process from the core: bars, joints, tools,
robots, and the planner's ground and obstacle meshes. Every document layer the preview stands in for is hidden
for the session (tube previews, joint and tool blocks, Walkable Ground, Environment Obstacles) and restored on
exit. The active bar's walkable ground is drawn light grey, any other ground dark grey.

- Other previews (RSIKKeyframe) keep the document's ground and obstacles on screen: the base is picked on the
  document's ground. `ik_viz.update_state` then leaves its own copies of them out, so nothing is drawn twice
  (two coplanar surfaces flicker).

## Stale IK: the core decides

A keyframe on a bar is a robot base and joints in world values. Nothing moves them when the bar, a connector or a
tool is moved afterwards, or when the bar is copied with its user text; the bar still counts as solved.

- **The test** is the core's plan check **B15** (`plan_check.unreached_targets`): where a movement's target gives both
  joints and link poses, forward kinematics of the joints must land on the link poses (0.1 mm, 1 mrad). The link
  poses are the flange targets `process_source` reads from the placed tool blocks, so they follow the document; the
  joints are the saved keyframe.
- **Back to Rhino**: the core returns `UnreachedTarget(action, movement, bar, link, distance, angle)`;
  `core/ik_status.py` turns them into `{bar id: reason}`. `PlanReport.unreached` carries the same data for an export.
- **Where it shows**: RSUpdatePreview paints stale bars yellow and lists them; RSExportProcess prints them with the B15
  errors and names them in its message box (the process is written as it is: the plan check says it is not valid).
- **In the plan viewers** (RSShowBarActionPlan, RSShowAssemblyPlan): the stale bars are listed when the viewer opens;
  at a stale bar's steps the robot is drawn yellow, at its saved pose, and the step line says `STALE IK`.
  The viewers also colour the drawn bar and joint meshes by sequence (`rhino_bar_registry.sequence_colors`,
  `ik_viz.update_state(colors=...)`): they hide the document's tubes, which is where those colours used to go.
- The verdict is computed from the document every time, never saved on the bars.
- ! It is about reaching, not colliding: a bar added next to a solved one changes the scene, not the target.
- RSIKKeyframeAll treats a stale bar as unsolved: it is painted yellow at entry, and when selected it is solved
  again without the overwrite question, which is asked only for bars whose IK is still valid.
- ! A support robot's hold is not caught: its flange target comes from the saved grasp frame, a world value like
  the saved joints (`process_source._hold_record`), so the two agree wherever the bar is now.

## Dataflow

```
Rhino doc ──RhinoProcessSource(cache=sc.sticky)──► Process (same Geometry/Tool objects while unchanged)
   │
   ├─ RSIKKeyframe / RSIKKeyframeAll
   │     process_planning.keyframe_movements(process, sc.sticky, bar)
   │        scene_at(transfer | insert | retreat) (contacts derived by the core) → scenes.seeded(scene, cindy)
   │        → mirror_for("robots/cindy").sync(scene) (others with unknown state: mirror.parked, logged)
   │        → {M1, M2, M3: start_state, target_ee_frames; M4: home}
   │     with mirror.lend() as planner: tamp solve_keyframe_chain(planner, ...)   (base sampling as before)
   │     previews: process_planning.state_scene(scene_at(transfer), cindy, solver state) → ik_viz.update_state
   │     → keyframes written to user text (keyframe_store) → the next process reads them
   │
   ├─ support flow (after the grasp pick, which puts the hold into the process)
   │     hold.hold_scenes(process, "<bar>_H") → (start of the closing movement, start of the hold release)
   │     solve in the first: process_planning.hold_solve_state → mirror_for("robots/alice"|"robots/belle"), lent
   │     support_ik.solve_support_ik(planner, state, base, flange); each base that solves held + approach is
   │     checked in the second (hold_release_collisions) before it is accepted  → user text
   │     release_check: the second scene again, report-only, after the accept
   │
   ├─ viewers / Grasshopper: plan_steps.bar_steps | timeline → step_scene → ik_viz.update_state (every robot,
   │     tool and body by its id); CheckCollision: process_planning.scene_collisions (every placed robot's mirror)
   │
   └─ scripts/plan_process.py <process folder>   (headless)
         read → per Cindy movement: scene_at → mirror → tamp motion API → solutions/<action>.json
         → RSLoadSolvedBarAction: solutions → keyframes in user text; motion viewer: trajectories → ik_viz
```

- The mirrors live in `sc.sticky` (`bar_joint:core_mirrors`), one PyBullet "direct" world per robot. Each rebuilds
  its cell only when a `Geometry`, `Tool` or robot model object changes; the geometry cache keeps those objects while
  the document is unchanged. Measured headless on `YH_ik_test.3dm`: first mirror sync 1.5 s, next command 0.04 s.
- Drawing (`core/ik_viz.py`) has no cell: it draws a core `Scene` part by part (`process_display.drawn_parts`, from the
  core's `scene_parts`), keyed by layer key and part id ("bars/B3", "robots/cindy/<link>", "tools/cindy/AT3L"). Each part is
  baked once (visual and collision mesh, on `IK Cache::<key>::Visual` / `::Collision`, named by its id) and then moved
  by the change in its transform; a part is baked again only when its `Geometry` object changes. Robots that are
  absent, whose base is unknown, or (other than the acting one) whose joints are unknown are hidden, with what they
  hold. Offenders are looked up by design id (`ik_viz.objects_of`). The solvers' compas_fab states are drawn as their
  scene posed by them (`process_planning.state_scene`).
- No workarounds left in `process_planning.py`: the decisions of 2026-10-09 are in the core.
  - **#1 contacts**: `scene_at` derives them (`relations.allowed_contacts`): a tool on its whole part, mated halves on
    each other and each other's bar, a ground-mated half on every ground body.
  - **#3 unknown robots**: `scenes.seeded` fills in the robot being planned for; the mirror parks the others whose
    base or joints are unknown (`mirror.parked`, logged as `core planning: … parked`).
  - **#4 hold scenes**: `hold.hold_scenes`: solved at the start of the closing movement (Cindy still holds the bar
    at its assembled pose), checked at the start of the hold release. The old "bars until supported at their process
    pose, held bar hidden" rule is gone.
- Robot files are resolved once per session (`package://` → relative copies in the temp folder): compas_fab cannot
  load the source URDFs from an in-memory process.

## Choices, and why

| Choice | Why |
| --- | --- |
| No legacy engine or export behind options | The goal: the core is the only process → compas_fab interface. tamp's `headless_bar_action_planner.py` read the old export; `plan_process.py` replaces it for motion planning |
| Previews draw the scene posed by the mirror's states (`state_scene`), not a compas_fab cell | Drawing needs no compas: the core's parts carry the visual meshes; the state only gives the robot's base and joints and what it holds |
| The viewers keep their poses (approach … release) | The Grasshopper timeline and its docs are built on them; `plan_steps` maps each to a process moment |
| `process_source` reads user text and Rhino objects itself | Reused: `get_bar_seq_map`, `collect_hold_inputs`, `derive_hold_plan`, `build_action_schedule`, `find_tool_for_joint`, `block_instance_xform_mm`, `bar_world_frame_mm`, `coerce_brep`, the registries |
| OBJ files parsed in `process_source.read_obj`, not with the core's `read_mesh` | trimesh needs PIL for OBJ files with texture coordinates (the Robotiq mesh fails without it) |
| Cindy's tool per flange = the variant placed most often (fallback: the active pair) | Schema 2 has one tool per flange; the wrong TCP gives false B14 errors. Bars placed with another variant are listed |
| Insert direction: mean of the two flange z axes; ground bars: against the walkable ground's normal | As the legacy builder (approach = assembled − direction × 15 mm) |
| Retreat: each flange along its half's −z, 50 mm; support approach and retreat 100 mm along the flange z | The distances come from `config` |
| Cindy's grasp = flange⁻¹ · bar from the placed tool blocks; Alice's from forward kinematics of the held keyframe when solved | B14 checks TCP × the half's `pose_in_tcp` × mount |
| Home joints from `HUSKY_DUAL_ARM_HOME_CONF_12` | As before |
| Bars without a left and a right tool get no J and R but a `bar_placing` action (`<bar>_P`, no robot, one manual step) at their place in the schedule, and are listed | They cannot be jointed, but the bars after them must be planned around them: absent before their place in the sequence, built after it |
| Duplicate ids and scaled or mirrored joint blocks refuse the export | A dict would silently drop one body |
| The ungrasp drives `loosen` on Cindy's tools; the bar stays built | Decision #5: the untighten is folded into the ungrasp |
| `on` in targets: the mount, the support's linear approach, the retreats | Decision #2; B11 refuses `on` changing between movements |
| Hold releases get a `leave` movement (free, target null) before the robot drives away | The role table of the format (§3.1) |
| A hold candidate must clear its release scene, during the solve (RSIKKeyframe) and the support pass (RSIKKeyframeAll) | Decision #4: the second hold scene; replaces the legacy partial release check |
| Stale against the document: action hashes only | `process.json` records the producer (commit, dirty, command), so its hash differs between any two exports |

## Known gaps

- **Run in Rhino 8 on 2026-10-13:** `RSExportProcess`, `RSIKKeyframe`, `RSIKKeyframeAll`, `RSShowBarActionPlan`,
  `RSShowAssemblyPlan` and `RSUpdatePreview`, on `261008_2ModulesDemo.3dm` (`logs/rs_session_20261013_*.log`).
  **Not yet run in Rhino:** the motion viewer, `RSLoadSolvedBarAction(All)`, `RSShowPlanningWorld`,
  `RSResetPlanning`, and anything in Grasshopper. Headless, everything is tested on Rhino's CPython 3.9.10 with Rhino's pins (numpy
  1.24.4, scipy 1.13.1, trimesh 4.12.2) and compas_fab from the submodule: pytest (fake Rhino API, rhino3dm) and an
  import of every command and core module against stand-ins for the Rhino API. Rhino-only calls to watch:
  `Brep.ClosestPoint(point, maxDistance)` (out parameters), `Mesh.CreateFromBrep`, ik_viz's mesh baking and
  moving (`Mesh.Vertices.UseDoublePrecisionVertices`, `sc.doc.Objects.AddMesh` with attributes, `rs.TransformObjects`;
  tested on a fake canvas only), yourdfpy's install in Rhino, Eto in the jog dialog.
- **No headless keyframe solving.** `plan_process.py` plans motions between solved keyframes; base sampling and the
  keyframe chain run only in Rhino (RSIKKeyframe / RSIKKeyframeAll). tamp's legacy `--solve-keyframes` and
  `replay_bar_action_plan.py` read the old export and are not ported.
- Support-robot movements are not planned (decision #11); movements without arm motion are `still` only where the
  robot's joints are known (the process's start, else the previous result's end): Cindy's `place_bar` and `grasp` follow
  the unplanned `load` (target null) and stay `not_planned`; the bar transfer failed on the constructed process
  (`rrt_failed`).
- **The cradle contact is gone.** The legacy touch policy (and the removed `allow_part_contacts`) let a tool on a male
  touch the cradle receiver it seats into (`bar_cradle`: the subfloor `T20Sub*` females). The core's part rules do not
  (a tool touches only its own part), so subfloor bars may now fail IK on a tool/cradle collision. Fix in the core
  (a contact rule) or in the cradle meshes.
- Hold releases have a `leave` movement with no target: nothing plans it yet.
- Drawn parts have no colours of their own: every preview mesh takes its layer's colour (`Geometry` carries none, and
  the URDFs' materials are not read). A part is baked the first time it is shown, so the first frame a robot appears
  in pays its bake (`RSGHPreviewWarmup` places every robot to pay it up front).
- The core keeps `mirrors/cell.py` (`CompasFabCell`): `CompasFabMirror` builds its planning cell with it. Its
  `visual=True` mode has no user left.
- A ground half mates only the nearest walkable ground (review P4).
- The process folder holds copies of the robot files (~40 MB) on every export.
- The core mirror names collision pairs by geometry object: two bodies sharing one `Geometry` are reported under one
  name. The geometry cache never shares; other callers of the core must not either.
- `tests/captures/` (365 MB of legacy capture JSONs) and the `external/rs_data_structure` submodule are unused by the
  plugin (the core's optional `legacy/` converter still needs `rs_data_structure`).

## Testing in Rhino 8

**Setup** (once):

1. `git fetch && git checkout claude/schema2-core`, then `git submodule update --init external/bar_assembly_core
   external/compas_fab external/husky_assembly_tamp asset/husky_urdf`. The core is pinned at `03a18e1`.
2. Restart Rhino, or ScriptEditor → Tools → Reload Python 3 (CPython) Engine.
3. Optional: run `scripts/register_command_aliases.py` once so `RSExportProcess` and `RSCheckProcess` can be typed
   (both are also on the RSDesign toolbar: left- and right-click of one button).
4. The first command installs `requirements-rhino.txt` (README, step 3): several minutes, pybullet is compiled and needs
   Visual Studio Build Tools. yourdfpy is new in it: editing the file makes Rhino install a fresh environment. Without the ssik venv (`external/ssik_env`), IK uses the slow gradient backend: a warning on every run and a message box once per session. The solver modules themselves are committed in `asset/ssik/`.

**Test 1: export** (`YH_ik_test.3dm`, in the repo; 4 bars, only B3 has a solved keyframe, and it is stale):
`RSExportProcess` → `Wrote …: 13 bodies, 4 mates, 2 actions, 8 movements; 1/1 bars with IK`, `A (file checks): passed`,
B2 / B14 / B5 findings as before (B1, B2, B4 left out: no tool pair; since `bar_placing` they are placed by hand instead, `B1_P` …), B15 errors and a `STALE IK` line for B3, and **no B11 `on` errors**. The folder: role-named
ids (`actions/B3_J.json` with `B3_J_load` … `B3_J_insert`, `actions/B3_R.json` with `B3_R_ungrasp`, `B3_R_retreat`,
`B3_R_home`), meshes named `<owner>.<hash>.obj`, `pose_in_tcp` on the male and ground halves, tool ids `tools/cindy/…`, targets shaped as `target.robots[<robot>]` with `joints` and `link_poses`, obstacles listed in `present`, `writer.schema` 2.1, `writer.library` `bar_assembly_core`. `RSCheckProcess`
on the folder: the same.

**Test 2: IK** (the study document): `RSIKKeyframe` on a bar whose tools are placed. Expected: `rebuilt the planning
cell for robots/cindy … first build`, the built bars and holding robots drawn while picking the base (on
`IK Cache::Assembly::Visual`, each object named by its id, e.g. `bars/B3`), the ghost and reach spheres, then the chain solve and the pose preview (visual meshes;
MeshMode flips to collision). Run it again on another bar: no rebuild lines unless the geometry changed. Re-click a
held bar for the support flow: Cindy holding the bar drawn before the grasp pick, `rebuilt … robots/alice`, the support
solve (a base whose poses collide in the release scene is reported `collide in the release scene` and sampling goes
on), then `release check for held bar …`. Expect `core planning: robots/… parked` lines for robots not solved yet. `RSIKKeyframeAll` on two or three bars: solved/failed colors as before.
With the ssik backend, fail a solve on purpose and `InspectCandidates`: offenders turn red.

**Test 3: viewers**: `RSShowBarActionPlan` on a solved bar: Enter cycles approach → assembled → (hold) → retreat →
home → release …; CheckCollision prints pairs and paints them red; InteractiveCollisionCheck jogs Cindy's arms.
`RSShowAssemblyPlan`: the whole timeline, GoToBar, Jump. Grasshopper: `RSGHPreviewWarmup`, then `RSGHSequencePreview`
— the same steps, robots only (the document shows the bars).

**Test 4: planning round trip** (outside Rhino, Python with the core's `[mirrors]` extra and tamp):
`HUSKY_IK_BACKEND=gradient python scripts/plan_process.py <process folder> --bars B10`; then in Rhino
`RSLoadSolvedBarAction` → pick B10 → `motion`: the trajectory scrubs; `keyframes`: the viewer opens on B10. The
solutions list `B10_R_ungrasp` as `solved` with no trajectory (still); each planned movement ends exactly on the process's
target joints. Move a tool block of B10 in Rhino and load again: `SOLUTION STALE: B10_J …` (the document changed since
planning).

**Test 5: session commands**: `RSShowPlanningWorld` (Cindy's planning world in PyBullet's window),
`RSResetPlanning`.

**Send back**: the command history of each run, and the time each export, solve and first draw took.
