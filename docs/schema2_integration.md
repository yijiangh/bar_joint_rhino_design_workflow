# Schema 2 export through the shared core

Branch `claude/schema2-core`, 2026-10-08. The plugin writes **schema 2 design folders** through the shared core
`bar_assembly_core` (submodule `external/bar_assembly_core`), then runs the core's file checks (A) and plan checks (B).
Spec: `external/bar_assembly_core/doc/design_format.md`. The old compas_fab export still works, behind an option.

## What works

| Piece | Where | State |
| --- | --- | --- |
| Core as a pinned submodule | `external/bar_assembly_core` (jeliag/bar_assembly_core `5d17448`), put on `sys.path` by `scripts/core/core_lib.py` | imported once, never reloaded |
| Document → `Design` | `scripts/core/design_source.py`, `RhinoDesignSource(command).build()` | tested on a fake document and on `YH_ik_test.3dm` and `examples/Example4_two_box.3dm` (read with rhino3dm) |
| Keyframe records → actions | `scripts/core/design_builder.py` (Rhino-free) | pytest: an unsolved three-bar design with a hold passes A and B with 0 errors and 0 warnings |
| `RSExportDesign` | `scripts/rs_export_design.py` | builds, writes (`write`), copies `source/`, prints A and B |
| `RSCheckDesign` | `scripts/rs_check_design.py` | reads a design folder, prints A and B |
| `RSExportAllBarActions` | asks for the format: `Schema2` (default, same as RSExportDesign) or `Legacy` (old export, for tamp's headless planner) | |

The schema 2 path needs numpy, scipy and trimesh only: no PyBullet, compas, compas_fab or the tamp submodule.
`core/config.py` no longer raises when `external/husky_assembly_tamp` is missing; only the IK commands need it.

**What is read** (all converted to metres once, in `design_source.py`):

- bars: `bars/<id>`, a cylinder (`tube_radius` or `config.BAR_RADIUS`) at the centerline's midpoint frame;
- halves: `joints/<joint id>_<role>`, the block's collision OBJ, `part` from the block name (`T20_Male` → `T20/Male`),
  `mount` from `parent_bar_id`, `markers` from `marker_points_mm`;
- ground: `ground/<WGn>`, the walkable ground breps meshed coarse; obstacles: `obstacles/<name>` from the environment
  layer;
- robots `robots/cindy|alice|belle` with the calibrated URDFs from `asset/husky_urdf`; tools: Cindy's pair and
  `tools/<robot>/SupportGripper` (Robotiq);
- mates: male with its receiver (female or MoCap) by joint id; each ground half with the nearest walkable ground;
- schedule: `hold_schedule.build_action_schedule` (J, H, R, HR), as the legacy export;
- per bar: the placed tool blocks (flange poses at the assembled pose), the half each tool is on, the base and the
  approach / assembled / retreat joints; per hold: the support grasp frame, base, approach and held joints.

**Actions** (`design_builder.py`, from the prototype `proto_builder.py`; ids are role-named):

| Action | Movements |
| --- | --- |
| `B10_J` | `load` (free, target null), `mount` (operator; bar attached to both flanges), `grasp` (grip closed), `transfer` (free, coupled), `insert` (linear, coupled, compliant, `drives: tighten`, `ends_on: tools`; bar built) |
| `B10_R` | `ungrasp` (form B: grips open, bar released), `retreat` (per-arm linear along each half's −z), `home` |
| `B3_H` | `approach` (free), `open`, `to_grasp` (linear along the flange z), `close` (Alice holds the built bar too) |
| `B3_HR` | `open` (Alice lets go), `retreat` (linear); then Alice drives away (`null`) |

`on` is set after the mount and after the support's linear approach, and cleared after the retreats. Unsolved bars
get `null` joints; a base is kept whenever one was picked.

## Choices, and why

| Choice | Why |
| --- | --- |
| A new `RSExportDesign`, and `RSExportAllBarActions` asks Schema2 / Legacy | The task asked for the old export behind an option; the legacy imports (compas_fab, PyBullet, rs_data_structure, tamp) moved inside the legacy path so Schema2 runs without them |
| `design_source` reads user text and Rhino objects itself instead of `env_collision.collect_*`, `bar_action`, `hold_action_builder` | Those import compas, compas_fab, rs_data_structure or tamp at module level. Reused: `get_bar_seq_map`, `collect_hold_inputs`, `derive_hold_plan`, `build_action_schedule`, `find_tool_for_joint`, `block_instance_xform_mm`, `_bar_world_frame_mm`, `_coerce_env_brep`, the registries |
| OBJ files parsed in `design_source.read_obj`, not with the core's `read_mesh` | trimesh needs PIL for OBJ files with texture coordinates (the Robotiq mesh fails without it) |
| Cindy's tool per flange = the variant placed most often (fallback: the active pair) | Schema 2 has one tool per flange; `YH_ik_test.3dm` places `AT3_E2L/R`, not the active `AT3L/R`, and the wrong TCP gives false B14 errors. Bars placed with another variant are listed |
| Insert direction: mean of the two flange z axes; ground bars: against the walkable ground's normal under their ground halves | As `bar_action._compute_approach_targets_mm` (approach = assembled − direction × 15 mm) |
| Retreat: each flange along its half's −z, 50 mm; support approach and retreat 100 mm along the flange z | As `_retreat_tool0_target_mm` and `approach_tool0_from_grasp`; the distances come from `config` |
| Cindy's grasp = flange⁻¹ · bar from the placed tool blocks; Alice's from forward kinematics of the held keyframe when solved, else from the grasp frame | Cindy's must match the part seat (B14 checks TCP × seat × mount); Alice's then agrees with her joints within B14's 0.1 mm |
| Home joints from `HUSKY_DUAL_ARM_HOME_CONF_12`, named `<side>_ur_arm_<UR joint>` | The legacy export's home |
| Constants: serials `0806/0804/0805`, `ground_links` = the URDF's wheel links, tool kinds `scaffolding_v3` / `robotiq`, mount contacts wrist 2–3 (Cindy) and wrist 3 (support) | Rhino has no source (review §2); same values as the converter and `robot_cell` |
| Bars without a left and a right tool are left out (J and R) and listed | They cannot be jointed; leaving them in as pre-built would hide an unfinished design. Bars that mate to them then show B2 errors |
| Holds without a picked grasp frame are left out and listed; a keyframe solved for another robot than the plan assigns keeps its grasp but loses base and joints | The grasp is the only thing the hold cannot be built without |
| Duplicate ids and scaled or mirrored joint blocks refuse the export, listing all of them | Review P15; a dict would silently drop one body |
| `source/` gets `joint_pairs.json`, `robotic_tools.json` and the .3dm as last saved | Unsaved edits are not in the copy; writing the open document to a copy was not tried |
| The untighten before the ungrasp is dropped | The spec: untighten removes a bar. **Open decision** (README #5, review C1) |

## Known gaps

- **Not run in Rhino.** Tested on CPython 3.9 with Rhino's pins (numpy 1.24.4, scipy 1.13.1, trimesh 4.12.2), with a
  fake Rhino API and rhino3dm. Rhino-only calls to watch: `Brep.ClosestPoint(point, maxDistance)` (out parameters, used
  for the ground normal; falls back to −z with a note), `Mesh.CreateFromBrep`, `sc.doc.Name`, `rs.BrowseForFolder`.
- **Open decisions, not solved** (`husky-assembly-teleop` `tasks/2026-10-09_core_integration/README.md`): allowed
  contacts (#1), `on` in targets (#2), `null` meanings and unknown robots at the origin (#3), two hold scenes (#4),
  untighten (#5), hashes and `adopt` (#6), chaining tolerance (#7), part seats catalogue (#9), who plans support moves
  (#11). No IK or collision checks run on export.
- Part seats are hard-coded in the core (`T20/Male`, `T20/Ground`): tools on `T20Deck12/Male` give B14 warnings.
- A ground half mates only the nearest walkable ground (review P4).
- Cindy stays in the scene after her last release (review C2); support robots leave after their release.
- `writer.library` is still `design_io`; mesh file names follow dict order (review P8).
- The design folder holds copies of the robot files (~40 MB) on every export.
- Standalone MoCap halves and males whose partner is on a fake bar have no mate: B5 warnings.
- The IK, viewer and load-solution commands still use the legacy cell and the old movement ids (`_J_M5_`).

## Testing in Rhino 8 tomorrow

**Setup** (once):

1. `git fetch && git checkout claude/schema2-core`, then
   `git submodule update --init external/bar_assembly_core asset/husky_urdf` (tamp is not needed for this).
2. Restart Rhino, or ScriptEditor → Tools → Reload Python 3 (CPython) Engine: a core loaded from elsewhere would be
   used instead (the export then prints `core.core_lib: WARNING bar_assembly_core is loaded from ...`).
3. Optional: run `scripts/register_command_aliases.py` once so `RSExportDesign` and `RSCheckDesign` can be typed.
   Without it: `_-ScriptEditor _R "<repo>/scripts/rs_export_design.py"`. The first run installs trimesh 4.12.2
   (`# r:` line).

**Test 1: `YH_ik_test.3dm`** (in the repo; 4 bars, only B3 has a solved keyframe, and it is stale).

1. Open it, run `RSExportDesign`, pick a parent folder, keep the name `YH_ik_test_design`.
2. Expected in the command history:
   - `Notes`: B1 needs a left and a right tool (has 0), B2 and B4 (have 1); `bars/B1 J/R`, `bars/B2 J/R`,
     `bars/B4 J/R` left out.
   - `Wrote …: 13 bodies, 4 mates, 2 actions, 8 movements; 1/1 bars with IK` (12 bodies if the walkable ground could
     not be meshed: then a note says so).
   - `A (file checks): passed`.
   - `B (plan checks)`: B2 errors (B3 built but none of its mates engaged: B1, B2, B4 are not exported); B14 errors
     at `B3_J_insert` and `B3_R_ungrasp` (the stored keyframes put the flanges 100–470 mm from the placed tools:
     stale IK); B5 warnings for the 4 male–female mates.
   - The folder holds `design.json`, `actions/B3_J.json`, `actions/B3_R.json`, `meshes/`, `robots/`, `source/`.
3. Run `RSCheckDesign` on that folder: the same A and B lines.
4. Run `RSExportAllBarActions`, press Enter at `Export format <Schema2>`: the same as step 2.

**Test 2: the study document** (the .3dm the 260814 export came from, with IK and holds).

1. `RSExportDesign`. Expected: about 48 actions (J 5, R 3, H 4, HR 2 movements each), `A passed`, and in B only data
   findings: B5 warnings for the 4 males whose partners sit on fake bars, B14 for keyframes that no longer match
   the placed tools, and B2 only if a bar was left out (see `Notes`).
2. `Notes` names every bar left out, every hold left out, and every bar placed with a tool other than the design's.
3. Optional: `RSExportAllBarActions` → `Legacy` (needs RSPBStart and the tamp submodule) still writes the old files.

**Send back**: the command history of each run (Notes, A, B), and the time each export took.
