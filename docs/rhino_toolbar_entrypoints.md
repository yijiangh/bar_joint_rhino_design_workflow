# Rhino Toolbar Entrypoints

This is the canonical Rhino entrypoint reference for this repository.

## Toolbar source

- Toolbar file: `scaffolding_toolbar.rui`
- Rhino Search Path to add: `<repo>/scripts`

## Canonical entrypoint table

| Toolbar | Button | Script | Primary use | Typical users |
|---|---|---|---|---|
| RSDesign | RSCreateBar | `rs_create_bar.py` | Register selected curves as bars and create tube previews | Workshop participants |
| RSDesign | RSBarSnap | `rs_bar_snap.py` | Snap a new bar to contact distance from an existing bar | Workshop participants |
| RSDesign | RSBarBrace | `rs_bar_brace.py` | Solve and pick brace-bar candidates between two bars | Workshop participants |
| RSDesign | RSSequenceEdit | `rs_sequence_edit.py` | Interactive assembly sequence viewer/editor | Workshop participants |
| RSDesign | RSJointPlace | `rs_joint_place.py` | Place female/male connector blocks on a selected bar pair | Workshop participants |
| RSDesign | RSGroundPlace | `rs_ground_place.py` | Place and orient a ground joint on a bar | Workshop participants |
| RSDesign | RSJointEdit | `rs_joint_edit.py` | Re-edit orientation of a previously placed joint pair | Workshop participants |
| RSDesign | RSBarEdit | `rs_bar_edit.py` | Color, filter, and resize bars by length | Workshop participants |
| RSDesign | RSSelectBar | `rs_select_bar.py` | Type a bar id (e.g. `B4`) to select + zoom to that bar; comma-separated ids select several; loops. Read-only. | Workshop participants |
| RSDesign | RSIKKeyframe | `rs_ik_keyframe.py` | Dual-arm IK keyframe solve and save on Ln bar | Advanced IK users |
| RSDesign | RSShowBarActionPlan | `rs_show_bar_action_plan.py` | Left-click: view a bar's IK keyframes (cycle M1-M4 + base frame). Right-click (`rs_show_bar_action_plan_motion.py`): load the bar's planned motion if needed and scrub the trajectory with a slider. | Advanced IK users |
| RSDesign | ~~RSIKSupportKeyframe~~ | `rs_ik_support_keyframe.py` | Single-arm support IK keyframe workflow — **removed from the toolbar; script archived in `scripts/`** | Advanced IK users |
| RSSetup | RSDefineJointHalf | `rs_define_joint_half.py` | Define one joint half (Male/Female/Ground) and collision mesh | Joint-library authors |
| RSSetup | RSDefineJointMate | `rs_define_joint_mate.py` | Define mate between existing joint halves | Joint-library authors |
| RSSetup | RSMeasureGap | `rs_measure_gap.py` | Measure closest segment between two finite lines | Workshop participants |
| RSSetup | RSUpdatePreview | `rs_update_preview.py` | Rebuild stale/missing bar tube previews | Workshop participants |
| RSSetup | RSReorderBarID | `rs_reorder_bar_id.py` | Renumber bars to match sequence and cascade IDs | Workshop participants |
| RSSetup | RSExportPrefab | `rs_export_prefab.py` | Export bar/joint prefabrication JSON | Workshop participants |
| RSSetup | RSExportCase | `rs_export_case.py` | Export T1-S2 debug case JSON for solver replay | Developers/debugging |
| RSSetup | RSExportConfig | `rs_export_config.py` | Export CAD-derived connector config and frame snapshot | Joint-library authors |
| RSSetup | RSBakeFrame | `rs_bake_frame.py` | Bake right-handed frame group from picked points | Joint-library authors |
| RSSetup | RSDefineRoboticTool | `rs_define_robotic_tool.py` | AssemblyTool mode: define a tool candidate (block + TCP points + picked collision meshes, exports `.3dm` + `.obj`). SupportGripper mode: export bar-grasp -> tool0 transform | Advanced IK users |
| RSSetup | RSSwapRoboticTool | `rs_swap_robotic_tool.py` | Prints registered tool names; type either L/R member to make its pair ACTIVE (registry updated, all placed tools re-placed, block geometry refreshed, cell rebuild offered) | Advanced IK users |
| RSSetup | RSInspectRoboticTool | `rs_inspect_robotic_tool.py` | PyBullet viewer for ALL tool candidates: a slider steps through L/R pairs (L at origin, R at +0.30 m), each showing its tool0 + TCP frame (`M_tcp_from_block`); a two-step GUI confirm deletes a whole candidate pair (registry entries + `.3dm`/`.obj`). Reads/edits `robotic_tools.json` only — no Rhino geometry. | Advanced IK users |
| RSSetup | RSPBStart | `rs_pb_start.py` | Left-click: start shared PyBullet client/planner in Direct mode. Right-click (`rs_pb_start_gui.py`): start in GUI mode | Advanced IK users |
| RSSetup | RSPBStop | `rs_pb_stop.py` | Disconnect shared PyBullet client | Advanced IK users |
| RSSetup | RSRebuildRobotCell | `rs_rebuild_robot_cell.py` | (Re)build the static assembly collision cell (bars + joints + env obstacles + arm ToolModels), AND auto-assign WalkableGround surfaces to bars by distance (non-destructive: keeps manual picks). Run after adding/moving/resizing geometry. | Planning/export users |
| RSSetup | RSExportBarAction | `rs_export_bar_action.py` | Left-click: export ONE picked bar's action JSON. Right-click: batch-export ALL bars (runs `rs_export_all_bar_actions.py`). | Planning/export users |
| RSSetup | RSExportAllBarActions | `rs_export_all_bar_actions.py` | Right-click companion of RSExportBarAction (no standalone button). Exports EVERY bar (IK optional) + RobotCell.json + WalkableGround.json. Bars without IK are solved later by the headless base+IK sampler. | Planning/export users |
| RSSetup | RSClearIKKeyframe | `rs_clear_ik_keyframe.py` | Left-click: pick ONE bar. Right-click (`rs_clear_ik_keyframe_all.py`): EVERY bar. Either way, then choose what to erase via two toggles — Keyframe (M1-M3 IK config + legacy blob) and BasePosition (mobile base) — both default Erase. M4 home is a constant, left untouched. No PyBullet needed. | Planning/export users |
| RSSetup | RSLoadSolvedBarAction | `rs_load_solved_bar_action.py` | Left-click: pick a bar, load its `<bar>.solved_<kind>.json` (prompts keyframe/motion), sync IK to user-text, open RSShowBarActionPlan. Right-click (`rs_load_solved_bar_action_all.py`): load all solved bar actions in BarActions/ and draw every base frame. | Planning/export users |
| RSSetup | RSExportRobotCell | `rs_export_robotcell.py` | Export robot cell configuration JSON | Planning/export users |
| RSDesign | RSAssignAndShowWalkableGround | `rs_assign_and_show_walkable_ground.py` | Pick a bar; view its assigned WalkableGround(s) + the default mobile-base placement (half-transparent ghost robot), then optionally overwrite the assignment by selecting brep(s) — loops back to re-show. Auto-assigns nearest if none. Auto-assign also runs in RSRebuildRobotCell; use this to inspect/correct a specific bar. | Planning/export users |
| RSMoCap | RSReadMoCapBar | `rs_read_mocap_bar.py` | Read one rigid body from Motive and bake markers | Mocap users |
| RSMoCap | RSAlignModelThreeBars | `rs_align_model_three_bars.py` | Fit model bars to mocap bars and transform managed layers | Mocap users |

## Detailed behavior by entrypoint

### RSCreateBar (`rs_create_bar.py`)

- Registers selected axis curves as managed bars.
- Creates/update bar tube previews.

### RSBarSnap (`rs_bar_snap.py`)

- Picks existing bar `Le` and new bar `Ln`.
- Moves `Ln` to target line-line contact distance for the active joint pair.

### RSBarBrace (`rs_bar_brace.py`)

- Picks two bars and two contact points.
- Shows colored candidate braces; click candidate to accept.
- `SlidePointOn1` and `SlidePointOn2` re-pick contact points.

### RSSequenceEdit (`rs_sequence_edit.py`)

- Views and edits assembly sequence indices interactively.

### RSJointPlace (`rs_joint_place.py`)

- Picks two registered bars, then assigns lower sequence as female (`Le`) and later sequence as male (`Ln`).
- Solves assembly variants and lets users click female or male block to flip orientation.
- Enter / `Accept` bakes selected orientation.

### RSGroundPlace (`rs_ground_place.py`)

- Anchors a ground joint to a selected bar and point.
- Uses an auto-`jr` heuristic to align block +Y toward world up.

### RSJointEdit (`rs_joint_edit.py`)

- Re-opens orientation editing for an existing placed joint pair.
- Reads stored orientation state from user-text.

### RSBarEdit (`rs_bar_edit.py`)

- Groups bars by rounded length and color-codes groups.
- Supports select-by-length, resize-about-midpoint, refresh, and clean exit.

### RSSelectBar (`rs_select_bar.py`)

- Type a bar id at the command line — `B4`, `b4`, or a bare `4` all resolve to bar `B4` — and it selects that bar's centerline curve **and** tube preview, then `ZoomSelected` frames it. Handy for finding one bar in a crowded model.
- Comma-separate ids (`B4,B7`) to select several at once. The prompt loops so you can jump from bar to bar; each entry **replaces** the selection. Enter on an empty prompt (or Esc) ends the command.
- **Read-only** — reads each bar's stored `bar_id` as-is and never heals / renumbers / moves anything, so it is safe to run any time. No PyBullet needed. On the RSDesign toolbar.

### RSDefineJointHalf (`rs_define_joint_half.py`)

- Defines one half: `Male`, `Female`, or `Ground`.
- Exports `asset/<block_name>.3dm` and collision `asset/<block_name>.obj`.
- Upserts half data in `scripts/core/joint_pairs.json`.

### RSDefineJointMate (`rs_define_joint_mate.py`)

- Selects female and male halves, computes `contact_distance_mm`, and saves the named mate.
- Requires both halves to exist first.

### RSMeasureGap (`rs_measure_gap.py`)

- Computes shortest segment between two finite line objects.
- Draws the result and stores measured distance in user text.

### RSUpdatePreview (`rs_update_preview.py`)

- Rebuilds missing or stale tube previews for all registered bars.

### RSReorderBarID (`rs_reorder_bar_id.py`)

- Renumbers bars so `B<n>` matches sequence `n`.
- Cascades updated IDs into dependent joint/ground/tool metadata.

### RSExportPrefab (`rs_export_prefab.py`)

- Exports prefabrication JSON from current bar/joint state.

### RSExportCase (`rs_export_case.py`)

- Exports current T1-S2 solver input and live results as a reproducible debug case JSON.

### RSExportConfig (`rs_export_config.py`)

- Reconstructs CAD-backed transforms from baked Rhino frame groups.
- Writes `scripts/core/config_generated.py` and `scripts/core/cad_frames_snapshot.json`.

### RSBakeFrame (`rs_bake_frame.py`)

- Prompts for origin, +X point, and +Y-side point.
- Rebuilds a right-handed orthogonal frame and bakes axis geometry.

### RSDefineRoboticTool (`rs_define_robotic_tool.py`)

Two modes, chosen at the first prompt:

- **AssemblyTool** (default): define/update one assembly-tool candidate. Pick the tool
  block instance (geometry baked at the flange -- the block-local origin IS tool0),
  then the TCP origin / +X tip / +Y tip points (the frame of the male joint while
  held), then the collision-mesh source(s), then type the tool name.
  The name MUST end in `L` (left arm) or `R` (right arm); the two sides of one
  candidate share a prefix (`AT4L`/`AT4R`).
  Collision-mesh sources: hand-modeled low-poly MESH object(s) and/or a block
  instance whose definition already IS a low-poly mesh (e.g. the tool block
  itself -- it stays visible for this pick). From an instance only actual Mesh
  objects are taken; breps are never auto-meshed. All picked sources are merged
  into one OBJ, so pick the block OR the coincident loose meshes, not both
  (overlap = duplicated triangles).
  Exports `asset/<tool-name>.3dm` + the merged meshes as `asset/<tool-name>.obj`
  (block-local frame, mm) and saves `M_tcp_from_block` + `collision_filename`
  into `scripts/core/robotic_tools.json`. Reusing an exact tool name replaces
  that entry instead of adding a duplicate. Defining does NOT activate the
  candidate -- run RSSwapRoboticTool for that.
- **SupportGripper**: pick a baked bar-grasp frame group + a baked tool0 frame
  group; writes `BAR_GRASP_TO_TOOL0[gripper_kind]` into
  `scripts/core/config_generated_ik.py` (other gripper kinds preserved).

What you prepare in Rhino per candidate pair: two block definitions (left + right,
block local frame = flange/tool0; this is the visual geometry), a hand-modeled
low-poly collision mesh per side (positioned on the tool instance; a few simple
meshes are fine -- or skip the extra meshes entirely when the block itself is
already a low-poly mesh and pick the block as the collision source), plus the
three TCP points per side. Then run AssemblyTool mode twice, once per side.

### RSSwapRoboticTool (`rs_swap_robotic_tool.py`)

- Prints all tool names in `robotic_tools.json`, then prompts for the exact name
  of either L/R member. The suffix resolves the pair partner and both sides
  activate together; no candidate block is required in the active Rhino file.
- Imports or force-refreshes both selected tool definitions from their exported
  `.3dm` files even when no tool instances are currently placed.
- Updates the registry `active` entry, re-places EVERY placed tool instance with
  the side-matching new tool (each joint keeps its arm side), force-refreshes both
  block definitions from `asset/`, warns about bars whose solved IK keyframes are
  now stale, and offers an immediate RSRebuildRobotCell (or tells you to run it).
- Pre-flight: both sides must be registered with their `.3dm` + collision `.obj`
  on disk -- otherwise nothing is changed.

### RSInspectRoboticTool (`rs_inspect_robotic_tool.py`)

- Opens a **standalone PyBullet GUI window** (its own client; not the shared
  RSPBStart planner) to visually inspect every candidate in `robotic_tools.json`.
- A `candidate index` slider steps through candidates grouped into left/right
  pairs by shared name prefix (`AT4_E3_L` + `AT4_E3_R`). For the selected pair
  the left `.obj` loads at the world origin and the right `.obj` 0.30 m along +X
  (meshes are mm, scaled to metres). An unpaired candidate shows just its one
  side.
- Each tool draws two triads: a **longer** one at the mesh base = tool0 / robot
  flange, and a **shorter** one at the TCP frame from `M_tcp_from_block`
  (`base @ M_tcp_from_block`), with `<name> tool0` / `<name> tcp` labels.
- **Delete** is a two-step, GUI-only confirm (no command-line prompt): click
  `DELETE candidate (step 1: arm)` — a red banner + printout lists exactly what
  goes — then `CONFIRM DELETE (step 2)` to remove BOTH sides' registry entries
  and their `.3dm` + `.obj` files. Moving the slider cancels a pending delete.
  A file another remaining tool still references is kept. Deleting a tool that is
  the registry `active` tool is not blocked; that `active` side is cleared —
  re-pick the active pair afterwards with RSSwapRoboticTool.
- Read-only toward the Rhino document (touches only `robotic_tools.json` + asset
  files). Close the PyBullet window to end the command. Needs PyBullet (provided
  by the `scaffolding_env` `# r:` header, same as RSIKKeyframe).

### RSPBStart (`rs_pb_start.py`) and RSPBStop (`rs_pb_stop.py`)

- Start/stop shared PyBullet planner state used by IK workflows.
- Left-click starts a Direct (headless) connection; right-click (`rs_pb_start_gui.py`) starts a GUI connection. Neither prompts for the mode.
- IK scripts reuse the same client between runs.

### RSIKKeyframe (`rs_ik_keyframe.py`)

- Dual-arm IK for two selected male joints sharing one Ln bar.
- Supports collision options and base-sampling fallback when direct solve fails.
- Saves `ik_assembly` JSON on the shared Ln bar.
- **Save base / continue off-ramp:** right after you pick the base origin + heading (or reuse a saved base), it asks **Continue** vs **SaveBaseAndExit**. Enter/Continue runs the in-Rhino solve as before; SaveBaseAndExit keeps the just-saved base frame and stops, so you can solve the keyframes headlessly with `external/husky_assembly_tamp/scripts/headless_bar_action_planner.py --solve-keyframes --base saved` (export the bar first). Not shown on a RetrySameBase loop.

### RSShowBarActionPlan (`rs_show_bar_action_plan.py`) / …Motion (`rs_show_bar_action_plan_motion.py`)

- **Left-click** — the IK KEYFRAME viewer. Pick a bar; Enter cycles M1→M2→M3→M4→M4-home from the `KEY_ASSEMBLY_*` user-text; draws the active bar's **base frame** (axis triad + footprint). When `RSLoadSolvedBarAction` has populated the loaded-solved cache, it auto-starts on those bar(s), draws every base frame, and offers **NextBar/PrevBar** (the all-bars map).
- **Right-click** — the MOTION viewer. Pick a bar; if its trajectory isn't cached it loads `<bar>.solved_motion.json` from the export root and syncs the condensed IK to user-text, then **steps through the concatenated M1..M4 trajectory from the command line** (Enter/Next/Prev/Jump — a prompt, not a modal dialog, so the viewport stays free to zoom/orbit). FK-only, so the held bar follows the arm smoothly. Requires PyBullet (RSPBStart).
- **Both clicks** also temporarily color the active bar's assigned **WalkableGround** brep(s) green and print each ground's id + Rhino object id to the command line, so you can see which ground surface the robot base is allowed to stand on. The color is reverted to ByLayer when you switch bars or exit.

### RSLoadSolvedBarAction (`rs_load_solved_bar_action.py`) / …All (`rs_load_solved_bar_action_all.py`)

- **Loader only** — the visualization is RSShowBarActionPlan's job. Prompts `solved_keyframe` (base + IK keyframe configs) vs `solved_motion` (planned trajectories), and finds files under the shared export root's `BarActions/`.
- **Left-click**: pick a bar → load `<bar_id>.solved_<kind>.json`. **Right-click**: load every `<bar>.solved_<kind>.json` in the folder.
- For each loaded action it: caches the full `BarAssemblyAction` (`core.solved_action_cache`, for the motion view) and **syncs the condensed IK** (base + approach/assembled/retreat) back onto the bar's user-text via `bar_action.write_bar_keyframe_from_action` — the reverse of the export, so the on-curve record + a later re-export stay consistent. Then it launches RSShowBarActionPlan.

### RSIKSupportKeyframe (`rs_ik_support_keyframe.py`)

- **Removed from the toolbar** — no `.rui` button. The script is kept in `scripts/` for archive only; run it by hand via ScriptEditor if you need it.
- Solves support-arm IK keyframe for holding/assisting a just-assembled bar.
- Saves support keyframe data for downstream replay/export.

### RSExportBarAction (`rs_export_bar_action.py`)

- Left-click: exports one picked bar's movement/action schema for planner/monitor integration.
- Right-click on the same button runs the batch export below (`rs_export_all_bar_actions.py`) — there is no separate "RSExportAllBarActions" button.

### RSExportAllBarActions (`rs_export_all_bar_actions.py`)

- Batch export of ALL bars (not just IK-solved ones): writes `<root>/BarActions/<bar_id>.json`, `<root>/RobotCell.json`, and `<root>/WalkableGround.json`.
- Each BarAction carries the bar's `walkable_ground_ids` (set by RSRebuildRobotCell auto-assign or RSAssignAndShowWalkableGround). `WalkableGround.json` holds every WalkableGround brep meshed and keyed by its stable id.
- Bars exported without IK get a placeholder base; the headless keyframe solver (`external/husky_assembly_tamp/scripts/headless_bar_action_planner.py --solve-keyframes`) samples their base + IK afterward.

### RSAssignAndShowWalkableGround (`rs_assign_and_show_walkable_ground.py`)

- **Single left-click**, a view↔re-assign loop:
  1. **View.** Pick a bar (first run). Its assigned WalkableGround(s) are highlighted green; the default mobile-base placement is computed — the same heuristic the headless solver uses in `--base sample` mode (`husky_assembly_tamp.keyframe.walkable_ground`: `derive_seed_base` + `frame_from_origin_normal_heading`): the base stands a standoff **behind** the bar and **faces along the average male-joint insertion direction** (the way the bar is pushed to mate), which removes the left/right side ambiguity. It's drawn as a base-frame marker + a **half-transparent ghost robot** standing on that base. The ground ids + base position are printed. Then it asks: *change the assignment?*
  2. **Re-assign** (only if you answer Yes). Select the WalkableGround brep(s) you want (multiple allowed), Enter to confirm. That set **overwrites** the bar's assignment (saved to user-text), and it loops back to step 1 to re-visualize. Answer No / Esc at step 1 to finish.
- If a bar has no assignment yet, the nearest ground(s) are auto-assigned + saved first so step 1 always has something to show.
- The ghost robot is FK-only (no PyBullet needed): its link meshes are harvested once at the home pose (`core.ik_viz.get_robot_link_meshes_at_state`, from a **bare** robot cell so only the robot is drawn) and rendered translucent via `core.dynamic_preview.mesh_preview` — the same style as the IK base-sampling ghost — rigidly placed on each computed base frame. Loading the robot URDF on first use can take a moment.
- Auto-assignment also runs inside RSRebuildRobotCell, so this command is for inspecting/correcting a specific bar. On the RSDesign toolbar.

### RSClearIKKeyframe (`rs_clear_ik_keyframe.py`) / RSClearIKKeyframeAll (`rs_clear_ik_keyframe_all.py`)

- **Scope first**: **Left-click** (`rs_clear_ik_keyframe.py`) picks ONE bar; **Right-click** (`rs_clear_ik_keyframe_all.py`) targets EVERY registered bar.
- **Then what to erase**: a two-toggle command-line prompt (`rs.GetBoolean`), both defaulting to **Erase**:
  - **Keyframe** — the approach/assembled/retreat per-arm configs (`KEY_ASSEMBLY_IK_APPROACH` / `IK_ASSEMBLED` / `IK_RETREAT`, the M1–M3 results).
  - **BasePosition** — the mobile base frame (`KEY_ASSEMBLY_BASE_FRAME`).
  - Untick one to keep it — e.g. keep the base to re-solve at the same spot (RSIKKeyframe's "reuse saved base" path), or keep the keyframe and drop only the base. Both unticked = nothing to do.
- Both clicks share `clear_ik(all_bars)`; the underlying helper is `rhino_bar_registry.clear_assembly_ik_keyframe(bar_oid, clear_keyframe=..., clear_base_frame=...)`.
- The legacy `ik_assembly` blob bundles base + configs and can't be split, so it is removed whenever **either** toggle is on; the authoritative split keys carry whatever is kept. M4's home config is a constant (`config.HOME_CONF_*`), not stored per bar, so it is left untouched.
- Only edits user-text on the bar curves — no PyBullet required.

### RSExportRobotCell (`rs_export_robotcell.py`)

- Exports robot cell/environment definition for external planners.

### RSReadMoCapBar (`rs_read_mocap_bar.py`)

- Reads one OptiTrack rigid body's marker set and bakes points in Rhino.

### RSAlignModelThreeBars (`rs_align_model_three_bars.py`)

- Fits model bars to three mocap bars and applies a rigid transform to managed geometry.

## Manual macro pattern

If a toolbar button is missing, run the script directly in Rhino ScriptEditor:

```text
! _-ScriptEditor _R "rs_<name>.py"
```

Rhino resolves filenames through Search Paths, so `<repo>/scripts` must be added.

## Maintenance rule

When adding, renaming, or removing a Rhino entrypoint script:

1. Update `scaffolding_toolbar.rui`.
2. Update this file.
3. Keep install and quickstart wording in `README.md` aligned.
