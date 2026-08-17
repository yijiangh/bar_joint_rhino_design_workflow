# Rhino Toolbar Entrypoints

This is the canonical Rhino entrypoint reference for this repository.

## Toolbar source

- Toolbar file: `scaffolding_toolbar.rui`
- Rhino Search Path to add: `<repo>/scripts`

## Canonical entrypoint table

| Toolbar | Button | Script | Primary use | Typical users |
|---|---|---|---|---|
| RSDesign | RSCreateBar | `rs_create_bar.py` | Register selected curves as bars and create tube previews | Workshop participants |
| RSDesign | RSRemoveBar | `rs_remove_bar.py` | Remove selected bar(s) and clean up all associated joint and tool instances | Workshop participants |
| RSDesign | RSBarSnap | `rs_bar_snap.py` | Snap a new bar to contact distance from an existing bar | Workshop participants |
| RSDesign | RSBarBrace | `rs_bar_brace.py` | Solve and pick brace-bar candidates between two bars | Workshop participants |
| RSDesign | RSBarSubfloor | `rs_bar_subfloor.py` | Add a subfloor bar between two existing bars, with independent left/right joint pairs | Workshop participants |
| RSDesign | RSSequenceEdit | `rs_sequence_edit.py` | Interactive assembly sequence viewer/editor | Workshop participants |
| RSDesign | RSJointPlace | `rs_joint_place.py` | Place female/male connector blocks on a selected bar pair | Workshop participants |
| RSDesign | RSGroundPlace | `rs_ground_place.py` | Place and orient a ground joint on a bar | Workshop participants |
| RSDesign | RSJointEdit | `rs_joint_edit.py` | Re-edit orientation of a previously placed joint pair | Workshop participants |
| RSDesign | RSBarEdit | `rs_bar_edit.py` | Color, filter, and resize bars by length | Workshop participants |
| RSDesign | RSSelectBar | `rs_select_bar.py` | **SelectByName**: type a bar id (e.g. `B4`) to select + zoom to that bar; comma-separated ids select several. **SelectByLength**: type a length in mm to select every bar of that length plus their male/ground joints, grouped exactly as RSBarEdit groups them. Both loop. Read-only. | Workshop participants |
| RSDesign | RSSelectJoint | `rs_select_joint.py` | Type a joint id (e.g. `J40-53_female`, or `J40-53` for both halves) to select + zoom to that placed joint block; comma-separated ids select several; loops. Read-only. | Workshop participants |
| RSDesign | RSTempPlace | `rs_temp_place.py` | **Temporary, mock-up only.** For every male/ground joint with no female yet, create a 200 mm stub bar centred under where its female belongs: **PlaceFemaleJointBar** adds the mating female half too, **PlaceBar** adds the bar only. Ground joints have no mate frame in `joint_pairs.json`, so you pick which pair's male plug they mate like. Stub bars are marked **fake** — they sequence and the IK sees their joints, but the prefab / bar-action / robot-cell exports skip them. | Workshop participants |
| RSDesign | RSIKKeyframe | `rs_ik_keyframe.py` | The one IK keyframe button: assembly flow (dual-arm approach/assembled/retreat) by default; re-clicking a solved bar that needs holding runs the SUPPORT flow (gripper grasp pick, support robot base pick, held+approach IK, release validation) | Advanced IK users |
| RSDesign | RSShowAssemblyPlan | `rs_show_assembly_plan.py` | Step through the WHOLE assembly movement by movement: every bar's poses concatenated in sequence order, with support robots appearing/leaving per the hold schedule. Enter=next, Prev/Jump/GoToBar, or click a bar to jump there. | Advanced IK users |
| RSDesign | RSShowBarActionPlan | `rs_show_bar_action_plan.py` | Left-click: view a bar's timeline (approach / assembled / hold / retreat / home + base frame), with support robots appearing per pose exactly as the hold schedule says; the last stabilizer of a hold also plays that hold's release pose. Right-click (`rs_show_bar_action_plan_motion.py`): load the bar's planned motion if needed and scrub the trajectory with a slider. | Advanced IK users |
| RSDesign | ~~RSIKSupportKeyframe~~ | (deleted) | Single-arm support IK — **folded into RSIKKeyframe's support flow**; the old script is deleted (reusable pickers live in `core/support_grasp_pick.py`) | Advanced IK users |
| RSSetup | RSDefineJointHalf | `rs_define_joint_half.py` | Define one joint half (Male/Female/Ground) and collision mesh | Joint-library authors |
| RSSetup | RSDefineJointMate | `rs_define_joint_mate.py` | Define mate between existing joint halves | Joint-library authors |
| RSSetup | RSMeasureGap | `rs_measure_gap.py` | Measure closest segment between two finite lines | Workshop participants |
| RSSetup | RSUpdatePreview | `rs_update_preview.py` | Left-click: one idempotent repair pass -- rebuild stale/missing bar tube previews, reload joint blocks whose `asset/*.3dm` changed (in place), restore/re-side/re-snap robotic tools, then report unmated joint pairs and broken bar/joint/tool links. Never moves a joint. Then paints the IK + broken-link overlay and pops up a tally — there is no longer a job prompt, the overlay is always painted. Right-click, **RSClearColorPreview** (`rs_clear_color_preview.py`): clear every color preview. | Workshop participants |
| RSSetup | RSReorderBarID | `rs_reorder_bar_id.py` | Renumber bars to match sequence and cascade IDs | Workshop participants |
| RSSetup | RSExportPrefab | `rs_export_prefab.py` | Export bar/joint prefabrication JSON | Workshop participants |
| RSSetup | RSExportBarTool0TF | `rs_export_bar_tool0_tf.py` | Pick a bar; export its bar-OCF -> `tool0_left` / `tool0_right` transforms to `<root>/BarTool0TF/<bar_id>.json` | Advanced IK users |
| RSSetup | RSBakeFrame | `rs_bake_frame.py` | Bake right-handed frame group from picked points | Joint-library authors |
| RSSetup | RSDefineRoboticTool | `rs_define_robotic_tool.py` | AssemblyTool mode: define a tool candidate (block + TCP points + picked collision meshes, exports `.3dm` + `.obj`). SupportGripper mode: export bar-grasp -> tool0 transform | Advanced IK users |
| RSSetup | RSSwapRoboticTool | `rs_swap_robotic_tool.py` | Prints registered tool names; type either L/R member to make its pair ACTIVE (registry updated, all placed tools re-placed, block geometry refreshed, cell rebuild offered) | Advanced IK users |
| RSSetup | RSInspectRoboticTool | `rs_inspect_robotic_tool.py` | PyBullet viewer for ALL tool candidates: a slider steps through L/R pairs (L at origin, R at +0.30 m), each showing its tool0 + TCP frame (`M_tcp_from_block`); a two-step GUI confirm deletes a whole candidate pair (registry entries + `.3dm`/`.obj`). Reads/edits `robotic_tools.json` only — no Rhino geometry. | Advanced IK users |
| RSSetup | RSPBStart | `rs_pb_start.py` | Left-click: start shared PyBullet client/planner in Direct mode. Right-click, **RSPBStartGUI** (`rs_pb_start_gui.py`): start in GUI mode | Advanced IK users |
| RSSetup | RSPBStop | `rs_pb_stop.py` | Disconnect shared PyBullet client | Advanced IK users |
| RSSetup | RSRebuildRobotCell | `rs_rebuild_robot_cell.py` | (Re)build the static assembly collision cell (bars + joints + env obstacles + arm ToolModels), AND auto-assign WalkableGround surfaces to bars by distance (non-destructive: keeps manual picks). Run after adding/moving/resizing geometry. | Planning/export users |
| RSSetup | RSExportBarAction | `rs_export_bar_action.py` | Left-click: export ONE picked bar's action JSON. Right-click: batch-export ALL bars (runs `rs_export_all_bar_actions.py`). | Planning/export users |
| RSSetup | RSExportAllBarActions | `rs_export_all_bar_actions.py` | Right-click companion of RSExportBarAction (no standalone button). Exports EVERY bar (IK optional) + RobotCell.json + WalkableGround.json. Bars without IK are solved later by the headless base+IK sampler. | Planning/export users |
| RSSetup | RSClearIKKeyframe | `rs_clear_ik_keyframe.py` | Left-click: pick ONE bar. Right-click (`rs_clear_ik_keyframe_all.py`): EVERY bar. Either way, then choose what to erase via two toggles — Keyframe (M1-M3 IK config + legacy blob) and BasePosition (mobile base) — both default Erase. M4 home is a constant, left untouched. No PyBullet needed. | Planning/export users |
| RSSetup | RSLoadSolvedBarAction | `rs_load_solved_bar_action.py` | Left-click: pick a bar, load its `<bar>.solved_<kind>.json` (prompts keyframe/motion), sync IK to user-text, open RSShowBarActionPlan. Right-click, **RSLoadSolvedBarActionAll** (`rs_load_solved_bar_action_all.py`): load all solved bar actions in BarActions/ and draw every base frame. | Planning/export users |
| RSSetup | RSExportRobotCell | `rs_export_robotcell.py` | Export robot cell configuration JSON | Planning/export users |
| RSDesign | RSAssignAndShowWalkableGround | `rs_assign_and_show_walkable_ground.py` | Pick a bar; view its assigned WalkableGround(s) + the default mobile-base placement (half-transparent ghost robot), then optionally overwrite the assignment by selecting brep(s) — loops back to re-show. Auto-assigns nearest if none. Auto-assign also runs in RSRebuildRobotCell; use this to inspect/correct a specific bar. | Planning/export users |
| RSMoCap | RSReadMoCapBar | `rs_read_mocap_bar.py` | Read one rigid body from Motive and bake markers | Mocap users |
| RSMoCap | RSAlignModelThreeBars | `rs_align_model_three_bars.py` | Fit model bars to mocap bars and transform managed layers | Mocap users |
| RSSetup | RSRegisterAliases | `register_command_aliases.py` | Register a typeable command alias for every RS\* toolbar macro, so you can **type** `RSJointEdit` in the command line instead of hunting for its button. Run once per machine; re-run after macro changes. Aliases persist across Rhino sessions. | Workshop participants |
| RSStability | RSSetRodLayer | `rs_set_rod_layer.py` | Select bars (centerlines or tube previews) and assign their `layer_id`: `0` also sets `grounded` true, `1`/`2`/... set it false. Both fields go straight into `rod_list` on the next RSExportScaffoldJSON | Workshop participants |
| RSStability | RSImportScaffoldJSON | `rs_import_scaffold_json.py` | Import a `node_list`/`rod_list`/`coupler_list` layout JSON as registered bars (`bar_id` = `B<rod_id>`), sequenced by `layer_id` then `rod_id` | Workshop participants |
| RSStability | RSExportScaffoldJSON | `rs_export_scaffold_json.py` | Export the current bars back to the same JSON schema, with node positions taken from the snapped geometry | Workshop participants |

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
- The robotic tool it auto-places is rolled by the definition's
  `M_tool_from_block` (picked in RSDefineJointHalf, see below). That rotation
  moves the **tool only** — never the ground block, whose own orientation stays
  pinned by the auto-`jr` heuristic above.

### RSJointEdit (`rs_joint_edit.py`)

- Re-opens orientation editing for an existing placed joint pair.
- Reads stored orientation state from user-text.
- Clicking a **tool instance** toggles it between the active pair's L/R tools. That is
  treated as a deliberate hand-pick: it stamps `config.KEY_TOOL_SIDE_MANUAL` on the joint
  block, and from then on the automatic approach-based side rule (RSUpdatePreview step 4,
  and both IK keyframe commands) leaves that joint alone rather than putting it back.
  An explicit base **Flip** in RSIKKeyframeAll clears the mark, since that is you
  re-deciding which side the robot approaches from.

### RSBarEdit (`rs_bar_edit.py`)

- Groups bars by rounded length and color-codes groups.
- Supports select-by-length, resize-about-midpoint, refresh, and clean exit.

### RSSelectBar (`rs_select_bar.py`)

Two modes, chosen at the first prompt: **SelectByName** (the default — press Enter) or **SelectByLength**.

- **SelectByName.** Type a bar id at the command line — `B4`, `b4`, or a bare `4` all resolve to bar `B4` — and it selects that bar's centerline curve **and** tube preview, then `ZoomSelected` frames it. Handy for finding one bar in a crowded model.
- Comma-separate ids (`B4,B7`) to select several at once. The prompt loops so you can jump from bar to bar; each entry **replaces** the selection. Enter on an empty prompt (or Esc) ends the command.
- **SelectByLength.** Type a length in mm (`1050`) and it selects every bar of that length **plus the male and ground joint blocks sitting on them** — the halves that belong to those bars, the way their tube previews do. The *female* half of a joint belongs to the bar on the other side of the pair and is deliberately left alone. A summary of every length, its bar count and its bar ids is printed on entry, and the prompt loops with your last length as the default.
- The grouping, the length prompt and that summary are RSBarEdit's own (`build_length_groups` / `pick_length_group` / `print_length_summary`), so a length means the **same set of bars** in both commands. Only the selecting is this command's.
- **Read-only** — reads each bar's stored `bar_id` as-is and never heals / renumbers / moves anything, so it is safe to run any time. Note RSBarEdit reaches the same groups through `get_all_bars`, which *does* heal bar ids as it goes; this command does not. No PyBullet needed. On the RSDesign toolbar.

### RSSelectJoint (`rs_select_joint.py`)

- Type a joint id at the command line — `J40-53_female`, `j40-53_male`, or `G4-floor-0_ground` (all case-insensitive) — and it selects that placed joint block instance, then `ZoomSelected` frames it. Handy for finding one joint in a crowded model.
- The role suffix is optional: `J40-53` selects **both** halves of the pair at once, and bare pair numbers (`40-53`) get the `J` prefix added automatically.
- Canonical PyBullet body keys like `joint_J25-26_male` work too — paste them straight from a collision log and the `joint_` prefix is stripped.
- Comma-separate ids (`J40-53_female,J12-7`) to select several at once. The prompt loops so you can jump from joint to joint; each entry **replaces** the selection. Enter on an empty prompt (or Esc) ends the command.
- Matches blocks by the object-name convention `{joint_id}_{female|male|ground}` written at placement time, with a fallback to the `joint_id` user text + instance layer for blocks that lost their name. Tool blocks are never selected (they carry `joint_id` user text too, but live on the tool layer).
- **Read-only** — never edits the document, so it is safe to run any time. No PyBullet needed. On the RSDesign toolbar.

### RSDefineJointHalf (`rs_define_joint_half.py`)

- Defines one half: `Male`, `Female`, or `Ground`.
- Exports `asset/<block_name>.3dm` and collision `asset/<block_name>.obj`.
- Upserts half data in `scripts/core/joint_pairs.json`.
- **Ground picks two direction points, not a bar axis line.** A ground block's
  own orientation is not free — RSGroundPlace's auto-`jr` rotates it about the
  bar until its local +Y points at the floor — so the frame the arm approaches
  on has to be chosen separately. It is picked the same way RSDefineRoboticTool
  picks a TCP frame, except the **origin is not picked**: it is the ground
  block's own insertion point, and both points are read as directions from it.
  - **+X direction point** — where the tool's +X should point. With the origin
    it also *is* the bar axis, feeding the unchanged `M_block_from_bar` math.
  - **+Y direction point** — direction only, and only to fix the roll. The frame
    is re-orthonormalized from X, so this one can be picked loosely.
  - A **ghost tool** (magenta) then appears at the resulting frame; `Accept`
    keeps it, `Repick` loops back to the +X pick. The ghost is the active pair's
    default tool; with no active pair the preview is skipped, not blocked.
  - The rotation between the picked frame and the block frame is saved as
    `M_tool_from_block` and applied by `core.rhino_tool_place.tool_attach_frame`
    at tool-placement time. Run RSUpdatePreview afterwards to re-snap tools that
    are already placed.
  - Anchoring the bar axis at the block origin gives `M_block_from_bar` a **zero
    translation**, so RSGroundPlace lands the block origin exactly on the point
    you click on the bar. Ground joints defined with the older bar-axis-*line*
    pick carry whatever offset that line's start point happened to have
    (`T20Ground`: 25 mm), so re-defining one shifts where a given `jp` puts it.
  - The +X *direction* also sets the sense of the bar axis, so re-defining can
    flip which way new placements face along the bar — RSGroundPlace's `Flip`
    covers that. Already-baked instances never move either way.

### RSDefineJointMate (`rs_define_joint_mate.py`)

- Selects female and male halves, computes `contact_distance_mm`, and saves the named mate.
- Requires both halves to exist first.

### RSMeasureGap (`rs_measure_gap.py`)

- Computes shortest segment between two finite line objects.
- Draws the result and stores measured distance in user text.

### RSUpdatePreview (`rs_update_preview.py`)

**Left-click** repairs the model, paints the diagnostic overlay, and pops up a
tally. **Right-click** (RSClearColorPreview) removes the overlay again. There is
no longer a job prompt: seeing what a repair pass could *not* fix is part of
running it, so the old `ShowColorsPreview` option is now always-on.

The repair pass is idempotent: running it twice in a row reports the same and
changes nothing. In order:

1. Rebuilds missing or stale tube previews for all registered bars.
3. **Reloads joint block definitions whose `asset/*.3dm` changed** since they were
   imported (`core.rhino_joint_refresh.refresh_stale_joint_blocks`). This is the
   joint-side counterpart of what RSSwapRoboticTool does for tools: every joint
   placement path uses `require_block_definition`, which skips the import when a
   definition of that name already exists, so an edited block would otherwise never
   reach an open document. Change detection is an *asset stamp* (the source file's
   mtime + size) recorded in the block definition's description by
   `core.rhino_block_import`. The reload is
   `update_block_definition_geometry`, which swaps the geometry **in place** via
   RhinoCommon's `InstanceDefinitions.ModifyGeometry` — nothing is deleted or
   re-created, so every placed instance keeps its id, world transform, object name
   and user text. A block with no stamp (document saved before stamping existed) is
   reloaded once. **After editing a joint half with RSDefineJointHalf, click
   RSUpdatePreview to pull the new geometry in.**
4. Restores robotic tools that went missing entirely, then **sets each bar's L/R tool
   layout from the robot's approach** (`core.rhino_tool_place.enforce_bar_tool_sides`).
   A bar is meant to carry one `*L` and one `*R`, and WHICH end holds which follows
   from where the mobile base stands: the robot faces the bar, so the end on its left
   (`ground_normal × heading`) takes the left tool. For every bar with an assigned
   WalkableGround the pass resolves that heading
   (`core.rhino_walkable_ground.resolve_bar_heading`) and derives both sides from it,
   so the whole document converges on one rule — the same one RSIKKeyframeAll applies
   when it places a base, including after a Flip. Bars with no walkable ground have no
   approach to derive from, and fall back to the older narrow repair: only a bar
   holding two SAME-side tools is rewritten, near joint keeps what it has, far one
   flips. **A side you cycled by hand in RSJointEdit is never overridden** — that
   stamps `config.KEY_TOOL_SIDE_MANUAL` on the joint block and this pass skips it.
   Finally, any tool that drifted off its joint is re-snapped —
   "on its joint" meaning its TCP sits on the joint's **tool-attach frame**
   (`core.rhino_tool_place.tool_attach_frame`), not necessarily on the raw block
   frame: a ground joint whose definition carries an `M_tool_from_block` is
   supposed to hold its tool rolled, and is left alone.
5. **Reports pairs whose halves no longer mate** (`report_unmated_joints`), judged
   with the creation side's own definition of a good interface: the female and male
   screw frames must coincide within `VARIANT_OK_ORIGIN_TOL_MM` /
   `VARIANT_OK_Z_AXIS_TOL_RAD`, measured by
   `core.joint_pair_solver.screw_alignment_diagnostics` — exactly the test
   `is_variant_acceptable` applies when the solver picks a variant. Measured from
   the blocks' **actual** transforms, so it says where the joints really are.
6. **Counts what cannot be repaired automatically** (`find_broken_links`): a joint
   that lost its bar **or its other half** (a female with no male is as broken as
   one with no bar — there is nothing for it to mate with; ground joints are
   single-sided by design and exempt), a tool that lost its joint or is no longer
   on it, and a registered bar carrying no joint. Detached tools come from
   `core.rhino_tool_place.find_detached_tools`, which shares `is_tool_on_joint`
   with the re-snap pass — so "attached" means one thing everywhere. They are
   painted, selected and listed in the popup tally at the end of every run. Fix
   them with RSJointPlace / RSGroundPlace, or delete the orphan.

### RSClearColorPreview (`rs_clear_color_preview.py`)

Right-click companion of RSUpdatePreview: removes every diagnostic overlay and
unselects. Bar colors (IK status and the bare-bar color) revert to by-layer on both
the centre-line and its tube, joint/tool instances revert, and the marker dots are
deleted. Appearance only — no geometry or metadata is touched.

**Fake bars keep their pink tint.** Which bars are staging is a property of the
model, not a diagnostic, so clearing the overlay must not clear it — otherwise
the one marker you want on screen permanently is the one that vanishes every
time you tidy up.

**How the marking works**, since no single channel covers everything:

- **bars** go through `paint_bar`, which colors the centre-line *and* its tube
  preview — coloring the curve alone is invisible because the tube covers it.
  `BARE_BAR_COLOR` is a muted indigo on purpose: a green would be hard to tell
  apart from the green R-side robotic tools, and a bright purple would compete
  with the orange orphan color for attention;
- **joint blocks** take an ordinary `ObjectColor` override — color only, no dot;
- **tools cannot be recolored at all** — a block instance's color only reaches
  sub-objects whose color source is *by parent*, and the tool assets carry baked
  colors (that is what makes them read as red/green). So each flagged tool gets a
  colored **text dot** labelled with its tool id (`T<joint_id>`) on the
  `Diagnostic Marks` layer. That layer is deliberately **not** in `MANAGED_LAYERS`
  — the managed-layer enforcer would evict the markers as strays;
- everything flagged is left **selected**, ready to inspect or delete. A bare bar
  contributes **both** its centre-line curve and its tube preview
  (`_bar_curve_and_tube`), so pressing Delete removes the whole bar instead of
  leaving an orphaned tube behind. Nothing is ever deleted for you — a bare bar
  may simply be one you have not jointed yet.

**This command never moves a joint.** Re-deriving a solved placement from the
stored `(jp, jr)` is not a reliable enough authority to act on silently — earlier
attempts moved joints that were correct — so joints are only ever reported. One
consequence: editing `M_block_from_bar` in `joint_pairs.json` affects only NEWLY
placed joints; re-place an existing one with RSJointEdit / RSGroundPlace to adopt a
changed transform.

### RSReorderBarID (`rs_reorder_bar_id.py`)

- Renumbers bars so `B<n>` matches sequence `n`.
- Cascades updated IDs into dependent joint/ground/tool metadata.

### RSExportPrefab (`rs_export_prefab.py`)

- Exports prefabrication JSON from current bar/joint state.

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
- **A rejected pick points at the joints, not the bar.** The bar must carry exactly two tool-bearing joints (male or ground) and be in the bar registry. If it does not, the offending **joint blocks** are painted `COLOR_FAILED` pink and named on the command line — you already know which bar you just clicked; what you cannot see is which of its joint blocks the count is complaining about. A bar with no tool-bearing joint at all is marked itself, there being nothing else to point at. The count is of *blocks*, so a duplicated block instance makes a visibly-two-joint bar report three; that case is called out by name, since it is otherwise baffling. The mark clears on the next pick and on exit, and restores whatever colour the block carried before — it never eats a sequence colour or a broken-link mark.
- **Base guide lines.** Right after the bar pick, five lines are drawn on the bar's walkable ground and stay up through the whole base pick: the line joining the two grabbed-joint centres **projected onto the ground** (teal), three copies offset *against* the assembly direction by 375 / 500 / 625 mm (`config.BASE_GUIDE_OFFSETS_MM`, light teal, each labelled with a text dot), and the **extension line** through their midpoints (yellow) — the line an auto-placed base sits on. So you can read a standoff straight off the canvas before committing. See `docs/coordinate_conventions.md` §7.
- **Arm reach ghost.** The translucent robot that tracks the cursor now carries two translucent spheres of `config.ARM_REACH_RADIUS_MM` (850 mm, the UR5e's nominal reach) centred on each arm's mount link, so you can see what the arms can actually touch from a candidate base. Approximate — the real reachability test is still the IK solve. Display-conduit only; nothing is baked.
- **Tool sides follow the approach.** Which end of the bar carries the LEFT tool depends on which side the robot stands (`ground_normal × heading` = the robot's left). The command corrects the two tools from the resolved heading *before* building the movements, and warns if you then hand-pick a base facing the other way. A side you cycled by hand in RSJointEdit is never overridden.
- **Flip the base to the other side of the bar.** When the bar carries a saved base, the reuse prompt offers **Reuse / NewPick / Flip**. Flip does not leave the prompt: it re-derives the base on the opposite side, moves the ghost and the guide lines there, and asks again — so you can look at both sides before committing, and a second Flip puts you back. The base is re-derived from the bar each time rather than mirrored from the previous answer, so repeated flips cannot drift, and the origin is re-snapped to the ground (the two sides of a bar are rarely at the same height). The L/R tool sides follow, because the heading is re-read from the flipped frame — but unlike RSIKKeyframeAll's FlipOne, this does **not** clear a hand-picked tool side. If the other side has no assigned, meshable WalkableGround, the base stays where it was and the reason is printed. This is the single-bar equivalent of FlipOne below.
- **Save base / repick / continue off-ramp:** right after you pick the base origin + heading (or reuse a saved base), it asks **Continue** vs **Repick** vs **SaveBaseAndExit**. Enter/Continue runs the in-Rhino solve as before; SaveBaseAndExit keeps the just-saved base frame and stops, so you can solve the keyframes headlessly with `external/husky_assembly_tamp/scripts/headless_bar_action_planner.py --solve-keyframes --base saved` (export the bar first). **Repick** goes back to the base prompt with the base you just placed as the candidate — the base is the one thing you cannot re-judge until you see the robot standing at it, and it is also how you reach **Flip** for a bar that had no saved base to begin with. Not shown on a RetrySameBase loop.
- The guide lines are transient: cleared on **every** exit path, including ESC at any prompt.

### RSIKKeyframeAll (`rs_ik_keyframe_all.py`) — right-click on the RSIKKeyframe button

- Multi-select bars; places a mobile base on each and optionally solves them all in one batch.
- **Standoff** is typed at the prompt, defaulting to `config.IK_BASE_STANDOFF_MULTIBAR_MM` = **500 mm** — deliberately the *middle* base-guide offset, so the auto-placed base lands exactly on the middle guide line and 375 / 625 read as the two alternatives either side of it.
- Each processed bar gets its own guide-line set plus a base marker and reach circle.
- **FlipAll / FlipOne** at the off-ramp prompt move bases to the other side of their bar (FlipOne asks you to click a base marker). The guides, the saved base frame, and the L/R tool sides all follow the flip; a flip also clears any hand-picked tool side, since it is you re-deciding the approach.
- Bars whose base side could not be derived with confidence — anchor axes cancelling, or the ground running out before the standoff — are flagged **"Ambiguous heading — verify the side"** in the summary popup, with the per-bar diagnosis printed to the command line.
- All preview geometry (guides, base markers, reach circles) is removed on **every** exit path: solved, ESC at any prompt, SaveAndExit, or an exception. The saved base frames on the bars are untouched.

### RSShowBarActionPlan (`rs_show_bar_action_plan.py`) / …Motion (`rs_show_bar_action_plan_motion.py`)

- **Left-click** — the IK KEYFRAME viewer. Pick a bar; Enter cycles that bar's timeline — approach → assembled → *hold* (held bars only) → retreat → home, plus a *release* pose per hold whose LAST stabilizing bar is this one — read from the `KEY_ASSEMBLY_*` / `KEY_SUPPORT_*` user-text; draws the active bar's **base frame** (axis triad + footprint). Support robots appear per pose exactly as the hold schedule says (earlier holders frozen throughout; this bar's own holder from the hold pose on). When `RSLoadSolvedBarAction` has populated the loaded-solved cache, it auto-starts on those bar(s), draws every base frame, and offers **NextBar/PrevBar** (the all-bars map).
- **Right-click** — the MOTION viewer. Pick a bar; if its trajectory isn't cached it loads `<bar>.solved_motion.json` from the export root and syncs the condensed IK to user-text, then **steps through the concatenated trajectory from the command line** (Enter/Next/Prev/Jump — a prompt, not a modal dialog, so the viewport stays free to zoom/orbit). FK-only, so the held bar follows the arm smoothly. Requires PyBullet (RSPBStart).
- **Both clicks** also temporarily color the active bar's assigned **WalkableGround** brep(s) green and print each ground's id + Rhino object id to the command line, so you can see which ground surface the robot base is allowed to stand on. The color is reverted to ByLayer when you switch bars or exit.

### RSShowAssemblyPlan (`rs_show_assembly_plan.py`)

- The whole-assembly twin of RSShowBarActionPlan: **every bar's poses concatenated in assembly-sequence order**, so Enter walks the build end to end in the order `ActionSchedule.json` records (`B20 approach → … → B20 home → B21 approach → … → release B21 → …`).
- Each step is one movement ENDPOINT (the poses that carry a solved keyframe). The tool/manual movements between them — gripper open/close, screw driving, the operator mounting the bar — are printed rather than drawn, since they are instants at a pose already on screen.
- Bars that are **fake** or have **no solved IK keyframe** contribute no steps and are listed up front; holds with no solved support keyframe are named too, so a short timeline is never a silent one.
- Command line: **Enter/Next**, **Prev**, **Jump** (step index), **GoToBar** (bar id), click a bar to jump there, **MeshMode**, **CheckCollision**, **ShowUnbuilt/HideUnbuilt**, Esc to exit. Requires PyBullet (RSPBStart).

### RSLoadSolvedBarAction (`rs_load_solved_bar_action.py`) / …All (`rs_load_solved_bar_action_all.py`)

- **Loader only** — the visualization is RSShowBarActionPlan's job. Prompts `solved_keyframe` (base + IK keyframe configs) vs `solved_motion` (planned trajectories), and finds files under the shared export root's `BarActions/`.
- **Left-click**: pick a bar → load `<bar_id>.solved_<kind>.json`. **Right-click**: load every `<bar>.solved_<kind>.json` in the folder.
- For each loaded action it: caches the full `BarAssemblyAction` (`core.solved_action_cache`, for the motion view) and **syncs the condensed IK** (base + approach/assembled/retreat) back onto the bar's user-text via `bar_action.write_bar_keyframe_from_action` — the reverse of the export, so the on-curve record + a later re-export stay consistent. Then it launches RSShowBarActionPlan.

### RSIKSupportKeyframe (deleted)

- The support-robot IK is now the SUPPORT flow of the single RSIKKeyframe
  button: picking a bar that needs holding (non-empty `supported_until`) and
  already has its assembly keyframe runs the support flow instead.
- The old script's reusable pickers (two-phase grasp pick with the ghost
  gripper, base pick with the ghost support robot, dual-arm ghost bake)
  moved to `core/support_grasp_pick.py`; keyframe IO + release validation
  live in `core/hold_action_builder.py`; the hold derivation (which robot
  holds which bar until when) is `core/hold_schedule.py`.
- Results are written to the split `KEY_SUPPORT_*` user-text keys on the
  held bar (robot name, base frame, grasp frame, approach + held configs) —
  the legacy `ik_support` blob is no longer written.

### RSExportBarAction (`rs_export_bar_action.py`)

- Left-click: exports one picked bar's movement/action schema for planner/monitor integration.
- Right-click on the same button runs the batch export below (`rs_export_all_bar_actions.py`) — there is no separate "RSExportAllBarActions" button.

### RSExportAllBarActions (`rs_export_all_bar_actions.py`)

- Batch export of ALL bars (not just IK-solved ones): writes `<root>/BarActions/<bar_id>.json`, `<root>/RobotCell.json`, and `<root>/WalkableGround.json`.
- Each BarAction carries the bar's `walkable_ground_ids` (set by RSRebuildRobotCell auto-assign or RSAssignAndShowWalkableGround). `WalkableGround.json` holds every WalkableGround brep meshed and keyed by its stable id.
- Bars exported without IK get a placeholder base; the headless keyframe solver (`external/husky_assembly_tamp/scripts/headless_bar_action_planner.py --solve-keyframes`) samples their base + IK afterward.

### RSAssignAndShowWalkableGround (`rs_assign_and_show_walkable_ground.py`)

- **Single left-click**, a view↔re-assign loop:
  1. **View.** Pick a bar (first run). Its assigned WalkableGround(s) are highlighted green; the default mobile-base placement is computed — the same heuristic the headless solver uses in `--base sample` mode (`husky_assembly_tamp.keyframe.walkable_ground`: `derive_seed_base` + `frame_from_origin_normal_heading`): the base stands a standoff **behind** the bar and **faces along the average anchor-joint insertion direction** (the way the bar is pushed to mate), which removes the left/right side ambiguity. That average now covers **both** male and ground joints, discards near-vertical axes, and detects the case where a bar's two anchors cancel, falling back to the open side of the bar instead of to a noise direction (`core.rhino_walkable_ground.resolve_bar_heading`; see `docs/coordinate_conventions.md` §7). It's drawn as a base-frame marker + a **half-transparent ghost robot** standing on that base. The ground ids + base position are printed. Then it asks: *change the assignment?*
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

### RSImportScaffoldJSON (`rs_import_scaffold_json.py`)

- Reads a `node_list` / `rod_list` / `coupler_list` layout JSON (e.g. `large_scaffold.json`, produced by the upstream stability/layout generator) and bakes **one registered bar per rod** — same registration RSCreateBar does (`bar_type` / `bar_id` / `bar_guid` / `bar_seq` user text, object name, centerline layer, tube preview).
- `bar_id` is set to **`B<rod_id>`** so the Rhino id reads the same as the rod id. The rod id is *also* written to the `rod_id` user-text key, which is what the export reads — so RSReorderBarID renumbering `bar_id`, or a copy/paste making `ensure_bar_id` re-issue one, cannot break the correspondence.
- Assembly sequence (`bar_seq` 1..N) follows `layer_id` then `rod_id`, i.e. bottom-up. This is what decides joint roles later: RSJointPlace gives the earlier bar of a pair the female half.
- **Nothing in the source file is dropped.** Per-rod fields (`layer_id`, `grounded`, `end_node_ids`, the as-imported endpoint positions, and any field this schema doesn't model) go on the bar curve as user text; `coupler_list`, node-level extras and top-level extras go into **document** user text. Each bar also gets a `rod_coupled_rod_ids` list denormalised from `coupler_list`, as a design-time convenience.
- Warns if the document is not in millimetres (the JSON and every joint constant are mm).
- If the document already contains imported bars, it offers to delete them **with** their joints and robotic tools before re-importing, or to cancel and leave the document untouched.
- Prints a summary: bars created, bars per `layer_id`, grounded count, and any rod skipped as degenerate.

### RSExportScaffoldJSON (`rs_export_scaffold_json.py`)

- The inverse of RSImportScaffoldJSON: rebuilds `node_list` / `rod_list` / `coupler_list` from the **current** bar geometry and writes the same schema. With nothing moved, the output is identical to the input file.
- **Nodes follow the geometry.** In the source file rods meet exactly at shared nodes; a buildable model can't, because each joint holds its two bars apart by the pair's contact distance. So a source node whose incident bar ends have been pulled apart is split into one node per distinct position — the original `node_id` stays with the lowest-numbered rod, the others get fresh ids past the top of the original range. Rod-to-rod connectivity is unaffected: it lives in `coupler_list`, not in shared node ids.
- A curve reversed in Rhino still gets its `end_node_ids` on the right physical ends — the as-imported endpoint positions are used to detect the flip.
- `coupler_list` is the imported list **merged with** any additional pairs discovered from joint blocks placed since (`parent_bar_id` / `connected_bar_id` on the female/male instances), so joints added in Rhino appear as new couplers. Ground joints are skipped — they anchor a bar to the ground, not to another rod. Couplers whose rods no longer exist are dropped, and both changes are printed.
- Bars drawn in Rhino without imported rod data are exported too, with freshly minted rod ids, `layer_id` 0 and `grounded` false; they are listed in the summary so the defaults are visible.
- Prints a pre-export sanity report: a histogram of the actual centerline gap for every coupled pair (pairs still at < 1 mm have no joint fitted yet), and any *uncoupled* pair closer than two bar radii, whose tubes therefore overlap.
- Shared round-trip logic lives in `scripts/core/scaffold_json.py` (pure Python, covered by `tests/test_scaffold_json.py`).

### RSSetRodLayer (`rs_set_rod_layer.py`)

- Select bars — centerlines or tube previews, either works — and give them a `layer_id`.
- `layer_id 0` also sets `grounded` **true**; any other value sets it **false**. The two fields are not independent, so there is one prompt, not two.
- Both values are written as user-text and go straight into `rod_list` on the next RSExportScaffoldJSON. Bars that were never given a layer export as `layer_id` 0 / `grounded` false — see the RSExportScaffoldJSON notes above.

### RSTempPlace (`rs_temp_place.py`)

> **Temporary, mock-up only.** A one-click way to give the robot cell and bar actions something to run simple IK tests against — not a design tool.

- Two modes, neither of them the Enter default (you pick):
  - **PlaceFemaleJointBar** — for every male/ground joint with no female yet, place the mating female half plus a **200 mm stub bar** centred under it.
  - **PlaceBar** — the same stub bar in the same place, without the female block.
- The bar is centred on the **female block**, not on the bar-frame origin those two are ~20–30 mm apart along the bar (`M_block_from_bar`), so centring on the origin would put the bar's midpoint on the joint's face. 200 mm rather than 100 so the joint stays well inside the bar after a flip, which moves the mate ~60 mm along it (see `docs/Su_note.md` §27).
- Ground joints carry no mate frame in `joint_pairs.json`, so you are asked which pair's male plug they should be treated as mating like.
- **Stub bars are marked fake automatically.** They sequence normally and the IK still sees their joints, but the prefab, bar-action and robot-cell exports skip them. Delete them when the mock-up is done, or clear the mark with RSBarEdit > FakeBar > Delete.
- **The female is not optional for IK.** At M2 the male seats into it, and `core.bar_action` whitelists that contact by looking up `joint_<jid>_female`; with no female present the mate reads as a real collision. The stub bar is what keeps the collision scene honest about what will physically be there.
- **Check stubs near subfloor joints by eye.** A female is placed from the mate frame alone, which says where the half must sit — not whether there is room for a bar behind it. Where a stub runs through another bar, delete it and copy the real bar from the design instead.

### RSRegisterAliases (`register_command_aliases.py`)

Makes every toolbar command **typeable at the Rhino command line**, which is usually faster than finding its button.

- Rhino macros normally only run when their button is clicked. This walks every `<macro_item>` in `scaffolding_toolbar.rui` and registers a command *alias* whose name is the macro's `<text>` (e.g. `RSJointEdit`) and whose expansion is that macro's `<script>` line.
- Aliases live in Rhino's **per-user settings**, not in the `.rui`, so this is a per-machine step. They persist across Rhino sessions.
- Run it **once per machine**, and again whenever the macros change — it is idempotent, replacing an alias of the same name.
- After running it, type the first few letters (`RSJoint…`) and let Rhino autocomplete.
- Because the alias name comes from the macro's `<text>`, **renaming a macro's `<text>` renames its alias**. Anything decorative added there (a suffix marking a right-click companion, for instance) becomes part of the typed name and breaks it.

## Manual macro pattern

If a toolbar button is missing, run the script directly in Rhino ScriptEditor:

```text
! _-ScriptEditor _R "rs_<name>.py"
```

Rhino resolves filenames through Search Paths, so `<repo>/scripts` must be added.

Every macro is also registered as a typeable command alias by **RSRegisterAliases** (above); if
typing a command name does nothing, that script has not been run on this machine yet.

## Toolbar tab icons

The little picture on each toolbar *tab* (RSDesign, RSSetup, RSMoCap,
RSStability) is not a file reference — Rhino stores icons **inside** the
`.rui` as three base64 PNG sprite sheets (16 / 24 / 32 px cells). A
`<tool_bar bitmap_id="…">` attribute points at a cell; without it the tab
renders blank.

Source icons live in [`icon/`](../icon/) as `icon_<Toolbar>.png` (e.g.
`icon/icon_RSDesign.png`), one 48×48 RGBA PNG per toolbar, matched to the
toolbar by name. To (re)pack them into the `.rui`:

```bash
python scripts/apply_toolbar_icons.py            # apply
python scripts/apply_toolbar_icons.py --check    # report only, write nothing
```

The script downsamples each icon to the three sizes (source PNGs may be any
square 8-bit RGBA size — the current set is 355×355), rebuilds the
`<bitmaps>` block, and sets `bitmap_id` on each matching toolbar; it is
idempotent (guids are derived from the toolbar name) and touches nothing
else in the file. To add an icon for a new toolbar, drop
`icon/icon_<Name>.png` and re-run it.

Because the icons live *inside* the `.rui`, a user only has to **open**
`scaffolding_toolbar.rui` in Rhino to get the tabs — there is no per-user
icon step and no Rhino "on open" hook to install; the committed `.rui`
already carries the sheets.

### Keep it packed automatically (pre-commit hook)

So no one has to remember to re-run the packer, a tracked git hook at
[`.githooks/pre-commit`](../.githooks/pre-commit) re-packs the icons and
re-stages the `.rui` whenever a commit touches `icon/*.png`,
`scaffolding_toolbar.rui`, or the packer itself. Then the workflow is just:
edit the PNG, commit. Enable it once per clone (it finds the `py` launcher
on Windows automatically, and is a no-op for unrelated commits):

```bash
git config core.hooksPath .githooks
```

It also self-heals the "Rhino stripped the `<bitmaps>` block on save" case
below: the next commit repacks the sheets before they can be lost.

**Rhino overwrites `.rui` files it has loaded.** Whenever Rhino saves the
UI layout it re-serialises the whole file from memory and drops the
`<bitmaps>` block, blanking the tabs again. If that happens, close Rhino
(or unload the toolbar) and re-run the script.

## Maintenance rule

When adding, renaming, or removing a Rhino entrypoint script:

1. Update `scaffolding_toolbar.rui`.
2. Update this file.
3. Keep install and quickstart wording in `README.md` aligned.
4. If you add or rename a *toolbar* (not just a button), add a matching
   `icon/icon_<Toolbar>.png`. With the `.githooks` pre-commit hook enabled
   (`git config core.hooksPath .githooks`) the icon is packed into the `.rui`
   on your next commit automatically; otherwise re-run
   `python scripts/apply_toolbar_icons.py` by hand.
