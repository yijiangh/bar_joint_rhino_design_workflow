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
| RSDesign | RSIKKeyframe | `rs_ik_keyframe.py` | Dual-arm IK keyframe solve and save on Ln bar | Advanced IK users |
| RSDesign | RSShowIK | `rs_show_ik.py` | Replay saved `ik_assembly` keyframe (final/approach) | Advanced IK users |
| RSDesign | RSIKSupportKeyframe | `rs_ik_support_keyframe.py` | Single-arm support IK keyframe workflow | Advanced IK users |
| RSSetup | RSDefineJointHalf | `rs_define_joint_half.py` | Define one joint half (Male/Female/Ground) and collision mesh | Joint-library authors |
| RSSetup | RSDefineJointMate | `rs_define_joint_mate.py` | Define mate between existing joint halves | Joint-library authors |
| RSSetup | RSMeasureGap | `rs_measure_gap.py` | Measure closest segment between two finite lines | Workshop participants |
| RSSetup | RSUpdatePreview | `rs_update_preview.py` | Rebuild stale/missing bar tube previews | Workshop participants |
| RSSetup | RSReorderBarID | `rs_reorder_bar_id.py` | Renumber bars to match sequence and cascade IDs | Workshop participants |
| RSSetup | RSExportPrefab | `rs_export_prefab.py` | Export bar/joint prefabrication JSON | Workshop participants |
| RSSetup | RSExportCase | `rs_export_case.py` | Export T1-S2 debug case JSON for solver replay | Developers/debugging |
| RSSetup | RSExportConfig | `rs_export_config.py` | Export CAD-derived connector config and frame snapshot | Joint-library authors |
| RSSetup | RSBakeFrame | `rs_bake_frame.py` | Bake right-handed frame group from picked points | Joint-library authors |
| RSSetup | RSExportGraspTool0TF | `rs_export_grasp_tool0_tf.py` | Export male-joint/bar-grasp to tool0 transforms | Advanced IK users |
| RSSetup | RSPBStart | `rs_pb_start.py` | Start shared PyBullet client and planner | Advanced IK users |
| RSSetup | RSPBStop | `rs_pb_stop.py` | Disconnect shared PyBullet client | Advanced IK users |
| RSSetup | RSExportBarAction | `rs_export_bar_action.py` | Export one bar assembly action JSON | Planning/export users |
| RSSetup | RSExportAllBarActions | `rs_export_all_bar_actions.py` | Export all bar assembly action JSONs | Planning/export users |
| RSSetup | RSExportRobotCell | `rs_export_robotcell.py` | Export robot cell configuration JSON | Planning/export users |
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

### RSExportGraspTool0TF (`rs_export_grasp_tool0_tf.py`)

- `Joint` mode: writes male-joint OCF -> tool0 transform.
- `Gripper` mode: writes bar-grasp -> tool0 transform.
- Persists in `scripts/core/config_generated_ik.py`.

### RSPBStart (`rs_pb_start.py`) and RSPBStop (`rs_pb_stop.py`)

- Start/stop shared PyBullet planner state used by IK workflows.
- IK scripts reuse the same client between runs.

### RSIKKeyframe (`rs_ik_keyframe.py`)

- Dual-arm IK for two selected male joints sharing one Ln bar.
- Supports collision options and base-sampling fallback when direct solve fails.
- Saves `ik_assembly` JSON on the shared Ln bar.

### RSShowIK (`rs_show_ik.py`)

- Reads saved `ik_assembly` from a picked bar and replays `final` or `approach`.

### RSIKSupportKeyframe (`rs_ik_support_keyframe.py`)

- Solves support-arm IK keyframe for holding/assisting a just-assembled bar.
- Saves support keyframe data for downstream replay/export.

### RSExportBarAction (`rs_export_bar_action.py`)

- Exports one bar's movement/action schema for planner/monitor integration.

### RSExportAllBarActions (`rs_export_all_bar_actions.py`)

- Batch export of all bar action files.

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
