# Rhino Toolbar Entrypoints

This document is the canonical map of Rhino toolbar buttons to script entrypoints.

## Toolbar Source

- Toolbar file: `scaffolding_toolbar.rui`
- Script folder expected in Rhino Search Paths: `scripts/`

## RSDesign Buttons

| Button | Script | Purpose |
|---|---|---|
| RSCreateBar | `rs_create_bar.py` | Register selected curves as bars and create tube previews |
| RSBarSnap | `rs_bar_snap.py` | Snap a new bar to a target contact distance from an existing bar |
| RSBarBrace | `rs_bar_brace.py` | Solve and pick brace-bar candidates between two bars |
| RSSequenceEdit | `rs_sequence_edit.py` | Interactive assembly sequence viewer and editor |
| RSJointPlace | `rs_joint_place.py` | Place female and male connector blocks on a selected bar pair |
| RSJointEdit | `rs_joint_edit.py` | Re-edit the orientation of a previously placed joint pair by clicking a block |
| RSIKKeyframe | `rs_ik_keyframe.py` | Dual-arm IK keyframe workflow: pick two male joints, solve IK, save on shared Ln bar |
| RSShowIK | `rs_show_ik.py` | Replay a saved `ik_assembly` keyframe (final or approach) on a picked bar |

## RSSetup Buttons

| Button | Script | Purpose |
|---|---|---|
| RSBakeFrame | `rs_bake_frame.py` | Bake named CAD reference frames from picked points |
| RSExportConfig | `rs_export_config.py` | Export CAD-backed config values to generated core config files |
| RSMeasureGap | `rs_measure_gap.py` | Measure closest segment between two finite bar lines |
| RSExportCase | `rs_export_case.py` | Export a reproducible S2 case JSON from Rhino picks |
| RSUpdatePreview | `rs_update_preview.py` | Rebuild stale or missing bar tube previews |
| RSExportPrefab | `rs_export_prefab.py` | Export bar/joint prefabrication data JSON |
| RSExportJointTool0TF | `rs_export_joint_tool0_tf.py` | Export male-joint OCF → tool0 transform for an IK-capable joint type |
| RSPBStart | `rs_pb_start.py` | Start the shared PyBullet client (GUI or Direct); load robot cell into the planner |
| RSPBStop | `rs_pb_stop.py` | Disconnect the shared PyBullet client |
| RSExportPineappleOBJ | `rs_export_pineapple_obj.py` | Export pineapple block definitions to OBJ (meters) for IK collision tool models |

## Maintenance Rule

When adding, renaming, or removing a Rhino entrypoint script:

1. Update `scaffolding_toolbar.rui`.
2. Update this file.
3. Update the toolbar section in `README.md`.
