# Note for Su — the MoCap marker export, version 2 (2026-10-16)

Su, I merged `hs/mocap-experiment` into `main` today (merge commit `3473583`) and then reworked the
MoCap marker export on top of it. Your `T20_MoCap` joint, the registry marker points, the JointOnly
placement, the naming rule and the Rhino label layer all stay. What changed is **what the file
says and when the export refuses to write it**. This note explains the new scheme and why, so you
can pick it up without reading the diff first.

## 1. What the monitor needs from us

The goal on the robot side is to read a bar's **full 6-DoF pose** from Motive (position and
rotation), for two uses: aim the robot at a grounded bar where it really is, and record how bars
move or bend after each assembly / hold action.

Two facts about Motive shape the design:

- Motive assigns each marker of a rigid body an id, but the ids **shift between sessions and
  sometimes within one**, so the monitor cannot trust "marker 3 is M1 of J20-21".
- The markers on a `T20_MoCap` plate are not symmetric about the bar axis (48.3 mm pair, 25.5 mm
  off the axis), so the old `bar_rig` trick (pair up markers, take midpoints, fit a line) is wrong
  for them — it would put the bar 25.5 mm off and gives no roll anyway.

So the monitor will work **from geometry alone**: take the designed marker positions of a bar,
find which measured point is which designed marker by trying 3-point correspondences and keeping
the one where *all* markers line up, and fit the rigid motion (Kabsch). That motion *is* the
measured bar pose. For that to work the file must give the designed markers in a frame the monitor
already knows, and the design must make the answer unique.

## 2. What the file says now (schema 2)

Your schema 1 wrote each marker in the Rhino world (`world_mm`) and in the bar frame (`bar_mm`),
plus each bar's centre line and each joint's 4×4 world transform. All of that is already in
`RobotCell.json` (bodies) and the BarAction states (poses), and the world position only describes
the *final assembled* state, which is not what the monitor compares against while a bar is being
carried. Schema 2 keeps only what nothing else has:

```json
{
  "schema_version": 2, "units": "m", "cell_file": "RobotCell.json",
  "marker_blocks": {"T20_MoCap": {"marker_diameter_m": 0.019,
                                  "points_block_m": {"M1": [-0.01, 0.02415, 0.0255], "M2": [-0.01, -0.02415, 0.0255]}}},
  "bars": {
    "bar_B20": {"bar_id": "B20", "assembly_seq": 1, "layout": {"n_markers": 8, "nearest_miss_m": 0.0483},
                "markers": [{"name": "J20-22_mocap_M1", "joint": "joint_J20-22_mocap", "label": "M1",
                             "block": "T20_MoCap", "bar_m": [-0.0274, -0.022, 0.0849]}, "..."]}},
  "bars_without_markers": ["bar_B22"]
}
```

- **Bar body frame, metres.** `bar_m` is the marker centre in the frame the bar body has in
  `RobotCell.json` and in every BarAction `RigidBodyState.frame` (origin at the bar start, z along
  the bar — your `canonical_bar_frame_from_line`). The monitor fits these points onto the measured
  markers; the fit transform is directly comparable to the designed state frame. Metres because the
  cell and the BarActions are in metres.
- **Keyed by cell body names** (`bar_B20`, `joint_J20-22_mocap`), so the monitor needs no name
  parsing. Only bars that carry markers are listed; the rest go to `bars_without_markers`.
- **Marker diameter** per block (new registry field `marker_diameter_mm`, 19 mm on `T20_MoCap`,
  measured from the OBJ). The monitor uses it to drop Motive markers of another size.
- Dropped: `world_mm`, `start_mm/end_mm/length_mm`, `frame_world_mm`, `position_mm`,
  `marker_indices`, `index`, `warnings`. A world position is `bar_state_frame @ bar_m`.

Why still a separate file and not a field inside `RobotCell.json`: compas_fab's `RigidBody`
serialises exactly three keys and loads with `cls(**data)`, so any extra key makes the monitor's
`json_load` crash. The only free slot is the mesh `attributes` dict, which is a hack. A sidecar
read with plain `json` is the pattern we already use for `WalkableGround.json` and
`ActionSchedule.json`.

## 3. The file is built from the cell snapshot, before anything is written

- Marker positions come from the **cached cell snapshot** (the `bodies` of
  `ensure_assembly_cell`: bar body frame and joint block transform), not from live curves, so
  `MoCapMarkers.json` and `RobotCell.json` can never disagree. In a From..Until range export the
  sidecar is cut to the same bars as the cell.
- The payload is built **first**; only then are the cell and the sidecar written. If something is
  wrong the command shows "Export stopped, nothing was written: …" and the folder is untouched.
- Things that used to be warnings are now errors (our no-silent-fallback rule): a MoCap block with
  no recorded marker points or no diameter, a MoCap joint whose bar is not in the export.
- With no MoCap marker placed, a stale `MoCapMarkers.json` in the folder is **deleted**.
- The label layer is redrawn from the file's own numbers (`bar frame @ bar_m`), same dots and
  pair lines as before.

## 4. The layout rule — why the export now refuses some bars

If some *other* rigid motion maps a bar's marker set onto itself, Motive sees two poses that look
identical and the monitor cannot choose. The export refuses such a bar by name.

The precise rule (`scripts/core/mocap_layout_check.py`): a layout is ambiguous when a
**non-identity permutation** of its markers can be produced by a **proper** rigid transform
(rotation with det = +1 — mirrors do not count, a bar cannot be mirrored) with a max residual
≤ 2 mm (`config.MOCAP_LAYOUT_TOL_MM`). Fewer than 3 markers, or all markers on one line, is
refused too (the roll is free).

Two traps I ran into while designing this, worth knowing:

- **Pairwise-distance patterns are not enough.** On a long bar, M1 and M2 of one pair have almost
  identical distances to every other marker (those are a metre away) — on B20 they differ by
  0.35 mm. A rule based on distance patterns would refuse B20, which is actually fine. Distances
  only shrink the search; the decision is always the full-set residual of a proper fit.
- **Two exact ambiguities of the T20 plate.** *Two joints on a bar are never enough*, whatever
  their spacing or relative turn (a half-turn about the line bisecting the two plate normals maps
  the set onto itself end for end). *Joints that all face the same way are never enough* (a
  half-turn about the plate midline swaps M1 and M2 in every joint).

**Rule of thumb for designing: at least three MoCap joints per tracked bar, not all facing the
same way, not mirror-symmetric about the bar's middle.** Both bars of `261016_2ModulesDemo_YH`
(4 joints each, mixed facings) pass; the nearest wrong pose is 48 mm away. The refusal message
carries this hint.

## 5. What changed in the code (so you know where to look)

| File | Change |
|---|---|
| `scripts/core/mocap_layout_check.py` (new) | `fit_rigid`, `match`, `check_layout`, `require_unambiguous` — pure numpy, metres; also the reference for the monitor's matcher |
| `scripts/core/mocap_markers.py` | `build_payload(bodies, halves, bar_seq, …)` replaces `designed_markers`; `MoCapExportError`; `world_points_mm`, `layout_notes` |
| `scripts/core/rhino_mocap_markers.py` | `build_mocap_sidecar` / `write_mocap_sidecar` / `label_markers_in_rhino(payload, bodies)` replace `export_mocap_markers`; no live curves |
| `scripts/rs_export_robotcell.py`, `scripts/rs_export_all_bar_actions.py` | build first, write later; "Export stopped, nothing was written" on refusal |
| `scripts/core/joint_pair.py`, `joint_pairs.json` | `JointHalfDef.marker_diameter_mm` (T20_MoCap: 19.0) |
| `scripts/rs_define_joint_half.py` | `_pick_marker_points` also measures the diameter; refuses non-spheres / mixed sizes |
| `scripts/rs_export_prefab.py` | kept (mm, live doc, for the jig); gains `mocap.marker_diameter_mm` |
| `scripts/core/config.py` | `MOCAP_LAYOUT_TOL_MM = 2.0` |
| `tests/test_mocap_layout_check.py` (new), `tests/test_mocap_markers.py` (rewritten) | 34 tests pin the rule, the schema and the invariant `bar_frame @ bar_m == joint_frame @ block_point` |
| `docs/mocap_marker_export.md`, README MoCap section, `docs/coordinate_conventions.md` | rewritten for schema 2 |

Run the tests with Rhino's own Python (the default `python` has no compas):

```bash
PYTHONPATH=~/.rhinocode/py39-rh8/site-envs/scaffolding_env-d8rZiG+M ~/.rhinocode/py39-rh8/python.exe -m pytest -q
```

## 6. What I would like you to do

1. Re-export your designs with RSExportRobotCell (or RSExportAllBarActions) so they carry schema 2;
   nothing reads schema 1 any more. Check the dots still sit on the spheres.
2. When a bar is refused, add a third MoCap joint turned about the bar (RSJointPlace › JointOnly ›
   MoCap, Rotate) or move one, RSRebuildRobotCell, export again.
3. The old RSReadMoCapBar / RSAlignModelThreeBars are still for the 4-marker `bar_rig` only, as you
   documented; no change there.
4. The monitor side is written up for the Ubuntu session in
   `tasks/mocap_bar_pose_monitor_handover.md` — read §B1 if you want to see how the matching works
   or want to try it on the Oct 1 takes (they are the old rig, so they must come out as "no match").

If anything in the rule or the format gets in the way of your design work, tell me — the tolerance
and the hint text are the two knobs I expect we might turn.

— YH
