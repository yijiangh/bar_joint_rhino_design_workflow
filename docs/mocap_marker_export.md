# MoCap marker export — `MoCapMarkers.json`

How the designed positions of the MoCap marker spheres leave Rhino, how the Motive markers are
named to match, and what the teleop monitor is expected to do with them. Step-by-step use: the
README, *Rhino 8 Workflow › MoCap joints*.

**Status (2026-10-16).** The export and the Rhino labels work (tested in Rhino on
`260916_2ModulesDemo.3dm`). Reading `T20_MoCap` markers back into Rhino and the live comparison
in the teleop monitor are not built yet. Planned next: this file's content moves into
`<doc>_prefab.json`, written by RSExportPrefab alone, with each joint's distance to the bar end
and each bar's marker-to-marker distance table added.

## Why a separate file

`RobotCell.json` carries every MoCap joint as a mesh (`joint_J20-21_mocap`) but no pose, and the
BarActions carry the joints' world frames but not the spheres. The sphere centres exist only in the
joint registry, block-local: `scripts/core/joint_pairs.json` › `T20_MoCap.marker_points_mm`,
recorded by **RSDefineJointHalf** when you pick the sphere objects and label them `M1`, `M2`.
`MoCapMarkers.json` puts the two together so the teleop side never needs the registry or Rhino.

## Writing it

**RSExportRobotCell** writes it next to `RobotCell.json`, in the same export root; so does
**RSExportAllBarActions**. No separate button. When no MoCap joint with recorded markers is
placed, the file is not written and the console says so.

The same export redraws the layer **MANAGED Scaffolding › MoCap Marker Labels**:

- a text dot at every designed sphere, `B20 #3 M1`: the bar, the marker's number on that bar,
  and its label on the joint. The dot's object name is the full marker name, `J20-21_mocap_M1`;
- a line through each joint's spheres — its designed **marker-pair line**.

Use these dots to name the markers in Motive.

## Names and numbering

- A marker's name is `<joint object name>_<label>`: `J20-21_mocap_M1` is the sphere labelled `M1`
  on the MoCap joint of pair J20-21 (the MoCap half sits on bar 20, the first bar of the id).
  A standalone MoCap joint's sphere is e.g. `M20-T20-0_mocap_M2`. One rule, in
  `core/joint_name_conventions.py` › `marker_name`.
- Each bar's markers are also numbered `1..N`: MoCap joints ordered along the bar from its start,
  and within a joint its labels in order (`M1`, `M2`). Any number of MoCap joints per bar works.

## File format (`schema_version` 1)

All positions in **mm**, in the **Rhino world** (Z up). The bar frame is the one the RobotCell bar
bodies use: origin at the bar's start point, z along the bar
(`joint_pair.canonical_bar_frame_from_line`).

```json
{
  "schema_version": 1, "source": "261008_2ModulesDemo.3dm", "units": "mm", "world": "rhino",
  "bars": [
    {
      "bar_id": "B20", "assembly_seq": 1,
      "start_mm": [x, y, z], "end_mm": [x, y, z], "length_mm": 1800.0,
      "joints": [
        {"joint_id": "J20-22", "name": "J20-22_mocap", "paired": true,
         "block_name": "T20_MoCap", "position_mm": 104.9,
         "frame_world_mm": [[4x4]], "marker_indices": [1, 2]}
      ],
      "markers": [
        {"index": 1, "name": "J20-22_mocap_M1", "joint_id": "J20-22", "label": "M1",
         "world_mm": [1240.83, -2250.59, 127.97], "bar_mm": [x, y, z]}
      ]
    },
    {"bar_id": "B22", "assembly_seq": 3, "start_mm": [x, y, z], "end_mm": [x, y, z],
     "length_mm": 1800.0, "joints": [], "markers": []}
  ],
  "warnings": []
}
```

- Every real bar is listed, in assembly order; fake bars are not. A bar without MoCap joints has
  empty `joints` / `markers` — its centre line is still there.
- `marker_indices` are a joint's markers in order: the designed marker-pair line.
- `position_mm` is the joint block's origin along the bar from its start.
- `paired` is false for a standalone MoCap joint (`M…` id).
- `warnings` lists MoCap joints that were skipped (parent bar not exported, or a block with no
  recorded marker points).

## Setting up Motive

1. Export the design: RSExportRobotCell (or RSExportAllBarActions).
2. Put the joints on the physical bar with the jig (`joint_jig_controller`, from the prefab export).
3. In Motive, make **one rigid body per instrumented bar** from its spheres. Naming each marker
   after the Rhino dots (`J20-21_mocap_M1`, …) is recommended; the teleop monitor reads the names
   from Motive's asset descriptions and shows them.
4. The teleop monitor works out which streamed asset is which bar from the marker geometry, and
   records it; it warns before recording if anything looks ambiguous.

## Reading markers back into Rhino — not for `T20_MoCap` yet

RSReadMoCapBar and RSAlignModelThreeBars (RSMoCap tab) are built for the older 4-marker bar rig:
exactly 4 markers in two pairs 70–90 mm apart, with the line between the pair centres on the bar
axis. A `T20_MoCap` pair is 48.3 mm long and its centre sits 25.5 mm off the axis, so
RSReadMoCapBar rejects such a rigid body and RSAlignModelThreeBars would fit the bar 25.5 mm off.

## What the teleop side will do with it (husky-assembly-teleop; planned, not built)

- **Which asset is which bar.** Compare each asset's pairwise marker distances with every
  designed bar of the same marker count; the best rigid fit names the bar and orders its markers.
  The result (streaming id, bar, marker order, Motive names) is kept in the problem folder and
  re-checked each session; a match to a bar already recorded under another id, a near tie between
  two bars, or no match at all, stops recording with a warning.
- **Placement.** Fit the designed markers onto the measured ones with one rigid motion (Kabsch).
  That motion applied to the designed centre line gives the measured bar: start / middle / end
  distance, rotation angle, lateral offset.
- **Bending.** With that rigid motion removed, what remains is shape change: per-marker residual;
  per pair, its midpoint's distance from the axis and its angle about the axis vs designed, and its
  length (48.3 mm on `T20_MoCap`; a length error means a mislabelled or occluded marker); between
  neighbouring pairs, the distance along and across the axis vs designed.
- Do **not** reuse the old `bar_rig` fit (a line through pair midpoints): on `T20_MoCap` each
  pair's midpoint sits 25.5 mm off the bar axis.
