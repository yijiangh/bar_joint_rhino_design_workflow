# MoCap marker export — `MoCapMarkers.json` (schema 2)

How the designed positions of the MoCap marker spheres leave Rhino, what the file guarantees, how
the Motive markers are named to match, and what the teleop monitor does with it. Step-by-step use:
the README, *Rhino 8 Workflow › MoCap joints*.

**Status (2026-10-16).** Schema 2 replaces schema 1 (world positions, bar lines, joint transforms).
Nothing reads schema 1 any more; re-export. The monitor side (matching live Motive markers to this
file and fitting the bar pose) is planned in `tasks/mocap_bar_pose_monitor_handover.md` and
implemented in [husky-assembly-teleop].

## Why a separate file, and why it is slim

`RobotCell.json` carries every bar and every MoCap joint as a collision body, with no pose. The
BarAction states carry every body's pose per movement (metres, world). The sphere centres exist
only in the joint registry, block-local (`scripts/core/joint_pairs.json` › `T20_MoCap` ›
`marker_points_mm`, recorded by **RSDefineJointHalf**). compas_fab's `RigidBody` has no free field
to carry them, and adding one would break the monitor's loader, so they travel in this sidecar.

The sidecar holds **only what the cell and the BarActions do not have**: per bar, where each marker
sits in the **bar body's own frame**, in metres. That frame is the one the bar body uses in
`RobotCell.json` and in every BarAction `RigidBodyState.frame` (origin at the bar start, z along
the bar), so a transform that puts these points onto the measured Motive markers *is* the measured
`T_world_from_bar`, directly comparable to the designed state frame. World positions, bar centre
lines and joint transforms are not repeated.

## Writing it

**RSExportRobotCell** and **RSExportAllBarActions** write it next to `RobotCell.json`, from the
same cached cell snapshot as the cell itself (not from live curves), cut to the same bars in a
From..Until range export. So the two files can never disagree.

The file is built **before** anything is written. If a bar is refused (below) or a MoCap block has
no recorded markers or diameter, the command stops with a message naming the joint or bar and the
folder is left untouched. When no MoCap marker is placed the file is not written and a stale
`MoCapMarkers.json` in the folder is **deleted**, so the monitor never reads last week's markers.

The same export redraws the layer **MANAGED Scaffolding › MoCap Marker Labels** from the file's
own numbers:

- a text dot at every designed sphere, `B20 #3 M1`: the bar, the marker's number on that bar, and
  its label on the joint. The dot's object name is the full marker name, `J20-21_mocap_M1`;
- a line through each joint's spheres — its designed **marker-pair line**.

Use these dots to name the markers in Motive.

## The layout rule: a bar is refused when Motive could see it in two poses

The monitor does not trust Motive's marker ids (they shuffle). It works out which measured point is
which designed marker from geometry alone, then fits the bar pose. That has one answer only when
**no other rigid motion maps the bar's designed marker set onto itself**. Precisely, a layout is
ambiguous when some non-identity permutation of its markers can be produced by a *proper* rigid
transform (a rotation, det = +1 — no mirror) with a maximum residual at or below the tolerance,
`config.MOCAP_LAYOUT_TOL_MM` = 2 mm (about twice the Motive per-marker noise). The check lives in
`scripts/core/mocap_layout_check.py` and is also the reference for the monitor's matcher.

Why not compare pairwise-distance patterns? On a long bar the two markers of one 48 mm pair have
almost the same distances to every other marker (those sit a metre away), so a distance-only rule
refuses perfectly fine bars; and distances cannot tell a mirror image from a rotation, but a mirror
is not a pose the bar can take. So distances only shrink the search; the decision is the full-set
residual of a proper fit.

Two exact ambiguities of the `T20_MoCap` plate (two spheres, 48.3 mm apart, 25.5 mm off the axis):

- **two joints on a bar are never enough**, whatever their spacing or relative turn — a half-turn
  about the line bisecting the two plate normals maps the set onto itself end for end;
- **joints that all face the same way are never enough** — a half-turn about the plate midline
  swaps M1 and M2 in every joint.

**Rule of thumb: at least three MoCap joints per tracked bar, not all facing the same way, not
mirror-symmetric about the bar's middle.** A bar with fewer than three markers, or with all
markers on one line, is refused too (the roll is free). Each bar's entry records how close the
nearest rejected candidate came (`layout.nearest_miss_m`); the export prints a note when that is
under three times the tolerance.

## Names and numbering

- A marker's name is `<joint object name>_<label>`: `J20-21_mocap_M1` is the sphere labelled `M1`
  on the MoCap joint of pair J20-21 (the MoCap half sits on bar 20, the first bar of the id). A
  standalone MoCap joint's sphere is e.g. `M20-T20-0_mocap_M2`. One rule, in
  `core/joint_name_conventions.py` › `marker_name`.
- Per bar, markers are listed with the MoCap joints ordered along the bar from its start, and
  within a joint its labels in order (`M1`, `M2`). The Rhino dot's number is the marker's position
  in that list, counting from 1.

## File format (`schema_version` 2)

All positions in **metres**. Bars are keyed by their cell body name; only bars that carry markers
are listed, the rest of the export's bar bodies appear in `bars_without_markers`.

```json
{
  "schema_version": 2,
  "source": "261016_2ModulesDemo_YH.3dm",
  "units": "m",
  "cell_file": "RobotCell.json",
  "bar_frame": "frame of the bar body of the same name in the cell / BarAction states: origin at the bar start, z along the bar; world = bar_state_frame @ bar_m",
  "marker_name": "<joint object name>_<label>, e.g. J20-21_mocap_M1 (name the Motive marker this)",
  "layout_tolerance_m": 0.002,
  "marker_blocks": {
    "T20_MoCap": {
      "marker_diameter_m": 0.019,
      "points_block_m": {"M1": [-0.01, 0.02415, 0.0255], "M2": [-0.01, -0.02415, 0.0255]}
    }
  },
  "bars": {
    "bar_B20": {
      "bar_id": "B20",
      "assembly_seq": 1,
      "layout": {"n_markers": 8, "nearest_miss_m": 0.0412},
      "markers": [
        {"name": "J20-22_mocap_M1", "joint": "joint_J20-22_mocap", "label": "M1",
         "block": "T20_MoCap", "bar_m": [-0.0274, -0.022, 0.0849]},
        {"name": "J20-22_mocap_M2", "joint": "joint_J20-22_mocap", "label": "M2",
         "block": "T20_MoCap", "bar_m": [-0.0235, 0.0261, 0.0849]}
      ]
    }
  },
  "bars_without_markers": ["bar_B22", "bar_B23"]
}
```

| Field | Meaning |
|---|---|
| `marker_blocks.<block>.marker_diameter_m` | the sphere size recorded on the block (RSDefineJointHalf); the monitor drops Motive markers of another size |
| `marker_blocks.<block>.points_block_m` | the registry's block-local centres, for reference |
| `bars.<body>.assembly_seq` | the bar's sequence number (`null` when it has none) |
| `bars.<body>.layout.nearest_miss_m` | residual of the best *rejected* self-match; `null` when distances ruled every candidate out |
| `markers[].joint` | the joint body's name in `RobotCell.json` (`joint_J20-22_mocap`) |
| `markers[].bar_m` | the sphere centre in the bar body frame, rounded to 0.01 mm |

Dropped from schema 1: `world_mm`, `start_mm` / `end_mm` / `length_mm`, joint `frame_world_mm`,
`position_mm`, `marker_indices`, `index`, `warnings`. A marker's world position is
`bar_state_frame @ bar_m` with the frame from the BarAction state (or from the cell snapshot).

## Setting up Motive

1. Export the design: RSExportRobotCell (or RSExportAllBarActions).
2. Put the joints on the physical bar with the jig (`joint_jig_controller`, from the prefab export).
3. In Motive, make **one rigid body per instrumented bar** from its spheres and name the asset
   after the bar body (`bar_B20`). Naming each marker after the Rhino dots (`J20-21_mocap_M1`, …)
   is recommended for reading the live view; the monitor does not rely on it.

## Reading markers back into Rhino — not for `T20_MoCap`

RSReadMoCapBar and RSAlignModelThreeBars (RSMoCap tab) are built for the older 4-marker bar rig:
exactly 4 markers in two pairs 70–90 mm apart, with the line between the pair centres on the bar
axis. A `T20_MoCap` pair is 48.3 mm long and its centre sits 25.5 mm off the axis, so
RSReadMoCapBar rejects such a rigid body and RSAlignModelThreeBars would fit the bar 25.5 mm off.

## What the teleop monitor does with it (planned, see `tasks/mocap_bar_pose_monitor_handover.md`)

- Loads this file next to `RobotCell.json`; accepts schema 2 only.
- Per instrumented bar, reads Motive's labelled markers as loose points (ids ignored), keeps those
  of the recorded diameter, and matches them to `bar_m` by geometry: a three-point hypothesis,
  a proper (det = +1) Kabsch fit, then the full-set residual decides. The fit transform is the
  measured `T_world_from_bar`; compared with the bar body's designed state frame it gives the
  placement error along / across the bar and the rotation error. Per-marker residuals show
  bending or a mislabelled / occluded marker.
- Follow-ups: use the measured pose of a grounded bar as the visual-servo target; log measured vs
  designed after every assembly / hold action (as-built study).
