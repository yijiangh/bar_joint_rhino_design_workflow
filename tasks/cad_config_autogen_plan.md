# CAD Config Autogen Plan

## Goal

Use the annotated Rhino frames as the source of truth for connector geometry, auto-generate the CAD-backed config data, and drive both FK and URDF generation from the same fixed transforms.

The agreed chain is:

1. `le_bar_link`
2. `fjp_joint` -> `fjp_link`
3. `fjr_joint` -> `fjr_link`
4. `female_fixed_rot_from_bar` -> `female_link`
5. `female_male_gap_offset` -> `female_screw_hole_link`
6. `jjr_joint` -> `male_screw_hole_link`
7. `male_screw_hole_offset` -> `male_link`
8. `male_fixed_rot_to_bar` -> `mjr_link`
9. `mjr_joint` -> `mjp_link`
10. `mjp_joint` -> `ln_bar_link`

## Agreed Joint Conventions

### `fjp_joint`

- Type: prismatic
- Parent: `le_bar_link`
- Child: `fjp_link`
- Motion: translate along local `+Z` of `le_bar_link`

### `fjr_joint`

- Type: revolute
- Parent: `fjp_link`
- Child: `fjr_link`
- Motion: rotate around local `+Z` of `fjp_link`

### `female_fixed_rot_from_bar`

- Type: fixed
- Parent: `fjr_link`
- Child: `female_link`
- Translation: zero
- Rotation remap:
  - old `Z` -> new `X`
  - old `X` -> new `Y`
  - old `Y` -> new `Z`

### `female_male_gap_offset`

- Type: fixed
- Parent: `female_link`
- Child: `female_screw_hole_link`
- Translation: along local negative `Z` of `female_link`
- Rotation: identity

### `jjr_joint`

- Type: revolute
- Parent: `female_screw_hole_link`
- Child: `male_screw_hole_link`
- Motion: rotate around local `+Z` of `female_screw_hole_link`
- Joint origin should come from the measured zero-pose transform between the two screw-hole frames, with validation that it preserves the shared `Z` axis.

### `male_screw_hole_offset`

- Type: fixed
- Parent: `male_screw_hole_link`
- Child: `male_link`
- Rotation: first rotate `male_link` by +90 degrees around local `Z` of `male_screw_hole_link`
- Translation: then apply the measured local XYZ translation from CAD

### `male_fixed_rot_to_bar`

- Type: fixed
- Parent: `male_link`
- Child: `mjr_link`
- Translation: zero
- Rotation remap:
  - old `Y` -> new `X`
  - old `Z` -> new `Y`
  - old `X` -> new `Z`

### `mjr_joint`

- Type: revolute
- Parent: `mjr_link`
- Child: `mjp_link`
- Motion: rotate around local `+Z` of `mjr_link`

### `mjp_joint`

- Type: prismatic
- Parent: `mjp_link`
- Child: `ln_bar_link`
- Motion: translate along local `+Z` of `mjp_link`

## Rhino Export Workflow

Add a Rhino command script `scripts/export_cad_config.py`.

The script should:

1. Prompt the user to select the baked frame groups for:
   - `le_bar_link`
   - `female_link`
   - `female_screw_hole_link`
   - `male_screw_hole_link`
   - `male_link`
   - `ln_bar_link`
2. Reconstruct each frame from the grouped axis lines produced by `bake_frame.py`.
3. Convert Rhino document units to millimeters.
4. Compute the bar-axis shortest distance from the two bar-link frame `Z` axes using `core.geometry`.
5. Derive and validate the fixed transforms:
   - `female_fixed_rot_from_bar`
   - `female_male_gap_offset`
   - `jjr_joint` zero transform
   - `male_screw_hole_offset`
   - `male_fixed_rot_to_bar`
6. Write:
   - `scripts/core/config_generated.py`
   - `scripts/core/cad_frames_snapshot.json`
7. Optionally bake the shortest-distance segment between the two bar axes as a validation aid.

## Config Structure

Keep `scripts/core/config.py` as the stable public module and have it import generated CAD values from `config_generated.py` when present.

The generated layer should expose:

- Bar/reference data
  - `BAR_CONTACT_DISTANCE`
  - `LE_BAR_REFERENCE_FRAME`
  - `LN_BAR_REFERENCE_FRAME`
- Fixed transforms
  - `FEMALE_FIXED_ROT_FROM_BAR`
  - `FEMALE_MALE_GAP_OFFSET`
  - `JJR_ZERO_TRANSFORM`
  - `MALE_SCREW_HOLE_OFFSET`
  - `MALE_FIXED_ROT_TO_BAR`
- Visual assets
  - `FEMALE_MESH_FILENAME`
  - `MALE_MESH_FILENAME`
  - mesh scales if needed

For compatibility, `config.py` may also expose derived legacy values if any existing scripts still read them.

## Kinematics Refactor

Refactor `scripts/core/kinematics.py` to use the measured fixed transforms instead of the old scalar radial/axial offsets.

The optimized alignment should be based on the screw-hole frames:

- Build the female side from `Le`, `fjp`, and `fjr` through `female_screw_hole_link`.
- Build the male side from `Ln`, `mjp`, and `mjr` backward to `male_screw_hole_link`.
- Use the measured `jjr` zero transform plus the `jjr` angle to compare the two screw-hole frames.

Return the solved link frames needed by Rhino and tests:

- `female_frame`
- `female_screw_hole_frame`
- `male_screw_hole_frame`
- `male_frame`

## URDF Refactor

Update `scripts/generate_urdf.py` to:

- emit the agreed link and joint names
- use the measured fixed transforms from config
- replace placeholder female/male box visuals with:
  - `scripts/female_joint_mesh_m.obj`
  - `scripts/male_joint_mesh_m.obj`
- replace `mjr_out_link` with `mjp_link`

The OBJ files are already baked at their link OCFs, so the visual origins should stay identity unless later validation shows otherwise.

## Static Viewer Update

Update `scripts/pb_viz_urdf_static.py` to:

- load the regenerated URDF
- annotate the renamed links
- preserve the real mesh visuals instead of globally recoloring all links red

## Verification

Add or update tests so they validate:

- the fixed transform conventions from config
- the FK alignment against the new screw-hole-based chain
- the URDF structure and renamed links
- the mesh URDF references for `female_link` and `male_link`

## Notes

- `female_link` origin is intentionally constrained to lie on the `Le` bar axis.
- `male_fixed_rot_to_bar` was clarified from the typo and completed by right-handed basis closure.
- The CAD exporter should fail loudly if a measured transform contradicts one of the agreed fixed-joint assumptions.
