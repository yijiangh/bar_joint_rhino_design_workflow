# `tests/test_kinematics.py` Explained

This file documents what each test in [test_kinematics.py](/C:/Users/yijiangh/Dropbox/0_Projects/2025_husky_assembly/Code/bar_joint_rhino_design_workflow/tests/test_kinematics.py) is checking after the CAD-backed FK refactor.

The updated test module now covers five areas:

- `perpendicular_to()`: builds a stable unit vector orthogonal to a given axis.
- `frame_distance()`: scores how far apart two 4x4 frames are in translation and orientation.
- CAD fixed-transform sanity: confirms the generated fixed transforms are internally consistent.
- `fk_female()` / `fk_male()` and the side-chain helpers: validates the new bar-side and screw-hole-side frame conventions.
- `optimize_joint_placement()`: solves for the 5 DOFs that align the male and female screw-hole frames.

## What Changed Conceptually

The old tests assumed the connector link origins were offset away from the bar axes by simple radial distances.

That is no longer the model.

The new chain uses CAD-derived fixed transforms:

- `female_link` lies directly on the `Le` bar axis because `female_fixed_rot_from_bar` is rotation-only.
- `female_screw_hole_link` is offset from `female_link` by the measured gap transform.
- `male_link` is reached from `male_screw_hole_link` by a fixed quarter-turn around local `Z` plus the measured translation.
- `mjr_link` is a pure rotational remap from `male_link` back to the bar convention.
- The optimizer aligns screw-hole frames, not just simplified connector origins.

## Shared Tolerances

- `TOL_POSITION = 1e-4`
  Used when comparing positions in millimeters.
- `TOL_ORIENTATION = 1e-4`
  Used for axis and rotation-matrix consistency checks.
- `TOL_RESIDUAL = 1e-4`
  Used for optimization residual quality.

## `TestPerpendicularTo`

These tests did not change conceptually.

### `test_result_is_unit`

Checks that `perpendicular_to()` always returns a unit-length vector.

Why it matters:

- Bar-frame construction depends on normalized basis vectors.
- If this helper drifted away from unit length, downstream transforms would accumulate scale errors.

### `test_result_is_perpendicular`

Checks that the returned vector is truly orthogonal to the input axis.

Why it matters:

- The helper is used when constructing stable bar-side frames.
- If the vector is not perpendicular, the resulting basis is not a valid frame.

### `test_deterministic`

Calls `perpendicular_to()` twice on the same input and requires an identical answer.

Why it matters:

- Deterministic basis construction prevents random frame flips.
- That makes the FK, optimizer seeds, and debugging output much easier to reason about.

## `TestFrameDistance`

These tests also keep the same intent as before.

### `test_identical_frames`

Passes the same identity frame twice and expects `frame_distance()` to be zero.

### `test_translation_only`

Translates one frame by `(3, 4, 0)` and checks that the translation contribution is `25`.

### `test_rotation_only`

Keeps the origin fixed and rotates the basis by 90 degrees in-plane to confirm the orientation term contributes as expected.

Why this block matters:

- `frame_distance()` is the scalar objective used inside the optimizer.
- These tests keep that metric grounded before the harder kinematic tests run.

## `TestCADTransforms`

This is new. These tests verify that the generated CAD fixed transforms form a coherent chain before any FK is exercised.

### `test_fixed_rotations_are_inverses`

Checks that:

- `FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM[:3, :3]`
- `MALE_FIXED_ROT_TO_BAR_TRANSFORM[:3, :3]`

multiply to the identity.

What it proves:

- The bar-to-female remap and the male-to-bar remap are consistent inverses.
- The clarified axis-remap convention was encoded correctly in config.

### `test_gap_transform_is_translation_only`

Checks that `FEMALE_MALE_GAP_OFFSET_TRANSFORM` has identity rotation.

What it proves:

- The female-to-female-screw-hole relation is a pure positional gap.
- Rotation around the screw-hole axis belongs to `jjr_joint`, not to the fixed gap transform.

### `test_male_screw_hole_offset_is_quarter_turn_about_local_z`

Checks that `MALE_SCREW_HOLE_OFFSET_TRANSFORM[:3, :3]` is exactly the +90 degree rotation about local `Z`:

```text
[[ 0, -1, 0],
 [ 1,  0, 0],
 [ 0,  0, 1]]
```

What it proves:

- The male screw-hole to male-link fixed joint is not being simplified away as translation-only.
- The intended local quarter-turn is explicitly encoded in config and will be shared by FK and URDF generation.

## `TestFKSanity`

These tests validate the direct FK behavior under the new chain conventions.

### `test_fk_female_origin_lies_on_bar_axis`

Setup:

- `Le` runs along global `+Z`.
- Evaluate `fk_female()` at `fjp = 50`, `fjr = 0`.

Checks:

- The returned `female_link` origin is exactly `(0, 0, 50)`.

What it proves:

- `fjp` is interpreted as motion along the parent bar axis.
- `female_link` itself sits on the bar axis, matching the agreed rotation-only `female_fixed_rot_from_bar`.

### `test_fk_male_origin_lies_on_bar_axis`

Setup:

- `Ln` also runs along global `+Z`, offset in `Y` by `BAR_CONTACT_DISTANCE`.
- Evaluate `fk_male()` with `mjp = -125`, `mjr = 0`.

Checks:

- The returned `male_link` origin is exactly `(0, -BAR_CONTACT_DISTANCE, 125)`.

Why the test uses `-125`:

- In the agreed URDF chain, `mjp_joint` is a prismatic joint from `mjp_link` to `ln_bar_link`.
- The direct inverse FK therefore uses the opposite sign when reconstructing the upstream `male_link` from a known `ln_bar_link`.

What it proves:

- The male-side FK is consistent with the actual parent/child ordering of the URDF.
- `male_link` also lies on the `Ln` bar axis in this CAD-backed model.

### `test_fk_female_x_axis_along_bar`

Setup:

- `Le` runs along global `+X`.

Checks:

- The `female_link` X axis is parallel to the bar direction.

What it proves:

- The female link frame convention is correct after the fixed axis remap.
- The old bar-axis direction really becomes the new local `X` of `female_link`.

### `test_fk_male_x_axis_along_bar`

Setup:

- `Ln` runs along global `+Y`.

Checks:

- The `male_link` X axis is parallel to the `Ln` bar direction.

What it proves:

- The male-side remap back from screw-hole/bar conventions is being applied correctly.

### `test_fjr_rotates_female_contact_direction`

Setup:

- Compare `fk_female_side()` at `fjr = 0` and `fjr = pi`.

Checks:

- The vector from `female_link` to `female_screw_hole_link` has magnitude `BAR_CONTACT_DISTANCE`.
- That vector flips direction between `fjr = 0` and `fjr = pi`.

What it proves:

- `fjr` rotates the female-side contact direction around the bar axis.
- The rotating quantity is now the screw-hole/contact offset, not the `female_link` origin itself.

### `test_male_side_bar_axis_matches_mjr_frame_z`

Setup:

- `Ln` is placed diagonally in the XY plane.
- Evaluate `fk_male_side()`.

Checks:

- The `mjr_frame` local `Z` axis is parallel to the actual `Ln` bar direction.

What it proves:

- The chain reaches the correct bar-side convention before `mjr_joint`.
- `mjr_joint` is indeed rotating around the intended bar axis.

## `TestOptimizeJointPlacement`

These tests now validate the screw-hole-based solver.

The key result is no longer “female and male connector origins coincide.” Instead, the key result is that the predicted male screw-hole frame reconstructed from the female side and `jjr` matches the actual male screw-hole frame reconstructed from the male side.

### `test_parallel_bars_at_contact_distance`

Setup:

- `Le` and `Ln` are parallel along `+Z`.
- They are separated by `BAR_CONTACT_DISTANCE`.

Checks:

- Optimization residual is small.
- `predicted_male_screw_hole_frame` matches `male_screw_hole_frame`.

What it proves:

- The new solver handles the simplest CAD-aligned case.
- The screw-hole alignment target is working for parallel bars.

### `test_perpendicular_bars_at_contact_distance`

Setup:

- `Le` runs along `+Z`.
- `Ln` runs along `+X`.
- The bars are placed so their shortest distance is `BAR_CONTACT_DISTANCE`.

Checks:

- Optimization residual is small.
- The predicted and actual male screw-hole frames match.

What it proves:

- The solver works when the two bar axes are orthogonal.
- The fixed-transform chain plus the 5 DOFs can recover a valid connector placement in a less symmetric pose.

### `test_dof_values_within_bounds`

Setup:

- Reuses the parallel-bar contact-distance case.

Checks:

- `fjp`, `fjr`, `mjp`, `mjr`, and `jjr` all stay within the configured ranges.

What it proves:

- The optimizer is respecting the declared joint limits while solving the CAD-backed chain.

## Why There Is No `viz` Discussion Anymore

The old explainer mentioned the optional `viz` fixture because many tests rendered debug plots.

The rewritten `tests/test_kinematics.py` is now purely assertion-based and does not use `viz`.

That is intentional:

- the kinematic contract is now tighter and more CAD-specific,
- the assertions are easier to read directly,
- and the visualization burden has shifted to the URDF/static PyBullet tools that now show the real connector meshes and link frames.
