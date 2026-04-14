# `tests/test_kinematics.py` Explained

This file explains what the current kinematics tests verify after the inward-FK refactor.

The key contract is now:

- `fk_female_side()` starts from a fixed `le_bar_link` frame and marches inward to `female_screw_hole_link`.
- `fk_male_side()` starts from a fixed `ln_bar_link` frame and reverse-traces inward to `male_screw_hole_link`.
- `optimize_joint_placement()` solves only `fjp`, `fjr`, `mjp`, and `mjr`.
- The optimizer no longer solves `jjr`. Its objective only requires the two screw-hole origins to coincide and their local `Z` axes to align.

## Test Groups

The module is organized into five blocks:

- `TestPerpendicularTo`: keeps the stable helper coverage for constructing debug bar frames.
- `TestFrameDistance`: keeps the generic full-frame mismatch metric grounded.
- `TestScrewHoleOriginZError`: validates the new optimizer objective.
- `TestCADTransforms`: checks that the generated fixed transforms still obey the agreed CAD conventions.
- `TestFKSanity` and `TestOptimizeJointPlacement`: validate the new inward FK APIs and the 4-DOF solve.

## `TestScrewHoleOriginZError`

This is the most important conceptual addition.

The new objective intentionally ignores twist around the shared screw-hole axis. That means:

- identical frames should have zero error,
- rotating one frame about its own local `Z` should still have zero error,
- but changing the frame origin or tilting the local `Z` should increase the error.

The diagnostics helper reports:

- `origin_error_mm`
- `z_axis_error_rad`

Those are the quantities we now care about physically at the interface.

## `TestCADTransforms`

These tests now focus on the agreed fixed-joint assumptions that still matter for inward FK:

- `female_fixed_rot_from_bar` has zero translation,
- `female_male_gap_offset` is translation-only,
- `jjr` zero pose preserves the screw-hole origin and local `Z`,
- `male_screw_hole_offset` includes the required +90 degree local-`Z` quarter turn,
- `male_fixed_rot_to_bar` has zero translation.

The old "female and male fixed rotations are exact inverses" assertion was removed because the real exported CAD transforms are no longer best validated through that single algebraic shortcut.

## `TestFKSanity`

These tests now treat the side helpers as bar-frame driven.

### Female side

- `fk_female_side()` is given a 4x4 `le_bar_link` frame directly.
- `fjp` moves along that frame’s local `+Z`.
- `fjr` rotates around that same local `+Z`.
- The resulting `female_link` origin remains on the bar axis because `female_fixed_rot_from_bar` is rotation-only.
- The vector from `female_link` to `female_screw_hole_link` has the magnitude encoded by `FEMALE_MALE_GAP_OFFSET_TRANSFORM`, and `fjr` rotates that gap vector around the bar axis.

### Male side

- `fk_male_side()` is given a 4x4 `ln_bar_link` frame directly.
- It reverse-traces the URDF chain:
  - inverse of `mjp_joint`
  - inverse of `mjr_joint`
  - inverse of `male_fixed_rot_to_bar`
  - inverse of `male_screw_hole_offset`
- The resulting `male_link` origin remains on the `Ln` bar axis.
- The recovered `mjr_frame` local `Z` stays aligned with the actual `Ln` bar axis.

### Line-based wrappers

The tests also keep coverage for `fk_female_side_from_lines()` and `fk_male_side_from_lines()`.

Those wrappers are intentionally small:

- build the runtime bar frame from the two line endpoints,
- then call the new bar-frame-driven side helper.

That lets older call sites keep working while the core API matches the real conceptual model.

## `TestOptimizeJointPlacement`

The optimizer now solves 4 DOFs instead of 5:

- `fjp`
- `fjr`
- `mjp`
- `mjr`

`jjr` is gone from the solve because twist about the shared screw-hole axis is intentionally free.

The tests check:

- one assembled bar pose generated from a known CAD-consistent chain state,
- a second assembled bar pose with different `fjr / jjr / mjr / mjp`,
- all solved DOFs remain within configured bounds,
- optional optimizer debug/convergence reports are available when requested,
- the final screw-hole origin error is small,
- the final screw-hole `Z`-axis misalignment is small,
- and the result dictionary no longer contains `jjr`.

## Why This Matches The Viewer Work

This test structure now lines up with the live PyBullet validation workflow:

- FK sanity means each side helper should reproduce the URDF link frames when fed the URDF bar frames and slider values.
- Optimization sanity means the inward 4-DOF solve should place the two screw-hole frames on the same axis, while any remaining free twist can be inspected visually through mesh overlays.
