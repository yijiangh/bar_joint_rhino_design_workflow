# `tests/test_kinematics.py` Explained

This file documents what each test in [test_kinematics.py](/C:/Users/yijiangh/Dropbox/0_Projects/2025_husky_assembly/Code/bar_joint_rhino_design_workflow/tests/test_kinematics.py) is checking.

The test module covers four main areas:

- `perpendicular_to()`: builds a stable unit vector orthogonal to a given axis.
- `frame_distance()`: scores how far apart two 4x4 frames are in translation and orientation.
- `fk_female()` / `fk_male()`: forward kinematics for the female and male connector frames.
- `optimize_joint_placement()`: solves for the 5 DOFs that align the female and male connector frames for two bars.

## Shared Tolerances

- `TOL_POSITION = 1e-4`
  Used when comparing positions in millimeters.
- `TOL_ORIENTATION = 1e-4`
  Used for axis alignment checks.
- `TOL_RESIDUAL = 1e-3`
  Used for optimization residual quality.

## `TestPerpendicularTo`

### `test_result_is_unit`

This checks that `perpendicular_to()` always returns a unit-length vector, even for axes aligned with coordinate directions and for arbitrary 3D directions.

Why it matters:
- Many downstream frame constructions assume normalized axes.
- If this helper returned a non-unit vector, frame math and distances would drift.

### `test_result_is_perpendicular`

This verifies the returned vector is actually orthogonal to the input axis by checking the dot product is essentially zero.

Why it matters:
- The helper is used to construct local bar frames.
- If the vector is not perpendicular, the resulting frame basis is invalid.

### `test_deterministic`

This calls `perpendicular_to()` twice on the same input and checks the result is exactly the same.

Why it matters:
- A stable deterministic basis avoids solution jitter.
- Optimization and visualization are easier to debug when the frame construction does not randomly flip.

## `TestFrameDistance`

### `test_identical_frames`

This passes the same identity frame twice and expects `frame_distance()` to be essentially zero.

Why it matters:
- It is the base sanity check for the metric.

### `test_translation_only`

This creates two identical frames except for a translation of `(3, 4, 0)`. The expected distance is `3^2 + 4^2 = 25`.

Why it matters:
- It confirms the translation term is squared Euclidean distance.

### `test_rotation_only`

This creates two frames with the same origin but with one frame rotated 90 degrees in the XY plane. The expected result is `4.0`.

Why it matters:
- It confirms the orientation part of the metric is contributing as intended.
- It separates rotation behavior from translation behavior.

## `TestFKSanity`

These tests validate the geometry produced by the direct FK helpers before the optimizer is involved.

### `test_fk_female_origin_on_correct_side`

Setup:
- Existing bar `Le` runs from `(-100, 0, 0)` to `(100, 0, 0)`.
- The female FK is evaluated at `fjp = 50`, `fjr = 0`, `jjr = 0`.

Checks:
- The female frame origin lands at `x = -50`.
- Its distance from the selected point on the bar axis equals `config.FEMALE_RADIAL_OFFSET`.

What it proves:
- `fjp` moves the contact location to the correct place along the bar.
- The female origin is then offset radially by the configured amount.

### `test_fk_male_origin_on_correct_side`

Setup:
- New bar `Ln` runs from `(0, -100, 20)` to `(0, 100, 20)`.
- The male FK is evaluated at `mjp = 100`, `mjr = 0`.

Checks:
- The male frame origin is exactly `config.MALE_RADIAL_OFFSET` away from the corresponding bar-axis point.

What it proves:
- The male FK applies the radial offset consistently relative to the chosen point on `Ln`.

### `test_fk_female_x_axis_along_bar`

Setup:
- `Le` runs along the global X axis.
- The female frame is evaluated at the midpoint.

Checks:
- The female frame X axis is parallel to the bar direction.

What it proves:
- The female OCF convention is being enforced correctly.
- The local X axis follows the parent bar axis.

### `test_fjr_rotates_radial_direction`

Setup:
- Same straight bar along X.
- Compare `fjr = 0` and `fjr = pi`.

Checks:
- The radial offset direction at `fjr = pi` is almost exactly opposite the direction at `fjr = 0`.

What it proves:
- `fjr` rotates the female frame around the bar axis as intended.
- The radial direction is not fixed in space; it follows the rotational DOF.

## `TestOptimizeJointPlacement`

These tests exercise the full 5-DOF solver that tries to align the female and male connector frames for two bars.

### `test_perpendicular_bars_at_contact_distance`

Setup:
- `Le` lies along X through the origin.
- `Ln` lies along Y and is offset in Z by `BAR_CONTACT_DISTANCE`.
- This is the cleanest symmetric case.

Checks:
- Optimization residual is very small.
- Female and male frame origins coincide.

What it proves:
- The optimizer can solve the simplest valid configuration.
- Low residual really corresponds to frame alignment.

### `test_angled_bars`

Setup:
- `Le` stays along X.
- `Ln` is tilted 60 degrees in the XZ plane.
- The bar is shifted along the common normal by the required contact distance.

Checks:
- Residual stays below `TOL_RESIDUAL`.

What it proves:
- The solver works for non-perpendicular, non-parallel 3D cases.

### `test_skew_bars_3d`

Setup:
- `Le` stays along X.
- `Ln` is skew in 3D along a `(0, 1, 1)` direction.
- The closest pair between the bar axes is positioned at the desired contact distance.

Checks:
- Residual stays below `TOL_RESIDUAL`.

What it proves:
- The optimizer handles fully skew 3D bar configurations, not just planar ones.

### `test_dof_values_within_bounds`

Setup:
- Uses the same perpendicular case as the basic solver test.

Checks:
- `fjp`, `fjr`, `mjp`, `mjr`, and `jjr` all lie within their configured ranges.

What it proves:
- The optimizer respects the joint bounds from `config.py`.
- A low-residual answer is not being achieved by stepping outside allowed DOFs.

### `test_fjp_mjp_near_contact_point`

Setup:
- Again uses the perpendicular contact-distance case.

Checks:
- `fjp` is close to `200`.
- `mjp` is close to `200`.

Why those values make sense:
- Each bar is 400 mm long in the test setup.
- The expected contact region is near the midpoint of each bar, which is 200 mm from the start point.

What it proves:
- The optimizer is not just finding any mathematically valid alignment.
- It is finding one near the intuitive contact location on the bars.

## About the `viz` Fixture

Many tests take a `viz` fixture. This does not change the assertions.

It only adds optional plots when you run pytest with `--viz`, so you can inspect:

- the bar geometry,
- the computed frames,
- the optimizer result,
- and the recovered DOF values.

In other words:

- the assertions are the real pass/fail logic,
- the visualization is a debugging aid layered on top.
