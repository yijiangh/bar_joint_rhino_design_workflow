# Codex Task: URDF chain restructure + kinematics update + FK cross-check

## Overview

The `jjr_joint` directly connects `female_link → male_link` at origin (0,0,0), causing box overlap. Fix: introduce screw-hole links between them. This changes the kinematic model, so kinematics.py, the optimizer, t2_joint_placement.py, tests, and the pybullet viewer all need updating.

**Files to modify (in dependency order):**
1. `scripts/core/config.py` — add screw hole chain parameters
2. `scripts/core/kinematics.py` — remove jjr from fk_female, add `female_to_male_chain`, update optimizer
3. `scripts/generate_urdf.py` — URDF chain restructure
4. `scripts/pybullet_viewer.py` — simplify FK cross-check
5. `scripts/t2_joint_placement.py` — adapt to new frame semantics
6. `tests/test_kinematics.py` — update tests
7. `tests/test_kinematics_explained.md` — update docs

---

## 1. `scripts/core/config.py`

Add two new parameters for the screw hole chain geometry:

```python
# Screw hole chain offsets
# female OCF → screw hole: translate -z by this distance
SCREW_HOLE_Z_OFFSET = BAR_CONTACT_DISTANCE
# screw hole → male OCF: translate -y by this distance
MALE_LINK_Y_OFFSET = 12.0  # = MALE_BOX_SIZE[1]/2 from generate_urdf.py (24/2)
```

Constraint: `FEMALE_BOX_SIZE[2] + MALE_BOX_SIZE[1]` (in generate_urdf.py) should equal `BAR_CONTACT_DISTANCE`. Currently 18+24=42≠20, so either update the box sizes or BAR_CONTACT_DISTANCE. **Ask user or leave a TODO comment if unclear.**

---

## 2. `scripts/core/kinematics.py`

### 2a. Remove `jjr` parameter from `fk_female`

Change signature from:
```python
def fk_female(Le_start, Le_end, fjp, fjr, jjr, config) -> np.ndarray:
```
to:
```python
def fk_female(Le_start, Le_end, fjp, fjr, config) -> np.ndarray:
```

Remove lines 122-124 (the jjr rotation of x,y axes). The function now returns the pure female OCF.

### 2b. Add `female_to_male_chain` function

New function that computes the predicted male frame by traversing the screw hole chain from the female OCF:

```python
def female_to_male_chain(female_ocf, jjr, config) -> np.ndarray:
    """Traverse female → screw_hole → jjr rotation → male offset → predicted male frame.
    
    Chain: female_ocf @ T1(-z * SCREW_HOLE_Z_OFFSET) @ Rz(jjr) @ T2(-y * MALE_LINK_Y_OFFSET)
    All distances in mm (same units as female_ocf).
    """
    female_ocf = np.asarray(female_ocf, dtype=float)
    
    # T1: translate to screw hole in female's local -z
    T1 = np.eye(4, dtype=float)
    T1[2, 3] = -float(config.SCREW_HOLE_Z_OFFSET)
    
    # Rz(jjr): revolute around local z at screw hole
    R_jjr = np.eye(4, dtype=float)
    R_jjr[:3, :3] = _rotation_matrix([0, 0, 1], float(jjr))
    
    # T2: translate to male center in rotated frame's local -y
    T2 = np.eye(4, dtype=float)
    T2[1, 3] = -float(config.MALE_LINK_Y_OFFSET)
    
    return female_ocf @ T1 @ R_jjr @ T2
```

### 2c. Update `optimize_joint_placement` objective

In the `objective` inner function (line ~176-178), change:

```python
# OLD:
female_frame = fk_female(le_start, le_end, fjp, fjr, jjr, config)
male_frame = fk_male(ln_start, ln_end, mjp, mjr, config)
return frame_distance(female_frame, male_frame)

# NEW:
female_frame = fk_female(le_start, le_end, fjp, fjr, config)  # no jjr
predicted_male = female_to_male_chain(female_frame, jjr, config)
actual_male = fk_male(ln_start, ln_end, mjp, mjr, config)
return frame_distance(predicted_male, actual_male)
```

Also update the return block (lines ~217-228):

```python
# OLD:
female_frame = fk_female(le_start, le_end, fjp, fjr, jjr, config)
male_frame = fk_male(ln_start, ln_end, mjp, mjr, config)

# NEW:
female_frame = fk_female(le_start, le_end, fjp, fjr, config)  # no jjr
male_frame = fk_male(ln_start, ln_end, mjp, mjr, config)
```

The returned `female_frame` is now the pure female OCF (connector body center). `male_frame` is the pure male OCF. They do NOT coincide — they're connected through the screw hole chain.

---

## 3. `scripts/generate_urdf.py`

### 3a. Remove `visual_origin_xyz` from female_link and male_link

In the `female_link` `_add_link` call, remove `visual_origin_xyz=(0.0, 0.0, -FEMALE_BOX_SIZE[2] / 2.0 * s)`.
In the `male_link` `_add_link` call, remove `visual_origin_xyz=(0.0, 0.0, MALE_BOX_SIZE[2] / 2.0 * s)`.

### 3b. Add two new invisible links

After `female_link` and before `male_link`:

```python
_add_link(robot, "female_screw_hole_link", mass=0.01, inertia=0.0001)
_add_link(robot, "male_screw_hole_link", mass=0.01, inertia=0.0001)
```

### 3c. Replace `jjr_joint` with three joints

Delete the current `jjr_joint` (lines ~190-200). Replace with:

```python
# Joint 1: female → screw hole (fixed, -z by BAR_CONTACT_DISTANCE)
_add_joint(
    robot,
    "female_male_gap_offset",
    "fixed",
    "female_link",
    "female_screw_hole_link",
    (0.0, 0.0, -config.BAR_CONTACT_DISTANCE * s),
    (0.0, 0.0, 0.0),
)

# Joint 2: jjr revolute at screw hole
_add_joint(
    robot,
    "jjr_joint",
    "revolute",
    "female_screw_hole_link",
    "male_screw_hole_link",
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    axis_xyz=(0.0, 0.0, 1.0),
    limits=config.JJR_RANGE,
)

# Joint 3: screw hole → male (fixed, -y by MALE_BOX_SIZE[1]/2)
_add_joint(
    robot,
    "male_screw_hole_offset",
    "fixed",
    "male_screw_hole_link",
    "male_link",
    (0.0, -MALE_BOX_SIZE[1] / 2.0 * s, 0.0),
    (0.0, 0.0, 0.0),
)
```

Everything after `male_link` (male_offset_joint → mjr_link → mjr_joint → mjr_out_link → mjp_joint → ln_bar) stays unchanged.

### Final chain:
```
le_bar → fjp → fjp_link → fjr → fjr_link → female_offset(fixed) → female_link [BOX]
  → female_male_gap_offset(fixed, -z) → female_screw_hole_link
    → jjr(revolute) → male_screw_hole_link
      → male_screw_hole_offset(fixed, -y) → male_link [BOX]
        → male_offset(fixed) → mjr_link → mjr → mjr_out_link → mjp → ln_bar
```

---

## 4. `scripts/pybullet_viewer.py`

### 4a. Delete these functions

- `_compose_frame`, `_rotate_about_local_z`, `_translate_along_local_z`
- `_build_analytic_chain_frames`, `_male_frame_from_fk_male`
- `_create_visual_shape`, `_create_visual_body`, `_create_ghost_bodies`
- `_update_body_pose`, `_set_ghost_body_poses`
- `_comparison_lines`, the old `_build_status_line`

Clean up unused imports (`FIXED_JOINT_RPY`, `FEMALE_BOX_SIZE`, `MALE_BOX_SIZE` if orphaned).

### 4b. Add `_draw_pose()`

```python
def _draw_pose(frame_m, line_ids, prefix, offset, length=0.02):
    origin = frame_m[:3, 3] + offset
    for axis_name, col_idx, color in [("x",0,[1,0,0]), ("y",1,[0,1,0]), ("z",2,[0,0,1])]:
        key = f"{prefix}_{axis_name}"
        tip = origin + length * frame_m[:3, col_idx]
        line_ids[key] = p.addUserDebugLine(
            origin.tolist(), tip.tolist(), color,
            lineWidth=3.0, lifeTime=0.0,
            replaceItemUniqueId=line_ids.get(key, -1),
        )
```

### 4c. Add `_compute_fk_crosscheck()`

```python
def _compute_fk_crosscheck(le_bar_frame_m, ln_bar_frame_m, values):
    le_start_mm, le_end_mm = _le_endpoints_mm(le_bar_frame_m)
    ln_start_mm, ln_end_mm = _ln_endpoints_mm(ln_bar_frame_m)

    female_ocf_mm = fk_female(
        le_start_mm, le_end_mm,
        0.5 * LE_BAR_LENGTH + values["fjp_joint"],
        values["fjr_joint"],
        config,
    )
    male_ocf_mm = fk_male(
        ln_start_mm, ln_end_mm,
        0.5 * LN_BAR_LENGTH - values["mjp_joint"],
        values["mjr_joint"],
        config,
    )
    return _mm_frame_to_m(female_ocf_mm), _mm_frame_to_m(male_ocf_mm)
```

Note: `fk_female` call no longer has `jjr` parameter (removed in step 2a).

### 4d. Add simplified `_build_status_line()`

```python
def _build_status_line(freeze_flags, female_ocf_m, male_ocf_m, jjr_rad):
    pos_err_mm = float(np.linalg.norm(female_ocf_m[:3, 3] - male_ocf_m[:3, 3])) / SCALE
    T_rel = np.linalg.inv(female_ocf_m) @ male_ocf_m
    trans_residual_mm = float(np.linalg.norm(T_rel[:3, 3])) / SCALE
    extracted_jjr_rad = math.atan2(T_rel[1, 0], T_rel[0, 0])
    jjr_err_deg = math.degrees(extracted_jjr_rad - jjr_rad)
    return (
        f"\rFreeze F:{int(freeze_flags['female'])} M:{int(freeze_flags['male'])} | "
        f"pos_err {pos_err_mm:6.3f} mm | "
        f"jjr extracted {math.degrees(extracted_jjr_rad):7.2f} vs slider {math.degrees(jjr_rad):7.2f} "
        f"(diff {jjr_err_deg:6.3f}) deg | "
        f"T_rel trans {trans_residual_mm:6.3f} mm"
    )
```

### 4e. Simplify `main()` loop

After driving URDF joints, replace the ghost/comparison/status block with:

```python
            le_bar_frame = _get_link_frame(robot_id, link_map, "le_bar")
            ln_bar_frame = _get_link_frame(robot_id, link_map, "ln_bar")

            female_ocf_m, male_ocf_m = _compute_fk_crosscheck(
                le_bar_frame, ln_bar_frame, effective_values,
            )

            _draw_pose(female_ocf_m, line_ids, "female", GHOST_VISUAL_OFFSET)
            _draw_pose(male_ocf_m, line_ids, "male", GHOST_VISUAL_OFFSET)

            status_line = _build_status_line(
                freeze_flags, female_ocf_m, male_ocf_m,
                effective_values["jjr_joint"],
            )
```

Remove `ghost_bodies`, `analytic_frames`, `male_from_male_frame`, `pybullet_frames` (8-frame dict). Keep `line_ids`.

---

## 5. `scripts/t2_joint_placement.py`

### 5a. Update `fk_female` calls

The import `from core.kinematics import ...` stays. But any call to `fk_female` that passed `jjr` must drop that argument. Search for `fk_female` calls — currently only used indirectly via `optimize_joint_placement`. No direct fk_female calls in this file, so **no call-site changes needed here**.

### 5b. Review `_bake_debug_ocf_frames` (line 81-90)

`result["female_frame"]` is now the pure female OCF (no jjr applied). `result["male_frame"]` is the pure male OCF. Both are connector body centers. The debug frames will now show where the connectors actually sit. **No code change needed** — the semantics are actually more intuitive now.

### 5c. Review `_placeholder_visual_frame` (line 136-153)

This shifts the visual frame by `0.5 * radial_sign * radial_offset * z_axis` (line 152). With the new model, the frame origin IS at the connector body center, so this half-offset shift may no longer be correct. **Review and test visually in Rhino.** If blocks appear misaligned, adjust or remove the z-axis offset on line 152.

### 5d. Update `place_joint_blocks` print output (line 270-275)

No structural change needed — it prints DOF values which are still valid.

---

## 6. `tests/test_kinematics.py`

### 6a. Update `fk_female` calls — remove `jjr` arg

All `fk_female` test calls currently pass `jjr=0.0` as the 5th positional arg. Remove it:

```python
# OLD: fk_female(le_start, le_end, 50.0, 0.0, 0.0, config)
# NEW: fk_female(le_start, le_end, 50.0, 0.0, config)
```

Affected tests:
- `test_fk_female_origin_on_correct_side` (line 95)
- `test_fk_female_x_axis_along_bar` (line 130)
- `test_fjr_rotates_radial_direction` (lines 146-147)

### 6b. Update `test_perpendicular_bars_at_contact_distance` (line 170)

Replace the coincidence assertion:

```python
# OLD: assert female_frame[:3,3] ≈ male_frame[:3,3]
# NEW: verify screw hole chain connects them
from core.kinematics import female_to_male_chain
predicted_male = female_to_male_chain(result["female_frame"], result["jjr"], config)
np.testing.assert_allclose(predicted_male[:3, 3], result["male_frame"][:3, 3], atol=TOL_POSITION)
```

### 6c. Add new test for `female_to_male_chain`

```python
class TestFemaleToMaleChain:
    def test_identity_at_jjr_zero(self):
        """With jjr=0, the chain should produce a deterministic offset from female OCF."""
        female_ocf = np.eye(4, dtype=float)
        result = female_to_male_chain(female_ocf, 0.0, config)
        # screw hole at (0, 0, -SCREW_HOLE_Z_OFFSET), then -y by MALE_LINK_Y_OFFSET
        expected_pos = np.array([0.0, -config.MALE_LINK_Y_OFFSET, -config.SCREW_HOLE_Z_OFFSET])
        np.testing.assert_allclose(result[:3, 3], expected_pos, atol=1e-10)

    def test_jjr_rotates_male_position(self):
        """With jjr=pi/2, the male offset rotates 90deg around z at the screw hole."""
        female_ocf = np.eye(4, dtype=float)
        result = female_to_male_chain(female_ocf, np.pi / 2.0, config)
        # After Rz(pi/2) at screw hole, local -y becomes local +x
        expected_pos = np.array([config.MALE_LINK_Y_OFFSET, 0.0, -config.SCREW_HOLE_Z_OFFSET])
        # Wait: Rz(pi/2) maps -y → +x? Let's check:
        # Rz(pi/2) @ [0, -d, 0] = [d*sin(pi/2), -d*cos(pi/2), 0] 
        #   Rz(pi/2) matrix: [[0,-1,0],[1,0,0],[0,0,1]]
        #   [0,-1,0]@[0,-d,0]=d, [1,0,0]@[0,-d,0]=0 → (d, 0, 0)
        # So expected = (MALE_LINK_Y_OFFSET, 0, -SCREW_HOLE_Z_OFFSET)
        expected_pos = np.array([config.MALE_LINK_Y_OFFSET, 0.0, -config.SCREW_HOLE_Z_OFFSET])
        np.testing.assert_allclose(result[:3, 3], expected_pos, atol=1e-10)
```

### 6d. Import `female_to_male_chain`

Add to the imports at top:
```python
from core.kinematics import (
    fk_female,
    fk_male,
    female_to_male_chain,
    frame_distance,
    optimize_joint_placement,
    perpendicular_to,
)
```

---

## 7. `tests/test_kinematics_explained.md`

Add a new section for `TestFemaleToMaleChain`:

```markdown
## `TestFemaleToMaleChain`

### `test_identity_at_jjr_zero`
Verifies that with jjr=0, `female_to_male_chain` produces the expected offset:
translate -z by SCREW_HOLE_Z_OFFSET, then -y by MALE_LINK_Y_OFFSET.

### `test_jjr_rotates_male_position`
Verifies that jjr=pi/2 rotates the male offset 90 degrees around z at the screw hole,
changing the -y offset into a +x offset.
```

Update the `test_perpendicular_bars_at_contact_distance` explanation:

```markdown
### `test_perpendicular_bars_at_contact_distance` (UPDATED)
...
Checks:
- Optimization residual is very small.
- `female_to_male_chain(female_frame, jjr, config)` origin coincides with `male_frame` origin.
  (Female and male OCFs no longer coincide directly — they are connected through the screw hole chain.)
```

---

## Files touched summary

| File | Changes |
|---|---|
| `scripts/core/config.py` | Add `SCREW_HOLE_Z_OFFSET`, `MALE_LINK_Y_OFFSET` |
| `scripts/core/kinematics.py` | Remove jjr from `fk_female`, add `female_to_male_chain`, update optimizer objective |
| `scripts/generate_urdf.py` | Add 2 links, replace 1 joint with 3, remove visual offsets |
| `scripts/pybullet_viewer.py` | Delete 12 functions, add 3 new, simplify main loop |
| `scripts/t2_joint_placement.py` | Review `_placeholder_visual_frame` z-offset (may need adjustment) |
| `tests/test_kinematics.py` | Remove jjr from fk_female calls, update optimizer test, add `TestFemaleToMaleChain` |
| `tests/test_kinematics_explained.md` | Add `TestFemaleToMaleChain` docs, update optimizer test docs |

## Verification

```bash
# 1. Regenerate URDF
cd scripts && python generate_urdf.py

# 2. Run tests
cd .. && pytest tests/test_kinematics.py -v

# 3. Visual check in PyBullet
cd scripts && python pybullet_viewer.py
# Verify: boxes don't overlap, RGB frames visible, status line shows errors

# 4. Test in Rhino (manual)
# Run t2_joint_placement.py, verify blocks placed correctly
```
