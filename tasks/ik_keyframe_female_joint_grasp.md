# Female-joint IK grasp in RSIKKeyframe — design note

> Status: planned, not implemented. Pick up later.

## Context

`scripts/rs_ik_keyframe.py` today assumes BOTH arms grasp male joints sharing the same `male_parent_bar`. For a vertical bar where one end is screwed into existing structure (per `tasks/support_assembly_keyframes.png`), the right grasp pair is **one male + one female on the same bar B_n** — both joints physically sit on B_n. The dual-arm tool already supports this physically; we need the IK keyframe script to support it.

## Which TF system to use — verdict

The two systems are orthogonal:

- `scripts/rs_export_grasp_tool0_tf.py` → writes `MALE_JOINT_OCF_TO_TOOL0[block][arm_side]` and `BAR_GRASP_TO_TOOL0[kind]` to `scripts/core/config_generated_ik.py`. **Consumed directly by IK** (`rs_ik_keyframe.py:_tool0_from_male` → `solve_dual_arm_ik` → `compas_fab`).
- `scripts/rs_define_robotic_tool.py` → writes `RoboticToolDef.M_tcp_from_block` to `scripts/core/robotic_tools.json`. **CAD-side only**: consumed by `scripts/core/rhino_tool_place.py` to insert tool blocks at male joints in the Rhino doc. `M_tcp_from_block` is never read by IK or `compas_fab`.

→ **Use `rs_export_grasp_tool0_tf.py`** for the new female grasp TF. Victor's `RoboticToolDef` is unrelated to IK.

## Geometry assumption (confirmed by user)

**α**: both picks (male + female) physically sit on the same bar B_n. Both blocks expose `parent_bar_id == B_n` user-text. How those joints got placed on B_n is OUT OF SCOPE — the IK script only consumes whatever is in the scene.

## Data-model facts (verified)

- Female-joint Rhino blocks already carry the user-text we need (`scripts/core/joint_placement.py:287–311`):
  - `joint_subtype = "Female"`
  - `parent_bar_id` = bar this joint physically sits on
  - `male_parent_bar`, `female_parent_bar` = pair-level metadata (set on both sides of a pair)
- Female block's `InstanceXform` is a well-defined OCF, accessible via `_block_instance_xform_mm(oid)` (`rs_ik_keyframe.py:112–117`), same path as male.

## Recommended changes

### 1. `scripts/rs_export_grasp_tool0_tf.py` — add Female mode

Mirror existing `_run_joint_mode` (lines 179–230). Top-level mode prompt becomes 3 options: `Joint` (male, kept for back-compat) / `FemaleJoint` / `Gripper`. Female mode prompts joint type (default `T20_Female`) + arm side, picks `(female_ocf_frame_group, tool0_frame_group)`, computes `tf = invert_transform(female_frame) @ tool0_frame`, writes into a new `FEMALE_JOINT_OCF_TO_TOOL0[block_name][arm_side]` dict in `scripts/core/config_generated_ik.py`. Preserve the other two dicts on partial reruns via the same `_load_existing_*` + `_write_generated_ik` pattern (lines 91–110, 113–131); add `_load_existing_female` + `_format_female_dict` parallel to the male pair.

### 2. `scripts/core/config.py` + `config_generated_ik.py`

Add `FEMALE_JOINT_OCF_TO_TOOL0` to the auto-generated module (header docstring updated to document the third dict). In `config.py`, add a sanitizer call parallel to `MALE_JOINT_OCF_TO_TOOL0` (`config.py:138–161`) so import-time validation catches malformed female entries.

### 3. `scripts/rs_ik_keyframe.py`

- `_pick_male_joint` (lines 187–202) → `_pick_joint`: filter accepts `joint_subtype in ("Male", "Female")`. Update prompts: `"Pick LEFT arm joint block (Male or Female)"`.
- `_tool0_from_male` (lines 533–563) → `_tool0_from_joint(oid, arm_side)`: read `joint_subtype` user-text, dispatch to `config.MALE_JOINT_OCF_TO_TOOL0` or `config.FEMALE_JOINT_OCF_TO_TOOL0`. Same fallback path (block-name → `f"{joint_type}_{joint_subtype}"`). Return `(tool0_world_mm, ocf_world_mm, subtype)` so `main()` can drive the approach-direction logic.
- `_resolve_target_bar` (lines 517–530): **switch** from `male_parent_bar` to `parent_bar_id`. Both picks must have the same `parent_bar_id` (= the bar each joint physically sits on). Error messages updated. The bar-registry lookup (`get_bar_seq_map()`) remains the same.
- Approach direction (`main()` lines 795–804): replace the current `z_avg = (ocf_left[:3,2] + ocf_right[:3,2]) / 2` with a subtype-aware version — flip the female pick's z-axis so both vectors point along the insertion direction:

  ```
  z_l = ocf_left[:3, 2]  if subtype_left  == "Male" else -ocf_left[:3, 2]
  z_r = ocf_right[:3, 2] if subtype_right == "Male" else -ocf_right[:3, 2]
  z_avg = (z_l + z_r) / 2.0
  approach_dir = -_unit(z_avg)
  ```

  Reduces to current behavior when both picks are male. Same `ValueError` guard for zero-length avg.
- Capture writer (`_save_capture`): record `joint_subtype_user_text` per arm so headless replay can pick the right dict. Header `source` already has the slot.

### 4. `tests/debug_ik_collisions.py` — small replay update

`_tool0_from_ocf` (lines 72–78) currently hardcodes the male table. Make it look at a `subtype` field on the capture's `left`/`right` block dicts (default `"Male"` for back-compat) and pick `MALE_JOINT_OCF_TO_TOOL0` or `FEMALE_JOINT_OCF_TO_TOOL0`. Also update the approach-phase z-flip in `--phase approach` (line 261) to use the same male/-female formula.

## Critical files

- `scripts/rs_export_grasp_tool0_tf.py` (add Female mode)
- `scripts/core/config_generated_ik.py` (auto-gen, new dict)
- `scripts/core/config.py` (sanitizer for the new dict)
- `scripts/rs_ik_keyframe.py` (joint picker + dispatcher + resolve-target-bar key + approach-axis formula + capture)
- `tests/debug_ik_collisions.py` (replay parity)

## Reuse, no new helpers

- `_block_instance_xform_mm`, `_solve_with_sampling`, `_pick_base_frame_on_walkable` reused as-is.
- `_run_joint_mode` and `_run_gripper_mode` patterns mirrored for `_run_female_joint_mode`.
- `resolve_frame_group`, `reconstruct_frame`, `invert_transform` reused.

## Verification

1. **Export TF** for one female joint type both arms: run RSExportGraspTool0TF in `FemaleJoint` mode for `T20_Female` left + right. Confirm `config_generated_ik.py` now has all three dicts populated.
2. **In Rhino**: place a vertical bar B_n carrying both a male joint and a female joint (one at each end). Run RSIKKeyframe, pick the male with the LEFT arm and the female with the RIGHT arm. Verify `_resolve_target_bar` accepts (both picks share `parent_bar_id = B_n`).
3. **IK solves** at the seed base with `check_collision=True`; final + approach poses preview correctly. Verify approach offset moves AWAY from B_n (the average of male_z and -female_z, then negated, should be unambiguous along the insertion axis).
4. **Sanity: two-male regression** — run RSIKKeyframe on an existing dual-male bar; confirm behavior unchanged (approach formula collapses to the old `(z_l + z_r) / 2`).
5. **Headless replay**: run `python tests/debug_ik_collisions.py tests/captures/<latest>.json --phase final --headless` using the Rhino py3.9 (`C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe` — pyenv 3.10 fails on `cryptography` PyO3). Confirm `[OK] IK PASSED with collision check`.

## Out of scope

- How a bar ends up carrying both a male and a female joint (joint-placement workflow). The user is expected to place these in the scene by other means; this plan only changes the IK consumer.
