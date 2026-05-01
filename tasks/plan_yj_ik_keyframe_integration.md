# Plan: Integrate `yj_functions/rs_ik_keyframe.py` with our tool-placement pipeline

Scope: refactor only [scripts/yj_functions/rs_ik_keyframe.py](scripts/yj_functions/rs_ik_keyframe.py).
`rs_ik_support_keyframe.py` left untouched this round.

## User decisions (locked)

1. **Arm-side detection**: hardcode by suffix on `tool_name` user-text — `*L` -> left, `*R` -> right (e.g. `AT3L`, `AT3R`).
2. **tool0 source**: world `InstanceXform` of the placed tool block IS tool0. Drop `MALE_JOINT_OCF_TO_TOOL0` lookup entirely.
3. **Tool collision mesh**: defer. Do **not** insert any collision tool mesh into the robot cell for now. (Future: extend `robotic_tools.json` with an `obj_filename` pointing to a separately-prepared collision OBJ in `asset/`; attach as a rigid body — NOT a `compas_fab` `ToolModel`. One-shot at robot-cell setup. Track in todo.)
4. **Approach Z**: use the tool block's local Z-axis (mean of left+right tool Z), not the male joint Z.
5. **Walkable Ground layer**: rename constant to `"Walkable Ground"`, add to `MANAGED_LAYERS`, full managed treatment (auto-create + evict strays on entry).
6. **Capture / debug code**: keep the existing `_save_capture` + failure-prompt machinery as-is.

## Pre-work (small, in `core/`)

### a) `core/config.py`
- Rename `WALKABLE_GROUND_LAYER = "WalkableGround"` -> `"Walkable Ground"`.
- Place it under `MANAGED_LAYER_ROOT`:
  `LAYER_WALKABLE_GROUND = MANAGED_LAYER_ROOT + LAYER_PATH_SEP + "Walkable Ground"`.
- Keep `WALKABLE_GROUND_LAYER` as alias = `LAYER_WALKABLE_GROUND` (back-compat for support script).
- Append `LAYER_WALKABLE_GROUND` to `MANAGED_LAYERS` tuple.
- Drop / leave-unused `LEFT_PINEAPPLE_BLOCK`, `RIGHT_PINEAPPLE_BLOCK`,
  `LEFT_PINEAPPLE_TOOL_MESH`, `RIGHT_PINEAPPLE_TOOL_MESH` (do not delete — support script may still ref; just stop importing in keyframe).

### b) `core/rhino_bar_registry.py::enforce_managed_layers`
- No code change required if it iterates `config.MANAGED_LAYERS` (it does, line 997). New layer is created automatically. Verify eviction policy — should be fine (Brep stays on the layer because user puts it there, eviction targets only stray *managed objects*; if it evicts ANY object, we need an exception. Confirm during implementation.)

### c) `core/robot_cell.py` (defer-compatible knob)
- `get_or_load_robot_cell` currently attaches `LEFT_PINEAPPLE_TOOL_MESH` / `RIGHT_PINEAPPLE_TOOL_MESH` as `ToolModel`s named `AssemblyLeftTool` / `AssemblyRightTool`.
- Add an opt-out: if both mesh paths are missing OR a new `config.IK_ATTACH_TOOL_MESHES = False` flag is set, skip attachment. Default `False` for the new flow.
- Keep `LEFT_TOOL_NAME` / `RIGHT_TOOL_NAME` constants — `solve_dual_arm_ik` references them. When tools aren't attached, `template_state.tool_states` simply omits them. Verify `solve_dual_arm_ik` tolerates missing tool entries (likely guards with `tool_states.get(...)`); if not, add a guard.
- This is the minimum to make IK run with no tool collision body.

## `yj_functions/rs_ik_keyframe.py` rewrite outline

Keep the file's overall scaffold (module reload, capture saving, base-pick + base-sampling, dual phase prompts, env collision, highlight, dynamic preview). Replace only the picking / tool0 derivation / pineapple-insertion stages.

### 1) Imports & constants
- Remove `_insert_pineapple`, `_insert_pineapples`, `PINEAPPLE_LAYER`, `PINEAPPLE_ROLE_KEY`, `_require_block_definition` calls for pineapple blocks.
- Imports stay otherwise.

### 2) New helper: pick a bar then resolve its two arm-tool block instances
Pseudocode:
```
def _pick_bar_and_arm_tools():
    bar_oid = pick_bar("Pick the Ln bar to be assembled (must carry exactly 2 male joints with L/R tools placed)")
    if bar_oid is None: return None
    bar_id = rs.GetUserText(bar_oid, BAR_ID_KEY)

    # all male joint blocks whose parent_bar_id == bar_id
    male_oids = [oid for oid in rs.ObjectsByLayer(config.LAYER_JOINT_MALE_INSTANCES) or []
                 if rs.GetUserText(oid, "parent_bar_id") == bar_id]
    if len(male_oids) != 2:
        msg("Bar %s has %d male joints; need exactly 2 (single-joint case TBD)." % (bar_id, len(male_oids)))
        return None

    # for each male joint, locate its tool by joint_id, classify L/R by tool_name suffix
    left = right = None
    for moid in male_oids:
        jid = rs.GetUserText(moid, "joint_id")
        toid = find_tool_for_joint(jid)         # core.rhino_tool_place.find_tool_for_joint
        if toid is None:
            msg("Joint %s has no tool placed. Run RSToolPlace." % jid); return None
        tname = rs.GetUserText(toid, "tool_name") or ""
        if tname.endswith("L"):
            if left is not None: msg("Two left tools on bar %s." % bar_id); return None
            left = (moid, toid)
        elif tname.endswith("R"):
            if right is not None: msg("Two right tools on bar %s." % bar_id); return None
            right = (moid, toid)
        else:
            msg("Tool '%s' on joint %s has no L/R suffix." % (tname, jid)); return None

    if left is None or right is None:
        msg("Bar %s missing one of (left, right) tools." % bar_id); return None
    return bar_id, bar_oid, left, right    # left/right = (male_oid, tool_oid)
```
- `BAR_ID_KEY` import: `from core.rhino_bar_registry import BAR_ID_KEY` (or whichever module exports it; currently `rs_sequence_edit.py` imports it).
- `find_tool_for_joint` is already in `core.rhino_tool_place`.

### 3) tool0 derivation (replaces `_tool0_from_male`)
```
def _tool0_from_tool_block(tool_oid):
    return _block_instance_xform_mm(tool_oid)   # already in mm
```
No `MALE_JOINT_OCF_TO_TOOL0` indirection. The two tool0 frames feed `solve_dual_arm_ik` exactly like before.

### 4) Approach offset
Replace the male-Z block:
```
tool_z_left  = tool0_left_final[:3, 2]
tool_z_right = tool0_right_final[:3, 2]
approach_dir = -_unit((tool_z_left + tool_z_right) / 2.0)
offset_pre   = approach_dir * float(config.LM_DISTANCE)
```
Note sign assumption: tool block's local +Z points from flange OUT toward the joint/bar (i.e. the insertion direction). `-Z = retreat`. Validate visually on first run; flip sign if the retreat is reversed.

### 5) Drop pineapple insert / cleanup blocks
- The "inspect pineapple preview" prompts go away. The placed tool block instances are already in the doc and visible — that IS the proxy.
- Remove `pineapple_ids = _insert_pineapples(...)` calls and the surrounding cleanup, but KEEP the `_ask_accept` checkpoints (gate flow on user OK after IK preview, same as today).
- Net: between bar-pick and base-pick, we can show one early `_ask_accept("Inspect placed tools at FINAL target...")` if we want a parity step, but since user already sees them placed, this prompt is optional. Keep it for symmetry; just drop the block insertion.

### 6) `_resolve_target_bar` removed
Bar id comes directly from step 2. Keep the existing `(target_bar_id, target_bar_oid)` return contract for downstream code paths (`env_collision.collect_built_geometry(target_bar_id, ...)`, capture writer, payload write on the bar's curve oid).

### 7) `main()` wiring (delta)
```
# before:
left_oid = _pick_male_joint(...); right_oid = _pick_male_joint(...)
target_bar_id, target_bar_oid = _resolve_target_bar(left_oid, right_oid)
tool0_left_final, ocf_left  = _tool0_from_male(left_oid, "left")
tool0_right_final, ocf_right = _tool0_from_male(right_oid, "right")

# after:
result = _pick_bar_and_arm_tools()
if result is None: return
target_bar_id, target_bar_oid, (male_left_oid, tool_left_oid), (male_right_oid, tool_right_oid) = result
tool0_left_final  = _tool0_from_tool_block(tool_left_oid)
tool0_right_final = _tool0_from_tool_block(tool_right_oid)
# `ocf_left` / `ocf_right` (formerly male-block OCFs) are kept for the
# capture writer payload — fill from male blocks via _block_instance_xform_mm
ocf_left  = _block_instance_xform_mm(male_left_oid)
ocf_right = _block_instance_xform_mm(male_right_oid)
```
Everything below (env collection, base pick, sampling, IK solve, ik_viz preview, accept, write `ik_assembly` user-text on `target_bar_oid`, capture) remains structurally identical.

### 8) Capture payload (`_save_capture` call)
- `left_block_name` / `right_block_name` should now be the TOOL block names (still useful for replay), not the male block names. Add `left_tool_block_name = rs.BlockInstanceName(tool_left_oid)` etc.
- Add a `left_tool_oid` / `right_tool_oid` pair to `source` so headless replay knows which world frames to use.
- Schema change is additive — keep old fields for back-compat. Bump `suffix=""` semantics unchanged.

### 9) Robot-cell-setup tool-mesh defer
- Either set `config.IK_ATTACH_TOOL_MESHES = False` in `config.py`, or pass an explicit `attach_tools=False` arg through `robot_cell.get_or_load_robot_cell`. Preferred: a config flag (single switch), default `False` in this branch.
- Smoke-test: with no tool meshes attached, `solve_dual_arm_ik(..., check_collision=True)` must not raise on missing tool. Add a `tool_states.get(name)` guard if needed.

## Out of scope / TODO follow-ups (record in `todos.md`)

- Add `obj_filename` field to `RoboticToolDef` + `robotic_tools.json`; render the OBJ for each tool's actual collision geometry.
- Wire those OBJs into `robot_cell` as **rigid bodies** attached to flanges (NOT `ToolModel`). Single-shot during cell setup.
- Single-male-joint case: when bar carries exactly 1 male joint (assigned to either L or R), allow keyframing one arm only, with the other arm fed a "park" config.
- Mirror these refactors into `rs_ik_support_keyframe.py` once the dual-arm flow is validated.

## Validation steps

1. Open a doc with a bar carrying two male joints, both with L/R tools placed via `RSJointEdit`.
2. Run new `RSIKKeyframe`. Pick the bar.
3. Verify it identifies the L and R tools, derives tool0 at each tool block's origin, and solves IK.
4. Confirm the `Walkable Ground` layer was auto-created on entry and lives under `MANAGED Scaffolding::Walkable Ground`.
5. Add a Brep there, complete the workflow, accept, verify `ik_assembly` user-text on the bar curve.
6. Re-run on the same bar — should idempotently overwrite.
