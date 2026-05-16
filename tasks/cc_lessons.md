# Claude Code Lessons

Short notes on patterns we've hit before. Lead with the pattern; add **Why** and **How to apply**.

---

## Curve-constrained `GetPoint` should leave object snaps enabled

When a Rhino picker uses `Rhino.Input.Custom.GetPoint().Constrain(curve, False)` to slide along a bar, do NOT also call `gp.PermitObjectSnap(False)`. That blocks osnaps to *other* geometry, so the user can't snap to e.g. another bar's endpoint or a joint center for precise alignment.

**Why:** `Constrain(curve)` already projects every cursor sample (including snapped ones from other geometry) back onto the constrained curve. Disabling object snaps just removes the precision-pick affordance with no benefit. `rs.GetPointOnCurve` (used by `rs_bar_brace.py`) leaves osnaps on by default, which is why brace point-picks felt "snappier" than ground-place ones.

**How to apply:** Default to `gp.PermitObjectSnap(True)` (or omit the call entirely). Only disable osnaps when there is a concrete reason (e.g. the constraint surface is itself made of snap-magnet geometry that would cause feedback).

---

## Standalone "refresh" entry points must call `repair_on_entry`, not `update_all_previews` alone

`rs_update_preview.py` originally called only `update_all_previews(bar_radius)`. That helper iterates registered bars and rebuilds stale tubes per `_tube_geometry_matches`, but it does NOT touch ORPHAN tubes whose `tube_axis_id` belongs to a different (still-registered) bar -- which is exactly what user copy-paste of a bar+tube produces. Orphan deletion lives in `_enforce_tube_layer`, only invoked from `enforce_managed_layers` -> `repair_on_entry`. Skipping that pass leaves duplicate tubes overlapping the new bar's tube forever.

**How to apply:** Every entry-point script that asks the user to "fix the previews" must call `repair_on_entry(bar_radius, caller=...)` first. `update_all_previews` is the post-repair canonical pass and may be re-invoked separately for diagnostics (`verbose=True`).

Also: `update_all_previews` previously returned `len(bars)`, so callers that printed "Updated N bar previews" lied even when nothing changed. Make it return `created + regenerated` (excluding `reused`) and have `ensure_bar_preview` return `(baked_ids, status)` with status in `{"reused", "regenerated", "created"}` so the count is meaningful.

---

## Post-IK prompts should loop on replan in both success and failure paths

When an IK preview is valid or an IK solve fails, offer explicit replan actions like `RetrySameBase`, `RetryNewBase`, and `GiveUp` instead of any branch that immediately exits the command. Add `Accept` only on the success path.

**Why:** In the Rhino IK workflow, both a valid preview and a failed solve can still be one sampling pass away from success. Forcing exit on failure is the same UX bug as forcing exit on reject: it makes the user rerun the whole command and re-enter context they already chose. `RetrySameBase` should reuse the current base directly; `RetryNewBase` should jump back to the walkable-ground pick without re-asking unrelated options.

**How to apply:** Keep the solve/preview/save block inside a tight local loop. On success, branch to `accept`, `retry_same_base`, `retry_new_base`, or `give_up`. On failure, branch to `retry_same_base`, `retry_new_base`, or `give_up`. Reuse the existing base-save and base-pick helpers rather than introducing a second workflow.

When wiring `RetrySameBase`, do NOT clear the walkable `brep_id` unless you intentionally want seed-only retry. Preserve the current brep when available, and if the current path came from a reused saved base with no cached brep, recover the nearest walkable brep from `WALKABLE_GROUND_LAYER` so `_solve_with_sampling(...)` still gets its fallback samples.

---

## Edit live scripts, not the `yj_functions/` snapshots

`scripts/yj_functions/` holds backup copies of YJ-contributed scripts for reference only. The user iterates on `scripts/rs_*.py` directly. Confirm the target file before editing if both names exist.

## Validation/picking loops: re-prompt instead of aborting on a bad pick

When a user picks a wrong-shape input (e.g. a bar that doesn't have the required L+R tool layout), `print` a helpful diagnostic and `continue` the pick loop. Only `return` from `main` on Esc/explicit cancel. Keeps the user in flow without re-running the command.

## Bar-pick filters: build a factory when you need to mask out specific bar IDs

When a `Rhino.Input.Custom.GetObject` workflow needs to forbid the user from picking certain bars (e.g. an active bar can't depend on itself for support), use `core.rhino_bar_pick.make_bar_or_tube_filter(exclude_bar_ids=[...])` instead of writing an inline lambda. It composes on top of the canonical `bar_or_tube_filter` and resolves both centerline curves and tube-preview Breps to their bar IDs.

**Why:** Without a shared factory, every command would re-implement the curve-vs-tube branch and the bar-id lookup, and would silently break when the tube-preview path is used (the `BAR_ID_KEY` UserText lives on the curve, not the tube — tubes carry `TUBE_BAR_ID_KEY`). The factory hides that detail and returns the bare `bar_or_tube_filter` when the exclusion set is empty so we don't pay closure overhead.

**How to apply:** `go.SetCustomGeometryFilter(make_bar_or_tube_filter(exclude_bar_ids=[active_bar_id]))`. For multi-pick prompts use `go.GetMultiple(0, 0)` so Enter-with-no-selection returns `GetResult.Nothing` (treat as empty list) and Esc returns `GetResult.Cancel` (treat as no-op).

---

## PyBullet verbose flag inside Rhino 8 ScriptEditor

Always start `PyBulletClient` with `verbose=True` when the script runs inside Rhino's ScriptEditor.

**Why:** `compas_fab.backends.pybullet.utils.redirect_stdout(enabled=not verbose)` calls `os.fdopen(os.dup(fd), "w")` on stdout. Rhino's ScriptEditor stdout is not backed by a real OS file descriptor, so `os.dup(fd)` raises. Setting `verbose=True` disables the redirect path entirely. The GH prototype already used `verbose=True` — the Rhino-side port should match.

**How to apply:** Any new Rhino script that starts a PyBullet client through compas_fab must pass `verbose=True`. In this repo, default is set inside `scripts/core/robot_cell.py::start_pb_client`. Do not expose a "Quiet" toggle that would flip it unless you handle the stdout fd issue upstream.

---

## Joint OCF → tool0 dispatch key must be the full block name

Use the block-definition name (e.g. `"T20_Male"`) as the dispatch key for `MALE_JOINT_OCF_TO_TOOL0`, not the `joint_type` user-text (`"T20"`).

**Why:** `rs_joint_place.py` assigns `joint_type = block_name.split("_", 1)[0]` and `joint_subtype = block_name.split("_", 1)[1]`. The full block name uniquely identifies the geometric variant (Male vs Female, different families). Dispatching by just `joint_type` collides when we later add a female grasp TF.

**How to apply:** In Rhino scripts, read the dispatch key via `rs.BlockInstanceName(oid)`. Fall back to `f"{joint_type}_{joint_subtype}"` when the block name has been renamed. The `RSExportGraspTool0TF` prompt defaults to `"T20_Male"` to keep the two ends consistent.

---

## RobotCell holds only ONE actuated robot — wrap a second robot as a `ToolModel`

`compas_fab.RobotCell` has a singular `robot_model` attribute. To include another articulated robot in collision check, wrap it as a `ToolModel` via `ToolModel.from_robot_model(model.copy(), Frame.worldXY())` (compas_robots ≥ 0.6.0).

**Why:** When the support-IK script runs, the dual-arm assembly robot is "frozen" at its captured IK pose but must still be in collision with the support arm. Loading two URDFs as actuated robots in one `RobotCell` is not supported. The pattern came from `support_materials/gh_keyframe_demos/python/GH_create_robot_cell.py:16-17`. The dual-arm tool's pose is set via `state.tool_states['DualArm'].frame` (world-frame placement of the husky base) and its internal joint configuration via `state.tool_states['DualArm'].configuration` — both honored by PyBullet collision check.

**How to apply:** When you need a static-but-articulated obstacle in `compas_fab` PyBullet IK:
1. Add `cell.tool_models[NAME] = ToolModel.from_robot_model(other_robot_model.copy(), Frame.worldXY())`.
2. Leave `state.tool_states[NAME].attached_to_group = None` so the tool is treated as static (NOT attached to the actuated robot's planning group).
3. Set `state.tool_states[NAME].frame` (mm-converted-to-m world frame) and `state.tool_states[NAME].configuration` (a `compas_robots.Configuration` matching the tool's `zero_configuration().joint_names`).
4. Call `planner.set_robot_cell(cell)` once, then `planner.set_robot_cell_state(state)` per IK call to update the obstacle's pose without rebuilding collision geometry.

When the same Rhino session must run both the dual-arm and the support cell, use **separate sticky keys for the cells** but **share the single PB client + planner**. A `_STICKY_CURRENT_CELL_KIND` flag tracks which cell is loaded; `set_cell_state` swaps cells via `planner.set_robot_cell(...)` only when the kind changes. See `scripts/core/robot_cell_support.py`.

---

## Cursor-tracking preview = `DisplayConduit` + `GetPoint.OnDynamicDraw`

For Rhino-Orient3Pt-style live previews (a half-transparent mesh follows the cursor while the user picks a point), subclass `Rhino.Display.DisplayConduit` with a mutable transform, and feed the transform from a `Rhino.Input.Custom.GetPoint` subclass via `OnDynamicDraw`.

**Why:** `rs.GetPoint` alone has no preview hook. Baking + transforming + deleting per mouse-tick is too slow and clutters the doc. `DisplayConduit.PostDrawObjects` runs in the viewport's draw pipeline (no doc baking) and `GetPoint.OnDynamicDraw(e)` fires every cursor move with `e.CurrentPoint` in world coords. `e.Display.PushModelTransform / DrawMeshShaded(mesh, DisplayMaterial)` is the canonical primitive; `DisplayMaterial.Transparency` is the **complement** of alpha (1.0 = invisible, 0.0 = opaque), which surprised us once.

**How to apply:** Use `core.dynamic_preview.mesh_preview(meshes_at_identity, alpha=0.5)` as a context manager and pass a `TrackingGetPoint(conduit, frame_from_cursor=...)` whose callback returns a `Rhino.Geometry.Transform` for the cursor frame. To preview the support robot, bake meshes once via `ik_viz.show_state(state, robot_model=...)` at identity base, copy the GUIDs out via `rs.coercemesh`, then `ik_viz.clear_scene()` to delete the doc bake — the captured `Mesh` values are then in base-local coordinates, ready to feed `mesh_preview` under different world-from-base candidate transforms. See `scripts/rs_ik_support_keyframe.py::_bake_support_robot_meshes_at_zero`.

---

## `ik_viz.show_state(state, *, robot_model=...)` accepts an explicit model

When more than one `RobotCell` is in play, pass `robot_model` to `ik_viz.show_state` so the cached compas SceneObject keys on the right model.

**Why:** Default behavior (`get_or_load_robot_cell().robot_model`) is hardcoded to the dual-arm cell. The support workflow needs to draw the single-arm husky too. The cached `_STICKY_SCENE_OBJECT` already keys by `cached.item is robot_model` so swapping models invalidates correctly — the missing piece was the parameter itself.

**How to apply:** Always pass `robot_model=cell.robot_model` from new scripts that load a non-default cell. Existing dual-arm callers are unchanged because `robot_model=None` falls back to `get_or_load_robot_cell()`.

---

## Rhino toolbar merges: never reuse existing macro GUIDs for new buttons

When adding toolbar buttons in `scaffolding_toolbar.rui`, treat existing `macro_item guid` values as immutable. New buttons must use brand-new GUID suffixes in both `<tool_bar_item ...>` and `<macro_item ...>`.

**Why:** Reusing an existing macro GUID silently replaces the old command binding. We hit this when incoming IK buttons reused `...000000000012` and setup IDs `...00000000000b/00c`, which made legacy buttons disappear (`RSBarSubfloor`, `RSDefineJointPair`, `RSDefineRoboticTool`).

**How to apply:**
1. Before editing, diff against the pre-merge RUI and keep all legacy `<macro_item guid=...>` blocks unchanged.
2. Add new buttons with unused GUIDs only (e.g. next free numeric suffix).
3. Ensure each new `<tool_bar_item>` references the matching new `<macro_item>` ID.
4. Run a duplicate check for `macro_item guid` entries before commit.

---

## Pick bar centerlines via `core.rhino_bar_pick.pick_bar`, never by layer-path filter

When an RS* script needs the user to pick a bar centerline curve, call `pick_bar(prompt)` from `core.rhino_bar_pick`. Do NOT roll a `GetObject` filter that compares `Attributes.LayerIndex` -> `FullPath` against `config.LAYER_BAR_CENTERLINES`.

**Why:** The canonical filter keys on the curve's `bar_type` user-text tag (set by the bar registry) and additionally accepts the surrounding tube preview (resolving back to the curve). A layer-path filter silently rejects every pick when the bar curve sits on a slightly different path (e.g. the user moved it, or the parent layer was renamed) — symptom: the prompt appears in the Rhino command line but no curve is selectable. Hit on the first click-test of `RSIKSupportKeyframe`.

**How to apply:** `from core.rhino_bar_pick import pick_bar` and `oid = pick_bar(prompt)`. Returns the bar curve oid (never the tube), or `None` on cancel.

---

## Direction picks: constrain GetPoint to a plane + DrawLineFromPoint, like `_Rotate`

When an interactive pick defines a *direction* relative to a fixed pivot (e.g. heading rotation about a bar axis), constrain the `GetPoint` to a plane through the pivot perpendicular to the rotation axis, disable object snap, and turn on `DrawLineFromPoint(pivot, True)` for a rubber-band line.

**Why:** Without constraint, osnap pulls the cursor onto nearby curves/blocks (the bar centerline, gripper preview, dual-arm bake), making it hard to pick a clean direction. Confining the cursor to the rotation plane makes it slide freely on a 2D plane and removes any 3D snap noise; the rubber-band line gives the same affordance as Rhino's `_Rotate` so the user can see the angle as they move.

**How to apply (Rhino.Input.Custom.GetPoint):**
1. `picker.SetBasePoint(pivot, True)`
2. `picker.DrawLineFromPoint(pivot, True)` — rubber-band line
3. `plane = Rhino.Geometry.Plane(pivot, axis_unit_vec)`; `picker.Constrain(plane, False)`
4. `picker.PermitObjectSnap(False)`

See `scripts/rs_ik_support_keyframe.py::_pick_grasp_frame_on_bar` phase B.

---

## Detach `ik_viz.show_state` GUIDs to keep multiple robots on screen at once

To display TWO robots in the doc simultaneously (e.g. dual-arm at the captured assembly pose AS COLLISION CONTEXT while the user picks a grasp for the support arm), bake the first robot via `ik_viz.show_state(state, robot_model=...)`, then immediately steal the GUIDs out of the tracking sticky:
```python
guids = list(sc.sticky.get(ik_viz._STICKY_DRAWN_IDS, []) or [])
sc.sticky[ik_viz._STICKY_DRAWN_IDS] = []
sc.sticky.pop(ik_viz._STICKY_SCENE_OBJECT, None)
```
Track those guids in your own list and clean them up manually. Subsequent `ik_viz.show_state` / `clear_scene` calls won't touch them.

**Why:** `ik_viz.show_state` calls `_delete_tracked_ids()` first thing, which deletes whatever is in `_STICKY_DRAWN_IDS`. Without detaching, the second `show_state` (e.g. for the support robot's final IK pose) wipes the dual-arm context the user just wanted to see. Popping `_STICKY_SCENE_OBJECT` is also needed so the next `show_state` rebuilds a fresh SceneObject for the new model.

**How to apply:** See `scripts/rs_ik_support_keyframe.py::_bake_dual_arm_at_captured_pose`. Caller tracks GUIDs in its own list and runs `_cleanup_ids(da_oids)` in the workflow's `finally`.

---

## Every IK solver must guarantee the planner has the matching cell loaded

When two `RobotCell`s share a single PyBullet planner (dual-arm assembly + support), each `solve_*_ik` entry-point must verify the planner's currently-loaded cell BEFORE calling `planner.inverse_kinematics(state=...)`, and swap via `planner.set_robot_cell(matching_cell)` if not.

**Why:** compas_fab compares the state's `tool_states` dict against the planner's `robot_cell.tool_models` and raises *"The tools in the cell state do not match the tools in the robot cell. Mismatch: {...}"* on any difference. `start_pb_client` only loads the dual-arm cell. So the first `RSIKSupportKeyframe` run after `RSPBStart` blows up with `Mismatch: {'SupportGripper', 'DualArm', 'AssemblyLeftTool', 'AssemblyRightTool'}` — state has SupportGripper/DualArm, cell has AssemblyLeftTool/AssemblyRightTool. Symmetric breakage if the user runs RSIKSupportKeyframe then RSIKKeyframe.

It is NOT sufficient that `set_cell_state` knows how to swap — it's only called AFTER IK, to push the final result. The IK solver itself runs many times (sampling) and is the one that actually needs the cell loaded.

**How to apply:** Each module exports a `_ensure_<kind>_cell_loaded(planner)` helper:
```python
def _ensure_support_cell_loaded(planner):
    if _STICKY.get(_STICKY_CURRENT_CELL_KIND) != "support":
        planner.set_robot_cell(get_or_load_support_cell())
        _STICKY[_STICKY_CURRENT_CELL_KIND] = "support"
```
Call it as the first line of `solve_<kind>_ik`. See `scripts/core/robot_cell.py::_ensure_dual_arm_cell_loaded` and `scripts/core/robot_cell_support.py::_ensure_support_cell_loaded`. Verify with `tests/debug_ik_support_collisions.py <capture>.json` — it prints the cell-kind sticky right before the IK call.

---

## ACM whitelisting: only allow the contact that the design INTENDS

Whitelist a collision pair (`touch_bodies` / `touch_links`) only when the contact is an **intentional part of the kinematic design** — e.g. the tool engaging the joint at assembly is a deliberate mode-change moment, the gripper is supposed to be there. Do NOT whitelist contacts that happen to occur in some pose but aren't part of the design intent — e.g. wrist-vs-joint clipping. That's a real bad pose worth flagging.

**Why:** `touch_*` is "this contact is by design", not "ignore collisions in this region". For the active male joint:
- joint ↔ matched-arm **tool**: WHITELIST — the gripper is gripping the joint, that's the whole point.
- joint ↔ matched-arm **wrist**: NOT whitelisted — the wrist isn't meant to touch the joint; if it does, IK has produced a bad pose and we want to know.

Historical note (now obsolete): an earlier pineapple-style proxy mesh extended backward into the arm's `wrist_1/2/3_link`, so `_ARM_TOOL_TOUCH_LINKS` whitelisted those wrist links as a *modeling artifact*. The current per-tool collision OBJs (in `robotic_tools.json`) are authored to start at tool0 and stay forward of the wrist, so that whitelist is gone (`_ARM_TOOL_TOUCH_LINKS = {"left": [], "right": []}` as of commit `cf2eeb2`). Any tool↔wrist contact now is a real bad pose we want IK to reject.

**How to apply:** When exposing ACM seams for future use:
- Whitelist tool body (`touch_bodies = [tool_name]`) — the gripper is at the joint by design.
- Do NOT pre-add the wrist link to `touch_links`. If a real engagement pose later needs it, that means the tool collision mesh has been re-authored to overlap the wrist again — fix the mesh, not the ACM.
- Document each `touch_*` entry inline with WHY it is design-intent (which kinematic step it covers).

---

## Coloring helpers: don't repurpose holistic display modes for narrow IK overlays

When an IK / debugging button needs to "color the things this pass considers" (e.g. green env-collision bars), write a focused helper (`highlight_env_for_ik(active_bar_id)`) instead of reusing the holistic sequencing display (`show_sequence_colors`).

**Why:** The holistic display answers "what's the assembly state?" — it paints active blue, unbuilt grey, repaints joints, hides/shows tools. The narrow overlay answers "what is this IK pass treating as static obstacles?" — only the green-built-bar slice. Reusing the holistic helper repaints unrelated objects (active bar turns blue mid-IK-prompt, unbuilt bars hide), which is misleading. Each owner (sequencing vs IK) will iterate independently — a Victor sequencing change shouldn't ripple into the IK button's visuals.

**How to apply:** When the user says "color X green for purpose Y", and there's an existing `show_*_colors` that incidentally already paints X green — DO NOT reuse it. Write a focused helper that touches only the IK-relevant subset (built bars + their joints) and resets only what it changed. Document the boundary in the helper's docstring: "Mutates only built bars + joints; leaves active/unbuilt untouched so the holistic sequencing colors remain authoritative outside this preview."

---

## Distinct URDF `<robot name>` per husky variant — and the SRDF must match

Each husky URDF in `asset/husky_urdf/` was named `<robot name="husky">` upstream, which makes PyBullet bodies indistinguishable when more than one husky lives in the same world (assembly cell's actuated dual-arm + dual-arm wrapped as a `DualArm` tool obstacle on the support cell). Fix at the source: rewrite each URDF's `<robot name>` to a unique identifier, AND update the matching SRDF's `<robot name>` so compas_fab's `RobotSemantics.from_srdf_file(...)` doesn't reject the load.

**Why:** PyBullet's `getBodyInfo(uid)` returns the URDF's robot name verbatim, and compas_fab's PyBullet backend round-trips the URDF text via `loadURDF`, so body labels in the GUI only honor the URDF's `<robot name>` attribute. The SRDF carries its own `<robot name>` and compas's semantics loader compares the two; mismatched names raise. Edits in the submodule are kept in `asset/husky_urdf/`'s working tree (the submodule is detached HEAD locally — investigate before pushing upstream).

**How to apply:** Pair every URDF rename with the matching SRDF rename. Currently:
- `mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf` + `config/dual_arm_husky.srdf` → `dual-arm_husky_Cindy`
- `mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf` + `config/husky.srdf` → `single-arm_husky_Alice`

---

## "IK can't find a solution" — first replay headless before changing code

When the user reports `RSIKKeyframe`/`RSIKSupportKeyframe` failing on a target they expect to work, the failure capture in `tests/captures/*_ik_fail*.json` is enough to verify whether the scene is actually solvable. Run `python tests/debug_ik_collisions.py tests/captures/<file>.json --phase final --headless` from the repo root using `C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe` (NOT pyenv 3.10 — `cryptography` PyO3 needs cpython 3.9 from the Rhino site-env). If that prints `[OK] IK PASSED with collision check`, the seed is geometrically reachable and IK is healthy.

**Why:** Failure captures save the *seed* base frame plus the input `ocf_world_mm` per arm — exactly the inputs `_solve_with_sampling` consumed. If headless replay solves on attempt 0 and a `for _ in range(15)` loop is 15/15, the IK solver itself is not the bug. We've seen the dual-arm IK pass at this rate even after a deliberate support-cell→dual-arm swap, so the swap path is not contaminating state. That leaves the user's Rhino session as the suspect — most likely a stale PyBullet world from earlier RSIK runs that we can't reproduce in a fresh process.

**How to apply:** Before changing any IK code, prove the scene is solvable headless. If it is, instruct the user to `RSPBStop` + `RSPBStart` and re-run; the failure usually clears. Bumping `IK_BASE_SAMPLE_MAX_ITER` is not a real fix — when headless solves on attempt 0 every time, more samples won't change a Rhino-only failure. Only investigate further (Rhino-side `print(planner.client.tools_puids, ...)` instrumentation) if the failure persists across a fresh `RSPBStart`.

Caveat: `mt_husky_moveit_config/config/husky.srdf` is shared by `husky_ur5_e_no_base_joint_Belle_Calibrated.urdf` (currently unused). When Belle is wired up, give her a dedicated SRDF copy named `single-arm_husky_Belle` rather than reusing Alice's.

---

## Two `compas_fab.PyBulletClient` instances coexist cleanly — no swap needed

When you need to host more than one robot cell in PyBullet (e.g. a dual-arm assembly cell + a single-arm support cell), you can run TWO `PyBulletClient` instances side by side in the same Python process. compas_fab's per-client scoping is correct (every `pybullet.*` call passes `physicsClientId=self.client_id`), so state isolation is automatic. Constraints: only ONE `connection_type="gui"` per process (PyBullet limit), and the `pybullet_planning` (pp) helpers — `pp.set_pose`, `pp.get_pose`, `pp.disconnect`, `pp.is_connected`, `pp.link_from_name`, `pp.get_link_pose` — bind to a module-global `CLIENT` and must be avoided (or bracketed with `pp.set_client(client.client_id)`).

**Why:** Verified empirically by `tests/debug_dual_client_isolation.py` on 2026-04-30: with two DIRECT clients, isolation is perfect (mutating one never leaks to the other) and 5000 iterations of alternating mutations + reads completed in 0.05ms mean / 0.06ms p95 with zero failures. Re-verified on 2026-05-01 with the `--three-way` mode (Cindy + Alice + Belle, all DIRECT): 18/18 isolation assertions pass, 2000-iter soak with 0 failures, mean 0.06ms / p95 0.09ms. The current "single planner + cell-swap" design's slow `planner.set_robot_cell(other_cell)` URDF reload was the only motivation for swapping — the multi-client model removes that cost entirely and scales linearly with the number of robots.

**How to apply:** The actual refactor (replace `_STICKY_CURRENT_CELL_KIND` + `_ensure_<kind>_cell_loaded` with two persistent planners) needs to fix the pp footguns first. Concrete sites + preferred fix recorded in `tests/debug_dual_client_isolation.py`'s `# === REFACTOR NOTES ===` footer block. compas_fab's `set_robot_cell_state` already pushes the base correctly via `client._set_base_frame` (client.py:702-704), so the redundant `pp.set_pose` calls in `core/robot_cell.py:469` and `core/robot_cell_support.py:247` can be deleted outright (not just rescoped).

Run the debug script under Rhino's bundled python — `C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe` — NOT pyenv py3.10 (PyO3 / cryptography in the Rhino site-env requires cpython 3.9).

---

## `pp.set_client` is BROKEN for most pybullet_planning helpers — patch every submodule's CLIENT or use raw pybullet

`pp.set_client(client_id)` only mutates `pybullet_planning.utils.shared_const.CLIENT`. But ~28 pp submodules (e.g. `pybullet_planning.interfaces.env_manager.pose_transformation`, `...interfaces.robots.joint`) do `from pybullet_planning.utils import CLIENT, ...` at import time, which creates a SEPARATE per-submodule binding. `pp.set_client` does not update those, so `pp.set_pose` / `pp.set_joint_position(s)` / `pp.get_link_pose` keep reading their stale local 0 and silently target the WRONG client.

**Why we care:** Verified empirically by `tests/debug_dual_client_isolation.py` — calling `pp.set_client(client_b.client_id)` then `pp.set_pose(client_b.robot_puid, target)` actually moved client A's body 0, not client B's. This is the silent-bug class that would break a multi-client refactor.

**How to apply:** In a multi-client process, you have two options:

1. **Raw pybullet** (preferred for production code): every call uses `pybullet.X(..., physicsClientId=client.client_id)`. Per-call scoping, no globals. This is what `tests/debug_dual_client_isolation.py::_push_state_safely` does.

2. **Patch every pp submodule's CLIENT** (only when pp ergonomics matter enough): walk `pybullet_planning` once at startup, find every submodule with a `CLIENT` int attribute, and on each client switch overwrite all of them. The test ships a working `pp_active_client` context manager that does exactly this (~28 modules patched, restored on exit). It works but depends on every relevant submodule already being imported at walk time — lazy imports inside individual helpers can slip past it.

The shipping `pp.set_client(...)` API is unsafe and should not be used as the primary scoping primitive in any new multi-client code.
---

## env_collision RigidBody cache: share the SAME RigidBody instance under multiple cell names

When many env entries (e.g. all placed T20_Female joints) share identical geometry, build the compas_fab.robots.RigidBody ONCE and reuse the same Python object as the value under every `robot_cell.rigid_body_models[name]` key.

**Why:** `compas_fab` PyBullet backend (`pybullet_set_robot_cell.py` -> `client._add_rigid_body`) iterates `rigid_body_models.items()` and creates a fresh PyBullet body per name regardless of whether the underlying `RigidBody` instance is shared. The slow path was `Rhino brep -> mesh -> mesh.join -> to_obj -> vhacd`; eliminating duplicate work per joint placement drops `ensure_env_registered` from ~40 s to a few hundred ms after the first cold load.

**How to apply:** Cache `RigidBody` in sticky keyed by geometry source (`block_name` for joint OBJ, `(bar_oid, length+radius signature)` for procedurally-built bars). In `register_env_in_robot_cell`, detect change with object identity (`existing is rb`) instead of comparing vertex/face counts. Always store the geometry in the body's LOCAL frame and put the world placement on `RigidBodyState.frame` so the same instance can be reused across positions.

## Procedural bar collision: 12-sided cylinder mesh, no length subdivision

For bar-shaped env obstacles, build the collision mesh in pure Python (compas `Mesh.from_vertices_and_faces`) as a 12-sided polygon ring extruded along `+Z` length L plus two cap fans. Place via `RigidBodyState.frame`.

**Why:** Going through Rhino `Cylinder.ToBrep` -> `Mesh.CreateFromBrep` -> per-vertex Python loop into compas `Mesh` is slow (Brep meshing alone is hundreds of ms per bar). 12 facets * 1 length segment = 12 quads + 2 fans -> tiny mesh; PyBullet handles the long thin triangles fine.

**How to apply:** Build the mesh in METERS in the local frame (origin at bar start, Z = bar axis). Cache key = bar oid + `(round(length_mm,3), round(radius_mm,3))`. Compute the world placement frame separately via `frame_from_axes` so identical-length bars share one `RigidBody` and only `frame_world_mm` differs across entries.

## Attached rigid body transform must use live PyBullet link frame (not model-only FK)

For `RigidBodyState.attached_to_link`, compute the link frame from PyBullet's live robot (`client._get_link_frame(...)`) before applying `attachment_frame`.

**Why:** `robot_model.forward_kinematics(...)` can ignore the runtime mobile-base world pose, so attached rigid bodies look offset/floating in GUI and collision checks desync from what the user sees.

**How to apply:** In `pybullet_set_robot_cell_state`, replace model FK in the attached-to-link branch with `pcf_link_id = client.robot_link_puids[link_name]` + `link_frame = client._get_link_frame(pcf_link_id, client.robot_puid)`, then compose with `attachment_frame` as before.


---

## Cached Rhino RobotCellObject �� bake once, delta-transform every frame

For interactive IK preview in Rhino, **stop deleting + re-adding** robot meshes per state. Use `compas_fab.rhino.scene.RobotCellObject` (vendored at `external/compas_fab/src/compas_fab/rhino/scene/`) with `BaseRobotCellObject.update(state)`: meshes are baked once on the first `update()`, then each subsequent call applies only the *delta* transform of every link via `Rhino.RhinoDoc.Objects.Transform(guid, T, deleteOriginal=True)`. The cache survives across UI frames and across the Esc/restart cycle (sticky-stored).

**Why:** The legacy `ik_viz.show_state` re-baked the entire dual-arm mesh per frame (~30 link meshes deleted + re-added through `RhinoSceneObject.draw_visual`). Pose cycling, bar switching, and IK retry all paid the bake cost. The compas-side machinery already supports incremental updates �� the missing piece on the Rhino side was a `RobotCellObject` that `_initial_draw`s once and lets the base class drive subsequent transforms.

**How to apply:**
1. Use `core.ik_viz.begin_session(robot_cell, mesh_mode=..., layer_key="Assembly") / update_state(state, layer_key="Assembly") / end_session()` for the new cached path.
2. Each cell goes on its own sub-layer under `config.LAYER_IK_CACHE` (`layer_key` arg) so dual-arm + support coexist and can be hidden independently via `set_layer_visible(layer_key, False)`.
3. End-of-session does NOT delete geometry �� `end_session` only hides the root layer; the next session reuses the same baked meshes from wherever they were last placed.
4. Use `discard_cache()` only when the underlying `RobotCell` has been rebuilt (PyBullet restart) or the user explicitly asks for a flush.
5. Legacy `show_state` / `clear_scene` still exist as back-compat shims for `yj_functions/` callers.

**Gotchas:**
- `BaseRobotModelObject.update(config, base_frame)` already folds `base_frame` into per-link transforms. **Do NOT** apply the base xform separately as a post-bake step �� that double-transforms the meshes.
- `sc.doc.Objects.Transform(guid, T, deleteOriginal=True)` always returns a NEW guid; cache the returned value or your sticky goes stale on the very first delta.
- Short-circuit identity transforms in `_transform`: stationary links would otherwise burn ~30 GUID allocations per frame.
- compas_fab's plugin system keys SceneObject registrations on `(item_type, context)`. Adding `context="Rhino"` for an item already registered for `"Grasshopper"` does NOT conflict. The vendored Rhino stack registers under `category="factories", requires=["Rhino"]`.
- `BaseRobotCellObject._initial_draw` is deferred to the first `update()` / `draw()` call, so `Scene().add(robot_cell, sceneobject_type=RobotCellObject, ...)` is cheap; the bake only happens on the first state push.
- `native_scale` interpretation: it is "meters per Rhino doc unit" (the base class scales `Frame.from_factors([1/native_scale]*3)`). For a doc in mm, pass `native_scale=0.001`. Compute via `doc_unit_scale_to_mm() / 1000.0`.


---

## compas plugin `register_scene_objects` MUST call `compas.scene.register`, not `SceneObject.register`

When writing a `@plugin(category="factories")` `register_scene_objects` function that maps an item type to a Rhino/GH/etc SceneObject, import the **module-level function** `from compas.scene import register` and call `register(ItemType, SceneObjectCls, context="Rhino")`. **Never** call `SceneObject.register(...)` �� there is no such classmethod on `compas.scene.SceneObject`.

**Why:** `SceneObject.register(...)` raises `AttributeError`. The compas `PluginManager` invokes plugin functions via `selector="collect_all"` and silently swallows per-plugin exceptions, so the registration never lands in `ITEM_SCENEOBJECT[context]`. Discovery prints from neighboring plugins (e.g. `compas_robots/rhino/scene` printing `"Rhino Robot Object registered."`) succeed and the failure looks invisible. The next call to `Scene().add(<item>)` then raises `SceneObjectNotRegisteredError: No scene object is registered for this data type: <ItemType> in this context: Rhino` even though the package's plugin module was correctly listed in `__all_plugins__`.

**How to apply:**
```python
from compas.scene import register  # NOT `from compas.scene import SceneObject`

@plugin(category="factories", requires=["Rhino"])
def register_scene_objects():
    register(RobotCell, RobotCellObject, context="Rhino")
```
Keep the print inside the registration function (not at module top) so the trace clearly shows the function actually ran. If a registration `print` is missing from the trace despite the package being in `compas_fab.__all_plugins__`, suspect a swallowed exception inside the plugin body first.


---

## Cached `ik_viz` migration: harvest meshes by layer, swap `clear_scene` for `end_session`

When migrating a caller that used the legacy `ik_viz.show_state` + `_STICKY_DRAWN_IDS` pattern to the new cached path, two follow-ups are required for correctness/perf:

1. **Anything that read `sc.sticky[ik_viz._STICKY_DRAWN_IDS]` to harvest baked link guids must switch to `rs.ObjectsByLayer(config.LAYER_IK_CACHE)`** (or the relevant sub-layer). The new `BaseRobotCellObject` keeps its guid cache inside the SceneObject instance, not in the legacy sticky. `_bake_robot_meshes_at_zero` in `rs_ik_keyframe.py` is the canonical example: bake -> `ObjectsByLayer` -> `discard_cache` to leave a clean slate before the dynamic-preview conduit takes over.

2. **Mid-loop `ik_viz.clear_scene()` calls should become `ik_viz.end_session()`.** `clear_scene` (== `discard_cache`) DELETES every guid on the cache layer AND drops the cached SceneObject, which forces a full re-bake on the next `show_state` (~30 link meshes). `end_session` only HIDES the cache layer; the very next `show_state` call re-shows the layer and delta-transforms the existing meshes in place. In `rs_ik_keyframe.py` the swap turned ~11 mid-IK-loop full re-bakes into pure visibility toggles + ~30 in-place transforms.

**Why:** Symptom of skipping (1): the dynamic mesh preview that follows the cursor over the walkable ground brep silently shows nothing �� the conduit gets an empty mesh list because the legacy sticky is empty. Symptom of skipping (2): IK retry loops feel sluggish because every `RetrySameBase` rebakes the entire dual-arm.

**How to apply:** Search for `ik_viz._STICKY_DRAWN_IDS` and `ik_viz.clear_scene` together; in the same caller, harvesting + nuking are usually paired. `discard_cache()` is still the right call when you genuinely want to flush (e.g. after harvesting meshes for a one-shot preview, or on PyBullet restart).


---

## ik_viz: pre-bake per-mesh-mode sub-sub-layers, toggle = visibility flip

When the user wants to swap visual<->collision mesh display without paying a re-bake cost:

- Cache key = `(id(cell), layer_key, mesh_mode)` (3-tuple), NOT (id(cell), layer_name).
- Layer hierarchy = `LAYER_IK_CACHE / <layer_key> / <MeshMode>` (mesh_mode is the LEAF). Each mode lives on its own sub-sub-layer so toggling is a single `rs.LayerVisible` call.
- `begin_session(..., mesh_modes=(visual, collision), active_mesh_mode=visual)` pre-builds BOTH cells.
- `update_state(state, mesh_modes=...)` defaults to updating EVERY cached mode for that `(cell, layer_key)`, so toggling later shows the latest pose immediately (~2x transform cost is fine for interactive viewer).
- `set_active_mesh_mode(layer_key, mesh_mode)` iterates the cache and flips `LayerVisible` per entry �� no rebake, no geometry mutation.
- For collision-highlight: expose `get_link_native_geometry(cell, layer_key, mesh_mode) -> {link_name: [guid, ...]}` and `get_tool_native_geometry` that read `BaseRobotModelObject._links_visual_mesh_native_geometry` / `_links_collision_mesh_native_geometry` (and per-tool via `BaseRobotCellObject._tool_scene_objects`).

## CollisionCheckError: use `.collision_pairs` attribute, not `args[1]`

`CollisionCheckError(message, collision_pairs)` calls `super().__init__(message)` so `exc.args == (message,)`. The pairs list is on the instance attribute `exc.collision_pairs`. Reading `exc.args[1]` IndexErrors. Each pair item is `(Link|ToolModel|RigidBody, Link|ToolModel|RigidBody)`; switch on `type(item).__name__` and read `.name` to map back to env_geom keys / cached link guids.

## CheckCollision in viewer: re-seed env from doc on every press

The viewer doesn't own env state across the session. Each `check_collision()` press should re-run `env_collision.collect_built_geometry(active_bar_id, get_bar_seq_map())` + `robot_cell.ensure_env_registered(rcell, env_geom, planner)` + `env_collision.build_env_state(state, env_geom)` + `set_cell_state`, so changes to built/unbuilt sequencing since the last render are reflected.


---

## Collision-check parity: viewer must mirror solver's RB attachments

IK solver in `rs_ik_keyframe` calls `robot_cell.attach_arm_tool_rigid_bodies` + `configure_arm_tool_rigid_body_states` to attach `AssemblyLeftArmToolBody` / `AssemblyRightArmToolBody` to the corresponding `*_ur_arm_tool0` link BEFORE running `check_collision`. Any standalone collision-check (e.g. ShowIK `CheckCollision` button) MUST replay the same attach steps, otherwise the per-arm tool RBs are either missing from the state or attached_to_link=None (sit at world origin) and CC.2/CC.3/CC.5 silently skip every pair involving them �� yielding the puzzling `IK fails on collision but viewer reports no collision`. Shared helper: `core/ik_collision_setup.prepare_assembly_collision_state(rcell, planner, state, bar_id)` does the full setup (arm-tool RBs + env_*) and is the single source of truth used by both the solver entry path and the viewer's check_collision.


---

## ShowIK viz: pre-attach all RBs before first cell bake; discard cache on bar switch

BaseRobotCellObject._initial_draw walks 
obot_cell.rigid_body_models exactly ONCE and caches a _rigid_body_scene_objects[id] dict. RBs added (or removed) AFTER the first draw will silently NOT appear (or worse: stale entries reference deleted models). When a viewer wants to show env_* + arm-tool RBs alongside the robot/tools, you must:
- Mutate 
cell.rigid_body_models (via ttach_arm_tool_rigid_bodies + 
egister_env_in_robot_cell) BEFORE calling Scene().add(robot_cell, draw_rigid_bodies=True, ...).
- Discard + rebake the cached scene object whenever the RB key-set changes (e.g. user navigates to a different active bar with a different built-before set).

In 
s_show_ik._PreviewSession: _render calls prepare_assembly_collision_state BEFORE ik_viz.begin_session, and set_active_bar calls ik_viz.discard_cache() when switching to a different bar.

## ik_viz.begin_session: hide doc layers as a list, not boolean flags

When the cached cell viz visually duplicates user-modeled doc geometry (tube previews, joint block instances, tool block instances), hide those doc layers for the duration of the session and restore prev visibility on end_session. Generic mechanism: hide_doc_layers: Iterable[str] = None kwarg (default = the assembly-overlap layer set) + _STICKY_HIDDEN_DOC_LAYERS dict {layer: prev_visible} that _restore_hidden_doc_layers() walks on session exit. Avoids per-layer boolean flags that don't scale.

---

## ShowIK viz: pre-attach all RBs before first cell bake; discard cache on bar switch

BaseRobotCellObject._initial_draw walks 
obot_cell.rigid_body_models exactly ONCE and caches a _rigid_body_scene_objects[id] dict. RBs added (or removed) AFTER the first draw will silently NOT appear (or worse: stale entries reference deleted models). When a viewer wants to show env_* + arm-tool RBs alongside the robot/tools, you must:
- Mutate 
cell.rigid_body_models (via ttach_arm_tool_rigid_bodies + 
egister_env_in_robot_cell) BEFORE calling Scene().add(robot_cell, draw_rigid_bodies=True, ...).
- Discard + rebake the cached scene object whenever the RB key-set changes (e.g. user navigates to a different active bar with a different built-before set).

In 
s_show_ik._PreviewSession: _render calls prepare_assembly_collision_state BEFORE ik_viz.begin_session, and set_active_bar calls ik_viz.discard_cache() when switching to a different bar.

## ik_viz.begin_session: hide doc layers as a list, not boolean flags

When the cached cell viz visually duplicates user-modeled doc geometry (tube previews, joint block instances, tool block instances), hide those doc layers for the duration of the session and restore prev visibility on end_session. Generic mechanism: hide_doc_layers: Iterable[str] = None kwarg (default = the assembly-overlap layer set) + _STICKY_HIDDEN_DOC_LAYERS dict {layer: prev_visible} that _restore_hidden_doc_layers() walks on session exit. Avoids per-layer boolean flags that don't scale.

## Upstream hygiene: avoid workflow-specific constructor flags in vendored libs

If a SceneObject option only serves one local workflow policy (for example, hiding rigid bodies in one preview mode), keep that policy in caller code instead of adding a new public constructor parameter in compas_fab. Prefer stable, generic defaults in shared libraries; put project-specific visibility control in app-level layer toggles.


---

## Cross-script cache sharing: same (cell_id, layer_key) bundle = shared bake

When two scripts both render the same robot cell, route both through the same `ik_viz` bundle by passing identical `layer_key` (e.g. `ik_viz.LAYER_KEY_ASSEMBLY`). Cache key is `(id(robot_cell), layer_key)` so the singleton `rcell` from `robot_cell.get_or_load_robot_cell()` plus a shared layer-key string makes both scripts hit the SAME cached SOs (robot, tools, RBs). Switching between RSIKKeyframe and RSShowIK on the same bar then incurs zero rebake cost.

**Why:** Each `RobotModelObject` / `RigidBodyObject` bake is expensive (one Rhino `AddMesh` per link face). Without shared keys, each command re-bakes its own copy. Hard-coded layer-key strings duplicated across scripts also drift -- one ends up using `""` (no key) and the other `"Assembly"`, silently splitting the cache.

**How to apply:** Define `LAYER_KEY_*` constants in `core/ik_viz.py` and import them from every script that calls `begin_session` / `update_state`. Never pass an inline string literal.

---

## Direct-ownership > RobotCellObject when you need per-type cache invalidation

`ik_viz` now owns one `RobotModelObject` (robot) + dict of `RobotModelObject` (tools) + dict of `RigidBodyObject` (RBs) per `(cell, layer_key)`, instead of going through `RobotCellObject`. RB keyset is diffed against `rcell.rigid_body_models` on every `update_state` so bar/env switches incrementally add/remove only the changed RBs -- robot + tools stay baked.

**Why:** `BaseRobotCellObject._initial_draw` snapshots `rcell.rigid_body_models` once and never re-syncs. Adding/removing RBs after that requires `discard_cache()` (rebakes EVERYTHING). With direct ownership the robot/tool bake survives across bar switches.

**How to apply:** Use `ik_viz.update_state` exclusively; it calls `_sync_rb_keyset` automatically. Reserve `ik_viz.discard_cache()` for hard resets (PyBullet restart, explicit user-flush).

---

## Mesh-mode toggle via per-GUID hide/show, not separate SO instances

When a SceneObject is built with `draw_visual=True, draw_collision=True`, both visual and collision native geometries are baked onto the same layer. Toggle visibility via `rs.HideObjects` / `rs.ShowObjects` on the per-GUID lists in `_links_<mode>_mesh_native_geometry` (RobotModelObject) / `_<mode>_mesh_native_geometry` (RigidBodyObject). Avoid the older approach of building one SO per mode -- it doubles bake cost and complicates cache invalidation.

---

## PyBullet rigid bodies: split sub-meshes into separate bodies

`compas_fab.backends.pybullet.client._add_rigid_body` was joining all meshes in `rigid_body.collision_meshes` into one OBJ before `createCollisionShape`, which makes PyBullet wrap the whole thing in a convex hull -- false-positive collisions for L-shaped or hollow geometry. Fix: create one PyBullet body per Mesh in the list, append all body-ids to `rigid_bodies_puids[name]` (already typed as list), and the existing pose-update loop iterates them.

---

## Querying a Rhino layer's contents does NOT recurse into sub-layers

`rs.ObjectsByLayer("Parent")` returns only objects whose layer is exactly `Parent`. Objects on `Parent::Child` are NOT included. When meshes are baked onto a sub-layer (e.g. `IK Cache::Visual`), querying the parent layer returns `[]`. Either query the actual sub-layer OR read the SceneObject's native-geometry dicts directly.

---

## ik_viz cache must be invalidated on .3dm reopen (sticky persists across files)

sc.sticky is per Rhino SESSION, not per .3dm. Closing a file and opening another (or reopening the same one with stale baked geometry from a previous save) leaves _STICKY_BUNDLE_CACHE pointing at deleted GUIDs and any orphan meshes saved on LAYER_IK_CACHE stay on screen.

**Why:** the one-shot _STICKY_CACHE_INITIALIZED flag never resets within a Rhino session; without doc-change detection the next `begin_session` returns the dead bundle, `update_state` silently no-ops, and the user sees old geometry.

**How to apply:** Detect doc change with `sc.doc.RuntimeSerialNumber` stored in sticky (key `_STICKY_DOC_SERIAL`). On mismatch, drop `_STICKY_BUNDLE_CACHE` + `_STICKY_CACHE_INITIALIZED` BEFORE the orphan-purge runs so the next bundle build starts from a clean LAYER_IK_CACHE.

---

## Orphan-purge must walk sub-layers AND cover sibling preview layers

Corollary of the "ObjectsByLayer doesn't recurse" lesson plus the doc-reopen orphan-purge above. Robot/tool/RB meshes are baked at `LAYER_IK_CACHE::<layer_key>::<MeshMode>` (3 levels deep), so `_delete_layer_objects(LAYER_IK_CACHE)` with a non-recursive `ObjectsByLayer` deletes nothing on file-reopen and the user sees stale geometry from the previous save. Additionally, IK previews bake block instances on sibling layers (`IKPineapplePreview`, `config.SUPPORT_PREVIEW_LAYER`) that are also session-transient.

**How to apply:** In `_flush_cache_layer_once`, (a) recurse via `rs.LayerChildren` when collecting oids and (b) iterate over the full preview-layer list (`LAYER_IK_CACHE`, `SUPPORT_PREVIEW_LAYER`, `IKPineapplePreview`) before setting `_STICKY_CACHE_INITIALIZED`.

---

## `update_state` must force LAYER_IK_CACHE root visible, not just the sub-layer

After `get_robot_link_meshes_at_zero()` hides `LAYER_IK_CACHE` (so the ghost preview is alone during base picking), a successful IK solve calls `update_state` which only ensured the sub-layer (`Assembly`/`Support`) was visible. Rhino layer visibility is hierarchical: a hidden parent hides every child regardless of the child's own visible flag, so nothing renders.

**How to apply:** In `update_state`, force BOTH the root `config.LAYER_IK_CACHE` and the resolved sub-layer visible before issuing the SO updates.

---

## Cached `RobotModelObject` carries the LAST applied pose; re-pose before harvesting

`get_robot_link_meshes_at_zero()` reads `rmo._links_visual_mesh_native_geometry` -- those GUIDs sit at whatever pose the most recent `update_state` applied. For a ghost-preview-at-zero use case this means the ghost appears at the previous IK solution, not at zero, producing a click<->ghost offset.

**How to apply:** Before harvesting, call `rmo.update(zero_state.robot_configuration, Frame.worldXY())` (where `zero_state = default_cell_state()`). Wrap in `Views.RedrawEnabled = False` to avoid a flash.

---

## ACM whitelist for active-bar / mating-joint contacts (IK keyframe / ShowIK)

The IK keyframe + ShowIK path (`scripts/rs_ik_keyframe.py` / `scripts/rs_show_ik.py` / `scripts/core/ik_collision_setup.prepare_assembly_collision_state`) sets the same five-category whitelist on the template state because those workflows run as a single snapshot at the assembled pose (no per-Mi distinction). `env_collision.configure_active_assembly_acm` is the implementation.

**Pairs to whitelist (per design intent):**
* `active_joint_<jid>_male` ↔ `env_joint_<jid>_female` (same `joint_id`, opposite subtype) — the mate during snap.
* `active_joint_*` ↔ both arm tool RBs — gripper engages joint heads.
* `active_bar_<bid>` ↔ all `active_joint_*` on it — rigid bond.
* `active_bar_<bid>` ↔ both arm tool RBs — over-permissive but harmless; bar held via gripped joints.

**Do NOT whitelist:** wrist links vs joints, tool vs env bars, robot links vs robot links — those are real collisions.

**bar_action export uses a different model:** the per-bar export pipeline (`scripts/core/bar_action.py`) does NOT use this template-level whitelist. It clears all touch_bodies after canonicalize and each Mi (M1/M2/M3/M4) builder opts in only the contacts that are by-design for THAT movement, scoped to this bar's joints/pairs. See the next lesson.


---

## Joint registry is normalized: halves keyed by `block_name`, mates reference them

scripts/core/joint_pairs.json switched from a denormalized `{"pairs": [...]}`
schema (each pair embedded its full female + male halves) to a 3-table
normalized schema: `{"halves": [...], "mates": [...], "ground_joints": [...]}`.

**Why.** A single block (e.g. `T20_Male`) appears in multiple mates (T20,
T20SFloorLeft, T20SFloorRight). The denormalized schema duplicated its
geometry across pairs and required a delicate `_resolve_shared_half` check
at write time to detect transform drift. Normalizing means the half is
authored ONCE (via `RSDefineJointHalf`) and a mate is just a thin record
`{name, contact_distance_mm, female_block_name, male_block_name, ranges}`.

**How to apply.** Read via `load_joint_registry()` -> `JointRegistry` (with
`halves`, `mates`, `ground_joints` dicts). The legacy mate-centric API
(`load_joint_pairs`, `save_joint_pair`, `get_joint_pair`,
`list_joint_pair_names`) is preserved as a thin wrapper, so existing
callers work unchanged. `save_joint_pair(pair, overwrite_halves=False)`
upserts only the mate record (use this in `RSDefineJointMate` so picking
new bar lines doesn't accidentally clobber the half geometry authored by
`RSDefineJointHalf`). Ground joints are first-class: a `GroundJointDef`
has `M_block_from_bar` only (no screw frame, no mate partner).

---

## OBJ collision meshes for joint halves live in `asset/<block_name>.obj` (mm)

`RSDefineJointHalf` exports the picked block definition's renderable
geometry as a single-mesh OBJ in MILLIMETRES via
`core.rhino_block_obj_export.export_block_definition_to_obj_mm` (mirrors
the pineapple OBJ export, but in mm not metres). The file becomes
`half.collision_filename` and is consumed by
`env_collision._joint_obj_path_map` for env-collision rigid bodies.

**Why.** Hand-prepared collision OBJs were a recurring papercut whenever a
new block definition was added; the half-author was the natural place to
emit a fresh, registered collision mesh in one shot. Choosing mm here keeps
the asset in the same unit system as the joint registry (everything else
in this project lives in mm).

**How to apply.** When adding a new `Define...` workflow that needs a
collision mesh, reuse `export_block_definition_to_obj_mm(block_name,
output_path)` directly. The function combines all sub-meshes inside the
block definition into one `compas.datastructures.Mesh` and writes via
`cmesh.to_obj`. NOTE this is one mesh per OBJ -- the existing `RigidBody`
"never merge sub-meshes" lesson is about `rigid_body.collision_meshes`
(the in-memory list passed to PyBullet), not about how many `o`/`g`
groups live in a single OBJ file.



---

## Joint-half collision OBJs are user-picked, not auto-exported from block geometry

The first cut of `RSDefineJointHalf` auto-rendered the picked block
definition's full render mesh (Brep meshing + SubD subdivision) and
wrote it as the collision OBJ. That produced 30k-vertex meshes for a
single connector -- way too heavy for IK env-collision and not what
the user wanted: collision geometry must be HAND-MODELED low-poly meshes.

**Why.** Render meshes are visually-tessellated and inherit nested-block
sub-geometry; collision needs convex / few-tri primitives the user
explicitly authors. Auto-exporting silently couples the two roles.

**How to apply.** Add a multi-pick mesh step (`rs.GetObjects(filter=32,
minimum_count=1)`) AFTER the bar/screw geometry picks (so we can hide the
joint annotations and pick stacked meshes cleanly). Use
`export_picked_meshes_to_obj_mm(mesh_ids, block_xform_doc, output_path,
label=...)` -- it applies `inverse(block_xform_doc)` to bring picked
world-space meshes into the block's local frame, then scales doc-units to
mm. The OBJ then re-attaches correctly at any future block placement.
Cancel cleanly if no meshes are picked: a half without collision_filename
falls back to the slow Rhino render-mesh path in env_collision and is
almost certainly an authoring mistake.


---

## Ground-joint placement: closed-form jr from world-up alignment

When you need to auto-orient a connector block about a bar axis so that the block's local +Y points as close to world +Z as possible (RSGroundPlace heuristic), there's a closed form -- no optimizer needed.

**Why:** With `v = bar_R^T @ world_up` (world-up expressed in bar coords) and `b = M_block_from_bar[:3, 1]` (block local +Y in bar coords before the jr rotation), the projected dot product against world up is

    f(jr) = A cos(jr) + B sin(jr) + C

with `A = v_x*b_x + v_y*b_y`, `B = v_y*b_x - v_x*b_y`, `C = v_z*b_z`. Maximum at `jr = atan2(B, A)`. `atan2(0, 0) == 0` keeps it finite even when the bar is colinear with world up (block-Y can never align in the bar's XY plane in that case anyway).

**How to apply:** `core.ground_placement.auto_jr_world_up(bar_start, bar_end, ground)`. Use the canonical bar frame from `core.joint_pair.canonical_bar_frame_from_line` for both the heuristic and the FK forward path so they match. Same closed form generalises to "align block local +k_i to any world direction" by swapping `world_up` and `b`.

## RSJointEdit click-routing: branch on ObjectLayer, then read role-specific UserText

`rs_joint_edit.py` discovers placed-joint instances by ObjectLayer (`FEMALE_INSTANCES_LAYER` / `MALE_INSTANCES_LAYER` / `GROUND_INSTANCES_LAYER` / `LAYER_TOOL_INSTANCES`). Each branch reads its OWN UserText keyset BEFORE any deletion, then re-bakes via the appropriate `place_*` helper.

**Why:** Female/male blocks share `le_rev` / `ln_rev` / `joint_pair_name` / `female_parent_bar` / `male_parent_bar` because they participate in a 4-variant solver. Ground blocks share none of that -- they only have `ground_joint_name` / `parent_bar_id` / `position_mm` / `rotation_deg` and a single `jr += pi` flip semantics. Trying to read `le_rev` from a ground block yields empty strings and silently corrupts the flip path.

**How to apply:** Add the layer check FIRST (before any common metadata read), short-circuit with a dedicated helper (`_flip_ground_block(clicked_id)`), and `continue` the loop. Per-role helpers own their own metadata schema and their own deletion + re-bake call -- the main loop stays a thin dispatcher.

---

## Ground-joint "flip" must NOT be `jr += pi` -- post-multiply M by `R_y(pi)` instead

For RSGroundPlace, the user-facing "flip" = reverse the block's X axis along the bar while keeping the block's Y axis pointing world-up. With the (jp, jr) parametrization and a fixed `M_block_from_bar`, this is NOT expressible as a jr change: rotating about the bar Z by pi flips both block X AND block Y in world (block ends upside-down).

**Why:** The two "block X along +bar" vs "block X along -bar" orientations differ by 180 deg about the WORLD UP axis. For a horizontal bar, world-up has zero component along bar Z, so this is NOT a rotation about bar Z. So no jr value can turn one into the other while keeping Y up.

**How to apply:** Post-multiply `M_block_from_bar` by `R_y(pi)` (block-local Y, NOT world Y). `R_y(pi) = diag(-1, 1, -1)`: it reverses block-local +X and +Z columns and preserves block-local +Y. Because +Y is preserved, the closed-form `auto_jr_world_up` returns the SAME jr (it depends only on b = M[:3,1]). The composed FK then reverses block world-X while preserving block world-Y. Carry the flag as a `flipped` bool through the session and persist as a UserText (`"flipped": "True"/"False"`) for re-edit. See `core.ground_placement.effective_M_block_from_bar`.

## Lesson: Bake-time UserText is the source of truth for joint identity �� never derive joint_id deterministically from (asset, parent) alone.

**Why.** Initial `make_ground_joint_id(bar_id, ground_name) -> "G{bar_num}-{ground_name}"` collided whenever the user placed a second ground with the same ground definition on the same bar �� re-baking with the same ObjectName/UserText silently clobbered the first instance (or got clobbered by the pre-bake `remove_placed_*` cleanup). Symptom: "the first joint disappears".

**How to apply.**
- Joint id formula must include a uniqueness suffix discovered by *scanning existing baked instances*, not by the user's input alone.
- Pattern used: `make_ground_joint_id(bar_id, name, *, index)` + `next_ground_joint_index(bar_id, name)` that walks `rs.ObjectsByLayer(GROUND_INSTANCES_LAYER)` and reads each `joint_id` UserText to find the smallest free integer suffix.
- `place_*_block` allocates a fresh id when `joint_id=None`; re-edit code paths (flip, etc.) MUST pass the existing `joint_id` explicitly so the suffix is preserved across re-bakes.
- Drop any pre-bake `remove_placed_*` "redo cleanup" �� once ids are unique per placement, that call destroys the previous joint instead of cleaning up.

---

## Lesson: Tool placement is a generic "TCP coincides with a block-instance frame" operation �� share one core, not two parallel implementations.

**Why.** Male joints and ground joints both want the robotic tool's TCP to land on a block instance's world frame (`world_tool_block = block_world @ inv(M_tcp_from_block)`). The original `place_tool_at_male_joint` hard-coded `_male_world_frame_from_object` and a `pair` argument it never used in the math, blocking reuse for ground joints (which have no JointPairDef).

**How to apply.**
- Extract a generic `place_tool_at_block_instance(block_oid, joint_id, tool)` that takes ANY InstanceObject and writes the tool's UserText (`joint_id`, `tool_name`, `tool_id`, `block_name`).
- Variant auto-placers (`auto_place_tool_at_male_joint`, `auto_place_tool_at_ground_block`) only differ in tool *selection* (per-pair preferred name vs. doc-default fallback), then delegate to the same core.
- Cycling/finding generalizes too: `find_attached_block_for_joint` dispatches on `joint_id` prefix (`J*` male, `G*` ground) so `cycle_tool_at_tool_instance` works on tools attached to either kind of joint without branching.
- Re-edit flows (e.g. flip) should capture `get_tool_name_for_joint(joint_id)` BEFORE re-baking the joint block, then re-place via `place_tool_by_name_at_*` so the user's tool choice persists.

---

## Lesson: RSExportRobotCell must force-register the full assembly before dumping the cell.

**Why.** The cached `rcell.rigid_body_models` is mutated by every call to `env_collision.register_env_in_robot_cell` (run by every `prepare_assembly_collision_state` from RSIKKeyframe, RSShowIK, the BarAction builder, etc.) — it shrinks the cell to whichever bar's *active context* that last call ran for. So dumping the cached cell as-is produces a per-bar snapshot, NOT the full assembly, unless the very last operation before the dump happened to be a full-assembly registration. Symptom: BarAction state has 32 bars + their joints; cell has only 25; `assert_cell_state_match` raises *"workpieces in the cell state do not match the workpieces in the robot cell. Mismatch: {bar_B235, bar_B237, …, joint_J234-256_male, …}"*.

**How to apply.**
- In `scripts/rs_export_robotcell.py`, call `bar_action._register_full_assembly_geom(rcell, planner)` immediately before `dump_cell_canonical`. The dump is now deterministic regardless of session history. Requires PyBullet (RSPBStart).
- Same belt-and-suspenders in `scripts/rs_export_all_bar_actions.py` right before its final cell dump, in case the last build iteration failed AFTER `prepare_assembly_collision_state` (which shrinks the cell) but BEFORE its own `_register_full_assembly_geom`.
- Diagnostic: load both `RobotCell.json` and `BarActions/<bar>.json`; diff `cell.rigid_body_models.keys()` against `mv.start_state.rigid_body_states.keys()`. If the cell is a strict subset, the cell was dumped from a shrunken per-bar context.

**Follow-up — also re-assert arm-tool RBs at dump time.** The above only covers bar / joint bodies. The two per-arm tool collision RBs (`AssemblyLeftArmToolBody`, `AssemblyRightArmToolBody`) come from `robot_cell.attach_arm_tool_rigid_bodies`, which is only called inside `prepare_assembly_collision_state`. If `RSExportRobotCell` runs in a session that never went through an IK pass or BarAction export, the arm-tool RBs are missing and every BarAction state mismatches on those two names. Symptom: *"Mismatch: {'AssemblyLeftArmToolBody', 'AssemblyRightArmToolBody'}"*. Fix: `bar_action._attach_arm_tools_to_cell(rcell, planner)` scans the doc for any bar with both left+right tools placed, resolves collision paths via `robotic_tools.json`, attaches. Called in both export scripts right before `dump_cell_canonical`.

---

## Per-movement ACM scoping in BarAssemblyAction export (M1-M4)

The bar-action export pipeline (`scripts/core/bar_action.py::build_bar_assembly_action`) does NOT use the template-level `configure_active_assembly_acm` whitelist. After canonicalizing the start state, it CLEARS every `rigid_body_states[*].touch_bodies` and then each per-movement builder (`_build_m1` / `_build_m2` / `_build_m3` / `_build_m4`) opts in ONLY the contacts that are by-design for THAT movement, scoped to the active bar.

**Why.** The template snapshot says "everything that will touch sometime during the whole action is whitelisted everywhere" — which over-permits. M1 (arm reaches in with bar + held joints, mate not yet engaged) must still flag spurious mate↔female contact; only M2 (the snap moment) and M3 (held against mate) should whitelist mate-pair touch. The user's framing: *"the template cell state should only contain the relationship of the tool and the robot, the two tools in the two arms. The rest of the explicit exceptions should be made per movement."*

**Per-Mi whitelist matrix (this bar's keys only):**

| Movement | bar↔arm-tool RB | bar↔this bar's joints | male↔matching-arm tool RB | mate (male↔female) pair |
|---|---|---|---|---|
| M1 reach   | ✓ | ✓ | ✓ | ✗ |
| M2 snap    | ✓ | ✓ | ✓ | ✓ |
| M3 hold    | ✓ | ✓ | ✓ | ✓ |
| M4 retract | ✗ | ✗ | ✗ | ✗ |

Tools↔stationary env-bar/env-joint contacts are auto-skipped by `_skip_collision_check` (stationary↔hidden / stationary↔stationary) — never whitelist them.

**How to apply.**
- Use `_clear_all_touch_bodies(state)` after canonicalize; never inherit upstream `touch_bodies`.
- Use `_touch_pair(state, a, b)` for every intentional contact (mutates both sides).
- Drive the per-Mi opt-in via `_apply_per_movement_acm(state, *, bar_key, jids, arm_to_male, arm_tool_rb_names, whitelist_bar_tools=…, whitelist_bar_joints=…, whitelist_male_tools=…, whitelist_mates=…)` with the matrix above.
- Wrist↔anything is NEVER in this matrix. If a real M2 pose needs the wrist into the joint, fix the tool OBJ (or the IK seed), not the ACM.

---

## Cached `rcell.rigid_body_models` is mutated by every export; snapshot+restore in script-level try/finally

`build_bar_assembly_action` + `_register_full_assembly_geom` + `_attach_arm_tools_to_cell` all mutate `rcell.rigid_body_models` in place (necessary: the dumped `RobotCell.json` must be a state-independent superset across all bars + arm tools). The cached `rcell` is the same singleton other workflows in the same Rhino session see, so unrestored mutations leak: a subsequent RSShowIK / RSIKKeyframe runs against a cell whose key set no longer matches the state it builds, producing `assert_cell_state_match` mismatches on bars/joints registered only for export.

**How to apply.**
- Before the polluting call(s) in each export entry-point, capture `snap = bar_action.snapshot_cell_rigid_bodies(rcell)`.
- Wrap everything that mutates the cell (build, register-full, attach-arm-tools, dump) in `try:` and put `bar_action.restore_cell_rigid_bodies(rcell, snap, planner)` in `finally:`. The restore re-pushes the cell to the planner so the next collision check sees the pre-export shape.
- Wired in `scripts/rs_export_bar_action.py`, `scripts/rs_export_all_bar_actions.py`, `scripts/rs_export_robotcell.py`. Other future export-style scripts that call any of those three helpers MUST follow the same pattern.

---

## Shared schemas loaded from Rhino: submodule + sys.path prepend, not `pip git+<sha>` in requirements.txt

When a python package is shared across this repo and sibling Rhino-loading repos (e.g. monitor / planner), vendor it as a git submodule under `external/<pkg>/` and prepend its package root onto `sys.path` from the first module that imports it — mirroring how `core.robot_cell` loads `external/compas_fab/src`. Do NOT add `# r: <pkg>` to any Rhino script, and prefer this over `pip install git+...@<sha>` in `requirements.txt`.

**Why:** Rhino's ScriptEditor `# r:` install caches by package name, not by git SHA, so a later `git+...@<newsha>` won't refresh the venv — the gotcha documented in `tasks/yh_lesson.md`. A submodule pins the bytes directly in the working tree and a sys.path prepend gives deterministic resolution that won't be shadowed by a stale install from a sibling Rhino session.

**How to apply:** `git submodule add <url> external/<pkg>` and `git checkout <sha>` inside it. In the consuming module, compute `_ROOT = <repo>/external/<pkg>`, tolerate flat vs `src/` layout, raise a `RuntimeError` with `git submodule update --init --recursive` instructions if missing, then `sys.path.insert(0, _SRC)` before the `import <pkg>`. Document under a README "Submodule dependency for ..." subsection and add an entry to the Project Structure tree. Wired this way for `rs_data_structure` (BarAction schema) in `scripts/core/bar_action.py`.
