# Claude Code Lessons

Short notes on patterns we've hit before. Lead with the pattern; add **Why** and **How to apply**.

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

Counter-example for context: the existing pineapple `tool_state.touch_links = [wrist_1/2/3]` is a different beast — the proxy mesh geometrically extends backward into those links as a *modeling* artifact (the OBJ encompasses the wrist visualization), so the overlap is a property of the proxy, not of the assembly contact. Both kinds of whitelisting exist; just be explicit about which one each entry is.

**How to apply:** When exposing ACM seams for future use:
- Whitelist tool body (`touch_bodies = [tool_name]`) — the gripper is at the joint by design.
- Do NOT pre-add the wrist link to `touch_links`. If a real engagement pose later needs it, justify it pose-by-pose, not as a blanket allowance.
- Document each `touch_*` entry inline with WHICH category it belongs to: design-intent contact, or modeling-artifact overlap. The two have different lifecycles (design-intent expires when the assembly step ends; modeling-artifact persists as long as the mesh).

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

BaseRobotCellObject._initial_draw walks obot_cell.rigid_body_models exactly ONCE and caches a _rigid_body_scene_objects[id] dict. RBs added (or removed) AFTER the first draw will silently NOT appear (or worse: stale entries reference deleted models). When a viewer wants to show env_* + arm-tool RBs alongside the robot/tools, you must:
- Mutate cell.rigid_body_models (via ttach_arm_tool_rigid_bodies + egister_env_in_robot_cell) BEFORE calling Scene().add(robot_cell, draw_rigid_bodies=True, ...).
- Discard + rebake the cached scene object whenever the RB key-set changes (e.g. user navigates to a different active bar with a different built-before set).

In s_show_ik._PreviewSession: _render calls prepare_assembly_collision_state BEFORE ik_viz.begin_session, and set_active_bar calls ik_viz.discard_cache() when switching to a different bar.

## ik_viz.begin_session: hide doc layers as a list, not boolean flags

When the cached cell viz visually duplicates user-modeled doc geometry (tube previews, joint block instances, tool block instances), hide those doc layers for the duration of the session and restore prev visibility on end_session. Generic mechanism: hide_doc_layers: Iterable[str] = None kwarg (default = the assembly-overlap layer set) + _STICKY_HIDDEN_DOC_LAYERS dict {layer: prev_visible} that _restore_hidden_doc_layers() walks on session exit. Avoids per-layer boolean flags that don't scale.

---

## ShowIK viz: pre-attach all RBs before first cell bake; discard cache on bar switch

BaseRobotCellObject._initial_draw walks obot_cell.rigid_body_models exactly ONCE and caches a _rigid_body_scene_objects[id] dict. RBs added (or removed) AFTER the first draw will silently NOT appear (or worse: stale entries reference deleted models). When a viewer wants to show env_* + arm-tool RBs alongside the robot/tools, you must:
- Mutate cell.rigid_body_models (via ttach_arm_tool_rigid_bodies + egister_env_in_robot_cell) BEFORE calling Scene().add(robot_cell, draw_rigid_bodies=True, ...).
- Discard + rebake the cached scene object whenever the RB key-set changes (e.g. user navigates to a different active bar with a different built-before set).

In s_show_ik._PreviewSession: _render calls prepare_assembly_collision_state BEFORE ik_viz.begin_session, and set_active_bar calls ik_viz.discard_cache() when switching to a different bar.

## ik_viz.begin_session: hide doc layers as a list, not boolean flags

When the cached cell viz visually duplicates user-modeled doc geometry (tube previews, joint block instances, tool block instances), hide those doc layers for the duration of the session and restore prev visibility on end_session. Generic mechanism: hide_doc_layers: Iterable[str] = None kwarg (default = the assembly-overlap layer set) + _STICKY_HIDDEN_DOC_LAYERS dict {layer: prev_visible} that _restore_hidden_doc_layers() walks on session exit. Avoids per-layer boolean flags that don't scale.
