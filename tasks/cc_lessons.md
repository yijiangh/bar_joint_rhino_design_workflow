# Claude Code Lessons

Short notes on patterns we've hit before. Lead with the pattern; add **Why** and **How to apply**.

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

**Why:** When the support-IK script runs, the dual-arm assembly robot is "frozen" at its captured IK pose but must still be in collision with the support arm. Loading two URDFs as actuated robots in one `RobotCell` is not supported. The pattern came from `support_materials/gh_keyframe_demos/python/GH_create_robot_cell.py:16-17`. The dual-arm tool's pose is set via `state.tool_states['DA'].frame` (world-frame placement of the husky base) and its internal joint configuration via `state.tool_states['DA'].configuration` — both honored by PyBullet collision check.

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
