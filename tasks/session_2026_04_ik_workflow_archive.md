# Session archive — IK keyframe workflow build-out (Apr 2026)

Concise handoff doc. Pair with [yh_lesson.md](yh_lesson.md), [cc_lessons.md](cc_lessons.md), [test_pipeline_plan.md](test_pipeline_plan.md), [subagent_onboarding_plan.md](subagent_onboarding_plan.md).

## Scope built

End-to-end dual-arm IK keyframe workflow with capture/replay debug loop. Designer clicks two male joints in Rhino → pineapple (wrist+tool proxy) preview → pick mobile-base on `WalkableGround` brep → `compas_fab` PyBullet IK for final + approach poses → Rhino mesh viz → save `ik_assembly` JSON on Ln bar. Plus headless replay scripts so Claude can self-debug without Rhino clicks.

## File map

### Scripts (Rhino entry points)
| File | One-liner |
|---|---|
| `scripts/rs_pb_start.py`, `rs_pb_stop.py` | PyBullet client lifecycle (sticky-cached) |
| `scripts/rs_export_joint_tool0_tf.py` | Pick male-joint + tool0 frame groups → write `MALE_JOINT_OCF_TO_TOOL0[block][side]` |
| `scripts/rs_export_pineapple_obj.py` | Live-export pineapple block defs to `asset/*.obj` (m), reuse-or-overwrite prompt |
| `scripts/rs_ik_keyframe.py` | Main workflow. Captures on Accept AND on IK failure |
| `scripts/rs_show_ik.py` | Replay `ik_assembly` from a bar |
| `scripts/rs_reset_py.py` | Lightweight `sys.modules` purge, scoped to `scripts/` only |

### Core (Rhino-side helpers, also pure-Python compatible)
| File | Notes |
|---|---|
| `scripts/core/config.py` | Joint-pair-agnostic constants. Imports `config_generated_ik` (optional). NO `os`/`np` imports were missing post-Victor merge — patched. |
| `scripts/core/config_generated_ik.py` | Auto-gen by `rs_export_joint_tool0_tf.py`. Schema: `{block_name: {"left"|"right": 4x4}}`. |
| `scripts/core/robot_cell.py` | Compas stack lazy-import. Submodule sys.path injection. Tool attachment via `_TOOL_TARGETS`. `scriptcontext` import is optional (try/except → falls back to `_STICKY = {}` so headless tests work). |
| `scripts/core/ik_viz.py` | Rhino scene cache. `show_state(state, mesh_mode="visual"|"collision")`. Sticky-cached via `set_mesh_mode`/`get_mesh_mode`. Uses `compas_robots.rhino.scene.RobotModelObject` (RobotCell isn't Rhino-registered in compas_fab wip_process). |

### Tests (headless, plain Python)
| File | Purpose |
|---|---|
| `tests/_rhino_env_bootstrap.py` | `bootstrap_rhino_site_envs()` — `site.addsitedir` for `~/.rhinocode/py39-rh8/site-envs/scaffolding_env-*` so plain `python.exe` can `import compas`. Skip when `scriptcontext` is already loaded. |
| `tests/replay_ik.py` | Pure-Python replay. Tolerates partial captures (failure mode). |
| `tests/replay_ik_with_viz.py` | Same + `rhinoinside` for `ik_viz.show_state` regression. |
| `tests/debug_ik_collisions.py` | PyBullet GUI/DIRECT collision inspector. Flags: `--phase`, `--headless`, `--use-saved-ik`, `--collision-shapes`. |

### Assets / submodules
- `external/compas_fab/` — submodule, branch `wip_process`, loaded via `sys.path` injection. **NEVER** `# r: compas_fab`.
- `asset/AssemblyLeft_Pineapple_m.obj`, `AssemblyRight_Pineapple_m.obj` — tool meshes in meters.
- `asset/husky_urdf/` — submodule.

### Subagent definitions
`.claude/agents/{planner,implementer,reviewer}.md` — `model: opus`, `thinking: enabled`. Tool sandboxing: planner read-only, reviewer read-only + Bash, implementer Edit/Write/Bash.

## Key design decisions

1. **Capture is the unit of debug.** `RSIKKeyframe` writes a JSON every Accept and (with prompt) on every IK failure. The JSON is self-contained: OCFs, base frame, collision flags, optional expected configs. Replay scripts re-solve from inputs and compare.
2. **Capture lives in `tests/captures/<timestamp>_<bar>[_ik_fail_*]?.json`.**
3. **`MALE_JOINT_OCF_TO_TOOL0` is nested per-arm.** Block-name keyed (`"T20_Male"`), then arm-side (`"left"|"right"`). Two arms carry different tools.
4. **`touch_links` for tools = `[wrist_1, wrist_2, wrist_3]`.** The pineapple OBJ extends 157 mm back from tool0; this is inherent overlap, not real collision. `forearm_link` deliberately OUT — IK should steer away if it ever reaches there.
5. **compas_fab `CC.x` filter ladder is correct as-is.** SRDF `disable_collisions` (98 pairs for husky) handles CC.1 link-link. Tool `touch_links` handle CC.2 link-tool. We just need to populate the right side data.
6. **`ik_viz` uses `RobotModelObject` + post-bake base xform.** compas_fab wip_process doesn't register a Rhino `RobotCellObject`; only compas_robots registers `RobotModelObject`. We compose `world_from_base` ourselves and apply via `rs.TransformObjects`.
7. **Always call `scene_object.update(joint_state)` before `draw_*()`.** Setting `.configuration` alone doesn't run FK; meshes draw at last pose.
8. **`PyBulletClient(verbose=True)` inside Rhino.** `verbose=False` triggers `os.dup(stdout_fd)` which dies in ScriptEditor.
9. **`# r:` requirements pinned exact.** baseline: numpy==1.24.4, scipy==1.13.1, compas==2.13.0, compas_robots==0.6.0, pybullet==3.2.7, pybullet_planning==0.6.1.

## Critical gotchas (read these first)

1. **Rhino `# venv` is a site-env, not a virtualenv.** Base interpreter `site-packages` leaks in. `pip install -e` against the base interpreter pollutes everything. Cleanup commands in `yh_lesson.md`.
2. **Don't selectively purge `sys.modules` for compas/compas_fab/compas_robots.** Class identity drift → `super(X, self)` fails. To pick up a new submodule SHA, use **ScriptEditor → Tools → Reload Python 3 (CPython) Engine**. Our `ResetPy` alias purges only `scripts/` subtree.
3. **`pp.clone_body` global state.** Call `pp.set_client(client_id)` first; pp helpers (`set_pose`, `INFO_FROM_BODY`) read `pp.utils.shared_const.CLIENT`.
4. **`pp.clone_body` does NOT retain link names.** Query names from the ORIGINAL body using the same link index — clone link i ↔ original link i for default `links=None`.
5. **PyBullet renders `visualShapeIndex=-1` bodies via collision shape.** That's how `clone_body(visual=False)` becomes a "show me collision proxy" view. Tint via `changeVisualShape(rgbaColor=...)`.
6. **URDF `husky` body has 37 links.** Only 22 are UR-arm (`*ur_arm*`). Chassis/wheels/bumpers have huge box collision proxies that obscure debug GUI — hide them by setting `rgbaColor=[0,0,0,0]` on those clone links.
7. **Site-env hash changes per session.** Path is `~/.rhinocode/py39-rh8/site-envs/scaffolding_env-<HASH>`. Bootstrap helper globs by name prefix.
8. **`tests/conftest.py` only adds `tests/` and `scripts/` to sys.path.** It does NOT bootstrap site-envs. The replay/debug scripts call `bootstrap_rhino_site_envs()` themselves.

## Current state of IK collision check

- Production `RSIKKeyframe` IK with `check_collision=True` — **passing** for both validated captures (`20260428_140445_B3.json`, `20260428_232410_B3.json`).
- All own-arm tool↔wrist overlaps correctly `SKIPPED (ALLOWED TOUCH LINK)`.
- All SRDF-disabled link↔link pairs correctly `SKIPPED (SEMANTICS)`.
- Cross-arm tool↔opposite-arm-links and tool↔forearm remain real checks (will fail when they actually clip).
- Visual debug: `python tests/debug_ik_collisions.py <capture> --use-saved-ik --collision-shapes` — clones bodies with `pp.clone_body(collision=True, visual=False)`, hides non-arm husky links, tints semi-transparent blue.
- In Rhino, `RSIKKeyframe`'s collision-options prompt also has `MeshMode visual/collision` toggle (cached in sticky for session).

## Outstanding TODOs

From `test_pipeline_plan.md`:
- Phase A1: extract IK math (`_translate_frame`, `_unit`, `_frame_from_origin_normal_heading`, `tool0_from_ocf`, etc.) from `rs_ik_keyframe.py` into `core/ik_workflow_math.py` for unit-test coverage.
- Phase B: pytest suites for math + `config_sanitize` + `robot_cell` IK roundtrip.
- Phase B (collision plan): register assembled bars + joint blocks as `rigid_body_models` so the IK catches arm-vs-assembly collisions (the "yellow joint" class). Currently NOT implemented.
- Phase C: `WalkableGround` brep as a rigid body (lower priority).
- A Rhino-native `RobotCellObject` upstream PR-able to compas_fab wip_process so `ik_viz` can drop the manual base-frame composition.
- Possible: bump `# r: numpy==1.24.4` to match what scaffolding_env actually has (2.0.2). The pin is currently respected on first install but mismatches after Engine reset.

## Subagent test drive (not yet run)

3-agent chain in `.claude/agents/`. Suggested first task: Phase A1 math extraction. User has not yet driven the chain end-to-end; do that on the next non-trivial change.

## Open behavioural questions

- Pineapple proxy is over-modelled (extends 157 mm back to wrist_1's volume). Could re-author `.3dm` block to be tool-only forward of tool0 — would let us drop `wrist_1` from `touch_links` and catch real wrist_1 collisions. Not done yet — current `touch_links` workaround is acceptable.
- `IK_BASE_SAMPLE_MAX_ITER = 5` (was 20). If sampling failures become common, raise.
- `forearm_link` is OUT of `touch_links` deliberately. If a real-world capture shows this rejecting a clearly-valid pose, revisit.

## How to pick this up in a future session

1. Read this archive + `yh_lesson.md` + `cc_lessons.md` (5 minutes).
2. Verify environment: submodule checked out, scaffolding_env has compas/pybullet/etc., base `~/.rhinocode/py39-rh8/Lib/site-packages/` is free of editable `compas_fab` `.pth` (cleanup commands in `yh_lesson.md`).
3. Smoke test: `"C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe" tests/debug_ik_collisions.py tests/captures/20260428_232410_B3.json --use-saved-ik --headless` — should print `[OK] check_collision passed` and exit 0.
4. If you need to iterate Rhino code without restarting the Engine, use the `ResetPy` alias.
5. For multi-file refactors, drive the planner → implementer → reviewer chain (`.claude/agents/`).
