# Implementation plan: exporter fixes D1–D9 (`docs/support_export_issues_from_monitor.md`)

## Status (2026-10-02)

- Phases 0-5 implemented on `hs/mocap-experiment` (uncommitted). Full test suite: 240 passed,
  10 skipped (baseline before: 206 passed).
- Step 0 check done headlessly (Rhino bridge was down): in today's `B3__H`, frozen Cindy collides
  with `env_bar_B5` in H_M0/H_M2/H_M3 and nothing else, so rule (c) is needed. With rules (a)-(c)
  applied in memory, all three states are clear; the approach pose clears the held bar without
  the gripper allowance; the held pose needs only `SupportGripper` (not the wrist links).
- D1 dry run: all four held bars' release states (+ retreat / home targets) are clear with the own
  holder placed. D6 dry run with a `ground_WG0` slab: wheels / chassis / arm tools clear in Cindy's
  and Alice's cells; B1 / B5 ground joints touch the floor only at the insert target (allowed).
- NOT done (needs live Rhino): re-solve the four holds (RSIKKeyframeAll), re-export into a new
  folder, `python tests/check_export_bundle.py <folder> --cells` -> 0 failed rules, RSShowBarActionPlan
  spot checks. Tell the monitor side before overwriting the shared folder.

## Context

The live monitor found that the exported action files are wrong as stand-alone collision scenes
(issues D1–D8), and the designer added a rule the export does not follow yet (D9: ground bars use no
jointing motor; an operator fixes the foundation). The issue doc now has every cause, pointer and
decision. This plan turns it into code changes in this repo. Outcome: a fresh export of
`260920_RobArch_demo_revamp` passes an automatic checker and the monitor needs no workarounds.

Decisions already made (designer):

| Topic | Decision |
|---|---|
| D5 hold scene | Keep release-time bars in `__H` and `__HR`; fix the allowed contacts instead |
| D6 floor | Export floors as rigid bodies, **only the walkable grounds the actions actually use** (here `WG0`; the vertical `WG2` and unused `WG1` stay out) |
| D7 | Release = ungrasp, retreat, home (no untighten) |
| D9 | Ground bar: no tighten; operator's fix step is the last movement of `__J`; a bar mixing a ground joint and a male joint is an error |
| Data | Test on `260920_RobArch_demo_revamp`; never touch `260929_phase1_retest` (Su's) |

One inferred rule, checked before it is built (Phase 3, step 0): in `__H`, a frozen robot may touch
bars built after it leaves.

## Step 0 — paperwork (no code)

- Add the WG decision to D6 in the issue doc; link this plan from the doc's status table.
- Save this plan as `tasks/2026-10-02_support_export_fixes_plan.md` (work spans sessions).
- New read-only checker `tests/check_export_bundle.py <export folder>` (plain Python + json, grown from
  the scan scripts used on 2026-10-02). It prints one PASS/FAIL line per rule; each phase below adds its
  rules. Running it today on the reference export gives the "before" picture.

## Build order

What is built first: the movement list (Phase 1) and the scene bodies with their names (Phase 2).
What layers on top: allowed contacts for the floor (end of Phase 2), then the hold/release scenes
(Phase 3), then sequence and schedule bookkeeping (Phase 4).

### Phase 1 — movement list: D7 + D9

Files: `scripts/core/bar_action.py`, `scripts/core/ik_collision_setup.py`, `scripts/rs_ik_keyframe.py`,
`scripts/rs_ik_keyframe_all.py`, `scripts/rs_show_bar_action_plan.py`,
`external/rs_data_structure/rs_data_structure/bar_action.py` (docstrings only), `tests/test_bar_action_ground.py`.

1. `build_split_assembly_movements` returns two ordered dicts keyed by step **name**, not number:
   - jointing, normal bar: `free_to_load`, `manual_mount_bar`, `tool_grasp_bar`,
     `CDFM_transfer_to_approach`, `tool_tighten_joint`, `LM_insert`
   - jointing, ground bar: same first four, then `LM_insert`, `manual_fix_foundation`
   - release, every bar: `tool_ungrasp_bar`, `LM_retreat`, `free_home`
   The numbers in the ids (`B1_J_M4_LM_insert`) are stamped in one place, in order, after the dict is
   built. `_build_m0.._build_m4` stop hard-coding ids.
2. Split the "timeline" part (today lines 1523-1574) into a small Rhino-free function taking the five arm
   movements + `is_ground_bar`, so both shapes are unit-tested without Rhino.
3. Ground bar (`arm_to_ground` not empty): no tighten movement; insert notes
   `ends_on = "target_reached"` (normal bars keep `"tool_stall_signal"`; two named constants); add
   `ManualMovement` `manual_fix_foundation`, tag "Operator fixes the foundation to the ground (robot
   keeps holding the bar)", state = insert's end (bar attached, assembled config; today's `r_m0_state`).
   The insert stays on the compliant controller (open point for the monitor side, noted in the doc).
4. Mixed bar (ground joint + male joint): clear error naming the bar, in the builder and in both tool
   gates (`ik_collision_setup.resolve_arm_tools_on_bar`, `rs_ik_keyframe._resolve_arm_tools_on_bar`).
5. Replace `_movement_by_role(actions, "J_M5")` with a lookup by name ending (`"J"`, `"LM_insert"`).
   It also reads files exported before this change, since those names did not change.
6. Callers switch from numbers to names: `write_bar_keyframe_from_action` (approach = `LM_insert` start,
   assembled = `tool_ungrasp_bar` or `LM_retreat` start, retreat = `free_home` start);
   `rs_ik_keyframe.py:2849-2854`; `rs_ik_keyframe_all.py:229-233`; `rs_show_bar_action_plan.py:679`
   and `:854-911` (the "assembled" pose uses `tool_ungrasp_bar`'s state);
   `build_bar_assembly_actions` uses `list(dict.values())`.
7. Update the two action-class docstrings in `rs_data_structure` (roles by name; ground-bar variant).

Checker rules: no `untighten` anywhere; `__R` has exactly 3 movements; ground bars (B1, B5) have no
`tighten`, end with `manual_fix_foundation`, insert `ends_on == "target_reached"`; other bars unchanged.

### Phase 2 — scene bodies: D2 names, D3 obstacles, D6 floor

Files: `scripts/core/env_collision.py`, `robot_cell.py`, `hold_action_builder.py`, `robot_obstacles.py`,
`ik_collision_setup.py`, `bar_action.py`, `config.py`, `scripts/rs_select_joint.py`, tests.

1. **One naming (D2).** `bar_body_name(bar_id)`, `joint_body_name(joint_id, subtype)` and one
   `MANAGED_BODY_PREFIXES` tuple in `env_collision.py`; delete `ENV_RB_*` and the copies in
   `bar_action.py:281-282`, `rs_select_joint.py:54`. `collect_built_geometry` becomes a filter over
   `collect_assembly_geometry` (keep bodies whose parent bar's step is before / up to the given bar), so
   the duplicated loop goes away and support scenes get the same names and `parent_bar_id` as Cindy's.
   `whitelist_frozen_contact` uses the one name. Delete unused `robot_cell.ensure_env_registered`.
2. **Static bodies, collected once (D3 + D6).** New cached `collect_static_scene_geometry(force=False)`
   in `env_collision.py` = `collect_environment_geometry()` + new `collect_floor_geometry()`.
   - Floors: union of `get_bar_ground_ids(oid)` over real bars (run the existing
     `auto_assign_walkable_ground_ids_all_bars` first). One body per used ground, named `ground_<id>`
     (e.g. `ground_WG0`), meshed with the existing `_env_object_to_compas_mesh`, given a 50 mm slab
     below the surface (a flat face alone is not a solid). A used ground that is not flat raises a clear
     error asking to split it. No used ground: loud note, no floor.
   - Each floor's body info carries `kind="ground"` and its always-allowed contacts: the four wheel links
     (`config.FLOOR_TOUCH_LINKS`) and the frozen-robot tools (`config.OBSTACLE_TOOL_NAMES`).
3. **Cindy's cell.** `rebuild_assembly_cell` calls the collector with `force=True`; the staleness
   fingerprint also counts the walkable layer and the used ground ids.
   `build_full_assembly_state` never hides `environment`/`ground` bodies and copies their allowed contacts.
4. **Support cells.** `get_env_union` = all bars/joints + the cached static bodies (same objects, so no
   re-sending of the cell). `build_env_state` copies the allowed contacts; `build_support_scene_state`
   hides only bars/joints that are not in the scene; the stale-body scans use `MANAGED_BODY_PREFIXES`.
5. **Ground joints on the floor.** In `_apply_movement_touch_policy`, grasped and tool-less ground
   joints add the floor names in the insert state only (policy key `"M2"`: insert + operator's fix
   step), not during the transfer.

Checker rules: no `env_bar_`/`env_joint_` names; every state has `ground_WG0`, shown, with the wheel
links and frozen-robot tools allowed; no `ground_WG1`/`ground_WG2`; B1/B5 ground joints list the floor
in the insert and fix states and not in the transfer state.

### Phase 3 — hold and release scenes: D5 + D1

Files: `scripts/core/hold_action_builder.py`, `hold_schedule.py`, `bar_action.py`, `rs_ik_keyframe.py`,
new `tests/test_hold_scene_rules.py`.

0. **Check first.** In live Rhino, load today's `B3` hold scene and run one collision check: confirm
   frozen Cindy hits B5. If she does not, drop rule (c) below and tell the designer.
1. One scene builder `build_hold_scene_state(robot_name, held_bar_id, hold_plan, env_union, bar_map)`
   used by the interactive solve (`rs_ik_keyframe.py:2517-2549`), the batch re-solve
   (`resolve_support_keyframe_noninteractive`) and the `__H` export, replacing three copies. It keeps
   the release-time bars and:
   (a) shows the held bar and its joints (stop excluding them in `collect_hold_window_geometry`);
   (b) frozen Cindy may touch the held bar, the joints on it, and the mate female of each of its males;
   (c) a frozen robot may touch bars/joints built after it leaves (Cindy: after the hold starts; another
       holder: after its own release). The "which bodies" part is a Rhino-free function in
       `hold_schedule.py` with unit tests.
2. Gripper ↔ held bar is a per-movement switch (`set_gripper_bar_contact(state, bar_id, allowed)`):
   allowed in `H_M2`, `H_M3`, `HR_M0`, `HR_M1`; not in `H_M0`, `H_M1`. The solves follow the same
   rule through one helper `solve_hold_pair(...)`: held pose with contact allowed, approach pose
   without. `validate_release_confs` does the same for its two checks.
3. Release scene (`build_release_scene_state`): held bar shown; a hold released earlier in the same
   step is left out (order = hold-start order, same as `build_action_schedule`); the ordering rule is a
   Rhino-free function in `hold_schedule.py`, also used by `rs_show_bar_action_plan.py:287-295`.
4. **D1.** In `build_bar_assembly_actions`, after the movements exist: if the bar is in the hold plan
   and its support keyframe is solved, place its holder at the held pose in the three release states
   and allow holder ↔ bar (`configure_robot_obstacle` + `whitelist_frozen_contact`, with the robot-name
   mismatch check). Unsolved: loud note, holder stays parked. The IK solver path is not changed.
5. Fix the wrong docstring in `rs_data_structure/hold_action.py:25-27`; re-solve the four holds
   (RSIKKeyframeAll) since stored approach poses are now checked against the held bar.

Checker rules: held bar shown in `__H`/`__HR` with `SupportGripper` allowed only in the four contact
movements; `B3__R`, `B7__R`, `B12__R`, `B15__R` show their holder at the hold pose; `B7__HR` no longer
shows Alice, `B15__HR` no longer shows Alice.

### Phase 4 — sequence and schedule bookkeeping: D4 + D8

Files: `scripts/core/rhino_bar_registry.py`, `bar_action.py`, `hold_action_builder.py`,
`rs_export_bar_action.py`, `rs_export_all_bar_actions.py`, `tests/test_hold_schedule.py`.

1. `get_real_bar_seq_map()` (bar map without fake bars) in `rhino_bar_registry.py`; use it for every
   `assembly_seq` and every hold-plan derivation (`bar_action.py:1380`, `:1676`,
   `_assembly_seq_and_grounds`, `build_action_schedule_payload`, both export scripts). The collectors
   read the fake marks from the document themselves, so a filtered map no longer prints false "orphan
   joint" notes.
2. `build_action_schedule_payload(bar_map, written=None)`: the batch passes the files it wrote; the
   single-bar refresh checks which files exist. Entries without a file are left out, listed under a new
   `"not_exported"` key, printed, and shown in the batch's final message box.

Checker rules: every `assembly_seq` equals the schedule's (no B2, B6, B11, B14); every schedule file exists.

### Phase 5 — docs and hand-off

- `docs/support_ik_spec.md` §2.2 and §5, `docs/action_movement_report.md`, `docs/action_flowchart_2026.py`
  (+ regenerated `.drawio`/`.png`), `docs/ik_keyframe_scene_reconstruction.md`.
- Issue doc: mark each D as fixed with the new file shape, and a short "what changed for readers of the
  files" list for the monitor side: release has 3 movements; ground bars have a different `__J`; ids
  should be read by name; `bar_*` names everywhere; `ground_<id>` bodies replace the monitor's
  `obstacle_ground`; `ends_on` values; `not_exported` in the schedule.

## Verification

- Unit tests after each phase:
  `external\husky_assembly_tamp\.venv\Scripts\python.exe -m pytest tests -v`
  (updated `test_bar_action_ground.py`, `test_robot_obstacles_whitelist.py`,
  `test_env_collision_duplicate_guard.py`, `test_hold_schedule.py`; new `test_hold_scene_rules.py`).
- In Rhino on the `260920_RobArch_demo_revamp` model: RSRebuildRobotCell, RSIKKeyframeAll (re-solve),
  RSExportAllBarActions into a **new local folder**, then `python tests/check_export_bundle.py <folder>`
  must be all PASS. The shared Google Drive folder is only overwritten after you tell the monitor side.
- Collision checks in live Rhino through the lamcp bridge (check `bridge_health` first): B1 insert state
  is clear with the floor present; B1 transfer state still reports a ground joint pushed into the floor;
  B3 `H_M0` and `H_M2` start states are clear; B3 release states are clear with Alice present. Watch for
  two things the plan cannot know in advance: the robot chassis or the arm tools touching the floor, and
  stored approach poses touching the now-shown held bar. Either one is reported, not silently allowed.
- RSShowBarActionPlan: step through B1 (ground), B3 (held) and the B9 releases; poses match before/after.

## Not in this plan

- Monitor (`husky-assembly-teleop`) and `husky_assembly_tamp` headless planner changes (the FYI block).
- Making Cindy's retreat IK see her own bar's holder (D1 note): a separate decision.
- Anything in `260929_phase1_retest`.
- Commits: none unless you ask; submodule edits (`rs_data_structure`) get committed there first when you do.
