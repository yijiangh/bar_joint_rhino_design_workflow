# Exporter issues found while adapting the live monitor

Started 2026-09-30 by the husky-assembly-teleop session that adapts the live monitor to the
support-robot schedule. Observed on the export `data_design_study/260920_RobArch_demo_revamp_backup`
(and identically on `260814_RobArch_support_ik`); D5–D7 come from the fake-hardware dry run of the
schedule monitor (entries 0..8: B1, B3 held by Alice, B4, B5), D8 from `260929_phase1_retest`. The
monitor works around all of these, but the exported files are wrong as stand-alone collision scenes.

Refined 2026-10-01/02 on the design side: every claim below was checked against this checkout
(`hs/mocap-experiment`), the line pointers were corrected, fix sketches were added, and the open
questions were decided by the designer. The teleop-side plan
`husky-assembly-teleop/tasks/2026-10-01_dryrun_fixes_plan.md` is not in the local teleop checkout (it
lives in the Linux checkout), so it was not read.

**Reference design problem.** Test against
`Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study/260920_RobArch_demo_revamp`
(exported 2026-09-30 13:08-13:10; its numbers match the `_backup` copy quoted below, e.g. Alice's B3
hold pose). **Do not touch `260929_phase1_retest`**: Su uses it for her tests, from her own commit that
is out of sync with this branch.

**Checked on the exported files (2026-10-02, read only).** 20 bars in the schedule (`B1, B3, B4, B5,
B7, B8, B9, B10, B12, B13, B15 ... B24, B23`); fake bars `B2, B6, B11, B14`; ground bars `B1` and `B5`
(two `T20Ground` joints each, no male joint); holds `B3` (Alice) and `B7` (Belle) released after `B9`,
`B12` (Alice) and `B15` (Belle) released after `B21`; 48 schedule entries, 48 files.

## Status

| Issue | Status (2026-10-02) | Seen in `260920_RobArch_demo_revamp`? | Decision / direction | Order |
|---|---|---|---|---|
| D1 own holder parked in `__R` | fixed in code | yes, all four held bars | stamp the holder into the release states at export | 3 |
| D2 `env_bar_*` vs `bar_*` names | fixed in code | yes | canonical names everywhere | 2 |
| D3 no static obstacles in support cells | fixed in code | no effect: the design has no environment obstacles at all | add them through `get_env_union` | 2 |
| D4 fake bars in single-bar refresh | fixed in code | yes, in every `__J`/`__R` | one filtered sequence helper | 4 |
| D5 hold scene vs approach collisions | fixed in code | yes | **keep release-time bars**, fix the allowed contacts | 3 |
| D6 no floor in the exported cell | fixed in code | yes; plus a vertical "walkable ground" WG2 | **export the floor as rigid bodies** (used grounds only) | 2 |
| D7 extra `untighten` release step | fixed in code | yes, all 20 `__R` | **remove it**: ungrasp, retreat, home | 1 |
| D8 schedule lists files never exported | fixed in code | **no**: schedule complete | schedule only from exported files | 4 |
| D9 ground bars get tighten/untighten (new) | fixed in code | yes, B1 and B5 | **no jointing motor; operator fixes the foundation** | 1 |

"Fixed in code" = implemented and unit-tested on `hs/mocap-experiment`, not committed, and NOT yet
re-exported: the files in `260920_RobArch_demo_revamp` still show every issue. Re-export (Rhino:
RSRebuildRobotCell, RSIKKeyframeAll for the four holds, RSExportAllBarActions into a new folder) and
run `python tests/check_export_bundle.py <folder> --cells`; it must print 0 failed rules.

Implementation plan: `tasks/2026-10-02_support_export_fixes_plan.md`.

Order: (1) D7 + D9 together (same builder, same consumers, one renumbering). (2) D2, then D3 and D6:
naming first, then obstacles and floor through the same route. (3) D5 and D1, both about frozen robots
and allowed contacts in hold/release states. (4) D4 and the D8 code fix, small and independent. After
each group, re-export `260920_RobArch_demo_revamp` (tell the monitor side first, it reads that folder).

---

## D1 — a held bar's `__R` (release) action parks its own holder

- **Symptom.** `BarActions/B3__R.json`: every movement's `start_state.tool_states['ObstacleRobotAlice']`
  has `frame` = (50, 50, 0) m and an all-zero configuration (parked), although in the schedule
  `B3_H_hold (Alice)` runs BEFORE `B3_R_release`. The next bar's `B4__J.json` correctly shows Alice
  at her hold pose (base ≈ (-4.51, 1.76, -0.016) m, joints ≈ [1.035, -1.612, 2.393, -0.793, 0.331, 0.012]).
  So Cindy's ungrasp/retreat/home scenes miss the robot standing at the bar she is releasing.
- **Seen in `260920_RobArch_demo_revamp`.** All four held bars: `B3__R` (Alice), `B7__R` (Belle),
  `B12__R` (Alice), `B15__R` (Belle) park their own holder in all four movements. The bars after them
  are right (`B4__J` shows Alice at the B3 hold, `B13__J` at the B12 hold, `B8__J` both holders).
- **Cause (pointers confirmed).** `scripts/core/bar_action.py:1368-1386`, inside
  `build_split_assembly_movements` (`:1278`): one `template_state` per bar is built and
  `hold_action_builder.freeze_holding_robots(template_state, hold_plan, seq, skip_unsolved=True)` is
  called. `hold_schedule.robots_holding_at_step` (`hold_schedule.py:120-141`) deliberately excludes the
  bar's own hold (rule `hold_start_seq < seq <= release_after_seq`); the code comment says "the bar's OWN
  hold ... cannot be known at solve time".
  - R_M0 is `r_m0_state = m2.start_state.copy()` (`:1553`), R_M1 is `start_state=m3.start_state.copy()`
    (`:1568`). R_M2 and R_M3 are not copies: they ARE the `m3`/`m4` movements (`_build_m3` `:1048`,
    `_build_m4` `:1148`), built from the same template. All four inherit the parked holder.
  - `BarAssemblyReleaseAction` is wrapped at `:1694-1702` in `build_bar_assembly_actions` (`:1583`).
  - The NOTE printed at `:1387-1392` is about OTHER robots' unsolved holds and covers both halves; there
    is no separate "J half" note.
  - Parked pose: `config.ROBOT_PARKED_BASE_FRAME_MM` (`config.py:338-343`), all joints zero
    (`robot_obstacles.park_robot_obstacle`, `:396-412`).
- **Already handled elsewhere.** The Rhino viewer adds the own holder for the hold, retreat and home
  poses (`rs_show_bar_action_plan.py:236-312`, `support_presence_for_step`; reused by
  `gh_seq_preview.py:423-433`). The export should match it.
- **Fix sketch.** In `build_bar_assembly_actions` (export only), if `read_bar_support_keyframe(bar_oid)`
  (`hold_action_builder.py:103-136`) is not None, stamp the four release states with
  `robot_obstacles.configure_robot_obstacle(state, robot_name, payload["base_frame_world_mm"],
  payload["held"]["joint_values"], payload["held"]["joint_names"])` and
  `robot_obstacles.whitelist_frozen_contact(state, robot_name, [bar_id])`. If not solved yet, print a
  loud NOTE and leave it parked. Three points:
  1. Stamp **after** the movements are built, and only on the release states (`r_m0.start_state`,
     `r_m1.start_state`, `m3.start_state`, `m4.start_state`). `_apply_movement_touch_policy`
     (`bar_action.py:866-878`) overwrites the active bar's `touch_bodies`, so stamping the template
     would lose the allowed contact and would also put the holder into the `__J` states (wrong: the
     hold starts after J).
  2. Do it in the export wrapper, not in `build_split_assembly_movements`: the IK solver calls that
     builder too (`rs_ik_keyframe.py:2836-2854`, `rs_ik_keyframe_all.py:220`), and nothing re-checks
     Cindy's retreat against her own bar's holder after the support solve. Making the retreat IK see the
     holder is physically right, but it is a separate decision.
  3. Repeat the robot-name mismatch check of `freeze_holding_robots` (`hold_action_builder.py:349-355`).
  - No special re-export step is needed: the batch rebuilds `__R` from user text on every run, so
    re-running the export after the support flow refreshes the file.
  - Unknown: whether Alice's gripper also overlaps the joint halves on the bar in R_M0 (the bar is still
    attached to Cindy there). If it does, the joints need the same allowed contact.
- **Same-step rule (checked).** `build_action_schedule` (`hold_schedule.py:144-189`) orders a step as
  J, H, R, then releases sorted by `hold_start_seq`. `build_release_scene_state`
  (`hold_action_builder.py:456-528`, filter at `:503`: `hold_start_seq <= release_seq <= release_after_seq`)
  agrees on two points: a hold that starts at the release step is present, and holds that release later
  in the same step are present. It disagrees on one: a hold released EARLIER in the same step is still
  frozen at its held pose. Seen in `260920_RobArch_demo_revamp`: B3 (Alice) and B7 (Belle) both release
  after B9 and the schedule runs `B3__HR` (entry 16) before `B7__HR` (entry 17), but `B7__HR` still shows
  Alice at her B3 hold (with `env_bar_B3` allowing `ObstacleRobotAlice`). Same for `B12__HR` (entry 40)
  and `B15__HR` (entry 41): `B15__HR` still shows Alice at her B12 hold. (The unit test
  `tests/test_hold_schedule.py:72-75` pins the same order.) `validate_release_confs` (`:582`) has the
  same extra robot. Safe (an extra obstacle) but not what the
  schedule says; the viewer gets it right (`rs_show_bar_action_plan.py:287-295`).
  - Also known: HR parks Cindy (rationale `config.py:333-337`) but the schedule has no "Cindy drives
    away" action between R(Y)'s home and the HR actions. And a hold that starts and ends strictly inside
    another hold's window is frozen in neither the solve scene nor the HR scene (commit `9198360`).

## D2 — support cells name bars `env_bar_*` / `env_joint_*`, Cindy's cell `bar_*` / `joint_*`

- **What the code does.** `env_collision.py:44-45` (`ENV_RB_BAR_PREFIX="env_bar_"`,
  `ENV_RB_JOINT_PREFIX="env_joint_"`) vs `:49-50` (`CANONICAL_BAR_PREFIX="bar_"`,
  `CANONICAL_JOINT_PREFIX="joint_"`). `collect_built_geometry` (`:371`, `:419`), the only collector for
  support cells, uses the `env_` names; `collect_assembly_geometry` (`:497`, `:544`) uses the canonical
  ones. So `RobotCell_Alice.json`/`RobotCell_Belle.json` and every H/HR state key bars as `env_bar_B1`,
  while `RobotCell.json` and J/R states use `bar_B1`. `whitelist_frozen_contact`
  (`robot_obstacles.py:365-386`) probes both names. Seen in `260920_RobArch_demo_revamp`: every
  `__H`/`__HR` uses `env_bar_*`/`env_joint_*`, every `__J`/`__R` uses `bar_*`/`joint_*`.
- **Why the split exists.** Left over from an older design: names used to be prefixed in memory and made
  canonical on export (`canonical_rb_name`, removed in `f367ea5`, 2026-06-15, when Cindy's cell became
  canonical). The support pipeline came later (`dfa0c88`) and reused the old collector. No cell ever
  holds the same bar under both names (the held bar is excluded or hidden in support cells, never
  attached), so the split is accidental.
- **Fix sketch.** Add `bar_body_name(bar_id)` / `joint_body_name(tag)` in `env_collision.py` and use them
  everywhere:
  - `collect_built_geometry` (`:371`, `:419`) and the two stale-body scans in `register_env_in_robot_cell`
    (`:754-757`) and `build_env_state` (`:803-806`);
  - reduce `whitelist_frozen_contact` to one name;
  - remove the duplicate prefix constants in `bar_action.py:281-282` and `rs_select_joint.py:54`;
  - update `tests/test_robot_obstacles_whitelist.py`, `tests/test_env_collision_duplicate_guard.py`,
    `docs/ik_keyframe_scene_reconstruction.md:229,695`.
  - Trap: delete the unused `robot_cell.ensure_env_registered` (`robot_cell.py:457-488`, no callers).
    After the rename its stale scan would remove Cindy's canonical bodies.
  - This renames bodies in exported files, so the monitor must drop its special case at the same time.

## D3 — support cells carry no static environment obstacles

- **What the code does.** `robot_cell.py:792-793` is the only call of
  `env_collision.collect_environment_geometry()` (`obstacle_*` bodies from `LAYER_ENVIRONMENT`), for
  Cindy's cell. `robot_cell_support.get_or_load_support_cell` (`:97-141`) and
  `hold_action_builder.ensure_support_env_registered` (`:228-240`) register bars/joints only, so Alice's
  and Belle's approach/retreat plans are never checked against walls/tables/mocap tripods.
- **Not seen in `260920_RobArch_demo_revamp`.** No state in any of its 48 files has a body other than
  bars and joints, so its environment layer is empty and Cindy's cell has no obstacles either. The
  issue is real in the code but does not change any file of this problem until obstacles are modelled
  (e.g. the mocap tripods). Test the fix on a copy of the problem with one obstacle added.
- **Fix sketch.** Every support path (interactive and batch IK, `validate_release_confs`, the H/HR
  builders, the batch export) goes through `hold_action_builder.get_env_union` (`:208-225`), so add the
  obstacles there. Three follow-on changes:
  1. `build_support_scene_state` (`:277`, hiding at `:297-299`) hides every body not in the visible set;
     it must keep environment bodies shown.
  2. Add `OBSTACLE_PREFIX` to the stale-body scans (`env_collision.py:754-757`, `:803-806`), or deleted
     obstacles pile up in the support cells.
  3. Take the obstacle bodies from the assembly cell's snapshot (`robot_cell.ensure_assembly_cell`)
     instead of calling `collect_environment_geometry()` again. That function builds new objects on every
     call (`:699`), so the "already registered" identity check (`:767`) would always fail and every
     support command would re-send the whole cell. Using the snapshot also keeps both cells' obstacles
     identical and under the same staleness check (`_live_assembly_fingerprint` already counts
     `LAYER_ENVIRONMENT`, `robot_cell.py:733-737`).
  - State shape: `ik_collision_setup.build_full_assembly_state` (`:191-207`) writes obstacles shown, with
    empty touch lists; `env_collision.build_env_state` (`:781-830`) already writes the same shape.

## D4 — fake bars leak into single-bar schedule refreshes (wider than first written)

- **What a fake bar is.** A bar curve with user text `scaffolding.fake_bar == "1"`
  (`rhino_bar_registry.py:55-69`, `is_fake_bar` / `get_fake_bar_ids` `:573-597`): a staging bar that only
  gives a real bar's male joint something to be modelled against. It keeps its id and sequence number;
  the collision collectors drop it (`env_collision.py:348-355`, `:485-489`, `:521`).
- **Where it leaks.**
  - `rs_export_all_bar_actions.py:108-110` filters before building `ActionSchedule.json`;
    `rs_export_bar_action.py:148` passes the unfiltered `get_bar_seq_map()` to the hold plan (`:150`) and
    to the schedule refresh (`:226`).
  - Each action's own `assembly_seq` (`bar_action.py:1676-1679`) is unfiltered, so even the batch is
    inconsistent with itself: `__J`/`__R` list fake bars, `__H`/`__HR` and the schedule do not. Seen in
    `260920_RobArch_demo_revamp`: every `__J`/`__R` lists 24 bars including `B2, B6, B11, B14`; the
    `__H`/`__HR` files and `ActionSchedule.json` list the 20 real bars.
  - `hold_action_builder._assembly_seq_and_grounds` (`:760-767`) is a fourth `assembly_seq` builder;
    filtered in the batch, unfiltered in the single-bar export.
  - The builder derives its own hold plan from the unfiltered map (`bar_action.py:1380-1383`). If a
    `supported_until` names a fake bar, `derive_hold_plan` raises in the batch but not in the single-bar
    path, so the two plans can differ.
- **Fix sketch.** One `get_real_bar_seq_map()` in `rhino_bar_registry.py` used by all of the above and by
  `build_action_schedule_payload` (`hold_action_builder.py:1019-1023`). Keep giving the collision
  collectors the full map: they drop fake bars themselves, and a filtered map makes
  `collect_built_geometry` print a misleading "orphan joint" warning (`env_collision.py:435-445`).

## D5 — hold actions (`__H`, `__HR`) collide at the start of the hold

- **What the files contain.** `B3__H.json` (Alice grabs B3 right after Cindy jointed it): every
  movement's `start_state` shows as built the bars `env_bar_B1, B4, B5, B7, B8, B9` and their joints,
  i.e. everything that exists when the hold is RELEASED (after B9), not only what exists when Alice
  approaches (B1 and B3). The held bar `env_bar_B3` itself is hidden. `ObstacleRobotCindy` is frozen at
  her B3 assembled pose. Seen identically in `260920_RobArch_demo_revamp` (Cindy's base at
  (-5.15, 3.33, -0.02) m, Belle parked). No rigid body in any `__H` state has an allowed contact at all,
  so frozen Cindy, standing where she jointed B3, may touch nothing. B5, the body she was reported to
  hit, is the second ground bar of the design.
- **Where.** `build_bar_holding_action` (`hold_action_builder.py:803-912`, scene at `:834-849`) and
  `build_bar_holding_release_action` (`:915-987`, scene from `build_release_scene_state`). Both take the
  bars from `collect_hold_window_geometry` (`:243-274`): bars up to and including the release step, minus
  the held bar and its joints. So H and HR have the same bars and differ only in which robots are shown
  (H: Cindy at her assembled pose, other holders at grasp time; HR: Cindy parked, holders at release
  time). The held bar is hidden because "the gripper is wrapped around it, so its tube would always
  false-positive" (`:255-256`). The hold IK solve uses the same scene as `__H` (interactive
  `rs_ik_keyframe.py:2517-2549`, batch `resolve_support_keyframe_noninteractive`
  `hold_action_builder.py:662-681`).
- **Symptom in the monitor.** Planning Alice's approach fails before it starts:
  `MPStartStateInCollisionError ... CC.5 between tool 'ObstacleRobotCindy' and rigid body 'env_bar_B5'`
  (H_M2 linear approach) and `start_or_goal_in_collision` (H_M0 free approach): Cindy, standing where she
  was when she jointed B3, overlaps bar B5, which is not built yet at that moment. The only workaround
  today is to switch off collision checking against the whole built structure, which also hid the built
  bars in the view (worklog 2026-10-01: "no built bars are properly rendered").
- **Decision (designer, 2026-10-01).** Keep the release-time bars for both `__H` and `__HR`, so the held
  pose stays checked against every bar built during the hold. Fix the allowed contacts instead:
  1. **Show the held bar.** It is already built when the support robot arrives, so it is a real
     obstacle for the free approach. Allow the gripper (`SupportGripper` / the support robot's links) to
     touch it only from the linear approach on (`H_M2_LM_to_grasp`, `H_M3_gripper_close`,
     `HR_M0_gripper_open`, `HR_M1_LM_retreat`), not in `H_M0_free_to_approach` / `H_M1_gripper_open`.
     Frozen Cindy, who still grips the bar during `__H`, is allowed to touch the bar and its joints.
  2. **Frozen robots vs bars built later** (inferred on the design side, to confirm). Item 1 alone does
     not remove the reported collision (Cindy vs B5). In `__H`, a frozen robot shown at its grasp-time
     pose (Cindy, other holders) must be allowed to touch the bars and joints built AFTER the hold
     starts, because they never exist at the same time as that pose. In `__HR` no such allowance is
     needed: Cindy is parked and the other holders are shown at release time.
  3. Build the IK solve scene with the same function so solve and export keep matching.
  - Also fix `external/rs_data_structure/rs_data_structure/hold_action.py:25-27`, whose docstring says
    the hold scene is the held bar's own assembly step (grasp time), and update the `__H` docstring
    (`hold_action_builder.py:806-811`), `docs/action_movement_report.md:222-231,332-336`,
    `docs/ik_keyframe_scene_reconstruction.md:722`.
  - Unverified: with collision checking on, the current solve scene (release-time bars + Cindy at B3)
    should also have rejected every candidate if Cindy really overlaps B5. So B3's hold was likely solved
    with collision checks off, or Cindy's B3 keyframe or the sequence changed after the support solve
    (the export re-reads the current assembly keyframe at `:838` and never collision-checks H states).
    Re-solve B3's hold after the fix. This can be settled on `260920_RobArch_demo_revamp` by loading
    `B3__H` H_M2's start state into `RobotCell_Alice.json` and running the collision check once, which
    would also confirm whether rule 2 above is needed.
- **Movement ids for reference.** H: `H_M0_free_to_approach`, `H_M1_gripper_open`, `H_M2_LM_to_grasp`
  (100 mm), `H_M3_gripper_close`. HR: `HR_M0_gripper_open`, `HR_M1_LM_retreat`. Suffix map
  `SCHEDULE_KINDS` (`hold_action_builder.py:995-1000`).

## D6 — no floor in the exported cell, so ground joints have no allowed contact with it

- **Symptom.** Loading `B1_J_M5_LM_insert` (Cindy lowers ground bar B1 to its assembled pose) the
  collision check reports
  `CC.4 between attached rigid body 'joint_G1-T20Ground-1_ground' and rigid body 'obstacle_ground' - COLLISION`
  (same for `...Ground-0_ground`). The ground joints travel with the bar (`attached_to_link`) and at the
  assembled pose they stand on the ground, which is exactly where they should be.
- **Why.** The floor is not part of the exported `RobotCell*.json` at all. Only `WalkableGround.json` is
  written (`rs_export_all_bar_actions.py:257-269`: `{"grounds": {WG<n>: compas Mesh}}`, in **mm**), read
  by `husky_assembly_tamp/keyframe/walkable_ground.py:156-168` for base sampling only. The monitor turns
  it into a rigid body it names `obstacle_ground`, with allowances it invents itself. So any other
  consumer (headless planner, Rhino replay) has no floor. The code says so too: `bar_action.py:699-704`,
  `:820-822` ("the floor is not collision geometry today"), and the open item in `todos.md:14-19`.
  Ground joints' allowed contacts are `[their arm tool, bar_B1]` (`_apply_movement_touch_policy`,
  `bar_action.py:680-879`, ground branch `:820-839`).
- **Seen in `260920_RobArch_demo_revamp`.** `B1__J` and `B5__J`: each ground joint is attached to its
  arm's flange with `touch_bodies = [AT3L | AT3R, bar_B1 | bar_B5]` in every movement, and no state has
  a floor body. The insert is only 15 mm (`lm_distance_mm`). Every action names `walkable_ground_ids =
  ["WG0"]`. `WalkableGround.json` holds three single-face grounds:
  - `WG0`: flat at z = -15.55 mm, x -14319..16889, y -1220..6422 (the one in use);
  - `WG1`: flat at the same height, x -12808..21092, y 657..6927 (overlaps WG0);
  - `WG2`: a **vertical** plane at y = 6543 mm, z -3135..3135 (a wall, not a floor).
- **Decision (designer, 2026-10-01).** Export the walkable ground as rigid bodies of every cell, with the
  allowed contacts written by the export, so every consumer shares one floor and one contact list.
- **Fix sketch.**
  - Geometry: a `collect_walkable_ground_geometry()` that loops `get_all_walkable_grounds()`
    (`rhino_walkable_ground.py:126-139`, layer `LAYER_WALKABLE_GROUND`, `config.py:409`) and reuses
    `env_collision._env_object_to_compas_mesh` (already outputs metres). Name bodies per ground id, e.g.
    `ground_WG0`. Not `obstacle_ground`: an object named "ground" on the environment layer already
    produces that name (`_sanitize_obstacle_name`, `env_collision.py:700`).
  - Cell: merge at `robot_cell.py:793`, add the prefix to the managed list (`:825-829`), add the walkable
    layer to `_live_assembly_fingerprint` (`:733-737`). Support cells get it through the D3 route.
  - Allowed contacts per state:
    - the four wheel links (`front_left_wheel_link`, `front_right_wheel_link`, `rear_left_wheel_link`,
      `rear_right_wheel_link`, same names in all URDFs) in every state, on the floor's `touch_links`;
      also check whether the chassis collision mesh comes close to the ground;
    - the frozen-robot tools (`config.OBSTACLE_TOOL_NAMES`) in every state, on the floor's `touch_bodies`;
    - ground joints only during the insert and the operator's fix step (see D9), and once the bar is part
      of the structure; not during the transfer, so a ground joint scraping the floor on the way is still
      caught.
  - Caveat: each collision mesh is loaded as one convex shape (`compas_fab` `client.py:357-398`), so a
    flat ground is fine but a stepped ground must be split into flat pieces. The grounds are single
    faces with no thickness; give each body a thin slab (e.g. 10 mm, top face at the surface) so the
    collision check has a real solid.
  - **Decided (designer, 2026-10-02): only the walkable grounds the actions actually use become floor
    bodies**, i.e. the union of every real bar's `walkable_ground_ids` (here only `WG0`). The vertical
    `WG2` and the unused `WG1` stay out of the collision scene.
  - Keep `WalkableGround.json` (the base sampler reads it). The monitor drops its own `obstacle_ground`
    once the export carries the floor.

## D7 — the release action starts with an "untighten" step that should not exist

- **What the files contain.** `B<n>__R.json` = `R_M0_tool_untighten_joint`, `R_M1_tool_ungrasp_bar`,
  `R_M2_LM_retreat`, `R_M3_free_home` (`bar_action.py:1552-1574`; R_M0 is "Jointing screws untighten
  (bar still gripped)", `tool_action="untighten"`). Seen in all 20 `__R` files of
  `260920_RobArch_demo_revamp`.
- **Problem.** Worklog 2026-10-01: "Here there should not be an untighten step, only an ungrasp step".
  The tightened jointing screw is what keeps the joint, so running it backwards would undo the joint
  that was just made.
- **Where it comes from.** `todos.md:90-93`, the original prompt, describes ONE step: "both tools
  activate the untighten screws to ungrasp the grasped joint and bar". `docs/support_ik_spec.md` §2.2
  (`:36-40`) turned it into "run the screws backwards to let go of the joint and the bar", and the code
  split it into R_M0 `untighten` + R_M1 `ungrasp`.
- **Decision.** Release = three movements: `R_M0_tool_ungrasp_bar`, `R_M1_LM_retreat`, `R_M2_free_home`.
  This renumbers the movement ids of every `__R` file.
- **Spots that key on the old numbering.**
  - `write_bar_keyframe_from_action` (`bar_action.py:531-537`): assembled = R_M0, R_M1 or R_M2; retreat
    = R_M3. Retreat would silently stop syncing.
  - `rs_show_bar_action_plan.py:679` (roles `J_M3`/`J_M5`/`R_M2`/`R_M3`) and `release_mvts["M0"]` as the
    preview's assembled pose (`:854-911`).
  - `rs_ik_keyframe.py:2850-2853`, `rs_ik_keyframe_all.py:230-232` (`release_mvts["M2"/"M3"]`).
  - `tests/test_bar_action_ground.py` pins the tool sequence.
  - Suggest looking movements up by the end of their name (`LM_insert`, `LM_retreat`, `free_home`)
    instead of the number, because D9 renumbers `__J` for ground bars too.
- **Docs to update with it.** `support_ik_spec.md` §2.2, `action_movement_report.md:107-135`,
  `action_flowchart_2026.py:218-219` (and the `.drawio`/`.png`), the `BarAssemblyReleaseAction`
  docstring (`external/rs_data_structure/rs_data_structure/bar_action.py:387`).
- **Monitor side.** The monitor already treats `untighten` as a no-op and folds it away; after the fix it
  no longer appears.

## D8 — `260929_phase1_retest`: the schedule lists files that were never exported

- **What the files contain.** `260929_phase1_retest/ActionSchedule.json` lists 180 Cindy entries (90 J +
  90 R), but `BarActions/` holds 60 files (30 `__J` + 30 `__R`): 120 referenced files are missing.
- **Why it matters now.** The monitor is becoming schedule-only (the old BarAction file list is being
  removed, user decision 2026-10-01). A schedule that names missing files is refused with one error, so
  this problem — the one the bar-holding accuracy test uses — cannot be opened until the export is
  complete.
- **Not seen in the reference problem.** `260920_RobArch_demo_revamp/ActionSchedule.json` has 48 entries
  and all 48 files exist. Use that problem for the monitor's schedule-only work.
- **Do not touch `260929_phase1_retest`.** Su uses it for her tests, from her own commit that is out of
  sync with this branch; do not re-export or edit it from here. The missing files are hers to sort out
  (or to be regenerated by her once her tests are done).
- **Possible cause in the code (design side, not checked on that data).** The batch export logs and
  skips any bar whose action fails to build (for example a bar that fails the tool gate: no joints, one
  anchor, only females; `rs_export_all_bar_actions.py:174-184`), but still builds `ActionSchedule.json`
  from the full bar map (`:231`). So the schedule always lists every real bar, whether or not its files
  were written. A single-bar export (`rs_export_bar_action.py:226`) refreshes the schedule from the full
  map too. The Rhino command line of that batch run should show which bars were skipped and why.
- **Fix wanted (code only).** Build the schedule only from the bars that actually exported, and print a
  loud summary of the skipped bars (or stop the batch with a clear error, per "no silent fallbacks").

## D9 (new, design side) — ground bars are exported with tighten / untighten steps they do not have

- **Design rule (designer, 2026-10-01).** A bar assembled onto ground joints (a foundation) has no
  male-female joint, so the jointing motor never runs. The dual arm only grasps the bar and follows the
  insert motion. At the end of the insert, a human operator comes in and tapes or fixes the foundation to
  the ground. Then the robot lets go and leaves as usual.
- **What the export does today.** A ground bar gets the same 6 + 4 movements as every other bar:
  - `J_M4_tool_tighten_joint` (`tool_action="tighten"`, both tools, `bar_action.py:1544-1551`) and
    `R_M0_tool_untighten_joint` (`:1552-1562`);
  - `J_M5_LM_insert` always says `ends_on = "tool_stall_signal"` (`:1043`) and runs on the compliant
    controller because "the J_M4 tightening screws keep running through it and their stall signal ends
    it" (`:1028-1031`). For a ground bar that signal never comes.
  - The only ground-specific parts are the insert axis (ground normal, `:1416-1424`), the attachments
    (`:645-651`), the allowed contacts (`:820-839`) and the retreat direction (`:1093-1111`). There is no
    ground-bar flag (ground joints are found by layer, `_classify_ground_joints_per_arm` `:364-395`) and
    no operator step.
  - Seen in `260920_RobArch_demo_revamp`: the two ground bars `B1` and `B5` (each with
    `G<n>-T20Ground-0/1`, no male joint; their female halves for later bars ride along) both export
    `J_M4_tool_tighten_joint` on `AT3L`+`AT3R`, `J_M5_LM_insert` with `ends_on: tool_stall_signal` on
    the `cartesian_compliant` controller, and `R_M0_tool_untighten_joint`. No bar in this design mixes a
    ground joint with a male joint, which matches the rule below.
  - History: the 2025 draft flowchart (`docs/action_flowchart_draft_2025.drawio.xml`) had this branch:
    "if foundation bar → Manual fix foundation standoff → Cindy opens gripper". The 2026 redesign dropped
    it; `docs/action_flowchart_2026.py:286-289` lists it as "Not modeled (yet)".
- **Target shape for a ground bar.**
  - `__J`: `J_M0_free_to_load`, `J_M1_manual_mount_bar`, `J_M2_tool_grasp_bar`,
    `J_M3_CDFM_transfer_to_approach`, `J_M4_LM_insert` (ends when the target is reached, no tighten
    step), `J_M5_manual_fix_foundation` (a `ManualMovement`, same kind as `J_M1`, instruction in its
    `tag`; the robot keeps holding the bar while the operator works; its state = the insert's end).
  - `__R`: ungrasp, retreat, home (same as D7).
  - Normal bars keep `J_M4_tool_tighten_joint` + `J_M5_LM_insert` unchanged.
- **Mixed bars are not valid designs.** A bar with one ground joint and one male joint does not happen in
  the designs. Today the tool gate accepts it (`ik_collision_setup.resolve_arm_tools_on_bar`, `:36-98`;
  `rs_ik_keyframe._resolve_arm_tools_on_bar`, `:291-381`, whose docstring allows "one male + one
  ground"), and any ground joint switches the whole bar to the ground-normal insert. Change both gates to
  raise a clear error naming the bar.
- **Fix sketch.** In `build_split_assembly_movements` (`bar_action.py:1523-1574`), branch on
  `arm_to_ground`: skip the tighten movement, renumber the insert to `J_M4_LM_insert`, add
  `J_M5_manual_fix_foundation` after it, and set the insert's `ends_on` to reaching the target. Open
  point for the monitor side: whether the ground insert stays on the compliant controller.
- **Who is affected.**
  - The monitor: its old insert ends only when both jointing motors stall (300 s fallback,
    `husky_world.py:2670-2694, 2847-2851` in the old local checkout), so it must read `ends_on`, and it
    must show the manual step as an operator pause.
  - `husky_assembly_tamp` does not read tool actions; its role numbering is already broken (see FYI).
  - In-repo consumers: same list as D7.
  - Docs: `support_ik_spec.md`, `action_movement_report.md` §5 (`:417-419`, says ground bars only differ
    in what the tools grasp), `action_flowchart_2026.py:283-289`.
- **Ties to D6.** The ground joints may touch the floor during `J_M4_LM_insert` and
  `J_M5_manual_fix_foundation`, and after, when the bar is part of the structure.

## What changes for readers of the exported files (monitor side)

Once the problem is re-exported with the fixed code:

- **Release (`__R`) has 3 movements**: `R_M0_tool_ungrasp_bar`, `R_M1_LM_retreat`, `R_M2_free_home`.
  No `untighten` anywhere.
- **Ground bars' `__J`** (B1, B5 here): `J_M0..J_M3` as before, then `J_M4_LM_insert` with
  `notes["ends_on"] == "target_reached"` and `J_M5_manual_fix_foundation` (a `ManualMovement`: an
  operator pause while the robot holds the bar). Normal bars keep `J_M4_tool_tighten_joint` +
  `J_M5_LM_insert` with `ends_on == "tool_stall_signal"`. Read the insert's `ends_on` instead of
  assuming the screw stall, and find movements by the name after the number, not by the number.
- **One body naming in every cell**: `bar_<id>`, `joint_<jid>_<sub>`, `obstacle_<name>`,
  `ground_<id>`. The `env_bar_*` / `env_joint_*` special case can go.
- **The floor is in every cell and state**: `ground_WG0` here (only the grounds some bar uses), a
  50 mm slab under the surface, with the wheels in `touch_links` and every `ObstacleRobot*` in
  `touch_bodies`; ground joints list it in their `touch_bodies` during the insert and the fix step.
  The monitor's own `obstacle_ground` and its allowances can go. `WalkableGround.json` is unchanged.
- **Hold files**: the held bar is shown; `SupportGripper` is allowed on it in `H_M2`, `H_M3`,
  `HR_M0`, `HR_M1` only; frozen robots carry the allowances described in D5. No "ignore built-bar
  collisions" switch should be needed for the hold approach.
- **Release states of a held bar** show its own holder at the hold pose (D1); a hold released
  earlier in the same step is gone from later `__HR` scenes.
- **`assembly_seq`** lists the real bars only, in every file; `ActionSchedule.json` names only files
  that exist and lists the rest under a new `not_exported` key.

## FYI (not exporter bugs, but they bite the same data)

- `external/husky_assembly_tamp/scripts/headless_bar_action_planner.py` (still true at the current pin
  `68ecb6c`) cannot read the split files:
  - `_ROLE_RE = r"_M([0-9])_"` (`:291`) reads `B6_J_M3_CDFM_transfer_to_approach` as old "M3";
  - `--all` (`:2026-2040`, `:2640-2658`) picks up `__J/__R/__H/__HR` as separate actions;
  - `home_conf12_from_action` (`:259-288`) looks for "M4": in `__J` that finds the tighten step, which
    has no target, and `__R` has no M4. The home pose is now the target of `R_M3_free_home` (`R_M2`
    after D7);
  - its sidecar name `B6__J.solved_keyframe.json` (`:796-819`) does not match what
    `core/solved_action_cache.py:24-26` / `rs_load_solved_bar_action.py:112` expect
    (`B6.solved_<kind>.json`).
- The Rhino loader is not split-aware either: `rs_load_solved_bar_action.py:147-149` keeps one action
  per bar, while `write_bar_keyframe_from_action` needs the insert from `__J` and the release poses from
  `__R`. Both sides need one agreed sidecar naming: either the planner merges J+R into
  `B6.solved_<kind>.json`, or the loader reads `B6__J`/`B6__R` sidecars as a pair.
