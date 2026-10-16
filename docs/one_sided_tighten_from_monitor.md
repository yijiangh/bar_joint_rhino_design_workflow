# D10 — one-sided bars: the tighten step names a tool that has nothing to screw

Written 2026-10-07 by the husky-assembly-teleop session (Linux, no Rhino) for the Windows side to
pick up. It continues the D1–D9 list in `docs/support_export_issues_from_monitor.md` but lives in
its own file; that file was not changed. Line pointers are against `hs/mocap-experiment` at
`7ffbd7a`.

## Status (2026-10-07)

| | |
|---|---|
| Monitor side | **done** (teleop `yh/multi-robot-monitor`, uncommitted): the monitor works out the mating arm(s) from the insert's start state, tightens and waits for those only, and warns when the export's tighten step names other tools |
| Exporter side | **done** (2026-10-08, branch `yh/joint_dependent_insertion_policy`): `bar_action.tighten_tools_for` + `assemble_timeline(tighten_tools=...)`, `notes["stall_tools"]` on `LM_insert`, checker rule D10, tests; a MoCap receiver counts like a Female; re-export pending |
| Data | `261006_3bar_holding_test` and `260920_RobArch_demo_revamp(_backup)` both show it; nothing needs re-exporting for the monitor to work, the re-export only removes the warning |

## Symptom

On a normal bar both of Cindy's arms hold a male joint, and both males screw into a female on an
already-built bar. On a **one-sided** bar only one of them does: the other male's female sits on a
staging ("fake") bar that is never built, so that male is only a grasp point. Example B3: the left
arm holds `joint_J1-3_male` (its female is on built B1); the right arm holds `joint_J2-3_male` (its
female is on fake bar B2).

The export still tells **both** tools to tighten: every `J_M4_tool_tighten_joint` has
`tool_names = ["AT3L", "AT3R"]`. Read literally, the right joint motor screws against nothing, and
the insert, which ends when the tighten tools stall (`notes["ends_on"] == "tool_stall_signal"`),
waits up to 30 s for a stall that never comes.

## Seen in

Read-only scan of every `__J` insert start state, both exports give the same table:

| bars | left male | right male | what should run |
|---|---|---|---|
| B3, B7 | female present | female absent (on fake B2 / B6) | tighten **AT3L** only |
| B12, B15 | female absent (on fake B11 / B14) | female present | tighten **AT3R** only |
| B1, B5 | ground joint | ground joint | no tighten step (D9, already right) |
| the other 14 | present | present | both tools (already right) |

The one-sided bars are exactly the four bars a support robot holds afterwards (`B3__H`, `B7__H`,
`B12__H`, `B15__H`). In the files, "female present" means the body `joint_<jid>_female` is in the
insert's `start_state.rigid_body_states`, and it is then also in the male's `touch_bodies`.

## Cause

- `scripts/core/bar_action.py:1761-1762`, end of `build_split_assembly_movements`:
  `# Both arm tools act together on every scaffolding-tool step.`
  `acting_tools = sorted(t for t in tool_ids.values() if t)`.
- `assemble_timeline` (`:1376`) uses that one list for all three tool steps: grasp (`:1433`),
  tighten (`:1466`) and ungrasp (`:1479`). Its docstring (`:1412`) says "both act in every tool
  step".
- The mate test the tighten step needs already exists, but only for the allowed contacts:
  `_apply_movement_touch_policy` (`:753`), M2 branch, `if female_key in env_geom:` (`:854`). It is
  the same test the monitor now uses on the exported state.

## Decision (proposed; yours to confirm)

The export says which tools screw, and the monitor follows it. No schema change: the vocabulary
is already there.

- The tighten step's `tool_names` = the tools of the arms whose male has its female in the scene.
  Grasp and ungrasp keep both tools (both arms clamp the bar).
- The insert's `notes["ends_on"]` stays `"tool_stall_signal"`, meaning the stall of the tighten
  step's tools ends it. One rule for every reader: *drive the tools the tighten step names; the
  insert ends when all of them stall; no tighten step (ground bar) = it ends at the target.*
- A non-ground bar with no mating male is not a valid design: raise, the same way
  `mixed_ground_male_error` does for a bar that mixes ground and male joints.

## Fix sketch

1. In `build_split_assembly_movements`, next to `acting_tools` (`:1762`):
   ```python
   # Only the arms whose male has its female in the scene screw a joint; the other
   # male (its female on a staging bar) is just a grasp point.
   tighten_tools = sorted(
       tool_ids[side] for jid, side in arm_to_male.items()   # {joint_id: 'left'|'right'}, :380
       if tool_ids.get(side) and joint_body_name(jid, "female") in env_geom
   )
   if not is_ground_bar and not tighten_tools:
       raise RuntimeError(f"{bar_id}: no male joint has its female in the scene; nothing to screw")
   ```
   Pass it to `assemble_timeline` as a new argument and use it for the tighten step only
   (`:1466`); update the docstring at `:1412`.
2. Optional, for a reader that looks at the insert alone: `notes["stall_tools"] = tighten_tools`
   on `LM_insert`. The monitor does not need it (it reads the tighten step).
3. Checker rule in `tests/check_export_bundle.py`, in `check_movement_lists` (`:215-246`):
   "D10 each tighten step names exactly the tools whose male has its female in the insert state".
   Reuse `ground_bars`' pattern (`:164-184`): per `__J`, take `LM_insert`'s start state, for each
   attached `joint_<jid>_male` map `attached_to_link` (`left_ur_arm_tool0` / `right_...`) to
   `AT3L` / `AT3R` and keep it when `joint_<jid>_female` is in the state; compare with the tighten
   step's sorted `tool_names`.
4. Tests: `tests/test_bar_action_timeline.py:95` pins `tighten.tool_names == TOOLS` (both); add a
   one-sided case (`tighten_tools=["AT3L"]`) and a "no mating male raises" case.
5. Docs: `docs/action_movement_report.md` (§2.2 `ToolMovement` paragraph near `:94`, the J_M4 row
   at `:136`), `docs/support_ik_spec.md:33` ("both tools start driving the jointing screws"),
   `docs/action_flowchart_2026.py:179` (J_M4 label), and the `BarAssemblyJointingAction` docstring
   in `external/rs_data_structure` (docstring only; a new dataclass field would force a submodule
   pin bump in every consumer).

## What changes for readers of the exported files

- One-sided bars' `J_M4_tool_tighten_joint` names one tool (`["AT3L"]` for B3, B7; `["AT3R"]` for
  B12, B15). Every other file is byte-identical in that field.
- Monitor: today it prints, per insert of a one-sided bar,
  `'B3_J_M4_tool_tighten_joint': the export lists tool_names ['AT3L', 'AT3R']; the monitor tightens only ['AT3L'] ... (Rhino note D10)`.
  After the re-export that warning is gone and the behaviour is the same.
- `husky_assembly_tamp` does not read tool steps: not affected.

## Verification (Windows)

- `external\husky_assembly_tamp\.venv\Scripts\python.exe -m pytest tests -v`.
- RSExportAllBarActions into a **new local folder**, then
  `python tests/check_export_bundle.py <folder>`: 0 failed rules, the new D10 rule included.
- Tell the monitor side before overwriting a shared folder. Never touch `260929_phase1_retest`.
