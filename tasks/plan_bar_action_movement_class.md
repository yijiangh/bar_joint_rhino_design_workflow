# BarAssemblyAction — schema, semantics & motion-planning hand-off

> Status: implemented (2026-05-11). This document was originally a build plan; rewritten as the spec for the downstream motion-planning consumer. Items marked **DEVIATION** moved away from the original plan during implementation — listed explicitly so a future agent doesn't read stale guidance.

## 1. Context

The Rhino IK pipeline (`RSIKKeyframe`) already produces and persists per-bar approach + assembled joint configurations on the bar curve user-text (`KEY_ASSEMBLY_BASE_FRAME`, `KEY_ASSEMBLY_IK_APPROACH`, `KEY_ASSEMBLY_IK_ASSEMBLED`). Downstream we feed a dual-arm motion planner that has no access to Rhino.

`RSExportBarAction` (button in `scaffolding_toolbar.rui`, macro GUID suffix `00000000001d`, script `scripts/rs_export_bar_action.py`) produces one JSON per bar at `<root>/BarActions/<bar_id>.json` describing the four robot movements of one bar's assembly:

| # | Motion class | Description |
|---|---|---|
| M1 | `RoboticDualArmConstrainedMovement` | Both arms gripping bar, home → approach. Relative `tool0_left↔tool0_right` transform fixed. |
| M2 | `RoboticLinearMovement` | Both arms drive grasped male joints into mating env females (linear in tool0). |
| M3 | `RoboticLinearMovement` | Robot releases bar; each arm independently retreats `LM_DISTANCE` along its joint's local −Z. |
| M4 | `RoboticFreeMovement` | Both arms unconstrained, no grasp, return to `HOME_CONFIG`. |

Companion exports:
- `RSExportRobotCell` (`scripts/rs_export_robotcell.py`, macro suffix `1b`) — writes `<root>/RobotCell.json` with canonicalized rigid-body names.
- The `RSExportRobotCellState` button **was removed**: the BarAction superseded it. Don't try to import `rs_export_robotcell_state` — it no longer exists.

## 2. File layout produced

```
<root>/
  RobotCell.json              # compas_fab.RobotCell, RB names canonicalized (see §4)
  BarActions/
    <bar_id>.json             # BarAssemblyAction, one per bar
  Trajectories/               # reserved (empty) for the consumer to write back into
```

## 3. Schema

Implemented in `scripts/core/bar_action.py`. All classes subclass `compas.data.Data`; the class type IS the discriminator (no `motion_kind` string field). compas's JSON encoder stamps each record with `dtype = "core.bar_action/<ClassName>"`. On `compas.data.json_load`, `cls_from_dtype` does `__import__("core.bar_action", fromlist=[...])` and instantiates via `cls(**data)` (default `__from_data__`).

```python
class Movement(Data):
    movement_id: str                       # e.g. "B5_M1_CDFM_home_to_approach"
    tag: str                               # human-readable
    start_state: RobotCellState | None     # full state (compas_fab); NOT a diff
    target_ee_frames: dict | None          # {"left": Frame, "right": Frame} in METERS
    target_configuration: Configuration | None
    trajectory: JointTrajectory | None     # reserved for the consumer's planner output
    notes: dict                            # free-form metadata (motion-class specifics)

class RoboticFreeMovement(Movement):                ...
class RoboticLinearMovement(Movement):              ...
class RoboticDualArmConstrainedMovement(Movement):  ...   # M1's class

class Action(Data):
    action_id: str
    tag: str
    movements: list[Movement]

class BarAssemblyAction(Action):
    active_bar_id: str                     # the bar this action assembles
    assembly_seq: list[str]                # **DEVIATION**: full ordered list of bar ids
```

**DEVIATION — `assembly_seq` is a list, not an int.** Original plan had `assembly_seq: int` (= position index). It's now the **complete ordered list of bar ids** sorted ascending by Rhino's `bar_seq` user-text. The consumer can derive the active bar's position via `assembly_seq.index(active_bar_id)` and use the prefix `assembly_seq[:idx]` as the set of "already-built env" bars.

## 4. Rigid-body naming (state-independent / canonical)

**DEVIATION — names are canonical on disk, prefixed in memory.** Upstream `env_collision` uses state-dependent prefixes (`active_bar_<bid>` / `env_bar_<bid>` / `active_joint_<jid>_<sub>` / `env_joint_<jid>_<sub>`). On export, prefixes are stripped:

```
active_bar_B6       -> bar_B6
env_bar_B3          -> bar_B3
active_joint_J3-6_male  -> joint_J3-6_male
env_joint_J1-2_female   -> joint_J1-2_female
AssemblyLeftArmToolBody -> AssemblyLeftArmToolBody   (pass-through)
```

Implemented in `core.bar_action`:
- `canonical_rb_name(name)` — pure string transform; idempotent.
- `canonicalize_state(state)` — mutates `state.rigid_body_states` keys AND remaps every `touch_bodies` entry via `canonical_rb_name(t)` regardless of whether `t` was a state key (catches dangling cross-refs from tool RBs to bar/joint bodies).
- `canonical_rigid_body_models(rcell)` — returns NEW dict; source untouched.
- `dump_cell_canonical(rcell, fileobj)` — swap-dump-restore so the cached in-Rhino `rcell.rigid_body_models` keeps the upstream prefixes (`env_collision.register_env_in_robot_cell`'s prefix scan depends on them).

Why the rename only happens on the export copies: if the cached rcell were mutated to canonical names, `register_env_in_robot_cell` would not recognize them as "stale env" on the next call, and the cell would accumulate duplicate bodies (one under canonical, one re-added under active_*/env_*). So the in-Rhino session stays prefixed; the on-disk artifacts are canonical.

Collision invariant: for any joint id `J`, the male and female halves end up on at most one bar each (a joint half is unique). So `canonical_rb_name` never produces a key collision in practice. Both `canonicalize_state` and `canonical_rigid_body_models` raise `RuntimeError` if they detect one.

**State-independent cell = full-assembly cell + full-assembly states.** `compas_fab.RobotCellState.assert_cell_state_match` requires the state's `rigid_body_states` key-set to EXACTLY equal the cell's `rigid_body_models` key-set. So:
- `RobotCell.json` carries EVERY bar (`bar_<bid>` for all bars in `assembly_seq`) + EVERY joint half (`joint_<jid>_<sub>`) + the two arm-tool RBs. (Implemented by `core.bar_action._register_full_assembly_geom`, which calls `env_collision.collect_built_geometry(last_bar)` ∪ `collect_active_geometry(last_bar)` = the full assembly, then `robot_cell.ensure_env_registered`, before `dump_cell_canonical`.)
- Every `Movement.start_state.rigid_body_states` therefore also carries that same full key-set. Bars/joints not yet built at that movement's assembly stage (`seq > active_seq`, or a joint half mounted on such a bar) are present as `RigidBodyState(frame=<final assembled pose>, is_hidden=True)` — compas_fab skips hidden bodies for collision and visualization, so they cost nothing but keep the key-sets equal.

Consumer note: a `RigidBodyState` with `is_hidden=True` is a not-yet-built workpiece. Ignore it for planning; do not collision-check against it; do not render it. It becomes a normal env body in the BarAction whose `active_bar_id` builds it (and in every later bar's action).

## 5. Per-movement details

All four movements share the same `RobotCellState`-bearing `start_state` shape:
- `state.robot_base_frame` — saved base frame in meters (Frame).
- `state.robot_configuration` — left + right arm joint values (or `None` for M4 only).
- `state.tool_states` — wired by `robot_cell.configure_arm_tool_rigid_body_states`. The left/right arm tools have `attached_to_group` set to their planning group + wrist touch_links.
- `state.rigid_body_states` — canonical-keyed (see §4); contains the FULL assembly key-set in every movement. Per-stage classification:
  - **built env** (`seq < active_seq`): `frame` = final assembled world pose (Frame, meters), `attached_to_*` = None, `is_hidden` = False. Real static obstacle.
  - **active** (the bar being assembled now + its joints): M1/M2 → `attached_to_link` set; M3/M4 → detached at final pose. See per-movement below.
  - **not-yet-built** (`seq > active_seq`, or a joint half mounted on such a bar): `frame` = final assembled world pose, `attached_to_*` = None, `is_hidden` = True. Ignore for planning.
  - arm-tool RBs (`AssemblyLeftArmToolBody`, `AssemblyRightArmToolBody`): `attached_to_link` = the matching `*_ur_arm_tool0`, wrist `touch_links` set.

Geometry references:
- `tool0_<arm>_assembled_world_mm` = 4×4 world transform of the placed tool block instance (= tool0 at IK_ASSEMBLED). Recoverable from the bar via Rhino, not stored in the action.
- Approach EE targets = `tool0_<arm>_assembled` translated by `-unit(avg(tool_z)) * LM_DISTANCE` (averaged across arms).
- Retreat EE targets = per-arm `tool0_<arm>_assembled` translated along that arm's male-joint local `-Z` axis by `LM_DISTANCE`.
- `LM_DISTANCE = 15 mm` (`core.config.LM_DISTANCE`).

### M1 — `RoboticDualArmConstrainedMovement` (home → approach, gripped)

- `start_state.robot_configuration` = HOME (joined from `core.config.HOME_CONFIG_LEFT` + `HOME_CONFIG_RIGHT`). **TODO(yh)**: replace zero-config placeholders with real safe-pose values.
- `start_state.rigid_body_states`:
  - `bar_<active_bar_id>` → `attached_to_link = "left_ur_arm_tool0"` (convention: bar parents to left arm).
  - `joint_<jid>_male` (every male joint on the active bar) → `attached_to_link = "<arm>_ur_arm_tool0"` per arm classification.
  - `joint_<jid>_female` (every female on the active bar) → `attached_to_link = "left_ur_arm_tool0"` (rigidly bonded to bar).
  - `attachment_frame = inv(tool0_<arm>_assembled_world_mm) @ body_world_at_assembled_mm`, converted to a meters Frame. **DEVIATION from original plan**: original said `inv(tool0_<arm>_at_HOME)` — but the grasp is invariant w.r.t. tool0; using the assembled-pose tool0 frame gives the *same* mathematical grasp without needing to push HOME_CONFIG into PyBullet to read a live FK. compas_fab's PyBullet backend computes the live world placement via `tool0_live_link_frame @ attachment_frame` on every state push.
- `target_ee_frames` = {`"left"`: Frame(approach left), `"right"`: Frame(approach right)} in **meters**.
- `target_configuration = None`.
- `notes = {"constraint": "fixed_relative_ee_transform", "approach_offset_mm": 15.0, "bar_arm_side": "left"}`.

### M2 — `RoboticLinearMovement` (mate)

- `start_state` = copy of M1's start_state with `robot_configuration` overwritten by `KEY_ASSEMBLY_IK_APPROACH` (deserialized from bar user-text).
- Bar/joint attachments unchanged from M1 (same `attachment_frame`s — invariant w.r.t. tool0).
- ACM (`touch_bodies`) inherits the M1/M2 ACM set by upstream `configure_active_assembly_acm` and canonicalized: mate pair `joint_<jid>_male ↔ joint_<jid>_female`, bar↔arm-tool RB, joints↔arm-tool RBs.
- `target_ee_frames` = {`"left"`: Frame(tool0_left_assembled), `"right"`: Frame(tool0_right_assembled)} in meters.
- `target_configuration = None`.
- `notes = {"lm_axis": "per_tool0_z_avg", "lm_distance_mm": 15.0, "bar_arm_side": "left"}`.

### M3 — `RoboticLinearMovement` (retreat)

- `start_state.robot_configuration` = `KEY_ASSEMBLY_IK_ASSEMBLED`.
- Bar/joints are **detached**: every `bar_<…>` / `joint_<…>` RB on the active bar has `attached_to_link = None`, `attached_to_tool = None`, `attachment_frame = None`, `frame = world_frame_at_assembled` (Frame, meters).
- ACM (`touch_bodies`):
  1. **DEVIATION from original plan** — mate pairs `joint_<jid>_male ↔ joint_<jid>_female` are STILL whitelisted (every male/female pair in the state, regardless of active/env origin). Plan originally said "remove mate pair in M3" because retreat should pull apart; in practice the just-mated halves are in static contact at M3.start and removing the whitelist produces false-positive collision flags. The whitelist remains; the planner will see clean contact at start, then no contact mid-retreat.
  2. Each just-mated `joint_<jid>_male` has the matching arm's tool RB added to `touch_bodies` (gripper adjacent to joint head as it pulls back). Implemented in `_whitelist_gripper_male_adjacency`.
  3. All other `touch_bodies` on active bodies are cleared (no bar↔tool whitelist in M3).
- `target_ee_frames`:
  - Per-arm independent retreat. For each arm `s`: read joint OCF (= block instance world xform of `joint_<arm_to_male[s]>_male`) from the action's M2.start_state, take its world `-Z` axis as the retreat direction, translate `tool0_<s>_assembled` by `axis * LM_DISTANCE` (mm), then convert to a meters Frame.
- `target_configuration = None`.
- `notes = {"lm_axis": "per_joint_neg_z", "lm_distance_mm": 15.0, "retreat_axes_world": {"left": [x,y,z], "right": [x,y,z]}}`.

### M4 — `RoboticFreeMovement` (home)

- `start_state.robot_configuration = None`. **Consumer contract: this is the "planner fills it" slot** — the consumer must derive M4.start config from M3's solved retreat trajectory's final waypoint (after the planner runs).
- `start_state.rigid_body_states` — same as M3 (bar/joints env, mates still whitelisted; tool RBs keep their default wrist touch_links).
- `target_ee_frames = None`.
- `target_configuration` = HOME (Configuration with joined left+right joint values; the HOME placeholder until `HOME_CONFIG_LEFT/RIGHT` are replaced).
- `notes = {"start_config_is_none": True, "planner_fills": "start_state.robot_configuration"}`.

## 6. State chaining contract (consumer-side)

Movements form a directed chain. The exporter writes each `Movement.start_state` from *known* inputs (Rhino user-text + canonical helpers); the consumer's planner produces the *intermediate* trajectories.

- `M1.start` is fully populated (HOME config + gripped bar).
- `M2.start` is fully populated (IK_APPROACH config + same attachments).
- `M3.start` is fully populated (IK_ASSEMBLED config + detached bodies).
- `M4.start.robot_configuration is None` — the consumer must compute M4.start config = the final waypoint of M3's planned retreat. Everything else in `M4.start` is already correct (detached bodies, ACM, tool states).

`Movement.trajectory` is the slot for the consumer's solved joint trajectory. It is `None` in the export; the consumer writes back into the same JSON to round-trip the result.

## 7. ACM convention reference

ACM lives **inside `start_state`**, not as a top-level field. Specifically:
- `state.rigid_body_states[<rb_name>].touch_bodies: list[str]` — other RB names that may touch this one without flagging.
- `state.rigid_body_states[<rb_name>].touch_links: list[str]` — robot link names that may touch this RB.
- `state.tool_states[<tool_name>].touch_links` — same idea for tool models.

Whitelists are symmetric (added on both sides). Canonical references after export:
- Mate pair: `joint_<jid>_male ↔ joint_<jid>_female` (always).
- M1/M2: bar/joint ↔ matching arm tool RB (`AssemblyLeftArmToolBody`, `AssemblyRightArmToolBody`).
- M3: just-mated `joint_<jid>_male` ↔ matching arm tool RB only.
- M4: only the mate pair persists; the gripper-adjacency whitelist is dropped.

The planner should run `check_collision(state)` on each `Movement.start_state` after pushing it into PyBullet via `planner.set_robot_cell_state(state)`. Pass = no collision; whitelisted pairs are silently skipped.

## 8. Parsing & unit conventions

```python
# Make scripts/ importable so dtype `core.bar_action/<ClassName>` resolves.
import sys; sys.path.insert(0, "<repo_root>/scripts")
sys.path.insert(0, "<repo_root>/external/compas_fab/src")  # in-repo submodule

from compas.data import json_load
rcell  = json_load("<root>/RobotCell.json")            # compas_fab.RobotCell
action = json_load("<root>/BarActions/B5.json")        # bar_action.BarAssemblyAction
for mv in action.movements:
    type(mv).__name__   # "RoboticDualArmConstrainedMovement" | "RoboticLinearMovement" | "RoboticFreeMovement"
    mv.start_state      # compas_fab.RobotCellState   (or None for explicit gap movements)
    mv.target_ee_frames # {"left": Frame, "right": Frame}  in meters, or None
    mv.target_configuration  # compas_robots.Configuration in radians, or None
    mv.notes            # dict
```

Units:
- All `compas.geometry.Frame` objects (everywhere in `start_state`, `target_ee_frames`) — **meters**, radians.
- All 4×4 transforms in `notes` (retreat axes etc.) — **unit vectors in world frame**.
- `lm_distance_mm` is in **millimeters** (explicit suffix). Anything in `notes` without a `_mm` suffix is meters/radians.

Reference test (headless verification): `python tests/debug_load_bar_action.py [<data_root>]` boots PyBullet DIRECT, loads `RobotCell.json` once, then walks every `BarActions/*.json`, pushes each `start_state` through `planner.set_robot_cell_state(...)` and `planner.check_collision(...)`. M4's `None` config is filled with HOME_CONFIG for the test.

## 9. Lessons captured during this build

These are recorded in `tasks/cc_lessons.md` and replicated here so the planner agent doesn't have to context-switch.

1. **Downstream artifact ≠ upstream refactor.** The exporter calls `core.ik_collision_setup.prepare_assembly_collision_state`, `core.env_collision.*`, `core.robot_cell.*` read-only. None of those upstream modules were modified — the Rhino IK pipeline keeps working untouched.

2. **Canonical names on export only.** The cached `rcell.rigid_body_models` keeps prefixed names because `register_env_in_robot_cell` keys its "stale env to remove" pass on prefixes. The canonical rename happens on the *export-bound* copies. See `dump_cell_canonical` (swap-dump-restore) and `canonicalize_state` (in-place mutation of per-movement state copies).

3. **compas Data dtype = first 2 module-path components.** `core.bar_action.RoboticFreeMovement` serializes as `dtype = "core.bar_action/RoboticFreeMovement"`. Headless loader works as long as `scripts/` is on `sys.path` (mirroring the in-Rhino path injection). Don't put Data subclasses any deeper than one level under `core/` or `cls_from_dtype` will fail to resolve.

4. **Grasp frame is tool0-invariant.** `attachment_frame = inv(tool0_world_at_some_config) @ body_world_at_some_config` is the same value regardless of which config you sample, as long as the bar is rigidly grasped at that config. We use the assembled-pose tool0 to compute it (already known); we do NOT need to push HOME_CONFIG into PyBullet first.

5. **Active females rigidly bonded to the bar.** Female joints on the active bar are parented to the bar's gripper arm (left, by convention) — they're part of the same rigid body as the bar, so they follow tool0.

6. **The mate whitelist stays through M3/M4.** Two halves of a snap joint are in static contact once mated; un-whitelisting them yields persistent false-positive collision flags. The whitelist persists; if the planner detects a flag, it's a real overlap somewhere else.

7. **M4.start.robot_configuration = None is a feature.** Don't substitute a placeholder during export. The consumer's planner is responsible for filling it from the M3 plan's terminal config. The headless test fills it with HOME_CONFIG only to prove the rest of the state pushes through PyBullet cleanly.

## 10. Files (final)

| Path | Role |
|---|---|
| `scripts/core/bar_action.py` | Data classes + builder + canonicalization helpers (`canonical_rb_name`, `canonicalize_state`, `canonical_rigid_body_models`, `dump_cell_canonical`). |
| `scripts/rs_export_bar_action.py` | Rhino entry point; macro GUID suffix `1d`. |
| `scripts/rs_export_robotcell.py` | Rhino cell exporter; uses `dump_cell_canonical`. Macro suffix `1b`. |
| `scripts/core/config.py` | `HOME_CONFIG_LEFT` / `HOME_CONFIG_RIGHT` (zero-config placeholders, TODO). |
| `tests/debug_load_bar_action.py` | Headless verifier. Bundles its own `_start_planner` / `_verify_state` / `_find_robot_cell_path` (originally a separate file, inlined here when `RSExportRobotCellState` was removed). |
| `scaffolding_toolbar.rui` | Toolbar definitions; `RSExportBarAction` is GUID `…1d`, `RSExportRobotCell` is `…1b`. |
| `tasks/cc_lessons.md` | All lessons listed in §9. |

## 10b. Migration note (2026-05-12)

The "state-independent cell" change (full-assembly cell + hidden future bodies in every state) invalidates older `RobotCell.json` / `BarActions/*.json` pairs. **Re-export both** with the current code: run `RSExportBarAction` for each bar, then `RSExportRobotCell` once. Any single `RSExportBarAction` already registers the full assembly onto the cached cell, so the order among bars doesn't matter, and `RSExportRobotCell` can run any time after at least one `RSExportBarAction` (the warning still applies if the cell is empty).

## 11. Out of scope (consumer follow-ups)

- **Support-robot mode.** Schema does not yet carry `actuated_robot_id`. When you wire the support arm, extend `Movement` with a new field (or new subclass like `SupportLinearMovement`) and the support cell pipeline. Mirror the existing `core.robot_cell_support` conventions for the "frozen dual-arm as articulated tool" pattern (see `tasks/cc_lessons.md` "RobotCell holds only ONE actuated robot").
- **HOME_CONFIG values.** Currently zeros; the consumer needs real safe-pose joint values. Source: ask the user; planner can also derive a workspace-safe pose via reachability sampling.
- **Trajectory write-back.** `Movement.trajectory` is a slot for `compas_fab.robots.JointTrajectory`. The consumer should write its solved trajectory back into this slot and `compas.json_dump` the updated `BarAssemblyAction` for round-trip.
