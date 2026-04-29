# Spec: assembly-IK env collision + visibility

Plan log for the "wire built bars/joints into IK collision world + surface what's being checked" feature. Built from session `assembly_ik` (2026-04-29). One commit.

---

## Goals

1. **Wire env collision (b)** — register built bars + their female/male joints as `RobotCell.rigid_body_models` so compas_fab CC.3/CC.5 actually evaluate them.
2. **Surface pairs (a-print)** — after each IK solve, print a CC.1–CC.5 pair-count summary (PASS / SKIPPED-by-reason / FLAGGED) so users can see what was considered.
3. **Surface env in Rhino (a-color)** — paint built bars + joints green BEFORE the collision-options prompt, leave the green after IK accept, reset on cancel/reject.
4. **Replay parity** — `debug_ik_collisions.py` and `debug_ik_support_collisions.py` show env collision bodies in the PB GUI, tinted green to match Rhino.
5. **Capture format v2** — flatten capture into `RobotCell` (saved once, by hash) + `RobotCellState` (per run) + IK-target header. **No v1 fallback** — existing captures are easy to regenerate; carrying a fallback branch is dead weight.
6. **ACM seam** — expose `allow_active_joint_touch(state, joint_rb_name, arm_side)` as a stub for the future "active male joint" wiring, scoped to tool whitelisting only (NOT wrist).

## Non-goals

- Active-bar male/female joints in the IK collision world. Pending Victor's joint sequencing wiring; the seam is exposed but unused.
- Bar-being-placed modelled as `attached_to_tool` rigid body (CC.4). Future.
- Migration of pre-v2 captures. v1 captures are deleted/regenerated at the same commit that lands this change. No fallback branch.

---

## Design corrections from user (this session)

1. `allow_active_joint_touch` whitelists ONLY `touch_bodies = [tool_name]`. The tool engaging the joint is an **intentional kinematic switch / mode-change moment** — by-design contact, hence whitelisted. The wrist is NOT meant to touch the joint, so wrist-vs-joint stays a real collision check. See `tasks/cc_lessons.md` → "ACM whitelisting: only allow the contact that the design INTENDS".
2. Coloring uses a focused new `highlight_env_for_ik(active_bar_id)` rather than reusing `show_sequence_colors`. See `tasks/cc_lessons.md` → "Coloring helpers: don't repurpose holistic display modes".
3. No v1 capture fallback. Existing `tests/captures/*.json` are deleted at the same commit; debug scripts hard-fail on non-v2 schema.

## Compas_fab ACM cheatsheet (for future joint wiring)

| Bucket | Field | Skips | Populate from |
|---|---|---|---|
| link↔link | `client.unordered_disabled_collisions` | CC.1 | SRDF only (read-only at runtime) |
| tool→link | `tool_state.touch_links` | CC.2 | per-state, runtime |
| rb→link | `rigid_body_state.touch_links` | CC.3 | per-state, runtime |
| rb↔body | `rigid_body_state.touch_bodies` | CC.4, CC.5 | per-state; entries can be RB names OR tool names |
| RB-as-attached | `rb.attached_to_tool` / `attached_to_link` | CC.5 (vs that tool) + makes RB participate in CC.4 | per-state |

---

## File-by-file changes

### NEW `scripts/core/env_collision.py`

```py
# -- env collision wiring for assembly IK --

from __future__ import annotations
import numpy as np
from core import config
from core.rhino_bar_registry import get_bar_seq_map  # only when called from Rhino

ENV_RB_BAR_PREFIX   = "env_bar_"     # e.g. env_bar_L7
ENV_RB_JOINT_PREFIX = "env_joint_"   # e.g. env_joint_J42_male

def collect_built_geometry(active_bar_id, bar_seq_map):
    """
    Walk bar_seq_map for seq < active_seq.
    Returns: {rb_name: dict(mesh, frame_world_mm, kind, source_oid)}
      - bar  : tube cylinder mesh (compas Mesh) along bar curve, radius=BAR_RADIUS
      - joint: load mesh from joint block def (Rhino) → compas Mesh, frame=ocf_world_mm
    Skips active bar AND its joints.
    """
    # Implementation reads Rhino docs (rs_doc) only when run inside Rhino.
    # Headless replay does NOT call this — it loads from capture instead.

def register_env_in_robot_cell(robot_cell, env_geom, *, deps):
    """Mutates robot_cell.rigid_body_models: adds RigidBody(name=name, mesh=mesh).
       Idempotent by name (skip if already present with same mesh hash)."""

def build_env_state(template_state, env_geom):
    """Returns template_state.copy() with rigid_body_states[name].frame populated.
       Sets is_hidden=False, attached_to_tool=None, attached_to_link=None.
       Static obstacles."""

def list_env_summary(env_geom) -> str:
    """'5 built bars, 8 joints (T20_Female: 4, T20_Male: 4)' — for prompt header."""

# --- ACM seam (stub for future active-joint wiring) ---

def allow_active_joint_touch(state, joint_rb_name: str, arm_side: str):
    """Whitelist active male joint ↔ matched arm tool ONLY.

    Wrist-vs-joint is intentionally NOT whitelisted: that's a real
    collision check (a property of the kinematics, not of the proxy mesh).
    """
    rb = state.rigid_body_states[joint_rb_name]
    tool = config.LEFT_TOOL_NAME if arm_side == "left" else config.RIGHT_TOOL_NAME
    rb.touch_bodies = sorted(set(rb.touch_bodies) | {tool})
```

### NEW `scripts/core/highlight_env.py`

```py
# Focused env-collision overlay. NOT a replacement for show_sequence_colors.
# Mutates only built bars + their joints; leaves active/unbuilt untouched.

import rhinoscriptsyntax as rs
from core.rhino_bar_registry import get_bar_seq_map, _bar_curve_and_tube, _joint_layer_objects

ENV_GREEN = (60, 179, 60)  # match SEQ_COLOR_BUILT for visual consistency

def highlight_env_for_ik(active_bar_id):
    """Paint built bars + joints green; record originals for revert.
       Returns a token (dict) to pass back to revert_env_highlight.
       Does NOT touch active bar, unbuilt bars, tools, or visibility."""

def revert_env_highlight(token):
    """Restore by-layer color on every oid touched by the matching highlight call."""
```

### EDIT `scripts/core/robot_cell.py`

- New top-level: `_ensure_env_registered(robot_cell, env_geom, planner)` — calls `register_env_in_robot_cell`, then if any RB names changed since last call, `planner.set_robot_cell(robot_cell)`.
- `solve_dual_arm_ik(...)` signature gain: `verbose_pairs: bool = False`. When True, after a successful inner solve, call `planner.check_collision(state, options={"verbose": True, "full_report": True})`, capture stdout via `contextlib.redirect_stdout(io.StringIO())`, parse CC.x lines into a counter dict, print the summary table (see §"Pair-count summary" below).

### EDIT `scripts/rs_ik_keyframe.py`

Insertion points (current line numbers as of session start):

1. **L645–L652** (after `_resolve_target_bar`, before pineapple insert):
   ```py
   env_geom = env_collision.collect_built_geometry(target_bar_id, get_bar_seq_map())
   _ensure_env_registered(rcell, env_geom, planner)
   template_state = env_collision.build_env_state(template_state, env_geom)
   print(f"RSIKKeyframe: env collision — {env_collision.list_env_summary(env_geom)}")
   ```

2. **L685** (right BEFORE `_ask_collision_options`):
   ```py
   _env_token = highlight_env.highlight_env_for_ik(target_bar_id)
   ```

3. **L688** (collision options prompt) — append env count to prompt:
   ```py
   # in _ask_collision_options: prepend "(env=N bodies) " to "Collision options"
   ```

4. **L691, L735** (after each IK solve attempt) — pass `verbose_pairs=True` so the summary prints. Or call a new helper `_print_collision_summary(planner, state)` directly.

5. **L767–L786** (accept/reject branch):
   - On accept: leave the highlight in place (user wanted "color stays after accept"). Do not call `revert_env_highlight`.
   - On reject / cancel / failure: `highlight_env.revert_env_highlight(_env_token)` in `finally`.
   - Subtle: the existing `finally` deletes pineapples + clears robot meshes. Add highlight-revert there gated on a flag set true at accept.

### EDIT `scripts/rs_ik_keyframe.py` capture (`_save_capture`)

Replace the current v1 payload with v2:

```jsonc
{
  "schema_version": 2,
  "captured_at": "...",
  "robot_cell_ref": "robot_cells/<sha8>.json",   // sibling file
  "initial_state": <RobotCellState __data__>,    // base + rb poses + tool attach + ACM
  "ik_targets": {
    "final":    {"left_tool0_world_mm": [[..]], "right_tool0_world_mm": [[..]]},
    "approach": {"left_tool0_world_mm": [[..]], "right_tool0_world_mm": [[..]]}
  },
  "ik_options": {"check_collision": true, "include_self": true, "include_env": true},
  "expected": {
    "final":    <RobotCellState __data__> | null,
    "approach": <RobotCellState __data__> | null
  },
  "source": {
    "target_bar_id": "L7",
    "left_block_name":  "T20_Male",
    "right_block_name": "T20_Male",
    "left_arm_side":    "left",
    "right_arm_side":   "right",
    "lm_distance_mm":   200.0
  }
}
```

`robot_cells/<sha>.json` is written via `core/capture_io.save_robot_cell_if_changed(rcell)`, which content-hashes `RobotCell.__data__` (after env registration) and only writes when the hash is new. Captures across the session de-dupe to one big file.

### NEW `scripts/core/capture_io.py`

```py
def save_robot_cell_if_changed(robot_cell, captures_dir) -> str:
    """Hashes RobotCell.__data__ → returns 'robot_cells/<sha8>.json'.
       Writes only if new. Returns the relative ref."""

def load_robot_cell_ref(capture_dir, ref) -> RobotCell:
    """Inverse — RobotCell.__from_data__(json.load(...))."""

def save_capture_v2(path, *, robot_cell_ref, initial_state, ik_targets,
                    ik_options, expected, source): ...

def load_capture(path) -> dict:
    """Loads a v2 capture. Hard-fails on schema_version != 2."""
```

**Migration**: at the same commit, delete every existing `tests/captures/*.json` (they're v1, regeneratable). Don't carry a fallback branch.

### EDIT `tests/debug_ik_collisions.py`

1. Top of `main()`: `payload = load_capture(args.capture)` (hard-fails on non-v2).
2. Load `RobotCell` from `payload["robot_cell_ref"]`, push via `planner.set_robot_cell(rcell)`. Use `payload["initial_state"]` directly (already includes env rigid_body_states). Solve from `payload["ik_targets"][phase]`.
3. After solve, before contact dump: tint env rigid bodies green (`_ENV_GREEN = [0.235, 0.7, 0.235, 0.85]`) — call `_safe_change_color` on each PB body whose label starts with `env_`.
4. Print env summary upfront ("env: N built bars + M joints").

### EDIT `tests/debug_ik_support_collisions.py` (parity)

Same shape as `debug_ik_collisions.py`. Support cell already loads the dual-arm robot as a static `ToolModel` obstacle; now the env rigid bodies (built bars + joints) are present too.

The captured `RobotCell` is whichever cell was active when the capture was written:
- assembly capture → `rcell` (dual-arm) with env RBs registered.
- support capture → `rcell_support` (single-arm) with env RBs registered AND DualArm tool.

Replay just deserializes via `RobotCell.__from_data__(...)` from `robot_cell_ref` — same helper for both scripts. The two debug scripts diverge only in:
- which IK solver they call (`rc.solve_dual_arm_ik` vs `rcs.solve_support_ik`),
- which IK targets they read from `payload["ik_targets"]` (left+right tool0 vs single tool0).

Tint env bodies green identically.

NOTE: `env_collision.collect_built_geometry` runs in BOTH `rs_ik_keyframe.py` AND `rs_ik_support_keyframe.py` — same helper, called with different cells (`rcell` vs `rcell_support`). The support arm runs AFTER assembly, so its `built_bars` cutoff at the same `active_bar_id` includes more bars; the helper doesn't care which cell receives the RBs.

---

## Pair-count summary format

After each successful IK, print:

```
RSIKKeyframe: collision report (final)
  env: 5 built bars + 8 built joints registered as rigid bodies
  CC.1 link↔link    pairs=210  passed=193  skipped=17(srdf)            flagged=0
  CC.2 link↔tool    pairs=84   passed=78   skipped=6(touch_links)      flagged=0
  CC.3 link↔env     pairs=336  passed=336  skipped=0                   flagged=0
  CC.4 rb↔rb        pairs=0    skipped=12(static)
  CC.5 tool↔env     pairs=16   passed=16   skipped=0                   flagged=0
```

Implementation: redirect compas_fab's verbose stdout into a `StringIO`, parse with a regex against the line format `"CC.{n} between ... - {PASS|SKIPPED (REASON)|COLLISION}"`. Aggregate, print, swallow raw lines (don't surface them to the Rhino console — too noisy).

On IK FAILURE: the `CollisionCheckError` already carries `collision_pairs`. Print those structured pairs as the "flagged" rows; pair counts come from a final dry `check_collision(verbose=True, full_report=True)` against the no-check seed for the curious user.

---

## Test plan

1. **Live (Rhino)**: open a scene with ≥3 bars at varying sequence numbers. Click IK button on a mid-sequence bar; verify:
   - Built bars+joints turn green; active stays default; unbuilt unchanged.
   - Console prints "env collision — N built bars, M joints registered".
   - Collision-options prompt mentions env count.
   - Per-IK summary prints CC.3 / CC.5 line counts > 0.
2. **Replay (headless)**: `python tests/debug_ik_collisions.py tests/captures/<v2>.json --headless` — exit code 0, env summary printed.
3. **Replay GUI**: same with `--phase final` + GUI; visually confirm green env bars next to robot.
4. **Backwards compat**: replay an old v1 capture (`tests/captures/20260429_230013_support_B3_ik_fail.json` etc.) — must still work via fallback branch.
5. **Empty env**: pick the seq=1 (first) bar — `built_count=0`. IK runs, prompt says "(env=0 bodies)", CC.3/CC.5 pair counts=0 with no error.
6. **ACM stub callable**: `from core.env_collision import allow_active_joint_touch; allow_active_joint_touch(state, "env_joint_J5_male", "left")` and verify `state.rigid_body_states["env_joint_J5_male"].touch_bodies == ["AssemblyLeftTool"]` (or similar). No `touch_links` mutation.

---

## Out-of-scope reminders for next session

- Active male joint: when Victor wires per-step joint metadata, register the active-bar male joint as `env_joint_<id>_male` too, and call `allow_active_joint_touch(state, "env_joint_<id>_male", arm_side)` for each picked arm. Wrist remains in the collision check.
- Bar-being-placed: model as `attached_to_tool="AssemblyLeftTool"` (or right) once we want CC.4 to fire on it during approach motion. Different feature.
- The capture's `robot_cells/<sha>.json` files are NOT auto-cleaned. Add a `tests/captures/cleanup_orphan_cells.py` later if disk usage becomes a concern.
