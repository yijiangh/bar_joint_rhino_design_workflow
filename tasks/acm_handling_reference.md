# ACM (Allowed Collision Matrix) handling in `rs_ik_keyframe.py`

Reference for which collisions are intentionally whitelisted during dual-arm IK, and where in the code each rule lives.

The ACM is built in two layers inside `_prepare_collision_template_state` (`scripts/rs_ik_keyframe.py:962-996`). Both layers mutate `template_state.rigid_body_states[*].touch_links` / `touch_bodies`.

---

## Layer 1 — Tool-attached-to-wrist (link-level)

`robot_cell.configure_arm_tool_rigid_body_states`
- Called at `scripts/rs_ik_keyframe.py:981` and again at `:991` (because `build_env_state` returns a fresh copy of the state, which drops the previous attachment wiring).
- Implementation: `scripts/core/robot_cell.py:756-793`.

Sets on each arm-tool RB:
- `attached_to_link = left_ur_arm_tool0` / `right_ur_arm_tool0`
- `touch_links = [wrist_1_link, wrist_2_link, wrist_3_link]` of that arm (`scripts/core/robot_cell.py:661-672`)

→ Whitelists **arm tool mesh ↔ its own wrist 1/2/3 links** (the tool is bolted there; otherwise self-collision would always fire).

---

## Layer 2 — Design-intent body-vs-body ACM

`env_collision.configure_active_assembly_acm`
- Called at `scripts/rs_ik_keyframe.py:994`.
- Implementation: `scripts/core/env_collision.py:619-688`.

Four whitelisted categories, all mutually symmetric (`touch_bodies` set on both sides via `_add_touch_bodies`):

### 1. Active joint half ↔ its env mating half
`scripts/core/env_collision.py:651-665`

`active_joint_<jid>_male` ↔ `env_joint_<jid>_female` (and the mirror female-active / male-env). The two halves snap together at the end of the assembly motion; flagging this contact would make assembly impossible by definition.

### 2. Active joints ↔ both arm tool RBs
`scripts/core/env_collision.py:666-671`

Gripper engages the joint head. Both arms whitelisted to every active joint — over-permissive, but matches design intent.

### 3. Active bar ↔ all active joints on that bar
`scripts/core/env_collision.py:673-676`

Rigid structural bond (joints are welded/clamped to the bar).

### 4. Active bar ↔ both arm tool RBs
`scripts/core/env_collision.py:677-681`

Bar held via gripped joints; tool envelope can graze the bar body. Comment in code calls this "over-permissive but bar body rarely touches tool envelope."

---

## Notably NOT whitelisted (kept as real checks)

Per `allow_active_joint_touch` docstring (`scripts/core/env_collision.py:600-609`) and the absence of any wrist-vs-joint entry in `configure_active_assembly_acm`:

- **Wrist links ↔ joint heads** — real collision, treated as a kinematic property of the IK solution, not a proxy-mesh artefact.
- **Everything else env ↔ robot links** — built bars, env joints, active bar/joints all collide normally against arm links other than the whitelisted wrists.

---

## Runtime verification

Summary line printed by `configure_active_assembly_acm` (`scripts/core/env_collision.py:683-687`):

```
core.env_collision.configure_active_assembly_acm: active_joints=<N> active_bars=<N> mating_pairs_whitelisted=<N> tools=[<tool_rb_names>]
```

Use this at runtime to confirm which categories actually populated.

---

## Quick map

| Allowed collision | Defined in | Lines |
|---|---|---|
| arm tool ↔ own wrist 1/2/3 | `scripts/core/robot_cell.py` | 661-672, 756-793 |
| active joint ↔ mating env joint | `scripts/core/env_collision.py` | 651-665 |
| active joint ↔ arm tools | `scripts/core/env_collision.py` | 666-671 |
| active bar ↔ active joints (same bar) | `scripts/core/env_collision.py` | 673-676 |
| active bar ↔ arm tools | `scripts/core/env_collision.py` | 677-681 |
