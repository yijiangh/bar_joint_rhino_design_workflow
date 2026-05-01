# Dual PyBullet Client Isolation Probe

A note about `tests/debug_dual_client_isolation.py` — what it tests, what we learned, how to run it, how to read its output.

## Rationale

Today the IK workflows share **one** `compas_fab.PyBulletClient` and **one** `PyBulletPlanner`:

- `RSIKKeyframe` uses the dual-arm cell (Cindy + assembly pineapple tools).
- `RSIKSupportKeyframe` uses the support cell (Alice + Robotiq gripper + dual-arm wrapped as a `DualArm` tool obstacle).

When the user ping-pongs between the two scripts in a Rhino session, our `set_cell_state` calls `planner.set_robot_cell(other_cell)`, which **re-uploads URDFs and rebuilds collision geometry** in PyBullet — slow.

The proposed refactor: keep **two** persistent `PyBulletClient` + `PyBulletPlanner` pairs alive (one per cell) for the whole session, so cell switching is just "use the other planner". No URDF re-upload.

Two prerequisites had to be verified before committing to that refactor:

1. **Isolation** — do two `PyBulletClient` instances in one Python process actually keep their state separate, or does mutating one leak into the other?
2. **Stability** — do they stay alive cleanly under long-running headless use (the realistic IK workload), or do they degrade / crash?

This script answers both with the existing cells, no production code changes.

## Conclusion

**Both prerequisites pass.** Verified 2026-04-30 on Rhino's bundled Python 3.9:

- Isolation: 8/8 `[OK]` checks across four mutation phases. Mutating Client A's actuated robot never bleeds into Client B's state, and vice versa.
- Stability: 5000 alternating mutate-+-read iterations across both clients, **0 failures**, mean per-iteration **~0.05 ms**, p95 **0.06 ms**. Both clients remained `isConnected == 1` throughout.

**What this depends on**

- `compas_fab.PyBulletClient` is per-client clean: every `pybullet.*` call passes `physicsClientId=self.client_id`. Per-client state lives on the instance (`robot_puid`, `tools_puids`, `disabled_collisions`).
- The script avoids `pybullet_planning` helpers like `pp.set_pose` / `pp.disconnect` / `pp.is_connected` / `pp.link_from_name` / `pp.get_link_pose` — those bind to a module-global `CLIENT` and would break two-client setups. The actual production refactor will need to replace those callsites; the script's `# === REFACTOR NOTES ===` footer enumerates them.
- PyBullet limit: only **one GUI per process**. Two DIRECT works; one GUI + one DIRECT works; two GUIs is rejected by the script's argparse (`--gui both` errors).

**Bonus discovery**: compas_fab's `set_robot_cell_state` already pushes the base via `client._set_base_frame` (per-client-scoped). The `pp.set_pose(...)` calls at `core/robot_cell.py:469` and `core/robot_cell_support.py:247` are **redundant** and can be deleted outright when the refactor lands — not just rescoped.

**Green light:** the multi-persistent-planner refactor is safe to start. (3-client mode verified separately — see `--three-way` below.)

### Important caveat: `pp.set_client` is broken for most pp helpers

When investigating whether the `pp.*` ergonomic helpers (`pp.set_pose`, `pp.set_joint_positions`, `pp.get_link_pose`, …) could be kept in multi-client code, the test surfaced a real footgun:

- `pp.set_client(client_id)` only mutates `pybullet_planning.utils.shared_const.CLIENT`.
- About 28 pp submodules do `from pybullet_planning.utils import CLIENT, ...` at import time, creating their OWN module-local `CLIENT` bindings.
- `pp.set_client` does NOT update those rebound copies. So `pp.set_pose` etc. silently keep reading their stale local 0 — your call goes to the wrong client.

The test's `pp Phase 0` reproduces the brokenness; phases 1–5 use a corrected `pp_active_client` context manager (in the same file) that walks pybullet_planning once and patches every submodule's `CLIENT` on each switch. With that helper, all of `pp.set_pose`, `pp.set_joint_positions`, `pp.get_link_pose` route correctly per client.

**Recommendation for the production refactor:** prefer raw `pybullet.X(..., physicsClientId=client.client_id)` calls. The patched `pp_active_client` works but is brittle — any pp submodule lazy-loaded after the walker runs will not be patched. Per-call `physicsClientId=` is enforced at the call site and has no global state to corrupt.

## How to run

The script must run under Rhino's bundled Python 3.9. The native modules in the Rhino site-envs (`cryptography` in particular) are 3.9-only — loading them under py3.10+ raises a PyO3 once-per-process init error.

```
~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py [args]
```

CLI flags:

| Flag | Default | Meaning |
|---|---|---|
| `--gui {a,b,none}` | `a` | Which client gets the GUI window. `both` rejected (one GUI per process). |
| `--capture <path>` | none | Optional support-IK capture JSON to seed both initial states (otherwise zero-config + identity-base). |
| `--stability-iterations N` | `0` | If `>0`, run a soak loop. Forces all clients to DIRECT regardless of `--gui`. |
| `--stop-on-error` | off | Bail the soak loop on first failure instead of accumulating. |
| `--seed S` | `42` | Deterministic per-iteration mutations. |
| `--verbose` | off | Print `physicsClientId` per client at connect time. |
| `--three-way` | off | Stress mode: load THREE clients (Cindy + Alice + Belle) and verify isolation across all of them. Requires the renamed Belle URDF + dedicated `belle.srdf` (already in tree). Belle is always DIRECT. |

Recipes:

```
# 1) Headless smoke (programmatic isolation only)
~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --gui none

# 2) GUI demo (one window — see below for what it shows)
~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --gui a

# 3) Long soak (DIRECT, both clients)
~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --stability-iterations 5000 --stop-on-error

# 4) Three-way stress: Cindy + Alice + Belle, all DIRECT
~/.rhinocode/py39-rh8/python.exe tests/debug_dual_client_isolation.py --three-way --stability-iterations 2000 --stop-on-error
```

Recipes 1 + 3 are the 2-client green-light bar. Recipe 4 verifies the multi-client model scales to the realistic deployment shape (one assembly + N supports). Both 2-client and 3-client soaks have completed cleanly with zero failures (mean ~0.05–0.06 ms per iteration).

## How to read the output

### Headless (`--gui none`)

Every mutation phase prints two assertion lines:

```
=== Isolation phase 1: mutate A; expect B unchanged ===
  [OK] A vs A0: changed=True
  [OK] B vs B0: base=True quat=True joints=True
```

- `[OK] A vs A0: changed=True` — the mutated client moved.
- `[OK] B vs B0: base=True quat=True joints=True` — the OTHER client's base position, base quaternion, and joint values are all bit-identical to before. **This is the isolation proof.**
- `[FAIL]` on either line means the two clients are bleeding into each other (would block the refactor).

End: a single verdict line `=== Verdict: ISOLATION OK ===` (or `FAILED`) plus per-client disconnect lines.

After the four base-isolation phases, the test runs a `pp.*` scoping investigation:

- **pp Phase 0 (negative)** uses a deliberately-broken `pp_active_client_BROKEN` (only `pp.set_client`) to PROVE `pp.set_client` does not route `pp.set_pose` correctly. A passing line here reads `[OK] (proof) pp.set_client + pp.set_pose did NOT route to client B (brokenness reproduced…)`.
- **pp Phases 1–5** use the corrected `pp_active_client` (patches every pp submodule's `CLIENT`). They exercise `pp.set_pose`, `pp.set_joint_positions`, and `pp.get_link_pose` on each client and assert isolation.

### GUI demo (`--gui a` or `--gui b`)

The single GUI window shows the chosen client's actuated robot **plus** that client's "obstacle tool" body. After every mutation, the script reads the current pose + joint values of the OTHER (DIRECT) client's actuated robot and writes them onto the GUI client's obstacle tool body via raw `pybullet.resetBasePositionAndOrientation` and `pybullet.resetJointState` (both scoped with `physicsClientId=gui_client.client_id`). The obstacle tool body is therefore a passive **mirror** — it follows whatever the other client's robot is doing, whenever the script asks it to. You see two robots in one window:

- **The fully-actuated robot** — Cindy if `--gui a`, Alice if `--gui b`. Movements are driven by the script's `_mutate_base_and_joint` calls on this client.
- **A mirror of the other client** — drawn as the `SupportRobotObstacle` tool body (Alice mesh) with `--gui a`, or the `DualArm` tool body (Cindy mesh) with `--gui b`. Movements are driven by the script's `_sync_mirror_raw` calls (it reads the other client's robot, then writes those values onto this body).

The mirror has no automatic link to the other client — every update is an explicit raw-pybullet copy in `_sync_mirror_raw`. That makes the test honest: if the script forgets to call sync, the mirror stays frozen even when the other client changes; if isolation were broken, the mirror would move WITHOUT a sync (because the body itself would somehow share state across clients, which it must not).

Between phases the script prints either `mirror should STAY put` or `mirror should MOVE` and pauses on `press Enter for next mutation...`. Confirm visually:

| Phase | Mutated | Expected GUI |
|---|---|---|
| 1 | A | Actuated A moves; mirror **stays** ← isolation proof |
| 2 | B | Actuated A stays; mirror **moves** to wherever B now is |
| 3a | A | Actuated A moves; mirror **stays** |
| 3b | B | Actuated A stays; mirror **moves** |

If at any point the mirror moves on a same-client mutation (the "stays" rows), isolation has failed.

### Stability soak (`--stability-iterations N`)

Progress lines every 10%:

```
  iter 500/5000 (last=0.04ms, mean so far=0.05ms, failures=0)
```

Final summary:

```
Stability summary: iterations=5000 completed=5000 failures=0 mean=0.05ms p95=0.06ms
```

Read this as:

- `completed == iterations` and `failures == 0` → both clients survived the soak. Pass.
- `failures > 0` with `--stop-on-error` → first 5 failures printed; investigate the iteration index + exception type. Most likely cause if it ever fails: a `pp.*` helper got reintroduced somewhere and bound to the wrong global `CLIENT`.
- `mean` and `p95` are sanity checks on per-iteration overhead (mutate + read both clients + connection check). Numbers in the 0.05–0.1 ms range are healthy; > 1 ms means something started thrashing.

## See also

- `tasks/cc_lessons.md` — the lesson "Two `compas_fab.PyBulletClient` instances coexist cleanly" is the short-form summary.
- `tests/debug_dual_client_isolation.py` — the script itself; the trailing `# === REFACTOR NOTES ===` block lists the exact call sites that need fixing for the production refactor (preferred fix per site).
- `external/compas_fab/src/compas_fab/backends/pybullet/client.py:702-704` — `_set_base_frame`, the per-client-scoped base push that makes `pp.set_pose` redundant in our wrappers.
