# Dual-arm BarAction planner — session handoff notes

Working notes for `tests/headless_bar_action_planner.py` and the dual-arm
motion-planner API. Written so a fresh chat can pick this up. **Nothing here is
committed yet.**

## Goal / what this session did

Make `tests/headless_bar_action_planner.py` load a Rhino-exported
`BarAssemblyAction`, plan each movement (M1–M4), and chain them. Along the way
we fixed several planner/library issues and retired the legacy `core.bar_action`
compat alias.

## How to run the headless test

Plain `python.exe` does NOT have compas/pybullet on its path — those live in a
Rhino "site-env" that only the RhinoCode launcher wires up. To run from a normal
shell, bootstrap the site-env first (there's a helper for it). Wrapper used all
session:

```
C:/Users/yijiangh/.rhinocode/py39-rh8/python.exe -c "
import os, sys, runpy
REPO=os.path.abspath('.')
TESTS=os.path.join(REPO,'tests'); SCRIPTS=os.path.join(REPO,'scripts')
sys.path.insert(0, TESTS)
from _rhino_env_bootstrap import bootstrap_rhino_site_envs
bootstrap_rhino_site_envs(verbose=False)          # puts compas/pybullet on sys.path
if SCRIPTS not in sys.path: sys.path.append(SCRIPTS)
sys.argv = ['headless_bar_action_planner.py','--bar-action','B6.json','--movement','M1','--no-replay','--max-time','30']
try:
    runpy.run_path(os.path.join(TESTS,'headless_bar_action_planner.py'), run_name='__main__')
except SystemExit as e:
    print('EXIT CODE =', e.code)
"
```

- `--movement` accepts `M1`/`M2`/`M3`/`M4`/`all`. `all` chains M1→M2→M3→M4.
- `--no-derive-start` opts out of M1 start derivation (default = derive).
- `--gui` (without `--no-replay`) opens a slider replay.
- Active dataset: `2026-05-16_double_kissing_jig_demo`, `B6.json` (on the
  Google-Drive `data_design_study` dir; path is the default in the script).
- Inside Rhino / RhinoCode the bootstrap is unnecessary — the launcher wires the
  env and `scripts/` (via `~/.rhinocode/python-3.pth`).

## Changes made this session (by file)

**`tests/headless_bar_action_planner.py`** (main test)
- Moved `from core import config` to module top (import-order fix).
- Added batch mode `--movement all` + helpers: `select_movement`, `plan_movement`,
  `apply_conf12`, `_path_from_jt`, `replay_segments` (combined slider), summary.
- Added chaining adapted from teleop `husky_monitor.py`: `accept_trajectory`
  (role-based: M1/M4 own their start from traj[0]; M2/M3 validate propagated
  start; M1/M2/M3 propagate traj[-1] into the next movement's start_state;
  backward continuity check) + `print_movement_roster` + `vec12_from_conf`.
  Added `import numpy as np`.
- M1 uses `derive_start=True`; M4 goal overridden with
  `config.HUSKY_DUAL_ARM_HOME_CONF_12`.
- Simplified the now-stale `from core import config` comment after the alias
  removal.

**`external/husky_assembly_tamp/.../motion_planner/api.py`**
- `plan_constrained_dual_arm`: added opt-in `derive_start` (+ `start_random_seed`,
  `start_max_ik_attempts`, `start_bar_sweep_box`). New helper
  `_derive_constrained_start_for_plan`: goal-first grasps (from the authored bar
  attachment + FK at goal) → `derive_constrained_start` → feasible, collision-free
  start; also re-solves the goal collision-free (`solve_endpoint_dual_arm_ik`).
  Returns `info["derived_start_conf"]`. Default path (`derive_start=False`) is
  unchanged, so Rhino/teleop callers are unaffected.
- `plan_constrained_dual_arm_linear` (M2): reads the live pybullet bar pose
  instead of the attached bar's `None` `.frame`.

**`external/husky_assembly_tamp/.../dual_arm_task_space_rrt/core.py`**
- Defined `HUSKY_DUAL_URDF_PATH`/`HUSKY_DUAL_SRDF_PATH` locally (from `DATA_DIR`)
  so `get_joint_collision_fn` no longer does `from .run import ...` (that import
  was dragging in the broken `run.py`; also removes a core↔run cycle).
- `derive_constrained_start`: added optional `joint_collision_fn` param — when
  provided it's used as-is (the api passes a cfab-backed predicate, so no husky
  URDF/SRDF file on disk is needed); otherwise it builds one from URDF as before.

**`scripts/core/config.py`**
- Added `HUSKY_DUAL_ARM_HOME_CONF_12` (fixed 12-vec home for M4's goal).

**`external/rs_data_structure/rs_data_structure/__init__.py`**
- REMOVED the legacy `core.bar_action` dtype alias block entirely + trimmed the
  docstring. `import rs_data_structure` now has no `core`/`sys.modules` side
  effects. (`core.bar_action` = the builder module only; the data classes are
  `rs_data_structure.bar_action`.)

**Data migration (Google-Drive `data_design_study`, not in repo)**
- 8 old BarAction JSONs still had `dtype: "core.bar_action/..."`. Migrated in
  place with a prefix swap `core.bar_action/` → `rs_data_structure.bar_action/`
  (5 per file), each backed up as `<name>.json.bak`. All 8 re-verified to load.
  Files: `2026-05-08_dual-arm_transfer_test/{B1,B7,B8,B11}`,
  `2026-05-11_mocap_bar_reach_test/{B4,B5,B6}`, `2026-05-14_foc_demo_reduced/B226`.
  (The active `2026-05-16` demo was already on the new dtype.)

**Reverted (net-zero):** a "closest-branch" IK selection in
`_run_dual_arm_cartesian_ik_loop` was added then reverted at the user's request.
`api.py` is back to the original first-result IK there.

## Current status

- M1 plans end-to-end (derives a feasible, collision-free start; goal re-solved
  collision-free). One green roll showed **M1 ok, M2 ok, M3 ok**; M4 then failed
  on the *old* placeholder goal (since replaced with `HUSKY_DUAL_ARM_HOME_CONF_12`).
- Alias removal verified; all 8 legacy files migrated and load.

## Open items (not done — deliberate)

1. **M1 RRT is stochastic** — solves maybe ~1 in 4 rolls at 40–90 s, failing as
   `rrt_failed` or `goal_in_collision`. The proper fix is `run.py`'s
   `start_retries` loop (re-derive a different start on RRT failure). User chose
   to leave as-is for now; use a larger `--max-time` / re-run to get a green chain.

2. **M2/M3 linear-IK "branch flip"** — the real blocker for the chain. In
   `api._run_dual_arm_cartesian_ik_loop`, the per-waypoint IK takes compas_fab's
   *first* `inverse_kinematics` result. Per the compas_fab pybullet IK docs, the
   first attempt is the seed descent ONLY if it converges/passes collision;
   otherwise it returns a *random* restart branch. So a tiny Cartesian step (≤5 mm
   / 0.05 rad) can come back as a 3–4 rad wrist/elbow flip and get rejected by the
   continuity check (threshold ~0.1745 rad) → `linear-ik failed`. Confirmed
   seed-independent (M2 fails standalone too; the authored M2 start is also
   self-colliding, a separate data issue). Proposed fix (was implemented then
   reverted): iterate `planner.iter_inverse_kinematics(...)` per arm/waypoint and
   pick the collision-valid solution nearest the previous waypoint. User wanted to
   read into it before applying.

3. **`run.py` is broken at import** —
   `external/husky_assembly_tamp/.../dual_arm_task_space_rrt/run.py` imports
   `get_pose_collision_fn` from `core` (line ~49, used ~line 1165), but that
   symbol no longer exists anywhere in the package (removed in a refactor). So
   `run.py` (the standalone Stage-1/2/3 runner) does not import/run. This predates
   this session and is untouched. Our cfab path (`api.py`) is independent of it
   after the core.py change above. To unbreak: either restore `get_pose_collision_fn`
   in `core.py`, or drop the dead import + guard its single (stage-1-smoothing)
   use. Also a stray `run.py.tmp.25516.*` file is sitting next to it (deletable).

## Key relationships (mental model)

- `run.py` (standalone, builds its own pybullet world from URDF) and `api.py`
  (cfab-integrated, uses the loaded RobotCell) are **parallel front-ends** over the
  same algorithms in `dual_arm_task_space_rrt/core.py` (`plan_pose_rrt`,
  `derive_constrained_start`, `solve_endpoint_dual_arm_ik`,
  `smooth_dual_arm_pose_path`). They don't import each other. The `derive_start`
  path in `api.py` is a reimplementation of `run.py`'s Stage-3 procedure against cfab.
- `scripts/core/bar_action.py` = the Rhino/planner-side **builder** (turns a picked
  bar + placed tools + RobotCell into Movement/BarAssemblyAction objects). It
  imports the data classes from `rs_data_structure.bar_action`. Not to be confused
  with the (now-deleted) `core.bar_action` dtype alias.
