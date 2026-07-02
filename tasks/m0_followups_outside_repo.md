# M0 change — follow-ups outside the parent repo

These items are part of the "Add M0 + rename movement classes" change but live
**outside** the `bar_joint_rhino_design_workflow` tracked tree, so they could not
be edited from the isolated worktree (`husky_assembly_tamp` is an untracked local
dependency; the stale `.tmp` files are untracked in the main working tree). Apply
them in the main checkout.

## 1. `rs_data_structure` submodule (separate git repo)

The schema rename lives in the submodule at `external/rs_data_structure`
(github.com/yijiangh/rs_data_structure). The worktree edited its checkout at
commit `ba2940b`. To land this you must, in that submodule:

1. Commit the edited `rs_data_structure/bar_action.py`, `rs_data_structure/__init__.py`,
   and `README.md` (the 4 new movement classes; no legacy alias).
2. Push, then bump the submodule pointer recorded by the parent repo to the new
   submodule commit.

## 2. TAMP `run.py` — already M0-safe; optional clarity edit

`external/husky_assembly_tamp/.../dual_arm_task_space_rrt/run.py` needs **no
functional change**: `build_gdrive_bar_action_scene_spec` chains via
`action.movements[movement_index + 1]`, where `movement_index` comes from
`find_movement(action, movement)` (resolved by role/id). That offset is relative
to the found index, so M0 at index 0 does not shift M1's chaining.

Optional clarity edit (comments + clearer error) in
`build_gdrive_bar_action_scene_spec`, just after `movement_index, mv = find_movement(...)`:

```python
    # `find_movement` resolves the movement by role/id, returning its actual index
    # in `action.movements`. The forward-chaining below uses `movement_index + 1`
    # RELATIVE to that index, so it stays correct after M0 was inserted at index 0
    # (M1 is now index 1, its "next" is M2 at index 2, etc.).
    movement_index, mv = find_movement(action, movement)
    ...
    if not mv.target_ee_frames or "left" not in mv.target_ee_frames or "right" not in mv.target_ee_frames:
        # M0 (the live-deployment lead-in) carries no target_ee_frames on purpose;
        # it is planned live by the monitor, not through this EE-frame scene spec.
        raise ValueError(
            f"Movement {mv.movement_id!r} has no left/right target_ee_frames "
            "(M0 is unplanned offline -- plan it live, not via this path)."
        )
```

**Coordination note:** `find_movement` / `parse_bar_action` live in the separate
`husky-assembly-teleop` repo (`husky_assembly_teleop.bar_action_io`). Confirm
`find_movement` selects by role/id (not by a hardcoded index 0 == M1). That repo's
live monitor also already has a "synthetic-M0 backfill"; align it with the real M0
now emitted in the export (M0 = `IndependentDualArmFreeMovement`, id `<bar>_M0_free_to_M1_start`,
`start_state` = bar-released context with `robot_configuration = None`,
`target_ee_frames = None`, `target_configuration` = None until M1 is planned).

## 3b. Diagram script (untracked WIP in main, not in the worktree)

`docs/robotcell_state_lifecycle.py` (and its `.png`) are untracked in the main
working tree, so they were not in the isolated worktree. Update the movement
boxes/labels there to the new class names and add an **M0** box (released,
live-planned) before M1. Current stale label example: `"M1  DualArmConstrained ·
home->approach"` → use the new class names; the gripped/released colouring is
M1/M2 gripped, M0/M3/M4 released.

## 3. Stale `.tmp` files (untracked, in the main working tree)

Delete these leftovers flagged in git status / notes:
- `tests/headless_bar_action_planner.py.tmp.25816.bc595502b557`
- `external/husky_assembly_tamp/.../dual_arm_task_space_rrt/run.py.tmp.*` (if present)

## 4. Re-export existing BarAction JSONs

Old `BarActions/*.json` carry the old class dtypes and only 4 movements. Re-export
with `RSExportAllBarActions` (they will then have 5 movements, M0 first).
