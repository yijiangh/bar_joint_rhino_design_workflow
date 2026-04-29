# Test Pipeline Plan — Reduce Rhino-in-the-Loop

## Goal

Cut Rhino click-test cycles. As much logic as possible should be exercisable from a plain Python terminal (`python -m pytest`) so Claude can self-debug without asking the user to click buttons.

## What can / cannot be tested without Rhino

| Layer | Rhino-required? | Reason |
|---|---|---|
| `core/transforms.py`, `core/kinematics.py`, `core/geometry.py`, `core/t1_s2_case_export.py` | **No** | Pure numpy/scipy. Already covered by `tests/test_*.py`. |
| `core/config.py`, `core/config_generated_ik.py` | **No** | Module-level constants + sanitizer. |
| `core/robot_cell.py` | **No** | compas_fab + PyBullet, all headless via `connection_type="direct"`. URDF/SRDF live in the asset submodule. |
| New: `core/ik_workflow_math.py` (extracted from `rs_ik_keyframe.py`) | **No** | Pure numpy frame math + payload serialization. |
| `core/rhino_helpers.py`, `core/rhino_bar_registry.py`, `core/rhino_frame_io.py` | **Yes** | Direct `rhinoscriptsyntax` / `Rhino.*` calls. Mocking is fragile, skip. |
| `core/ik_viz.py` | **Yes** | Bakes Rhino mesh objects via `compas_robots.rhino`. |
| `rs_*.py` | **Yes** | Interactive picks via `Rhino.Input.Custom.GetObject` / `GetPoint`. |

Rule of thumb: anything that imports `Rhino`, `rhinoscriptsyntax`, `scriptcontext`, or `compas_rhino` cannot be unit-tested outside Rhino. Everything else can.

## Phase A — Refactor: split logic from Rhino plumbing

Aim: shrink `rs_ik_keyframe.py` to *picks + UI prompts + side effects*, push everything pure into `core/`.

1. New module **`scripts/core/ik_workflow_math.py`**. Move from `rs_ik_keyframe.py`:
   - `_unit` → `unit`
   - `_frame_from_origin_normal_heading` → `frame_from_origin_normal_heading`
   - `_translate_frame` → `translate_frame`
   - approach-direction logic → `approach_direction(ocf_left_z, ocf_right_z)`
   - tool0-from-OCF dispatch (the nested-dict lookup that resolves `block_name + arm_side`) → `tool0_from_ocf(ocf_world_mm, block_name, arm_side, table)`
   - `_build_assembly_payload` → `build_assembly_payload(...)` (parameterized so it doesn't depend on `robot_cell` import)
   `rs_ik_keyframe.py` keeps only the Rhino-touching helpers and re-imports the moved functions.

2. New module **`scripts/core/walkable_sampling.py`** (later). Wraps `_sample_base_offsets` in a Rhino-free way; the brep snap stays in `rs_ik_keyframe.py`.

3. **`core/robot_cell.py`** stays as-is but its public API becomes the contract tested in Phase B.

## Phase B — pytest suites

Tests under `tests/`. Run via `python -m pytest tests/ -v` from the repo root.

1. **`tests/test_ik_workflow_math.py`** (Phase A required; pure numpy):
   - Frame from origin/normal/heading: orthonormality, X projects onto tangent plane, Y = Z×X.
   - Heading collinear with normal raises.
   - `translate_frame` only touches translation column.
   - `approach_direction` returns `-unit(z_L + z_R)`; raises on antiparallel z's that sum to zero.
   - `tool0_from_ocf` looks up nested dict by block_name + arm_side; raises on missing entries (current bug-prone path).
   - `build_assembly_payload` shape: `robot_id`, `base_frame_world_mm` (4×4 list), `final` and `approach` each with `left`/`right` configs of `joint_names` + `joint_values`.

2. **`tests/test_config_sanitize.py`** (no Rhino):
   - Nested-schema dict round-trips through `_sanitize_ocf_to_tool0_dict`.
   - Flat-schema input raises with "Re-export via RSExportJointTool0TF" message.
   - Unknown arm side raises.
   - Each surviving matrix is 4×4 and orthonormal in its rotation block.

3. **`tests/test_robot_cell.py`** (no Rhino, requires pybullet + the submodule):
   - Module-scoped fixture: `start_pb_client(use_gui=False)` → yield → `stop_pb_client()`.
   - `get_or_load_robot_cell` returns a RobotCell with `LEFT_GROUP` and `RIGHT_GROUP` configurable joints.
   - `solve_dual_arm_ik` against a known-reachable target (e.g. take the default cell-state's tool0 frames as targets — IK should converge in 1 attempt).
   - `extract_group_config` returns the right joint names for each group.
   - **Regression**: `start_pb_client` runs cleanly with `verbose=True` (catches the `os.dup` issue from the redirect_stdout lesson).
   - **Regression**: URDF path resolves (catches the `urdf/urdf/` doubling we hit).

4. **`tests/test_ik_viz_smoke.py`** (skipped outside Rhino via `pytest.importorskip("rhinoscriptsyntax")` — runs only when invoked from inside Rhino's ScriptEditor; serves as documentation).

5. **`tests/conftest.py`**:
   - Inject `scripts/` and `external/compas_fab/src/` onto `sys.path` once for the session.
   - Verify the submodule is checked out; otherwise `pytest.skip` collection-time.

## Phase C — End-to-end smoketest CLI

`scripts/_dev_smoketest.py` (single-file, no Rhino, runs from a normal terminal):

```text
python scripts/_dev_smoketest.py
```

Does:

1. `start_pb_client(use_gui=False)`.
2. Load robot cell.
3. Build a synthetic dual-arm scenario:
   - Pick a base frame with origin (0,0,0), Z=world_Z, X=world_X.
   - Synthesize "male joint" 4×4 OCFs (e.g. one in front of each shoulder, Z pointing toward each other).
   - Compute tool0 targets via `tool0_from_ocf` with both arms' nested-dict entries.
4. Call `solve_dual_arm_ik` for final + approach.
5. Print the same `[OK] / [x] / [X]` lines as the Rhino script. Optionally write `payload.json` matching the `ik_assembly` schema.
6. Exit code 0 on success, 1 on IK failure.

When Claude needs to debug an IK regression, it runs this script and reads stdout — no Rhino needed.

## Phase D — Quick-rerun helpers

1. **`tests/Makefile`-equivalent** (or `pyproject.toml` test alias): one-line invocation.
2. **VS Code task or `tasks/run_tests.ps1`** so the user can also trigger the same suite from the IDE with one keystroke.
3. **CI later**: same `pytest tests/` plus a pre-commit hook running `python -m pytest tests/ -m "not rhino" --tb=short`.

## What would have caught past bugs

| Past bug | Test that would have caught it |
|---|---|
| `urdf/urdf/...urdf` doubled path | `test_robot_cell::test_load_robot_cell_urdf_path_resolves` |
| `os.dup(fd)` failure with `verbose=False` | `test_robot_cell::test_start_pb_client_verbose_false_does_not_call_dup` |
| Flat→nested schema mismatch | `test_config_sanitize::test_legacy_flat_schema_raises_with_guidance` |
| `block_name`-keyed lookup vs `joint_type` lookup | `test_ik_workflow_math::test_tool0_from_ocf_uses_block_name_then_subtype_fallback` |
| Approach offset direction | `test_ik_workflow_math::test_approach_direction_is_negative_average_z` |
| `rs.coercebrep` returning None for Extrusion | (Rhino-only — not testable) |
| `Constrain(Plane, Boolean)` overload mismatch | (Rhino-only — not testable) |
| `super(Geometry/PyBulletBase, self)` class-identity drift | (Rhino-only side effect of selective module purging — covered now by lesson + no-purge policy) |

So roughly 6 of the 8 last-week bugs become catchable in CI; the other 2 are Rhino API quirks and stay manual.

## Open decisions for the user

1. Should we put the `_dev_smoketest.py` under `scripts/` (alongside the others) or under `tests/scripts/` to keep it out of the Rhino toolbar discovery? (I'd recommend the latter — `tests/smoketest.py`.)
2. Do you want a CI on push (GitHub Actions) once the suites land, or strictly local runs for now?
3. Is the existing `requirements-dev.txt` the right place to pin pybullet / compas_fab transitive deps for the test environment, or should we add `requirements-test.txt`?

## Suggested execution order

A1 (split math out) → B1 (pytest math tests) → B2 (config sanitize) → B3 (robot_cell IK) → C (smoketest CLI) → A2 + B4 + D as they become useful.

Each step is small and merges cleanly without touching Rhino. Recommend starting with **A1 + B1** because the IK math has been the densest source of bugs and is fully Rhino-free.

---

## Phase E — Capture-and-Replay (highest leverage for Claude)

The single most useful thing for self-debugging: when the user clicks through `RSIKKeyframe` once with the picks they care about, the script saves a JSON capture of all inputs + outputs. Claude then replays that capture as many times as needed from a plain terminal — no further clicks required.

### What gets captured

Every quantity needed to re-execute the workflow end-to-end without Rhino picks:

```jsonc
{
  "schema_version": 1,
  "captured_at": "2026-04-23T15:42:11",
  "robot_id": "dual-arm_husky_Cindy",
  "doc_unit_scale_to_mm": 1.0,
  "lm_distance_mm": 15.0,
  "include_self_collision": true,
  "include_env_collision": true,
  "target_bar_id": "B5",
  "left":  { "block_name": "T20_Male", "arm_side": "left",
             "ocf_world_mm": [[...4x4...]],
             "joint_type_user_text": "T20", "joint_subtype_user_text": "Male" },
  "right": { ... same shape ... },
  "base_frame_world_mm": [[...4x4...]],   // the SUCCESSFUL base frame after sampling
  "expected": {
    "final":    { "left": {"joint_names":[...], "joint_values":[...]},
                  "right": {"joint_names":[...], "joint_values":[...]} },
    "approach": { "left": {...}, "right": {...} }
  }
}
```

Notes:
- Captures the *successful* base frame post-sampling, not the original pick. Replay then exercises the full math chain (frame composition, OCF→tool0 dispatch, approach offset, dual-arm IK at both poses) but skips the Rhino-only base-sampling step.
- Brep is intentionally not serialised — sampling is Rhino-only.
- `expected` configs are the captured solver outputs; replay can compare against them with a tolerance.

### Implementation

1. Capture writer in `scripts/rs_ik_keyframe.py`: after the user clicks Accept, before clearing the preview, dump the capture to `tests/captures/<timestamp>_<bar_id>.json`. Always-on, uniquely named per click so nothing is overwritten.

2. Make `core/robot_cell.py` importable outside Rhino. `import scriptcontext as sc` becomes a try/except fallback to a local dict surrogate; everything else (URDF load, PyBullet client, `solve_dual_arm_ik`, `extract_group_config`) is already Rhino-free.

3. **`tests/replay_ik.py` — pure Python (no Rhino).** Reads a capture JSON, starts PyBullet headless, loads the robot cell, recomputes tool0 frames from the captured OCFs through the live `MALE_JOINT_OCF_TO_TOOL0` table, computes the approach offset, runs `solve_dual_arm_ik` for both final and approach poses, compares against `expected.*` configs within a joint-angle tolerance, and exits 0 / 1.

4. **`tests/replay_ik_with_viz.py` — `rhinoinside` Python.** Same pipeline as (3), plus boots `rhinoinside.load()` and exercises `core/ik_viz.show_state(final_state)` and `show_state(approach_state)` so the recently-fragile scene-viz code path is regression-tested too. Falls back gracefully when `rhinoscriptsyntax` / `scriptcontext` are not available in the rhinoinside environment.

### How the user uses it

- **Capture once:** click `RSIKKeyframe`, do the picks, Accept. Note the printed capture path.
- **Claude debugs forever:** `python tests/replay_ik.py tests/captures/<file>.json`. If a viz bug is suspected: `python tests/replay_ik_with_viz.py tests/captures/<file>.json`.
- **CI later:** a pytest fixture loops over all `tests/captures/*.json` and runs the pure-Python replay. New captures added by the user automatically become regression tests.

### What this catches without Rhino

- IK math regressions (unit conversion mm/m, base-frame composition, approach direction).
- Nested-dict OCF→tool0 dispatch bugs (the Rhino-side fallback behaviour from `_tool0_from_male` is replicated as a pure function).
- compas_fab solver regressions when the submodule SHA changes.
- (with rhinoinside) `core/ik_viz.show_state` exceptions, scene-object registration drift, transform-application bugs.

The two recurring Rhino-only categories — interactive picks (`GetPoint.Constrain`, `GetObject` filters) and document-side block / layer behaviour — still need a real-Rhino click, but those have been stable for several iterations.
