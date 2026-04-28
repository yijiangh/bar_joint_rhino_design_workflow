# Tests

Two layers:

- **Existing pytest suites** (`test_geometry.py`, `test_kinematics.py`, `test_s2_t1.py`, `test_urdf_chain.py`) — pure-numpy / scipy / PyBullet checks. Run with `python -m pytest tests/ -v`.
- **Capture / replay** for the IK keyframe workflow — see below.

## Capture / replay (no Rhino clicking required)

The full `RSIKKeyframe` workflow saves a JSON snapshot whenever the user clicks Accept. Replay scripts re-execute the same scenario from a plain terminal so we can iterate on IK math, OCF→tool0 dispatch, approach offset, and the scene-viz path without touching Rhino again.

### Capture

Click `RSIKKeyframe` in Rhino normally. After Accept, the script writes:

```text
tests/captures/<YYYYMMDD_HHMMSS>_<bar_id>.json
```

Each file is self-contained (OCFs in mm, successful base frame, collision flags, robot id, expected per-arm joint configs, etc.). Brep geometry for the WalkableGround is intentionally NOT serialised — sampling is Rhino-only and the captured base frame is already the post-sampling result.

### Replay (pure Python — recommended for inner-loop debugging)

Headless. No Rhino, no rhinoinside. Validates: IK math, OCF→tool0 dispatch, approach direction, dual-arm IK convergence, expected vs actual joint angles.

```bash
python tests/replay_ik.py tests/captures/<file>.json
# Optional: tighten / loosen joint-angle tolerance (default 1e-3 rad)
python tests/replay_ik.py tests/captures/<file>.json --tol 5e-4
```

Exit code 0 on success, 1 on first failure. Output uses the same `[OK]` / `[X]` markers the Rhino script does.

### Replay with viz (rhinoinside — covers the scene-bake path)

Same pipeline plus `core.ik_viz.show_state` for both FINAL and APPROACH solutions, so any regression in the Rhino-side mesh-bake / scene-object path is caught.

```bash
pip install rhinoinside    # one-time
python tests/replay_ik_with_viz.py tests/captures/<file>.json
```

Requires Rhino 8 installed locally (Windows). Adds ~3-5 s startup for `rhinoinside.load()`.

### When to use which

| Symptom | Use |
|---|---|
| IK fails to converge / wrong joint angles | `replay_ik.py` |
| Bumped compas_fab submodule SHA, want regression check | `replay_ik.py` |
| Scene viz throws / robot mesh appears in wrong pose | `replay_ik_with_viz.py` |
| Interactive pick filter / GetPoint behaviour | Inside Rhino only — manual click |

### Debugging an IK collision rejection / acceptance

When the production IK rejects a configuration that visibly should be reachable, OR accepts one that visibly clips in Rhino, use the GUI debugger:

```bash
python tests/debug_ik_collisions.py tests/captures/<file>.json
python tests/debug_ik_collisions.py tests/captures/<file>.json --phase approach
```

It will:

- Boot a PyBullet GUI (so you can see the scene).
- Run IK twice — once with collision checking on (production path), once off (always returns a config). Tells you which path agrees with reality.
- Push the no-check config into the PyBullet world, run `performCollisionDetection`, and color every colliding link red.
- Print every contact pair as `<body>/<link>  <->  <body>/<link>  (N pt)`, plus a one-line diagnosis matching the most common failure modes (rejected with contacts, accepted with contacts, etc.).
- Keep the GUI open until you close it.

If the production-path IK rejects but the no-check config shows no PyBullet contacts, the solver likely tried a different seed that did clip; force a single attempt by setting `IK_MAX_RESULTS = 1` in [scripts/core/config.py](../scripts/core/config.py) and re-run the debugger.

### Building a regression suite

Captures accumulate in `tests/captures/` as the user clicks through different scenarios. Each new file becomes a regression test. A pytest wrapper that auto-discovers them is a one-liner away (Phase D in `tasks/test_pipeline_plan.md`).
