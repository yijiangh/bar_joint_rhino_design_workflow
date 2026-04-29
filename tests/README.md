# Tests

Two layers:

- **Existing pytest suites** (`test_geometry.py`, `test_kinematics.py`, `test_s2_t1.py`, `test_urdf_chain.py`) — pure-numpy / scipy / PyBullet checks. Run with `python -m pytest tests/ -v`.
- **Capture / replay** for the IK keyframe workflow — see below.

## Capture / replay (no Rhino clicking required)

The full `RSIKKeyframe` workflow saves a JSON snapshot whenever the user clicks Accept (or opts in on failure). The capture is a v2 schema flattening a serialized `compas_fab.RobotCell` (cell geometry, content-hashed under `tests/captures/robot_cells/<sha8>.json`) plus a `RobotCellState` (poses + tool/RB attachments) plus IK target frames.

### Capture

Click `RSIKKeyframe` in Rhino normally. After Accept, the script writes:

```text
tests/captures/<YYYYMMDD_HHMMSS>_<bar_id>.json
tests/captures/robot_cells/<sha8>.json   # written lazily; reused across captures
```

Both `RSIKKeyframe` and `RSIKSupportKeyframe` write through `core.capture_io.save_capture_v2`. Replay loads via `load_capture` (hard-fails on `schema_version != 2`).

### Debugging an IK collision rejection / acceptance

The GUI debugger replays a capture in PyBullet and shows what compas_fab considered:

```bash
python tests/debug_ik_collisions.py tests/captures/<file>.json
python tests/debug_ik_collisions.py tests/captures/<file>.json --phase approach
python tests/debug_ik_support_collisions.py tests/captures/<file>_support_*.json
```

It will:

- Boot a PyBullet GUI, load the captured `RobotCell` (robot + tools + env rigid bodies).
- Tint env rigid bodies (built bars + joints) green so they're visually distinct from robot.
- Run IK twice — once with collision checking on (production path), once off (always returns a config). Tells you which path agrees with reality.
- Push the no-check config into the PyBullet world, run `performCollisionDetection`, and color every colliding link red.
- Print every contact pair as `<body>/<link>  <->  <body>/<link>  (N pt)`, plus a one-line diagnosis.
- Print `compas_fab.check_collision(verbose=True, full_report=True)` so you can read the CC.1–CC.5 PASS / SKIPPED / COLLISION breakdown.

If the production-path IK rejects but the no-check config shows no PyBullet contacts, the solver likely tried a different seed that did clip; force a single attempt by setting `IK_MAX_RESULTS = 1` in [scripts/core/config.py](../scripts/core/config.py) and re-run.

For interactive pick filter / GetPoint behaviour, replay inside Rhino (no headless equivalent).
