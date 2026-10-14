# Tests

Run with `python -m pytest tests/ -q` (`--viz` adds matplotlib visualisations). What the suites cover:

- **Geometry and joints** (`test_geometry.py`, `test_s2_t1.py`, `test_joint_pair_roundtrip.py`,
  `test_three_bar_scene.py`, `test_base_guide_geom.py`, ...): pure numpy / scipy.
- **Schema 2 through the shared core** (`external/bar_assembly_core`):
  - `test_process_builder.py`, `test_process_source.py`: the Rhino document (a fake Rhino API in `fake_rhino.py`, or a
    real `.3dm` read with rhino3dm) becomes a process that passes the core's file and plan checks;
  - `test_process_planning.py`: planning on the core's `CompasFabMirror` (tamp keyframe chain, support IK, the allowed
    part contacts, `scripts/plan_process.py` writing `solutions/`);
  - `test_process_display.py`, `test_ik_viz.py`, `test_plan_steps.py`: the parts drawn of a core scene and their
    placement, the viewers' steps, solutions read back;
  - `test_ik_status.py`: stale IK, decided by the core's plan check B15.
- **Plugin plumbing** (`test_rhino_env.py`, `test_session_log.py`): the one-call package install and the session log.

The planning and drawing suites need compas, compas_robots, compas_fab (the `external/compas_fab` submodule's `src/`
on the path) and PyBullet, plus the tamp submodule; they are skipped otherwise. They were last run on Rhino's own
CPython 3.9.10 with numpy 1.24.4, scipy 1.13.1, trimesh 4.12.2.

`tests/equivalence/` is not collected by pytest. `export_equivalence.py <old export> <new process folder>` compares
two exports of one `.3dm`: the old plugin's compas_fab export (hs/mocap-experiment @ 069275f) and this plugin's
`RSExportProcess` folder. It checks that both give the planner the same world (cells, states, flange poses, body
placement, colliding pairs) and that the old export, converted by the core, equals the new process. Every
difference is counted under a kind; the explained ones are named in its `KNOWN`, any other fails the run. It needs
about 4 GB and 6 minutes (2 with `--no-cells`), and prints the time of each step. `compas_compare.py` holds the
measures.

`tests/captures/` holds JSON snapshots of the retired legacy planning cell (the capture / replay flow and its
`core.capture_io` are gone); nothing reads them any more.
