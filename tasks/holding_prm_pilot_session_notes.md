# Holding-constraint PRM pilot — session hand-off notes

## Update 2026-10-02 (third session): the roadmap as a planning backend, in changing scenes

Asked for: use a precomputed roadmap (robot + tools + bar only) in the real, changing
collision scene; test on B1, B3, B4 of `data_design_study/260920_RobArch_demo_revamp`;
adapt `scripts/headless_bar_action_planner.py` and `motion_planner/api.py`; keep
`plan_constrained_dual_arm` and add a flag to switch between BiRRT and roadmap.
Plan: `C:\Users\yijiangh\.claude\plans\pickup-from-tasks-holding-prm-pilot-sess-purring-umbrella.md`.
Report: `docs/holding_prm_report.pdf` Section 12 (source `docs/holding_prm_scene.tex`;
version 2 kept as `holding_prm_report_v2.pdf`).

### What was built (all in `external/husky_assembly_tamp`, uncommitted)

- `holding_prm/sampling.py`: `make_collision_fns` gives three checks on one scene
  (`full`, `robot_side`, `scene_only`) by editing a private copy of the cell state;
  `robot_side_signature` / `signature_differences` describe the load a roadmap was built for.
- `holding_prm/scenes.py` (new): reads the split export layout (`<bar>__J.json`,
  `<bar>__R.json`), the planned goal as seen from the base, `shifted_base`,
  `build_scene_cell` (real scene cut down to what a movement shows), `save_scene`
  (cell file keyed by geometry, state file by placement).
- `holding_prm/pilot.py` + `scripts/prm_pilot.py`: `build --problem-dir <export> --bar B4`
  (goal anchors = planned goal + 12 shifted bases; start anchors = the 18 carry poses).
- `holding_prm/parallel.py`: workers that open a scene from files or use the caller's
  planner; tasks `task_check`, `task_fine`.
- `holding_prm/refine.py`: `Tracker(scene_check=True)`: edges from the overnight file
  get a scene-only check of their stored waypoints, lazily; `spread_size`.
- `holding_prm/online.py` (new): `plan_on_roadmap` (Alg. 12 of the report), `RoadmapMismatch`.
- `motion_planner/api.py`: `plan_constrained_dual_arm(..., backend="birrt"|"prm", prm_options=...)`.
- `scripts/headless_bar_action_planner.py`: `--movement cdfm`, `--cdfm-backend {birrt,prm}`,
  `--prm-root/--prm-run/--prm-workers/--prm-tolerances-mm/--prm-tube-rounds/--prm-no-pretracked`,
  `--save-tag`, `--save-dir`, `--base-shift dx,dy,dyaw_deg`; writes `<...>.plan_info.json`.
- `scripts/prm_check_solved.py` (new): independent check of a saved path (production
  repair at 2 mm, full collision, lowest point, path length).
- `scripts/prm_runs/robarch_*.ps1`: precompute, plans per bar, split timing, everything
  one at a time (`robarch_all_quiet.ps1`).
- Tests: `benchmark/tests/test_holding_prm.py` now 31 (collision split on the real B4
  scene via `collision_split_helper.py`, mismatch refused, unknown backend refused,
  scene-blocked edge avoided, cache keys, quaternion sign); `test_search_recording.py`
  now names the smoke library's scene cache (two caches exist since `build-cells` ran
  for the RobArch export). `pytest benchmark/tests`: 36 pass.

### How to run

```
# once per export (trimmed scenes), then per bar: build, track, (export for the dashboard)
.venv\Scripts\python.exe scripts\benchmark_cdfm.py build-cells <problem folder> --bars B1,B3,B4
.venv\Scripts\python.exe scripts\prm_pilot.py build --name 260920_RobArch_demo_revamp__B4 ^
    --problem-dir <problem folder> --bar B4 --nodes 6000 --starts-file carry/carry_v1/starts.json --workers 8
.venv\Scripts\python.exe scripts\prm_pilot.py track --name 260920_RobArch_demo_revamp__B4 --max-score-mm 15 --workers 8
# plan the bar carry with either planner, then check the saved path on its own
.venv\Scripts\python.exe -u scripts\headless_bar_action_planner.py <data_design_study> ^
    --problem 260920_RobArch_demo_revamp --bar-action B4__J.json --movement cdfm ^
    --cdfm-backend prm --prm-workers 8 --no-replay --no-show-plot --save-tag prm --save-dir <out>
.venv\Scripts\python.exe -u scripts\prm_check_solved.py <problem folder> --bar B4 --tags prm --solved-dir <out>
```
`HUSKY_BENCH_ROOT` must be set (roadmaps are under `<root>/prm/<problem>__<bar>`).
Benchmark commands now need `--source-key`: the data root holds two scene caches.

### Results (idle machine, 16 logical cores; all in `<root>/prm/robarch_plans/`)

| | B1 | B3 | B4 |
|---|---|---|---|
| scene (built bars + joint halves) | 0 | 9 | 17 |
| roadmap build / tracking | 11 / 26 min | 11 / 62 min* | 8 / 114 min* |
| roadmap planner, 8 workers: plan / whole call | 28 / 89 s | 36 / 84 s | 30 / 63 s |
| ... of which worker start | 15 s | 17 s | 15 s |
| ... edges blocked by the scene | 0 | 0 | 9 of 23 |
| roadmap planner, 1 worker: plan | 57 s | 86 s | 93 s |
| search with / without the overnight tracking | 7 / 6 s | 7 / 12 s | 3 / 16 s |
| BiRRT, 3 runs (whole call) | 183, 188, 185 s (partial path) | 64, 65, 63 s | 1624, 504, 397 s |
| shifted base (4 per bar) | 3 planned, 1 goal unholdable | 4 planned | 4 planned |

\* ran while other jobs used the machine.

- 29 of 30 plans returned a path and all 29 pass the independent check; the other is a
  goal no arm posture can hold (B1, base 10 cm back-left, -5 deg).
- Collision split: 0 disagreements on 252 configurations per bar; scene-only check
  7-11 ms, full check 21-23 ms.
- B1: the BiRRT's start derivation reached no home anchor and returned a partial start
  0.41 m from the goal; the roadmap planner returns a whole path from a carry pose.
- B3: the BiRRT's straight track is collision-free (no search); it is the quicker one.
- B4: the roadmap planner's whole call is 6-26 times shorter; joint travel 190 deg
  against 563-779 deg.
- The overnight tracking (most of the precompute) saved at most 13 s online here.
- Both planners end 155-298 deg from the export's `target_configuration` (the script
  passes the goal as flange frames); later movements need re-solving, or pass `goal_conf`.
- No planner checks the floor (no ground body in the cell): two shifted-base paths of
  B4 went 1-4 cm lower than their goal (lowest 0.031 m).
- Smoothing (shared with the BiRRT) takes 17-57 s after a 30 s roadmap plan.

### Open decisions

1. Keep workers and roadmap alive across bars (saves 15 s per plan).
2. Floor: a ground body in the export, or a floor rule in both planners.
3. Goal posture: restrict to the authored `target_configuration` (pass `goal_conf`
   from the headless script) or re-solve the movements after the carry.
4. Skip the overnight tracking for sparse scenes? Tune node budget and the 15 mm limit.
5. Smoothing cost (its time budget is only checked between shortcut attempts).
6. Still open from before: bump the parent repo's submodule pointer; which carry poses to keep.

### Gotchas met this session

- Old single-file actions (`<bar>.json`, class `BarAssemblyAction`) no longer load with
  the current `rs_data_structure` (the class was split), so the old-layout roles of the
  headless script could not be re-tested.
- `--max-time 300` for the BiRRT is per attempt; a run may take five attempts.
- pybullet_planning's `interpolate_poses` reads a quaternion sign flip as a near-full
  turn (about 1500 steps); path poses are now made sign-continuous (`continuous_poses`).
- Worker processes re-import the calling script on Windows, so with the headless
  script as the caller they import the planner core (and empty its log), as the caller
  itself already does.
- A second precompute queue for B1 was started by mistake while the first still had B1
  queued; the first rebuilt B1 later with the same result (the build is deterministic).
- Experiment outputs must not go next to the bar actions (shared dataset folder): use
  `--save-dir`. Four early test files were moved to `<root>/prm/robarch_plans/dev/`.
- Leftover test roadmap: `<root>/prm/dev_robarch_B4` (300 nodes), safe to delete.

## Update 2026-10-01 (second session): refinement, carry search, precompute

Plan approved by the user (from their notes on `holding_prm_report.pdf`, saved as
`docs/holding_prm_report_v1.pdf`): `C:\Users\yijiangh\.claude\plans\pickup-from-tasks-holding-prm-pilot-sess-purring-umbrella.md`.
Decisions: refinement = exact tracker, then tube draws (way 2), then seeded BiRRT (way 1);
night before = bar + grasps known, goal not; carry poses = one per class, research use first
(no change to `core.py` or `cdfm_v1`); order = refinement, carry, precompute.

New code (all in `external/husky_assembly_tamp`, uncommitted):
- `holding_prm/tracking.py`: `track_edge` (knots every 2.5 deg on the joint line, bar pose halfway
  between the arms, BiRRT-style 1 cm / 0.025 rad waypoints, chain warm start, tie-break toward the
  roadmap line, steps > 2 deg re-walked at 2 mm / 0.002 rad = `fine_walk`), `track_edge_landing`
  (edge counts only if its chain arrives within the 10 deg gate of the far node; tries the other
  direction; else reason `branch_change`), `exact_path_report`.
- `holding_prm/refine.py`: `Tracker` (tracking verdict table, batched `assemble_many`, blame rule),
  `refine_query` (exact gaps loop, tube rounds, per-tolerance shortest exact route, `unresolved`
  routes for the BiRRT), `pretrack_roadmap` (overnight tracking of roadmap edges).
- `holding_prm/carry.py`: carry pose geometry (class axis + roll + grasp midpoint) and scoring.
- `sampling.py`: `path_tube` draw mode (added at the END of `SAMPLING_MODES`; old seeds unchanged).
- `search.py`: `minimax_to_goals` now scipy (20x faster, identical gaps); networkx version kept as
  `minimax_to_goals_nx` for the test.
- `pilot.py`: `--starts-file` for build and query (`file_starts`, `file_anchors`), paths may be
  relative to `<root>/prm`.
- `dual_arm_task_space_rrt/seeded.py` (imports core; never import from holding_prm): seeded BiRRT.
- Scripts: `prm_pilot.py refine|track`, `prm_bridge.py validate|birrt` (moves the core log folder to
  `husky_assembly_tamp/logs/prm_bridge/` before importing core; checks the real log is untouched),
  `prm_carry_search.py stage1|starts|stage2`, `scripts/prm_runs/*.ps1` (detached queues; output via
  `cmd /c` because PowerShell 5.1 turns native stderr into errors).
- Tests: 23 in `benchmark/tests/test_holding_prm.py`.

Findings so far (B18 goal 2, 4000-node roadmap, all 807 starts):
- E1 (160 deg edges): 801 instead of 657 starts reachable, but only through edges breaking holding by
  220-1787 mm; nothing within 30 mm changed; query 553 s vs 212 s.
- E4 (every IK branch, 10,848 draws): 47 starts within 10 mm, 471 within 30 (one-branch 2000-node
  seeds: 24-27 and 171-405); 769 s build, 232k collision checks; better per draw, worse per check.
- Refinement (before the fine-walk fix): exact paths for 11/11 starts within 5 mm, 34/36 within
  10 mm, 298/462 within 30 mm; 20 of the 21 BiRRT-solved starts included; 189 horizontal starts
  (BiRRT: 0). Production check then failed 31/38 on "endpoint_mismatch" = hidden branch hops at
  near-straight elbows; fixed by `fine_walk`; check on starts 117/312/12/115: 8 of 8 pass.
- Elbow branch change: about 5 % of edges have joint lines through a stretched elbow; their exact
  path ends on the mirror branch (finer steps do not help) -> `branch_change`.

FINAL tracker rules (after four rounds of the production check, 7/38 -> 217/244 -> 243/268 ->
249/249): steps > 0.5 deg re-walked at 2 mm (`fine_walk`); each edge tracked in BOTH directions,
each keeping its own waypoints; an edge direction counts only if its chain ends on the far node's
own IK solution (`SAME_SOLUTION_RAD` = 1e-3 rad), not merely within the 10 deg gate.

Final results, B18 goal 2, 4000-node roadmap, all 807 starts (`refine/all_k2.5_p10mm_mid*`):
- Exact paths: 11/11 starts within 5 mm, 34/36 within 10 mm, 245/462 within 30 mm (164 horizontal,
  81 back); ALL 245 (249 stored paths) pass `core.repair_joint_path` at 2 mm steps. BiRRT exp. 2
  solved 21 starts; 20 of them are among the 245. Refine 1210 s, production check 3607 s (8 workers).
- Tube rounds (way 2, 3 rounds x ~3400 nodes): exact within 10 mm 34 -> 120 (42 horizontal),
  within 5 mm 11 -> 86, within 2 mm 0 -> 20; best route 0.89 mm (start 206). +698 s.
- Seeded BiRRT (way 1) on the 24 hardest leftover starts: chains 0/24, ends-only 1/24 (120 s each).
  Their 49 failed edges: 30 branch changes, 17 IK-unreachable.
- Carry (E5, per-goal 3000-node roadmaps, 14 goals): back best class (today's back anchor ~ best
  candidate, 4/14 goals within 10 mm); horizontal: today's 0/14, best candidate (0.45, 0.10, 1.20) m
  roll 60 deg 2/14 (4 any); vertical nothing. Provisional picks in `asset/carry_poses.json`
  (horizontal_0, vertical_2, back_1) with top 3 per class listed; USER TO CHOOSE.
- Precompute (E6, 6000 nodes / 17 goals, `B18_overnight`): online query ~6 s + refine ~2 s after a
  10 s worker start; but 0 of 5 unseen goals within 10 mm -> 30k-node night build queued (queue E,
  `B18_night30k`, log `queueE_night_big_20261001.log`), chained after queue A.

- Night 30k (`B18_night30k`, 45 goals, 3 carry poses): build 35 min + tracking 16,883 edges 89 min.
  Online per unseen goal ~16 s query + ~13 s refine (8 s of each = worker start; most of the rest =
  reading the roadmap file). Goal 10: exact path (9.8 mm route), passes production check; goals 20/31
  within 12-13 mm; goal 2 40 mm (vs 2.9 mm from the library's 807 starts!); goal 47 no route.
  => fixing ONE carry pose per class throws away start-set freedom; next: a small family per class.
- Tube paths: 125/125 pass the production check. Variants (E2): within 10 mm 33/36 (end nodes only),
  24/24, 25/25, 27/27 (2000-node seeds).

Report v2: `docs/holding_prm_report.pdf` (33 pages; v1 kept as `holding_prm_report_v1.pdf`).
Rebuild: `.venv\Scripts\python.exe docs\figures\make_prm_figures.py`, then pdflatex twice in `docs/`.
Dev runs left in `B18_g2_n4000` (queries `dev_s117`, `dev_s16`, `dev_s80` and their refinements):
harmless, show up in that run's dashboard drop-down. Nothing committed.

Written 2026-10-01 at the end of the session that built and first ran the pilot
(2026-09-30). For a fresh chat to pick up. **Nothing here is committed yet**, and the
`external/husky_assembly_tamp` checkout also holds other sessions' uncommitted BiRRT
and benchmark edits (see "Git state" below).

## Goal

Build the "flipped priority" roadmap from the 2026-09-29 meeting with Caelan
(design draft: `external/husky_assembly_tamp/docs/prm_pilot_plan.md`) and use it to
answer: for B18 goal 2, does a joint-continuous path exist whose bar-holding error
stays under a small tolerance, and if not, how far off is it?

- Nodes = 12-joint configurations that hold the bar exactly (draw one arm's joints,
  forward kinematics to the bar pose, analytical IK for the other arm).
- Edges = straight lines in joint space, so no joint ever jumps (continuity holds by
  construction; no continuity gate).
- Edge score = worst holding error along the edge, in mm (rotation folded in at
  0.4 deg per mm). Collision is a hard reject.
- Query = search from a **start set** (every collision-free branch pair at every
  library start pose, on the library turn and the home turn) to a **goal set**
  (every branch pair at the goal pose). A minimum spanning tree gives every node its
  **gap**: the smallest tolerance at which it reaches the goal set, plus a parent
  pointer along that best route.

## The report (read this first)

`external/husky_assembly_tamp/docs/holding_prm_report.pdf` (19 pages, same style as
`cdfm_task_space_birrt_report.tex`). Source: `holding_prm_report.tex` +
`holding_prm_results.tex`; all tables and quoted numbers come from
`docs/figures/make_prm_figures.py`, which reads the saved runs. To rebuild:

```
cd external\husky_assembly_tamp
.venv\Scripts\python.exe docs\figures\make_prm_figures.py
cd docs; pdflatex holding_prm_report.tex; pdflatex holding_prm_report.tex
```

## Code (all new files, in `external/husky_assembly_tamp`)

| File | What it does |
|---|---|
| `husky_assembly_tamp/motion_planner/holding_prm/holding.py` | violation measure, numpy forward kinematics on cached arm bases, joint-space lines, 2*pi turns |
| `.../holding_prm/sampling.py` | own collision check, node sampler (4 draw modes), endpoint solver `expand_bar_pose` |
| `.../holding_prm/roadmap.py` | nearest-neighbour pairs, `evaluate_edge`, `edge_cost`, `Roadmap` tables |
| `.../holding_prm/search.py` | source/sink Dijkstra, `minimax_to_goals` (gaps), `lazy_search`, `path_report` |
| `.../holding_prm/parallel.py` | spawn `ProcessPoolExecutor`, one PyBullet scene per worker |
| `.../holding_prm/store.py` | files under `<root>/prm/`, provenance, dashboard copy |
| `.../holding_prm/pilot.py` | `build_roadmap`, `query_roadmap`, `export_dashboard`, `print_info` |
| `scripts/prm_pilot.py` | command line: `build`, `query`, `export`, `info` |
| `benchmark/dashboard/prm.html`, `static/js/prm-viewer.js` | roadmap page (imports the benchmark viewer, does not change it) |
| `benchmark/tests/test_holding_prm.py` | 15 tests (scene tests need `HUSKY_BENCH_ROOT`) |
| `docs/figures/make_prm_figures.py` | report figures and tables |

Rules kept on purpose (keep them):
- **Never import `dual_arm_task_space_rrt/core.py`** from this package. Importing it
  empties its log file, which a running BiRRT batch may be writing. That is why the
  package has its own collision closure (`SceneCtx.collision_fn()` imports core), reads
  the library JSON directly (`benchmark.library` imports core), and copies the
  dashboard itself. A test checks this in a fresh process.
- Results go only to `<root>/prm/`, never `runs/`, `runs.json` or `compare/`.
- Default 6 workers, so a BiRRT batch with 8 can run alongside.

## How to run

PowerShell, from `external\husky_assembly_tamp`, with
`HUSKY_BENCH_ROOT` = `C:\Users\yijiangh\Insync\yijiang94817@gmail.com\Google Drive - Shared with me\2025-03 Husky Assembly\data_experiment\cdfm_planner_benchmark`:

```
.venv\Scripts\python.exe scripts\prm_pilot.py build --name B18_g2_n4000 --library cdfm_v1 --bar B18 --goals 2 --anchor-starts 40 --nodes 4000 --workers 8
.venv\Scripts\python.exe scripts\prm_pilot.py query --name B18_g2_n4000 --goal 2 --starts all --tag all
.venv\Scripts\python.exe scripts\prm_pilot.py export --name B18_g2_n4000
.venv\Scripts\python.exe scripts\prm_pilot.py info --name B18_g2_n4000
.venv\Scripts\python.exe -m pytest benchmark\tests\test_holding_prm.py -q
```

- `--starts` takes `all`, classes (`free,blocked`) or indices (`0,12`).
- Long runs: launch detached (`Start-Process` with output to a log), not from a
  Claude Code background shell (10-minute limit). The 2026-09-30 batch files are
  in the session scratchpad (gone); their logs are in `<root>/prm/logs/`.
- Dashboard: `python -m http.server --directory <root> 8000`, then
  `http://localhost:8000/prm/<run>/dashboard/`. Pick a query in the dropdown; click a
  ball to see the configuration, gap and edges; "walk" steps along an edge.
- Timing: 4000-node build 232 s, all-807-starts query about 210 s (8 workers).

## Runs on disk (`<root>/prm/`)

| Run | What |
|---|---|
| `B18_g2_n500`, `_n1000`, `_n2000`, `_n4000` | node-budget sweep, seed 0; queries `s0_s12` and `all` |
| `B18_g2_n2000_seed1`, `_seed2` | seed variance |
| `B18_g2_n2000_allbranches` | every IK branch kept per draw (worse) |
| `B18_g2_n2000_noline` | no start-goal line draws (much worse precision) |
| `B3_g4-11-16_n2000` | B3 goals 4, 11, 16; queries `g4_night1`, `g11_night1`, `g16_night1` |
| `smoke_prm`, `dev_prm` | development runs (smoke library); `smoke_prm` has the lazy-vs-eager check |

## What the pilot found (B18 goal 2, 4000 nodes unless noted)

- Best gap 2.87 mm, from start 117 (a start the BiRRT solved). **No path under 2 mm
  at any budget.** The best route's error is a row of 1-3 mm peaks; only 2 of its 13
  edges exceed 2 mm (2.2 and 2.9 mm). It is a resolution limit, not a disconnection.
- Holding error of a straight joint edge grows with its length: median about 1 mm at
  5 deg, 3.6 mm at 10 deg, 6.9 mm at 15 deg. Only 1.1 % of free edges are under 2 mm.
- The gap ranks starts like the BiRRT did, without running it: BiRRT-solved starts
  (experiment 2) median 6.8 mm vs 24.3 mm for never-solved; the 5 smallest gaps were
  all BiRRT-solved. Same ordering on B3's night-1 starts.
- **All 21 BiRRT-solved starts and all 36 starts within 10 mm use the "back" carry
  anchor.** No horizontal or vertical start is within 10 mm. Production picks horizontal.
- For 610 of 657 reachable starts, a different branch pair or turn beats the library's
  own start configuration.
- 150 starts are unreachable at every budget (they sit in roadmap clusters with no
  edge of at most 90 deg to the goal's component).
- Start 0 (BiRRT 0/10 in night 1): gap 21-83 mm, set by a few long edges next to the
  start; draws around its anchor land 96-121 deg away from it.
- Cost: collision checks are 93 % of worker time, IK under 3 %. Edges parallelise
  8.0x on 8 workers. Lazy endpoint checking gives identical gaps to eager (380 starts,
  0 differences) at a quarter of the time.
- Seed variance at 2000 nodes: best gap 3.8-5.0 mm (stable); count within 30 mm
  171 / 400 / 405 (not stable).

## Next steps (in the order the data suggests)

1. **Refinement (plan step 6).** Around the best route: split over-tolerance edges at
   their worst point and project the split onto the holding surface (solve the other
   arm), or sample densely with a few-degree spread; search again. Natural home:
   a new `holding_prm/refine.py` + a `refine` subcommand. Target: a sub-2 mm route for
   start 117 / start 12.
2. **Local sampling mode at endpoints.** Add a draw mode with a spread of a few
   degrees around start and goal anchors (today's draws land 70-120 deg away).
   Should fix start 0's coverage.
3. **Bridge to production (plan step 7).** Convert a route to bar poses with the left
   flange + grasp, re-solve with `core.repair_joint_path` at production resolution and
   full collision. Must live in a separate script (it imports core).
4. **Try `start_home_anchor=back` in the BiRRT** on B18 goal 2: a one-flag test of the
   carry-anchor finding.
5. Maybe: map goal pair numbers to the library's numbering inside the query (today the
   query numbers pairs in solver order; the figure script does the mapping).

## Gotchas met this session

- Shell heredocs in this environment halve backslashes. Edit files that contain
  backslashes (LaTeX, regex, f-strings with `\\n`) with the editor, not `cat <<EOF`.
- `ls` in Git Bash adds a trailing `/` to folder names; don't feed it to `--name`.
- The dashboard bug "Cannot read properties of null (reading 'category')" on the bare
  address was fixed 2026-10-01 (first-visit check in `loadQuery`); all run pages were
  re-exported. If an old page still shows it, Ctrl+F5.
- The Chrome extension was not connected; pages were checked with headless Chrome over
  the DevTools protocol (`window.__prm` exposes the page state for such checks).

## Git state (nothing committed)

In `external/husky_assembly_tamp` (branch `yh/ssik-speedup`), this work is untracked
new files only:
`husky_assembly_tamp/motion_planner/holding_prm/`, `scripts/prm_pilot.py`,
`benchmark/dashboard/prm.html`, `benchmark/dashboard/static/js/prm-viewer.js`,
`benchmark/tests/test_holding_prm.py`, `docs/holding_prm_report.{tex,pdf}`,
`docs/holding_prm_results.tex`, `docs/figures/`.
The modified tracked files in that checkout (`api.py`, `core.py`, `ssik_ik.py`,
benchmark runner and dashboard JS, ...) belong to the other sessions' BiRRT work, not
to this pilot. Per the submodule rule, commit inside the submodule first, then bump the
pointer in this repo. This note itself is `tasks/holding_prm_pilot_session_notes.md`.
