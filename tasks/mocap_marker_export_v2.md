# MoCap joint markers: export (Rhino) and bar-pose estimation (monitor)

Date: 2026-10-16. Two parts: **Part A** is implemented in this repo; **Part B** is a hand-over note
for a session on the Ubuntu machine (husky-assembly-teleop). Save both into `tasks/` of the
respective repo when implementation starts (this repo: `tasks/mocap_marker_export_v2.md`).

## Context

Su's `hs/mocap-experiment` (merged into `main` today, `3473583`) adds the `T20_MoCap` joint: a
female-joint variant with two motion-capture balls. Several such joints on a bar make the bar's
6-DoF pose observable in Motive, which we want for (1) using a grounded bar's measured pose as the
visual-servo target and (2) an as-built study after each assembly / hold action.

What exists today and why it is not what we want:

- The marker centres are already stored **joint-local** in the registry
  (`scripts/core/joint_pairs.json` › `T20_MoCap.marker_points_mm`: `M1 = (-10, 24.15, 25.5)`,
  `M2 = (-10, -24.15, 25.5)` mm in the block frame; picked by RSDefineJointHalf). That is the right
  intrinsic representation: a joint is rigidly mounted to its bar with a known pose, so the markers
  move with it exactly like the joint's collision mesh does.
- Su's `MoCapMarkers.json` (schema 1, `scripts/core/mocap_markers.py`) writes each marker in the
  **world frame of the final assembled state** (`world_mm`) plus `bar_mm`, and re-exports bar
  centre lines and joint 4x4 world transforms. All of that duplicates `RobotCell.json` and the
  BarAction rigid-body states. It also reads bar lines live from curves while joint poses come
  from the cached cell snapshot, so the two files can disagree, and a range export writes the
  whole design while `RobotCell.json` is cut to the range.
- Putting the markers *inside* `RobotCell.json` is not possible without patching compas_fab:
  `RigidBody.__data__` has exactly three keys and `__from_data__` is `cls(**data)`, so any extra
  key makes the monitor's `json_load` raise. The only free slot (mesh `attributes`) is a hack.
- On the monitor side, the old `bar_rig` fit (`mocap_experiment.py:1203-1267`: pair 82 mm markers,
  take midpoints, fit a line) gives 5 DoF and is wrong for `T20_MoCap` (pairs are 48.3 mm, pair
  centres sit 25.5 mm off the axis). Motive marker ids shift, so matching must not rely on them.

Decisions taken with YH (2026-10-16):

1. **Slim sidecar** `MoCapMarkers.json` schema 2 next to `RobotCell.json`: per bar body name, each
   marker's position in the **bar frame, metres** (same frame and units as the bar body's
   `RigidBodyState.frame`), plus the block-local points once. Nothing that the cell / BarActions
   already carry.
2. **Geometry-only matching + Kabsch** on the monitor; Motive ids are ignored.
3. The Rhino export **refuses** a bar whose marker layout cannot give a unique 6-DoF pose.

### A finding that shapes the layout rule (checked on the real export `261016_2ModulesDemo_YH`)

Pairwise-distance "signatures" alone cannot separate the two markers of a pair: on B20 the sorted
distance lists of M1 and M2 of one joint differ by only 0.35 mm (the other markers are ~1 m away
along the bar). A signature-only uniqueness rule would wrongly refuse B20. The distance-preserving
permutations that do exist on B20 are **mirror swaps**; fitting a *proper* rotation (det = +1) to
them leaves 48–70 mm residuals. So:

> A layout is ambiguous only if some **non-identity permutation** of its markers can be produced by a
> **proper rigid transform** with max residual ≤ 2 mm. Distance signatures are only a pruning step.

With that rule both B20 and B21 are unambiguous (even at 5 mm). The same logic drives the monitor's
matcher: a 3-point hypothesis cannot reject mirrors or pick M1 vs M2; the decision comes from the
full-set residual with det = +1 enforced.

Two exact (zero-residual) ambiguities follow from the `T20_MoCap` geometry and become the design
rule of thumb written into the docs:

- **Two joints on a bar are never enough**, whatever their spacing or relative turn: a half-turn
  about the line bisecting the two plate normals (through the bar axis midway between the joints)
  maps the set onto itself end-for-end.
- **Joints that all face the same way are never enough**: a half-turn about the plate midline
  (parallel to the bar, 25.5 mm off its axis) swaps M1↔M2 in every joint.
- Rule: **≥ 3 MoCap joints per tracked bar, not all facing the same way, not mirror-symmetric about
  the bar's middle.** The check below enforces the general condition; the rule is the hint shown
  when it refuses. Both bars in the current demo (4 joints, mixed facings) satisfy it.

---

## Part A — Rhino side (this repo)

### A1. Marker diameter in the registry

- `scripts/core/joint_pair.py` `JointHalfDef` (~L80-217): add `marker_diameter_mm: float = 0.0`
  after `marker_points_mm`; `0.0` = not recorded. Add to `to_dict` / `from_dict` with a default, like
  `bar_cradle` (old registries load). Negative → `ValueError`.
- `scripts/core/joint_pairs.json` `T20_MoCap`: add `"marker_diameter_mm": 19.0` (measured from
  `asset/T20_MoCap.obj`: sphere radius 9.50 mm; Motive's ~18 mm "size" is its own estimate).
- `scripts/rs_define_joint_half.py` `_pick_marker_points` (L161-205): it already has the bounding
  box corners; compute the diameter as the mean extent, refuse if the three extents of one sphere
  differ by > 0.5 mm or two spheres differ by > 0.5 mm; return `(points, diameter_mm)` and pass it
  into the `JointHalfDef(...)` call (~L547-561). Pure helper `bounding_box_diameter_mm(corners,
  scale_to_mm)` in `scripts/core/marker_points.py` next to `bounding_box_centre`.

### A2. New pure module `scripts/core/mocap_layout_check.py` (numpy only; metres)

Also the test oracle for the monitor's matcher.

```python
DEFAULT_TOL_M = 0.002
class MoCapLayoutError(RuntimeError): ...
@dataclass(frozen=True) class Symmetry: permutation, angle_deg, axis, max_residual_m
@dataclass(frozen=True) class LayoutReport: n_markers, tol_m, collinear, symmetries, nearest_miss_m
    ok -> n_markers >= 3 and not collinear and not symmetries;  reason() -> one plain sentence
def fit_rigid(source, target) -> (R, t)          # Kabsch/SVD, det(R)=+1 enforced (flip last singular vector)
def match(design_m, measured_m, tol_m) -> list[Symmetry]   # every assignment whose proper fit has max residual <= tol
def check_layout(points_m, tol_m) -> LayoutReport          # = match(P, P) minus identity, + <3 / collinear tests
def require_unambiguous(points_m, tol_m, name) -> LayoutReport   # raises MoCapLayoutError naming `name`
```

`match` in plain words: (1) pairwise distance matrices of both sets; (2) per measured marker, the
candidate design markers are those whose sorted distance row agrees entry-by-entry within `2*tol`
(loose pruning only — it must never drop a true match); (3) depth-first assignment, pruning as soon
as an already-assigned pair's distance disagrees by > `2*tol`; (4) for every complete assignment fit
the proper rigid transform and keep it only if the **max** residual over all points ≤ `tol`.
`check_layout` adds: fewer than 3 markers → not ok (a pair leaves roll free); collinear (second
singular value of centred points ≤ tol) → not ok; `nearest_miss_m` = smallest residual among
rejected candidates (printed as a NOTE when < 3×tol, so "fine but only by a few mm" is visible).
`config.MOCAP_LAYOUT_TOL_MM = 2.0` is the Rhino-side knob.

### A3. Payload v2 — `scripts/core/mocap_markers.py`

Replace `designed_markers` (L78-175) with:

```python
def build_payload(bodies: dict, halves: dict, bar_seq: dict, *, source: str = "", tol_m: float = DEFAULT_TOL_M) -> dict
```

- `bodies` is the cell snapshot **already cut to the export** (same set as `RobotCell.json`).
  Bars: `kind == "bar"`, frame = `info["frame_world_mm"]` (set by `env_collision._bar_world_frame_mm`,
  identical to `canonical_bar_frame_from_line`). Joints: keep `mocap_joints(bodies)`. **No live
  curves, no `bar_lines` argument** — the two files can no longer disagree.
- Bar-frame marker in metres: reuse `marker_points.markers_in_frame_mm(joint_frame_world_mm,
  half.marker_points_mm, bar_frame_world_mm)` (= `inv(bar) @ joint @ point`), `/1000`, round to
  5 decimals.
- Refuse with `MoCapExportError(RuntimeError)` (no silent fallback; the old `warnings` list goes
  away): a MoCap joint whose block has no `marker_points_mm` or `marker_diameter_mm == 0`; a joint
  whose `parent_bar_id` has no bar body in the set.
- Order joints along the bar (z of joint origin in the bar frame), labels with `label_sort_key`.
- Per bar with markers: `require_unambiguous(points_m, tol_m, name=f"bar {bar_id}")` — the error
  names the bar, the marker count, the first symmetry ("end-for-end half-turn, residual 0.0 mm")
  and the hint ("add a third MoCap joint turned about the bar / move a joint so the layout is not
  mirror-symmetric").
- Keep `marker_count`, `summary` (adapted). Add `world_points_mm(payload, bodies) ->
  {marker_name: xyz_mm}` (bar body frame @ bar_m·1000) so the Rhino dots come from the same numbers.

Schema 2 (example; positions illustrative):

```json
{
  "schema_version": 2, "source": "261016_2ModulesDemo_YH.3dm", "units": "m", "cell_file": "RobotCell.json",
  "bar_frame": "frame of the bar body of the same name in RobotCell.json / BarActions states: origin at the bar start, z along the bar; world = bar_state_frame @ bar_m",
  "marker_name": "<joint object name>_<label>, e.g. J20-21_mocap_M1 (name the Motive marker this)",
  "layout_tolerance_m": 0.002,
  "marker_blocks": {"T20_MoCap": {"marker_diameter_m": 0.019,
                                  "points_block_m": {"M1": [-0.01, 0.02415, 0.0255], "M2": [-0.01, -0.02415, 0.0255]}}},
  "bars": {
    "bar_B20": {"bar_id": "B20", "assembly_seq": 1, "layout": {"n_markers": 8, "nearest_miss_m": 0.0412},
                "markers": [{"name": "J20-22_mocap_M1", "joint": "joint_J20-22_mocap", "label": "M1",
                             "block": "T20_MoCap", "bar_m": [-0.0274, -0.0220, 0.0849]}, "..."]}},
  "bars_without_markers": ["bar_B22", "bar_B23"]
}
```

Bars keyed by cell body name, listed only when they have markers; `bars_without_markers` lists the
other bar bodies of the same cut (assembly order). Dropped from v1: `world_mm`, `start/end/length`,
joint `frame_world_mm`, `position_mm`, `marker_indices`, `index`, `warnings`, `"world"`.

### A4. Rhino glue — `scripts/core/rhino_mocap_markers.py`

Split `export_mocap_markers` (L103-138) so the check runs **before anything is written**:

```python
def build_mocap_sidecar(bodies: dict, caller: str) -> dict        # may raise MoCapExportError / MoCapLayoutError
def write_mocap_sidecar(payload: dict, root: str, caller: str) -> str | None
def label_markers_in_rhino(payload: dict, bodies: dict) -> int
```

- `build_mocap_sidecar`: `halves = load_joint_registry().halves`; `bar_seq` from `get_bar_seq_map()`;
  `tol_m = config.MOCAP_LAYOUT_TOL_MM / 1000`; `source = sc.doc.Name`. Delete `_bar_lines_mm` and
  `_assembly_order`.
- `write_mocap_sidecar`: zero markers → **delete** a stale `<root>/MoCapMarkers.json` if present,
  clear the label layer, return `None`. Otherwise write, label, print summary + nearest-miss notes.
- `label_markers_in_rhino(payload, bodies)`: positions from `world_points_mm`; dot text
  `"B20 #3 M1"`, object name = marker name, user text as today; one polyline per joint (group
  consecutive markers by `joint`). Layer/colour unchanged (`config.LAYER_MOCAP_MARKER_LABELS`,
  `MOCAP_PREVIEW_COLOR`).

### A5. Callers

- `scripts/rs_export_robotcell.py` (L118-123): `payload = build_mocap_sidecar(...)` in a `try`
  **before** `write_robot_cell`; on `RuntimeError` → `rs.MessageBox("Export stopped, nothing was
  written:\n\n<exc>")`, print, return. Then write the cell, then `write_mocap_sidecar`.
- `scripts/rs_export_all_bar_actions.py`: inside the existing cut-bundle `try` (~L371-394, which
  already has the "Export stopped, nothing was written" handler): `mocap_bodies = {k: v for k, v in
  collision_bodies.items() if k not in dropped}` → `build_mocap_sidecar`. Replace the call at ~L445
  with `write_mocap_sidecar(mocap_payload, ...)`; fix the comment at L443-444 (now the same cut as
  `RobotCell.json`); add `MoCapMarkers.json` to `planned_files` (~L402-403).
- `scripts/core/export_root.py` docstring: adjust the one-line description.
- `scripts/rs_export_prefab.py` `_mocap_entry` (L106-117): **keep** (fabrication document, mm, live
  doc); add `"marker_diameter_mm"`; optionally print a `check_layout` NOTE per bar (never refuse
  here — fabrication must not be blocked).

### A6. Docs

- `docs/mocap_marker_export.md`: rewrite — why a slim sidecar; writing it (both commands, same cut,
  stale file deleted, refusal rule); schema 2 field by field; the layout rule in plain words (what
  ambiguous means, the two always-ambiguous cases, the three-joint rule, 2 mm tolerance,
  `nearest_miss_m`); how the monitor uses it (Part B summary). Drop the "planned move into prefab"
  status paragraph and the v1 schema.
- `README.md`: table row ~L57; MoCap section ~L231-314: step 1 becomes the three-joint rule ("two
  joints are never enough; the export refuses"); step 4 + table → v2 fields, metres, bar body frame,
  file removed when no markers.
- `docs/coordinate_conventions.md` §2 Bar Frame: one line that `MoCapMarkers.json` uses this frame
  in metres; MoCap joint bullet (~L65) mentions `marker_diameter_mm`; prefab `mocap` block (~L165)
  gains `marker_diameter_mm`.

### A7. Tests

`tests/test_mocap_layout_check.py` (new; build point sets with `fk_single_block_frame` + registry
points like `tests/test_mocap_markers.py` L27-31, /1000):
1. `fit_rigid` recovers a known R,t; a mirrored set gives det=+1 and a large residual.
2. One joint → not ok ("fewer than 3"). 3. Collinear → not ok.
4. Two joints same facing → ≥ 3 exact symmetries (angle ≈ 180°, residual ≤ 1e-6).
5. Two joints 90° apart, unequal spacing → exactly the end-for-end permutation `(3,2,1,0)`; the
   within-pair swaps are NOT symmetries (tens of mm) — pins "two joints are never enough".
6. Three same-facing joints → exactly one symmetry (all pairs swapped).
7. Three joints 0/90/0, equal and unequal spacing → ok, `nearest_miss_m > tol`.
8. Three joints 0/180/0 equal spacing → refused; middle joint shifted 20 mm → ok.
9. `match` with the measured set rotated/translated/shuffled, 0.5 mm and 1 mm jitter → exactly one
   assignment = inverse shuffle.
10. B20-style long bar: a pair's sorted distance rows differ by < 1 mm AND layout is ok — guards
    against reintroducing a signature-only rule.
11. `require_unambiguous` message contains the name. 12. 12 markers finishes well under 1 s.

`tests/test_mocap_markers.py` (rewrite against `build_payload`; fixture bodies also carry bar
bodies with `frame_world_mm = canonical_bar_frame_from_line(*B)` and the registry `halves`):
schema fields; bars keyed `bar_B20`, untracked bars only in `bars_without_markers`; `assembly_seq`
from a non-consecutive seq map; joint order along the bar + numeric label order + `jnc.marker_name`
names; **consistency invariant** `bar_frame_world_mm @ (bar_m·1000) == joint_frame_world_mm @
block_point` within 0.02 mm and `world_points_mm` agrees; cut set (B21 and its joints removed →
absent, no error); refusals (missing parent bar, no marker points, diameter 0, two-joint bar,
three same-facing joints) with the right names in the message; `json.dumps` round trip, 5-decimal
rounding. `tests/test_mocap_role.py`: `marker_diameter_mm` default, round trip, old JSON loads.
`tests/test_marker_points.py`: `bounding_box_diameter_mm` on a 19 mm cube.

### A8. Verification

1. `PYTHONPATH=~/.rhinocode/py39-rh8/site-envs/scaffolding_env-d8rZiG+M ~/.rhinocode/py39-rh8/python.exe -m pytest -q -p no:cacheprovider`
   (default `python` lacks compas). All green, incl. the two MoCap test files.
2. Rhino: open `261016_2ModulesDemo_YH`, reload engine, RSRebuildRobotCell, RSExportRobotCell into a
   scratch folder → `MoCapMarkers.json` is schema 2; dots sit on the spheres; B20/B21 pass the check.
   Then: remove one joint from B20 to leave 2 → export refuses naming B20, writes nothing. Export a
   design with no MoCap joints → stale file deleted.
3. RSExportAllBarActions with a From..Until range → sidecar's `bars` ∪ `bars_without_markers` equals
   the `bar_*` names in that `RobotCell.json`.
4. Optional cross-check from the monitor side once Part B exists: feed the sidecar + a BarAction
   state through `fit_bar_from_cache` with synthetic markers = `bar_state_frame @ bar_m`.

Implementation order: A1 → A2 (+tests) → A3 (+tests) → A4/A5 → A6. One commit unless asked.

---

## Part B — Monitor side (hand-over for the Ubuntu session; repo `husky-assembly-teleop`)

Save as `tasks/2026-10-16_bar_mocap_fit.md` in that repo (its AGENTS.md asks for that). All paths
relative to the repo root; line numbers checked against the current checkout on 2026-10-16.

### B0. Facts the implementer should know

- The sidecar on disk today is still schema 1 (mm, `bars` list). The loader accepts **schema 2
  only** and raises a clear error otherwise; until Part A ships, hand-convert the v1 file for a live
  test (`bar_mm/1000`, keys `bar_<bar_id>` / `joint_<name>`).
- `T20_MoCap`: M1/M2 cannot be told apart by distances (fact above); a 3-point triple cannot reject
  a mirror (a flipped triangle is reachable by a proper rotation); mirror rejection comes from
  det=+1 Kabsch **and** scoring on all markers with ≥ 4 inliers. Same-label markers on same-roll
  joints are exactly collinear → such triples must be skipped.
- `pp.superimposition_matrix` imports pybullet via the package `__init__` → the pure module carries
  its own 12-line Kabsch (same SVD + det fix as `transformations.py:845-856`).
- NatNet `__unpack_labeled_marker_data` (`optitrack/NatNetClient.py:526-581`) decodes
  occluded/point-cloud/model-solved flags but passes only `pos,size,error`; model id 0 is the
  unlabelled bucket; rigid-body descriptions (`:836-915`) carry asset name + streaming id, stored in
  `get_latest_data_descriptions().rigid_body_list` after `request_model_definitions()`
  (`husky_monitor.py:12210`).

### B1. New pure module `husky_assembly_teleop/bar_mocap_fit.py` (numpy + stdlib only)

Data classes: `DesignMarker(name, joint, label, bar_m)`, `BarMarkers(bar_name, assembly_seq,
markers; points(), names())`, `MoCapMarkerSet(schema_version, source, units, bar_frame_note,
blocks, bars: dict[str, BarMarkers], bars_without_markers, path, warnings)`,
`MoCapMarkersError(RuntimeError)`.

- `load_mocap_markers(problem_root) -> MoCapMarkerSet` — plain `json.load` (pattern:
  `WalkableGround.json`, `cfab_session.py:146`); raise on missing file, `schema_version != 2`
  (message names the found version, says to re-export from Rhino), `units != 'm'`, a bar with < 3
  markers, malformed `bar_m`; warn when a bar's markers are collinear.
- `kabsch(src, dst) -> 4x4` proper (det=+1).
- `match_and_fit(sensed_points_m (N,3), design_points_bar_m (M,3), *, tol_m=0.004,
  side_tol_m=0.002, min_inliers=None → max(4, ceil(0.75·M)), min_triangle_height_m=0.015,
  max_sensed=24) -> BarFit`:
  1. checks (N,M ≥ 3; N ≤ max_sensed else fail "too many markers in view");
  2. distance tables `dS`, `dD`;
  3. design triangles with smallest altitude ≥ 15 mm (skips collinear chains; pair+far-marker
     triangles have ≥ 25 mm altitude);
  4. per triangle, all **ordered** sensed triples whose three side lengths match within
     `side_tol_m` (mask on `dS`) — deliberately includes M1/M2-swapped and mirror hypotheses;
  5. per hypothesis: Kabsch on the triple → transform all design points → greedy one-to-one
     nearest assignment within `tol_m` → score `(n_inliers, -rms)`;
  6. refine: refit on all inliers, re-assign once, refit;
  7. ambiguity: dedupe hypotheses by inlier correspondence set; `second_best` = best set ≠ winner;
     `ambiguous = second_best.n == best.n and second_best.max_residual ≤ tol_m`;
  8. `ok = n_matched ≥ min_inliers and not ambiguous`, else a plain-English `reason`.
  Cost: N=M=8 → ~56 triangles × ≤ 336 triples, tens of ms; N=16 well under 1 s.
  `BarFit`: `ok, reason, bar_name, T_world_from_bar, correspondences, sensed_ids, residuals_m,
  rms_m, max_residual_m, n_matched, n_design, unmatched_sensed, missing_design, ambiguous,
  second_best, n_hypotheses, to_json()`.
- `fit_bar_from_cache(cache {id: {'pos','size','error'}}, design, *, age_s, max_age_s, **kw)`:
  int or string ids (saved takes use strings); drop markers with `error` > 3 mm or `size` outside
  12–26 mm; refuse when snapshot older than `max_age_s`.
- `pose_deviation(T_meas, T_design) -> PoseDeviation(dpos_mm, dang_deg, dpos_world_mm,
  dpos_bar_mm, rotvec_bar_deg, to_json())` — errors re-expressed in the **design bar frame**
  (x,y lateral; z along the bar).
- `transform_points(T, pts)`, `design_bar_axis_endpoints(T, length_m)` for drawing.

Constants to confirm on the first real take: `tol_m = 4 mm` (10× below the 48 mm pair spacing,
3–4× above Motive residual ~0.8 mm); `side_tol_m = 2 mm` (raise to 3 mm if the jig places joints
slightly off — check `max_residual_m` on the first live fit).

### B2. Monitor integration (`husky_monitor.py`, `husky_world.py`)

- **Designed bar frame**: `rigid_body_state_frame_for_bar(self, bar_name) -> 4x4 | None` next to
  `get_movement_start_bar_pose` (`:2072-2155`): active bar → that function (handles the held case);
  other bars → `movement_start_state.rigid_body_states[bar_name].frame` via
  `Transformation.from_frame`; `None` when attached to another robot.
- **Load sidecar** in `_finish_action_load` (`:3457`, after the cfab block ends `:3534`):
  `self._mocap_markers = load_mocap_markers(self._problem_dir())` in a try; warn on
  `MoCapMarkersError`. Init `self._mocap_markers = None` near `:495`.
- **Per-bar Motive assets**: `_register_mocap_bar_assets()` walks
  `get_latest_data_descriptions().rigid_body_list`; for each asset whose decoded name is a key of
  `self._mocap_markers.bars` set `name_from_mocap_id[id] = name` (refuse+warn on a clash, e.g.
  1002 = `bar_rig`); called lazily from `measure_bar_pose` and once from the tick. Fallback config
  `MOCAP_BAR_STREAMING_IDS = {}` in `husky_world.py` next to the `bar_rig` registration
  (`:535-538`). Keep `bar_rig` untouched.
- **Fix the labelled-marker cache** (`receive_labeled_marker`, `:12302-12325`): key by
  `name_from_mocap_id.get(model_id)` or `'_unlabeled'` for model 0; replace the **whole dict** per
  model per frame under a new `self._mocap_cache_lock`; record
  `self._mocap_labeled_marker_stamp[key] = time.monotonic()`. Add
  `labeled_markers_snapshot(name) -> (deep copy, age_s)` and switch the two existing readers
  (`husky_world.py:1139-1143`, `husky_monitor.py:9706-9709`) to it. Register the listener whenever
  `USE_MOCAP` is on (today gated on `BAR_ACTION_MOCAP_ACCURACY_TEST`, `:12197-12198`). Optional: add
  `'flags': param` to the NatNet listener dict (`NatNetClient.py:569-573`).
- **`measure_bar_pose(bar_name=None, *, max_age_s=0.5, allow_unlabeled=True, draw=True) ->
  BarFit | None`**: snapshot for the bar's asset (fallback `'_unlabeled'`), `fit_bar_from_cache`,
  `pose_deviation` against `rigid_body_state_frame_for_bar`, one log line (bar, source, n/M, rms,
  max, dpos, dang, dpos_bar, ambiguous), debug drawing (measured axis like
  `_show_assembled_bar_ghost` `:8142-8170` + matched design markers as points, unmatched sensed in
  red; keep uids for clearing).
- **Buttons** (`:12002-12008`): `Measure MoCap bar (active)`, `Record MoCap bar take`
  (mirrors `record_bar_take_with_shared_viz` `:9693-9731`; take = `{bar_name: markers,
  'markerset_key', 'bar_fit', 'deviation', 'marker_source', 'marker_age_s'}` appended to
  `marker_set_data`). Keep the three `bar_rig` buttons.

### B3. Recording and offline re-analysis

- `save_markerset_data` (`husky_world.py:1213-1294`, payload `:1280-1291`): add
  `mocap_markers_file`, `mocap_markers_design` (the active bar's design table so a take is
  self-contained), `design_T_world_from_bar` (4x4). Existing fields stay; the old
  `1_compare_to_cell_state.py` keeps working.
- New `data/bar_holding_acc_data/4_refit_mocap_bar.py <take.json> [--markers] [--bar] [--tol-mm]
  [--out]`: per take pick the marker dict (`markerset_key` → `'bar_rig'` → first dict of `{'pos'}`
  entries), require `mocap_axis_convention == 'rhino'`, `fit_bar_from_cache`, `pose_deviation`
  against `design_T_world_from_bar` (else `bar_start_position/quaternion`), one row per take, write
  `_refit.json`. On `20261001/bar_holding_acc_20261001_1514.json` against a T20 design every take
  must report `ok=False` (old 4-marker rig) — a good negative test.

### B4. Follow-ups (context only, not in this round)

- **Grounded-bar visual servo**: before `servo_to_movement_start_live` (`husky_world.py:1882`; the
  target is copied from `monitor._last_ik_target_ee_frames`, `:2053-2058`), measure the receiving
  bar once, `T_corr = T_meas @ inv(T_design)`, apply to each authored tool0 frame before IK/replan;
  refuse if `dpos > 30 mm` or `dang > 3°` (points at a wrong match, not an as-built offset); behind
  an operator checkbox. This is the "close the loop on the bar" exception in
  `doc/bar_mount_preload_report.md:183-193`.
- **As-built logging**: after each assembly/hold action is marked done, `measure_bar_pose` for every
  visible instrumented bar → append `{movement_id, bar, fit, deviation, timestamp}` to
  `<problem>/as_built.jsonl` next to `progress.json`; a dashboard run kind in
  `dashboard/run_writer.py` style for designed-vs-measured and per-marker residual (bending) plots.

### B5. Tests — `test/test_bar_mocap_fit.py` (numpy + pytest)

Fixture: joints at z = 0.075/0.430/1.135/1.670 m with rolls 0/90/180/45°, block-local M1/M2 from
the registry (metres). Cases: exact recovery after shuffle (< 1e-9); 1 mm noise → dpos < 1 mm,
dang < 0.2°; one dropped → ok, 7/8, `missing_design`; one ghost 0.3 m away → ok, in
`unmatched_sensed`; three dropped → not ok, reason "5 of 8"; mirrored input → not ok, det>0 on
every returned fit; symmetric layout (rolls all 0) → `ambiguous`, `second_best.n == 8`, not ok;
M1/M2 swap resolved by full-set scoring; collinear design → clear failure; loader round trip,
schema-1 fixture → error naming version 1, missing file → error; real negative on the Oct-1 take
(skip if absent) → `ok=False`, no exception; `pose_deviation` on 5 mm along z and on 1° roll.

### B6. Verification and open questions for the real system

Offline: `python -m pytest test/test_bar_mocap_fit.py -q`, then `colcon build --symlink-install
--packages-select husky_assembly_teleop` and the existing servo/skeleton tests.
Live: `USE_MOCAP=1`, `MOCAP_AXIS_CONVENTION='rhino'`, a Motive asset named `bar_B20`; load the
action → "[mocap bars] N instrumented bar(s)" and the id→bar map; bar on its designed spot →
8/8, rms ≈ 1 mm, dpos a few mm; slide it 50 mm → `dpos_bar_mm[2] ≈ 50`; cover one marker → 7/8;
disable the asset → `source=unlabeled` still fits; record + save → `4_refit_mocap_bar.py`
reproduces the live numbers.
Check on site: NatNet version (marker names need ≥ 4, residual ≥ 3); are per-bar assets defined
and named like the bars; do labelled markers still arrive as model 0 when the asset is off; does
Motive duplicate a tracked asset's markers in model 0 (then dedupe < 1 mm); first-take pairwise
distances vs design within `side_tol_m`; bar-frame z sign (`husky_monitor.py:8160-8161` comment)
— a wrong sign shows as 180° `dang_deg` on an otherwise perfect fit.

---

## Deliverables of this round (this repo)

1. Part A implemented and tested; `MoCapMarkers.json` re-exported for `261016_2ModulesDemo_YH`.
2. `tasks/mocap_marker_export_v2.md` (Part A, for the record) and
   `tasks/mocap_bar_pose_monitor_handover.md` (Part B, to copy to the Ubuntu session).
