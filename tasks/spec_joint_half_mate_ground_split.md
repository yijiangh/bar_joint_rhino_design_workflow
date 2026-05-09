# Spec: Split joint definition into Half / Mate / Ground

Decided 2026-05-09 via plan-mode interview (revised after first review).
Implementation pending.

COMPLETED 

## Summary

Replace the monolithic `RSDefineJointPair` workflow with **two** commands:

1. **`RSDefineJointHalf`** — single entry point that first asks the user
   to choose `kind = Male | Female | Ground`, then runs the appropriate
   sub-flow. Male/Female pick block + bar axis + screw axis + center +
   collision meshes. Ground picks block + bar axis + collision meshes
   (no screw). All three persist into the same `joint_registry.json`.
2. **`RSDefineJointMate`** — join two existing male/female halves into a
   mate; auto-derive `contact_distance_mm` from the as-placed pose;
   print for confirmation. Ground joints are NOT mated (they have no
   counterpart).

The collision-mesh-on-disk problem is solved by adding a **per-half OBJ**
(multi-object, NOT merged) at `asset/<block_name>_collision.obj`, exported
via Rhino's `_-Export` of the user-picked meshes. This matches the
existing convention used by tools and IK already consumes
`collision_filename` via `_resolve_tool_collision_paths` /
`env_collision._joint_collision_paths`.

## Data model changes (`scripts/core/joint_pair.py` -> rename to `joint_registry.py`)

### Existing dataclasses

* `JointHalfDef` — keep as-is, already carries `collision_filename`,
  `M_block_from_bar`, `M_screw_from_block`, `block_name`, `asset_filename`.
  Add no fields.
* `JointPairDef` — keep field names but **change registry storage** so
  halves are referenced by `block_name` rather than embedded.

### New dataclass

```python
@dataclass(frozen=True)
class GroundJointDef:
    name: str                          # registry key
    block_name: str
    asset_filename: str                # .3dm block export
    collision_filename: str            # multi-object OBJ
    M_block_from_bar: np.ndarray       # 4x4, intrinsic to this ground joint
    jp_range: tuple[float, float] = DEFAULT_JP_RANGE
    jr_range: tuple[float, float] = DEFAULT_JR_RANGE
    # No screw frame: ground joints have no bore/screw.
```

Note: ground joints have **no `M_screw_from_block`** because there is
nothing to align against. At placement, the constraint is "block local X
is colinear with the parent bar axis (sign user-flippable); local Z
points toward the world +Z surface".

### Compatibility check with existing placement / solver workflow

Verified by inspecting `core/joint_pair_solver.py`,
`core/joint_placement.py`, `core/joint_auto_place.py`,
`core/env_collision.py`, `rs_bar_snap.py`, `rs_bar_brace.py`. All
consumers touch `JointPairDef` only via these attributes:

* `pair.name`
* `pair.contact_distance_mm` — used by `optimize_pair_placement`
  (`orientation_weight_mm`) and by the bar-snap target distance.
* `pair.jr_range` — used by the solver as bounds AND seed range. Same
  value applied to both female and male joints (line 147 of
  `joint_pair_solver.py`: `bounds = [fjp_range, pair.jr_range, mjp_range,
  pair.jr_range]`). → Stays per-pair, NOT per-half.
* `pair.jp_range` — used by `_random_seeds` only; the actual `(fjp_range,
  mjp_range)` bounds are overridden at solve-time using bar lengths.
  → Stays per-pair.
* `pair.female`, `pair.male` (each a `JointHalfDef`) — attributes accessed:
  `block_name`, `asset_filename`, `asset_path()`, `M_block_from_bar`,
  `M_screw_from_block`, `collision_filename`, `collision_path()`.

**Conclusion**: the new normalized `halves[]` + `mates[]` registry
shape rebuilds an identical `JointPairDef` at load time by joining the
halves table to the mate entry's `male_block` / `female_block` keys.
Downstream code is untouched.

Also verified dead code: `config.FJP_RANGE`, `MJP_RANGE`, `FJR_RANGE`,
`MJR_RANGE`, `JJR_RANGE` are defined but never referenced anywhere
(grep returned only the definitions). Migration step will delete them.

### Registry layout

Single file: **`scripts/core/joint_registry.json`** (replaces
`joint_pairs.json`). Three top-level lists — halves, mates, and ground
joints all live together so there's one source of truth and one loader
lock:

```json
{
  "halves": [
    {
      "block_name": "T20_Male",
      "kind": "male",                // "male" | "female"
      "asset_filename": "T20_Male.3dm",
      "collision_filename": "T20_Male_collision.obj",
      "mesh_filename": "",
      "mesh_scale": [1, 1, 1],
      "preferred_robotic_tool_name": "",
      "M_block_from_bar": [[...]],
      "M_screw_from_block": [[...]]
    },
    ...
  ],
  "mates": [
    {
      "name": "T20",
      "female_block": "T20_Female",  // FK joins halves[] by block_name
      "male_block": "T20_Male",
      "contact_distance_mm": 12.5,
      "jp_range": [-500, 500],
      "jr_range": [-3.14159, 3.14159]
    },
    ...
  ],
  "ground_joints": [
    {
      "name": "Ground_T20",
      "block_name": "Ground_T20",
      "asset_filename": "Ground_T20.3dm",
      "collision_filename": "Ground_T20_collision.obj",
      "M_block_from_bar": [[...]],
      "jp_range": [-500, 500],
      "jr_range": [-3.14159, 3.14159]
    }
  ]
}
```

### Loader changes

* `load_joint_pairs(path)` → replaced by `load_joint_registry(path)`
  returning a `JointRegistry` dataclass with three fields:
  ```python
  @dataclass
  class JointRegistry:
      halves: dict[str, JointHalfDef]        # keyed by block_name
      mates:  dict[str, JointPairDef]        # keyed by mate name
      ground_joints: dict[str, GroundJointDef]
  ```
  `JointPairDef` is reconstructed at load time by joining `male_block` /
  `female_block` to the `halves` table. Half-block-name de-dup is
  implicit (each `block_name` lives once in `halves`).
* Add back-compat shims:
  - `load_joint_pairs(path) -> dict[str, JointPairDef]` returns
    `load_joint_registry(path).mates` so existing callers in
    `env_collision`, `joint_placement`, `joint_auto_place` keep working
    without edits.
  - `get_joint_pair(name)` likewise.
  - New: `get_ground_joint(name)`, `list_ground_joint_names()`.

### Migration

One-shot script `scripts/migrate_joint_pairs.py`:

1. Read old `joint_pairs.json`.
2. For each pair, push its `female` and `male` halves into `halves`
   (skip if `block_name` already present, and warn if the existing entry's
   transforms disagree beyond tol).
3. Push the pair's `(name, contact_distance, ranges)` into `mates`.
4. Write `joint_registry.json`. Then delete `joint_pairs.json` and
   `rs_define_joint_pair.py`, remove the toolbar entry, update README.

## Command flows

### `RSDefineJointHalf` (single entry point, three sub-flows)

**Step 0 — prompt the kind first:** show a CommandLineOption
`Kind = Male | Female | Ground`. The remaining picks branch on this
choice. Pre-prompt happens BEFORE any object pick so the user can cancel
cheaply, and the picks are minimized for the Ground branch.

**Common picks (Male / Female / Ground):**

1. **block instance** — the 3D block in its current placed pose.
2. **bar axis line** — the centerline that defines this half's parent bar.
3. **collision meshes** — multi-select; any number of Rhino mesh objects.
4. **prompt** name (default = the Rhino block-definition name; for
   ground, default `Ground_<block_name>`).

**Additional picks (Male / Female only):**

5. **screw axis line**
6. **screw center point**

(Each pick auto-hides previous geometry, matching current
`RSDefineJointPair` UX.)

Computes:

* `bar_frame = canonical_bar_frame_from_line(...)` (existing helper).
* `M_block_from_bar = inv(bar_frame) @ block_world_frame` (mm).
* For Male/Female only:
  `M_screw_from_block = inv(block_world_frame) @ screw_world_frame` (mm).

Exports (all three kinds):

* `asset/<block_name>.3dm` via existing `export_block_definition_to_3dm`
  (overwrite on re-define, preserved current behavior).
* `asset/<block_name>_collision.obj` via Rhino's `_-Export` of the
  selected meshes only (use `rs.Command` with the meshes pre-selected;
  pass OBJ writer options that **preserve per-object grouping** — Rhino's
  default OBJ writer already emits one `o <name>` block per Rhino object,
  which our PyBullet `_add_rigid_body` patch reads as separate sub-meshes
  per the existing lesson).

Persists into the registry:

* Male/Female → `halves[]` (overwrite if `block_name` already exists,
  with a console warning listing any existing mates that reference it so
  the user knows re-defining affects those mates).
* Ground → `ground_joints[]` (overwrite if `name` already exists).

### `RSDefineJointMate`

Picks (placed in their as-mated pose in Rhino):

1. **female block instance** (must already exist as a registered female
   half — looked up by block-definition name).
2. **female bar axis line**.
3. **male block instance** (registered male half).
4. **male bar axis line**.
5. **prompt** mate name.

Computes:

* For each picked instance, compute the world block frame from the block
  instance transform (mm).
* Verify each picked block's name matches a registered half; bail with a
  helpful error if not (suggest running `RSDefineJointHalf` first).
* Compute female and male `screw_world_frame = block_world @ M_screw_from_block`.
* Verify their Z axes are antiparallel within tol (else warn the user the
  blocks are not correctly mated; ask to abort or proceed).
* Auto-derive `contact_distance_mm = signed_z_distance(male_screw_frame.origin,
  female_screw_frame.origin)` projected onto the screw Z.
* Print derived value, prompt `Accept` / `Edit` / `Cancel`.

Persists into `mates[]` (overwrite if `name` already exists).

### Placement (future, NOT in this change)

* `RSJointPlace` for ground joints needs a new code path — pick a bar
  endpoint, snap the joint's local X to the bar axis (sign-flippable),
  expose interactive flip toggle, no mate solver. Out of scope for this
  spec; flag in `todos.md`.

## Files to create / edit

### New
* `scripts/rs_define_joint_half.py` (single entry point for Male / Female / Ground)
* `scripts/rs_define_joint_mate.py`
* `scripts/migrate_joint_pairs.py` (one-shot)
* `scripts/core/joint_registry.py` (rename of `joint_pair.py`; keep a
  back-compat shim `scripts/core/joint_pair.py` that re-exports the
  symbols downstream code currently imports — `JointHalfDef`,
  `JointPairDef`, `load_joint_pairs`, `save_joint_pair`,
  `canonical_bar_frame_from_line`, `fk_half_from_bar_frame`,
  `DEFAULT_ASSET_DIR`, `DEFAULT_REGISTRY_PATH` — to minimize churn).

### Modified
* `scripts/core/joint_pair.py` → split into `joint_registry.py`
  (halves/mates/ground loaders, `GroundJointDef`, back-compat shims).
* `scripts/core/env_collision.py`: switch from `load_joint_pairs` to
  `load_joint_registry`; iterate `registry.halves` directly for
  `_joint_collision_paths` (more efficient than scanning embedded pair
  halves, but result identical). Also include ground-joint collision
  paths so they're visible to the env_collision pass.
* `scripts/core/joint_placement.py`, `scripts/core/joint_auto_place.py`:
  no API change required (they consume the back-compat
  `JointPairDef`). Just verify imports still resolve via the shim.
* `scripts/rs_joint_place.py`: no change required (uses the shim).
* `tests/test_joint_pair_roundtrip.py`: extend to test (a) the new
  normalized loader, (b) `GroundJointDef` roundtrip, (c) that the
  reconstructed `JointPairDef` matches the pre-split fixture exactly.
* `scaffolding_toolbar.rui`: replace the `RSDefineJointPair` button with
  `RSDefineJointHalf` and `RSDefineJointMate`. (No third button — ground
  is a sub-mode of `RSDefineJointHalf`.)
* `README.md`, `docs/rhino_toolbar_entrypoints.md`: rewrite the joint
  workflow section.
* `todos.md`: add follow-up "RSJointPlace ground-joint path".
* `scripts/core/config.py`: delete the unused `FJP_RANGE`, `MJP_RANGE`,
  `FJR_RANGE`, `MJR_RANGE`, `JJR_RANGE` constants (verified dead).

### Deleted (after migration)
* `scripts/rs_define_joint_pair.py`
* `scripts/core/joint_pairs.json`

## Implementation order

1. Branch the dataclasses + registry I/O (no behavior change yet, both old
   and new loaders coexist temporarily).
2. Write the migration script + run it to produce `joint_registry.json`.
3. Update downstream consumers (`env_collision`, `joint_placement`, tests).
4. Write the three new RS commands.
5. Update the `.rui` toolbar + docs.
6. Delete old files; remove back-compat shims.
7. Add lesson note to `tasks/cc_lessons.md` (key insight: collision OBJs
   from Rhino keep one `o <name>` per source object — never re-merge in
   our pipeline).

## Open questions deferred to implementation time

* Exact Rhino `_-Export` arg string for OBJ writer that emits per-object
  groups — verify by inspecting the OBJ produced for an existing pair.
* How to invoke a "select-then-export" without the user seeing the export
  dialog. Likely: `rs.Command("_-Export OBJ ... _Enter", echo=False)`
  with a pre-saved OBJ options string.
* Whether `RSDefineJointMate` should also accept Y-axis sign of the screw
  for orientation disambiguation (currently only Z-antiparallel checked).
