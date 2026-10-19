# Spec: the MoCap joint role (`T20_MoCap`)

> **Status: built (2026-10-08), with changes.** This is the plan as approved on 2026-09-25,
> kept for its reasoning. The last section, [What was built differently](#what-was-built-differently),
> says where the code departs from it. Line links point at the code as it was then.

## Context

The `hs/mocap-experiment` branch registers the real structure to the model with OptiTrack. Today
`RSReadMoCapBar` bakes loose marker points, someone draws a line through each bar's marker-pair
centres by hand, and `RSAlignModelThreeBars` fits three of those lines to three model bars.

`T20_MoCap` replaces that improvisation. The three parts in `docs/Demo_T20_Marker.mp4` are screws, a
female joint half, and a marker-mount plate bolted to its back. Because it is a real joint half, the
prefab process places it at an accurate, known position on a known bar — so every marker sphere's
world position becomes predictable from the model.

Named `T20_MoCap`, not `T20_Marker`: the repo already uses "MoCap" as its own word (`MoCap_Retrieval`
layer, `RSReadMoCapBar`), while "marker" already means four unrelated things here.

## Decisions

| | |
|---|---|
| Role model | First-class role — own `kind`, own layer, `_mocap` suffix, `joint_<jid>_mocap` key. |
| Placement | Both **paired** (mocap + male across two bars) and **standalone** (one bar, no mate). |
| Robot | Hand-fitted. Never tool-bearing. But a *paired* mocap half still receives a male, so the collision whitelist must treat it like a female. |
| RSJointPlace modes | `JointPairAndTool \| Tool \| Ground \| MoCap`. The mode prompt **is** the bearing-joint question. Only `JointPairAndTool` goes on to ask the receiving joint. |
| RSJointEdit | New `ReplaceJoint` mode swaps a placed receiver female ↔ mocap, keeping the joint number. |
| Prefab export | Always exported, tagged `subtype: "MoCap"`. |
| Marker points | Labelled — a name→position map, block-local, mm. |
| Module layout | `ground_placement.py` renamed `single_sided_placement.py`, holding both single-sided roles. |
| RSTempPlace | Deleted. |

---

## Background: the five places a joint records its role

Take bars `B40` and `B53` joined with the `T20` mate, and follow the female half.

| # | What | Lives in | Set from | Example |
|---|---|---|---|---|
| 1 | `kind` | `joint_pairs.json`, on disk | you, in RSDefineJointHalf | `"kind": "female"` |
| 2 | Rhino **layer** | a Rhino object property | hardcoded from the pair slot, [line 351](scripts/core/joint_placement.py#L351) | `MANAGED Scaffolding::Joint Female Instances` |
| 3 | `joint_subtype` | Rhino **user text** | `block_name.partition("_")`, [line 338](scripts/core/joint_placement.py#L338) | `Female` |
| 4 | object **name** | a *different* Rhino property | hardcoded literal, [line 357](scripts/core/joint_placement.py#L357) | `J40-53_female` |
| 5 | PyBullet body key | nowhere — rebuilt at run time | `joint_subtype`.lower(), [env_collision:405](scripts/core/env_collision.py#L405) | `joint_J40-53_female` |

The layer (#2) is the **authority** — it survives copy/paste, whereas user text is duplicated
verbatim onto copies.

**`kind` is written once and then never read again at placement time.** Everything downstream flows
from which *slot* of the pair the half occupies, plus the block's file name. Consequences:

- Naming the block `T20_MoCap` makes `partition("_")` yield `MoCap`, so **#3 and #5 come free.**
- **#2 and #4 need code**, because both are hardcoded rather than derived (Commit 4).
- **#1 needs code** only to stop `VALID_HALF_KINDS` rejecting the value (Commit 2).

Put a comment on line 338 recording that mocap depends on that `partition("_")`.

## Background: bearing vs receiving

| | Kinds | Meaning | Constants |
|---|---|---|---|
| **Receiving** joint | female, mocap | a male screws **into** it | `RECEIVER_HALF_KINDS` / `RECEIVER_JOINT_LAYERS` |
| **Bearing** joint | male, ground | it carries the **robot tool** | `BEARING_HALF_KINDS` / `TOOL_BEARING_JOINT_LAYERS` |

MoCap is in the first and deliberately absent from the second. That absence *is* "by hand, not by
robot": [rs_ik_keyframe](scripts/rs_ik_keyframe.py#L301) requires **exactly two** tool-bearing halves
on the bar being assembled, so a mocap half joining that set would break every bar carrying one.

---

## Commit 1 — `[refactor] name the joint-role layer sets in core/config`

**Goal:** the three joint-layer names are hand-copied in ~20 files, so adding a fourth means hunting
them all down and every miss is a silent bug. Name them once instead.

- Add to [config.py](scripts/core/config.py) after line 407: `JOINT_INSTANCE_LAYERS` (all of them),
  `LAYER_TO_JOINT_ROLE` + `JOINT_ROLE_TO_LAYER` (a two-way lookup between layer name and role word),
  `TOOL_BEARING_JOINT_LAYERS` (male + ground), `RECEIVER_JOINT_LAYERS` (female). **Still three
  roles** — mocap arrives in Commit 3.
- Point the existing sites at them, in three groups:
  - *all joint layers* → `JOINT_INSTANCE_LAYERS`: `env_collision`, `rhino_bar_registry` (4 sites),
    `rhino_joint_refresh` (note the constant is public `JOINT_LAYERS`, not `_JOINT_LAYERS`),
    `joint_relink`, `robot_cell`, `ik_viz`, `highlight_env`, `rs_remove_bar`,
    `rs_import_scaffold_json`, `rs_export_prefab`, `rs_select_joint`, `rs_joint_edit`.
  - *male + ground only, contents unchanged* → `TOOL_BEARING_JOINT_LAYERS`: `ik_collision_setup`,
    `rs_ik_keyframe`, `rhino_tool_place` (4 sites), `rhino_walkable_ground`, `rs_joint_place`,
    `rs_select_bar`, `rhino_bar_registry:790`.
  - *leave alone* — single-layer scans meaning exactly one role: `bar_action:346` and `:380` (two
    separate dicts feeding arm classification; merging them breaks it),
    `rhino_tool_place:577`/`:593`, `rs_reorder_bar_id:120`, and the module aliases in
    `joint_placement` / `ground_placement`.
- New `tests/test_config_joint_roles.py`: two-way lookup round-trips, all layers in
  `MANAGED_LAYERS`, exact-equality guard on `TOOL_BEARING_JOINT_LAYERS`.

**Zero behaviour change.** The only review question is "did any list's contents change?" — no.

## Commit 2 — `[feat] add the mocap half kind to the joint registry`

**Goal:** make the library file able to *describe* a MoCap part. Today
`JointHalfDef(kind="mocap")` raises `ValueError` at
[joint_pair.py:86](scripts/core/joint_pair.py#L86), which blocks everything else.

All in [joint_pair.py](scripts/core/joint_pair.py). Pure Python, no Rhino, no registry data yet.

- `VALID_HALF_KINDS` gains `"mocap"`.
- Add `MATED_HALF_KINDS` (not ground — i.e. "has a screw bore"), `RECEIVER_HALF_KINDS`,
  `BEARING_HALF_KINDS`. The last must always match `config.TOOL_BEARING_JOINT_LAYERS`; the test
  asserts it.
- Add `pair.receiver` / `receiver_kind` / `is_mocap_mate` — reads correctly whichever block sits in
  the `female` slot. The **stored** field keeps its name, so no JSON and no call site breaks.
- Add `mocap_alternative(pair)` / `female_alternative(pair)`: find the counterpart mate by matching
  the same male block. **Commit 10's `ReplaceJoint` cannot work without these.** On ambiguity prefer
  `pair.name + "MoCap"` and print a NOTE.
- Add `marker_points_mm` — a labelled `dict[str, (x,y,z)]` in mm, block-local, following the
  `bar_cradle` precedent (new field + `data.get(..., {})` default). Added **now** rather than in
  Commit 11 so the saved file format is final from here on.
- **The `mates` table is reused unchanged**: a mocap mate is an ordinary row with
  `female_block_name: "T20_MoCap"`. No schema migration; every existing file keeps loading.
- New `tests/test_mocap_role.py`.

Leave `migrate_joint_pairs.py` alone — it guards a legacy format that predates this. One comment.

**Not yet possible after this:** placing one. There is no layer (Commit 3) and no way to define one
(Commit 6).

## Commit 3 — `[feat] add the Joint MoCap Instances layer and role`

- `LAYER_JOINT_MOCAP_INSTANCES = "MANAGED Scaffolding::Joint MoCap Instances"`.
- Into `MANAGED_LAYERS`, `JOINT_INSTANCE_LAYERS`, `LAYER_TO_JOINT_ROLE`, `RECEIVER_JOINT_LAYERS` —
  and **not** `TOOL_BEARING_JOINT_LAYERS`.
- A layer colour in `rhino_bar_registry._JOINT_LAYER_COLORS` (suggest `(90, 170, 220)`).
- Collapse the three `_enforce_joint_layer` calls (1668-1670) into a loop, or the layer is never
  created.
- Update `test_config_joint_roles.py` to four roles, keeping the `TOOL_BEARING` guard.

Every "all joint layers" site from Commit 1 picks it up for free.

## Commit 4 — `[feat] make paired placement receiver-role-aware`

**Goal:** stop places #2 and #4 being hardcoded, so a paired mocap half lands on the right layer with
the right name.

[joint_placement.py](scripts/core/joint_placement.py):

```python
# today                                          # after
layer_name=FEMALE_INSTANCES_LAYER                receiver_role  = pair.receiver_kind
rs.ObjectName(fid, f"{joint_id}_female")         receiver_layer = config.JOINT_ROLE_TO_LAYER[receiver_role]
                                                 layer_name=receiver_layer
                                                 rs.ObjectName(rid, f"{joint_id}_{receiver_role}")
```

One function, two outcomes, nothing branching on the role:

```
mate "T20"       -> role female  layer …Joint Female Instances  name J40-53_female  key joint_J40-53_female
mate "T20MoCap"  -> role mocap   layer …Joint MoCap Instances   name J40-53_mocap   key joint_J40-53_mocap
```

- Add `ROLE_MOCAP`, `RECEIVER_PREVIEW_ROLES`, `MOCAP_INSTANCES_LAYER`. Preview blocks carry the
  honest role.
- `write_joint_user_text` gains one key, `receiver_role`, written on **both** halves — that is how a
  male answers "is my mate a mocap half?" without loading the registry. Do not rename
  `female_parent_bar` / `male_parent_bar`.
- `recover_side` guard (218-219): normalize the input instead of widening the check — see
  *Reference A*.
- `variant_index` unchanged; only the comment at line 81 (bit 0 is the **receiver** side).

`rs_joint_place` / `rs_joint_edit`: receiver wording, and `_remove_placed_joint` learns the mocap
layer and `_mocap` suffix — **a miss there does not fail loudly**, it leaves two blocks on one
`joint_id`.

Three places that break silently and must be fixed here — all detailed in *Reference B*:
`rhino_bar_registry:1262`, `rs_reorder_bar_id:329` and `:151`, and the role maps in `joint_relink` /
`rhino_joint_refresh`.

## Commit 5 — `[feat] whitelist the mocap mate contact in the assembly touch policy`

**Goal:** a male screwing into a mocap half must not be reported as a collision. **This is the one
part of the change that fails loudly** — without it, every bar carrying a MoCap joint fails IK.

In [bar_action.py](scripts/core/bar_action.py), three fixes — see *Reference C* for what each
whitelist is for:

- **line 844** `endswith("_female")` → `endswith(("_female", "_mocap"))`.
- **lines 772, 778, 790, 807** hard-code `f"joint_{jid}_female"` → one `_receiver_key(jid, env_geom)`
  helper trying both suffixes; also used at the cradle lookup (1434).
- **lines 641-653** — a `_mocap` tag already lands in the right `else` branch by accident; make it an
  explicit `elif`.

New `tests/test_bar_action_mocap.py`, cloning `test_bar_action_ground.py`'s harness.

## Commit 6 — `[feat] add a MoCap kind to the two define commands`

- **RSDefineJointHalf**: add `"MoCap"` to the kind prompt
  (`["Male", "Female", "Ground", "MoCap"]`). Five lines then read `if kind in ("male","female")`,
  meaning *"has a screw bore, so ask for the screw picks"* — a mocap half has one, so they become
  `kind in MATED_HALF_KINDS`. Defining one is then the identical sequence: block → bar axis → screw
  axis → screw centre → collision meshes → name.
- **RSDefineJointMate**: "Pick FEMALE block instance" → "Pick RECEIVER block (female or mocap)", and
  accept either kind. That check is already only a warning, so this just stops a spurious one.
- **Fix the stale collision-mesh cache** — see *Reference D*. Three lines, with an existing
  precedent in the same file.

## Commit 7 — `[feat] place a standalone MoCap joint on one bar`

**Goal:** put a MoCap half on a single bar with **no male partner** — the same shape RSGroundPlace
already uses for a ground joint (pick a bar, pick a point on it, spin it, accept).

- **Merge, don't copy.** `git mv scripts/core/ground_placement.py
  scripts/core/single_sided_placement.py`. Ground and standalone-mocap placement share the flip
  matrix, id-plus-index minting, the preview insert, the place-block user-text write and the remove.
  "Single-sided" is the repo's own phrase for this. `git mv` keeps history; update the importers.
- Add the mocap flow beside the ground one. One real difference: a mocap half comes from
  `registry.halves` and **has** a screw frame (so its FK reuses `fk_half_from_bar_frame`), while a
  ground def comes from `ground_joints` and has none.
- **RSJointPlace gains the `MoCap` mode**: pick bar → pick point on bar → pick the mocap half
  (`rs.ListBox` filtered to `kind == "mocap"`, auto-selected when there is one) → preview loop
  (Accept / Flip / Rotate) → bake. **No tool** — print that explicitly, since every other mode ends
  with one. A mocap half has no floor constraint, so `jr` starts at 0 and `Rotate` aims it.
- **Its own id format**: `M7-T20MoCap-0`, mirroring ground's `G4-floor-0`. `M` is unused as a
  joint-id prefix.
- **Tell the broken-joint check it is fine.** `find_broken_links` flags "a receiver with no male", so
  a standalone would be reported every time RSUpdatePreview runs. Exempt it the way ground already
  is, and tell a standalone from a half-built pair by the **absent `male_parent_bar`** user text.
  Same exemption in `report_unmated_joints`.
- **RSReorderBarID**: renaming `B7`→`B12` must rename the joint too, or an `M7-…` id is left on bar
  `B12`. Mirror the branch ground already has.
- `RSJointEdit`'s Flip loop routes a standalone mocap half to a copy of `_flip_ground_block`, with no
  tool to restore.
- Extract `rs_ground_place._pick_point_on_bar` into `core/rhino_bar_pick.pick_point_on_bar` so both
  branches share it.
- New `tests/test_mocap_placement.py`: the id format, the canonical-key split invariant
  (`f"joint_{jid}_mocap"` must `rsplit("_", 1)` back to `[jid, "mocap"]`), and the flip property.

## Commit 8 — `[chore] remove RSTempPlace`

Delete `scripts/rs_temp_place.py`, its two buttons from `scaffolding_toolbar.rui` and `.rui.cmds`,
and its section from `docs/rhino_toolbar_entrypoints.md`. Nothing imports it. Its own commit so it
can be reverted independently.

## Commit 9 — `[feat] register the T20_MoCap half and the T20MoCap mate`

`asset/T20_MoCap.3dm` + `.obj` plus the `halves` and `mates` entries — **produced by running
Commits 6-7 inside Rhino**. A hand-typed `M_block_from_bar` will fail the round-trip grid, so this
cannot be written blind and must come after 6-7.

`tests/test_joint_pair_roundtrip.py` parametrizes its 12-case grid over every registered mate, so
this buys twelve free correctness tests.

Do **not** set `bar_cradle` — `tests/test_bar_action_subfloor.py:190-194` asserts the cradle list is
exactly the two subfloor females, and that is the only way this commit can break an existing test.

## Commit 10 — `[feat] ask bearing-then-receiving in RSJointPlace; add ReplaceJoint`

**RSJointPlace** — `_ask_place_mode` becomes four-way. **The mode prompt is the bearing-joint
question**, asked first; Ground and MoCap are single-sided, so neither asks about a receiver:

```
JointPairAndTool (Enter) | Tool | Ground | MoCap

  JointPairAndTool -> bearing is Male, so ask: Receiving joint? [ Female | MoCap ]
                      -> Pair list FILTERED to matching mates -> 2 bars -> solve -> bake -> tool
  Tool             -> today's ToolOnly, renamed
  Ground           -> single bar, no receiving question (reuses single_sided_placement)
  MoCap            -> single bar, standalone, no receiving question, no tool (Commit 7)
```

This replaces the MoCap *toggle* I first proposed, and is better: filtering the `Pair` list by the
receiving answer **halves** it instead of doubling it, and asking explicitly each run removes the
sticky-default trap (`scaffolding.last_joint_pair` could otherwise leave every later bar quietly
getting a mocap half). Implementation: `pick_bar_with_pair_option` gains a `receiver_kind` argument
and filters on it. RSGroundPlace keeps its own button and behaves identically.

`RSBarSnap` / `RSBarBrace` / `RSBarSubfloor` are always male-bearing, so they get the receiving
question only, as an `AddOption("Receiver")` on their existing pickers.

**RSJointEdit `ReplaceJoint`** — a third mode beside FlipJoint / MoveJoint:

```
before   J40-53_female   block T20_Female   collision T20_Female.obj
after    J40-53_mocap    block T20_MoCap    collision T20_MoCap.obj
          ^^^^^^ joint number unchanged; only the role suffix differs
```

Pick a placed receiver or its male → resolve the counterpart with `mocap_alternative` /
`female_alternative` → call the existing `_replace_joint_pair`, which already does remember-tool →
delete → re-solve → re-place-keeping-id → restore-tool. The number is preserved by construction,
since `joint_id` is recomputed from the two bar ids and those do not change.

**Both instance and collision mesh change.** The instance because a different block definition is
inserted; the mesh automatically, because `env_collision` looks the `.obj` up by
`rs.BlockInstanceName`. **But** that lookup is cached — so `ReplaceJoint` must call
`clear_joint_obj_path_cache()` from Commit 6 (*Reference D*), or the swapped joint keeps its old mesh
for the rest of the session.

Standalone mocap halves are out of scope here — with no male there is no counterpart pair. Delete and
re-place.

## Commit 11 — `[feat] capture marker sphere centres on a mocap half`

New `core/marker_points.py`, as a seam:

- `capture_marker_points(...) -> dict` — Rhino-side; pick each sphere centre, prompt for its Motive
  label, convert block-local with the math `compute_M_screw_from_block` already uses. Returns `{}`
  when skipped, so the caller never branches.
- `world_marker_points_mm(...) -> dict` — pure numpy read side, so the future "predict mocap markers"
  command has an API and the round-trip is testable without Rhino.
- The pick step goes in `rs_define_joint_half.main` after the collision-mesh pick, guarded by
  `if kind == "mocap"`.

Fully separable: ship `capture_marker_points` returning `{}` as a documented stub if it slips. The
registry field from Commit 2 is already final.

## Commit 12 — `[docs] document the mocap joint role`

- `rhino_toolbar_entrypoints.md` — RSJointPlace (4 modes), RSJointEdit (3 modes),
  RSDefineJointHalf (4 kinds), RSDefineJointMate wording, and RSGroundPlace's new module home.
- `coordinate_conventions.md` — a **MoCap joint** subsection (axes identical to Female, plus the
  plate and `marker_points_mm`); add `receiver_role` to the user-text table. Lines 86-87 name the
  layers `FemaleJointPlacedInstances` / `MaleJointPlacedInstances`, **stale for some time** — fix to
  the four real names while here.
- `ik_keyframe_scene_reconstruction.md` — the M0-M4 `touch_bodies` matrix gains a `mocap` column
  identical to `female`, plus a line distinguishing a paired mocap half from a standalone.
- `Su_note.md` — the five-places table and the fact that `kind` is written once and never read again
  at placement time; *(real Python)* `str.endswith(tuple)`, `str.partition`, and why
  `rsplit("_", 1)` stays safe when an id contains an underscore. Follow the file's
  convention-vs-real-Python labelling.
- `tasks/spec_joint_half_mate_ground_split.md` — **append** an extension section; do not edit the
  COMPLETED body. `README.md` lines 10-12: "a female + male" → "a receiver (female or mocap) + male".
- **No new toolbar button** — MoCap is a sub-mode, the same call the ground spec made.

## Commit 13 — `[chore] sync scaffolding_toolbar.rui.cmds`

Unrelated housekeeping: the `.cmds` mirror lags the `.rui` (missing `RSSelectJoint`). Separate so it
does not muddy the diff.

---

## Verification

**Per commit, no Rhino:** `pytest tests/`. Commits 1-3, 5 and 7 each add tests that fail before and
pass after. Commit 9 is validated by the existing 12-case round-trip grid extending over the new mate
automatically.

**In Rhino, after Commits 1-9:**

1. `RSDefineJointHalf` → `MoCap` → confirm `asset/T20_MoCap.3dm` + `.obj` appear and the half lands
   in `joint_pairs.json` with `"kind": "mocap"`.
2. `RSDefineJointMate` → receiver `T20_MoCap`, male `T20_Male`, name `T20MoCap`.
3. `RSJointPlace` → `JointPairAndTool` → receiving **MoCap** → two bars. The receiver bakes on
   **Joint MoCap Instances**, named `J<le>-<ln>_mocap`, `receiver_role: mocap`. The male gets its
   tool; the mocap half does not. The `Pair` list showed only mocap mates.
4. `RSJointPlace` → `Ground` → the receiving question is **never asked**. Run RSGroundPlace itself
   too — it must behave exactly as before the module rename.
5. `RSJointPlace` → `MoCap` on one bar → `M<n>-T20MoCap-0_mocap`, no tool, and `RSUpdatePreview` does
   **not** report it broken. Then delete the male from step 3's pair and confirm that one **is**
   reported broken.
6. `RSJointEdit` → `ReplaceJoint` on step 3's joint → same joint number, block now `T20_Female`, tool
   survives. Swap back, then run `RSIKKeyframe` **in the same session** to confirm the collision mesh
   followed the swap — without the cache fix it silently keeps the old mesh.
7. `RSSelectJoint` with `J<le>-<ln>_mocap` and with `M<n>-T20MoCap-0`.
8. `RSExportPrefab` → the mocap half appears with `subtype: "MoCap"`.
9. `RSIKKeyframe` on a bar whose receiver is a mocap half → the M2 mate reports **no** false
   collision (the Commit 5 fix).
10. `RSUpdatePreview` twice → identical report, nothing moved.

---

## Reference

### A. Why `recover_side` is normalized, not widened

`recover_side` names **which side to flip** when a solved variant's interface error is too large.
Today only `"female"` / `"male"` are allowed, and four callers pass the literal `"female"`. New code
wants `pair.receiver_kind`, which is `"mocap"`. Translate once at the top rather than scattering
`in ("female","mocap")` through the body:

```python
_RECOVER_SIDES = {          # what the caller said  ->  what the function uses
    "female":   "female",   # the 4 existing callers, unchanged
    "mocap":    "female",   # a mocap half IS the receiving side -> same branch
    "receiver": "female",   # a clearer name new code can use
    "male":     "male",
}
```

Everything below the guard keeps comparing `"female"` / `"male"` and needs no edit.

### B. Three places that break silently in Commit 4

**`rhino_bar_registry.py:1262`** — `is_tool_side = layer != LAYER_JOINT_FEMALE_INSTANCES`. Today
"not female" *means* "tool-bearing", so a mocap half is recorded as its joint's tool owner and drags
tool visibility onto the wrong bar. → `layer in config.TOOL_BEARING_JOINT_LAYERS`. **The sharpest bug
in the change.**

**`rs_reorder_bar_id.py:329`** — `suffix = "female" if subtype.lower()=="female" else "male"` renames
a mocap half to `J7-9_male`, after which `report_joint_usertext_issues` flags it forever and
`find_male_block_for_joint` matches the wrong block. Derive the suffix **from the layer**. Same at
line 151 (`== "Female"`, case-sensitive today).

**`joint_relink.py`** — worth knowing what this module is: it is **copy/paste repair**, not an id
renumberer, and it never creates anything. Bars, joints and tools are tied by *strings only*
(`parent_bar_id = "B40"`), with no GUIDs, so nothing detects a copy/paste
([lines 5-16](scripts/core/joint_relink.py#L5)). Copy a group and the curves get fresh ids while the
joints still point at the originals; string remapping cannot fix it because both copies claim the
same name. The one surviving signal is geometry — the copy moved with its bar — so the module
measures which bar each block sits on and rewrites the strings.

Its geometry rule needs no change (a mocap half's `M_block_from_bar` has the same near-zero radial
offset as a female's). What it needs is role vocabulary: the role loop (453-456) gains the mocap
layer, and `_consistency_warnings` (407-427) becomes "exactly 1 **receiver** + 1 male" counting
`("female","mocap")` into one bucket, or every mocap pair warns. Sort order at 486 gets `"mocap": 0`.
Tool anchors at 465-470 stay `("male","ground")` — verify, do not change.

Same class of fix in `rhino_joint_refresh.py:226-228` (role-from-layer, or a paired mocap half
classifies as *male* and `report_unmated_joints` skips it).

### C. What the three touch-policy whitelists are for

The simulation flags collisions, but some contacts are *supposed* to happen. A per-movement whitelist
says "these two bodies may touch", and it is written in terms of `_female`.

**Line 844 — a carried half touching its own bar.** While the robot carries a bar, the female bolted
to it touches it; fine, whitelist it for M1/M2. `joint_J40-53_mocap` does not end in `_female`, so it
is skipped and the contact reads as a real collision.

**Lines 772-807 — the mate contact at M2.** M2 is the moment the male seats in. Allow male↔partner,
and also male↔the partner's *bar*, because on the coarse meshes the screw tip lands ~2.5 mm from it.
The file's own example: inserting `bar_B9` with `joint_J35-9_male`, whose mate
`joint_J35-9_female` belongs to `bar_B35`, so the male is whitelisted against `bar_B35`. With a mocap
receiver the key is `joint_J35-9_mocap`, the `in env_geom` test is False, and **no whitelist is
applied at all**.

```python
def _receiver_key(jid, env_geom):
    """Canonical key of the half the male seats into (female or mocap)."""
    for sfx in ("_female", "_mocap"):
        key = f"{CANONICAL_JOINT_PREFIX}{jid}{sfx}"
        if key in env_geom:
            return key
    return f"{CANONICAL_JOINT_PREFIX}{jid}_female"
```

**Lines 641-653 — which arm carries this half.** `else: arm = bar_arm_side` already gives a mocap
half the right answer by accident; make it an explicit `elif`.

### D. The stale collision-mesh cache

PyBullet needs a simplified collision shape per joint block — the `.obj` files in `asset/`. To find
one, `env_collision._joint_obj_path_map()` builds `block_name → obj path` from `joint_pairs.json`.
Reading the JSON is slow, so it is built once and **stored in `sc.sticky`** (Rhino's per-session
memory) at [line 120](scripts/core/env_collision.py#L120). **Nothing in the repo ever throws it
away.**

```
1. open Rhino, run anything needing collision  -> map built, 5 blocks
2. RSDefineJointHalf, create T20_MoCap         -> written to joint_pairs.json on disk
3. RSIKKeyframe, same session                  -> still the OLD 5-entry map; T20_MoCap.obj
                                                  is never found, so the half silently falls
                                                  back to the slow render-mesh path
4. restart Rhino                               -> works
```

"Works after a restart" is the signature. This already affects *any* newly defined half today; MoCap
just makes it likely to be hit. Add `clear_joint_obj_path_cache()` to `env_collision` and call it
where [rs_define_joint_half.py:479](scripts/rs_define_joint_half.py#L479) already calls
`clear_tool_attach_cache()` for the same reason. `ReplaceJoint` calls it too.

---

## Open item, non-blocking

**Is `T20_MoCap`'s `M_screw_from_block` identical to `T20_Female`'s?** Physically it is a female half
with a plate on the back, so the bore should be unchanged. The plan uses delete-and-re-solve in
`ReplaceJoint`, which is correct either way. If the two turn out to be geometrically interchangeable,
`ReplaceJoint` could become a pure block substitution preserving the exact world transform — nicer
UX, worth revisiting once Commit 9 measures the real geometry.

---

---

## What was built differently

Checked against the code on 2026-10-09. The commit subjects are on `hs/mocap-experiment`.

- **One naming module instead of role sets in `config`.** Every joint block is named
  `<Type>_<Subtype>` (Female / Male / Ground / MoCap), and every other name -- layer, joint id,
  object name, collision key, tool id, user-text keys -- is derived from it in
  `scripts/core/joint_name_conventions.py`. A half's kind is read from its block name, so the
  `kind` field of Commit 2 is checked on load, not chosen. The role sets of Commit 1
  (`TOOL_BEARING`, `RECEIVER`, `PAIRED`, `SINGLE_SIDED`) live there too.
- **No `T20MoCap` mate.** A MoCap half is a *receiver variant* of its Type's mate:
  `joint_pair.with_receiver` / `swapped_receiver` put `T20_MoCap` in mate `T20`'s receiving slot,
  so the mates table gains no row and `mocap_alternative` / `female_alternative` were not needed.
  The mates were renamed by Type -- `T20`, `T20Deck12`, `T20SubLeft`, `T20SubRight` -- and
  `T20Ground` became `T20_Ground`; `core/joint_name_migration.py` converts older documents.
- **RSJointPlace has three modes, not four**: `JointPairAndTool | ToolOnly | JointOnly`.
  JointOnly places a Ground *or* standalone MoCap joint: choose Ground or MoCap, then the bar,
  then the point; Ground previews `Accept | Flip`, MoCap `Accept | Rotate`, and no tool is placed.
  JointPairAndTool asks Female or MoCap only when the Type has both. **RSGroundPlace was
  removed** (the plan kept it), since JointOnly does the same placement.
- **Standalone ids are `M<bar>-<Type>-<i>`** (`M7-T20-0`), mirroring Ground's `G4-T20-0`, not
  `M7-T20MoCap-0`. `core/single_sided_placement.py` holds both, as planned.
- **No `receiver_role` user text.** The block's layer says what it is, and is the authority
  everywhere (copies duplicate user text verbatim, layers they do not). `recover_side` accepts
  `"receiver"`, as Reference A proposed.
- **Markers go further than planned.** RSDefineJointHalf records them by picking the sphere
  objects and labelling each (M1, M2, ...); `core/marker_points.py` holds the frame maths.
  RSExportPrefab writes `"mocap": {"paired": ..., "markers_mm": {...}}` per MoCap joint, in the
  bar frame, rather than only tagging the subtype. `T20_MoCap`'s two markers are recorded.
- **RSExportPrefab writes Ground joints as `T20` / `Ground`**, like every other joint.
- **Not built: the `Receiver` option for RSBarSnap / RSBarBrace / RSBarSubfloor.** They always
  place a Female; swap a joint afterwards with RSJointEdit › ReplaceJoint.
- **Tests** landed as `test_joint_name_conventions.py`, `test_mocap_role.py`,
  `test_single_sided_placement.py`, `test_marker_points.py` and `test_bar_action_mocap.py`
  (not `test_config_joint_roles.py` / `test_mocap_placement.py`).
