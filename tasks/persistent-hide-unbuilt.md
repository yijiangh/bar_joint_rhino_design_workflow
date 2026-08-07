# Persistent HideUnbuilt — a document-level "build stage" filter

> Supersedes `sequential-inventing-kazoo.md` (the original design) and
> `we-are-at-the-immutable-rocket.md` (the execution plan). Both are deleted; where they
> disagreed, see **Decisions changed during implementation** below.

## Context

`HideUnbuilt` in [rs_sequence_edit.py](scripts/rs_sequence_edit.py) hides every bar, joint and
tool later than the active assembly step — the fastest way to check what the scaffold looks like
at a given build stage. But the state used to be only a session flag
(`_SequenceSession.show_unbuilt`): pressing Esc ran `reset_sequence_colors()`, which
unconditionally `rs.ShowObject`s everything again. The same happened leaving RSShowBarActionPlan
and RSIKKeyframe, and any refresh command (RSUpdatePreview, RSSwapRoboticTool) that re-creates
tubes/joints/tools brought them back visible, because a newly created Rhino object is always
visible.

Goal: `HideUnbuilt` becomes a **latch stored in the document**. Once set, the unbuilt parts stay
hidden through command exits, refresh passes, and save/reload, until `ShowUnbuilt`.

## How it works, in plain terms

One line of text saved inside the `.3dm`:

```
scaffolding.build_stage = "B7|7"       # bar id | step number.  Missing = filter off.
```

`apply_build_stage_visibility()` reads that line and hides everything later than step 7. It is
called at the end of every place that could have made things visible again. That is the whole
feature — the rest is *where* to call it, and what to do when the stage bar is renamed or deleted.

### Three glossary points

- **Where the state lives.** `sc` is Rhino's `scriptcontext` module (nothing to do with the
  Python language). `sc.sticky` is a dict that survives between script runs but dies when Rhino
  closes — **not used here**. `sc.doc.Strings` (document user text) is saved *inside the .3dm*,
  so it survives Rhino restarts and travels with the file. That is what we use, and it matches
  per-object hidden state, which is also stored in the .3dm.
- **Why `"B7|7"` and not just `"B7"`.** Bar ids are not permanent — RSReorderBarID renames them,
  so after a renumber the bar you staged at might be called B4 while a *different* bar is now
  called B7. Keeping the step number alongside gives a safety net: use bar `B7` if it still
  exists; if it was deleted, fall back to the nearest earlier step so the view barely moves
  instead of the whole model reappearing; if neither resolves, switch the filter off, show
  everything, and print why.
- **Which bar does a robotic tool belong to.** A joint has two block halves that share one
  `joint_id` ([joint_placement.py:336](scripts/core/joint_placement.py#L336)) but store
  *different* `parent_bar_id` values — the female half stores the LE bar, the male half the LN
  bar. A tool block stores only `joint_id`, no bar at all. So scanning all three joint layers
  gives two conflicting answers per tool and whichever is read last wins. The repo already
  settled this in `get_active_tool_oids`: a tool belongs to the bar owning the **male** (or
  ground) half — physically, the bar the gripper is holding. The new code scans male + ground
  only, exactly like that function.

## Decisions confirmed with the user

1. **Visibility only.** Sequence colours (green/blue/grey) still reset on exit; only the hidden
   state persists.
2. **No new toolbar command.** Cleared from `RSSequenceEdit > ShowUnbuilt`, which already reads
   `ShowUnbuilt` on entry because the latch is remembered. No `.rui` edit, no alias
   re-registration.
3. **Strict reach.** Every command running the shared `repair_on_entry()` re-applies the filter
   at startup. An unbuilt bar is therefore unpickable in RSJointPlace/RSBarSnap until the filter
   is cleared — accepted trade-off for "nothing ever leaks back".
4. **Tools follow their bar.** The persistent filter hides a tool only when its owning bar is
   unbuilt; tools on built bars stay visible.
5. **Diagnostics see everything; marks only touch what's visible.** RSUpdatePreview's
   ShowColorsPreview still *scans and reports* hidden objects, but only *colours, dots and
   selects* visible ones. The filter is not suspended.

---

## Decisions changed during implementation

Recorded so nobody re-does work that was deliberately undone.

### 1. No per-tool creation chokepoint

- **Originally** (kazoo step 9): add a hide-on-create hook to `place_tool_at_block_instance`
  ([rhino_tool_place.py:246](scripts/core/rhino_tool_place.py#L246)), "one map lookup, covers
  `restore_missing_tools_at_joints`, `enforce_bar_tool_sides`, `resync_tools_to_joints` and
  `replace_all_tool_instances` in a single place."
- **We did not.** Verified that `rs_bar_snap.py` never imports `rhino_tool_place` at all, and
  that all four of those batch routines are reached only from RSUpdatePreview
  ([rs_update_preview.py:108](scripts/rs_update_preview.py#L108),
  [:116](scripts/rs_update_preview.py#L116), [:123](scripts/rs_update_preview.py#L123)) and
  RSSwapRoboticTool ([rs_swap_robotic_tool.py:205](scripts/rs_swap_robotic_tool.py#L205)) —
  both of which end with an apply under steps 8 and 9. Every other tool placement lands on a
  just-picked, therefore visible, bar. A per-tool hook could not fire on anything those two do
  not already cover.
- **Why it matters:** a helper (`hide_tool_if_beyond_build_stage`) plus a `bar_map` parameter
  and a docstring paragraph were written in `3c46b98`, never wired up, and reverted. The
  tube chokepoint (step 10) is genuinely needed and stays — tubes are created by five commands
  that run outside any repair pass.

### 2. `clear_build_stage()` clears the key and nothing else

- **Originally** (kazoo): also call `reset_sequence_colors(respect_stage=False)`.
- **We did not.** Both callers repaint immediately afterwards (`show_sequence_colors` or
  `reset_sequence_colors`), so it would only paint twice.

### 3. `reset_sequence_colors` has no "show everything anyway" switch

- **Originally**: kazoo gave it `respect_stage=False` for `clear_build_stage` to call; the
  rocket plan kept the parameter "for explicitness" after decision 2 removed that caller.
- **We deleted it.** A parameter with no caller is an untested branch whose only possible effect
  is to silently defeat the feature. Code that genuinely wants everything visible clears the
  latch first.

### 4. The pure logic lives in its own Rhino-free module

- **Beyond both plans.** `rhino_bar_registry.py` imports `rhinoscriptsyntax`/`Rhino`/
  `scriptcontext` at module level, so nothing in it is reachable by `pytest`. The resolver moved
  to [core/build_stage.py](scripts/core/build_stage.py), matching how `joint_pair.py`,
  `scaffold_json.py` and `geometry.py` stay importable — which is what makes the three hardest
  branches (bar renumbered / stage bar deleted / nothing resolves) automatically tested.

---

## Status

| Step | File(s) | State |
| --- | --- | --- |
| 0 | `core/rhino_helpers.py` — shared `get_doc_string`/`set_doc_string` | landed `db41376` |
| 1 | `core/rhino_bar_registry.py` — `_tube_index` speed-up | landed `db41376` |
| 2 | `core/build_stage.py` — key, wire format, resolver, reminder text | landed `3c46b98` |
| 3 | registry — `get`/`set`/`clear_build_stage`, `apply_build_stage_visibility` | landed `3c46b98` |
| 4 | registry — `reset_sequence_colors` re-applies the filter | landed `3c46b98` |
| 5 | registry — `repair_on_entry` re-applies the filter | landed `3c46b98` |
| 6 | `rs_sequence_edit.py` — the toggle drives the latch | **done, uncommitted** |
| 7 | `rs_show_bar_action_plan.py` — toggle only | **done, uncommitted** |
| **8** | `rs_update_preview.py` + `core/rhino_joint_refresh.py` | **not started** |
| **9** | `rs_swap_robotic_tool.py` | **done, uncommitted** — confirmed leaking in Rhino, then fixed |
| 10 | registry — tube creation chokepoint | landed `3c46b98` |
| **11** | `rs_reorder_bar_id.py` — remap the stage bar id | **not started** |
| **12** | `rs_create_bar.py`, `rs_import_scaffold_json.py` — warnings | **not started** |
| 13 | `Su_note.md` §20 done; **`rhino_toolbar_entrypoints.md` not started** | partial |
| 14 | `tests/test_build_stage.py` — 24 tests | landed `3c46b98` |

**Nothing has been verified in Rhino yet**, and nothing is committed.

## The four remaining steps

Each closes one place where "hidden stays hidden" still breaks. They are independent of each
other and were deferred until the core loop (6+7) is confirmed in a real `.3dm`.

### 8. `rs_update_preview.py` + `core/rhino_joint_refresh.py`

It regenerates tubes and restores/re-snaps tools *after* the entry filter ran, and a new Rhino
object is born visible → unbuilt tubes and tools pop back.

- `_run_update_preview()`: `apply_build_stage_visibility()` immediately before `rs.Redraw()`
  ([:150](scripts/rs_update_preview.py#L150)), after the tool restore/side/re-snap passes.
- `main()`: re-apply after the `show_colors_preview()` branch ([:169](scripts/rs_update_preview.py#L169)).
- `mark_broken_links` ([rhino_joint_refresh.py:387-421](scripts/core/rhino_joint_refresh.py#L387-L421)):
  the filter stays on and `find_broken_links()` already scans hidden objects, so the *report*
  stays complete. What changes is the *marking* — skip `set_object_color`
  ([:412](scripts/core/rhino_joint_refresh.py#L412)), `_drop_tool_marker`
  ([:414](scripts/core/rhino_joint_refresh.py#L414)) and the `rs.SelectObjects`
  ([:420](scripts/core/rhino_joint_refresh.py#L420)) entry for any oid where `rs.IsObjectHidden`
  is true, and print a trailing `N of the M broken links are hidden by the stage filter —
  listed above, not marked or selected.` Keep the return type. The legend at
  [:469](scripts/core/rhino_joint_refresh.py#L469) claims "everything listed below is SELECTED"
  and needs the same qualifier.

### 9. `rs_swap_robotic_tool.py` — DONE

Confirmed in Rhino: swapping the tool revealed every hidden tool. It deletes and re-inserts
*every* tool block and is the one command that never calls `repair_on_entry`, so nothing else
in the run put them back. Fixed with `apply_build_stage_visibility(caller="RSSwapRoboticTool")`
as the final statement of `main()` — outside `replace_all_tool_instances`
([:205](scripts/rs_swap_robotic_tool.py#L205)) so its `suspend_redraw` batching stays intact,
and last so the invariant holds.

### 11. `rs_reorder_bar_id.py`

**Observed in Rhino:** reordering hides a bar that has an *earlier* step but a *larger* id
number — i.e. the stage is being resolved against the bar **id** after the rename while the
step numbers say otherwise, so the wrong side of the boundary gets hidden. That is the symptom
this step predicts, now confirmed. Deferred by the user for the moment.

Renaming bars can leave the stored `"B7|7"` pointing at a *different physical bar*. Add a block
to `_apply_rename` ([:299-380](scripts/rs_reorder_bar_id.py#L299-L380)), inside the
`with suspend_redraw():` and before `sc.doc.Views.Redraw()` ([:380](scripts/rs_reorder_bar_id.py#L380)):
remap the saved stage bar id through `bar_rename` and rewrite the key with that bar's new seq.
Add the key to the storage-location list in the docstring at
[:35-43](scripts/rs_reorder_bar_id.py#L35-L43).

`_run_relink()` re-derives ids from geometry rather than using `bar_rename`, so it can also
invalidate the stage — the resolver's `fallback`/`clear` branches cover it. Leave it.

### 12. Warnings where work would silently vanish

**Observed in Rhino:** RSCreateBar hides the newly created bar's **tube** but leaves its
**centerline curve** visible. That is a real gap in the step-10 chokepoint, not just a missing
warning: `hide_if_beyond_build_stage(baked_ids, curve_id)`
([rhino_bar_registry.py:1189](scripts/core/rhino_bar_registry.py#L1189)) hides only the baked
tube ids and never `curve_id` itself. The next command's full apply corrects it, so the
inconsistency is transient — but it is visible, and "no ghost geometry" is check 6 of the
verification list. Fix alongside this step: hide the centerline too. Deferred by the user for
the moment.


- `rs_create_bar.py`: after `repair_bar_sequences()` ([:40](scripts/rs_create_bar.py#L40)) —
  seqs are only assigned there — print `RSCreateBar (stage filter): B12 is later than the
  current stage B7 and will be HIDDEN from the next command onward.`
- `rs_import_scaffold_json.py`: bulk-creates bars and joints outside any pick, so add an
  end-of-command `apply_build_stage_visibility()` after the summary block
  ([:331](scripts/rs_import_scaffold_json.py#L331)).

### 13 (remainder). Docs

[docs/rhino_toolbar_entrypoints.md:70-72](docs/rhino_toolbar_entrypoints.md#L70-L72) — the
RSSequenceEdit section is a heading plus one bullet; write the persistent-filter behaviour, that
only ShowUnbuilt clears it, and that it changes what *other* commands can pick. Add a line to
RSJointPlace ([:74](docs/rhino_toolbar_entrypoints.md#L74)), RSBarSnap
([:59](docs/rhino_toolbar_entrypoints.md#L59)), RSUpdatePreview
([:152](docs/rhino_toolbar_entrypoints.md#L152)) and RSReorderBarID
([:250](docs/rhino_toolbar_entrypoints.md#L250)).

---

## Reference: what already landed

### The API

```
BUILD_STAGE_KEY = "scaffolding.build_stage"        # value: "B7|7"

# core/build_stage.py -- pure, no Rhino imports
format_build_stage(bar_id, seq)        -> "B7|7"
parse_build_stage(raw)                 -> (bar_id, seq) | None
resolve_build_stage_seq(raw, bar_map)  -> (status, seq, bar_id, message)
stage_filter_note(caller, bar_id, seq) -> str

# core/rhino_bar_registry.py -- needs a document
get_build_stage()                      -> (bar_id, seq) | None   # ONE doc-string read
set_build_stage(bar_id, seq)
clear_build_stage()                                              # clears the key only
apply_build_stage_visibility(caller=None, verbose=True) -> (bar_id, n_hidden) | None
hide_if_beyond_build_stage(object_ids, curve_id)                 # tube creation chokepoint
```

`get_build_stage()` must stay a bare key read — it runs on every command entry even when the
filter is off, so the unlatched cost is one string read and nothing else.

| status | when | seq / bar_id | caller does |
| --- | --- | --- | --- |
| `off` | `raw` empty | `None` | nothing, filter is not latched |
| `ok` | stage bar still in `bar_map` | its **current** seq | hide past `seq` |
| `fallback` | stage bar gone, some bar at/before the stored seq | the nearest **earlier** bar | rewrite the key to that bar, print `message`, then hide |
| `clear` | unparseable, or nothing at/before the stored seq | `None` | wipe the key, print `message`, show everything |

**Invariant, stated in the docstrings and enforced everywhere: the apply is the LAST
visibility-touching step of any command or refresh pass.**

### Two views, one rule difference

| | bars + tubes | joints | tools |
| --- | --- | --- | --- |
| Inside RSSequenceEdit (`show_sequence_colors`) | `seq > stage` hidden | follow parent bar | **only the active step's tool visible** |
| Persistent filter (`apply_build_stage_visibility`) | `seq > stage` hidden | follow parent bar | hidden only if owning bar is unbuilt |

Pressing Esc therefore makes the built bars' tools reappear. That is intended (decision 4): it
keeps RSSwapRoboticTool / RSInspectRoboticTool usable while the filter is on, and matches the
"keep robotic tools visible" work in `1833a8c`.

Joint visibility is **per block half**: a joint between built B2 and unbuilt B9 keeps its female
half visible and hides its male half.

### Verified facts this rests on

- `rs.AllObjects()` and `rs.ObjectsByLayer()` **always include hidden objects** — confirmed in
  `~/.rhinocode/py39-rh8/site-rhinopython/rhinoscript/selection.py:54-61` (`HiddenObjects = True`,
  no parameter to turn it off). So the whole identity/registry layer — `get_bar_seq_map`,
  `repair_bar_sequences`, `_find_existing_tube`, the layer enforcers, every export path — is
  unaffected by hiding.
- What *does* break on hidden geometry: **picking and osnap** (`pick_bar`, `rs.GetObjects`), and
  **`rs.SelectObjects` silently no-ops**. Nothing else — colours, user text, transforms and
  deletion all work.
- Layer-level hiding is **not** an option: `ensure_layer()` force-shows every layer on its path
  and is reached from `enforce_managed_layers` → `repair_on_entry` at the top of ~25 commands.
  Per-object `rs.HideObject` is the only durable mechanism.
- The existing temporary-hide helpers will **not** leak: `rs.HideObject` returns `False` when the
  object is already hidden, and `joint_pick_helpers.temporarily_hidden`,
  `rs_ik_keyframe._hide_inactive_tool_blocks` and `rs_define_robotic_tool` all guard on that.
- `rs_swap_robotic_tool.py` does **not** call `repair_on_entry` — step 9 is mandatory.

### Behaviour after the change (latch on, stage = B7)

| Action | Result |
| --- | --- |
| Esc out of RSSequenceEdit | colours reset; B8+ stay hidden; built bars' tools reappear |
| RSUpdatePreview (repair) | tubes/joints/tools repaired, then re-hidden *(needs step 8)* |
| RSSwapRoboticTool | tools swapped; unbuilt bars' tools re-hidden *(needs step 9)* |
| RSUpdatePreview > ShowColorsPreview | full report incl. hidden links; only visible ones marked/selected *(needs step 8)* |
| RSIKKeyframe | runs normally; on exit B8+ hidden again |
| RSJointPlace / RSBarSnap | reminder printed; unbuilt bars unpickable and un-snappable |
| RSCreateBar | new bar warned about, visible during the command, hidden from the next *(needs step 12)* |
| Save + reopen the `.3dm` | still hidden (object state and doc key both persist) |
| RSReorderBarID renumber | stage still points at the same physical bar *(needs step 11)* |
| Stage bar deleted | falls back to the nearest earlier step and says so |
| RSSequenceEdit > ShowUnbuilt | latch cleared, everything revealed |

---

## Verification

`tests/conftest.py` limits the suite to pure geometry/solver logic, so everything except
`build_stage.py` is verified by hand through the toolbar. Run `py -m pytest` first — the
baseline is **151 passed, 18 skipped** (~3m30s); the 13 scipy divide-by-zero warnings in
`test_s2_t1.py` are pre-existing. Then, in a document with ≥5 bars carrying joints and tools:

**Persistence**
1. RSSequenceEdit → active B3 → HideUnbuilt → **Esc**. B4..N stay hidden. *(core requirement)*
2. Save, close, reopen the `.3dm`. Still hidden; Document Properties → User Text shows
   `scaffolding.build_stage = B3|3`.
3. RSJointPlace (cancel), RSBarSnap (cancel), RSIKKeyframe (Esc) — each prints the reminder and
   leaves B4..N hidden. *(RSUpdatePreview and RSSwapRoboticTool will still leak until 8 and 9.)*
4. RSSequenceEdit → ShowUnbuilt → Esc. Everything visible, doc key gone.
5. RSSequenceEdit re-entry while latched resumes at the stage bar with ShowUnbuilt offered.

**Per-object correctness**
6. B4's tube **and** centerline both hidden — no ghost tube.
7. Joint between built B2 and unbuilt B5: female half (parent B2) visible, male half hidden.
8. Tools on B1..B3 visible; tool on B5 hidden. Check a joint whose two bars straddle the stage to
   confirm the tool→bar binding used the **male** side.

**Destructive / renaming**
9. Stage B7 → RSRemoveBar on B7 → falls back to the nearest earlier step and prints why.
10. `Ctrl+Z` right after a latched command, then run any command — visibility and the doc key
    self-heal on the next apply.

**Interaction hazards**
11. Stage B3 → RSJointPlace: B5 cannot be picked, and the printed reminder says how to fix it.
    *(acceptance test for the strict-reach trade-off)*
12. Stage B3 → RSSequenceEdit → EditSupports: all bars pickable inside the picker; on exit the
    stage view returns and the latch is intact.
13. Stage B3 → RSIKKeyframe on B2: at exit nothing leaks — B4..N still hidden.

**Performance**
14. Latch **off**: RSJointPlace startup indistinguishable from before (one string read).
15. Latch **on**, ~100 bars: RSUpdatePreview shows no visible regression (this is what step 1
    and the `bar_map` threading in RSSequenceEdit buy).
16. Walking the sequence with Next is responsive — one document scan per step, not five.

**Once steps 8, 9, 11 and 12 land, add:**
17. RSUpdatePreview and RSSwapRoboticTool leave B4..N hidden.
18. ShowColorsPreview: broken links on hidden bars are **listed** with the hidden count and not
    marked/selected; visible ones are marked and selected as before.
19. Stage B7 → RSReorderBarID renumber → stage still points at the same physical bar.
20. Stage B7 → RSCreateBar → warning printed; new bar visible during the command, hidden after.
21. Stage B3 → RSBarSnap trims an unbuilt bar → its regenerated tube stays hidden (step 10).
