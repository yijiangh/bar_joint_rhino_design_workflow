# Finish the persistent HideUnbuilt build-stage filter (steps 2–12)

## Context

`HideUnbuilt` in RSSequenceEdit hides every bar, joint and tool later than the active
assembly step — the fastest way to see what the scaffold looks like at a given build
stage. Today that state is only a session flag ([rs_sequence_edit.py:84](scripts/rs_sequence_edit.py#L84)):
pressing Esc runs `reset_sequence_colors()` which unconditionally re-shows everything,
and any refresh command that re-creates tubes/joints/tools brings them back too, because
a newly created Rhino object is always visible.

Goal: make it a **latch stored in the document** — one document user-text entry,
`scaffolding.build_stage = "B7|7"`, saved inside the `.3dm`. Once set, the unbuilt parts
stay hidden through command exits, refresh passes, and save/reload, until `ShowUnbuilt`.

The full design lives in [tasks/sequential-inventing-kazoo.md](tasks/sequential-inventing-kazoo.md).
Steps 0 (shared `get_doc_string`/`set_doc_string`) and 1 (`_tube_index` speed-up) landed in
`db41376`. `grep -rn "build_stage" scripts/` returns zero hits, so **steps 2–12 are all
still to do**. This plan finishes them in one pass.

Note: the task file's line references for `rhino_bar_registry.py` are stale by +15..+40
(step 1 grew the file). The line numbers below are current.

## Decisions taken beyond the task file

1. **The pure arithmetic moves to its own Rhino-free module**, `scripts/core/build_stage.py`.
   `rhino_bar_registry.py` imports `rhinoscriptsyntax`/`Rhino`/`scriptcontext` at module
   level ([:15-18](scripts/core/rhino_bar_registry.py#L15-L18)), so nothing in it can be
   reached by `pytest`. Splitting the resolver out means the three branches that are hardest
   to check by hand (bar renumbered / stage bar deleted / nothing resolves) get an automated
   test. Matches how `joint_pair.py`, `scaffold_json.py`, `geometry.py` stay importable.
2. **Step 9 covers tubes only, not tools.** Every path that creates a robotic tool already
   ends with an apply: the three batch functions run only from RSUpdatePreview (step 7),
   `replace_all_tool_instances` only from RSSwapRoboticTool (step 8), and a tool created
   alongside a new joint always lands on a just-picked — therefore visible — bar. A per-tool
   hook would never fire on anything the other steps don't already cover.
3. **`clear_build_stage()` clears the key and nothing else.** The task file had it also call
   `reset_sequence_colors(respect_stage=False)`, but both callers repaint immediately
   afterwards anyway, so that would just double-paint. `respect_stage=False` still exists on
   `reset_sequence_colors` for explicitness.

Everything else follows the task file, including its five confirmed decisions (visibility
only, no new toolbar command, strict reach, tools follow their bar, diagnostics see
everything but mark only what's visible).

---

## 2. New file: `scripts/core/build_stage.py` — the pure part

No Rhino imports. Holds the key name, the wire format, the resolver and the reminder text.

```python
BUILD_STAGE_KEY = "scaffolding.build_stage"        # value: "B7|7"

format_build_stage(bar_id, seq)   -> "B7|7"
parse_build_stage(raw)            -> (bar_id, seq) | None
resolve_build_stage_seq(raw, bar_map) -> (status, seq, bar_id, message)
stage_filter_note(caller, bar_id, seq) -> str
```

`bar_map` is `get_bar_seq_map()`'s `{bar_id: (oid, seq)}` shape, passed straight through;
the resolver only reads `entry[1]`. `status` is one of:

| status | when | seq / bar_id | caller does |
| --- | --- | --- | --- |
| `off` | `raw` empty | `None` | nothing, filter is not latched |
| `ok` | stage bar still in `bar_map` | its **current** seq | hide past `seq` |
| `fallback` | stage bar gone, some bar sits at or before the stored seq | the nearest **earlier** bar | rewrite the key to that bar, print `message`, then hide |
| `clear` | unparseable, or nothing at/before the stored seq | `None` | wipe the key, print `message`, show everything |

Why the value carries both id and step: bar ids are not permanent — RSReorderBarID renames
them, so after a renumber the bar you staged at might be called B4 while a *different* bar
is now B7. The id is the primary key, the step is the safety net.

`stage_filter_note` is the single shared reminder, e.g.

```
RSJointPlace (stage filter): showing up to B7 (step 7). Later bars/joints/tools are HIDDEN
and cannot be picked or snapped to. RSSequenceEdit > ShowUnbuilt to reveal.
```

## 3. `scripts/core/rhino_bar_registry.py` — the Rhino part

New section right after `reset_sequence_colors` (~[:805](scripts/core/rhino_bar_registry.py#L805)):

```python
get_build_stage()                          -> (bar_id, seq) | None   # ONE doc-string read
set_build_stage(bar_id, seq)
clear_build_stage()                                                  # clears the key only
apply_build_stage_visibility(caller=None, verbose=True) -> (bar_id, n_hidden) | None
```

`get_build_stage()` must stay a bare key read — it runs on every command entry even when the
filter is off, so the unlatched cost is one string read and nothing else.

`apply_build_stage_visibility()`:

1. `raw = get_doc_string(BUILD_STAGE_KEY)`; return `None` immediately if empty. ← the whole unlatched cost
2. one `get_bar_seq_map()`, then `resolve_build_stage_seq`; act on `status` per the table above
3. inside `rhino_helpers.suspend_redraw()` ([:182-203](scripts/core/rhino_helpers.py#L182-L203)) —
   **not** a raw `rs.EnableRedraw` pair, so calling it from inside another batched block doesn't
   kill the outer batching:
   - one `_tube_index()` ([:830](scripts/core/rhino_bar_registry.py#L830)); per bar,
     `_set_visible(obj, seq <= stage_seq)` over `_bar_curve_and_tube(oid, tube_index)`
   - **one** pass over the three joint layers explicitly (not `_joint_layer_objects()`, which
     loses the layer): set each half's visibility from its own `parent_bar_id`, and while on
     the **male + ground** layers only, record `{joint_id: parent_bar_id}`
   - one `_tool_layer_objects()` pass: a tool follows the bar owning its **male/ground** half,
     exactly like `get_active_tool_oids` ([:667-695](scripts/core/rhino_bar_registry.py#L667-L695)).
     Unknown owner → leave visible, so a stray tool is never invisible-and-unfindable.
4. `verbose` → print `stage_filter_note(...)` once

**Sets visibility only, never colour.** Joint visibility stays per block half, as today: a
joint between built B2 and unbuilt B9 keeps its female half visible and hides its male half.

**Invariant to state in the docstring and enforce everywhere: the apply is the LAST
visibility-touching step of any command or refresh pass.**

## 4. `reset_sequence_colors(respect_stage=True)` — [:788-804](scripts/core/rhino_bar_registry.py#L788-L804)

Convert its raw `rs.EnableRedraw(False)/(True)` pair ([:793](scripts/core/rhino_bar_registry.py#L793)/[:804](scripts/core/rhino_bar_registry.py#L804))
to `with suspend_redraw():`, then call `apply_build_stage_visibility(verbose=False)` at the end
of the block when `respect_stage`. Nesting `suspend_redraw` is safe (it saves and restores the
previous state); mixing it with a raw pair is not.

This one change makes all three existing exits honour the latch with no per-caller edit:
[rs_sequence_edit.py:493](scripts/rs_sequence_edit.py#L493),
[rs_show_bar_action_plan.py:924](scripts/rs_show_bar_action_plan.py#L924),
[rs_ik_keyframe.py:2290](scripts/rs_ik_keyframe.py#L2290). Rewrite the docstring — it is no
longer an unconditional "show everything".

RSIKKeyframe's ordering is already correct: `_show_objects(extra_hidden_tools)` runs *before*
the reset, so the filter has the last word.

## 5. `repair_on_entry` hook — [:1141-1190](scripts/core/rhino_bar_registry.py#L1141-L1190)

Call `apply_build_stage_visibility(caller=caller, verbose=True)` as the **final** statement,
after the count-mismatch warning at [:1186-1190](scripts/core/rhino_bar_registry.py#L1186-L1190) —
i.e. after `enforce_managed_layers` has force-shown the layers and `update_all_previews` has
regenerated tubes. Not inside `enforce_managed_layers`: it runs *before* `update_all_previews`,
so fresh tubes would escape. This is what puts the filter in front of all 26 commands.

## 6. `scripts/rs_sequence_edit.py` — drive the latch from the toggle

The latch is written **only** by an explicit user toggle. `show_sequence_colors` stays pure.

- `__init__` ([:82-87](scripts/rs_sequence_edit.py#L82-L87)): if latched, `show_unbuilt = False`
  and pre-set `active_bar_id` to the stage bar, so the command resumes where you left it.
  Without this, latching at B7 and then merely clicking B3 to browse would rewrite the stage to B3.
- `main` ([:437](scripts/rs_sequence_edit.py#L437)): after the empty-`bar_map` guard, if the
  pre-set `active_bar_id` is in `bar_map`, call `session.set_active(...)` so the stage view and
  status line appear immediately. The guard matters — the stage bar may have been deleted.
- `set_active` ([:153-157](scripts/rs_sequence_edit.py#L153-L157)): while `show_unbuilt is False`,
  `set_build_stage(bar_id, seq)` — the assembly stage *is* the active step, so Next/Previous/
  typed-step keep the saved stage in sync with the screen.
- `toggle_unbuilt` ([:343-347](scripts/rs_sequence_edit.py#L343-L347)): hide → `set_build_stage`;
  show → `clear_build_stage()`. The existing `show_sequence_colors` call then repaints correctly
  either way.
- `do_edit_supports` ([:241-341](scripts/rs_sequence_edit.py#L241-L341)) needs **no change** — it
  forces everything visible via `show_sequence_colors(..., True)` at [:273](scripts/rs_sequence_edit.py#L273)
  and restores at [:338-339](scripts/rs_sequence_edit.py#L338-L339), never touching `set_active`.
  Verify this holds after the edit.
- `_print_status` ([:141-148](scripts/rs_sequence_edit.py#L141-L148)): say
  `hidden (persists after exit)`. Update the module docstring bullet
  ([:22-25](scripts/rs_sequence_edit.py#L22-L25)) and the exit message
  ([:494](scripts/rs_sequence_edit.py#L494)) — "Display restored" is untrue when latched.

## 7. `scripts/rs_show_bar_action_plan.py` — toggle only

Change **only** `toggle_unbuilt` ([:358-361](scripts/rs_show_bar_action_plan.py#L358-L361)) to
write/clear the latch. Leave `self.show_unbuilt = False` at [:301](scripts/rs_show_bar_action_plan.py#L301)
and the `refresh()` re-assert at [:474](scripts/rs_show_bar_action_plan.py#L474) alone — both run
with no user involvement, so if either wrote the latch, merely opening the IK plan viewer would
hide the model permanently. `cleanup()` needs nothing; it goes through `reset_sequence_colors`.

## 8. `scripts/rs_update_preview.py` + `core/rhino_joint_refresh.py`

- `_run_update_preview()`: `apply_build_stage_visibility()` immediately before `rs.Redraw()`
  ([:150](scripts/rs_update_preview.py#L150)), after the tool restore/side/re-snap passes.
- `main()`: re-apply after the `show_colors_preview()` branch ([:169](scripts/rs_update_preview.py#L169)).
- `mark_broken_links` ([rhino_joint_refresh.py:387-421](scripts/core/rhino_joint_refresh.py#L387-L421)):
  the filter stays on and `find_broken_links()` already scans hidden objects, so the *report* stays
  complete. What changes is the *marking* — skip `set_object_color`, `_drop_tool_marker` and the
  `rs.SelectObjects` entry for any oid where `rs.IsObjectHidden` is true (`rs.SelectObjects`
  silently no-ops on hidden objects anyway), and print a trailing
  `N of the M broken links are hidden by the stage filter — listed above, not marked or selected.`
  Keep the existing return type.
- The legend at [:469](scripts/core/rhino_joint_refresh.py#L469) claims "everything listed below is
  SELECTED" — add the qualifier.

## 9. `scripts/rs_swap_robotic_tool.py`

`apply_build_stage_visibility()` at [:213](scripts/rs_swap_robotic_tool.py#L213), after the
re-place summary print — i.e. after `replace_all_tool_instances` ([:205](scripts/rs_swap_robotic_tool.py#L205))
has deleted and re-inserted every tool block, and *outside* that call so it doesn't break its
`suspend_redraw` batching. This command never calls `repair_on_entry`, so the step is mandatory.

## 10. Tube creation chokepoint — `ensure_bar_preview` [:902-959](scripts/core/rhino_bar_registry.py#L902-L959)

Hide the freshly baked tube if its bar is beyond the stage, just before
`return baked_ids, status` at [:959](scripts/core/rhino_bar_registry.py#L959). Needed because
RSCreateBar, RSBarEdit, RSBarSnap, RSBarBrace and RSBarSubfloor all call it outside any repair
pass — and RSBarSnap/RSBarBrace/RSBarSubfloor regenerate tubes for **existing** bars after
trimming, which would leave a visible tube floating over a hidden centerline. The tube is what
you actually see, so the filter would look broken.

Only the create/regenerate path needs it: the early `return [existing], "reused"` at
[:917](scripts/core/rhino_bar_registry.py#L917) hands back an object whose hidden state the
last apply already set.

Keep it cheap — one doc-string read, then the bar's own `bar_seq` user text compared against the
**stored** stage seq (no `get_bar_seq_map()`, no resolve). `repair_on_entry`'s full apply corrects
any drift on the next command; say so in the comment. A brand-new bar has no `bar_seq` until
`repair_bar_sequences()` runs, so it stays visible during RSCreateBar — which is the intended
behaviour (step 12).

## 11. `scripts/rs_reorder_bar_id.py`

Add a block "8" to `_apply_rename` ([:299-385](scripts/rs_reorder_bar_id.py#L299-L385)), inside the
`with suspend_redraw():` and before `sc.doc.Views.Redraw()` at [:380](scripts/rs_reorder_bar_id.py#L380):
remap the saved stage bar id through `bar_rename` and rewrite the key with the bar's new seq, so
the stage keeps pointing at the same physical bar. Add the key to the storage-location list in the
docstring at [:35-43](scripts/rs_reorder_bar_id.py#L35-L43).

`_run_relink()` ([:446-473](scripts/rs_reorder_bar_id.py#L446-L473)) re-derives ids from geometry
rather than using `bar_rename`, so it can also invalidate the stage — the `fallback`/`clear`
branches of the resolver cover it. Leave it.

## 12. Warnings where work would silently vanish

- `rs_create_bar.py`: after `repair_bar_sequences()` at [:40](scripts/rs_create_bar.py#L40) (seqs
  are only assigned there), print
  `RSCreateBar (stage filter): B12 is later than the current stage B7 and will be HIDDEN from the
  next command onward.`
- `rs_import_scaffold_json.py`: bulk-creates bars and joints outside any pick
  ([:282-301](scripts/rs_import_scaffold_json.py#L282-L301)), so add an end-of-command
  `apply_build_stage_visibility()` after the summary block ends at
  [:331](scripts/rs_import_scaffold_json.py#L331).

## 13. Docs

- [docs/rhino_toolbar_entrypoints.md:70-72](docs/rhino_toolbar_entrypoints.md#L70-L72) — the
  RSSequenceEdit section is currently a heading plus one bullet; write the persistent-filter
  behaviour, that only ShowUnbuilt clears it, and that it changes what *other* commands can pick.
  Add a line to RSJointPlace ([:74](docs/rhino_toolbar_entrypoints.md#L74)), RSBarSnap
  ([:59](docs/rhino_toolbar_entrypoints.md#L59)), RSUpdatePreview
  ([:152](docs/rhino_toolbar_entrypoints.md#L152)) and RSReorderBarID
  ([:250](docs/rhino_toolbar_entrypoints.md#L250)).
- [docs/Su_note.md](docs/Su_note.md) — new `## 19.` (the file ends at 1059), covering the three
  glossary points, labelled convention vs. real Rhino/Python API: session var vs `sc.sticky` vs
  document user text vs per-object user text; why the stage stores id **and** step; why a tool
  resolves through its male/ground half.
- Docstrings that become wrong if left alone: [rs_sequence_edit.py:22-25](scripts/rs_sequence_edit.py#L22-L25),
  [rhino_bar_registry.py:706-726](scripts/core/rhino_bar_registry.py#L706-L726) and
  [:789-790](scripts/core/rhino_bar_registry.py#L789-L790),
  [rs_reorder_bar_id.py:35-43](scripts/rs_reorder_bar_id.py#L35-L43).
- [tasks/sequential-inventing-kazoo.md](tasks/sequential-inventing-kazoo.md) — record the three
  decisions above and refresh the stale `rhino_bar_registry.py` line references.

## 14. Test

New `tests/test_build_stage.py`, importing `core.build_stage` directly — no Rhino stubs needed,
which is the point of the split. Cover `resolve_build_stage_seq` over a hand-built
`{bar_id: (oid, seq)}` dict:

- empty/`None` raw → `off`
- stage bar present → `ok`, and returns the bar's **current** seq (the renumber case: `"B7|7"`
  against a map where B7 now sits at step 4 must yield 4, not 7)
- stage bar deleted, earlier bars exist → `fallback` to the nearest earlier bar
- stage bar deleted, nothing at or before the stored seq → `clear`
- garbage raw (`"B7"`, `"|"`, `"B7|x"`) → `clear`, never an exception

Plus round-trip `parse_build_stage(format_build_stage(...))`.

---

## Verification

`tests/conftest.py` limits the suite to pure geometry/solver logic, so everything except
`build_stage.py` is verified by hand through the toolbar. Run `pytest tests/test_build_stage.py`
first, then in a document with ≥5 bars carrying joints and tools:

**Persistence**
1. RSSequenceEdit → active B3 → HideUnbuilt → **Esc**. B4..N stay hidden. *(core requirement)*
2. Save, close, reopen the `.3dm`. Still hidden; Document Properties → User Text shows
   `scaffolding.build_stage = B3|3`.
3. RSUpdatePreview, RSSwapRoboticTool, RSJointPlace (cancel), RSBarSnap (cancel), RSIKKeyframe (Esc)
   — each prints the reminder and leaves B4..N hidden.
4. RSSequenceEdit → ShowUnbuilt → Esc. Everything visible, doc key gone. RSUpdatePreview re-hides nothing.
5. RSSequenceEdit re-entry while latched resumes at the stage bar with ShowUnbuilt offered.

**Per-object correctness**
6. B4's tube **and** centerline both hidden — no ghost tube.
7. Joint between built B2 and unbuilt B5: female half (parent B2) visible, male half hidden.
8. Tools on B1..B3 visible; tool on B5 hidden. Check a joint whose two bars straddle the stage to
   confirm the tool→bar binding used the **male** side.

**Refresh commands**
9. RSSwapRoboticTool with the latch on: re-placed tools on unbuilt bars must not reappear.
10. ShowColorsPreview: broken links on hidden bars are **listed** with the hidden count and not
    marked/selected; visible ones are marked and selected as before.

**Destructive / renaming**
11. Stage B7 → RSReorderBarID renumber → stage still points at the same physical bar.
12. Stage B7 → RSRemoveBar on B7 → falls back to the nearest earlier step and prints why.
13. Stage B7 → RSCreateBar → warning printed; new bar visible during the command, hidden after.
14. Stage B3 → RSBarSnap trims an unbuilt bar → its regenerated tube stays hidden (step 10).

**Interaction hazards**
15. Stage B3 → RSJointPlace: B5 cannot be picked, and the printed reminder says how to fix it.
    *(acceptance test for the strict-reach trade-off)*
16. Stage B3 → RSSequenceEdit → EditSupports: all bars pickable inside the picker; on exit the
    stage view returns and the latch is intact.
17. Stage B3 → RSIKKeyframe on B2: at exit nothing leaks — B4..N still hidden.
18. `Ctrl+Z` right after a latched command, then run any command — visibility and the doc key
    self-heal on the next apply.

**Performance**
19. Latch **off**: RSJointPlace startup indistinguishable from before (one string read).
20. Latch **on**, ~100 bars: RSUpdatePreview shows no visible regression.
