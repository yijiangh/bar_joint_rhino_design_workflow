# Persistent HideUnbuilt — a document-level "build stage" filter

## Context

`HideUnbuilt` in [rs_sequence_edit.py](scripts/rs_sequence_edit.py) hides every bar, joint
and tool later than the active assembly step — the fastest way to check what the scaffold
looks like at a given build stage. But the state is only a session flag
(`_SequenceSession.show_unbuilt`, [rs_sequence_edit.py:84](scripts/rs_sequence_edit.py#L84)):
pressing Esc runs `reset_sequence_colors()` ([rs_sequence_edit.py:493](scripts/rs_sequence_edit.py#L493)),
which unconditionally `rs.ShowObject`s everything again. The same happens leaving
RSShowBarActionPlan and RSIKKeyframe, and any refresh command (RSUpdatePreview,
RSSwapRoboticTool) that re-creates tubes/joints/tools brings them back visible, because a
newly created Rhino object is always visible.

Goal: `HideUnbuilt` becomes a **latch stored in the document**. Once set, the unbuilt parts
stay hidden through command exits, refresh passes, and save/reload, until `ShowUnbuilt`.

## How it works, in plain terms

We save one line of text inside the .3dm file:

```
scaffolding.build_stage = "B7|7"       # bar id | step number.  Missing = filter off.
```

Then one new function, `apply_build_stage_visibility()`, reads that line and hides everything
later than step 7. We call it at the end of every place that could have made things visible
again. That's the whole feature — the rest of the plan is *where* to call it and what to do
when the stage bar gets renamed or deleted.

Three glossary points, because they came up:

- **Where the state lives.** `sc` is Rhino's `scriptcontext` module (nothing to do with the
  Python language). `sc.sticky` is a dict that survives between script runs but dies when
  Rhino closes — **not used here**. `sc.doc.Strings` (document user text) is saved *inside
  the .3dm*, so it survives Rhino restarts and travels with the file. That is what we use,
  and it matches per-object hidden state, which is also stored in the .3dm.
- **Why `"B7|7"` and not just `"B7"`.** Bar ids are not permanent — RSReorderBarID renames
  them, so after a renumber the bar you staged at might be called B4 while a *different* bar
  is now called B7. Keeping the step number alongside gives a safety net: use bar `B7` if it
  still exists; if it was deleted, fall back to the nearest earlier step (step 6) so the view
  barely moves instead of the whole model reappearing; if neither resolves, switch the filter
  off, show everything, and print why.
- **Which bar does a robotic tool belong to.** A joint has two block halves that share one
  `joint_id` ([joint_placement.py:336](scripts/core/joint_placement.py#L336)) but store
  *different* `parent_bar_id` values — the female half stores the LE bar
  ([:366](scripts/core/joint_placement.py#L366)), the male half the LN bar
  ([:377](scripts/core/joint_placement.py#L377)). A tool block stores only `joint_id`, no bar
  at all. So scanning all three joint layers gives two conflicting answers per tool and
  whichever is read last wins. The repo already settled this in `get_active_tool_oids`
  ([rhino_bar_registry.py:665-676](scripts/core/rhino_bar_registry.py#L665-L676)): a tool
  belongs to the bar owning the **male** (or ground) half — physically, the bar the gripper
  is holding. The new code must scan male + ground only, exactly like that function.

## Decisions confirmed with you

1. **Visibility only.** Sequence colours (green/blue/grey) still reset on exit as today; only
   the hidden state persists.
2. **No new toolbar command.** Cleared from `RSSequenceEdit > ShowUnbuilt`, which will already
   read `ShowUnbuilt` on entry because the latch is remembered. No `.rui` edit, no alias
   re-registration.
3. **Strict reach.** Every command running the shared `repair_on_entry()` re-applies the filter
   at startup. An unbuilt bar is therefore unpickable in RSJointPlace/RSBarSnap until the filter
   is cleared — accepted trade-off for "nothing ever leaks back".
4. **Tools follow their bar.** The persistent filter hides a tool only when its owning bar is
   unbuilt; tools on built bars stay visible.
5. **Diagnostics see everything; marks only touch what's visible.** RSUpdatePreview's
   ShowColorsPreview still *scans and reports* hidden objects, but only *colours, dots and
   selects* visible ones. The filter is not suspended.

## The changes at a glance

| # | File | In one sentence |
| --- | --- | --- |
| 0 | `core/rhino_helpers.py` | Move the "read/write a string in the document" pair to a shared home so the registry can use it. No behaviour change. |
| 1 | `core/rhino_bar_registry.py` | Build the bar→tube lookup once instead of re-scanning the layer per bar. Pure speed-up. No behaviour change. |
| 2 | `core/rhino_bar_registry.py` | Add the build-stage functions: read/write the saved line, and the one function that does the hiding. Nothing calls them yet. |
| 3 | `core/rhino_bar_registry.py` | `reset_sequence_colors()` re-applies the filter after showing everything — this alone fixes "Esc reveals everything" in all three commands. |
| 4 | `core/rhino_bar_registry.py` | `repair_on_entry()` re-applies the filter at startup — this is what makes it stick across every other command. |
| 5 | `scripts/rs_sequence_edit.py` | HideUnbuilt saves the stage, ShowUnbuilt clears it, and re-entering resumes at the saved step. |
| 6 | `scripts/rs_show_bar_action_plan.py` | Its ShowUnbuilt/HideUnbuilt writes the same latch — one line's worth of change, nothing else touched. |
| 7 | `scripts/rs_update_preview.py` | Re-apply at the end of the repair pass; make ShowColorsPreview report hidden problems but not mark them. |
| 8 | `scripts/rs_swap_robotic_tool.py` | Re-apply after tools are swapped (this command has no startup hook, so it's required). |
| 9 | `core/rhino_bar_registry.py`, `core/rhino_tool_place.py` | Hide a tube/tool at the moment it is created if its bar is beyond the stage, so nothing flashes back on screen. |
| 10 | `scripts/rs_reorder_bar_id.py` | Rename the saved bar id along with the bars, so the stage keeps pointing at the same physical bar. |
| 11 | `scripts/rs_create_bar.py`, `scripts/rs_import_scaffold_json.py` | Warn when newly made bars are beyond the stage (they'd otherwise seem to vanish); re-apply after a bulk import. |
| 12 | docs | Update the entrypoint doc, `Su_note.md`, and the docstrings that would otherwise be wrong. |

Steps 0–4 are the feature. Steps 5–8 wire it to the buttons. Steps 9–12 are the rough edges.
Check in Rhino after 1, 4 and 5.

---

# Detailed steps

## Verified facts this rests on

- `rs.AllObjects()` and `rs.ObjectsByLayer()` **always include hidden objects** — confirmed in
  `~/.rhinocode/py39-rh8/site-rhinopython/rhinoscript/selection.py:54-61` (`HiddenObjects = True`,
  with no parameter to turn it off). So the whole identity/registry layer — `get_bar_seq_map`,
  `repair_bar_sequences`, `_find_existing_tube`, the layer enforcers, every export path — is
  unaffected by hiding. Independent in-repo proof: `ShowUnbuilt` can only un-hide joints/tools
  today because `_joint_layer_objects()` returns hidden ones.
- What *does* break on hidden geometry: **picking and osnap** (`pick_bar`, `rs.GetObjects`), and
  **`rs.SelectObjects` silently no-ops**. Nothing else — colours, user text, transforms and
  deletion all work.
- Layer-level hiding is **not** an option: `ensure_layer()`
  ([rhino_helpers.py:52-68](scripts/core/rhino_helpers.py#L52-L68)) force-shows every layer on its
  path and is reached from `enforce_managed_layers` → `repair_on_entry` at the top of ~25 commands.
  Per-object `rs.HideObject` is the only durable mechanism.
- The existing temporary-hide helpers will **not** leak: `rs.HideObject` returns `False` when the
  object is already hidden, and `joint_pick_helpers.temporarily_hidden` (:57-63),
  `rs_ik_keyframe._hide_inactive_tool_blocks` (:1189-1192) and `rs_define_robotic_tool` (:134-140)
  all guard on that. They cannot un-hide something the latch hid.
- `rs_swap_robotic_tool.py` does **not** call `repair_on_entry` — step 8 is mandatory.

## Two views, one rule difference (say this in the docstrings)

| | bars + tubes | joints | tools |
| --- | --- | --- | --- |
| Inside RSSequenceEdit (`show_sequence_colors`, unchanged) | `seq > stage` hidden | follow parent bar | **only the active step's tool visible** |
| Persistent filter (`apply_build_stage_visibility`, new) | `seq > stage` hidden | follow parent bar | hidden only if owning bar is unbuilt |

Pressing Esc therefore makes the built bars' tools reappear. That is intended (decision 4): it
keeps RSSwapRoboticTool / RSInspectRoboticTool usable while the filter is on and matches the
"keep robotic tools visible" work in `1833a8c`.

Joint visibility is **per block half**, exactly as today
([rhino_bar_registry.py:754-760](scripts/core/rhino_bar_registry.py#L754-L760)): a joint between
built B2 and unbuilt B9 keeps its female half visible and hides its male half.

## 0. `scripts/core/rhino_helpers.py` — one home for doc-string state

Add public `get_doc_string(key)` / `set_doc_string(key, value)` — the `None`-on-empty,
swallow-exceptions pattern currently duplicated at
[rhino_bar_pick.py:71-83](scripts/core/rhino_bar_pick.py#L71-L83) and
[rhino_tool_place.py:53-68](scripts/core/rhino_tool_place.py#L53-L68) — with a lazy
`scriptcontext` import so the module stays importable outside Rhino. Point both duplicates at it.

`rhino_helpers` is the only cycle-free home: `rhino_bar_pick` imports *from* `rhino_bar_registry`,
so the registry cannot pull the helpers from there, and `rhino_bar_registry` already imports from
`rhino_helpers`.

## 1. `scripts/core/rhino_bar_registry.py` — tube index (pure speed-up)

`_bar_curve_and_tube` ([:620](scripts/core/rhino_bar_registry.py#L620)) calls `_find_existing_tube`
([:814](scripts/core/rhino_bar_registry.py#L814)), which does a fresh layer scan plus a
`GetUserText` per tube — **for every bar**. `show_sequence_colors` already pays this inside an
interactive command, but step 4 puts it on *every command entry*, where it becomes a latency tax.

Add `_tube_index()` → `{axis_guid_str: tube_oid}` built in one layer pass, and
`_bar_curve_and_tube(curve_id, tube_index=None)`. Thread it through `show_sequence_colors`
([:736](scripts/core/rhino_bar_registry.py#L736)) and `reset_sequence_colors`
([:779](scripts/core/rhino_bar_registry.py#L779)). **Verify RSSequenceEdit behaves identically
before continuing.**

## 2. `scripts/core/rhino_bar_registry.py` — the build-stage API

New section after `reset_sequence_colors` (~[:788](scripts/core/rhino_bar_registry.py#L788)):

```
BUILD_STAGE_KEY = "scaffolding.build_stage"          # value: "B7|7"

get_build_stage()                  -> (bar_id, seq) | None    # bare key read, NO document scan
set_build_stage(bar_id, seq)
clear_build_stage()                -> clears the key, then reset_sequence_colors(respect_stage=False)
resolve_build_stage_seq(bar_map)   -> int | None              # id first, seq fallback, auto-clear + print
apply_build_stage_visibility(verbose=True) -> (bar_id, n_hidden) | None
stage_filter_note(caller)          -> the single shared reminder string
```

- `get_build_stage()` must stay a pure key read — it runs on every `repair_on_entry` even when the
  filter is off, so the unlatched cost is one string read and nothing else.
- `apply_build_stage_visibility()` early-returns `None` when unlatched. Otherwise: **one**
  `get_bar_seq_map()`, **one** `_tube_index()`, **one** male+ground pass to build
  `{joint_id: parent_bar_id}`, **one** joint-layer pass, **one** tool-layer pass. Sets
  **visibility only, never colour**. Reuses `_joint_layer_objects()`, `_tool_layer_objects()`,
  `_set_visible()`.
- Wrap it in `rhino_helpers.suspend_redraw` ([:139-154](scripts/core/rhino_helpers.py#L139-L154)),
  **not** a raw `rs.EnableRedraw(False)/(True)` pair — `suspend_redraw` saves and restores the
  previous state, so calling this from inside another batched block doesn't kill the outer batching.
- `verbose=True` prints one reminder naming the stage and the way out. Without it, a user who
  forgets the latch has no idea why the model looks empty:
  `RSJointPlace (stage filter): showing up to B7 (step 7). Later bars/joints/tools are HIDDEN and cannot be picked or snapped to. RSSequenceEdit > ShowUnbuilt to reveal.`

**Invariant to state once and enforce everywhere: the apply is the LAST visibility-touching step
of any command or refresh pass.**

## 3. `reset_sequence_colors(respect_stage=True)`

[rhino_bar_registry.py:773](scripts/core/rhino_bar_registry.py#L773). After resetting colours and
showing everything, call `apply_build_stage_visibility()` when `respect_stage` is true. This one
change makes all three existing exits honour the latch with no per-caller edit:
[rs_sequence_edit.py:493](scripts/rs_sequence_edit.py#L493),
[rs_show_bar_action_plan.py:924](scripts/rs_show_bar_action_plan.py#L924),
[rs_ik_keyframe.py:2290](scripts/rs_ik_keyframe.py#L2290). Rewrite the docstring — it is no longer
an unconditional "show everything". `respect_stage=False` exists for `clear_build_stage`.

Ordering in RSIKKeyframe is already correct: `_show_objects(extra_hidden_tools)`
([:2289](scripts/rs_ik_keyframe.py#L2289)) runs before the reset, so the filter has the last word.

## 4. `repair_on_entry` hook

[rhino_bar_registry.py:1102](scripts/core/rhino_bar_registry.py#L1102): call
`apply_build_stage_visibility(verbose=True)` as the **final** statement, after the sanity print at
[:1147-1151](scripts/core/rhino_bar_registry.py#L1147-L1151) — i.e. after `enforce_managed_layers`
has force-shown the layers and `update_all_previews` has regenerated tubes. Not
`enforce_managed_layers`: it runs *before* `update_all_previews`, so fresh tubes would escape.

## 5. `scripts/rs_sequence_edit.py` — drive the latch from the toggle

The latch is written **only** by an explicit user toggle. `show_sequence_colors` stays pure.

- `__init__` ([:82-88](scripts/rs_sequence_edit.py#L82-L88)): if latched, set
  `self.show_unbuilt = False` **and** pre-select the stage bar as `active_bar_id`, so the command
  resumes where you left it. This also removes a trap: without it, latching at B7 and then merely
  clicking B3 to browse would rewrite the saved stage to B3.
- `toggle_unbuilt` ([:343-347](scripts/rs_sequence_edit.py#L343-L347)): hide →
  `set_build_stage(active_bar_id, seq)`; show → `clear_build_stage()`.
- `set_active` ([:153](scripts/rs_sequence_edit.py#L153)): while `show_unbuilt is False`, update the
  saved stage — the assembly stage *is* the active step, so Next/Previous/typed-step keep the saved
  stage in sync with the screen.
- `do_edit_supports` ([:241](scripts/rs_sequence_edit.py#L241)) already forces everything visible at
  [:273](scripts/rs_sequence_edit.py#L273) and restores at
  [:338-339](scripts/rs_sequence_edit.py#L338-L339); it must **not** touch the latch.
- `_print_status` ([:141-148](scripts/rs_sequence_edit.py#L141-L148)): say
  `hidden (persists after exit)`. Update the module docstring
  ([:22-26](scripts/rs_sequence_edit.py#L22-L26)) and the exit message
  ([:494](scripts/rs_sequence_edit.py#L494)) — "Display restored" is untrue when latched.

## 6. `scripts/rs_show_bar_action_plan.py` — toggle only

Its `show_unbuilt` defaults to **False** ([:301](scripts/rs_show_bar_action_plan.py#L301)) and
`refresh()` re-asserts it on every bar switch and pose cycle
([:474](scripts/rs_show_bar_action_plan.py#L474)) with no user involvement. If either wrote the
latch, merely opening the IK plan viewer would hide the model permanently. **Change only
`toggle_unbuilt` ([:358-361](scripts/rs_show_bar_action_plan.py#L358-L361))**; leave `:301` and
`:474` untouched. `cleanup()` needs nothing — it goes through `reset_sequence_colors`.

## 7. `scripts/rs_update_preview.py`

- `_run_update_preview()`: `apply_build_stage_visibility()` immediately before `rs.Redraw()`
  ([:150](scripts/rs_update_preview.py#L150)), after the tool-restore/side/re-snap passes.
- `show_colors_preview()` branch ([:169](scripts/rs_update_preview.py#L169)): the filter stays on.
  `find_broken_links()` already scans hidden objects, so the *report* is complete. What changes is
  `mark_broken_links` ([rhino_joint_refresh.py:387-420](scripts/core/rhino_joint_refresh.py#L387-L420)):
  skip `rs.AddTextDot`, colour marking and `rs.SelectObjects` for objects where
  `rs.IsObjectHidden` is true, and print a trailing count — `N of the M broken links are hidden by
  the stage filter — listed above, not marked or selected.` Its legend at
  [:469](scripts/core/rhino_joint_refresh.py#L469) currently claims "everything listed below is
  SELECTED" and needs the same qualifier. Re-apply the filter after the branch.

## 8. `scripts/rs_swap_robotic_tool.py`

`apply_build_stage_visibility()` after the re-place summary print
(~[:212](scripts/rs_swap_robotic_tool.py#L212)), i.e. after `replace_all_tool_instances`
([:205](scripts/rs_swap_robotic_tool.py#L205)) has deleted and re-inserted every tool block. Put it
*after* the call returns, not inside, so it doesn't break that function's `suspend_redraw` batching.

## 9. Creation chokepoints

- `ensure_bar_preview` ([rhino_bar_registry.py:906-920](scripts/core/rhino_bar_registry.py#L906-L920)):
  hide the new tube if its bar is beyond the stage. Needed because RSCreateBar
  ([rs_create_bar.py:38](scripts/rs_create_bar.py#L38)) and RSBarEdit call it outside any repair
  pass — otherwise you get a **visible tube floating over a hidden centerline**, and the tube is
  what you actually see, so the filter looks broken.
- `place_tool_at_block_instance` ([rhino_tool_place.py:243](scripts/core/rhino_tool_place.py#L243)):
  one map lookup, covers `restore_missing_tools_at_joints`, `enforce_bar_tool_sides`,
  `resync_tools_to_joints` and `replace_all_tool_instances` in a single place.
- **Not** `place_joint_blocks` / `place_ground_block`: a joint is only created between two bars you
  just picked, and picking requires visibility, so the parent bar is visible by construction.

## 10. `scripts/rs_reorder_bar_id.py`

Remap the saved bar id through the existing rename map, and add the key to the storage-location list
in the docstring at [:35-43](scripts/rs_reorder_bar_id.py#L35-L43).

## 11. Warnings where work would silently vanish

- `rs_create_bar.py` (~[:40-44](scripts/rs_create_bar.py#L40-L44)): print
  `RSCreateBar (stage filter): B12 is later than the current stage B7 and will be HIDDEN from the
  next command onward.`
- `rs_import_scaffold_json.py` ([:222+](scripts/rs_import_scaffold_json.py#L222)): bulk-creates bars
  and joints outside any pick, so it needs an end-of-command apply.

## 12. Docs

- [docs/rhino_toolbar_entrypoints.md](docs/rhino_toolbar_entrypoints.md) — RSSequenceEdit section:
  HideUnbuilt is now a persistent document filter, cleared only by ShowUnbuilt, and it changes what
  other commands can pick.
- [docs/Su_note.md](docs/Su_note.md) — the three glossary points from the top of this plan
  (`sc.sticky` vs document user text; why the stage stores id + step; why tools resolve through the
  male/ground half), labelled convention vs. real Rhino/Python API.
- Docstrings that become wrong if left alone:
  [rs_sequence_edit.py:22-26](scripts/rs_sequence_edit.py#L22-L26),
  [rhino_bar_registry.py:693-713](scripts/core/rhino_bar_registry.py#L693-L713) and
  [:773-775](scripts/core/rhino_bar_registry.py#L773-L775),
  [rs_reorder_bar_id.py:35-43](scripts/rs_reorder_bar_id.py#L35-L43).

---

## Behaviour after the change (latch on, stage = B7)

| Action | Result |
| --- | --- |
| Esc out of RSSequenceEdit | colours reset; B8+ stay hidden; built bars' tools reappear |
| RSUpdatePreview (repair) | tubes/joints/tools repaired, then re-hidden |
| RSSwapRoboticTool | tools swapped; unbuilt bars' tools re-hidden, built bars' visible |
| RSUpdatePreview > ShowColorsPreview | full report incl. hidden links; only visible ones marked/selected; filter stays on |
| RSIKKeyframe | runs normally; on exit B8+ hidden again |
| RSJointPlace / RSBarSnap | reminder printed; unbuilt bars unpickable and un-snappable |
| RSCreateBar | new bar warned about, visible during the command, hidden from the next one |
| Save + reopen the `.3dm` | still hidden (object state and doc key both persist) |
| RSReorderBarID renumber | stage still points at the same physical bar |
| Stage bar deleted | falls back to the nearest earlier step and says so |
| RSSequenceEdit > ShowUnbuilt | latch cleared, everything revealed |

## Verification

`tests/conftest.py` limits the suite to pure geometry/solver logic; anything touching
`rhinoscriptsyntax` is verified by hand through the toolbar. The only genuinely pure logic here is
`resolve_build_stage_seq(bar_map)` — extract it as a pure function over a `{bar_id: (oid, seq)}`
dict and add one headless test using the `sys.modules` `SimpleNamespace` stub pattern from
[tests/test_rhino_tool_place.py:18-21](tests/test_rhino_tool_place.py#L18-L21). Everything else is
manual, in a document with ≥5 bars carrying joints and tools:

**Persistence**
1. RSSequenceEdit → active B3 → HideUnbuilt → **Esc**. B4..N stay hidden. *(core requirement)*
2. Save, close, reopen the `.3dm`. Still hidden; `Document Properties → User Text` shows
   `scaffolding.build_stage = B3|3`.
3. RSUpdatePreview, RSSwapRoboticTool, RSJointPlace (cancel), RSBarSnap (cancel), RSIKKeyframe (Esc)
   — each prints the reminder and leaves B4..N hidden.
4. RSSequenceEdit → ShowUnbuilt → Esc. Everything visible, doc key gone. Run RSUpdatePreview —
   nothing is re-hidden.

**Per-object correctness**
5. B4's tube **and** centerline both hidden — no ghost tube.
6. Joint between built B2 and unbuilt B5: female half (parent B2) visible, male half hidden.
7. Tools on B1..B3 visible; tool on B5 hidden. Check a joint whose two bars straddle the stage to
   confirm the tool→bar binding used the **male** side.

**Refresh commands**
8. RSSwapRoboticTool with the latch on: re-placed tools on unbuilt bars must not reappear.
9. ShowColorsPreview: broken links on hidden bars are **listed** with the hidden count and not
   marked/selected; visible ones are marked and selected as before.

**Destructive / renaming**
10. Stage B7 → RSReorderBarID renumber → stage still points at the same physical bar.
11. Stage B7 → RSRemoveBar on B7 → falls back to the nearest earlier step and prints why.
12. Stage B7 → RSCreateBar → warning printed; new bar visible during the command, hidden after.

**Interaction hazards**
13. Stage B3 → RSJointPlace: B5 cannot be picked, and the printed reminder says how to fix it.
    *(acceptance test for the strict-reach trade-off)*
14. Stage B3 → RSSequenceEdit → EditSupports: all bars pickable inside the picker; on exit the stage
    view returns and the latch is intact.
15. Stage B3 → RSIKKeyframe on B2: at exit nothing leaks — B4..N still hidden.
16. `Ctrl+Z` right after a latched command, then run any command — visibility and the doc key
    self-heal on the next apply.

**Performance**
17. Latch **off**: RSJointPlace startup indistinguishable from before (one string read).
18. Latch **on**, ~100 bars: RSUpdatePreview shows no visible regression (this is what step 1 buys).
