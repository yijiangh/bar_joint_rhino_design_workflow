# Su's learning notes

Personal notes on Python + this codebase, written while building the multi-bar IK
command (`rs_ik_keyframe_all.py`). Each section answers a question I had.

---

## 1. How do we know a bar already has IK? Where are IK results saved?

IK results are **saved on the bar curve itself, as Rhino UserText** (little
key → string pairs attached to an object; they persist inside the `.3dm` file).

When a bar is solved, the single-bar command writes four UserText keys onto the bar
centerline curve (see `rs_ik_keyframe._write_assembly_base_frame` /
`_write_assembly_keyframes`):

| UserText key (`core.config`)      | what it stores                         |
|-----------------------------------|----------------------------------------|
| `KEY_ASSEMBLY_BASE_FRAME`         | robot base frame, 4×4 matrix (JSON)    |
| `KEY_ASSEMBLY_IK_APPROACH`        | approach-pose joint config (JSON)      |
| `KEY_ASSEMBLY_IK_ASSEMBLED`       | assembled-pose joint config (JSON)     |
| `KEY_ASSEMBLY_IK_RETREAT`         | retreat-pose joint config (JSON, opt.) |

The values are JSON strings (`json.dumps(...)`). To read one back:
`rs.GetUserText(bar_oid, key)` → string, then `json.loads(...)`.

**The single source of truth** for "is this bar solved?" is
`core.bar_action.has_ik_keyframe(bar_oid)` → `True`/`False`. Internally it calls
`_read_bar_keyframe`, which returns the parsed data only when base + approach +
assembled are all present and valid. So in our command we just call
`ikf.bar_action.has_ik_keyframe(curve)` — we never re-read the raw keys ourselves.

> Rule of thumb: don't re-implement a check that already has a named function. Reuse
> `has_ik_keyframe`.

---

## 2. What do `oid`, `rwg`, `ctx` mean? (naming)

These are just variable names — Python doesn't care what they're called; they're
conventions to keep code short.

- **`oid`** = *object id*. A Rhino object's GUID (unique id of one object in the
  document, e.g. a bar curve or a block). Almost every `rhinoscriptsyntax` function
  takes or returns an oid. The codebase uses `oid` everywhere for this.
- **`rwg`** = my short alias for the `rhino_walkable_ground` **module** after I reload
  it: `rwg = importlib.reload(_rwg_module)`. Then `rwg.default_base_frame_for_bar(...)`
  = "call `default_base_frame_for_bar` from that module."
- **`ctx`** = *context*. A tuple I use to carry everything about ONE bar between
  functions: `(bar_id, curve, base_frame, left_tool_oid, right_tool_oid)`. Instead of
  passing 5 separate arguments around, I bundle them and unpack with
  `bar_id, curve, base_frame, left_tool_oid, right_tool_oid = ctx`.

---

## 3. Reading `origin_r = ikf._block_instance_xform_mm(right[0])[:3, 3]`

Step by step:

- `right` is a tuple `(male_joint_oid, tool_oid)` that came from
  `ikf._resolve_arm_tools_on_bar(curve)`. So `right[0]` = the **male joint's oid**
  (first item of the tuple), `right[1]` = the tool's oid.
- `ikf._block_instance_xform_mm(right[0])` returns a **4×4 NumPy matrix** = that
  joint block's position + orientation in the world, in millimetres.
- A 4×4 transform matrix looks like this — the last column is the XYZ position:

  ```
  [ r r r  X ]
  [ r r r  Y ]      (r = rotation, X/Y/Z = translation/position)
  [ r r r  Z ]
  [ 0 0 0  1 ]
  ```
- `[:3, 3]` is **NumPy slicing**: rows `0..2` (`:3`) of column `3` = `[X, Y, Z]` =
  the joint's position (a 3-number array). So `origin_r` = where the right joint is.

Then:
```python
return 0.5 * (np.asarray(origin_l) + np.asarray(origin_r))
```
= the **midpoint** of the two joint positions = the center between the two joints the
robot grabs. (`np.asarray(...)` just guarantees the values are NumPy arrays so `+`
and `*` do element-wise math on the XYZ.)

**Why can it find the male joint?** It doesn't search — `_resolve_arm_tools_on_bar`
already did the searching. That function scans the joint layers for blocks whose
`parent_bar_id` UserText equals this bar's id, checks there are exactly two with L/R
tools, and hands their oids back in the `left`/`right` tuples. We just read `[0]`.

---

## 4. When do we use `return`? When/why `= None`?

**`return`** ends a function and (optionally) hands a value back to the caller.
```python
def _place_base(...):
    if err is not None:
        return None, "not IK-ready"     # stop early, report failure
    ...
    return (bar_id, curve, ...), None   # success: hand back the context
```
- A `return` with a value gives that value to whoever called the function.
- A bare `return` (or reaching the end of the function) returns `None` automatically.
- Early `return` is used to stop as soon as we know the answer ("guard clauses"):
  `if not bars: return` = "nothing selected → stop the command."

**`= None`** shows up in two ways:

1. **Default parameter value:** `def f(grounds_map=None):` lets the caller omit that
   argument. Inside, we fill in a default:
   ```python
   if grounds_map is None:
       grounds_map = get_all_walkable_grounds()
   ```
   `None` here means "the caller didn't give one."
2. **A sentinel for "nothing / failure":** many functions `return None` to say "no
   result." E.g. `default_base_frame_for_bar` returns `None` when the bar has no
   ground. The caller checks `if base_frame is None:` and skips that bar.

`None` is Python's built-in "absence of a value." Compare with `is None` /
`is not None` (not `== None`).

---

## 5. `try` / `except`

This is how Python handles errors **without crashing** the whole command.

```python
try:
    movements, _env = ikf.bar_action.build_assembly_movements(...)
except (RuntimeError, ValueError) as exc:
    print(f"could not build movements ({exc})")
    return False
```

- Code inside `try:` runs normally.
- If it raises an error (an *exception*), Python **jumps** to the matching `except`
  block instead of stopping the program.
- `(RuntimeError, ValueError)` = the specific error types we expect and want to
  handle. Other error types would NOT be caught here (they'd still propagate).
- `as exc` names the error object so we can print it.
- Here, on failure we print a message and `return False` (this bar failed) — the
  batch keeps going to the next bar.

(Note: the user wrote "expect" — the keyword is spelled **`except`**.) A bare
`except Exception:` catches *everything*; use it sparingly, only when any failure
should be swallowed.

---

## 6. `if __name__ == "__main__": main()`

Every Python file automatically has a variable `__name__`:

- When the file is **run directly** (Rhino runs `rs_ik_keyframe_all.py`), Python sets
  `__name__ == "__main__"`.
- When the file is **imported** by another file
  (`import rs_ik_keyframe_all`), `__name__` is the module name
  (`"rs_ik_keyframe_all"`), **not** `"__main__"`.

So:
```python
if __name__ == "__main__":
    main()
```
means **"only run `main()` when this file is executed directly, not when it's
imported."** That's why another script can `import` this file to reuse its helper
functions (like `_place_base`) without accidentally launching the whole command —
exactly how `rs_ik_keyframe_all.py` imports `rs_ik_keyframe` to reuse its helpers.

---

## 7. Reuse habit (the recurring lesson)

Before writing new code, look for an existing function/button that already does it:
- IK solved-check → `has_ik_keyframe`
- base placement geometry → `default_base_frame_for_bar` (+ `derive_seed_base`)
- picking bars → `bar_or_tube_filter` + `resolve_picked_to_bar_curve`
- the right-click "…All" script imports the left-click script and calls its helpers.

Extend an existing function with a new optional parameter (like adding
`center_mm=None` / `standoff_mm=None` to `default_base_frame_for_bar`) instead of
copy-pasting a near-duplicate.

---

## 8. What is `scaffolding_toolbar.rui`? How do I read / edit it? What's a "macro"?

`scaffolding_toolbar.rui` is Rhino's **toolbar file**. Despite the `.rui`
extension it's just **XML** (plain text), so you can open and edit it in VS Code
like any other file — no special tool needed. Rhino reads it to draw our custom
toolbar buttons and to know which script each button runs.

It has two parts that matter:

### a) `<macros>` — the list of commands

A **macro** = one command definition: a name + tooltip + button label + the line
Rhino actually executes. Each macro is a `<macro_item>` with a stable **`guid`**
(a unique id string). Ours look like this:

```xml
<macro_item guid="a1b2c3d4-2222-4aaa-b222-000000000015">
  <text>          <locale_1033>RSIKKeyframe</locale_1033></text>
  <tooltip>       <locale_1033>Dual-arm IK keyframe: ...</locale_1033></tooltip>
  <button_text>   <locale_1033>RSIKKeyframe</locale_1033></button_text>
  <script>! _-ScriptEditor _R "rs_ik_keyframe.py"</script>
</macro_item>
```

- `<script>` is the command. `! _-ScriptEditor _R "rs_ik_keyframe.py"` tells Rhino
  to run our Python file. (`!` = cancel any running command first, `_-ScriptEditor
  _R` = run a script by filename.) To make a NEW button run a NEW `.py`, you copy a
  macro and change this line + the name.
- `guid` is how buttons find this command (see below). It must be **unique** and
  **stable** — don't reuse another macro's guid.

### b) `<tool_bars>` — the buttons you click

Each button is a `<tool_bar_item>`. A button does **not** contain the command — it
**points at macros by guid**, one for each mouse button:

```xml
<tool_bar_item guid="a1b2c3d4-1111-4aaa-b111-000000000015" ...>
  <left_macro_id>a1b2c3d4-2222-4aaa-b222-000000000015</left_macro_id>   <!-- left-click  -->
  <right_macro_id>a1b2c3d4-2222-4aaa-b222-000000000017</right_macro_id> <!-- right-click -->
</tool_bar_item>
```

So **one button = two commands**: left-click runs macro `...015` (RSIKKeyframe),
right-click runs macro `...017`. This is exactly the "left-click / right-click
companion" pattern used all over our toolbar (e.g. RSClearIKKeyframe /
…All, RSShowBarActionPlan / …Motion).

### c) The gotcha I hit

The IK KeyFrame button already **declared** `<right_macro_id>...017</right_macro_id>`,
but there was **no `<macro_item guid="...017">` defined anywhere** — a dangling
reference (right-click pointed at a command that didn't exist). To put
`rs_ik_keyframe_all.py` on the right-click of IK KeyFrame, I only needed to **add
the missing macro** with guid `...017`:

```xml
<macro_item guid="a1b2c3d4-2222-4aaa-b222-000000000017">
  <text>        <locale_1033>RSIKKeyframeAll</locale_1033></text>
  <tooltip>     <locale_1033>Right-click companion of RSIKKeyframe: ...</locale_1033></tooltip>
  <button_text> <locale_1033>RSIKKeyframeAll</locale_1033></button_text>
  <script>! _-ScriptEditor _R "rs_ik_keyframe_all.py"</script>
</macro_item>
```

No change to the button itself was needed — it was already wired to `...017`.

### How to edit the .rui safely

1. Edit the XML text (add/adjust a `<macro_item>`, or change a
   `<left_macro_id>`/`<right_macro_id>` on a `<tool_bar_item>`).
2. Keep guids unique; a right-click only works if its `right_macro_id` matches a real
   `<macro_item guid=...>`.
3. Rhino loads the `.rui` at startup (or via `Tools > Toolbar Layout`). After editing
   the file, restart Rhino or re-open the workspace so the change shows up. Rhino also
   caches a copy in its AppData — if an edit "doesn't take," make sure Rhino isn't
   overwriting the repo file on exit.

> Rule of thumb: a **macro** is the command; a **tool_bar_item** is the button that
> points at one or two macros (left / right). To add a right-click behavior, add the
> macro and make sure the button's `right_macro_id` points to its guid.

---

## 9. `importlib.reload(module)` vs plain `from ... import ...` — why we reload

Every command's `main()` starts with lines like:

```python
import importlib
from core import rhino_walkable_ground as _rwg_module   # top of file: import ONCE
...
def main():
    rwg = importlib.reload(_rwg_module)                  # re-run the file's source
    base_frame_viz = importlib.reload(_base_frame_viz_module)
```

**Why not just `from core import rhino_walkable_ground` and use it directly?**

- **Python caches modules.** The *first* time any file does `import X`, Python runs
  `X`'s source and stores the result in `sys.modules`. Every later `import X`
  **reuses that cached copy** — it does **not** re-read the file.
- **Rhino keeps one Python process alive for the whole session.** So the modules
  imported the first time you ran a command stay cached in memory. If you then
  **edit** `rhino_walkable_ground.py` and run the command again, Rhino still uses the
  **old** cached version — your edit doesn't show up until you restart Rhino.
- **`importlib.reload(module)` forces Python to re-execute that module's source**,
  replacing the cached copy with your latest edits. That's the whole point: it lets
  us develop the core modules **live inside a running Rhino** without restarting
  after every change.

The two-step shape is deliberate:

1. At the **top of the file**, import the module under a private alias
   (`_rwg_module`) — this just gets a handle to it.
2. Inside **`main()`**, `rwg = importlib.reload(_rwg_module)` reloads it and rebinds a
   short name (`rwg`) to the **fresh** module. We call `rwg.some_function(...)` so we
   always hit the reloaded version.

**When a plain import is fine:** if a module never changes during a session (or you
don't mind restarting Rhino to pick up edits), `from core import X` and using `X`
directly is simpler. We use `reload` because these files are edited constantly and
Rhino would otherwise run stale code. (`config` is reloaded for the same reason —
so a changed constant like a standoff distance takes effect without a restart.)

> One-liner: **`from ... import` runs a module once and caches it; `importlib.reload`
> re-runs it so live edits take effect without restarting Rhino.**

---

## 10. Reuse win: one color module for the IK preview (show / clear / paint)

The IK preview colors (`COLOR_HAS_IK`, `COLOR_FAILED`) and the show/clear/legend
helpers now live in **one place** — `core.rhino_bar_registry` (the module that
already owns `paint_bar` / `reset_bar_color` / the sequence colors). Three commands
share them instead of each keeping a copy:

- `rs_ik_keyframe_all.py` — paints bars during the multi-bar solve; its
  `_preview_existing_ik()` just calls `show_all_ik_preview()`.
- `rs_update_preview.py` (**left**-click) — calls `clear_ik_preview()` to revert the
  colors.
- `rs_show_ik_preview.py` (**right**-click, new) — calls `show_all_ik_preview()` +
  `print_ik_preview_legend()`.

The lesson (again): I first tried to make a **new** `core/ik_preview.py` module.
Better to put shared helpers in the **existing** module that already owns the same
kind of logic — no new file, one import site, nothing to keep in sync.

**Persisted vs transient state:** `show_all_ik_preview` can only re-show
`COLOR_HAS_IK`, because "has a solved keyframe" is **saved on the bar** (user-text,
see §1) and readable back via `has_ik_keyframe`. `COLOR_FAILED` is **transient** —
it's only painted live during a solve run and never stored — so it cannot be
reconstructed later. A color you can re-derive must come from persisted state.

**Base circle (static preview):** `core.base_frame_viz.draw_base_frame` grew an
optional `circle_radius_mm=` argument. When set it bakes a reach circle in the base
XY plane; when omitted (the default) existing callers draw exactly as before.
`rs_ik_keyframe_all` passes `config.IK_BASE_SAMPLE_RADIUS` so each auto-placed base
shows its circle — non-interactive (you can't drag it, unlike single-bar
RSIKKeyframe). Adding an **optional** parameter (default = old behavior) is the safe
way to extend a shared function without touching its other callers (same trick as
`center_mm=` / `standoff_mm=` in §7).
