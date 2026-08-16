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

---

## 11. The `M_<target>_from_<source>` naming system

> ⚠️ **This section is about NAMING ONLY — it is not Python knowledge.** Nothing here
> is enforced by the language. It's a house style we follow so that a variable's name
> tells you what you're allowed to multiply it by. Python would happily let you write
> `M_screw_from_block @ M_block_from_bar` in the wrong order; the naming is what stops
> *you* from doing it.

Every constant 4×4 in this codebase is named `M_<target>_from_<source>`. Read it
right-to-left: **"takes something expressed in the SOURCE frame and gives it back
expressed in the TARGET frame."**

| name | source frame | target frame |
|------|--------------|--------------|
| `M_block_from_bar`   | the bar axis frame | the joint block's frame |
| `M_screw_from_block` | the block's frame  | the screw-axis frame    |
| `M_tcp_from_block`   | the tool block's frame | the tool's TCP frame |
| `M_tool_from_block`  | the ground block's frame | the frame the tool attaches at |

### The one trick this naming buys you: adjacent names must touch

When you chain transforms, the **inner** word of one name has to match the **outer**
word of the next. Line them up and the matching words cancel:

```
bar_frame @ M_block_from_bar @ M_screw_from_block
             ^^^^^      ^^^      ^^^^^      ^^^^^
             block <--- bar      screw <--- block
                          ↑                   ↑
                        matches "bar_frame"   matches "block" above
```

Result: a screw frame in world coordinates. If the words *don't* line up
(`M_screw_from_block @ M_block_from_bar` — "block" meets "screw"), the multiplication
is wrong, and the name is what told you, not an error message. NumPy will silently
compute a meaningless matrix.

### Two more conventions that come with it

- **`inv(...)` flips the name.** `inv(M_tcp_from_block)` *is* `M_block_from_tcp` —
  it just isn't stored under that name. That's why placement reads
  `world_tool = attach_frame @ inv(M_tcp_from_block)`: we know where the TCP must
  end up, and we're solving backwards for where the block goes.
- **Post-multiply = act in the LOCAL frame; pre-multiply = act in the parent frame.**
  `block_world @ M_tool_from_block` rotates about the *block's own* axes.
  `M_tool_from_block @ block_world` would rotate about world axes. Same two matrices,
  completely different result — order is the whole meaning.

### Other name fragments worth knowing

- **`_mm` suffix** — translations are in millimetres. A frame with no suffix in a
  Rhino command is usually in **document units**, and mixing the two is the most
  common bug in this codebase. `picks.frame_to_mm(frame, scale_to_mm)` converts one
  way; dividing by `scale_to_mm` converts back.
- **`_world` / `_doc` / `_local`** — which frame the values are measured in.
  `block_world` = absolute; `tcp_local` = relative to its parent.
- **`M_` prefix specifically means a CONSTANT** baked into the registry, as opposed to
  a live frame computed from the document (`block_frame`, `bar_frame`, `tcp_world`).
  If it starts with `M_`, it came out of `joint_pairs.json` / `robotic_tools.json`
  and does not change as you move things around in Rhino.

---

## 12. What is a linter? What is `# noqa: PLC0415`?

> ⚠️ **Tooling, not Python.** Everything in this section is a *comment* as far as
> Python is concerned. Delete it all and every script still runs identically. It only
> talks to the code-checking tools and to the next human reading the file.

### A linter is a spell-checker for code

A **linter** reads your source file *without running it* and complains about things
that are legal Python but probably wrong or messy: an unused import, a variable
that's assigned but never read, a line that's too long, a bare `except:`. This repo
uses **Ruff** (the common modern one; `flake8` and `pylint` are older equivalents).

Each complaint has a short **rule code** so you can talk about it precisely:

| code | what it means |
|------|---------------|
| `F401` | imported something but never used it |
| `E501` | line too long |
| `PLC0415` | `import` is not at the top of the file |
| `PLC0415` ← the one all over our `core/` modules | |

### `# noqa` = "don't flag this line"

`noqa` is short for **"no QA"** (no quality-assurance warning). Written bare it
silences everything on that line; written with a code it silences exactly that rule:

```python
import rhinoscriptsyntax as rs   # noqa: PLC0415
#                                  ^^^^ ^^^^^^^^
#                             "skip it"  "...but only the not-at-top rule"
```

**Always use the specific code.** A bare `# noqa` hides future, unrelated problems on
that line too.

### Why *our* code is full of `# noqa: PLC0415`

Normally every `import` goes at the top of the file. Our `core/` modules deliberately
break that rule and import **inside** the functions:

```python
def remove_tool_for_joint(joint_id):
    import rhinoscriptsyntax as rs   # noqa: PLC0415
```

Because `rhinoscriptsyntax` **only exists inside Rhino**. If it were imported at the
top, then simply *importing* `core.rhino_tool_place` outside Rhino — which is exactly
what `pytest` does — would crash instantly with `ModuleNotFoundError`. Putting the
import inside the function means the module loads fine everywhere and only the
functions that actually need Rhino require Rhino.

So the `# noqa: PLC0415` is us saying: *"yes, this violates the rule; it's on purpose,
stop telling me."* Same idea, different audience, as the `# !` comments scattered
around the codebase — those are notes to humans about a non-obvious decision.

> Rule of thumb: if you add a lazy `import rhinoscriptsyntax` / `import Rhino` /
> `import scriptcontext` inside a function in `core/`, copy the `# noqa: PLC0415` too,
> and keep the module importable without Rhino — otherwise the tests stop running.

---

## 13. What is NumPy, and which bits of it does this repo actually use?

`import numpy as np` appears in **44 of the ~91 Python files** here, so it is worth
knowing properly. This section is the short tour.

### What NumPy is

NumPy gives Python one new thing: the **array** (`np.ndarray`) — a fixed-size grid of
numbers, all the same type. Everything else follows from that.

A plain Python list and a NumPy array look similar but behave completely differently:

```python
[1, 2, 3] + [10, 20, 30]                  # -> [1, 2, 3, 10, 20, 30]   (glued together!)
np.array([1, 2, 3]) + np.array([10, 20, 30])   # -> array([11, 22, 33])  (added, item by item)

[1, 2, 3] * 2                             # -> [1, 2, 3, 1, 2, 3]      (repeated!)
np.array([1, 2, 3]) * 2                   # -> array([2, 4, 6])        (each item doubled)
```

That item-by-item behaviour is the whole point. It means a 3D point is just an array of
3 numbers and you can write the maths the way you'd write it on paper:

```python
midpoint = 0.5 * (start + end)            # both are 3-number arrays -> so is midpoint
direction = end - start
```

Compare the pure-Python version, which is what we'd be writing everywhere without it:

```python
midpoint = [0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2])]
```

### Why *this* repo needs it

Two reasons, both structural:

1. **Everything geometric here is a 3-vector or a 4×4 matrix.** Points, directions,
   joint frames, robot base frames (see §3 and §11). Those are exactly what arrays are
   for, and matrix multiply (`@`) is what chains frames together.
2. **It is Rhino-free.** Rhino has its own `Point3d` / `Transform` / `Vector3d` types,
   but those only exist *inside Rhino*. NumPy works anywhere — headless planner,
   `pytest`, a plain terminal. So the rule in this codebase is: convert Rhino types to
   NumPy at the boundary, do all the maths in NumPy, convert back only to draw. (This
   is the same instinct as §12's lazy imports and §13's triangle soups.)

### Where the boundary is

```python
pt = rs.CurveStartPoint(oid)                              # Rhino Point3d (doc units)
start_mm = np.array([pt.X, pt.Y, pt.Z]) * scale_to_mm     # -> NumPy, millimetres
   ...all the maths happens here...
rs.AddLine(rs.CreatePoint(*(result_mm * scale_from_mm)))  # -> back to Rhino, to draw
```

Note the `_mm` suffix (§11): crossing that boundary is also where document units become
millimetres, and mixing the two up is the most common bug in this repo.

### The parts we actually use

Measured by grepping `scripts/`, in order of how often they appear:

| What | Times used | What it does |
|------|-----------:|--------------|
| `np.ndarray` | 227 | just the *name* of the type, in docstrings + type hints |
| `np.asarray(x, dtype=float)` | 133 | "make sure this is a float array" — see below |
| `np.linalg.norm(v)` | 76 | **length** of a vector. The workhorse |
| `np.array([...])` | 70 | build a new array from a list |
| `np.dot(a, b)` | 52 | dot product → "how much does a point along b" |
| `np.cross(a, b)` | 25 | cross product → a vector perpendicular to both |
| `np.eye(4)` | 19 | the 4×4 identity matrix ("no transform") |
| `np.degrees(x)` | 15 | radians → degrees (we store radians, §coordinate_conventions) |
| `np.linalg.inv/det/svd` | 8 | matrix inverse / determinant / decomposition |
| `np.clip(x, lo, hi)` | 4 | clamp a number into a range |

Plus the two operators that never show up in a grep:

- `A @ B` — **matrix multiply**. This is how frames chain (`M_world_from_bar @
  M_bar_from_block`, see §11). Do not use `*`, which multiplies item-by-item.
- `a[:3, 3]` — **slicing**, e.g. pulling the position out of a 4×4 (§3).

### The four idioms you'll see constantly here

**1. `np.asarray(x, dtype=float)` — the defensive front door.**

```python
v = np.asarray(vector, dtype=float)
```

Nearly every geometry function in `core/` starts with a line like this. Three reasons:

- **It accepts whatever the caller has** — a list, a tuple, a Rhino point unpacked into
  a list, or an array that is already an array. Without it, every function would have to
  document and check one exact input type.
- **`dtype=float` prevents silent integer truncation.** If someone passes `[1, 2, 3]`,
  NumPy infers an *integer* array, and then an innocent-looking line misbehaves:

  ```python
  np.asarray([1, 2, 3]) / 2               # -> [0.5, 1. , 1.5]   fine
  v = np.asarray([1, 2, 3]); v /= 2       # -> error / truncation: it's an int array
  np.asarray([1, 2, 3], dtype=float) / 2  # -> [0.5, 1. , 1.5]   always safe
  ```
  In millimetre geometry, silently rounding 0.5 mm down to 0 is exactly the kind of bug
  you never find.
- **It is free on the common path.** See the table below — `asarray` does not copy an
  array that is already the right type, so the safety costs nothing.

**`np.asarray` vs `np.array` — the difference is one word: *copying*.**

| | behaviour |
|---|---|
| `np.array(x)` | **always** builds a new array (copies the data) |
| `np.asarray(x)` | converts *only if needed*; if `x` is already a float array, returns **the same object** |

The catch with "no copy": if you then **modify** the array, you are modifying the
caller's array too. So a function that mutates asks for the copy explicitly —
`core/bar_action.py`:

```python
out = np.array(tool0_assembled_mm, dtype=float, copy=True)   # about to be modified
out[:3, 3] = out[:3, 3] + axis_world * lm_distance_mm
```

> Rule: reading only → `np.asarray(x, dtype=float)`. Going to modify it →
> `np.array(x, dtype=float, copy=True)`.

**And `np.ndarray` is not a function at all** — it is just the *name of the type*. It
shows up 227 times because our docstrings and type hints mention it, not because we call
it:

```python
def _bar_axis_mm(bar_oid):
    """...
    Returns:
        np.ndarray | None: the unit direction.     <- documentation, not code
    """
```

**2. Normalising a direction** — turn a vector into a pure direction of length 1:

```python
norm = float(np.linalg.norm(vec))
if norm > 1e-9:            # guard: a zero-length vector has no direction
    unit = vec / norm
```
This exact shape appears dozens of times. It is wrapped up as `transforms.unit(...)`.

**3. `np.dot` to answer "which side / how far along".**

```python
np.dot(offset, left) > 0        # is this joint on the robot's left?  (see arm_side_for_joint)
np.dot(v, normal) * normal      # the part of v pointing straight up
v - np.dot(v, normal) * normal  # ...so THIS is v flattened onto the ground
```
That last line — subtract the vertical part, keep the rest — is the projection trick
used all over the base-heading code.

**4. `np.cross` to get a perpendicular.**

```python
left = np.cross(normal, heading)   # up × forward = left   (arm_side_for_joint again)
```

### One gotcha to know now

Never test arrays with `==` in an `if`:

```python
if a == b:                  # ERROR: "truth value of an array is ambiguous"
if np.allclose(a, b):       # correct: are they equal within floating-point slop?
```
Floating-point maths is never exact, so we compare with a tolerance — that's also why
the tests use `TOL = 1e-9` style constants and `np.testing.assert_allclose`.

> Rule of thumb: Rhino types at the edges, NumPy in the middle. Start every geometry
> function with `np.asarray(x, dtype=float)`, get lengths with `np.linalg.norm`, and
> use `@` (never `*`) for matrices.

---

## 14. What is a "triangle soup"? (`soups`, `_bar_ground_soups`)

`soups` shows up all over the base-placement code and the name explains nothing, so:

### The problem it solves

The **WalkableGround** is a surface you draw in Rhino — the floor the robot is allowed
to drive on. In Rhino it is a **Brep** ("Boundary REPresentation"): a mathematically
smooth surface, described by equations. Rhino knows how to do geometry on Breps, but
**only inside Rhino**.

Our base-placement math needs to answer one question over and over:

> given a point in space, what is the closest point on the floor, and which way is up
> there?

and it needs to answer it **outside Rhino** — in the headless planner, and in `pytest`
(see §12 for why we care so much about running without Rhino). So we cannot use Rhino's
Brep functions. We need the floor in a form plain NumPy can chew on.

### The conversion chain

```
Rhino Brep                the smooth surface you drew
   │  Mesh.CreateFromBrep(...)         "tessellate": approximate the smooth
   ▼                                    surface with lots of flat triangles
Rhino Mesh
   │  brep_to_compas_mesh(...)
   ▼
COMPAS Mesh               triangles + adjacency (which triangle touches which,
   │                       which edges meet at a vertex, ...)
   │  _mesh_to_soup(...)              throw the adjacency away
   ▼
(vertices, triangles)     TWO NumPy arrays  ← this pair is "the soup"
```

### What is actually in a soup

Two arrays. Say the floor is one flat square, cut into two triangles:

```python
vertices = np.array([        # every corner point, in mm  (a 4x3 NumPy array, see §13)
    [   0,    0, 0],         #   index 0
    [1000,    0, 0],         #   index 1
    [1000, 1000, 0],         #   index 2
    [   0, 1000, 0],         #   index 3
])
triangles = np.array([       # each row = three INDEXES into `vertices`
    [0, 1, 2],               #   triangle A: corners 0, 1, 2
    [0, 2, 3],               #   triangle B: corners 0, 2, 3
])
```

That is the whole data structure. `triangles` stores *indexes*, not coordinates, so a
corner shared by several triangles is stored once (here corners 0 and 2 are shared).

### Why "soup"?

It is a standard graphics term for **an unstructured pile of triangles** — the
triangles are all floating in the bowl together, and nothing records which one is next
to which. A COMPAS `Mesh` *does* record that; a soup deliberately does not.

We drop the adjacency **because we never need it**. The only function that reads a soup
is `closest_point_on_meshes`, and it just loops over every triangle, computes the
closest point on that one triangle, and keeps the nearest:

```python
for tri in triangles:                       # no neighbours needed --
    a, b, c = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
    close = _closest_point_on_triangle(p, a, b, c)
    ...keep the nearest so far...
```

Less data, no Rhino, works in a test.

### Why the name is plural

A bar may be assigned up to `config.WALKABLE_ASSOC_MAX_COUNT` (= 2) walkable grounds —
e.g. a floor slab and a deck it can also stand on. So `soups` is a **list**, one soup
per ground the robot may stand on for that bar, and `closest_point_on_meshes` takes the
nearest hit across all of them.

```python
soups = _bar_ground_soups(bar_oid, grounds_map)   # -> [soup_WG0, soup_WG1]
```

### The one performance trap

`Mesh.CreateFromBrep` is the slow step. Calling `_bar_ground_soups` once per bar in a
loop re-tessellates the *same* floor for every bar. That is why it takes an optional
`soup_cache` dict: pass one shared `{}` into the loop and each ground is tessellated
once.

> Rule of thumb: `Brep` = Rhino-only smooth surface. `soup` = `(vertices, triangles)`
> NumPy pair, Rhino-free, cheap to loop over. Convert once, cache it, and keep the math
> on the soup side of the boundary.

---

## 15. Base placement: heading, guide lines, and which arm gets which end

Three new terms show up all over the base-placement code. All three are in
`core/base_guide_geom.py` (pure numpy, so `pytest` covers them).

### "Heading"

The direction the mobile base **faces** (its local +X). The base stands the standoff
distance *against* it, so driving forward carries the bar into the assembly. One vector,
and it decides everything downstream: which side of the bar the robot occupies, where the
guide lines are drawn, and which end of the bar gets the left arm's tool.

It comes from the bar's **anchor joints** — the two joint blocks the robot grabs. Each
block's local +Z is its insertion axis, so the heading is roughly "the average direction
the bar is pushed to mate".

### "Mean resultant length" — the test for whether that average means anything

Averaging directions is not like averaging numbers. Add two opposite unit vectors and you
get zero, not "the middle". This bites us for real, because a male joint's +Z is
*perpendicular* to its bar (see `coordinate_conventions.md` §3): a bar whose two couplers
grab from opposite sides has two insertion axes that cancel.

The mean resultant length measures how much a set of directions agree:

```
mrl = |sum of the unit vectors| / how many there are
```

| mrl | meaning |
|-----|---------|
| `1.0` | they all point exactly the same way |
| `0.5` | they are spread out, but there is a clear average direction |
| `0.0` | they cancel — there is **no** meaningful average |

Below `config.INSERTION_DIR_CANCEL_TOL` (0.30) we refuse to answer and fall back to
picking the more open side of the bar. The old code just returned the sum whenever its
length exceeded `1e-9` — which for two cancelling axes is pure floating-point noise, so
the base landed on a random side. That was the bug.

The same idea, one step earlier: an axis that is nearly *vertical* has a horizontal part
that is also noise, so it is dropped before the averaging (`INSERTION_DIR_MIN_HORIZONTAL`).
**Flatten first, then judge** — the old code flattened after summing, by which point the
sum was already spoiled.

### "Extension line"

Of the five guide lines drawn on the ground, four are copies of the joint line stepped
back from the bar, and the fifth joins their **midpoints**. That fifth one is the
extension line, and it matters because the placed base origin always sits on it:

```
                 |<-375->|<-500->|<-625->|      (offsets, against the heading)

  625   500   375   projection
   |     |     |        |
   |     |     |        |            bar (its two joint centres,
   +-----+-----+--------+             projected down onto the ground)
         ^ extension line through the midpoints
           the base origin sits HERE, at the offset matching the standoff
```

So the default standoff is kept equal to the middle offset (500 mm): the auto-placed base
lands exactly on a line you can see.

### "Robot left" — which end takes the L tool

Not a property of the bar. The robot faces the bar, so:

```
left = ground_normal x heading          (up x forward)
```

and the joint whose offset from the bar centre points that way takes the left tool:

```
   [tool R]
      |
     bar        [robot]        heading = robot -> bar
      |
   [tool L]
```

Move the base to the other side and `heading` negates, so `left` negates, so **both ends
swap**. That is why `enforce_bar_tool_sides` used to refuse to decide this: it had no idea
where the robot would stand. Now that the heading is resolved properly, it can.

> Rule of thumb: anything that asks "which side?" — base, guide lines, tool L/R — is
> answered by the heading, never by the bar's own geometry.

---

## 16. What is a "degree of freedom"? (`jp`, `jr`, and "it's already used up")

A **degree of freedom** (DOF) is one number you are still free to choose. Count them by
asking "how many independent knobs does this thing have?"

A joint block sitting on a bar has exactly **two**:

| Knob | Meaning | Where |
|------|---------|-------|
| `jp` | **j**oint **p**osition — slide along the bar | `translation_transform((0, 0, jp))` |
| `jr` | **j**oint **r**otation — spin about the bar axis | `rotation_about_local_z(jr)` |

You can see them lined up in the FK chain (`core/joint_pair.py`):

```python
block_frame = (bar_frame
               @ translation_transform((0.0, 0.0, jp))    # knob 1: slide
               @ rotation_about_local_z(jr)               # knob 2: spin
               @ half.M_block_from_bar)                   # constant, NOT a knob
```

`M_block_from_bar` is baked into the joint definition, so it is the same for every placement
of that block. Only `jp` and `jr` vary.

### "Used up"

For a **ground** joint the foot has to touch the floor. `ground_placement.auto_jr_y_down`
solves for the `jr` that points the block's local +Y at world down, and it returns **one
number**:

```python
return float(np.arctan2(B, A))     # exactly one answer
```

That is the whole `jr` knob spent. There is no "and also rotate it a bit more for the
gripper" — turning the block further would lift the foot off the ground.

So when the arm needs its TCP at some *other* roll, there is no knob left to express it
with. That is the entire reason `M_tool_from_block` exists: it is a **second constant
frame**, not a third DOF. The block keeps its one forced orientation; the tool gets its own.

> Rule of thumb: if a value is solved for, it is spent. Needing a second independent answer
> means you need a second frame, not a second attempt at the same knob.

---

## 17. Why is everything a 4x4 matrix? (and what `@` does)

Almost every geometric value here is a **4x4 NumPy array**: `M_block_from_bar`,
`M_screw_from_block`, `M_tool_from_block`, base frames, tool frames, block instance
transforms. Here is why one shape covers all of it.

### A 4x4 stores a position AND an orientation together

```
[ r  r  r  X ]     r = rotation (3x3, the top-left block)
[ r  r  r  Y ]     X/Y/Z = position (the last column)
[ r  r  r  Z ]
[ 0  0  0  1 ]     always this, always
```

Read it as a **frame**: "here is a location, and here is which way it faces". The three
rotation columns are the frame's own X, Y and Z axes written in the parent's coordinates:

```python
matrix[:3, 0]   # this frame's X axis
matrix[:3, 1]   # its Y axis
matrix[:3, 2]   # its Z axis   <- e.g. a joint's insertion direction
matrix[:3, 3]   # its origin   <- see §3
```

That's the whole idea. `_bar_axis_mm` reads column 2 to get a direction;
`_block_instance_xform_mm(oid)[:3, 3]` reads column 3 to get a position. Same object,
different slice.

### Why 4x4 and not a 3x3 plus a separate position?

Because of that bottom `[0 0 0 1]` row. It is what lets ONE multiplication apply the
rotation **and** the translation together. With a 3x3 you would write `R @ p + t`
everywhere and eventually get the order wrong. With 4x4 it is just multiply. (Technical
name: *homogeneous coordinates*. Practical effect: one operation instead of two.)

### `@` = matrix multiply = "chain these frames"

```python
return _block_instance_world_xform(block_id) @ tool_attach_offset(block_id)
```

`@` is Python's matrix-multiply operator. **It is not `*`.** `*` on two NumPy arrays
multiplies element by element (§13) and would give nonsense here.

Chaining left to right = walking down the hierarchy:

```python
block_frame = bar_frame @ T_z(jp) @ R_z(jr) @ M_block_from_bar
#             ^world      ^slide    ^spin     ^constant block offset
```

Each `@` says "and then, *in the frame we have so far*, apply this". Order matters —
`A @ B` is not `B @ A` — which is exactly why the `M_<target>_from_<source>` rule in §11
exists: adjacent names must touch.

Two more you will meet:

```python
invert_transform(M)   # the reverse frame: "block from tool" -> "tool from block"
np.eye(4)             # the IDENTITY: change nothing. This is what "no offset" means,
                      # and why M_tool_from_block defaults to it.
```

### `_as_4x4` — the front door for frames

```python
def _as_4x4(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")
    out = np.array(matrix, dtype=float, copy=True)
    out[:3, :3] = orthonormalize_rotation(out[:3, :3])
    out[3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return out
```

Line by line, it is §13's "defensive front door" applied to frames:

1. `np.asarray(..., dtype=float)` — accept a list-of-lists straight out of JSON.
2. Reject a wrong shape **loudly, at load time**, rather than letting it surface as a weird
   placement hours later.
3. `copy=True` because it is about to modify the array (§13 again).
4. **`orthonormalize_rotation`** — force the rotation part to be a *real* rotation. Values
   that round-tripped through JSON, or came from a slightly-scaled Rhino block, drift: axes
   stop being exactly perpendicular or exactly length 1. This snaps them back (via SVD).
   Without it, small errors compound every time you chain another `@`.
5. Force the bottom row to exactly `[0, 0, 0, 1]` — structural, never data.

> Rule of thumb: a 4x4 IS a frame. `[:3, :3]` is its orientation, `[:3, 3]` is where it is,
> `@` chains one onto another, and `np.eye(4)` means "no change".

---

## 18. Decorators: `@dataclass`, `@classmethod` (a different `@`!)

### First: this `@` is NOT the matrix `@`

Same symbol, two unrelated meanings, told apart by **position**:

```python
@dataclass                     # on its OWN line, above a def/class -> DECORATOR
class GroundJointDef:
    ...

frame = world_frame @ offset   # BETWEEN two values -> matrix multiply (§17)
```

Genuinely confusing, and worth knowing up front.

### What a decorator is

A function that takes your class or function and hands back a modified version. `@thing`
above a definition just means "run `thing` on this and use the result". You mostly *consume*
the ones Python and libraries provide rather than writing your own.

### `@dataclass(frozen=True)`

Used on every definition class here (`JointHalfDef`, `GroundJointDef`, `JointPairDef`,
`JointRegistry`). It reads the fields you listed and **writes the boilerplate for you**:

```python
@dataclass(frozen=True)
class GroundJointDef:
    name: str
    block_name: str
    M_tool_from_block: np.ndarray = field(default_factory=lambda: np.eye(4))
```

For free you get `__init__` (so `GroundJointDef(name="T20Ground", ...)` just works),
`__repr__` (printing shows the field values, not `<object at 0x...>`), and `__eq__` (two
with equal fields compare equal).

`frozen=True` also makes it **read-only after construction**: `ground.name = "x"` raises.
That is deliberate — these describe geometry loaded from a registry file, and nothing
downstream should quietly mutate a shared definition.

Two details that follow from `frozen=True`:

- **`field(default_factory=lambda: np.eye(4))`** rather than `= np.eye(4)`. A plain default
  is created **once**, at class-definition time, and shared by every instance — a classic
  bug for anything mutable like an array. `default_factory` says "call this to make a
  *fresh* one per instance".
- **`object.__setattr__(self, "name", value)`** inside `__post_init__`. Normal assignment is
  blocked by `frozen=True`, so validation has to go around the lock to write the cleaned-up
  value back. That is all those lines are doing:

  ```python
  def __post_init__(self):
      tool_from_block = _as_4x4(self.M_tool_from_block)
      tool_from_block[:3, 3] = 0.0                                  # zero the translation
      object.__setattr__(self, "M_tool_from_block", tool_from_block)
  ```

`__post_init__` is the dataclass hook that runs right after `__init__` — the one place to
validate and normalize before the object is handed out.

### `@classmethod`

A normal method receives the *instance* as `self`. A classmethod receives the *class* as
`cls` and is called on the class itself, no instance needed. That is what an alternative
constructor looks like:

```python
@classmethod
def from_dict(cls, data: dict) -> "GroundJointDef":
    return cls(name=str(data["name"]), ...)      # cls IS GroundJointDef here
```

```python
ground = GroundJointDef.from_dict(entry)   # called on the CLASS -- no instance yet
payload = ground.to_dict()                 # normal method: called on an INSTANCE
```

`to_dict` / `from_dict` are the pair that round-trips these definitions through
`joint_pairs.json`. Writing needs an existing object (`self`); reading has to *create* one
(`cls`) — hence one is a normal method and the other is a classmethod.

> Rule of thumb: `@` on its own line above a definition = decorator. `@dataclass` = "write my
> `__init__`/`__repr__`/`__eq__`, and with `frozen=True` make me read-only". `@classmethod` =
> "call this on the class, usually to build an instance from raw data".

---

## 19. What is `pytest`? What is `assert`? What is `*args`?

> ⚠️ **Two of these are real Python, one is a tool.** `assert` and `*args` are language
> features you can use anywhere. `pytest` is a separate program we run from the terminal.
> Same split as §12: the linter reads your code, pytest *runs* it.

### `assert` — "this must be true, or stop"

Plain Python, one keyword, no imports:

```python
assert 2 + 2 == 4       # true  -> does absolutely nothing, execution carries on
assert 2 + 2 == 5       # false -> raises AssertionError and the program stops
```

That is the entire feature. `assert <expression>` evaluates the expression; if it is
truthy nothing happens, if it is falsy you get an `AssertionError`. You can add a message
after a comma, which shows up in the error:

```python
assert seq is not None, f"bar {bar_id} has no sequence number"
```

**The trap worth knowing now:** running Python with the `-O` flag (capital letter O, for
"optimise") deletes every `assert` from the program. So `assert` is only for *"this can
never happen if my code is correct"* checks. Never use it to validate something a **user**
typed or a file contained — that needs a real `if ... raise`, which `-O` cannot delete:

```python
# WRONG - vanishes under python -O, and then bad input sails straight through
assert bar_radius > 0

# RIGHT - a real check that always runs
if bar_radius <= 0:
    raise ValueError(f"bar_radius must be positive, got {bar_radius}")
```

In this repo you'll mostly meet `assert` inside `tests/`, because a test is literally
"compute something, then assert what it should equal".

### `pytest` — the thing that runs the tests

`pytest` is a program, installed alongside numpy and scipy in `scaffolding_env`. You run
it from a terminal (**not** from inside Rhino):

```bash
python -m pytest -q                          # every test in the repo, quiet output
python -m pytest tests/test_geometry.py -v   # one file, verbose (one line per test)
```

Its job is boring and mechanical:

1. look in `tests/` for files named `test_*.py`;
2. inside those, collect every function named `test_*`;
3. call each one with no arguments;
4. if it returns normally → **PASS**. If it raises anything (usually `AssertionError`
   from a failed `assert`) → **FAIL**, and print what happened.

So a test file is not a special language — it is ordinary Python where the function names
happen to start with `test_`:

```python
def test_format_build_stage():
    assert format_build_stage("B7", 7) == "B7|7"
```

The one genuinely clever bit: when an assert fails, pytest rewrites it behind the scenes so
the error shows the **actual values**, not just "it was false":

```
E       assert 'B7|7' == 'B7-7'
E         - B7-7
E         + B7|7
```

Two pytest extras that appear in our tests:

- **`@pytest.mark.parametrize`** — a decorator (§18) that runs the *same* test once per
  input, so a failure names the exact input instead of hiding it inside a loop:

  ```python
  @pytest.mark.parametrize("raw", ["B7", "|", "B7|x", ""])
  def test_parse_rejects_garbage(raw):
      assert parse_build_stage(raw) is None
  ```

  That is **four** tests, reported separately as `test_parse_rejects_garbage[B7]`,
  `...[|]`, and so on.

- **`monkeypatch`** — an object pytest hands you if you name it as a parameter; it
  temporarily replaces something and undoes it after the test. `tests/test_rhino_tool_place.py`
  uses it to fake `rhinoscriptsyntax`, which leads directly to the next point.

### Which of our modules can be tested at all

This is the practical consequence of §12 and the reason `tests/README.md` says the suite
covers "pure geometry and solver logic" only.

**"Headless"** = running Python in a terminal with Rhino not open. Headlessly,
`rhinoscriptsyntax`, `Rhino` and `scriptcontext` do not exist — they are modules that live
*inside* the Rhino process. So:

| module style | example | can pytest reach it? |
|---|---|---|
| no Rhino imports at all | `core/geometry.py`, `core/joint_pair.py`, `core/build_stage.py` | **yes**, just import it |
| Rhino imported *inside* functions (§12) | `core/rhino_tool_place.py` | **yes** for the pure functions |
| Rhino imported at the **top** of the file | `core/rhino_bar_registry.py` | **no** — `import` fails before you reach any function |

That third row is why `core/build_stage.py` exists as its own file. The rule it holds
("given the saved stage and the current bars, which step do we hide past?") is pure
arithmetic with three awkward branches — bar renumbered, bar deleted, nothing resolves.
Left inside `rhino_bar_registry.py` it would have been untestable and verified only by
hand in Rhino; split out, `pytest` checks all three in a second.

> Rule of thumb: when you write logic that is *just* data in → answer out, put it in a
> Rhino-free module and write a test. Keep `rhino_*.py` for the parts that genuinely
> touch the document.

### `*args` — "collect the leftover arguments into a tuple"

A star in front of a **parameter** means "however many positional arguments follow, gather
them into one tuple under this name":

```python
def f(*pairs):
    print(pairs)

f(("B1", 1), ("B2", 2))     # prints: (('B1', 1), ('B2', 2))
f()                          # prints: ()
```

The name `args` is only a convention; `*pairs` above is the same feature. Two stars,
`**kwargs`, does the same for *keyword* arguments and gathers them into a dict. You have
already used a function like this without noticing — `print("a", "b", "c")` takes any
number of arguments because it is defined that way.

A star in front of an **argument at the call site** is the mirror image, "unpack this
sequence into separate arguments":

```python
point = [1.0, 2.0, 3.0]
rs.AddPoint(*point)          # same as rs.AddPoint(1.0, 2.0, 3.0)
```

> Rule of thumb: only reach for `*args` when the count genuinely varies. If a helper always
> takes one list, make the parameter a plain list — `_bar_map(pairs)` called as
> `_bar_map([("B1", 1)])` is clearer than `_bar_map(*pairs)` and costs the reader nothing.

---

## 20. Where does state live? (session vs `sc.sticky` vs document vs object)

This came up building the persistent `HideUnbuilt` filter, where the whole question was
"remember this — but for how long?". There are **four** places to put a remembered value in
a Rhino script, and they are easy to mix up because three of them look like a dict.

| where | how you write it | survives the script ending? | survives closing Rhino? | travels with the `.3dm`? |
|---|---|---|---|---|
| local / session variable | `self.show_unbuilt = False` | ❌ | ❌ | ❌ |
| `scriptcontext.sticky` | `sc.sticky["bar_joint:export_root_path"] = p` | ✅ | ❌ | ❌ |
| **document user text** | `sc.doc.Strings.SetString(key, value)` | ✅ | ✅ | ✅ |
| **object user text** | `rs.SetUserText(oid, "bar_id", "B7")` | ✅ | ✅ | ✅ (with that object) |

### `sc` is `scriptcontext`, Rhino's module — nothing to do with Python

`import scriptcontext as sc` gives you the running Rhino session. `sc.doc` is the open
document. Two different stores hang off it and the names are unhelpfully similar:

- **`sc.sticky`** — a plain dict Rhino keeps alive between script runs, so a command can
  remember something from last time it ran. It is **never written to the file** and it dies
  when Rhino closes. We use it for throwaway conveniences like the last export folder
  (`"bar_joint:export_root_path"` in `rs_export_bar_action.py`).
- **`sc.doc.Strings`** — **document user text**: a `{key: string}` dict serialised *inside
  the .3dm file*. Close Rhino, reopen the file next week, it is still there; email the file
  and it goes along. You can see it in Rhino under **Document Properties → User Text**. This
  is the same store `rs.GetDocumentUserText` / `rs.SetDocumentUserText` reads and writes —
  two APIs, one dictionary.

Our wrappers for it are `get_doc_string` / `set_doc_string` in `core/rhino_helpers.py`. They
add two things worth having everywhere: an empty string reads back as `None` (so
`set_doc_string(key, "")` is a clean "forget this setting"), and they never raise, so a
script that runs with no document open reports `None` instead of crashing.

Document keys currently in use — note they are all **document-wide preferences**, one value
for the whole file:

| key | set by | read by |
|---|---|---|
| `scaffolding.last_joint_pair` | RSBarBrace | `rhino_bar_pick.resolve_default_pair_index` |
| `scaffolding.last_brace_length` | RSBarBrace / RSBarSubfloor | both, to seed the length prompt |
| `scaffolding.last_subfloor_left_pair` / `..._right_pair` | RSBarSubfloor | RSBarSubfloor |
| `scaffolding.last_robotic_tool` | RSSwapRoboticTool | every tool placement |
| `scaffolding.build_stage` | RSSequenceEdit `HideUnbuilt` | every command, via `repair_on_entry` |

### Object user text is the different one

`rs.SetUserText(oid, key, value)` attaches the pair to **one object** (§1 uses it for the IK
results on a bar curve; `bar_id`, `bar_seq`, `joint_id`, `parent_bar_id` all work this way).
It is saved in the `.3dm` like document user text, but it is *per object*: delete the bar and
its `bar_id` goes with it; copy-paste the bar and the copy inherits it — which is exactly the
copy/paste duplicate problem `TUBE_SELF_GUID_KEY` exists to detect.

> Rule of thumb: "which bar is this?" → object user text. "what is this document set to?" →
> document user text. "what did the user click last time?" → `sc.sticky` if losing it is
> harmless, document user text if it is not.

### Why `scaffolding.build_stage` stores `"B7|7"` and not just `"B7"`

Because bar ids are **not permanent**. `RSReorderBarID` renumbers the whole model, so after
a renumber the bar you staged at might be called B4 while a *different* bar is now called
B7. Saving both gives a primary key and a safety net (`core/build_stage.py`):

1. use bar `B7` if it still exists — and use its **current** step, so a renumber is invisible;
2. if it was deleted, fall back to the nearest earlier step (6) so the view barely moves;
3. if neither resolves, switch the filter off, show everything, and print why.

### Which bar does a robotic tool belong to?

A tricky one, because the obvious answer is ambiguous. A joint has two block halves that
share one `joint_id` but store **different** `parent_bar_id` values — the female half stores
the LE bar, the male half the LN bar (`core/joint_placement.py`). A tool block stores only
`joint_id`, no bar at all. So scanning all three joint layers gives two conflicting answers
per tool and whichever is read last wins.

The repo settled this in `rhino_bar_registry.get_active_tool_oids`: a tool belongs to the bar
owning the **male (or ground) half** — physically, the bar the gripper is holding. Any new
code answering "whose tool is this?" must scan male + ground only, and skip the female layer.

---

## 21. "IK failed but InspectCandidates showed no collision" -- reading the failure log

The short version: **`2 hit(s)` in the inspector *is* a collision.** Only a candidate
printed as `CLEAR (no collision)` counts as clean. In the screenshot below every one of
the 64 candidates collided, so there was nothing for the solver to use.

### What the solver actually does, in order

`RSIKKeyframe` does not "try IK once". For each movement (M1 -> M2 -> M3) it runs three
gates in `keyframe/dual_arm_ik.solve_dual_arm_ik_ssik`, and a candidate must pass **all
three**:

| # | Gate | What it means | Failure line printed |
|---|------|---------------|----------------------|
| 1 | **Reachability** | `ssik` (the analytical IK) returns a list of *branches* per arm -- different arm shapes that put the tool on the same target (elbow up / elbow down / wrist flipped...). 0 branches = that arm physically cannot reach. | `... unreachable (ssik returned 0 analytical branches)` |
| 2 | **Collision** | Every left-branch x right-branch pair is posed and collision-checked. Pairs that collide are dropped. | `no collision-free branch pair among N candidate(s)` |
| 3 | **Polish** | Survivors get a gradient IK refinement on the calibrated URDF, then are collision-checked *again*. | `no candidate polished cleanly among N collision-free pair(s)` |

So "64 candidates" = 8 left branches x 8 right branches. The number is a product, which is
why it is always a square-ish number.

### Reading the breakdown from the screenshot

```
[32x] CC.1 between robot link 'base_link' and robot link 'left_ur_arm_upper_arm_link'
[18x] CC.4 between attached rigid body 'joint_J2-3_male' and rigid body 'joint_J2-3_female'
[8x]  CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link'
[6x]  CC.3 between robot link 'right_ur_arm_forearm_link' and rigid body 'bar_B3'
```

32 + 18 + 8 + 6 = **64**. Every candidate is accounted for -> gate 2 killed all of them ->
`M1 IK failed` -> the whole chain fails -> the base sampler moves the robot and tries
again -> `all samples exhausted`.

`CC.x` is compas_fab's collision *category* (`pybullet_check_collision.py`):

| Code | Between | Plain English |
|---|---|---|
| CC.1 | robot link <-> robot link | the robot hit **itself** |
| CC.2 | robot link <-> tool | an arm hit a gripper |
| CC.3 | robot link <-> rigid body | an arm hit a **bar / joint / ground** in the scene |
| CC.4 | attached rigid body <-> rigid body | something the robot is **carrying** hit the scene |
| CC.5 | tool <-> rigid body | a gripper hit the scene |

Now each line means something concrete:

- **`[32x] CC.1 base_link <-> left_ur_arm_upper_arm_link`** -- half of all candidates fold
  the left arm into the robot's own chassis. This is a **base placement** problem: the
  target is so close/awkward that reaching it forces the arm back into the body. You will
  never see this in the viewport -- it happens *inside* the robot.
- **`[8x] CC.1 dual_arm_bulkhead_link <-> ...`** -- same story, against the bulkhead.
- **`[6x] CC.3 right_ur_arm_forearm_link <-> bar_B3`** -- the forearm passes through an
  already-built bar. A real obstacle: the robot must approach from another side.
- **`[18x] CC.4 joint_J2-3_male <-> joint_J2-3_female`** -- the male half the robot is
  **carrying** overlaps the female it is going to mate with. Note this is **M1**, the
  *approach* keyframe, where the two halves are still supposed to be `lm_distance_mm`
  apart. `core/bar_action._apply_movement_touch_policy` whitelists male<->its own female
  **only in M2** (the mating keyframe), on purpose -- so an overlap at M1 means the
  approach offset is too small, or the two blocks really are interpenetrating.

### Why the viewport looks fine when it is not

1. Collision uses the **coarse convex collision meshes** (the `.obj` files), not the pretty
   visual blocks. Two parts with 1-3 mm of visual clearance can overlap in collision mesh.
   That is exactly why `_apply_movement_touch_policy` whitelists tool<->tube.
2. **Self-collisions are hidden inside the robot body.** The most common failure here
   (CC.1) is invisible from outside.
3. The inspector cycles candidates one at a time. Seeing one that looks fine says nothing
   about the other 63 -- read the `[Nx]` counts, they cover all of them.

### Also worth knowing: new geometry blocks IK

Every female half and every stub bar that exists in the document becomes collision
geometry (`core/env_collision.py` registers them as rigid bodies). So placing extra
females/bars -- e.g. with `RSTempPlaceFemaleJoint` -- **can turn a bar that used to solve
into one that fails**. In the other screenshot the carried `joint_J49-51_male` hits
`joint_J8-9_female`: a female belonging to a *different* joint, sitting in the arm's path.

### What to do about it, per dominant reason

| Dominant `CC` line | Meaning | First thing to try |
|---|---|---|
| CC.1 (link <-> link) | arm folds into the robot | `RetryNewBase`; if base sampling keeps failing, raise `IK_BASE_SAMPLE_RADIUS` (1000 mm) / `IK_BASE_SAMPLE_MAX_ITER` (10) in `scripts/core/config.py` |
| CC.3 / CC.5 vs `bar_*` | arm hits built structure | new base, from another side; check the assembly sequence -- should that bar already be up? |
| CC.4 male <-> its own female at M1 | halves overlap before mating | check the approach offset, and whether the female block sits where the design says |
| `0 analytical branches` | target out of reach for that arm | modelling problem: tool block pose or base frame, not a search problem |

> Rule of thumb: read the `[Nx]` numbers first. If they add up to the candidate count, the
> collision gate rejected everything, and no amount of re-running the same base will help.

---

## 22. `git reset --soft`, `HEAD~1`, `ORIG_HEAD` — undoing a commit without losing work

> ⚠️ **Real git behaviour**, not a repo convention. These are built-in commands and built-in
> names; nothing here is something this project made up, and none of it can be renamed away.

This came up right after committing the visualisation files: the commit was made, then the
staged files needed to come back. The fix was `git reset --soft HEAD~1`, and the confusion
afterwards was reasonable — the command *sounds* destructive and the recovery command
*sounds* like it restores files. Neither is quite true.

### Git has three layers, and every command touches a different subset

| layer | what it is | how you see it |
|---|---|---|
| **working tree** | the actual files on disk, what Rhino opens | your editor |
| **staging area** (a.k.a. *index*) | the snapshot you are building up for the next commit | `git status`, left column |
| **branch pointer** (`HEAD`) | which commit your branch currently says it is at | `git log -1` |

`git add` copies working tree → staging area. `git commit` turns the staging area into a new
commit and moves the branch pointer to it. Those are two separate steps, which is why
"undo the commit" and "lose my files" are two separate things.

`git status --short` prints **two columns**, and this is the single most useful thing to know:

```
M  scripts/rs_sequence_edit.py    <- 'M' in column 1: staged (in the next commit)
 M scripts/rs_select_bar.py       <- 'M' in column 2: modified on disk only, NOT staged
MM scripts/rs_bar_edit.py         <- staged, then edited again afterwards
?? tasks/persistent-hide-unbuilt.md  <- untracked; git has never seen this file
```

### `HEAD~1` is just arithmetic on that pointer

`HEAD` = where you are now. `HEAD~1` = one commit before that, `HEAD~2` = two before. So
`HEAD~1` names the commit you want to *land on*, not the one you want to remove.

### The three `reset` modes differ only in how far down they reach

| flag | branch pointer | staging area | files on disk | what you end up with |
|---|---|---|---|---|
| `--soft` | moves back | **untouched** | **untouched** | commit undone, changes still **staged** |
| `--mixed` (the default) | moves back | reset to new HEAD | **untouched** | commit undone, changes **unstaged** but still there |
| `--hard` | moves back | reset | **overwritten** | commit undone, changes **destroyed** |

Only `--hard` can lose work. `--soft` is the "I typed `git commit` too early" undo: it
rewinds the pointer and leaves everything else exactly as it was, so the files that were in
the commit are sitting in the staging area again, ready to be re-committed.

### `ORIG_HEAD` is a bookmark git leaves for you

Before any move that could disorient you — `reset`, `merge`, `rebase` — git writes down where
`HEAD` used to be, under the name `ORIG_HEAD`. After the `--soft` above, `ORIG_HEAD` pointed
at the commit that had just been undone. It is overwritten by the *next* such command, so it
means "one step ago", not "a safe permanent backup".

`git commit -c ORIG_HEAD` therefore does **not** restore files — the files were never gone.
It makes a *new* commit out of whatever is staged right now, pre-filling the message from
that old commit so a long message does not have to be retyped:

| command | what it reuses | opens your editor? |
|---|---|---|
| `git commit -c ORIG_HEAD` | message (and author) of that commit | yes, to edit it |
| `git commit -C ORIG_HEAD` | same | no, uses it as-is |

So the two halves are independent: `--soft` brings the **files** back to the staging area,
`-c ORIG_HEAD` brings the **prose** back.

### Nothing is actually deleted for a while

Even a commit you "removed" is still in the repo, findable with `git reflog` (a log of every
position `HEAD` has held, including ones no branch points at any more) for a couple of weeks.
`git reset --soft <that hash>` puts you back on it.

> Rule of thumb: `--soft` undoes the *commit*, `--mixed` also undoes the *`git add`*, `--hard`
> also undoes the *editing*. If you are unsure which you want, `--soft` is the one that cannot
> lose anything — and check the two columns of `git status --short` before and after.

---

## 23. Which module do I put a preview in? (the four viz mechanisms)

> ⚠️ **Naming, not Python.** Nothing here is enforced by the language. It is a house split
> by *how the preview reaches the screen*, and the module names have drifted away from it,
> so the table is the real map.

I wanted to show a marker during RSJointEdit → MoveJoint and could not tell which module it
belonged in — there are five files with "viz" or "preview" in the name. They are not five
flavours of the same thing. They are **four different mechanisms**, and picking the wrong one
is what makes previews get left behind in the document:

| module | mechanism | lives where | cleaned up by |
|---|---|---|---|
| `core/dynamic_preview.py` | **display conduit** | nowhere — painted per frame by Rhino's display pipeline | setting `Enabled = False` (the `with` block's `finally`) |
| `core/ik_viz.py` | **compas scene objects** | a cached scene, one drawing object per robot/tool/body | rebuilding or clearing the cache |
| `core/base_guide_viz.py`, `core/base_frame_viz.py` | **baked geometry** | real Rhino objects on a dedicated preview layer | deleting the layer's contents wholesale |
| `core/highlight_env.py`, `rhino_bar_registry.paint_bar` | **color override** | no new objects — recolors geometry that already exists | resetting `ObjectColorSource` back to by-layer |

### Why the split matters more than the naming

**A conduit cannot be left behind.** It never becomes a document object, so there is nothing
to delete, no layer to purge, and the `.3dm` is untouched. Escaping the command runs the
`with` block's `finally`, the conduit switches off, and the screen is clean. That is why
MoveJoint's markers are a conduit: the command can be cancelled at four different prompts.

**Baked geometry is the opposite** — it is in the document the moment you draw it, so every
exit path has to remember to clear it, and a crash leaves junk on the preview layer for the
next session to find. Worth it when you need the preview to be *pickable* (the base guides
are — you click one to choose a base position). You cannot click a conduit.

**Color overrides** touch geometry you did not create, so the only thing to restore is the
override itself, and it must be restored from a recorded list of what you touched — see
`highlight_env.highlight_env_for_ik`, which returns a token of every oid it painted.

### The name that lies

`core/dynamic_preview.py` is named after its *first* conduit (`MeshPreviewConduit`, the
half-transparent mesh that follows the cursor). It now holds four, and only that one follows
the cursor. Read the module as "the display conduits", not as "the dynamic preview".

> Rule of thumb: does the preview need to be **clicked**? Bake it on a preview layer. Does it
> only need to be **seen**? Conduit — nothing to clean up. Is the thing already an object in
> the document? Recolor it and keep a list so you can put it back.

---

## 24. `BeginUndoRecord` — making one Ctrl+Z undo a whole command

Before this, no script in `scripts/` opened an undo record, and MoveJoint carried a comment
promising "Ctrl+Z undoes the whole move". It did not. Rhino creates **one undo record per
document operation**, and a single MoveJoint iteration performs about fifteen of them — move
the bar, delete the old tube, add a new one, delete four block instances, insert four, delete
and re-insert two tools. Ctrl+Z peeled off the *last* one and left the model worse than
either end state.

`sc.doc.BeginUndoRecord(name)` opens a bracket; everything until `EndUndoRecord` collapses
into one entry in Rhino's undo stack:

```python
undo = sc.doc.BeginUndoRecord("RSJointEdit MoveJoint")
try:
    ...                       # any number of document edits
finally:
    sc.doc.EndUndoRecord(undo)   # finally: an exception must not leave it open
```

This is **real Rhino API** (`RhinoDoc.BeginUndoRecord`), not a repo convention. Two things I
got wrong first:

**It is not a rollback.** The record only groups edits so the *user* can undo them later. It
does nothing when the user presses Esc mid-command — at that moment the command is still
running and the record is still open. Cancelling has to actively put things back, which is
what `_restore()` in `_run_move_joint` does: transform the bar's endpoints back onto the ones
captured before the first edit, then rebuild the joints from the flags they started with.

**Open it after the cosmetics.** MoveJoint colors bars before it edits anything, and resets
those colors in its `finally`. Color changes are undoable operations too, so opening the
record *before* the painting would fold them in and make Ctrl+Z fight the reset. The record
is opened after the paint and closed before the reset, so it brackets the geometry only.

> Rule of thumb: `BeginUndoRecord` is for *after* the command ends. Cancelling *during* the
> command is your own job — capture what you need to restore before the first edit.
---

## 25. Grasshopper components — what is different from a Rhino command

Everything before this section was about `scripts/rs_*.py`: a file Rhino runs top to
bottom when you click a toolbar button. A **Grasshopper component** is a different
animal, and several of its habits will bite you if you assume otherwise. This came up
building `RSGHSequencePreview` / `RSGHCameraControl` (see
[grasshopper_animation.md](grasshopper_animation.md)).

Labelled as always: **convention** = a name this repo or Grasshopper chose, **real
Python** = language syntax you can look up.

### `ghenv` is injected, not imported — *(Grasshopper convention)*

In a normal script you `import` everything you use. In a GH script component there is a
variable called `ghenv` that you never defined and never imported: Grasshopper puts it in
the script's namespace before running it. `ghenv.Component` is the component itself — the
box on the canvas — and from it you can reach its parameters, its GUID, and the GH
document it lives in.

You will not find `ghenv` in any `import` line, and your editor will flag it as an
undefined name. That is expected; it only exists at runtime inside Grasshopper. This repo
already relies on it in
[GH_init_pb.py:7](../support_materials/gh_keyframe_demos/python/GH_init_pb.py#L7).

### Inputs and outputs are free variables — *(Grasshopper convention)*

A component's inputs arrive as bare variables. If the component has an input named
`enable`, then `enable` simply *exists* when your code runs, with no assignment anywhere.
Outputs work the same way in reverse: assign to a variable named after an output param and
GH reads it back out afterwards.

This is why a GH script looks like it is full of undefined names. It is a real
language-level oddity, not a style choice — the host is injecting them into the module
globals, the same mechanism as `ghenv`.

### Script mode vs SDK mode — *(Grasshopper convention)*

Rhino 8's Python 3 component has two ways of writing the same thing.

| | Script mode (the default) | SDK mode |
| --- | --- | --- |
| shape of the file | bare statements, top to bottom | a class deriving from `Grasshopper.Kernel.GH_ScriptInstance` |
| where inputs come from | free variables, as above | arguments of `def RunScript(self, ...)` |
| creating params | you zoom in on the component and click `+` for each one, then type its nickname | **generated from the signature** |
| type hint | set per-param in the UI | the annotation: `enable: bool` |
| List Access | right-click the param → List Access | inferred from `poses: List[str]` |
| outputs | assign to named variables | `return a, b` — order matters |

You get SDK mode by clicking **Convert To GH_ScriptInstance** on the Script Editor
dashboard. Our two components are written that way, purely so that pasting one file
produces a fully wired component. Four things to know:

- **The argument name becomes the parameter NickName.** `gh_bridge.ensure_int_slider(ghenv,
  "step", ...)` finds the right input only because the argument is called `step`. Rename
  the argument and you silently rename the parameter.
- **Sync goes both ways, and the editor wins.** Editing the signature rebuilds the params;
  adding a param on the canvas rewrites the signature. You do not own that line of code —
  the editor does, and it edits it in place while you watch. Two ways that surprises you,
  both of which cost an afternoon here:
  - it converts Python type hints to **.NET** ones, so `poses: List[str]` comes back as
    `poses: System.Collections.Generic.List[object]`. If the file has no `import System`
    the component then dies with `undefined name 'System'` — so ship the import even
    though nothing you wrote uses it. Items also arrive wrapped (`GH_String` rather than
    `str`), which is why `normalize_poses` unwraps `.Value` before reading them;
  - the rewrite can **dedent `def RunScript` out of the class**, which surfaces as
    `unindent does not match any outer indentation level` pointing straight at the def.
    The file on disk is fine; the editor's copy is not. Re-indent the def to 4 spaces
    under the `class` line and leave the signature text exactly as the editor wrote it.
    Never delete the `class` line — a module-level `def RunScript(self, ...)` is not a
    component.
- **The class is instantiated fresh on every solve.** `self.anything = x` is gone by the
  next solve. Anything that must persist goes in `sc.sticky` (see §20).
- **A Button is not a parameter setting.** SDK mode can generate a boolean input but not
  the Button component that feeds it; wire one yourself.

### An unconnected input is `None`, not your default — *(Grasshopper convention)*

This one looks like a Python question and is not. Given

```python
def run(ghenv, enable=False, step=0, show_unbuilt=True):
```

a Python caller that omits `step` gets `0`. **Grasshopper never omits it.** The host passes
every parameter explicitly, and an unwired input is passed as `None` — so the default in the
signature never fires. The symptom in the smoke test was
`TypeError: unsupported operand type(s) for +: 'NoneType' and 'int'` from a bare `b + 1`.

Left alone this is quietly wrong rather than loud: every unwired boolean reads as falsy, so a
component comes up with its whole legend switched off and nothing explains why. Normalise the
inputs once at the top of the function instead of trusting the signature:

```python
def _flag(value, default):
    return default if value is None else bool(value)
```

The defaults in the signature then serve as documentation, and `_flag(x, True)` is what
actually decides.

### `sc.doc` points at the wrong document — *(Grasshopper convention, and the nastiest one)*

Inside a GH component, `scriptcontext.doc` is the **Grasshopper** document, not the open
`.3dm`. Everything in §20 about `sc.doc` still holds — it just points somewhere else.

So a call like `rs.ObjectColor(oid, color)` or `rs.LayerVisible(name, False)` runs, returns
without error, and changes nothing you can see, because it addressed the GH document.
There is no exception and no warning; the viewport simply does not update, which sends you
looking for a bug in the colour logic that is not there.

The fix is to swap it for the length of your code and swap it back —
`core.gh_bridge.rhino_doc()`:

```python
with gh_bridge.rhino_doc():        # sc.doc = Rhino.RhinoDoc.ActiveDoc
    show_sequence_colors(...)      # now actually paints the model
# sc.doc restored here, even if the body raised
```

`with` is real Python (see the context-manager section earlier) — the restore sits in a
`finally`, so an exception mid-solve cannot leave the GH document pointing at the `.3dm`.

### You may not edit the GH document while it is solving — *(Grasshopper rule)*

`RSGHSequencePreview` creates its own `step` slider. But adding an object to the canvas
means mutating the GH document, and you are being *called by* that document mid-solution.
Doing it directly is illegal and Grasshopper will complain or crash.

The way round it is to hand GH a function to run in a moment, once the current solution is
finished:

```python
ghdoc.ScheduleSolution(1, Grasshopper.Kernel.GH_Document.GH_ScheduleDelegate(_edit))
```

"In 1 millisecond, start a fresh solution, and call `_edit` first." The practical
consequence for the caller is that **the slider does not exist yet when the function
returns** — you get its value on the *next* solve. That is why slider creation is bound to
a button rather than run on every pass.

### Two copies of the same pasted code — *(why `gh_bridge.state()` exists)*

Nothing stops you pasting the same component twice. If both copies wrote their remembered
values to one fixed sticky key — say `"bar_joint:gh:seq_preview"` — they would quietly
corrupt each other:

| shared value | what goes wrong |
| --- | --- |
| recorded layer visibility | A records "centerlines were visible" and hides them. B finds a record already there so records nothing. Disable A → it pops the record and restores. B is now hiding layers with no record of the original state, and disabling B restores nothing. Your layers stay hidden with nothing on the canvas explaining why. |
| button edge (`reload`) | A's press is stored under the key B reads; B's next solve overwrites it with `False`. The press is swallowed, or fires twice. |
| "session opened" flag | Disabling A ends the `ik_viz` session B thinks is still open, and B silently stops updating. |
| render-skip fingerprint | Each overwrites the other's, so the skip never matches and both re-pose the robot on every tick. |

`gh_bridge.state(ghenv)` keys the sticky slot by `ghenv.Component.InstanceGuid` — the GUID
Grasshopper gives each *instance* on the canvas — so each copy gets its own dict.

What that does **not** fix: two simultaneously enabled copies still fight over one Rhino
document (same colours, same layers, one globally cached `ik_viz` bundle), and GH's solve
order decides who wins. Per-component state only guarantees each copy can cleanly undo
what it did. Enable one at a time.

### Why the components re-import themselves every solve — *(real Python)*

```python
importlib.reload(_gh_bridge_module)
module = importlib.reload(_gh_seq_preview_module)
```

`importlib.reload` re-reads a module's source and re-executes it, replacing the functions
in the already-imported module object. Without it, Python's import cache would hold the
first version of `core.gh_seq_preview` for the whole Rhino session, and editing the file
would change nothing until you restarted Rhino. Same trick as `_reload()` at the top of
every `rs_*.py`.

The side effect: **module-level variables reset on every solve.** That is the other half of
why persistent state has to live in `sc.sticky` rather than in a module global.

### `idx // n` and `idx % n` — one slider, two dimensions *(real Python)*

`RSGHSequencePreview` has one integer slider, but a frame is really a pair: *which bar* and
*which of the robot's four movements*. The two are pulled back out with integer division
and remainder — the same arithmetic as reading a two-digit odometer.

With 20 bars and `poses = ["M1", "M4"]`:

```
idx        0        1        2        3        4        5      ...
bar     bar[0]   bar[0]   bar[1]   bar[1]   bar[2]   bar[2]
pose      M1       M4       M1       M4       M1       M4
```

- `idx // len(poses)` — `//` is **floor division**: divide and throw the remainder away.
  It answers "how many complete bars have I passed". `3 // 2 == 1` → `bar[1]`.
- `idx % len(poses)` — `%` is **modulo**, the remainder. It answers "how far into the
  current bar am I". `3 % 2 == 1` → `poses[1]` == `"M4"`.

`%` cycles `0,1,0,1,…` forever, which is exactly the repeating pose row; `//` ticks up once
per full cycle, which is exactly the bar row. The bar is the slow digit, so the ordering is
called **bar-major**.

---

## 26. `git add -p` — committing *part* of a file, and the everyday git commands

> ⚠️ **Real git behaviour**, not a repo convention. Every command here is built into git;
> nothing was invented by this project. Builds on [§22](#22-git-reset---soft-head1-orig_head--undoing-a-commit-without-losing-work),
> which explains the three layers (working tree / staging area / branch pointer) these all move
> things between.

### The problem it solves

One commit should be one idea. But a file often ends up holding **two unrelated edits at once**,
because you were working on both in the same session.

That happened exactly here. `scripts/rs_ik_keyframe.py` had **15 changed regions**:

- **1** of them renamed `LM_DISTANCE` to `LM_APPROACH_DISTANCE` in the docstring — that belongs
  with the config/bar_action commit, because shipping the rename without it leaves a docstring
  naming a constant that no longer exists.
- **14** of them were base-guide, pose-preview and joint-flag work — a completely different topic.

`git add scripts/rs_ik_keyframe.py` stages **the whole file**, so it would drag all 14 into a
commit about renaming a constant. `git add -p` stages the file *piece by piece* instead.

### What a "hunk" is

A **hunk** is one contiguous block of changed lines plus a few unchanged lines above and below
for context. Git splits every diff into hunks automatically and labels each with an `@@` header:

```
@@ -23,7 +23,7 @@ origins are tool0 (the robot flange frame) for IK. The script then:
    Brep face.
 4. Previews the robot via ``core.ik_viz``.
 5. Repeats 3-4 for the approach pose, offset along
-   ``-unit(avg(tool_z_L, tool_z_R)) * LM_DISTANCE``.
+   ``-unit(avg(tool_z_L, tool_z_R)) * LM_APPROACH_DISTANCE``.
 6. On accept, writes ``ik_assembly`` user-text (JSON payload) on the bar
```

Read `@@ -23,7 +23,7 @@` as "7 lines starting at line 23, before and after". Lines starting
with `-` are being removed, `+` added, and a leading space means unchanged context.

### Using it

```bash
git add -p scripts/rs_ik_keyframe.py     # one file
git add -p                               # every changed file, in turn
```

Git shows one hunk and waits. You answer per hunk:

| key | meaning |
|---|---|
| `y` | **yes** — stage this hunk |
| `n` | **no** — skip it (stays in the working tree, just not in this commit) |
| `s` | **split** — chop this hunk into smaller ones, when one hunk covers two ideas |
| `e` | **edit** — hand-edit the hunk, when even `s` cannot separate them |
| `q` | **quit** — stop here; whatever you already answered `y` to stays staged |
| `d` | skip this hunk **and all remaining hunks in this file** |
| `a` | stage this hunk **and all remaining hunks in this file** |
| `?` | print this list |

`n` is not destructive and does not undo anything. It only means "not in *this* commit" — the
change is still on disk, and shows up in the second column of `git status --short` (§22).

**`s` is the one worth remembering.** If a hunk contains both a rename and a real change, `s`
splits it at the blank/context lines. It only works when there is unchanged context between the
two parts; when there is not, `e` drops you into an editor where you delete the `+` lines you do
not want and turn the `-` lines you do not want back into context lines (replace the leading `-`
with a space).

### The same `-p` works on other commands

| command | what it does hunk-by-hunk |
|---|---|
| `git add -p` | stage part of your changes |
| `git restore -p` | **throw away** part of your changes (destructive — the only dangerous one) |
| `git restore --staged -p` | unstage part of what you staged |
| `git stash -p` | stash part of your changes, keep the rest |
| `git diff -p` | just show the diff (`-p` is the default here, so you rarely type it) |

### When you cannot use the interactive prompt

`git add -p` needs a live terminal to type `y`/`n` into. Some environments cannot give it one —
an editor's task runner, CI, or an AI agent driving a shell. The non-interactive equivalent is
to write the hunk to a `.patch` file and apply it **to the staging area only**:

```bash
git diff -U3 -- scripts/rs_ik_keyframe.py > full.patch
# keep the 4 header lines (diff/index/---/+++) plus the one @@ hunk you want,
# save as one.patch, then:
git apply --cached one.patch
```

`--cached` means "apply to the staging area, not to the files on disk", so the working tree is
untouched — exactly what `git add -p` + `y` does. Verify with
`git diff --cached -- <file>`: it should show only the hunk you kept.

### The everyday commands, grouped by what you are trying to do

**Where am I?**

| command | answers |
|---|---|
| `git status --short` | what is changed, and is it staged? (the two columns — see §22) |
| `git diff` | what have I changed but **not** staged? |
| `git diff --cached` | what exactly is going into the next commit? |
| `git diff --stat` | same, but just a per-file line count — good for a quick overview |
| `git log --oneline -10` | the last 10 commits, one line each |
| `git branch --show-current` | which branch am I on |

> Habit worth building: run `git diff --cached` **before** every `git commit`. It is the only
> way to see the commit you are actually about to make, rather than the one you think you are.

**Staging**

| command | does |
|---|---|
| `git add <file>` | stage the whole file |
| `git add -p <file>` | stage selected hunks (above) |
| `git restore --staged <file>` | unstage it; the file on disk is untouched |
| `git add -u` | stage every *tracked* file that changed (not new files) |

**Undoing** (see §22 for the commit-level ones)

| command | undoes |
|---|---|
| `git restore <file>` | your edits to that file — **destructive**, the edits are gone |
| `git restore --staged <file>` | the `git add`, not the edits — safe |
| `git reset --soft HEAD~1` | the last commit, keeping everything staged — safe |
| `git stash` / `git stash pop` | park all changes, then bring them back |

**History**

| command | shows |
|---|---|
| `git log --oneline --follow -- <file>` | that file's history, across renames |
| `git log -1 --format='%h %ad %s' --date=short <hash>` | one commit's hash, date and subject |
| `git show <hash>` | the full diff of one commit |
| `git blame <file>` | which commit last touched each line |
| `git reflog` | every position `HEAD` has held — the rescue log (§22) |

> Rule of thumb: `add` and `restore --staged` move things between the **staging area** and the
> **working tree** and can always be redone. Only `restore <file>` (no `--staged`) and
> `reset --hard` touch the files on disk destructively. If a command has `--staged` or `--cached`
> in it, it is not going to lose your edits.

---

## 27. `jp`, `jr`, LE and LN — where exactly is a joint measured from?

> ⚠️ **Repo convention**, not Python. These four names are this project's own; the maths
> behind them is in `core/joint_pair.py`. Related: [§11](#11-the-m_target_from_source-naming-system)
> on `M_<target>_from_<source>`, and [§17](#17-why-is-everything-a-4x4-matrix-and-what--does)
> on why everything is a 4x4.

Everything hangs off the **bar frame**, built by `canonical_bar_frame_from_line(bar_start,
bar_end)` (`core/joint_pair.py:235`):

- **origin** = `bar_start` — *one end of the bar*, not its middle
- **Z** = along the bar, pointing at `bar_end`
- **X** = `orthogonal_to(Z)` — *a* perpendicular, picked deterministically from Z
- **Y** = Z x X

A half is then placed with two numbers, `jp` and `jr` (`fk_half_from_bar_frame`, `:257`)::

    block_frame = bar_frame @ T_z(jp) @ R_z(jr) @ M_block_from_bar
    screw_frame = block_frame @ M_screw_from_block

### `jp` = joint position — a slide ALONG the bar

```
   bar_start                                                   bar_end
   (bar_frame origin)
        *=========================*=========================*
        |<------- jp (mm) ------->|
        |                      station                Z ----->
        |                       point             (along the bar)
```

**From** `bar_start`, **to** the station point, **along** the bar axis.
`jp = 0` is at `bar_start`; `jp = bar length` is at `bar_end`. It can go negative or past the
end — the solver's bounds allow 100 mm of overhang either side by default.

### `jr` = joint rotation — a spin ABOUT the bar

```
   LOOKING DOWN THE BAR, from bar_start toward bar_end

                   Y
                   ^
                   |        / the half, spun by jr
                   |      /
                   |    /
                   |  /  )
          ---------O----+------> X
                   |   jr, measured from X toward Y
                   |
              O = the bar axis, coming at you
```

**Axis of rotation** = the bar's own centre-line. **Measured from** the bar frame's X.

> Trap: X is only *a* perpendicular derived from the bar direction — it means nothing
> physically. So **`jr = 0` is not a meaningful pose**; only *differences* in `jr` are.

### LE and LN — which bar is which

The letters are never spelled out anywhere in this repo (I looked). What is defined is the
mapping:

| name | is | carries |
|---|---|---|
| **LE bar** | `female_parent_bar` user-text | the **female** half |
| **LN bar** | `male_parent_bar` user-text | the **male** half |

```
        LE bar  =  the bar carrying the FEMALE half
        ===========================================
                        | female
                        O  <- screw: both halves must meet here
                        | male
        -------------------------------------------
        LN bar  =  the bar carrying the MALE half
```

### `le_rev` / `ln_rev` — and why flipping MOVES a joint

They do **not** spin a block. They say "measure this bar from its *other* end":

```
le_rev = False :  *---------------->     bar_start here, Z this way
le_rev = True  :     <---------------*   bar_start at the OTHER end, Z reversed
```

Four combinations, numbered `variant_index` 0-3 (`variant_index(le_rev, ln_rev)`).

Reversing changes **three things at once**: the origin jumps to the far end, Z reverses, and
`X = orthogonal_to(Z)` becomes a different vector. The half now hangs off a completely
different frame — it is **mirrored, not rotated**. Its screw ends up on the other side of the
bar axis, and since the two halves must meet at the screw, the whole pair shifts.

Measured on a T20 pair, all four variants mate perfectly (0.000 mm error) but the mate point
sits at either `+30` or `-30` mm across the bar — a **60 mm jump** when you flip. The male's
`jp` did not change at all (975.0 in every variant): it is not sliding along its bar, its
screw is being mirrored across it.

> Rule of thumb: RSJointEdit's FlipJoint moving the pair is **not a bug**. A "flip" selects a
> different one of the four assemblies, and they genuinely mate in different places. Bar
> length and solver bounds make no difference — the 60 mm comes from the joint's own geometry.
> If a stub bar is too short to contain that movement, lengthen the bar, not the bounds.

---

## 28. Parked — understood, measured, NOT built yet

> 🚧 **NOTHING IN THIS SECTION IS IMPLEMENTED.** Both items were worked out and then
> deferred at a deadline. The point of writing them down is so the work can restart from
> here instead of re-deriving it. Each says what is known, what is *not* known, and what to
> learn first.

### a) Flipping a joint is ~50x slower than it needs to be — **NEED TO LEARN**

**Status:** measured, understood, not implemented.

**What happens today.** RSJointEdit's flip re-solves the pair with
`optimize_pair_placement`, which drops **48 starting guesses** and runs a local optimiser
from each, keeping whichever lands best:

- **36 grid seeds** — the two `jp` values fixed at the bars' closest-approach point, and the
  two `jr` angles swept every 60 degrees: 6 x 6 combinations.
- **12 random seeds** — the same `jp` +/- 50 mm, random `jr`. `np.random.default_rng(0)` is
  seeded with 0, so the "random" 12 are *the same 12 every run*.

**Measured** on a T20 pair, flipping the male side:

| approach | mate error | time |
|---|---|---|
| 48 blind seeds (what runs today) | 0.000000 mm | **772 ms** |
| 1 seed, taken from the pose it is already in | 0.000003 mm | **15.6 ms** |

Same answer, 50x faster. Both far inside the 0.05 mm acceptance tolerance.

**The proposed change.** Seed from the current pose instead of from nothing: for the side
being flipped map `jp -> bar_length - jp` and `jr -> -jr` (measure from the other end, spin
the other way), run **one** optimisation, and check `is_variant_acceptable`. If it passes,
done. If not, fall back to today's 48-seed sweep. The acceptance gate is unchanged, so
quality cannot drop; it is only ever faster.

**What to learn first**, in order:

1. What a **local optimiser** is, and why L-BFGS-B finds the bottom of *a* valley rather than
   the lowest one. A ball rolling downhill from wherever you drop it.
2. What a **seed / starting guess** is, and why a problem with several valleys needs more
   than one.
3. Why 48 and not 5 or 500 — read the comment on `_seed_grid`: 60-degree spokes are chosen so
   every valley has a seed within one spoke.
4. Then [§27](#27-jp-jr-le-and-ln--where-exactly-is-a-joint-measured-from) for what `jp` and
   `jr` mean, which is what the seed mapping is doing.

**Known risk.** The `jp -> L - jp, jr -> -jr` mapping is a *heuristic*. It gave the identical
answer in the one case measured, but it is not proven for every geometry — which is exactly
why the fallback is part of the design, not an afterthought.

### b) Four spellings of "pick an option" — **NEED TO DO**

**Status:** agreed, not implemented (deferred at a deadline).

Eight prompts across the repo ask the user to pick an option after a button is clicked. Each
is hand-written, about 20 lines, and there are **four different shapes**:

| shape | how Enter behaves | where |
|---|---|---|
| Enter takes a default | `SetCommandPromptDefault` + `AcceptNothing(True)` | RSBarEdit, RSJointEdit, RSJointPlace, RSSelectBar |
| must choose | `AcceptNothing(False)`, no default line | RSReorderBarID x2, RSTempPlace |
| default set but never *shown* | `AcceptNothing(True)` but **no** `SetCommandPromptDefault` | `rs_ik_keyframe._ask_reuse_saved_base` |
| default, but Esc returns `"cancel"` not `None` | | `rs_ik_keyframe._ask_save_base_or_continue` |

Four spellings of one idea is what makes the pattern hard to copy correctly.

**The proposed change.** One helper, `core/rhino_prompts.ask_option(prompt, options,
default=None)`, where `default=None` means must-choose. Every command keeps its own
`_ask_mode()` wrapper so no call site moves; the wrapper's body shrinks to a call plus a
label-to-internal-string mapping:

```python
def _ask_mode():
    """BarLength or FakeBar.  Returns "length" / "fake" / None."""
    choice = ask_option("Edit bar lengths, or mark bars as non-fabricated staging",
                        ["BarLength", "FakeBar"], default="BarLength")
    return {"BarLength": "length", "FakeBar": "fake"}.get(choice)
```

About 160 lines of near-duplicate code become one helper plus eight three-line wrappers, and
the only thing that varies between commands is whether they pass `default=`.

**Do it in this order**, so a mistake is cheap: write the helper, migrate ONE command, click
it through in Rhino, then do the rest. Migrating `_ask_reuse_saved_base` also fixes its
missing default line for free.

**Also worth tidying at the same time:** three wrappers are called `_ask_mode`, one is
`_ask_place_mode`.
