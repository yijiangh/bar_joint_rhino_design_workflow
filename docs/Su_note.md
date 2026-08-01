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
