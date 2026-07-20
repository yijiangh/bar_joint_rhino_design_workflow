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
