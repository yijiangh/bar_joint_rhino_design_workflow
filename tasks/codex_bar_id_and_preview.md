# Bar ID & Preview System — Implementation Spec

## Goal

Add a persistent bar identity system and managed tube previews so every
scaffolding bar has a human-readable ID (B1, B2, …) that survives
copy/paste, save/load, and drag operations.

---

## 1. New file: `scripts/core/rhino_bar_registry.py`

Rhino-only shared module (uses `rhinoscriptsyntax`, `scriptcontext`).
Imported by every `rs_*.py` script that touches bars.

### Constants

```python
BAR_TYPE_KEY      = "bar_type"
BAR_ID_KEY        = "bar_id"
BAR_GUID_KEY      = "bar_guid"
BAR_TYPE_VALUE    = "scaffolding_bar"   # default; extensible later

TUBE_LAYER        = "Tube preview"
TUBE_BAR_ID_KEY   = "tube_bar_id"
TUBE_AXIS_GUID_KEY = "tube_axis_id"     # already used by existing code
TUBE_CACHE_START  = "tube_cache_start"  # "x,y,z"
TUBE_CACHE_END    = "tube_cache_end"    # "x,y,z"

BAR_RADIUS_KEY    = "tube_radius"       # already used by existing code
```

### Functions

#### `next_bar_id() -> str`

Scan every object in the document that has `bar_type == "scaffolding_bar"`.
Parse integer suffix from each `bar_id` value (e.g. "B7" → 7).
Return `"B{max+1}"`.  If no bars exist, return `"B1"`.

#### `ensure_bar_id(curve_id, bar_type=BAR_TYPE_VALUE) -> str`

1. If curve has `bar_type` user text AND `bar_guid` matches
   `str(rs.coerceguid(curve_id))` → already registered, return existing
   `bar_id`.
2. If curve has `bar_type` but guid mismatch → it's a copy.  Assign new
   ID via `next_bar_id()`.
3. If curve has no `bar_type` → new bar.  Assign via `next_bar_id()`.
4. In cases 2 & 3:
   - `rs.SetUserText(curve_id, BAR_TYPE_KEY, bar_type)`
   - `rs.SetUserText(curve_id, BAR_ID_KEY, new_id)`
   - `rs.SetUserText(curve_id, BAR_GUID_KEY, str(rs.coerceguid(curve_id)))`
   - `rs.ObjectName(curve_id, new_id)`
5. Return the `bar_id` string.

#### `is_bar(curve_id) -> bool`

Return `rs.GetUserText(curve_id, BAR_TYPE_KEY) == BAR_TYPE_VALUE`.

#### `get_all_bars() -> dict[str, guid]`

Iterate all objects in the document.  For each curve with
`bar_type == BAR_TYPE_VALUE`:
- Call `ensure_bar_id(curve_id)` to heal any copy/paste or name drift.
  (This fixes guid mismatches and syncs ObjectName.)
- Collect into `{bar_id: curve_guid}` dict.

Return the dict.

#### `_format_point(arr) -> str`

Return `f"{x:.6f},{y:.6f},{z:.6f}"` for cache storage.

#### `_parse_cached_point(s) -> tuple or None`

Parse `"x,y,z"` string back to `(float, float, float)`.  Return None on
failure.

#### `_find_existing_tube(curve_id) -> guid or None`

Search all objects on the `TUBE_LAYER` that have
`tube_axis_id == str(curve_id)`.  Return the first match, or None.

This can use `rs.ObjectsByLayer(TUBE_LAYER)` filtered by user text.

#### `_tube_geometry_matches(tube_id, curve_id) -> bool`

Compare the tube's cached start/end (`tube_cache_start`, `tube_cache_end`)
with the current `rs.CurveStartPoint / rs.CurveEndPoint` of the axis
curve.  Return True if both match within `1e-3` tolerance.

#### `ensure_bar_preview(curve_id, color=None, bar_id=None) -> list[guid]`

1. Look up existing tube via `_find_existing_tube(curve_id)`.
2. If found AND `_tube_geometry_matches` → already correct, return it.
3. If found but geometry stale → delete old tube.
4. Create new tube from the axis curve endpoints (use the Rhino Cylinder
   approach from existing `_bake_axis_tube`; copy that logic here).
   Use `config.BAR_RADIUS` for radius.
5. Set user text on the new tube:
   - `TUBE_AXIS_GUID_KEY` = `str(curve_id)`
   - `TUBE_BAR_ID_KEY` = `bar_id or rs.GetUserText(curve_id, BAR_ID_KEY)`
   - `BAR_RADIUS_KEY` = `f"{config.BAR_RADIUS:.6f}"`
   - `TUBE_CACHE_START` = `_format_point(start)`
   - `TUBE_CACHE_END` = `_format_point(end)`
6. Place on `TUBE_LAYER`, apply color, set ObjectName to
   `"{bar_id}_tube"`.
7. Return list of baked tube object IDs.

#### `update_all_previews(color=None) -> int`

Call `get_all_bars()`.  For each bar, call `ensure_bar_preview`.
Return count of bars processed.

Note: this function needs access to config.BAR_RADIUS. Import config the
same way the existing scripts do (importlib.reload at call time, or accept
config as a parameter).

### Config access pattern

`ensure_bar_preview` and `update_all_previews` need `BAR_RADIUS` from
config.  Two options:
- (a) Accept `bar_radius` as a parameter.  Caller passes
  `config.BAR_RADIUS`.
- (b) Import `core.config` inside the function.

**Use option (a)** — pass `bar_radius` as a float parameter.  This keeps
the registry module free of reload-order coupling.  Every caller already
has config loaded and reloaded.

So the signatures become:
```python
def ensure_bar_preview(curve_id, bar_radius, color=None, bar_id=None) -> list
def update_all_previews(bar_radius, color=None) -> int
```

---

## 2. New script: `scripts/rs_create_bar.py`

Minimal Rhino command.  Venv directives same as other RS scripts.

Behavior:
1. Prompt: `rs.GetObject("Select a curve to register as a bar", rs.filter.curve)`
2. Call `ensure_bar_id(curve_id)`.
3. Call `ensure_bar_preview(curve_id, config.BAR_RADIUS)`.
4. Print: `"Registered bar {bar_id}"`.

Allow multi-select: use `rs.GetObjects` (plural) so multiple curves can
be tagged at once.  Loop over each.

---

## 3. New script: `scripts/rs_update_preview.py`

Minimal Rhino command.  Venv directives same as other RS scripts.

Behavior:
1. Reload config.
2. Call `update_all_previews(config.BAR_RADIUS)`.
3. Print: `"Updated {n} bar previews"`.

---

## 4. Modify `scripts/rs_bar_snap.py`

At the point where the output bar is finalized (after `rs.MoveObject`):

1. `ensure_bar_id(le_id)` — tag the existing bar if not already tagged.
2. `new_bar_id = ensure_bar_id(line_id)` — tag the newly positioned bar.
3. Replace `_bake_axis_tube(le_id, ...)` with
   `ensure_bar_preview(le_id, config.BAR_RADIUS, color=_S1_EXISTING_BAR_COLOR)`.
4. Replace `_bake_axis_tube(line_id, ...)` with
   `ensure_bar_preview(line_id, config.BAR_RADIUS, color=_DEFAULT_TUBE_COLOR)`.

Import `ensure_bar_id, ensure_bar_preview` from `core.rhino_bar_registry`.

The local `_bake_axis_tube` function can remain for now (it won't be
called in the main flow) or be deleted.  Prefer deleting it after both
snap and brace are migrated.

---

## 5. Modify `scripts/rs_bar_brace.py`

Same pattern as snap:

1. `ensure_bar_id(le1_id)` and `ensure_bar_id(le2_id)` at selection time.
2. `ensure_bar_id(line_id)` after the brace line is created.
3. Replace `_bake_axis_tube` calls with `ensure_bar_preview` calls.

---

## 6. Modify `scripts/rs_joint_place.py`

After bars are selected:

1. `le_bar_id = ensure_bar_id(le_id)`.
2. `ln_bar_id = ensure_bar_id(ln_id)`.
3. In `_place_joint_blocks`, change `joint_data` to use the human-readable
   bar_id instead of raw UUID:
   ```python
   "bar_id": le_bar_id,            # was: str(le_id)
   "connected_bar_id": ln_bar_id,  # was: str(ln_id)
   ```
   Also keep the raw UUIDs as `"bar_guid"` and `"connected_bar_guid"` for
   programmatic lookup.

---

## 7. Update toolbar RUI

Add two new buttons to the RSDesign toolbar group:

| Button | Script | Tooltip |
|--------|--------|---------|
| RSCreateBar | `rs_create_bar.py` | Register selected curves as scaffolding bars |
| RSUpdatePreview | `rs_update_preview.py` | Refresh all bar tube previews |

Use `generate_rhino_toolbar.py` if it supports adding buttons, otherwise
edit `scaffolding_toolbar.rui` directly.

---

## 8. Shared helpers to extract from existing scripts

The following helpers are currently duplicated across rs_bar_snap.py,
rs_bar_brace.py, and rs_joint_place.py.  They should be moved into
`rhino_bar_registry.py` (or a separate `rhino_helpers.py` if preferred):

- `_point_to_array(point)` → used everywhere
- `_curve_endpoints(curve_id)` → used everywhere
- `_ensure_layer(layer_name)` → used everywhere
- `_apply_object_display(object_ids, label, color, layer_name)` → used
  by tube creation and reference segments
- `_suspend_redraw()` → used by snap, brace, joint_place

**Decision**: Create `scripts/core/rhino_helpers.py` for these generic
Rhino utilities.  Keep `rhino_bar_registry.py` focused on bar identity
and preview logic.  `rhino_bar_registry` imports from `rhino_helpers`.

---

## 9. Data model summary

### Bar curve (the source line)
```
ObjectName:               "B7"
UserText[bar_type]:       "scaffolding_bar"
UserText[bar_id]:         "B7"
UserText[bar_guid]:       "<rhino uuid at assignment time>"
Layer:                    "Bar Axis Lines"  (set by _place_axis_line, unchanged)
```

### Tube preview (display cylinder)
```
ObjectName:               "B7_tube"
UserText[tube_axis_id]:   "<bar curve uuid>"
UserText[tube_bar_id]:    "B7"
UserText[tube_radius]:    "10.000000"
UserText[tube_cache_start]: "100.000000,200.000000,0.000000"
UserText[tube_cache_end]:   "100.000000,200.000000,500.000000"
Layer:                    "Tube preview"
```

### Joint block instance (existing, updated)
```
UserText[joint_data] (JSON):
  bar_id:             "B7"       ← human-readable (was UUID)
  connected_bar_id:   "B12"      ← human-readable (was UUID)
  bar_guid:           "<uuid>"   ← NEW: raw UUID kept for lookup
  connected_bar_guid: "<uuid>"   ← NEW: raw UUID kept for lookup
  type, dof, transform, residual, etc: unchanged
```

---

## 10. Files created/modified

| File | Action |
|------|--------|
| `scripts/core/rhino_helpers.py` | **NEW** — shared Rhino utilities extracted from rs_*.py |
| `scripts/core/rhino_bar_registry.py` | **NEW** — bar ID + preview management |
| `scripts/rs_create_bar.py` | **NEW** — register curves as bars |
| `scripts/rs_update_preview.py` | **NEW** — refresh all tube previews |
| `scripts/rs_bar_snap.py` | **MODIFY** — call ensure_bar_id + ensure_bar_preview |
| `scripts/rs_bar_brace.py` | **MODIFY** — call ensure_bar_id + ensure_bar_preview |
| `scripts/rs_joint_place.py` | **MODIFY** — call ensure_bar_id, store human bar_id in joint_data |
| `scaffolding_toolbar.rui` | **MODIFY** — add RSCreateBar + RSUpdatePreview buttons |

---

## 11. What NOT to do in this pass

- Do NOT add assembly sequence tracking yet.
- Do NOT add joint type classification yet (floor/subfloor/etc.).
- Do NOT add joint-moves-with-bar event watchers.
- Do NOT change the optimization or FK code.
- Do NOT remove the `_bake_axis_tube` from individual scripts until all
  callers are migrated and tested in Rhino.
