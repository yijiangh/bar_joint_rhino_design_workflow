# Prefab Export — Implementation Spec

## Goal

Export per-bar fabrication data from Rhino to JSON for the joint jig controller.
Two changes: (1) store `joint_id` at placement time, (2) new export script.

---

## 1. Joint ID — changes to `rs_joint_place.py`

### Joint ID format

`joint_id = f"J{le_num}-{ln_num}"` where `le_num` / `ln_num` are the numeric
part of bar_id (e.g. bar_id="B3" → num=3). Always one joint per bar pair.

### Generate joint_id

In `_place_joint_blocks`, after extracting `le_bar_id` / `ln_bar_id`:

```python
le_num = le_bar_id.lstrip("B")
ln_num = ln_bar_id.lstrip("B")
joint_id = f"J{le_num}-{ln_num}"
```

### Store joint_id

Add `"joint_id": joint_id` field to both `joint_data_female` and
`joint_data_male` dicts (add it right after `"type"`).

Also write it as a separate user text key on each block instance so it's
visible in Rhino Properties without parsing JSON:

```python
rs.SetUserText(female_id, "joint_id", joint_id)
rs.SetUserText(male_id, "joint_id", joint_id)
```

No other changes to `rs_joint_place.py`.

---

## 2. New script: `rs_export_prefab.py`

Rhino script with standard header (`#! python 3` / `# venv: scaffolding_env` /
`# r: numpy`).

### Algorithm

1. **Collect bars** — call `get_all_bars()` from `core.rhino_bar_registry`.
2. **Collect joint blocks** — `rs.ObjectsByLayer("FemaleJointPlacedInstances")`
   + `rs.ObjectsByLayer("MaleJointPlacedInstances")`. For each, read
   `joint_data` user text (JSON). Skip objects without it.
3. **Group joints by bar** — key = `joint_data["bar_id"]`. Each joint maps to
   the bar it sits on.
4. **For each bar**:
   - Compute `length_mm` = distance between curve start and end.
   - For each joint on this bar:
     - `joint_id` from `joint_data["joint_id"]`
     - `type` and `subtype` — split `joint_data["type"]` at `_`:
       `type, subtype = joint_data["type"].split("_", 1)`
     - `position_mm` from `joint_data["dof"]`:
       - If subtype == "Female": `dof["fjp"]`
       - If subtype == "Male": `dof["mjp"]`
     - `rotation_deg` from `joint_data["dof"]`:
       - If subtype == "Female": `degrees(dof["fjr"])`
       - If subtype == "Male": `degrees(dof["mjr"])`
     - `ori` — derive from block instance transform vs bar direction (see below)
5. **Sanity checks** per joint (see below).
6. **Build output dict**, sort joints by `position_mm` per bar.
7. **Write JSON** to file next to the .3dm document.

### Orientation (`ori`) derivation

The placed block instance has a 4×4 transform stored in
`joint_data["transform"]` (flat list of 16 floats, row-major, reshaped to 4×4).
The joint's local x-axis in world coords = column 0 of the 3×3 rotation part
(i.e. `transform[:3, 0]`).

The bar direction = `(bar_end - bar_start) / ||bar_end - bar_start||`
(from curve endpoints).

```python
dot = np.dot(joint_x_axis, bar_direction)
if dot > 0:
    ori = "P"   # x-axis points toward bar end
else:
    ori = "N"   # x-axis points toward bar start
```

### Sanity checks

For each joint, before accepting it:

1. **Origin on bar axis**: project joint origin onto the bar line. The
   perpendicular distance must be < `AXIS_TOL` (suggest 0.1 mm). If not →
   error message naming the joint_id and bar_id, skip this joint but continue
   with others.

2. **X-axis aligned with bar**: `|dot|` must be > `1.0 - ALIGN_TOL`
   (suggest `1e-3`, i.e. ~1.8 deg). If not → error message, skip joint.

Print a summary: how many bars, how many joints, how many skipped with errors.

### Output schema

```json
{
  "schema_version": 1,
  "project_id": "<rhino_document_filename_without_extension>",
  "bars": [
    {
      "bar_id": "B1",
      "length_mm": 500.0,
      "joints": [
        {
          "joint_id": "J1-2",
          "type": "T20",
          "subtype": "Female",
          "ori": "P",
          "position_mm": 226.09,
          "rotation_deg": 170.5
        }
      ]
    }
  ]
}
```

`bars` sorted by `bar_id` (numeric part). `joints` sorted by `position_mm`.

### File output

Default path: `{document_folder}/{document_stem}_prefab.json`
Use `rs.SaveFileName` dialog with this default. Print the path on success.

---

## 3. Toolbar update — `scaffolding_toolbar.rui`

Add `RSExportPrefab` button to the **RSSetup** toolbar (after RSExportCase).

Macro: `! _-ScriptEditor _R "rs_export_prefab.py"`

Tooltip: "Export bar prefabrication data as JSON"

Use guid pattern `a1b2c3d4-...-00000000000a` for the new toolbar item and
`a1b2c3d4-...-00000000000b` for the macro (follow existing numbering).

---

## Scope boundaries

- Do NOT change the joint jig controller repo.
- Do NOT add `connected_bar_id`, `residual`, or other solver metadata to the
  export — only fabrication-relevant fields.
- Do NOT change bar user text (no `bar_length` key).
- Do NOT change block definition names.
- Do NOT add tests (Rhino-only code, not testable without Rhino).
