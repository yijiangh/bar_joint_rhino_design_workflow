# Coordinate Conventions

Definitions of all frames, axes, and naming conventions used in the bar-joint workflow.
All lengths are in **millimetres**.

---

## 1. Naming Conventions

| Name | Meaning |
|------|---------|
| **Le** | The existing bar in a joint pair — receives the **female** connector block |
| **Ln** | The new bar in a joint pair — receives the **male** connector block |
| **B1, B2, …** | Bar IDs assigned by RSCreateBar in registration order |
| **J{le}-{ln}** | Joint ID, e.g. `J1-2` for the connection where B1 is female and B2 is male |
| **FJP / FJR** | Internal solver DOF variables: female joint axial position / rotation. Not the design-time source of truth. |
| **MJP / MJR** | Internal solver DOF variables: male joint axial position / rotation. Not the design-time source of truth. |

---

## 2. Bar Frame

Designers draw bars as Rhino line objects with a chosen **start point** and **end point**. The bar frame is derived from these two points:

| Axis/Point | Definition |
|-----------|-----------|
| **Z** | Unit vector from start → end |
| **X** | `normalize(world_Z × bar_Z)`. Falls back to `normalize(world_X × bar_Z)` when `\|dot(bar_Z, world_Z)\| ≥ 0.95` (bar is nearly vertical) |
| **Y** | `bar_Z × bar_X` (right-handed) |
| **Origin** | Bar **midpoint** |

This construction gives a stable, consistent frame for any bar orientation.

---

## 3. Joint Block Geometry

Joint blocks are Rhino block definitions. The geometry of a block is defined relative to its **local frame**. The following axes apply when the block is inserted at the world origin with identity rotation.

### Female joint

| Axis/Point | Physical meaning |
|-----------|----------------|
| **Z** | Points **into the screw hole** — the assembly direction from which the male side is inserted |
| **Origin** | Lies on the bar centre-line, at the midpoint of the hole that sandwiches the bar |
| **X** | Points towards one of the two possible directions of the bar's centerline. Defines orientation — see §4 |
| **Y** | `bar_Z × bar_X` (right-handed) |

### Male joint

| Axis/Point | Physical meaning |
|-----------|----------------|
| **Z** | Points **toward the female joint** — the direction the screw travels into the female side |
| **Origin** | Lies on the bar centre-line, at the midpoint of the hole that sandwiches the bar |
| **X** | Points towards one of the two possible directions of the bar's centerline. Defines orientation — see §4 |
| **Y** | `bar_Z × bar_X` (right-handed) |

The Z-axes of a correctly assembled female–male pair are anti-parallel and co-linear (screw enters from the male Z direction; female receives it from its own −Z direction).

### Where a joint's Z actually points in the world — and why it can cancel

The block's Z is **not** along the bar. Reading `M_block_from_bar` for `T20_Male` out of
`joint_pairs.json`, its rotation rows are `[0,1,0] / [0,0,-1] / [-1,0,0]`, so the block's
local **+Z is `-bar_X`** — horizontal, and *perpendicular* to the bar — then spun about
the bar axis by that joint's `rotation_deg` (`jr`, see §5).

Two consequences that the base-placement code has to handle:

- **Two anchors on one bar can point in opposite directions.** A bar whose two joints are
  ~180° apart in `jr` (one coupler grabbing from the left, one from the right) has two
  insertion axes that cancel. Averaging them yields floating-point noise, not a direction.
- **A `jr` near ±90° tips the axis toward vertical**, whose horizontal component is also
  noise.

Both are detected in `core.base_guide_geom.resolve_heading_from_axes` (mean resultant
length + a minimum-horizontal filter) rather than being averaged blindly — see §7.

---

## 4. Joint Placement and Source of Truth

During design, the **single source of truth** for any joint's position and orientation is the **world transform of the block instance in the Rhino document**. Derived quantities such as `position_mm` and `rotation_deg` are computed from this transform at export time, they may be cached but are not considered as the authoritative record.

Joint blocks are placed on two layers:

- `FemaleJointPlacedInstances`
- `MaleJointPlacedInstances`

Each placed instance carries user text keys written by RSJointPlace:

| Key | Content | Authoritative? |
|-----|---------|----------------|
| `joint_id` | e.g. `J1-2` | ✓ |
| `joint_type` | e.g. `T20` | ✓ |
| `joint_subtype` | e.g. `Female` | ✓ |
| `parent_bar_id` | Bar this joint is mounted on | ✓ |
| `connected_bar_id` | The other bar in the pair | ✓ |
| `female_parent_bar` | Le bar ID (same for both blocks in a pair) | ✓ |
| `male_parent_bar` | Ln bar ID (same for both blocks in a pair) | ✓ |
| `ori` | `"P"` or `"N"` — cached from placement, consistent with §5 | convenience |
| `position_mm` | Cached from solver DOF (FJP/MJP). Matches §5 for female; may differ for male due to negated FK convention. **Not used by RSExportPrefab.** | convenience |
| `rotation_deg` | Cached from solver DOF (FJR/MJR) in degrees. Uses a different zero reference than the geometric §5 convention. **Not used by RSExportPrefab.** | convenience |

RSExportPrefab ignores all cached convenience values and recomputes `position_mm`, `ori`, and `rotation_deg` fresh from the block's world transform (see §5).

---

## 5. Exported Quantities (RSExportPrefab)

The prefabrication export derives three quantities per joint block from the block's world transform and its parent bar's geometry. All three are computed fresh at export time; the stored block transform is the only input.

### `position_mm`

Signed distance along the bar from the bar **start point** to the joint block origin, projected onto the bar direction:

```
position_mm = dot(block_origin_world − bar_start, bar_Z)
```

Positive means the block origin is on the bar-end side of the start point.

### `ori`

Which direction the block's local X-axis points along the bar:

| Value | Meaning |
|-------|---------|
| `"P"` | Block X-axis points toward bar end (same sense as bar Z) |
| `"N"` | Block X-axis points toward bar start (opposite to bar Z) |

```
ori = "P" if dot(block_X_world, bar_Z) > 0 else "N"
```

### `rotation_deg`

The angle from the bar X-axis to the joint Z-axis (assembly axis), measured around bar Z with the right-hand rule:

```
proj = normalize(joint_Z_world − dot(joint_Z_world, bar_Z) · bar_Z)   # project onto bar XY plane
rotation_deg = atan2(dot(cross(bar_X, proj), bar_Z), dot(bar_X, proj))
```

At `rotation_deg = 0` the joint's assembly axis (Z) is aligned with the bar's X-axis.  
Positive rotation is counter-clockwise about bar Z (right-hand rule).

---

## 6. IK Keyframe Data (`ik_assembly` / `ik_support`)

The IK keyframe workflow (`RSIKKeyframe`) stores a JSON record as user-text on the shared **Ln bar axis line** — not on joint blocks and not on individual Le bars. `RSShowIK` reads the same record to replay a saved keyframe.

### Where

- `ik_assembly` is stored on the Ln bar user-text (key exactly `ik_assembly`, value = JSON string).
- Both picked male joint blocks must share the same `male_parent_bar` user-text — that bar is the Ln bar and receives the record.
- `ik_support` is reserved for a future single-arm workflow using the same bar, with a parallel shape.

### `ik_assembly` schema

```json
{
  "robot_id": "dual-arm_husky_Cindy",
  "base_frame_world_mm": [[...4x4 row-major...]],
  "final":    { "left":  {"joint_names": [...], "joint_values": [...]},
                "right": {"joint_names": [...], "joint_values": [...]} },
  "approach": { "left":  {"joint_names": [...], "joint_values": [...]},
                "right": {"joint_names": [...], "joint_values": [...]} }
}
```

| Field | Meaning |
|-------|---------|
| `robot_id` | Identifier of the robot model the keyframe was solved against. Currently `"dual-arm_husky_Cindy"`. |
| `base_frame_world_mm` | 4x4 homogeneous transform of the mobile-base frame in world coordinates, **translations in mm**. Z = Brep face normal at the picked base point; X = heading projected onto the tangent plane. |
| `final` | Joint configurations for both arms with the tools at the final contact poses (tool0 = the placed tool block's world transform, i.e. the joint's tool-attach frame ⊗ `inv(M_tcp_from_block)` from `robotic_tools.json`). The attach frame is the male OCF for male joints; for ground joints it is the ground OCF ⊗ `M_tool_from_block` from `joint_pairs.json` (identity unless the ground definition picked one). |
| `approach` | Joint configurations for both arms with tools offset by `-unit(avg(male_z_L, male_z_R)) * LM_DISTANCE` from the final poses. |
| Per-side `joint_names` / `joint_values` | Configurable joints of the planning group (`base_left_arm_manipulator` / `base_right_arm_manipulator`), order preserved from `RobotCell.get_configurable_joint_names(group)`. |

### Invariants

- Translations in `base_frame_world_mm` are in **millimetres**; joint values in radians.
- The base frame applies to both `final` and `approach` — the mobile base is stationary between the two poses.
- `ik_support` (when written) will follow the same field layout but contain a single `"arm"` entry instead of `"left"` + `"right"`.

### Not stored on the bar

- **Bar OCF** — always recomputed fresh from bar endpoints per §2.
- **Joint OCFs and types** — looked up at load time by scanning baked joint block instances for matching `male_parent_bar` / `female_parent_bar`; the authoritative joint transform is the block instance's world transform.

---

## 7. Mobile-Base Placement: Heading, Guides, and Arm Sides

Where the mobile base stands for a bar is decided by one vector: the **heading**, the
direction the base +X faces. The base stands the standoff distance *against* it, so
driving forward carries the bar into the assembly — and the heading alone therefore
decides which side of the bar the robot occupies.

### Resolving the heading

`core.rhino_walkable_ground.resolve_bar_heading` gathers the bar's anchor-joint insertion
axes (each joint block's local +Z, from **both** the male and ground joint layers — the
same set assembly IK accepts as arm anchors) and hands them to the Rhino-free
`core.base_guide_geom.resolve_heading_from_axes`, which:

1. **flattens each axis onto the ground plane first**, discarding any whose horizontal
   part is shorter than `config.INSERTION_DIR_MIN_HORIZONTAL` (a near-vertical axis
   carries no azimuth — see §3);
2. measures agreement with the **mean resultant length**, `|Σ unit axes| / count`
   (`1.0` = all axes agree, `0.0` = they cancel exactly). Below
   `config.INSERTION_DIR_CANCEL_TOL` the axes disagree about the side and no heading is
   returned.

| Outcome | `source` | Meaning |
|---------|----------|---------|
| Axes agree | `averaged` | The heading is their normalized sum |
| Axes cancel / all vertical | `perpendicular-open` | Fall back to the two bar-perpendicular horizontal directions and stand on whichever side has more clear walkable ground (standoff point on the ground, farthest from other bars and environment obstacles) |
| Neither side is standable | `world-fallback` | The legacy world-horizontal direction; reported as ambiguous |

Bars resolved by anything other than `averaged` are flagged `ambiguous` and listed in the
`RSIKKeyframeAll` summary as "verify the side".

### Base guide lines

Both IK keyframe commands draw five lines on the walkable ground
(`core.base_guide_geom.build_base_guides`, baked by `core.base_guide_viz`):

| Line | Colour | Definition |
|------|--------|-----------|
| Projection | teal | The line joining the bar's two anchor-joint centres, projected onto the walkable ground (only the projection is drawn) |
| ×3 offsets | light teal | That line stepped **against** the heading by each of `config.BASE_GUIDE_OFFSETS_MM` (375 / 500 / 625 mm), labelled with a text dot |
| Extension | yellow | Through the midpoints of the four lines above |

Every endpoint is re-snapped onto the ground, so the guides follow a stepped or sloped
surface. The placed base origin lies **on the extension line**, at the offset line
matching the standoff — `config.IK_BASE_STANDOFF_MULTIBAR_MM` is kept equal to the middle
offset so the default auto-placement lands on a drawn line.

All of this is transient preview geometry on `config.LAYER_BASE_GUIDE_PREVIEW`, removed on
every exit path.

### Which end of a bar takes the LEFT tool

This follows from the approach, **not** from the bar. The robot faces the bar along the
heading, so the end on its left is the one its left arm reaches:

```
   [tool R]
      |
     bar        [robot]      heading = robot → bar
      |                      left    = ground_normal × heading
   [tool L]
```

`core.base_guide_geom.arm_side_for_joint` dots each joint's offset from the bar centre
against that left direction; `core.rhino_tool_place.assign_tool_sides_from_heading`
re-places any tool that disagrees. Flip the base to the other side and the heading
negates, so both ends swap.

**A hand-picked side wins.** Cycling a tool in RSJointEdit stamps
`config.KEY_TOOL_SIDE_MANUAL` on the joint block, and the automatic rule skips joints
carrying it — so a deliberate manual edit is never silently undone. An explicit base Flip
clears the mark, because that is the user re-deciding the side.
