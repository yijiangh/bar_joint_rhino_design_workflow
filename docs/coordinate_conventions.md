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

---

## 4. Joint Placement and Source of Truth

During design, the **single source of truth** for any joint's position and orientation is the **world transform of the block instance in the Rhino document**. Derived quantities such as `position_mm` and `rotation_deg` are computed from this transform at export time, they may be cached but are not considered as the authoritative record.

Joint blocks are placed on two layers:

- `FemaleJointPlacedInstances`
- `MaleJointPlacedInstances`

Each placed instance carries user text keys for reference and traceability:

| Key | Content |
|-----|---------|
| `joint_id` | e.g. `J1-2` |
| `joint_type` | e.g. `T20` |
| `joint_subtype` | e.g. `Female` |
| `parent_bar_id` | Bar this joint is mounted on |
| `connected_bar_id` | The other bar in the pair |
| `female_parent_bar` | Le bar ID (same for both blocks in a pair) |
| `male_parent_bar` | Ln bar ID (same for both blocks in a pair) |

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
