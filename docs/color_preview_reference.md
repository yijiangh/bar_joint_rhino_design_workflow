# Colour preview reference

Every colour the toolbar commands paint into the viewport, grouped by where you see it.
Written to make the visual language **cohesive** — so the last section, [Collisions and
near-duplicates](#collisions-and-near-duplicates), is the point of the document.

Three things to know when reading it:

- **Where a colour is defined.** The "Defined as" column names the constant, not a line number,
  so it stays right when files move. A colour shared by several commands lives in
  [`scripts/core/config.py`](../scripts/core/config.py), one definition each; a colour only one
  module uses is a constant in that module. Inline literals are flagged **(inline)**.
- **One colour, one name.** Where two concepts are the same thing they share one constant
  (built bar and IK env highlight are both `SEQ_COLOR_BUILT`). Where two different concepts
  share an RGB on purpose, the second is defined *as* the first, so they cannot drift apart.
- **RGB 0-255 vs 0-1 floats.** Rhino object colours are 0-255 tuples. The tool-inspector meshes
  use 0-1 floats with alpha, because they go to a mesh material rather than an object colour.

---

## RSSequenceEdit — assembly sequence

Defined in `core/rhino_bar_registry.py`, applied by `show_sequence_colors`.

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| green | `60, 179, 60` | built — earlier than the active step | `rhino_bar_registry.SEQ_COLOR_BUILT` |
| blue | `30, 100, 220` | the active step | `SEQ_COLOR_ACTIVE` = `config.SELECTED_BAR_COLOR` |
| grey | `160, 160, 160` | unbuilt — later than the active step | `SEQ_COLOR_UNBUILT` |
| teal | `40, 170, 160` | built but still unstable (its supports are not up yet) | `SEQ_COLOR_UNSTABLE` |
| purple | `110, 40, 160` | support bar of the active step — shown in the normal view, and while the EditSupports picker is open | `SEQ_COLOR_SUPPORT_PICK` |
| pink | `230, 115, 150` | fake bar — staging that will not be fabricated (same pink as IK-failed, deliberately) | `SEQ_COLOR_FAKE` |

Precedence when a bar qualifies for more than one, loosest to tightest: sequence state → fake →
support. The active bar always keeps its blue.

The fake pink is also what `RSBarEdit > FakeBar` paints: entering the mode highlights every bar
already marked fake, and each Add / Delete repaints that bar immediately, so the pink on screen
is always the current mark set. It is left on at exit, and survives RSClearColorPreview.

### Turning individual tints off — `color_flags`

`show_sequence_colors(..., color_flags={...})` gates the tints per class, keys `"built"` /
`"active"` / `"unbuilt"` / `"support"`. A `False` entry leaves that class **ByLayer** — the
tint is dropped, the object stays exactly as visible as the rules above make it. `None` (the
default) paints everything, so every toolbar command is unaffected.

Written for the Grasshopper animation component
([grasshopper_animation.md](grasshopper_animation.md)), where you switch legend colours off
to film a clean frame. One tint is deliberately not switchable: **teal** is a *variant of
built*, not a class of its own, so it rides on `"built"`. **Pink (fake)** stays painted
whenever the bar is visible — matching `clear_ik_preview`, which re-asserts it rather than
resetting it, because a staging bar that renders like a real one is a fabrication error
waiting to happen. The filming view does not silence that tint; it hides the bar outright
with `show_fake=False` (below).

Joints follow their parent bar's *paint decision*, not just its colour, and the active step's
tool follows `"active"` — so switching a class off never leaves its joints or tools tinted.
Visibility is untouched in every case; to actually hide unbuilt bars use the separate
`show_unbuilt` argument.

### The filming arguments — `show_fake`, `tint_curves_only`, `geom_built_and_active_only`, `line_style`

Four more optional arguments on `show_sequence_colors`, all added for the Grasshopper
preview and all defaulting to today's behaviour, so no toolbar command is affected:

| argument | default | what the filming view passes |
| --- | --- | --- |
| `show_fake` | `True` | `False` — staging bars are hidden outright, not just tinted |
| `tint_curves_only` | `False` | `True` — class colours land on the bar CENTERLINES only; tubes, joints and tools keep their normal by-layer look |
| `geom_built_and_active_only` | `False` | `True` — tube + joint geometry only for built bars and the active bar; later bars keep at most their centerline |
| `line_style` | `None` | `{"thickness_mm", "dashed", "pattern"}` — per-object print width and dash on the centerlines |

Together they produce the filming split: **real model geometry for what has been built,
coloured guide lines for everything else.**

`line_style` writes two per-object attributes on the centerline curves. Print width only
renders in the viewport while **PrintDisplay** is on (the GH component switches it on and
off); dashes use a document linetype named after its own pattern (`RS_PreviewDash_4x2`),
created on demand so a changed pattern makes a new linetype instead of mutating one in
use. `reset_sequence_colors` resets both attribute sources back to by-layer alongside the
colours — so the usual cleanup path covers them too.

## RSUpdatePreview — model health

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| orange | `175, 55, 10` | joint/tool whose parent bar is gone (orphan link) | `config.ORPHAN_LINK_COLOR` |
| dark indigo | `75, 55, 110` | registered bar carrying no joint | `config.BARE_BAR_COLOR` |

Marks are drawn by `rhino_joint_refresh.mark_broken_links`; tools get a text dot rather than a
colour, because a block instance's colour does not reach sub-objects that carry baked colours.

## IK keyframe colours — RSIKKeyframe, RSIKKeyframeAll, RSShowIKColors

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| grey-blue | `75, 120, 150` | bar has a solved IK keyframe | `rhino_bar_registry.COLOR_HAS_IK` |
| pink | `230, 115, 150` | IK attempted at the placed base and failed — and, by design, also the fake-bar tint | `COLOR_FAILED` = `SEQ_COLOR_FAKE` |
| pink **(transient)** | `230, 115, 150` | RSIKKeyframe **rejected the bar you picked** — painted on its tool-bearing joint blocks (or on the bar, if it has none), cleared on the next pick and on exit | `COLOR_FAILED` |
| red | `255, 40, 40` | collision highlight (RSIKKeyframe, RSShowBarActionPlan) | `config.COLLISION_COLOR` |

## Base placement — guides, frame marker, reach

Guide lines (`core/base_guide_viz.py`):

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| teal | `0, 160, 160` | offset 0 — the joint line projected onto the ground | `base_guide_viz._PROJECTED_COLOR` |
| light teal | `120, 200, 200` | the 375 / 500 / 625 mm standoff lines + their labels | `_OFFSET_COLOR` |
| yellow | `255, 255, 0` | the midpoint extension line the base origin sits on | `_EXTENSION_COLOR` |

Base frame marker (`core/base_frame_viz.py`):

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| red | `220, 40, 40` | base +X — the heading (which way the robot faces) | `base_frame_viz._AXIS_COLORS["x"]` |
| green | `40, 180, 40` | base +Y | `_AXIS_COLORS["y"]` |
| blue | `40, 90, 220` | base +Z (ground normal) | `_AXIS_COLORS["z"]` |
| grey | `150, 150, 150` | base footprint rectangle | `_FOOTPRINT_COLOR` |
| light blue | `100, 100, 220` | baked reach circle — same as the live one | `config.REACH_CLEAR_COLOR` |

Live conduits during the pick (`core/dynamic_preview.py`) — these are display materials, not
object colours, so they vanish when the conduit closes:

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| light blue | `100, 100, 220` | reach outline clear of obstacles; IK sample circle | `config.REACH_CLEAR_COLOR` |
| salmon | `255, 100, 100` | reach outline touching an obstacle or the ground edge | `config.REACH_TOUCH_COLOR` |
| pale lavender | `180, 180, 220` | robot ghost mesh (default, alpha 0.5) | `config.GHOST_ROBOT_COLOR` |
| sky blue | `120, 200, 255` | arm reach volumes shown with the ghost (alpha 0.20) | **(inline)** in `MeshPreviewConduit` |
| cyan | `80, 200, 255` | IK sample seed | `IKSampleVizConduit._color_seed` |
| salmon | `255, 100, 100` | IK sample failed | `IKSampleVizConduit._color_failed` |
| light green | `100, 220, 100` | IK sample succeeded | `IKSampleVizConduit._color_success` |

## Ground and environment

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| green | `60, 200, 90` | WalkableGround highlight (RSAssignAndShowWalkableGround, RSShowBarActionPlan) | `config.WALKABLE_GROUND_HIGHLIGHT_COLOR` |
| green | `60, 179, 60` | env geometry highlighted for IK — the built bars, so the built green | `rhino_bar_registry.SEQ_COLOR_BUILT` |
| amber | `180, 120, 60` | Ground joint preview (RSJointPlace › JointOnly) | `config.GROUND_PREVIEW_COLOR` |
| steel blue | `60, 150, 200` | standalone MoCap joint preview (RSJointPlace › JointOnly) | `config.MOCAP_PREVIEW_COLOR` |

## Joint / bar placement previews

`VARIANT_PREVIEW_COLORS` cycles per solved variant — the colour means "variant index", not a state:

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| red / blue / green / amber | `230,80,80` · `80,80,230` · `80,200,80` · `200,160,50` | joint variant 0-3 (RSJointPlace); brace / subfloor candidate 1-4 and their two contact points (`core/two_contact_bar.py`) | `config.VARIANT_PREVIEW_COLORS` |
| blue | `30, 100, 220` | selected bar | `config.SELECTED_BAR_COLOR` |
| evenly spaced hues | HSV, s 0.65, v 0.95 | one hue per bar-length group (RSBarSelect › SelectByLength, RSBarEdit › BarLength) | `bar_length_groups.group_color` |

### Joint LAYER colours — the resting look of a baked joint

Not a state, and not a preview: this is the by-layer colour a placed joint block
renders in once it is committed. Applied by `rhino_bar_registry.enforce_managed_layers`
(`_JOINT_LAYER_COLORS`), so every command entry re-asserts them.

| Colour | RGB | Layer |
| --- | --- | --- |
| near-white | `230, 230, 230` | Joint Female Instances |
| mid grey | `105, 105, 105` | Joint Male Instances |
| warm grey | `200, 185, 170` | Joint Ground Instances |
| pale green-grey | `200, 220, 210` | Joint MoCap Instances |

The ground layer used to be amber `180, 120, 60`; it was muted to a warm grey when
the MoCap layer was added, so the four joint roles read as one family. The amber is
still in use for the ground *preview* (see [Ground and environment](#ground-and-environment)) —
the preview is a transient state, the layer colour is the resting look.

## Robotic tools

| Colour | RGB / RGBA | Meaning | Defined as |
| --- | --- | --- | --- |
| red | `0.90, 0.10, 0.10, 1.0` | LEFT tool mesh + labels (0-1 floats) | `rs_inspect_robotic_tool.LEFT_COLOR` |
| green | `0.10, 0.75, 0.15, 1.0` | RIGHT tool mesh + labels (0-1 floats) | `rs_inspect_robotic_tool.RIGHT_COLOR` |

## Miscellaneous

| Colour | RGB | Meaning | Defined as |
| --- | --- | --- | --- |
| orange | `255, 128, 0` | mocap bar layer | `rs_read_mocap_bar._LAYER_COLOR` |
| pure blue | `0, 0, 255` | closest segment in RSMeasureGap | `rs_measure_gap._CLOSEST_SEGMENT_COLOR` |
| light grey | `200, 200, 200` | length-group colour when there are no groups | `bar_length_groups._NO_GROUP_COLOR` |

---

## Collisions and near-duplicates

Ordered by how likely each is to mislead someone reading the screen.

### 1. Green means three different things

`60, 179, 60` is **built bar** in RSSequenceEdit and **env geometry** in the IK highlight (the
same idea, one constant); `60, 200, 90` is **WalkableGround**; `40, 180, 40` is the base **+Y
axis**; `0.10, 0.75, 0.15` is the **right tool**. Near-identical greens across unrelated
concepts. In RSIKKeyframe you can see built bars, highlighted env and walkable ground *at the
same time*.

**Suggested:** keep green for "already built / already there" (built bar + env), move
WalkableGround to a distinctly different hue, and leave the axis triad alone (RGB-for-XYZ is a
universal convention worth keeping).

### 2. Blue is both a state and an axis

`30, 100, 220` is the active step *and* the selected bar (one constant, one meaning: "the bar in
hand"); `40, 90, 220` is base +Z; `100, 100, 220` is the reach circle. **Suggested:** at minimum,
keep the reach circle visually distinct from the active bar — they co-occur throughout the base
pick.

### 3. Reds and pinks

`255, 40, 40` is a collision (`config.COLLISION_COLOR`); `220, 40, 40` is the base heading axis;
`230, 115, 150` is IK-failed *and* fake-bar; `255, 100, 100` is reach-touching-obstacle and a
failed IK sample. **Suggested:** one red for "collision / blocked", one for "failed".

The IK-failed / fake-bar overlap is deliberate rather than drift — both read as "the robot is not
building this one" — and the two cannot co-occur, because the IK overlay skips fake bars.
`COLOR_FAILED` is defined as `SEQ_COLOR_FAKE`, so they cannot split.

RSIKKeyframe's rejected-pick flag is a third user of the same pink, reading the same way. It is the
one case painted on **joint blocks** rather than bars, and the only one that is undone rather than
cleared: it snapshots each block's colour first and puts it back
(`rhino_bar_registry.snapshot_object_colors` / `restore_object_colors`), because the blocks it
marks may already carry a sequence colour or a broken-link mark that a reset-to-ByLayer would eat.
RSBarSelect's length preview restores colours the same way.

### 4. Teal is overloaded

`40, 170, 160` is an unstable bar (RSSequenceEdit) and `0, 160, 160` is the projected joint line
(base guides). Different commands, so low practical risk — noted for completeness.

### 5. Palette cycles read as state

`VARIANT_PREVIEW_COLORS` uses red/blue/green/amber to mean *variant 0-3*, which collides with
red=failure and green=built elsewhere. It is only on screen during an interactive pick, so the
risk is contained — but a red joint preview does not mean anything is wrong.

---

## Not yet assigned

Colours these planned features will need, listed here so they are chosen against the table above
rather than in isolation:

- ~~**fake bars**~~ — assigned: pink `230, 115, 150`, `SEQ_COLOR_FAKE`, shared with IK-failed
  (see [Reds and pinks](#3-reds-and-pinks)). Survives RSClearColorPreview.
- **joint with broken user text** (`no_joint_id` / `duplicate_joint_id` from the RSUpdatePreview
  tool-restore pass) — must NOT reuse orange `175, 55, 10`, which already means "parent bar is
  gone". Different fault, different colour.
