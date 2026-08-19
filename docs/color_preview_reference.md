# Colour preview reference

Every colour the toolbar commands paint into the viewport, grouped by where you see it.
Written to make the visual language **cohesive** — so the last section, [Collisions and
near-duplicates](#collisions-and-near-duplicates), is the point of the document.

Two things to know when reading it:

- **Named constant vs inline literal.** A colour defined as a module constant can be changed in
  one place. A colour written inline at the call site cannot, and is where drift comes from.
  Inline ones are flagged **(inline)**.
- **RGB 0-255 vs 0-1 floats.** Rhino object colours are 0-255 tuples. The tool-inspector meshes
  use 0-1 floats with alpha, because they go to a mesh material rather than an object colour.

---

## RSSequenceEdit — assembly sequence

Defined in [rhino_bar_registry.py:648-661](scripts/core/rhino_bar_registry.py#L648-L661),
applied by `show_sequence_colors` ([:776](scripts/core/rhino_bar_registry.py#L776)).

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| green | `60, 179, 60` | built — earlier than the active step | [rhino_bar_registry.py:648](scripts/core/rhino_bar_registry.py#L648) |
| blue | `30, 100, 220` | the active step | [rhino_bar_registry.py:650](scripts/core/rhino_bar_registry.py#L650) |
| grey | `160, 160, 160` | unbuilt — later than the active step | [rhino_bar_registry.py:652](scripts/core/rhino_bar_registry.py#L652) |
| teal | `40, 170, 160` | built but still unstable (its supports are not up yet) | [rhino_bar_registry.py:655](scripts/core/rhino_bar_registry.py#L655) |
| purple | `110, 40, 160` | support bar of the active step — shown in the normal view, and while the EditSupports picker is open | [rhino_bar_registry.py:658](scripts/core/rhino_bar_registry.py#L658) |
| pink | `230, 115, 150` | fake bar — staging that will not be fabricated (same pink as IK-failed, deliberately) | [rhino_bar_registry.py:661](scripts/core/rhino_bar_registry.py#L661) |

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

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| orange | `175, 55, 10` | joint/tool whose parent bar is gone (orphan link) | [config.py:92](scripts/core/config.py#L92) |
| dark indigo | `75, 55, 110` | registered bar carrying no joint | [config.py:96](scripts/core/config.py#L96) |

Marks are drawn by `mark_broken_links` / `_drop_tool_marker`
([rhino_joint_refresh.py:363-421](scripts/core/rhino_joint_refresh.py#L363-L421)); tools get a
text dot rather than a colour, because a block instance's colour does not reach sub-objects that
carry baked colours.

## IK keyframe colours — RSIKKeyframe, RSIKKeyframeAll, RSShowIKColors

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| grey-blue | `75, 120, 150` | bar has a solved IK keyframe | [rhino_bar_registry.py:1670](scripts/core/rhino_bar_registry.py#L1670) |
| pink | `230, 115, 150` | IK attempted at the placed base and failed — and, by design, also the fake-bar tint | [rhino_bar_registry.py:1671](scripts/core/rhino_bar_registry.py#L1671) |
| pink **(transient)** | `230, 115, 150` | RSIKKeyframe **rejected the bar you picked** — painted on its tool-bearing joint blocks (or on the bar, if it has none), cleared on the next pick and on exit | [rs_ik_keyframe.py:406](scripts/rs_ik_keyframe.py#L406) |
| red **(inline)** | `255, 40, 40` | collision highlight | [rs_ik_keyframe.py:1849](scripts/rs_ik_keyframe.py#L1849) |
| red **(inline)** | `255, 40, 40` | collision highlight | [rs_show_bar_action_plan.py:758](scripts/rs_show_bar_action_plan.py#L758) |

## Base placement — guides, frame marker, reach

Guide lines ([base_guide_viz.py:38-40](scripts/core/base_guide_viz.py#L38-L40)):

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| teal | `0, 160, 160` | offset 0 — the joint line projected onto the ground | [base_guide_viz.py:38](scripts/core/base_guide_viz.py#L38) |
| light teal | `120, 200, 200` | the 375 / 500 / 625 mm standoff lines + their labels | [base_guide_viz.py:39](scripts/core/base_guide_viz.py#L39) |
| yellow | `255, 255, 0` | the midpoint extension line the base origin sits on | [base_guide_viz.py:40](scripts/core/base_guide_viz.py#L40) |

Base frame marker ([base_frame_viz.py:29-35](scripts/core/base_frame_viz.py#L29-L35)):

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| red | `220, 40, 40` | base +X — the heading (which way the robot faces) | [base_frame_viz.py:30](scripts/core/base_frame_viz.py#L30) |
| green | `40, 180, 40` | base +Y | [base_frame_viz.py:31](scripts/core/base_frame_viz.py#L31) |
| blue | `40, 90, 220` | base +Z (ground normal) | [base_frame_viz.py:32](scripts/core/base_frame_viz.py#L32) |
| grey | `150, 150, 150` | base footprint rectangle | [base_frame_viz.py:34](scripts/core/base_frame_viz.py#L34) |
| light blue | `100, 100, 220` | baked reach circle | [base_frame_viz.py:35](scripts/core/base_frame_viz.py#L35) |

Live conduits during the pick ([dynamic_preview.py](scripts/core/dynamic_preview.py)) — these are
display materials, not object colours, so they vanish when the conduit closes:

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| light blue | `100, 100, 220` | reach outline, clear of obstacles | [dynamic_preview.py:47](scripts/core/dynamic_preview.py#L47) |
| salmon | `255, 100, 100` | reach outline touching an obstacle | [dynamic_preview.py:48](scripts/core/dynamic_preview.py#L48) |
| pale lavender | `180, 180, 220` | robot ghost mesh (default, alpha 0.5) | [dynamic_preview.py:191](scripts/core/dynamic_preview.py#L191), [:316](scripts/core/dynamic_preview.py#L316) |
| sky blue | `120, 200, 255` | arm reach volumes shown with the ghost (alpha 0.20) | [dynamic_preview.py:202](scripts/core/dynamic_preview.py#L202) |
| cyan | `80, 200, 255` | IK sample seed | [dynamic_preview.py:330](scripts/core/dynamic_preview.py#L330) |
| light blue | `100, 100, 220` | IK sample circle | [dynamic_preview.py:331](scripts/core/dynamic_preview.py#L331) |
| salmon | `255, 100, 100` | IK sample failed | [dynamic_preview.py:332](scripts/core/dynamic_preview.py#L332) |
| light green | `100, 220, 100` | IK sample succeeded | [dynamic_preview.py:333](scripts/core/dynamic_preview.py#L333) |

## Ground and environment

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| green | `60, 200, 90` | WalkableGround highlight | [rs_assign_and_show_walkable_ground.py:87](scripts/rs_assign_and_show_walkable_ground.py#L87) |
| green | `60, 200, 90` | WalkableGround highlight (**duplicated constant**) | [rs_show_bar_action_plan.py:86](scripts/rs_show_bar_action_plan.py#L86) |
| green | `60, 179, 60` | env geometry highlighted for IK | [highlight_env.py:22](scripts/core/highlight_env.py#L22) |
| amber | `180, 120, 60` | ground joint preview | [ground_placement.py:84](scripts/core/ground_placement.py#L84) |

## Joint / bar placement previews

`PREVIEW_COLORS` cycles per solved variant — the colour means "variant index", not a state:

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| red / blue / green / amber | `230,80,80` · `80,80,230` · `80,200,80` · `200,160,50` | joint variant 0-3 | [joint_placement.py:55-60](scripts/core/joint_placement.py#L55-L60) |
| red / blue / green / amber | same four | brace candidate index | [rs_bar_brace.py:64-69](scripts/rs_bar_brace.py#L64-L69) |
| red / blue / green / amber | same four | subfloor candidate index | [rs_bar_subfloor.py:79](scripts/rs_bar_subfloor.py#L79) |
| blue | `30, 100, 220` | selected bar | [config.py:86](scripts/core/config.py#L86) |

## Robotic tools

| Colour | RGB / RGBA | Meaning | Defined at |
| --- | --- | --- | --- |
| red | `0.90, 0.10, 0.10, 1.0` | LEFT tool mesh + labels (0-1 floats) | [rs_inspect_robotic_tool.py:79](scripts/rs_inspect_robotic_tool.py#L79) |
| green | `0.10, 0.75, 0.15, 1.0` | RIGHT tool mesh + labels (0-1 floats) | [rs_inspect_robotic_tool.py:80](scripts/rs_inspect_robotic_tool.py#L80) |

## Miscellaneous

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| orange | `255, 128, 0` | mocap bar layer | [rs_read_mocap_bar.py:59](scripts/rs_read_mocap_bar.py#L59) |
| pure blue | `0, 0, 255` | closest segment in RSMeasureGap | [rs_measure_gap.py:21](scripts/rs_measure_gap.py#L21) |
| light grey **(inline)** | `200, 200, 200` | RSBarEdit fallback colour | [rs_bar_edit.py:93](scripts/rs_bar_edit.py#L93) |

---

## Collisions and near-duplicates

Ordered by how likely each is to mislead someone reading the screen.

### 1. Green means three different things

`60, 179, 60` is **built bar** in RSSequenceEdit and **env geometry** in the IK highlight;
`60, 200, 90` is **WalkableGround**; `40, 180, 40` is the base **+Y axis**; `0.10, 0.75, 0.15`
is the **right tool**. Four near-identical greens across four unrelated concepts. In RSIKKeyframe
you can see built bars, highlighted env and walkable ground *at the same time*.

**Suggested:** keep green for "already built / already there" (built bar + env), move
WalkableGround to a distinctly different hue, and leave the axis triad alone (RGB-for-XYZ is a
universal convention worth keeping).

### 2. `60, 200, 90` is declared twice

[rs_assign_and_show_walkable_ground.py:87](scripts/rs_assign_and_show_walkable_ground.py#L87) and
[rs_show_bar_action_plan.py:86](scripts/rs_show_bar_action_plan.py#L86) define the same literal
independently. Change one and the two commands silently disagree. **Suggested:** move it to
`config.py` and import it in both.

### 3. Blue is both a state and an axis

`30, 100, 220` is the active step *and* the selected bar; `40, 90, 220` is base +Z;
`100, 100, 220` is the reach circle. **Suggested:** at minimum, keep the reach circle visually
distinct from the active bar — they co-occur throughout the base pick.

### 4. Two reds for two unrelated failures

`255, 40, 40` (inline, twice) is a collision; `220, 40, 40` is the base heading axis; `230, 115,
150` is IK-failed *and* fake-bar; `255, 100, 100` is reach-touching-obstacle. **Suggested:** one
red for "collision / blocked", one for "failed", and stop writing the collision red inline.

The IK-failed / fake-bar overlap is deliberate rather than drift — both read as "the robot is not
building this one" — and the two cannot co-occur, because the IK overlay skips fake bars. The two
literals are declared separately though ([:661](scripts/core/rhino_bar_registry.py#L661),
[:1671](scripts/core/rhino_bar_registry.py#L1671)), so changing one silently splits them.

RSIKKeyframe's rejected-pick flag is a third user of the same pink, reading the same way. It is the
one case painted on **joint blocks** rather than bars, and the only one that is undone rather than
cleared: it snapshots each block's colour first and puts it back
([rhino_bar_registry.py:1621](scripts/core/rhino_bar_registry.py#L1621)), because the blocks it
marks may already carry a sequence colour or a broken-link mark that a reset-to-ByLayer would eat.

### 5. The three inline literals

[rs_ik_keyframe.py:1849](scripts/rs_ik_keyframe.py#L1849),
[rs_show_bar_action_plan.py:758](scripts/rs_show_bar_action_plan.py#L758) and
[rs_bar_edit.py:93](scripts/rs_bar_edit.py#L93) hard-code colours at the call site. The first two
are the *same* colour for the *same* concept in two files — the classic drift setup.
**Suggested:** promote to a named `COLLISION_COLOR` in `config.py`.

### 6. Teal is overloaded

`40, 170, 160` is an unstable bar (RSSequenceEdit) and `0, 160, 160` is the projected joint line
(base guides). Different commands, so low practical risk — noted for completeness.

### 7. Palette cycles read as state

`PREVIEW_COLORS` uses red/blue/green/amber to mean *variant 0-3*, which collides with red=failure
and green=built elsewhere. It is only on screen during an interactive pick, so the risk is
contained — but a red joint preview does not mean anything is wrong.

---

## Not yet assigned

Colours these planned features will need, listed here so they are chosen against the table above
rather than in isolation:

- ~~**fake bars**~~ — assigned: pink `230, 115, 150`, `SEQ_COLOR_FAKE`, shared with IK-failed
  (see [Two reds](#4-two-reds-for-two-unrelated-failures)). Survives RSClearColorPreview.
- **joint with broken user text** (`no_joint_id` / `duplicate_joint_id` from the RSUpdatePreview
  tool-restore pass) — must NOT reuse orange `175, 55, 10`, which already means "parent bar is
  gone". Different fault, different colour.