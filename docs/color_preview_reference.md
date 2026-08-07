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

Defined in [rhino_bar_registry.py:609-619](scripts/core/rhino_bar_registry.py#L609-L619),
applied by `show_sequence_colors` ([:715](scripts/core/rhino_bar_registry.py#L715)).

| Colour | RGB | Meaning | Defined at |
| --- | --- | --- | --- |
| green | `60, 179, 60` | built — earlier than the active step | [rhino_bar_registry.py:609](scripts/core/rhino_bar_registry.py#L609) |
| blue | `30, 100, 220` | the active step | [rhino_bar_registry.py:611](scripts/core/rhino_bar_registry.py#L611) |
| grey | `160, 160, 160` | unbuilt — later than the active step | [rhino_bar_registry.py:613](scripts/core/rhino_bar_registry.py#L613) |
| teal | `40, 170, 160` | built but still unstable (its supports are not up yet) | [rhino_bar_registry.py:616](scripts/core/rhino_bar_registry.py#L616) |
| purple | `110, 40, 160` | support bar of the active step — shown in the normal view, and while the EditSupports picker is open | [rhino_bar_registry.py:658](scripts/core/rhino_bar_registry.py#L658) |
| pink | `230, 115, 150` | fake bar — staging that will not be fabricated (same pink as IK-failed, deliberately) | [rhino_bar_registry.py:661](scripts/core/rhino_bar_registry.py#L661) |

Precedence when a bar qualifies for more than one, loosest to tightest: sequence state → fake →
support. The active bar always keeps its blue.

The fake pink is also what `RSBarEdit > FakeBar` paints: entering the mode highlights every bar
already marked fake, and each Add / Delete repaints that bar immediately, so the pink on screen
is always the current mark set. It is left on at exit, and survives RSClearColorPreview.

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
| grey-blue | `75, 120, 150` | bar has a solved IK keyframe | [rhino_bar_registry.py:1612](scripts/core/rhino_bar_registry.py#L1612) |
| pink | `230, 115, 150` | IK attempted at the placed base and failed — and, by design, also the fake-bar tint | [rhino_bar_registry.py:1613](scripts/core/rhino_bar_registry.py#L1613) |
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
[:1613](scripts/core/rhino_bar_registry.py#L1613)), so changing one silently splits them.

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