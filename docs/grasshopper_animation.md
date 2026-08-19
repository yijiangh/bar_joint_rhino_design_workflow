# Grasshopper animation: assembly replay + camera control

Three Grasshopper Python 3 components that together turn the assembly plan into
something you can film:

| Component | File to paste | What it does |
| --- | --- | --- |
| `RSGHSequencePreview` | [scripts/gh/RSGHSequencePreview.py](../scripts/gh/RSGHSequencePreview.py) | one slider walks the **whole assembly timeline** — bars, joints, guide lines, the dual-arm robot and the support robots |
| `RSGHPreviewWarmup` | [scripts/gh/RSGHPreviewWarmup.py](../scripts/gh/RSGHPreviewWarmup.py) | one button, once per Rhino session: pre-loads the three robot models so the first scrub has no stall |
| `RSGHCameraControl` | [scripts/gh/RSGHCameraControl.py](../scripts/gh/RSGHCameraControl.py) | drives the Rhino viewport camera from GH points |

None is an `RS*` toolbar command, so none appears in
[rhino_toolbar_entrypoints.md](rhino_toolbar_entrypoints.md) or in
`scaffolding_toolbar.rui`.

The files above are **shims**: the real logic lives in
[scripts/core/gh_seq_preview.py](../scripts/core/gh_seq_preview.py),
[scripts/core/gh_warmup.py](../scripts/core/gh_warmup.py),
[scripts/core/gh_camera.py](../scripts/core/gh_camera.py) and the shared
[scripts/core/gh_bridge.py](../scripts/core/gh_bridge.py), and is re-imported on every
solve — so you edit those and the canvas picks it up without re-pasting anything.

The timeline itself is **not** re-implemented here: `gh_seq_preview` imports
[rs_show_bar_action_plan.py](../scripts/rs_show_bar_action_plan.py) and calls the same
`build_global_timeline` / `support_presence_for_step` / payload readers the
`RSShowAssemblyPlan` command uses, so the two viewers cannot drift apart. That module is
imported **once per session** (it is heavy); after editing it, reload the CPython engine.

---

## Building the components

All three are written in **SDK mode**: the component's inputs and outputs are generated
from the `RunScript` signature, so pasting one file produces a fully wired component.

1. Drop a **Python 3 Script** component on the canvas, open the Script Editor.
2. Click **Convert To GH_ScriptInstance** on the editor dashboard.
3. Select all (Ctrl+A), **delete**, then paste the file into the empty editor and close it.
4. The params appear. Wire the sources (Button, toggles, panels) and go.

> **⚠ The shim files must stay SHORT, and their prose must never name the code
> structure.** The paste-time parameter sync scans the whole pasted text for the
> component structure, and it is **not comment- or docstring-aware**: an earlier version
> of these files carried long docstrings that *talked about* the `RunScript` method and
> the class line, and the parser found those mentions inside the prose, rewrote around
> the wrong lines, and every paste failed with `unindent does not match any outer
> indentation level` — even a clean delete-then-paste. Deleting the prose fixed all
> three components (found empirically, 2026-08-19). Hence the standing rule: shim
> docstrings carry only a summary + INPUTS/OUTPUTS/NOTES, and the long instructions
> live HERE. Do not move them back.

> **If a paste still fails with the unindent error:** the editor has dedented the
> method definition out of the class. Fix the *editor's* copy: re-indent that line with
> **4 spaces typed on the spacebar — not the Tab key** (a tab character does not match
> spaces even though it looks identical), so it sits under the class line, body lines
> 8 spaces. Keep the signature text exactly as the editor rewrote it (with the .NET
> type names) and never delete the class line. Script-mode variants of both files —
> which avoid the mechanism entirely at the cost of naming every param by hand — exist
> in git history.

### RSGHSequencePreview parameters

9 inputs, generated from the signature:

| Input | Type | Wire it to | Meaning |
| --- | --- | --- | --- |
| `enable` | bool | Boolean Toggle | master switch; False restores the document and returns |
| `reload` | bool | **Button** | rebuild the timeline and the `step` slider |
| `step` | int | *nothing* | frame index — the component creates this slider itself |
| `show_unbuilt` | bool | Boolean Toggle | gray centerlines of not-yet-built bars |
| `show_current` | bool | Boolean Toggle | blue centerline on the active bar |
| `show_support` | bool | Boolean Toggle | purple centerlines on the supporting bars |
| `line_thickness` | float | Number Slider (1–6) | guide-line width in screen **pixels** |
| `line_style` | str | Panel: `continuous` / `dashed` | guide-line style |
| `dash_pattern` | str | Panel: `40,20` | dash/gap in mm along the bar |

6 outputs: `bar_id`, `seq`, `pose`, `frame_count`, `step_label`, `info`.

`step_label` is the caption — `step 75/88 | bar B37 | release B21` — identical to what
`RSShowAssemblyPlan` prints, so you can compare the two viewers step for step. Read
`info` when something looks wrong: skipped renders, missing keyframes, bars that
contribute no steps and undrawable support robots are all reported there.

Press `reload` once after pasting: the `step` slider appears, already ranged
`0 … frames-1`. **Right-click that slider → Animate…** is the film.

### RSGHPreviewWarmup parameters

1 input: `run` (bool, wire a **Button**). 1 output: `info`.

### RSGHCameraControl parameters

| Input | Type hint | Meaning |
| --- | --- | --- |
| `camera` | Point3d | camera position, **document units** |
| `target` | Point3d | what it looks at, document units |
| `lens` | float | 35mm-equivalent focal length; 50 is Rhino's default |
| `up` | Vector3d | camera up; empty = world Z |
| `active` | bool | False = no-op, the viewport is left alone |
| `viewport` | str | named viewport; empty = the active view |

Outputs: `ok`, `info`.

Points are in **document units** and are passed straight through. The repo's millimetre
convention ([coordinate_conventions.md](coordinate_conventions.md)) applies to robot and
IK data — base frames, joint payloads — not to viewport geometry.

---

## The frame index

One frame is one **step of the global assembly timeline** — the same list
`RSShowAssemblyPlan` walks, in assembly-sequence order. Each bar contributes:

| pose | the scene is | drawn from |
| --- | --- | --- |
| `approach` | arms at the approach keyframe, bar gripped | `assembly_ik_approach` user text |
| `assembled` | bar inserted, arms still gripping | `assembly_ik_assembled` user text |
| `hold` | *(held bars only)* the bar's own support robot grips too | assembled keyframe + that bar's support keyframe |
| `retreat` | arms pulled back, bar released | `assembly_ik_retreat` user text |
| `home` | arms at the fixed home pose | `config.HOME_CONF_LEFT_6` / `HOME_CONF_RIGHT_6` |
| `release X` | a hold whose LAST stabilizing bar is this one lets go | X's support keyframe (approach config = retreated) |

plus the robot base frame from `assembly_robot_base_frame_world_mm`. All of it is read
from the bar curves' user text in **this document** — nothing is loaded from the exported
`BarActions/*.json`, which are for the offline planner.

So the count is not `bars × poses`; it is the concatenation of each bar's own pose cycle.
For the `260814_RobArch_support_ik` study: 20 bars × 4 poses + 4 hold steps + 4 release
steps = **88 frames**.

A `release` frame is a pair (`("release", "B21")`), not a plain name — that is how the
code tells it apart from an ordinary pose. On those frames the assembly robot has driven
away and its layer is hidden; the built structure stays on screen because it is the
document's own geometry, not part of the robot preview.

Bars that are fake, or have no solved IK keyframe, contribute no steps (their count is
reported in `info`) — but a hold's release step is kept even when its host bar is
unsolved, so a support robot can never silently vanish between frames.

**Every registered bar is considered.** The persistent `HideUnbuilt` latch
(`scaffolding.build_stage`) is deliberately ignored: never read to filter the timeline,
never written. Hiding not-yet-built bars for the camera is the `show_unbuilt` input, a
per-frame view decision.

---

## What is on screen

| Class | Geometry | Line |
| --- | --- | --- |
| built bars | tubes + joint instances, shown | green centerline |
| current bar | tubes + joint instances, shown | blue centerline (`show_current`) |
| unbuilt bars | **never shown** | gray centerline (`show_unbuilt`) |
| supporting bars | per the rules above | purple centerline (`show_support`) |
| fake (staging) bars | **never shown** | — |
| robotic tool instances | **never shown** (layer hidden) | — |

This is the filming split: real model geometry for what has been built, guide lines for
everything else. Geometry visibility is driven by `show_sequence_colors`
([rhino_bar_registry.py](../scripts/core/rhino_bar_registry.py)) arguments that default
to the old behaviour everywhere else — `geom_built_and_active_only=True`,
`show_fake=False`, all tints off.

**The guide lines are a display-conduit overlay, not document curves.** A
`Rhino.Display.DisplayConduit` draws them on top of the geometry every repaint: exact
RGB colours in *any* display mode, thickness in screen pixels, dashes drawn as real
segments (pattern in mm along the bar). The document's own centerline curves are hidden
while the preview runs — the overlay replaces them. A first implementation styled the
document curves (object colours + print widths + PrintDisplay) and failed exactly where
it mattered: **PrintDisplay replaces the user's display mode with the print preview and
renders every curve in its print colour (black)**, killing the class colours, the
dashes and the custom display style in one stroke. The conduit touches none of that —
the viewport renders exactly as the user configured it. Being depth-less, the overlay
also stays readable where a line runs inside a built bar's tube (the purple line of an
already-built support, the blue line of the current bar).

Robots: the dual-arm robot on the `Assembly` layer, and each support robot on its own
`Support Alice` / `Support Belle` layer — the same layer keys the command viewer uses.
A support robot appears when its hold starts, stands frozen while it holds, shows
retreated on its release frame, and is hidden from the next frame on.

---

## No `RSPBStart` needed

Every robot is drawn by **forward kinematics only** — `ik_viz.update_state` with no
`set_cell_state`. Nothing in the component or its import chain touches PyBullet, so it
works on a freshly opened `.3dm` with no `RSPBStart`.

Two consequences:

- **No collision checking.** This is a viewer. A frame can look fine and still be a pose
  the planner would reject.
- **The arms carry their assembly tools automatically.** The component registers the
  active pair's ToolModels itself via `robot_cell.ensure_arm_tool_models` — built from
  `robotic_tools.json` + the tools' exported collision OBJs, PyBullet-free — so no
  planning command needs to have run first. Only when a tool's OBJ export is missing do
  the arms fall back to flange-only, reported in `info`.

---

## Speed: what is slow, and what the warm-up fixes

The only genuinely heavy work is **loading the three robot models and baking their
display meshes** — one-time per Rhino session, cached in `sc.sticky`. Everything else per
frame is a dictionary lookup plus mesh transforms.

`RSGHPreviewWarmup` pays that cost up front: click its Button once while setting up the
canvas and it loads Cindy, Alice and Belle and bakes each one's meshes into the same
cache entries the preview renders from, leaving the layers hidden. Skipping it is legal —
the preview then stalls once on the first frame that needs each robot (support robots are
only reached at hold/release steps, so that stall can arrive well into a scrub).

Three more things keep scrubbing cheap, all in `gh_seq_preview.run`:

- **The timeline is cached** in sticky and rebuilt only on the `reload` button or when
  the bar count changes — a slider move never re-derives the hold plan.
- **A render-skip fingerprint**: GH re-solves on any upstream tick or canvas move, and if
  nothing that affects the picture changed, the whole render is skipped
  (`info` says `unchanged (render skipped)`).
- **The heavy viewer module is imported once**, never `importlib.reload`ed per solve.

---

## Layer booleans vs. manual layer toggling

The component writes exactly **two** layers' visibility, each recorded once per enable
cycle and restored on disable: the robotic-tool layer and the bar-centerline layer —
both hidden (tools are never shown in the filming view; the guide-line overlay replaces
the centerline curves, which would otherwise draw a second, display-mode-coloured line
under every overlay line). Everything else is per-object visibility.

This matters because a component that re-asserted layer state on every solve would
overwrite anything you toggled by hand in Rhino's layer panel the instant you moved the
slider. Prior visibility is recorded on the **first** change only, so the component can
never record its own hidden state and then "restore" it.

## Troubleshooting: "the guide lines do not show / look wrong"

1. **No lines at all** — check `enable` is True and `frame_count` > 0, then read `info`:
   skipped bars and errors are reported there. The overlay draws only for classes whose
   boolean is on, and built bars deliberately carry no line (their geometry shows).
2. **Dashes look continuous** — the pattern is in mm along the bar; at a wide zoom
   `4,2` is sub-pixel. Use `40,20` or coarser.
3. **The viewport goes white/black and ugly** — that is Rhino's *PrintDisplay* preview,
   left on by an older build of this component or by hand. Run `_PrintDisplay` →
   `State=Off`. The current component never touches it.
4. **The arms have no tools** — `info` will carry a `flange-only` note naming the
   missing OBJ export; re-run `RSDefineRoboticTool` (AssemblyTool mode) for that tool.

---

## Cleanup contract

Set `enable=False` (or disable, then delete the component) and the document goes back to
how it was found:

- bar / joint / tool colours reset to ByLayer, and per-object **line width and linetype**
  reset to by-layer (`reset_sequence_colors`);
- everything hidden is shown again, then the build-stage latch gets the final word;
- the tool and centerline layers' prior visibility is restored;
- the guide-line overlay is disabled (and **PrintDisplay** switched off if a
  pre-overlay build of the component had left it on);
- the IK cache layers (`Assembly`, `Support *`) are hidden and the `ik_viz` session ended;
- **no `scaffolding.build_stage` document string is written**, so `RSSequenceEdit` will
  not resume at a latched bar afterwards.

State lives in `sc.sticky`, keyed by the component's `InstanceGuid`, so two pasted copies
each clean up after themselves — but two *enabled* copies still fight over one Rhino
document, and GH's solve order decides who wins. Enable one at a time.

---

## Composing into an animation

Drive both from one slider:

```
                  ┌─────────────────────────┐
  step slider ────┤ RSGHSequencePreview     ├── frame_count
        │         └─────────────────────────┘
        │
        └──> A/B (step / frame_count) ──> Curve|Evaluate ──> camera
                                                              │
                                     target (Point) ──┐       │
                                                  ┌───┴───────┴──┐
                                                  │ RSGHCamera   │
                                                  │ Control      │
                                                  └──────────────┘
```

Normalise the slider (`step / frame_count`) to get the curve parameter, so the camera
sweeps its path exactly once over the build. Then **right-click the `step` slider →
Animate…** to write the frames out; neither component writes image files itself.

Set up the viewport's display mode before filming — the components only decide *what* is
on screen, never how it is shaded.

---

## Verification

Rhino-dependent, so this is a manual pass; there is no pytest coverage for any of it.

1. **No regression on existing commands** — run `RSSequenceEdit`, step a few bars, Esc.
   Colours and visibility must behave exactly as before (this proves the new
   `show_fake` / `tint_curves_only` / `geom_built_and_active_only` / `line_style`
   defaults are true no-ops). Same for `RSUpdatePreview` and `RSShowBarActionPlan`.
2. **Param setup** — paste per the steps above; 9 inputs and 6 outputs must appear on
   the component by themselves. If the editor errors on paste, see the unindent note
   in "Building the components".
3. **Frame count** — press `reload`; the slider appears ranged 0..87 on the study file,
   and `frame_count` reads 88.
4. **Captions match the command** — spot-check `step_label` against
   `RSShowAssemblyPlan`'s printed caption at step 1, a hold step, both release pairs and
   the last step. They must be identical strings.
5. **Support robots** — scrub to a `hold` step: the bar's holder appears. Continue to its
   `release` step: that robot shows retreated. One step later: it is gone, and any robot
   still holding is unchanged.
6. **No PyBullet** — fresh Rhino, no `RSPBStart`, repeat 3–5.
7. **Warm-up** — fresh session, click `RSGHPreviewWarmup` once (`info` reports all three
   robots), then scrub: no stall on the first hold step.
8. **Animate** — right-click the `step` slider → Animate…; the frames come out in order.
9. **Lines** — `line_thickness=3`, `line_style=dashed`, `dash_pattern=40,20`: the guide
   lines get thick and coarsely dashed, in gray/blue/purple, in YOUR display mode. Flip
   `show_unbuilt` / `show_current` / `show_support` and confirm only that class changes,
   and that a purple support line stays readable over a built bar's tube.
10. **Never shown** — confirm fake bars and robotic tool instances stay off screen
    throughout, and that unbuilt bars show a line but no tube/joints.
11. **Manual layer toggling** — switch a layer on by hand mid-preview and move the
    slider: your change must survive.
12. **Cleanup** — `enable=False`, then delete the component; check the list above,
    including that PrintDisplay went back off and no linetype/width remains on any curve.
13. **Camera** — two points and `lens=50`, `active=True`; the Perspective view jumps.
    Supply a named viewport and confirm it targets that one instead.