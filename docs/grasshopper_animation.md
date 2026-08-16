# Grasshopper animation: sequence replay + camera control

Two Grasshopper Python 3 components that together turn the assembly sequence into
something you can film:

| Component | File to paste | What it does |
| --- | --- | --- |
| `RSGHSequencePreview` | [scripts/gh/RSGHSequencePreview.py](../scripts/gh/RSGHSequencePreview.py) | one slider walks the whole build order; drives bars, joints, colours and the dual-arm robot |
| `RSGHCameraControl` | [scripts/gh/RSGHCameraControl.py](../scripts/gh/RSGHCameraControl.py) | drives the Rhino viewport camera from GH points |

Neither is an `RS*` toolbar command, so neither appears in
[rhino_toolbar_entrypoints.md](rhino_toolbar_entrypoints.md) or in
`scaffolding_toolbar.rui`.

The thin files above are shims: the real logic lives in
[scripts/core/gh_seq_preview.py](../scripts/core/gh_seq_preview.py),
[scripts/core/gh_camera.py](../scripts/core/gh_camera.py) and the shared
[scripts/core/gh_bridge.py](../scripts/core/gh_bridge.py), and is re-imported on every
solve — so you edit those and the canvas picks it up without re-pasting anything.

---

## Building the components

Both are written in **SDK mode**, which means the component's inputs and outputs are
generated from the `RunScript` signature. You never click `+` or type a nickname.

1. Drop a **Python 3 Script** component on the canvas, open the Script Editor.
2. Click **Convert To GH_ScriptInstance** on the editor dashboard.
3. Select all (Ctrl+A), **delete**, then paste the file into the empty editor and close it.
4. The params appear. Wire a **Button** to `reload` (SDK mode can generate the
   parameter but not the Button — that's a separate component) and press it once.

> **The editor rewrites the `RunScript` signature — let it.** In SDK mode the Script Editor
> keeps the params and the signature in sync by editing that line in place. It converts
> Python type hints to .NET ones (`poses: List[str]` → `poses:
> System.Collections.Generic.List[object]`), which is why both files carry an
> `import System` that nothing visibly uses — without it you get `undefined name 'System'`.
> The rewrite can also **dedent `def RunScript` out of the class**, which shows up as
> `unindent does not match any outer indentation level` pointing at the def. The file on
> disk is fine when that happens; the editor's copy is not. Re-indent the def to 4 spaces
> under the `class` line, leave the signature text as the editor wrote it, and never delete
> the `class` line. Full write-up in [Su_note §25](Su_note.md#25-grasshopper-components--what-is-different-from-a-rhino-command).

Step 4 creates the `step` slider for you, already ranged to this document's frame
count. See [Su_note §25](Su_note.md#25-grasshopper-components--what-is-different-from-a-rhino-command)
for what SDK mode is doing and why the slider has to be created the way it is.

### RSGHSequencePreview parameters

| Input | Type | Meaning |
| --- | --- | --- |
| `enable` | bool | master switch; False restores the document and returns |
| `reload` | bool (Button) | rebuild the `step` slider from the current frame count |
| `step` | int | frame index — the auto-created slider |
| `poses` | List[str] | which movements to visit per bar, in play order; default `M3` |
| `show_unbuilt` | bool | show bars later in the sequence than the active one |
| `show_bars_and_joints` | bool | document bar/joint layer visibility |
| `show_assemble_robot` | bool | draw the dual-arm robot at this frame |
| `show_support_robot` | bool | accepted but inert — see [below](#the-support-robot-is-a-stub) |
| `preview_color_unbuilt` | bool | grey tint on / off |
| `preview_color_built` | bool | green tint on / off |
| `preview_color_current` | bool | blue tint on / off |
| `preview_color_support` | bool | purple tint on / off |

Outputs: `bar_id`, `seq`, `pose`, `frame_count`, `info`. Read `info` when something looks
wrong — every skipped render, missing keyframe and ignored input is reported there.

`show_unbuilt` and `preview_color_unbuilt` are **not** the same switch.
`preview_color_unbuilt=False` only drops the grey tint (the bars go ByLayer and stay on
screen); `show_unbuilt=False` actually hides them. For a clean shot of the structure
rising you normally want `show_unbuilt=False` and all four colour switches off.

### RSGHCameraControl parameters

| Input | Type | Meaning |
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

One frame is one `(bar, pose)` pair. `poses` picks which of the robot's four movements to
visit at each bar:

| pose | the robot is | source |
| --- | --- | --- |
| `M1` | home, bar gripped | the bar's saved base frame + `config.HOME_CONF_LEFT_6` / `HOME_CONF_RIGHT_6` |
| `M2` | approaching, bar held | `assembly_ik_approach` user text |
| `M3` | assembled, bar released | `assembly_ik_assembled` user text |
| `M4` | retreating | `assembly_ik_retreat` user text |

Frames are ordered **bar-major** — the bar is the slow-moving digit and the pose the fast
one, like a two-digit odometer. With 20 bars and `poses = ["M1", "M4"]`:

```
idx        0        1        2        3        4        5      ...
bar     bar[0]   bar[0]   bar[1]   bar[1]   bar[2]   bar[2]
pose      M1       M4       M1       M4       M1       M4
```

```python
n_frames  = len(bars) * len(poses)       # 20 * 2 = 40
bar_index = idx // len(poses)            # 3 // 2 = 1 -> bar[1]
pose      = poses[idx % len(poses)]      # 3 %  2 = 1 -> "M4"
```

`//` ticks up once per complete bar; `%` cycles through the poses forever. Leaving `poses`
at its `M3` default gives one frame per bar — the moment each one lands.

**Every registered bar is always in the list.** The persistent `HideUnbuilt` latch
(`scaffolding.build_stage`) is deliberately ignored here: it is never read to filter the
sequence and never written. Hiding not-yet-built bars for the camera is the `show_unbuilt`
input, which is a per-frame view decision.

---

## No `RSPBStart` needed

The robot is drawn by **forward kinematics only** — `ik_viz.update_state` with no
`set_cell_state` — which is the same render path as `RSShowBarActionPlan`'s motion viewer
([rs_show_bar_action_plan.py:443-474](../scripts/rs_show_bar_action_plan.py#L443-L474)).
Nothing here touches PyBullet or the planner, so the component works on a freshly opened
`.3dm` with no `RSPBStart`.

Two consequences:

- **No collision checking.** This is a viewer. A frame can look fine and still be a pose
  the planner would reject.
- **The arms may render flange-only.** Tools are attached only when
  `rcell.tool_models` is already populated, i.e. when `RSIKKeyframe` or
  `RSRebuildRobotCell` has run in this Rhino session. The component reports which path it
  took in `info` (`arms: tools attached` / `arms: flange-only`) and never mutates the
  shared cached cell to force the issue. Flange-only is not a hole in the picture — the
  document's own tool block instances are still on screen for the active step.

A bar with no `assembly_ik_assembled` record reports that in `info` and hides the robot
for that frame rather than raising.

---

## The support robot is a stub

`show_support_robot` is accepted, always hides the support layer, and notes itself as
pending in `info`. The support pipeline is not ready: `rs_ik_support_keyframe.py` is
archived and off the toolbar, the `KEY_SUPPORT_*` split is unwired, and essentially no bar
in the current file carries an `ik_support` record.

Wiring it later means filling in `_render_support_robot` in
[gh_seq_preview.py](../scripts/core/gh_seq_preview.py) — same `begin_session` /
`update_state` calls as the assemble robot, only `layer_key=LAYER_KEY_SUPPORT` and the
support cell. No structural change to `run()`.

---

## Cleanup contract

Set `enable=False` (or delete the component after disabling it) and the document goes
back to how it was found:

- bar / joint / tool colours reset to ByLayer (`reset_sequence_colors`);
- every layer the component hid is made visible again — it records their prior visibility
  on the first change only, so it cannot record its own hidden state;
- the IK cache layers are hidden and the `ik_viz` session ended;
- **no `scaffolding.build_stage` document string is written**, so `RSSequenceEdit` will
  not resume at a latched bar afterwards.

The state that makes this possible lives in `sc.sticky`, keyed by the component's
`InstanceGuid`. Two copies of the pasted code therefore each clean up after themselves —
but two *enabled* copies still fight over one Rhino document, and GH's solve order decides
who wins. Enable one at a time.

---

## Composing the two into an animation

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
sweeps its path exactly once over the build. Then record the viewport with whatever
capture tool you prefer — neither component writes image files.

---

## Verification

Rhino-dependent, so this is a manual pass; there is no pytest coverage for any of it.

1. **No regression on existing commands** — run `RSSequenceEdit`, step a few bars, Esc.
   Colours and visibility must behave exactly as before (this proves the `color_flags=None`
   default is a true no-op). Same for `RSUpdatePreview`.
2. **Colours only** — `enable=True`, `show_assemble_robot=False`. Press `reload`: a slider
   appears wired to `step`. Drag it; `bar_id` walks the build order and the blue "current"
   bar tracks it. Flip each `preview_color_*` off in turn and confirm only that class drops
   to its layer colour while staying visible.
3. **`poses` ordering** — feed a panel with `M1` then `M4`. `frame_count` doubles, and
   `step` 0→3 reports `(seq n, M1) → (seq n, M4) → (seq n+1, M1) → (seq n+1, M4)`.
4. **Robot, with PyBullet not started** — `show_assemble_robot=True`. The dual-arm robot
   appears at the bar's saved base frame in the selected pose. Cross-check one frame
   against `RSShowBarActionPlan` on the same bar and pose — same posture.
5. **Support stub** — `show_support_robot=True` must not raise and must not draw anything.
6. **Cleanup** — `enable=False`, then delete the component; check the list above.
7. **Camera** — two points and `lens=50`, `active=True`; the Perspective view jumps.
   Supply a named viewport and confirm it targets that one instead.
