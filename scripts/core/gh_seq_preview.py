"""Logic behind the ``RSGHSequencePreview`` Grasshopper component.

What it is
----------
A *viewer* that turns the document's WHOLE assembly plan into a scrubbable
frame index, so the Rhino viewport can be driven from a Grasshopper slider
(and therefore animated with the slider's built-in right-click ``Animate``).

One frame = one step of the global assembly timeline -- the SAME step list
the ``RSShowAssemblyPlan`` command walks (``core.plan_steps.timeline`` of the
document read as a schema 2 process): every bar contributes approach / assembled
/ (hold) / retreat / home, and support-robot releases appear as their own
steps.  In the current study file that is 88 steps: 20 bars x 4 poses + 4 hold
+ 4 release.  The caption output (``step 75/88 | bar B37 | release B21``)
matches the command's captions one-for-one, so the two viewers can be compared
step by step.

What is drawn per frame
-----------------------
* the document's own bar tubes + joint instances for BUILT bars and the
  CURRENT bar (real geometry, in whatever display mode the user set);
* guide lines as a **display-conduit overlay** -- gray future bars
  (``show_unbuilt``), blue current bar (``show_current``), purple supports
  (``show_support``) -- drawn ON TOP of the geometry with user-set pixel
  width and dash pattern.  The document's centerline curves are hidden while
  the preview runs: the overlay replaces them;
* every robot where the process has it at this step -- the dual-arm assembly
  robot WITH its assembly tools, the support robots holding bars (frozen
  holders, the bar's own holder from the hold pose onward, release retreats)
  -- the step's scene drawn part by part (``core.ik_viz``), on the
  ``Assembly`` preview layer;
* fake (staging) bars and robotic tool instances are NEVER shown.

Why a conduit and not object attributes
---------------------------------------
The first implementation tinted the centerline curves and gave them print
widths, with the viewport's PrintDisplay switched on to show them.  That was
wrong for filming: PrintDisplay REPLACES the user's display mode with the
print preview -- everything renders in print colours (black), which killed
the class colours, the dashes AND the custom display style all at once.  A
``Rhino.Display.DisplayConduit`` draws pure screen overlay: exact colours in
any display mode, pixel thickness with no PrintDisplay, dashes drawn as real
segments, no document attributes to restore -- and the lines stay visible
even where a centerline runs INSIDE its bar's tube.

What it deliberately does NOT do
--------------------------------
* **Never starts PyBullet.**  All robots are drawn by forward kinematics only,
  from the core's scene parts (``core.ik_viz``; no compas, no PyBullet).
* **Never renumbers a sequence** (``repair_on_entry`` is never called) and
  **never writes the build-stage latch**.
* **Never touches the display mode.**  No PrintDisplay, no display-mode
  switches -- the viewport renders exactly as the user configured it.
* **Never fights manual layer toggling.**  This component writes exactly two
  layers, each recorded once per enable cycle and restored at disable: the
  robotic-tool layer and the bar-centerline layer (both hidden -- tools are
  never shown, and the overlay replaces the centerlines).  Everything else is
  per-object visibility.

Everything it changes is restored by :func:`_teardown` (``enable=False``):
colours, visibility, the two layers, the overlay, and the robot preview
layers.

Speed notes
-----------
The process and its step list are computed on the ``reload`` button and cached;
a slider move builds that step's scene, repaints the sequence visibility and
re-poses the robots -- no PyBullet, no document scans beyond the bar registry.
The robot models themselves are loaded once per Rhino session (run
``RSGHPreviewWarmup`` once to pay that cost up front); after that, scrubbing is
mesh-transform only.
"""

from __future__ import annotations

import re

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
import session_log

from core import config
from core import core_lib  # noqa: F401  (puts bar_assembly_core on sys.path; never reloaded)
from core import process_source
from core import gh_bridge
from core import ik_viz
from core import plan_steps
from core.rhino_bar_registry import (
    SEQ_COLOR_ACTIVE,
    SEQ_COLOR_SUPPORT_PICK,
    SEQ_COLOR_UNBUILT,
    get_bar_seq_map,
    get_fake_bar_ids,
    get_supported_until,
    reset_sequence_colors,
    show_sequence_colors,
)

# ---------------------------------------------------------------------------
# Input coercion
# ---------------------------------------------------------------------------


def _unwrap(value):
    """Unwrap a Grasshopper type wrapper (``GH_Boolean`` / ``GH_Number`` / ...).

    An input whose Type hint is left unset hands over GH's own wrapper object
    instead of a Python value -- and ``bool(GH_Boolean(False))`` is **True**,
    because it is a non-empty object.  ``.Value`` is the underlying Python
    value; anything already plain passes straight through.
    """
    return getattr(value, "Value", value)


def _flag(value, default):
    """Coerce a GH boolean input to a real bool, treating ``None`` as *default*.

    An **unconnected** Grasshopper input arrives as ``None``, not as the Python
    default in the signature -- the host passes the argument explicitly, so the
    default never applies.  Left alone, every unwired switch would read as False
    and the component would come up with the whole legend silently off.
    """
    value = _unwrap(value)
    return default if value is None else bool(value)


def _index(value, default=0):
    """Coerce a GH integer input, treating ``None`` (unconnected) as *default*."""
    value = _unwrap(value)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _number(value, default=0.0):
    """Coerce a GH float input, treating ``None`` (unconnected) as *default*."""
    value = _unwrap(value)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _parse_dash_pattern(raw):
    """Parse the ``dash_pattern`` input: ``"40,20"`` -> ``(40.0, 20.0)``.

    Values are DOCUMENT UNITS along the bar (mm in this repo's files).
    Tolerant of spaces and ``x`` as the separator; anything unusable falls
    back to a default readable at structure scale.
    """
    try:
        parts = [p for p in re.split(r"[,x\s]+", str(_unwrap(raw)).strip()) if p]
        if len(parts) >= 2:
            dash, gap = float(parts[0]), float(parts[1])
            if dash > 0 and gap > 0:
                return (dash, gap)
    except (TypeError, ValueError):
        pass
    return (40.0, 20.0)


# ---------------------------------------------------------------------------
# Guide-line overlay (display conduit)
# ---------------------------------------------------------------------------


class _GuideLinesConduit(Rhino.Display.DisplayConduit):
    """Draws the guide lines as a screen overlay, on top of the geometry.

    ``items`` is a list of ``(line, color, thickness_px, dash, gap)`` where
    ``line`` is a ``Rhino.Geometry.Line`` (bar centerlines are straight),
    ``color`` a ``System.Drawing.Color``, thickness in PIXELS, and dash/gap in
    document units (``dash <= 0`` draws continuous).  ``DrawForeground`` is
    depth-less: the lines read over tubes and robots alike, which is exactly
    what a guide overlay is for.
    """

    def __init__(self):
        super().__init__()
        self.items = []

    def DrawForeground(self, e):  # noqa: N802  (RhinoCommon naming)
        try:
            for line, color, thickness, dash, gap in self.items:
                if dash <= 0.0:
                    e.Display.DrawLine(line, color, thickness)
                    continue
                total = line.Length
                if total < 1e-9:
                    continue
                direction = line.Direction
                direction.Unitize()
                t = 0.0
                while t < total:
                    seg_end = min(t + dash, total)
                    a = line.From + direction * t
                    b = line.From + direction * seg_end
                    e.Display.DrawLine(Rhino.Geometry.Line(a, b), color, thickness)
                    t = seg_end + gap
        except Exception:
            # A drawing error must never take down the display pipeline.
            pass


def _ensure_conduit(store):
    """Return the enabled overlay conduit, recreating it after a module reload.

    The shim reloads this module every solve, which redefines the conduit
    CLASS; an instance of the previous class keeps working but would run stale
    draw code.  Recreating on class mismatch keeps the overlay current and
    costs nothing (the object is tiny).
    """
    conduit = store.get("conduit")
    if conduit is not None and type(conduit) is not _GuideLinesConduit:
        conduit.Enabled = False
        conduit = None
    if conduit is None:
        conduit = _GuideLinesConduit()
        store["conduit"] = conduit
    conduit.Enabled = True
    return conduit


def _drop_conduit(store):
    conduit = store.pop("conduit", None)
    if conduit is not None:
        conduit.Enabled = False


def _update_guide_lines(store, bar_map, active_bar_id, active_seq,
                        show_unbuilt, show_current, show_support,
                        thickness_px, dashed, pattern):
    """Rebuild the overlay's line list for this frame.

    Line classes (built bars carry no line -- their real geometry is shown):
    blue = the current bar; purple = the current bar's declared supports (on
    top of built geometry too -- the overlay is depth-less, so a support that
    is already built still reads); gray = not-yet-built bars.
    """
    import System.Drawing

    def _color(rgb):
        return System.Drawing.Color.FromArgb(*rgb)

    colors = {
        "current": _color(SEQ_COLOR_ACTIVE),
        "support": _color(SEQ_COLOR_SUPPORT_PICK),
        "unbuilt": _color(SEQ_COLOR_UNBUILT),
    }
    thickness = max(1, int(round(thickness_px))) if thickness_px else 1
    dash, gap = pattern if dashed else (0.0, 0.0)

    fake_ids = get_fake_bar_ids(bar_map)
    support_ids = set()
    if show_support:
        active_oid = bar_map[active_bar_id][0]
        support_ids = {
            b for b in get_supported_until(active_oid)
            if b in bar_map and b != active_bar_id
        }

    items = []
    for b, (oid, s) in bar_map.items():
        if b in fake_ids:
            continue  # staging bars: never shown, in any form
        if b == active_bar_id:
            cls = "current" if show_current else None
        elif b in support_ids:
            cls = "support"
        elif s > active_seq:
            cls = "unbuilt" if show_unbuilt else None
        else:
            cls = None  # built: the tube geometry carries it
        if cls is None:
            continue
        start = rs.CurveStartPoint(oid)
        end = rs.CurveEndPoint(oid)
        if start is None or end is None:
            continue
        items.append(
            (Rhino.Geometry.Line(start, end), colors[cls], thickness, dash, gap)
        )

    conduit = _ensure_conduit(store)
    conduit.items = items


# ---------------------------------------------------------------------------
# Robot rendering (the core's scene parts -- no PyBullet anywhere below)
# ---------------------------------------------------------------------------


def _build_timeline(bar_map):
    """The document as a process, and its global step list (``core.plan_steps.timeline``)."""
    result = process_source.RhinoProcessSource(
        "RSGHSequencePreview", process_source.geometry_cache(sc.sticky)
    ).build()
    steps, skipped = plan_steps.timeline(result.process)
    return {"process": result.process, "steps": steps, "skipped": skipped, "n_bars": len(bar_map)}


def _render_robots(store, process, step, notes):
    """Draw every robot of this step: the assembly robot with its tools, and every support robot present.

    The step's scene (``plan_steps.step_scene``) puts each robot where the
    process has it at that moment, and ``ik_viz`` draws them all with their
    tools. Robots absent from the moment (or whose base is not solved yet) are
    hidden. Bodies are hidden: the document's own geometry shows the bars.
    """
    robot_id = process_source.robot_id(config.ASSEMBLY_ROBOT_NAME)
    rs.EnableRedraw(False)
    try:
        if not store.get("session"):
            # hide_doc_layers=() is essential: the default hides tubes / joints
            # for the session, which would override the per-object visibility
            # this component drives through show_sequence_colors.
            ik_viz.begin_session(
                active_mesh_mode=ik_viz.MESH_MODE_VISUAL,
                layer_key=ik_viz.LAYER_KEY_ASSEMBLY,
                hide_doc_layers=(),
            )
            store["session"] = True
        ik_viz.update_state(plan_steps.step_scene(process, step), robot_id=robot_id,
                            layer_key=ik_viz.LAYER_KEY_ASSEMBLY, mesh_mode=ik_viz.MESH_MODE_VISUAL,
                            hide_bodies=True)
        ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, True)
    except Exception as exc:  # noqa: BLE001 -- a bad step must not kill the solve
        session_log.caught(exc)
        notes.append(f"robots not drawn: {exc}")
        ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
        return False
    finally:
        rs.EnableRedraw(True)
    return True


# ---------------------------------------------------------------------------
# The two layers we manage (recorded once per enable, restored on disable)
# ---------------------------------------------------------------------------


def _hide_layer_once(store, key, layer):
    """Hide *layer*, recording its prior visibility ONCE per enable cycle.

    Recorded on the first hide only: if the user manually shows the layer
    mid-preview, this component does not fight them -- the next re-assert
    happens only after a disable/enable cycle.
    """
    if store.get(key) is None and rs.IsLayer(layer):
        store[key] = bool(rs.LayerVisible(layer))
        rs.LayerVisible(layer, False)


def _restore_layer(store, key, layer):
    prev = store.pop(key, None)
    if prev is not None and rs.IsLayer(layer):
        rs.LayerVisible(layer, prev)


# ---------------------------------------------------------------------------
# Teardown
# ---------------------------------------------------------------------------


def _teardown(store):
    """Put the document back exactly as the component found it."""
    _drop_conduit(store)
    reset_sequence_colors()  # colours + visibility (and legacy width/dash resets)
    if store.pop("session", None):
        ik_viz.end_session()
    ik_viz.set_layer_visible(ik_viz.LAYER_KEY_ASSEMBLY, False)
    _restore_layer(store, "tool_layer_prev", config.LAYER_TOOL_INSTANCES)
    _restore_layer(store, "centerline_layer_prev", config.LAYER_BAR_CENTERLINES)
    # Migration: an older build of this component switched PrintDisplay on to
    # show print widths; make sure no session leaves it stuck on.
    if store.pop("print_display_on", None):
        rs.Command("_-PrintDisplay _State=_Off _Enter", False)
    for key in ("session", "fingerprint", "timeline", "n_frames"):
        store.pop(key, None)
    sc.doc.Views.Redraw()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(
    ghenv,
    enable=False,
    reload=False,
    step=0,
    show_unbuilt=True,
    show_current=True,
    show_support=True,
    line_thickness=2.0,
    line_style="continuous",
    dash_pattern="40,20",
):
    """Drive the Rhino viewport to one step of the global assembly timeline.

    Args:
        ghenv: the component's ``ghenv`` (or ``self`` in SDK mode).
        enable (bool): False tears the preview down and returns immediately.
        reload (bool): button; on its rising edge, (re)build the timeline and
            the ``step`` slider.
        step (int): frame index into the global timeline.
        show_unbuilt (bool): gray guide lines on not-yet-built bars.
        show_current (bool): blue guide line on the active bar.
        show_support (bool): purple guide lines on the supporting bars.
        line_thickness (float): guide-line width in screen PIXELS (drawn as an
            overlay; needs no PrintDisplay).
        line_style (str): ``"continuous"`` or ``"dashed"``.
        dash_pattern (str): dash/gap in document units (mm), e.g. ``"40,20"``.

    Returns:
        dict: ``{"bar_id", "seq", "pose", "frame_count", "step_label", "info"}``.
    """
    # Unconnected GH inputs arrive as None -- normalise once, here.
    enable = _flag(enable, False)
    reload = _flag(reload, False)
    step = _index(step, 0)
    show_unbuilt = _flag(show_unbuilt, True)
    show_current = _flag(show_current, True)
    show_support = _flag(show_support, True)
    thickness = max(0.0, _number(line_thickness, 2.0))
    style_raw = _unwrap(line_style)
    dashed = str(style_raw or "continuous").strip().lower().startswith("dash")
    pattern = _parse_dash_pattern(dash_pattern)

    store = gh_bridge.state(ghenv)
    notes = []

    with gh_bridge.rhino_doc():
        if not enable:
            _teardown(store)
            return {
                "bar_id": None, "seq": None, "pose": None,
                "frame_count": 0, "step_label": None,
                "info": "disabled; document restored",
            }

        # One rs.AllObjects() scan per solve, shared with the colour pass.
        # repair_on_entry is NOT called: it renumbers, and a viewer must not.
        bar_map = get_bar_seq_map()
        if not bar_map:
            return {
                "bar_id": None, "seq": None, "pose": None,
                "frame_count": 0, "step_label": None,
                "info": "no registered bars in this document",
            }

        # The process and its timeline are cached in sticky and rebuilt only on the
        # reload button or when the bar count changes underneath it -- a slider
        # move must not re-read the document.
        reload_edge = gh_bridge.rising_edge(store, "reload", reload)
        timeline = store.get("timeline")
        if reload_edge or timeline is None or timeline.get("n_bars") != len(bar_map):
            timeline = _build_timeline(bar_map)
            store["timeline"] = timeline
            store.pop("fingerprint", None)  # force a redraw after a rebuild
            if timeline["skipped"]:
                notes.append(f"{len(timeline['skipped'])} bar(s) contribute no assembly steps")
        steps = timeline["steps"]
        n_frames = len(steps)

        if not n_frames:
            return {
                "bar_id": None, "seq": None, "pose": None,
                "frame_count": 0, "step_label": None,
                "info": "no bar has a solved IK keyframe -- run RSIKKeyframe "
                        "(or RSIKKeyframeAll) first",
            }

        # The slider is (re)built on the button's rising edge, and its bounds
        # are refreshed silently whenever the frame count changes under it.
        if reload_edge or store.get("n_frames") != n_frames:
            notes.append(gh_bridge.ensure_int_slider(ghenv, "step", 0, n_frames - 1))
        store["n_frames"] = n_frames

        idx = max(0, min(step, n_frames - 1))
        current = steps[idx]
        bar_id = plan_steps.short(current.bar)
        label = current.label.split(" ", 1)[1]
        if bar_id not in bar_map:
            return {
                "bar_id": bar_id, "seq": None, "pose": label,
                "frame_count": n_frames, "step_label": None,
                "info": f"bar {bar_id} is no longer in the document -- press reload",
            }
        seq = bar_map[bar_id][1]
        step_label = f"step {idx + 1}/{n_frames} | bar {bar_id} | {label}"

        # GH re-solves aggressively (any upstream tick, a canvas move).  Skip
        # the whole render when nothing that affects it changed; `reload`
        # clears this, the manual escape hatch when the document changed.
        fingerprint = (
            bar_id, label, n_frames, show_unbuilt, show_current, show_support,
            thickness, dashed, pattern,
        )
        if store.get("fingerprint") == fingerprint:
            return {
                "bar_id": bar_id, "seq": seq, "pose": label,
                "frame_count": n_frames, "step_label": step_label,
                "info": "unchanged (render skipped)",
            }

        # Geometry visibility only -- no tints, no line attributes: the guide
        # lines are the overlay's job now.  Fake bars are never shown; tube +
        # joint geometry is restricted to built + current.  set_build_stage is
        # never called: this is a viewer.
        show_sequence_colors(
            bar_id,
            show_unbuilt=show_unbuilt,
            bar_map=bar_map,
            highlight_supports=False,
            color_flags={
                "built": False, "active": False,
                "unbuilt": False, "support": False,
            },
            show_fake=False,
            geom_built_and_active_only=True,
        )
        # The only two layers this component writes, both recorded once per
        # enable cycle so manual layer toggles are not fought: tools hidden
        # (never shown in the filming view), centerlines hidden (the overlay
        # replaces them -- in a custom display mode the raw curves draw in
        # the mode's curve colour and would double the overlay lines).
        _hide_layer_once(store, "tool_layer_prev", config.LAYER_TOOL_INSTANCES)
        _hide_layer_once(store, "centerline_layer_prev", config.LAYER_BAR_CENTERLINES)

        _update_guide_lines(
            store, bar_map, bar_id, seq,
            show_unbuilt, show_current, show_support,
            thickness, dashed, pattern,
        )

        _render_robots(store, timeline["process"], current, notes)

        store["fingerprint"] = fingerprint
        sc.doc.Views.Redraw()

    info = step_label
    if notes:
        info += " | " + " | ".join(notes)
    return {
        "bar_id": bar_id,
        "seq": seq,
        "pose": label,
        "frame_count": n_frames,
        "step_label": step_label,
        "info": info,
    }