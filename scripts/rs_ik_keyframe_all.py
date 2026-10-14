#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""RSIKKeyframeAll - Multi-bar dual-arm IK keyframe workflow (right-click).

Right-click companion to ``RSIKKeyframe`` (left-click = one picked bar). The user
selects SEVERAL bars at once; for each the command places a mobile-base frame and,
optionally, solves its IK chain -- reusing the single-bar command's shared helpers
end to end (base write, save/continue + collision prompts, the chain solver, the
keyframe writers) and ``core.rhino_walkable_ground.default_base_frame_for_bar`` for
the placement geometry.

Flow
----
1. Preview: every bar in the document that ALREADY has a solved IK keyframe is
   colored ``COLOR_HAS_IK`` up front, so the user can see what is already done.
2. Select the bars to process (curve or tube; pre-selection honored). If any picked
   bar ALREADY has solved IK, a warning asks whether to recalculate (overwrite) it:
   Yes re-solves those bars too, No keeps their existing IK and skips them.
3. Type the base standoff distance (mm), default 750.
4. For each bar being processed that IS IK-ready (exactly two tool-bearing L/R
   joints + an assigned WalkableGround), place a mobile base: stand ``standoff`` mm
   behind the GRABBED-JOINT CENTER (midpoint of the two held joints), on the ground,
   opposite the insertion direction, facing the bar. The base is written on the bar
   immediately.
5. Choose Continue, FlipAll / FlipOne (stand on the other side of the bar), or
   Save-base-and-exit (keep the placed bases, stop).
6. On Continue: solve each bar's approach/assembled/retreat chain at its placed
   base. ``BaseSearch`` at the collision-options prompt (one answer for the whole batch,
   the same choice as RSIKKeyframe's) decides what a bar that fails there gets next,
   exactly as in RSIKKeyframe: Single = nothing, Random = random bases around it,
   Optimised (the default) = the bases ``core.base_search`` proposes. Solved bars turn ``COLOR_HAS_IK`` (same as already-solved),
   failed bars turn ``COLOR_FAILED``; a popup tallies the outcome. Esc stops the
   batch before the next bar: solved bars stay solved, the rest are listed as not
   tried and the support pass (step 7) is skipped.
7. Support pass: every bar in the hold plan whose grasp + base picks are already
   stored gets its support keyframe re-solved NON-interactively at the stored
   base; bars without picks are listed for the single-bar interactive flow
   (grasps are hand-picked, never auto-generated).

Base guides + preview cleanup
-----------------------------
Each processed bar gets five guide lines on its walkable ground (see
``core.base_guide_geom``): the line joining its two grabbed-joint centers projected
onto the ground, three copies offset against the assembly direction by
``config.BASE_GUIDE_OFFSETS_MM``, and the extension line through their midpoints --
the line the placed base origin sits on. All of that, plus the base markers and
reach circles, is transient: a ``try/finally`` removes it on EVERY exit path.

Which end of a bar carries the LEFT tool follows from which side the base stands
on, so the tool sides are re-derived from the resolved heading after every
placement and every flip (hand-picked RSJointEdit sides are left alone).

Already-solved bars are skipped by default (they stay ``COLOR_HAS_IK``) unless the
user opts into recalculation at the warning above. Failed bars keep their saved base
and should be retried with the single-bar ``RSIKKeyframe`` button, which lets you
place a different base interactively.
"""

from __future__ import annotations

import importlib
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import session_log  # noqa: E402

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


# The single-bar command owns the shared solve path; import it as a module and
# reuse its helpers (mirrors rs_clear_ik_keyframe_all -> rs_clear_ik_keyframe).
import rs_ik_keyframe as ikf
from core import base_frame_viz as _base_frame_viz_module
from core import base_guide_geom as _base_guide_geom_module
from core import base_guide_viz as _base_guide_viz_module
from core import rhino_tool_place as _rhino_tool_place_module
# The numpy half of WalkableGround lives in the husky_assembly_tamp submodule
# (core.config puts it on sys.path); rhino_walkable_ground imports THIS same
# module object, so reloading it below refreshes the shared solver code.
from husky_assembly_tamp.keyframe import walkable_ground as _wg_np_module
from core import rhino_walkable_ground as _rwg_module
from core.rhino_bar_pick import bar_or_tube_filter, resolve_picked_to_bar_curve
from core.rhino_helpers import block_instance_xform_mm
# IK preview colors + the show helper live in rhino_bar_registry (single source of
# truth, shared with RSUpdatePreview / RSClearColorPreview) -- see that module's
# "IK color preview" section. This command paints the same COLOR_HAS_IK /
# COLOR_FAILED as the standalone show/clear commands.
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    COLOR_FAILED,
    COLOR_HAS_IK,
    find_stale_ik_bars,
    get_bar_seq_map,
    get_fake_bar_ids,
    paint_bar,
    show_all_ik_preview,
)
from core.ik_status import stale_ik_lines


# Bound at import so the helpers below work even before main() runs; main()
# rebinds them to freshly reloaded copies (the note.md 9 reload idiom).
base_guide_geom = _base_guide_geom_module
base_guide_viz = _base_guide_viz_module
rhino_tool_place = _rhino_tool_place_module


def _preview_existing_ik():
    """Color every bar that already has a solved IK keyframe (REQ1); stale ones in their own color.

    Reuses the shared :func:`show_all_ik_preview` so the entry preview matches
    RSUpdatePreview's ShowColorsPreview option exactly.

    Returns:
        dict: ``{bar_id: reason}`` of the bars whose saved IK no longer reaches its
        target (the core's plan check B15). Their keyframe does not count as solved
        here: a selected stale bar is solved again without asking. Empty when the
        document could not be checked (the reason is printed); every keyframe then
        counts as solved, as before.
    """
    stale = find_stale_ik_bars("RSIKKeyframeAll") or {}
    n_has_ik, _n_total = show_all_ik_preview(stale)
    if n_has_ik:
        print(f"RSIKKeyframeAll: {n_has_ik - len(stale)} bar(s) already have IK "
              "(shown in the solved color)"
              + (f", {len(stale)} more have STALE IK (yellow)." if stale else "."))
    for line in stale_ik_lines(stale):
        print(f"RSIKKeyframeAll: {line}")
    return stale


class _GetMultiple:
    """A `GetObject` whose `Get()` is `GetMultiple(1, 0)`, so `session_log.ask` logs a multi-pick like any prompt."""

    def __init__(self, getter):
        self._getter = getter

    def Get(self):
        return self._getter.GetMultiple(1, 0)

    def __getattr__(self, name):
        return getattr(self._getter, name)


def _select_bars(stale=()):
    """Multi-select bars (curve or tube), or take them all with an option.

    The prompt offers two options beside picking (window-select works too):

    - ``AllBars``: every fabricated bar of the document, in assembly order.
    - ``UnsolvedBars``: those without valid IK -- no keyframe yet, or a stale one.

    Fake bars are never offered by the options (the robot does not assemble them);
    bars the options include but that cannot be solved (no tool pair, no ground)
    are reported as skipped later, as for a manual pick.

    Args:
        stale: Bar ids whose saved IK is stale (``_preview_existing_ik``).

    Returns:
        dict | None: ``{bar_id: curve_oid}``; None when nothing was selected.
    """
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select bars for multi-bar IK (Enter when done)")
    go.SetCustomGeometryFilter(bar_or_tube_filter)
    go.EnablePreSelect(True, True)
    go.SubObjectSelect = False
    go.GroupSelect = True
    go.AddOption("AllBars")
    go.AddOption("UnsolvedBars")
    result = session_log.ask(_GetMultiple(go))
    if result == Rhino.Input.GetResult.Option:
        which = go.Option().EnglishName
        bar_map = get_bar_seq_map()
        fake_ids = get_fake_bar_ids(bar_map)
        bars = {}
        for bar_id, (curve, _seq) in sorted(bar_map.items(), key=lambda item: item[1][1]):
            if bar_id in fake_ids:
                continue
            if which == "UnsolvedBars" and bar_id not in stale and ikf.keyframe_store.has_ik_keyframe(curve):
                continue
            bars[bar_id] = curve
        print(f"RSIKKeyframeAll: {which}: {len(bars)} bar(s)"
              + (f": {', '.join(bars)}." if bars else "."))
        return bars or None
    if result != Rhino.Input.GetResult.Object:
        return None
    picked = [go.Object(i).ObjectId for i in range(go.ObjectCount)]
    rs.UnselectAllObjects()  # as rs.GetObjects(select=False): the pick is not left selected
    bars = {}
    for pid in picked:
        curve = resolve_picked_to_bar_curve(pid)
        if curve is None:
            continue  # stale tube whose bar was deleted
        bar_id = rs.GetUserText(curve, BAR_ID_KEY)
        if bar_id and bar_id not in bars:
            bars[bar_id] = curve
    return bars or None


def _assembly_joint_centers_mm(left, right):
    """The two held joint block origins (mm), as ``(a_mm, b_mm)``.

    ``left`` / ``right`` are the ``(male_joint_oid, tool_oid)`` tuples from
    ``ikf._resolve_arm_tools_on_bar``. The male/ground joint block origin is where
    the robot grabs, so these two points are what the base guide lines join and
    what their midpoint (the base's reference point) is taken from.
    """
    origin_l = block_instance_xform_mm(left[0])[:3, 3]
    origin_r = block_instance_xform_mm(right[0])[:3, 3]
    return (np.asarray(origin_l, dtype=float), np.asarray(origin_r, dtype=float))


def _grabbed_joint_center_mm(left, right):
    """Midpoint (mm) of the two held joint block origins -- the base's reference."""
    a_mm, b_mm = _assembly_joint_centers_mm(left, right)
    return 0.5 * (a_mm + b_mm)


@session_log.timed()
def _place_base(rwg, curve, bar_id, grounds, standoff_mm, flip=False,
                soup_cache=None):
    """Place + persist a base on one bar. Return ``(ctx, None)`` on success or
    ``(None, reason)`` when the bar is not IK-ready / has no ground.

    ``ctx`` = ``(bar_id, curve, base_frame, left_tool_oid, right_tool_oid, diag)``,
    where ``diag`` is the :func:`resolve_bar_heading` diagnosis plus the guide
    lines under ``"guides"`` -- the caller uses it to draw the guides, correct the
    bar's L/R tool sides, and report ambiguous headings.

    Args:
        rwg: the reloaded ``core.rhino_walkable_ground`` module.
        curve: the bar centerline curve id.
        bar_id (str): the bar id.
        grounds (dict): ``{ground_id: oid}``.
        standoff_mm (float): how far behind the joints the base stands.
        flip (bool): stand on the opposite side of the bar (the Flip option).
        soup_cache (dict | None): shared ground-tessellation memo (see
            ``rhino_walkable_ground._bar_ground_soups``).
    """
    result, err = ikf._resolve_arm_tools_on_bar(curve)
    if err is not None:
        return None, err
    _bar_id, left, right = result  # left/right = (male_joint_oid, tool_oid)

    joint_a_mm, joint_b_mm = _assembly_joint_centers_mm(left, right)
    center_mm = 0.5 * (joint_a_mm + joint_b_mm)
    diag = {}
    base_frame = rwg.default_base_frame_for_bar(
        curve, bar_id, grounds, standoff_mm=standoff_mm, center_mm=center_mm,
        flip=flip, diag_out=diag, soup_cache=soup_cache, verbose=True,
    )
    if base_frame is None:
        return None, "no base could be placed: no assigned/meshable WalkableGround, or see the command history."
    ikf._write_assembly_base_frame(curve, base_frame)

    # Guide lines for this bar, built on the SAME heading the base just used, so
    # the placed base sits on the yellow extension line by construction.
    try:
        soups = rwg._bar_ground_soups(curve, grounds, soup_cache=soup_cache)
        diag["guides"] = base_guide_geom.build_base_guides(
            soups, joint_a_mm, joint_b_mm, diag["heading"]
        )
    except Exception as exc:  # noqa: BLE001 -- guides are an aid, never a blocker
        session_log.caught(exc)
        print(f"RSIKKeyframeAll: {bar_id}: guide lines unavailable ({exc}).")
        diag["guides"] = None
    # base +Z is the ground normal at the placed base -- what the L/R tool-side
    # rule needs alongside the heading.
    diag["ground_normal"] = np.asarray(base_frame, dtype=float)[:3, 2]
    return (bar_id, curve, base_frame, left[1], right[1], diag), None


@session_log.timed()
def _solve_one(design, ctx, include_self, include_env, base_search="single", soups=None, moved=None):
    """Solve one bar's chain on Cindy's mirror, from the design's scenes, at its placed base. True on success.

    Args:
        base_search (str): what to try after the placed base fails, as RSIKKeyframe does (the one
            ``BaseSearch`` answer of the batch): ``"single"`` nothing, ``"sample"`` random bases,
            ``"optimise"`` the bases ``core.base_search`` proposes.
        soups (list | None): the bar's ground triangle soups, which the optimised search stands the bases on.
        moved (list | None): gets ``(bar_id, mm moved, degrees turned)`` when a base other than the placed one
            solved, so the summary can say the saved base is not the one that was reviewed.
    """
    bar_id, curve, base_frame = ctx[0], ctx[1], ctx[2]
    try:
        mirror, movements = ikf.design_planning.keyframe_movements(design, sc.sticky, f"bars/{bar_id}")
        with mirror.lend() as planner:
            solved, used_base = ikf._solve_chain_with_sampling(
                planner, movements, base_frame,
                brep_id=None, heading_mm=None,
                include_self=include_self, include_env=include_env,
                viz=None, urdf=design.robots[design.actions[f"{bar_id}_J"].robot].urdf,
                base_search=base_search, soups=soups, command="RSIKKeyframeAll",
            )
        if solved is not None and moved is not None:
            placed_at = np.asarray(base_frame, dtype=float)
            shift = float(np.linalg.norm(used_base[:3, 3] - placed_at[:3, 3]))
            turn = float(np.degrees(np.arccos(np.clip(np.dot(used_base[:3, 0], placed_at[:3, 0]), -1.0, 1.0))))
            if shift > 1.0 or turn > 0.5:
                moved.append((bar_id, shift, turn))
    except Exception as exc:  # noqa: BLE001 -- one bar must not stop the batch
        session_log.caught(exc)
        print(f"RSIKKeyframeAll: {bar_id}: could not solve ({type(exc).__name__}: {exc}).")
        return False
    if solved is None:
        return False
    ikf._write_assembly_base_frame(curve, used_base)
    ikf._write_assembly_keyframes(curve, solved["M1"], solved["M2"], solved["M3"], mirror.cell)
    return True


@session_log.timed()
def _support_resolve_pass(include_self, include_env):
    """Re-solve every hold whose grasp and base picks are stored, then check every release.

    Runs after the assembly loop. For every bar in the hold plan (in hold-start
    order): if its grasp and base picks are stored (so the design holds the hold),
    its support IK is re-solved NON-interactively at the stored base, on the
    support robot's mirror in the hold's scene; otherwise it is listed for the
    single-bar interactive flow. Grasps are never auto-picked.

    Returns:
        tuple: ``(resolved_ids, need_pick_ids, failed)`` where ``failed`` is
        ``[(bar_id, reason)]``.
    """
    from core.hold_schedule import derive_hold_plan
    from core.rhino_bar_registry import collect_hold_inputs, get_bar_seq_map

    config, store, support_ik = ikf.config, ikf.keyframe_store, ikf.support_ik
    bar_map = get_bar_seq_map()
    try:
        bar_seq, supported = collect_hold_inputs(bar_map)
        hold_plan = derive_hold_plan(bar_seq, supported, config.SUPPORT_ROBOT_NAMES)
    except RuntimeError as exc:
        print(f"RSIKKeyframeAll: hold plan derivation failed ({exc}); support pass skipped.")
        return [], [], [("<hold plan>", str(exc))]
    if not hold_plan:
        return [], [], []
    design = ikf._core_design("RSIKKeyframeAll")
    resolved, need_pick, failed = [], [], []
    check_collision = bool(include_self or include_env)
    for bar_id in sorted(hold_plan, key=lambda b: hold_plan[b]["hold_start_seq"]):
        oid = bar_map[bar_id][0]
        grasp_raw = rs.GetUserText(oid, config.KEY_SUPPORT_GRASP_FRAME)
        base_raw = rs.GetUserText(oid, config.KEY_SUPPORT_BASE_FRAME)
        if not grasp_raw or not base_raw or f"{bar_id}_H" not in design.actions:
            need_pick.append(bar_id)
            continue
        robot_name = hold_plan[bar_id]["robot_name"]
        print(f"RSIKKeyframeAll: re-solving {robot_name}'s hold on {bar_id} ...")
        try:
            mirror, template = ikf.design_planning.hold_solve_state(design, sc.sticky, f"bars/{bar_id}")
            grasp_mm = np.asarray(json.loads(grasp_raw), dtype=float)
            base_mm = np.asarray(json.loads(base_raw), dtype=float)
            tool0_mm = grasp_mm @ np.asarray(config.BAR_GRASP_TO_TOOL0["Robotiq"], dtype=float)
            with mirror.lend() as planner:
                held = support_ik.solve_support_ik(planner, template, base_mm, tool0_mm,
                                                   check_collision=check_collision)
                approach = None if held is None else support_ik.solve_support_ik(
                    planner, held, base_mm, store.approach_tool0_from_grasp(tool0_mm),
                    check_collision=check_collision)
            if held is None or approach is None:
                raise RuntimeError("support IK failed at the stored base")
            approach_cfg = support_ik.extract_group_config(approach, config.SUPPORT_GROUP, mirror.cell)
            held_cfg = support_ik.extract_group_config(held, config.SUPPORT_GROUP, mirror.cell)
            if check_collision:
                # * The hold must also clear its release scene (what was built while it held).
                found = ikf.design_planning.hold_release_collisions(
                    design, sc.sticky, f"bars/{bar_id}", ikf.design_source.pose_from_mm(base_mm),
                    dict(zip(held_cfg["joint_names"], held_cfg["joint_values"])),
                    dict(zip(approach_cfg["joint_names"], approach_cfg["joint_values"])))
                if found:
                    raise RuntimeError(f"collides in its release scene: {'; '.join(found)}")
            store.write_bar_support_keyframe(oid, robot_name, base_mm, grasp_mm, approach_cfg, held_cfg)
            resolved.append(bar_id)
        except Exception as exc:  # noqa: BLE001 -- one hold must not stop the pass
            session_log.caught(exc)
            failed.append((bar_id, f"{type(exc).__name__}: {exc}"))
            print(f"RSIKKeyframeAll: {bar_id}: support re-solve FAILED ({exc}).")
    # * Every release in its own moment, now that the holds are solved.
    ikf._core_release_report("RSIKKeyframeAll", None, [f"bars/{b}" for b in resolved])
    return resolved, need_pick, failed


def _escape_pressed():
    """True once the user pressed Esc since the last call: the way to stop the batch between two bars.

    Never raises: where the key cannot be read, the batch simply cannot be stopped.
    """
    try:
        Rhino.RhinoApp.Wait()  # let Rhino see the key press; a running script does not pump messages
        # ! Not `Rhino.RhinoApp.EscapeKeyPressed`: that is an event, and an event object is always true.
        return bool(sc.escape_test(False, True))  # no exception; True = forget the press once read
    except Exception as exc:  # noqa: BLE001
        session_log.caught(exc)
        return False


def _ambiguous_ids(placed):
    """Bar ids whose base side could not be derived with confidence.

    Either the anchor insertion axes cancelled (so the side came from the
    open-ground fallback) or the base could not reach its guide line because the
    walkable ground ran out. Both mean "look at this one before trusting it".
    """
    out = []
    for ctx in placed:
        diag = ctx[5] or {}
        if diag.get("ambiguous") or float(diag.get("off_guide_mm", 0.0)) > 1.0:
            out.append(ctx[0])
    return sorted(out)


def _apply_tool_sides(placed, verbose=True):
    """Re-place each placed bar's tools so L/R matches where its base stands.

    Which end of a bar takes the LEFT tool follows from the approach, not from
    the bar -- see ``core.rhino_tool_place.assign_tool_sides_from_heading``, which
    also leaves hand-picked (RSJointEdit) sides alone. Run after every placement
    and every flip so the arms never end up crossed.

    ! Re-placing a tool DELETES the old instance and inserts a new one, so any
    tool oid captured before this call is dead. Every corrected bar therefore has
    its ctx refreshed in place -- without that, ``_solve_one`` later hands a stale
    guid to ``rs.coercerhinoobject`` and dies with "does not exist in ObjectTable".
    """
    n_fixed = 0
    for i, ctx in enumerate(placed):
        bar_id, curve, diag = ctx[0], ctx[1], (ctx[5] or {})
        if diag.get("heading") is None:
            continue
        try:
            changed = rhino_tool_place.assign_tool_sides_from_heading(
                bar_id, diag["heading"], diag["ground_normal"], verbose=verbose,
            )
        except Exception as exc:  # noqa: BLE001 -- never block the batch on this
            session_log.caught(exc)
            print(f"RSIKKeyframeAll: {bar_id}: tool-side check skipped ({exc}).")
            continue
        if not changed:
            continue
        n_fixed += changed
        result, err = ikf._resolve_arm_tools_on_bar(curve)
        if err is not None:
            print(f"RSIKKeyframeAll: {bar_id}: could not re-read tools after the "
                  f"side correction ({err}).")
            continue
        _bar_id, left, right = result
        placed[i] = (ctx[0], ctx[1], ctx[2], left[1], right[1], ctx[5])
    return n_fixed


def _redraw_base_preview(base_frame_viz, placed):
    """Re-bake the guide lines + base markers for every placed bar."""
    base_guide_viz.draw_base_guides({ctx[0]: (ctx[5] or {}).get("guides")
                                     for ctx in placed})
    base_frame_viz.draw_base_frames(
        {ctx[0]: ctx[2] for ctx in placed},
        circle_radius_mm=float(ikf.config.IK_BASE_SAMPLE_RADIUS),
    )
    sc.doc.Views.Redraw()


def _clear_base_preview(base_frame_viz):
    """Delete every transient base preview object (guides, markers, circles).

    Idempotent and never raises: it runs from a ``finally``, where an exception
    would mask whatever actually went wrong.
    """
    try:
        base_guide_viz.clear_base_guides()
        base_frame_viz.clear_base_frames()
        sc.doc.Views.Redraw()
    except Exception as exc:  # noqa: BLE001
        session_log.caught(exc)
        print(f"RSIKKeyframeAll: failed to clear the base preview ({exc}); continuing.")


def _ask_base_decision(n_ambiguous):
    """Off-ramp prompt with the Flip options; mirrors ``_ask_save_base_or_continue``.

    Returns ``"continue"`` / ``"save_and_exit"`` / ``"cancel"`` (the same strings
    the single-bar command uses) or ``"flip_all"`` / ``"flip_one"``, which the
    caller handles before re-prompting.
    """
    note = (f" [{n_ambiguous} with an ambiguous side -- check them]"
            if n_ambiguous else "")
    choice = rs.GetString(
        f"Bases placed{note}. Continue to solve, flip a side, or save and exit?",
        "Continue",
        ["Continue", "FlipAll", "FlipOne", "SaveAndExit"],
    )
    if choice is None:
        return "cancel"
    return {
        "Continue": "continue",
        "FlipAll": "flip_all",
        "FlipOne": "flip_one",
        "SaveAndExit": "save_and_exit",
    }.get(choice, "cancel")


def _flip_bars(rwg, placed, flipped, bar_ids, standoff, grounds, soup_cache):
    """Re-place the given bars' bases on the opposite side, in place.

    An explicit flip is the user re-deciding the side, so any hand-picked
    RSJointEdit tool side on those bars is cleared first -- otherwise the tools
    would stay on the ends that suited the OLD approach.
    """
    for i, ctx in enumerate(placed):
        bar_id, curve = ctx[0], ctx[1]
        if bar_id not in bar_ids:
            continue
        rhino_tool_place.clear_tool_side_overrides(bar_id, verbose=True)
        flipped[bar_id] = not flipped.get(bar_id, False)
        new_ctx, reason = _place_base(
            rwg, curve, bar_id, grounds, standoff,
            flip=flipped[bar_id], soup_cache=soup_cache,
        )
        if new_ctx is None:
            print(f"RSIKKeyframeAll: {bar_id}: flip failed ({reason}); "
                  "keeping the previous base.")
            flipped[bar_id] = not flipped[bar_id]
            continue
        placed[i] = new_ctx
        print(f"RSIKKeyframeAll: {bar_id}: base flipped to the other side of the bar.")


def _summary(bars, placed, solved_ids, failed_ids, already_ids, skipped, solved_run, stale_ids=()):
    """Build the multiline popup summary string."""
    lines = [f"Multi-bar IK on {len(bars)} selected bar(s):", ""]
    stale_ids = sorted(set(stale_ids or ()) & set(bars))
    if stale_ids:
        lines.append(f"Had STALE IK (treated as unsolved): {len(stale_ids)}  {', '.join(stale_ids)}")
    if solved_run:
        lines.append(f"Newly solved: {len(solved_ids)}"
                     + (f"  {', '.join(solved_ids)}" if solved_ids else ""))
        lines.append(f"Failed: {len(failed_ids)}"
                     + (f"  {', '.join(failed_ids)}" if failed_ids else ""))
        if failed_ids:
            lines.append("  (re-run failed bars with the single-bar RSIKKeyframe "
                         "to place a new base.)")
    else:
        lines.append(f"Bases placed & saved (not solved): {len(placed)}"
                     + (f"  {', '.join(c[0] for c in placed)}" if placed else ""))
    if already_ids:
        lines.append(f"Already had IK (left untouched): {len(already_ids)}"
                     f"  {', '.join(already_ids)}")
    ambiguous = _ambiguous_ids(placed)
    if ambiguous:
        lines.append("")
        lines.append(f"Ambiguous heading -- VERIFY THE SIDE: {len(ambiguous)}"
                     f"  {', '.join(ambiguous)}")
        lines.append("  (their anchor joints did not agree on a side, or the ground "
                     "ran out before the standoff; see the command line for details.)")
    if skipped:
        lines.append("")
        lines.append("Skipped:")
        lines.extend(f"  {bid}: {reason}" for bid, reason in skipped)
    return "\n".join(lines)


def main():
    global base_guide_geom, base_guide_viz, rhino_tool_place

    ikf._reload_runtime_modules()
    # Pick up the extended default_base_frame_for_bar(center_mm=, standoff_mm=) /
    # derive_seed_base signatures without a Rhino restart (numpy half first).
    importlib.reload(_wg_np_module)
    rwg = importlib.reload(_rwg_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
    base_guide_geom = importlib.reload(_base_guide_geom_module)
    base_guide_viz = importlib.reload(_base_guide_viz_module)
    rhino_tool_place = importlib.reload(_rhino_tool_place_module)

    ikf.warn_slow_ik_backend("RSIKKeyframeAll")
    ikf.repair_on_entry(float(ikf.config.BAR_RADIUS), "RSIKKeyframeAll")
    rwg.ensure_ground_and_base("RSIKKeyframeAll")

    # REQ1: preview which bars already have IK before the user selects anything.
    # A stale keyframe (the bar moved or was copied after its solve) is not a
    # solution of this bar any more: such bars are treated as unsolved below.
    stale = _preview_existing_ik()

    def has_valid_ik(bar_id, curve):
        return bar_id not in stale and ikf.keyframe_store.has_ik_keyframe(curve)

    bars = _select_bars(stale)
    if not bars:
        print("RSIKKeyframeAll: no bars selected.")
        return

    # Selected bars that already carry solved IK. Recalculating them IS allowed, but
    # warn first (before the standoff prompt) so an accidental pick of a solved bar
    # is caught up front and good IK is not overwritten by accident.
    already_selected = sorted(bid for bid, curve in bars.items() if has_valid_ik(bid, curve))
    stale_selected = sorted(bid for bid in bars if bid in stale)
    if stale_selected:
        print(f"RSIKKeyframeAll: {len(stale_selected)} selected bar(s) have STALE IK and are solved again: "
              f"{', '.join(stale_selected)}.")
    recalc_solved = False
    if already_selected:
        resp = rs.MessageBox(
            f"{len(already_selected)} selected bar(s) already have a SOLVED IK "
            f"keyframe:\n  {', '.join(already_selected)}\n\n"
            "Recalculate (OVERWRITE) their IK too?\n"
            "  Yes = re-place a base and re-solve them (overwrites existing IK)\n"
            "  No  = keep their existing IK and skip them",
            4 | 48,  # Yes/No buttons + warning icon
            "RSIKKeyframeAll - overwrite solved IK?",
        )
        if resp not in (6, 7):  # dialog closed / cancelled
            print("RSIKKeyframeAll: cancelled at the overwrite-IK warning.")
            return
        recalc_solved = (resp == 6)  # 6 = Yes

    # REQ2: standoff distance is user-typeable. The default is the MIDDLE base-guide
    # offset (500 mm), so the auto-placed base lands on the middle guide line and
    # 375 / 625 read as the two alternatives drawn either side of it.
    standoff = rs.GetReal(
        "Robot base standoff from bar (mm)",
        float(ikf.config.IK_BASE_STANDOFF_MULTIBAR_MM),
        0.0,
    )
    if standoff is None:
        print("RSIKKeyframeAll: cancelled at the standoff prompt.")
        return

    # Partition: solved bars we are NOT recalculating stay COLOR_HAS_IK and are
    # skipped; everything else (unsolved, or solved-and-opted-in) is processed.
    already_ids = []
    to_process = []  # (bar_id, curve)
    for bar_id, curve in bars.items():
        if has_valid_ik(bar_id, curve) and not recalc_solved:
            already_ids.append(bar_id)
            paint_bar(curve, COLOR_HAS_IK)
        else:
            to_process.append((bar_id, curve))
    already_ids.sort()

    # Place a base on every processable bar (fast); collect prerequisite failures.
    # One shared soup cache: tessellating a WalkableGround brep is the expensive
    # step and every bar would otherwise redo it (see note.md 14).
    grounds = rwg.get_all_walkable_grounds()
    soup_cache = {}
    placed = []   # (bar_id, curve, base_frame, left_tool, right_tool, diag)
    skipped = []  # (bar_id, reason)
    flipped = {}  # bar_id -> is this base currently flipped?

    # ! Everything below bakes transient preview geometry (guide lines, base
    # markers, reach circles). It MUST be gone on every exit path -- solved,
    # ESC at any prompt, SaveAndExit, or an exception -- so the whole block is
    # wrapped in try/finally. Before this guard existed, an ESC left the circles
    # and triads in the document with nothing able to clear them (neither
    # RSUpdatePreview nor RSClearColorPreview touches those layers).
    try:
        for bar_id, curve in to_process:
            ctx, reason = _place_base(rwg, curve, bar_id, grounds, standoff,
                                      soup_cache=soup_cache)
            if ctx is None:
                skipped.append((bar_id, reason))
                continue
            placed.append(ctx)

        if not placed:
            print("RSIKKeyframeAll: no processable bars (nothing to place).")
            rs.MessageBox(
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False, stale_ids=stale),
                0, "RSIKKeyframeAll",
            )
            return

        # Which end of each bar takes the LEFT tool follows from where its base
        # stands, so correct the tool sides now -- before the movements are built
        # from the tool frames. Hand-picked (RSJointEdit) sides are left alone.
        n_sides = _apply_tool_sides(placed)
        if n_sides:
            print(f"RSIKKeyframeAll: {n_sides} tool(s) re-placed to match the "
                  "robot's approach side.")

        # Show the guide lines + a static base marker and reach circle at every
        # placed base, so the user can see where each robot will stand (and on
        # which side) while deciding save-vs-continue and during the solve.
        _redraw_base_preview(base_frame_viz, placed)

        print(f"RSIKKeyframeAll: placed base on {len(placed)} bar(s) "
              f"(standoff {standoff:.0f} mm).")

        # Off-ramp: solve now, flip a base onto the other side of its bar, or keep
        # the placed bases and stop. Loops until the user continues or leaves.
        while True:
            decision = _ask_base_decision(len(_ambiguous_ids(placed)))
            if decision == "flip_all":
                _flip_bars(rwg, placed, flipped, {c[0] for c in placed},
                           standoff, grounds, soup_cache)
            elif decision == "flip_one":
                marker = rs.GetObject("Pick a base marker to flip", preselect=False,
                                      select=False)
                bar_id = base_frame_viz.bar_id_of_picked(marker) if marker else ""
                if not bar_id:
                    print("RSIKKeyframeAll: no base marker picked; nothing flipped.")
                    continue
                _flip_bars(rwg, placed, flipped, {bar_id},
                           standoff, grounds, soup_cache)
            else:
                break
            # A flip re-decides the approach, so the tool sides follow it.
            _apply_tool_sides(placed)
            _redraw_base_preview(base_frame_viz, placed)

        if decision != "continue":
            if decision == "cancel":
                print("RSIKKeyframeAll: cancelled (placed bases are still saved).")
            else:
                print("RSIKKeyframeAll: bases saved on the selected bars; "
                      "skipped IK solve.")
            rs.MessageBox(
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False, stale_ids=stale),
                0, "RSIKKeyframeAll",
            )
            return

        # One collision-option prompt for the whole batch, showing the design's
        # body count. The design is built once, after the bases and tool sides
        # are settled (they are in it).
        design = ikf._core_design("RSIKKeyframeAll")
        opts = ikf._ask_collision_options(env_count=len(design.bodies), offer_base_search=True)
        if opts is None:
            print("RSIKKeyframeAll: cancelled at collision options (bases still saved).")
            rs.MessageBox(
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False, stale_ids=stale),
                0, "RSIKKeyframeAll",
            )
            return
        include_self, include_env, _mesh_mode, base_search = opts

        # Solve each processable bar at its placed base (and, per BaseSearch, at other
        # bases after it). Esc stops the batch before the next bar: what is
        # solved stays solved, the rest is not tried.
        solved_ids, failed_ids, not_tried, moved = [], [], [], []
        total = len(placed)
        print(f"RSIKKeyframeAll: solving {total} bar(s); press Esc to stop after the bar in progress.")
        _escape_pressed()  # forget an Esc from the prompts before
        for i, ctx in enumerate(placed, start=1):
            bar_id, curve = ctx[0], ctx[1]
            if _escape_pressed():
                not_tried = [c[0] for c in placed[i - 1:]]
                print(f"RSIKKeyframeAll: stopped by Esc; {len(not_tried)} bar(s) not tried: {', '.join(not_tried)}.")
                break
            print(f"RSIKKeyframeAll: solving {bar_id} ({i}/{total}) ...")
            bar_soups = (rwg._bar_ground_soups(curve, grounds, soup_cache=soup_cache)
                         if base_search == "optimise" else None)
            ok = _solve_one(design, ctx, include_self, include_env,
                            base_search=base_search, soups=bar_soups, moved=moved)
            if ok:
                solved_ids.append(bar_id)
                paint_bar(curve, COLOR_HAS_IK)
                print(f"RSIKKeyframeAll: {bar_id} ({i}/{total}) -> solved.")
            else:
                failed_ids.append(bar_id)
                paint_bar(curve, COLOR_FAILED)
                print(f"RSIKKeyframeAll: {bar_id} ({i}/{total}) -> FAILED.")

        # Support pass: re-solve every hold whose grasp/base picks are stored;
        # list the rest for the single-bar interactive flow. Not after an Esc: it
        # is the other long step, and the user asked to stop.
        support_resolved, support_need_pick, support_failed = (
            ([], [], []) if not_tried else _support_resolve_pass(include_self, include_env))

        # Calculation finished: drop the transient preview BEFORE the modal summary
        # so the per-bar solved/failed colors are the only thing left on screen.
        # (The finally below repeats this for every other exit path; both are
        # idempotent.) Only the preview goes -- the saved base frames stay.
        _clear_base_preview(base_frame_viz)

        msg = _summary(bars, placed, solved_ids, failed_ids, already_ids, skipped,
                       solved_run=True, stale_ids=stale)
        support_lines = []
        if support_resolved:
            support_lines.append(f"Support holds re-solved: {len(support_resolved)}"
                                 f"  {', '.join(support_resolved)}")
        if support_need_pick:
            support_lines.append(
                f"Bars still needing an interactive support pick: "
                f"{', '.join(support_need_pick)}\n"
                "  (run RSIKKeyframe on each: assembly first if unsolved, then "
                "re-click for the support flow.)"
            )
        if support_failed:
            support_lines.append("Support re-solve failed:\n" + "\n".join(
                f"  {b}: {e}" for b, e in support_failed))
        if support_lines:
            msg += "\n\n" + "\n".join(support_lines)
        if not_tried:
            msg += (f"\n\nSTOPPED by Esc. Not tried: {len(not_tried)}  {', '.join(not_tried)}\n"
                    "  (their bases are saved; the support holds were not re-solved either. "
                    "Run the command again for them.)")
        if moved:
            msg += ("\n\nSolved at another base, NOT the placed one (the new base is saved on the bar):\n"
                    + "\n".join(f"  {b}: moved {mm:.0f} mm, turned {deg:.0f} deg" for b, mm, deg in moved))
        rs.MessageBox(msg, 0, "RSIKKeyframeAll")
    finally:
        _clear_base_preview(base_frame_viz)
        # The base search's DEBUG trace (BaseSearch=Optimised) of the last bar searched.
        ikf.base_search_viz.hide()


if __name__ == "__main__":
    session_log.run(__file__, main)
