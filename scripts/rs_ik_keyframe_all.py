#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# compas_fab is loaded from the in-repo submodule `external/compas_fab` via sys.path injection in `core.robot_cell`. Do not list it under `# r:` (pip cache would ignore SHA changes).
# r: compas_robots==0.6.0
# r: pybullet==3.2.7
# r: pybullet_planning==0.6.1
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
6. On Continue: solve each bar's approach/assembled/retreat chain ONCE at its placed
   base (no base sampling). Solved bars turn ``COLOR_HAS_IK`` (same as already-solved),
   failed bars turn ``COLOR_FAILED``; a popup tallies the outcome.

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
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

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
# IK preview colors + the show helper live in rhino_bar_registry (single source of
# truth, shared with RSUpdatePreview / RSClearColorPreview) -- see that module's
# "IK color preview" section. This command paints the same COLOR_HAS_IK /
# COLOR_FAILED as the standalone show/clear commands.
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    COLOR_FAILED,
    COLOR_HAS_IK,
    paint_bar,
    show_all_ik_preview,
)


# Bound at import so the helpers below work even before main() runs; main()
# rebinds them to freshly reloaded copies (the Su_note.md 9 reload idiom).
base_guide_geom = _base_guide_geom_module
base_guide_viz = _base_guide_viz_module
rhino_tool_place = _rhino_tool_place_module


def _preview_existing_ik():
    """Color every bar that already has a solved IK keyframe (REQ1).

    Reuses the shared :func:`show_all_ik_preview` so the entry preview matches
    RSUpdatePreview's ShowColorsPreview option exactly.
    """
    n_has_ik, _n_total = show_all_ik_preview()
    if n_has_ik:
        print(f"RSIKKeyframeAll: {n_has_ik} bar(s) already have IK "
              "(shown in the solved color).")


def _select_bars():
    """Multi-select bars (curve or tube); return ``{bar_id: curve_oid}`` or None."""
    picked = rs.GetObjects(
        "Select bars for multi-bar IK (Enter when done)",
        custom_filter=bar_or_tube_filter,
        preselect=True,
        select=False,
        minimum_count=1,
    )
    if not picked:
        return None
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
    origin_l = ikf._block_instance_xform_mm(left[0])[:3, 3]
    origin_r = ikf._block_instance_xform_mm(right[0])[:3, 3]
    return (np.asarray(origin_l, dtype=float), np.asarray(origin_r, dtype=float))


def _grabbed_joint_center_mm(left, right):
    """Midpoint (mm) of the two held joint block origins -- the base's reference."""
    a_mm, b_mm = _assembly_joint_centers_mm(left, right)
    return 0.5 * (a_mm + b_mm)


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
        return None, "no assigned/meshable WalkableGround to stand a base on."
    ikf._write_assembly_base_frame(curve, base_frame)

    # Guide lines for this bar, built on the SAME heading the base just used, so
    # the placed base sits on the yellow extension line by construction.
    try:
        soups = rwg._bar_ground_soups(curve, grounds, soup_cache=soup_cache)
        diag["guides"] = base_guide_geom.build_base_guides(
            soups, joint_a_mm, joint_b_mm, diag["heading"]
        )
    except Exception as exc:  # noqa: BLE001 -- guides are an aid, never a blocker
        print(f"RSIKKeyframeAll: {bar_id}: guide lines unavailable ({exc}).")
        diag["guides"] = None
    # base +Z is the ground normal at the placed base -- what the L/R tool-side
    # rule needs alongside the heading.
    diag["ground_normal"] = np.asarray(base_frame, dtype=float)[:3, 2]
    return (bar_id, curve, base_frame, left[1], right[1], diag), None


def _solve_one(planner, rcell, ctx, include_self, include_env):
    """Solve one bar's IK chain once at its placed base. Return True on success."""
    bar_id, curve, base_frame, left_tool_oid, right_tool_oid, _diag = ctx
    tool0_left = ikf._block_instance_xform_mm(left_tool_oid)
    tool0_right = ikf._block_instance_xform_mm(right_tool_oid)
    try:
        movements, _env_geom = ikf.bar_action.build_assembly_movements(
            rcell, planner, bar_id, base_frame, tool0_left, tool0_right,
        )
    except (RuntimeError, ValueError) as exc:
        print(f"RSIKKeyframeAll: {bar_id}: could not build movements ({exc}).")
        return False

    # brep_id=None => sampling disabled: exactly one attempt at the placed base.
    solved, used_base = ikf._solve_chain_with_sampling(
        planner, movements, base_frame,
        brep_id=None, heading_mm=None,
        include_self=include_self, include_env=include_env,
        viz=None,
    )
    if solved is None:
        return False
    ikf._write_assembly_base_frame(curve, used_base)
    ikf._write_assembly_keyframes(
        curve, solved["M1"], solved["M2"], solved["M3"], rcell
    )
    ikf._write_legacy_assembly_blob(
        curve, used_base, solved["M2"], solved["M1"], rcell
    )
    return True


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


def _summary(bars, placed, solved_ids, failed_ids, already_ids, skipped, solved_run):
    """Build the multiline popup summary string."""
    lines = [f"Multi-bar IK on {len(bars)} selected bar(s):", ""]
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

    ikf.repair_on_entry(float(ikf.config.BAR_RADIUS), "RSIKKeyframeAll")

    if not ikf.robot_cell.is_pb_running():
        rs.MessageBox("PyBullet is not running. Click RSPBStart first.", 0,
                      "RSIKKeyframeAll")
        return
    _client, planner = ikf.robot_cell.get_planner()
    rcell = ikf.robot_cell.get_or_load_robot_cell()
    if not ikf.robot_cell.prompt_if_cell_stale(rcell, planner):
        print("RSIKKeyframeAll: aborted (stale collision cell).")
        return

    # REQ1: preview which bars already have IK before the user selects anything.
    _preview_existing_ik()

    bars = _select_bars()
    if not bars:
        print("RSIKKeyframeAll: no bars selected.")
        return

    # Selected bars that already carry solved IK. Recalculating them IS allowed, but
    # warn first (before the standoff prompt) so an accidental pick of a solved bar
    # is caught up front and good IK is not overwritten by accident.
    already_selected = sorted(
        bid for bid, curve in bars.items() if ikf.bar_action.has_ik_keyframe(curve)
    )
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
        if ikf.bar_action.has_ik_keyframe(curve) and not recalc_solved:
            already_ids.append(bar_id)
            paint_bar(curve, COLOR_HAS_IK)
        else:
            to_process.append((bar_id, curve))
    already_ids.sort()

    # Place a base on every processable bar (fast); collect prerequisite failures.
    # One shared soup cache: tessellating a WalkableGround brep is the expensive
    # step and every bar would otherwise redo it (see Su_note.md 14).
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
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False),
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
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False),
                0, "RSIKKeyframeAll",
            )
            return

        # One collision-option prompt for the whole batch. Pass the collision-body
        # count so the prompt shows "env=N bodies" instead of env=0: single-bar
        # RSIKKeyframe displays len(env_geom), which is exactly
        # len(ensure_assembly_cell(...)) -- the cached {name: body_info} dict
        # (bar-independent; warm-cache read, no rebuild).
        opts = ikf._ask_collision_options(
            env_count=len(ikf.robot_cell.ensure_assembly_cell(rcell, planner))
        )
        if opts is None:
            print("RSIKKeyframeAll: cancelled at collision options (bases still saved).")
            rs.MessageBox(
                _summary(bars, placed, [], [], already_ids, skipped, solved_run=False),
                0, "RSIKKeyframeAll",
            )
            return
        include_self, include_env, _mesh_mode = opts

        # Solve each processable bar once at its placed base.
        solved_ids, failed_ids = [], []
        total = len(placed)
        for i, ctx in enumerate(placed, start=1):
            bar_id, curve = ctx[0], ctx[1]
            print(f"RSIKKeyframeAll: solving {bar_id} ({i}/{total}) ...")
            if _solve_one(planner, rcell, ctx, include_self, include_env):
                solved_ids.append(bar_id)
                paint_bar(curve, COLOR_HAS_IK)
                print(f"RSIKKeyframeAll: {bar_id} ({i}/{total}) -> solved.")
            else:
                failed_ids.append(bar_id)
                paint_bar(curve, COLOR_FAILED)
                print(f"RSIKKeyframeAll: {bar_id} ({i}/{total}) -> FAILED.")

        # Calculation finished: drop the transient preview BEFORE the modal summary
        # so the per-bar solved/failed colors are the only thing left on screen.
        # (The finally below repeats this for every other exit path; both are
        # idempotent.) Only the preview goes -- the saved base frames stay.
        _clear_base_preview(base_frame_viz)

        rs.MessageBox(
            _summary(bars, placed, solved_ids, failed_ids, already_ids, skipped,
                     solved_run=True),
            0, "RSIKKeyframeAll",
        )
    finally:
        _clear_base_preview(base_frame_viz)


if __name__ == "__main__":
    main()
