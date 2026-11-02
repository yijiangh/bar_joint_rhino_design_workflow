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
3. Type the base standoff distance (mm), default ``config.IK_BASE_STANDOFF_MM``
   (625 mm, the outer base-guide line).
4. For each bar being processed that IS IK-ready (exactly two tool-bearing L/R
   joints + an assigned WalkableGround), place a mobile base: stand ``standoff`` mm
   behind the GRABBED-JOINT CENTER (midpoint of the two held joints), on the ground,
   opposite the insertion direction, facing the bar. The base is written on the bar
   immediately.
   Which side of the bar it stands on is decided from the bar's anchor insertion
   axes; when those say nothing (both anchors insert vertically, or they cancel),
   the side comes from what the document already says: the tool sides the user
   hand-picked in RSJointEdit, else the base saved on the bar (a Flip of an
   earlier run), else the more open side of the bar.
5. Walk through the bars whose side nothing decided, one by one: each is selected
   and zoomed to, its base and the other side are marked on screen, and the
   command line asks Keep / Flip / KeepRest. Then choose Continue, FlipAll /
   FlipOne (stand on the other side of the bar), or Save-base-and-exit (keep the
   placed bases, stop).
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

No preview geometry
-------------------
The batch bakes no guide lines, base markers or reach circles: with thirty bars
they are clutter, and baking them was the slow part of the command (about a
second per bar). RSIKKeyframe is where one bar's guides are looked at. The two
preview layers are still cleared on every exit path, so leftovers of an aborted
single-bar run never survive a batch.

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
from core import base_guide_viz as _base_guide_viz_module
from core import rhino_tool_place as _rhino_tool_place_module
# The numpy half of WalkableGround lives in the husky_assembly_tamp submodule
# (core.config puts it on sys.path); rhino_walkable_ground imports THIS same
# module object, so reloading it below refreshes the shared solver code.
from husky_assembly_tamp.keyframe import walkable_ground as _wg_np_module
from core import rhino_walkable_ground as _rwg_module
from core.rhino_bar_pick import (
    bar_or_tube_filter,
    pick_bar_among,
    resolve_picked_to_bar_curve,
)
from core.rhino_helpers import block_instance_xform_mm, doc_unit_scale_to_mm
# IK preview colors + the show helper live in rhino_bar_registry (single source of
# truth, shared with RSUpdatePreview / RSClearColorPreview) -- see that module's
# "IK color preview" section. This command paints the same COLOR_HAS_IK /
# COLOR_FAILED as the standalone show/clear commands.
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    COLOR_FAILED,
    COLOR_HAS_IK,
    _find_existing_tube,
    find_stale_ik_bars,
    get_bar_seq_map,
    get_fake_bar_ids,
    paint_bar,
    show_all_ik_preview,
)
from core.ik_status import stale_ik_lines


# Bound at import so the helpers below work even before main() runs; main()
# rebinds them to freshly reloaded copies (the note.md 9 reload idiom).
base_frame_viz = _base_frame_viz_module
base_guide_viz = _base_guide_viz_module
rhino_tool_place = _rhino_tool_place_module

#: How far (mm) the heading arrow of a reviewed base reaches, and the colours of the review marks.
_REVIEW_ARROW_MM = 400.0
_REVIEW_BASE_COLOR = (220, 40, 40)      # the placed base, red as the base-frame X axis
_REVIEW_OTHER_COLOR = (150, 150, 150)   # where the base would stand after a Flip


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


def _side_from_document(bar_id, heading, ground_normal, saved_base):
    """What in the document says the base belongs on the OTHER side from ``heading``, if anything.

    Looked at in this order, the more deliberate signal first:

    1. tool sides the user hand-picked in RSJointEdit
       (``rhino_tool_place.hand_picked_side_verdict``): the left arm reaches the
       end on the robot's left, so hand-swapped tools mean the robot stands on
       the other side;
    2. the base saved on the bar facing the other way: a Flip of an earlier run,
       or a base picked there in RSIKKeyframe. This is the rule RSIKKeyframe
       itself applies on entry ("the base saved on this bar stands on the other
       side; guides and tool sides follow it"), so a flip now sticks in the batch
       instead of being undone on every run.

    Args:
        bar_id (str): the bar.
        heading (np.ndarray): the automatic base heading (base +X).
        ground_normal (np.ndarray): the ground normal at the base.
        saved_base (np.ndarray | None): the base frame saved on the bar before
            this run touched it.

    Returns:
        str | None: a short name of the signal that asks for the other side, or
        None when the automatic side stands.
    """
    verdict = rhino_tool_place.hand_picked_side_verdict(bar_id, heading, ground_normal)
    if verdict == "flipped":
        return "the hand-picked tool sides"
    if verdict is None and saved_base is not None:
        saved_heading = np.asarray(saved_base, dtype=float)[:3, 0]
        if float(np.dot(saved_heading, np.asarray(heading, dtype=float))) < 0.0:
            return "the base saved on the bar"
    return None


@session_log.timed()
def _place_base(rwg, curve, bar_id, grounds, standoff_mm, flip=None,
                soup_cache=None):
    """Place + persist a base on one bar. Return ``(ctx, None)`` on success or
    ``(None, reason)`` when the bar is not IK-ready / has no ground.

    ``ctx`` = ``(bar_id, curve, base_frame, left_tool_oid, right_tool_oid, diag)``,
    where ``diag`` is the :func:`resolve_bar_heading` diagnosis plus
    ``"ground_normal"`` (base +Z), ``"flipped"`` (is the base on the other side
    from the automatic one) and ``"side_source"`` (what decided the side when it
    was not the automatic rule: the document's hand-picked tools or saved base,
    or the user's answer) -- the caller uses it to correct the bar's L/R tool
    sides and to find the bars whose side is still undecided.

    Args:
        rwg: the reloaded ``core.rhino_walkable_ground`` module.
        curve: the bar centerline curve id.
        bar_id (str): the bar id.
        grounds (dict): ``{ground_id: oid}``.
        standoff_mm (float): how far behind the joints the base stands.
        flip (bool | None): None = decide the side here (automatic rule, then
            :func:`_side_from_document`); True / False = the user's explicit
            side (a Flip option), taken as is.
        soup_cache (dict | None): shared ground-tessellation memo (see
            ``rhino_walkable_ground._bar_ground_soups``).
    """
    result, err = ikf._resolve_arm_tools_on_bar(curve)
    if err is not None:
        return None, err
    _bar_id, left, right = result  # left/right = (male_joint_oid, tool_oid)

    center_mm = _grabbed_joint_center_mm(left, right)
    # Read before the write below replaces it: the saved base is one of the signals.
    saved_base = ikf._read_saved_assembly_base_frame(curve)
    no_base = "no base could be placed: no assigned/meshable WalkableGround, or see the command history."

    def place(flip_now):
        """The base on one side of the bar, with the heading diagnosis."""
        diag = {}
        frame = rwg.default_base_frame_for_bar(
            curve, bar_id, grounds, standoff_mm=standoff_mm, center_mm=center_mm,
            flip=flip_now, diag_out=diag, soup_cache=soup_cache, verbose=True,
        )
        return frame, diag

    base_frame, diag = place(bool(flip))
    if base_frame is None:
        return None, no_base
    if flip is None:
        # * The automatic side was placed first; the document may know better.
        source = _side_from_document(bar_id, diag["heading"], np.asarray(base_frame, dtype=float)[:3, 2],
                                     saved_base)
        if source is not None:
            base_frame, diag = place(True)
            if base_frame is None:
                return None, no_base
            diag["side_source"] = source
            print(f"RSIKKeyframeAll: {bar_id}: base on the other side of the bar, as {source} say(s).")
    else:
        diag["side_source"] = "your Flip"
    ikf._write_assembly_base_frame(curve, base_frame)
    # base +Z is the ground normal at the placed base -- what the L/R tool-side
    # rule needs alongside the heading.
    diag["ground_normal"] = np.asarray(base_frame, dtype=float)[:3, 2]
    return (bar_id, curve, base_frame, left[1], right[1], diag), None


@session_log.timed()
def _solve_one(process, ctx, include_self, include_env, base_search="single", soups=None, moved=None):
    """Solve one bar's chain on Cindy's mirror, from the process's scenes, at its placed base. True on success.

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
        mirror, movements = ikf.process_planning.keyframe_movements(process, sc.sticky, f"bars/{bar_id}")
        with mirror.lend() as planner:
            solved, used_base = ikf._solve_chain_with_sampling(
                planner, movements, base_frame,
                brep_id=None, heading_mm=None,
                include_self=include_self, include_env=include_env,
                viz=None, urdf=process.robots[process.actions[f"{bar_id}_J"].robot].urdf,
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
    order): if its grasp and base picks are stored (so the process holds the hold),
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
    process = ikf._core_process("RSIKKeyframeAll")
    resolved, need_pick, failed = [], [], []
    check_collision = bool(include_self or include_env)
    for bar_id in sorted(hold_plan, key=lambda b: hold_plan[b]["hold_start_seq"]):
        oid = bar_map[bar_id][0]
        grasp_raw = rs.GetUserText(oid, config.KEY_SUPPORT_GRASP_FRAME)
        base_raw = rs.GetUserText(oid, config.KEY_SUPPORT_BASE_FRAME)
        if not grasp_raw or not base_raw or f"{bar_id}_H" not in process.actions:
            need_pick.append(bar_id)
            continue
        robot_name = hold_plan[bar_id]["robot_name"]
        print(f"RSIKKeyframeAll: re-solving {robot_name}'s hold on {bar_id} ...")
        try:
            mirror, template = ikf.process_planning.hold_solve_state(process, sc.sticky, f"bars/{bar_id}")
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
                found = ikf.process_planning.hold_release_collisions(
                    process, sc.sticky, f"bars/{bar_id}", ikf.process_source.pose_from_mm(base_mm),
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


def _side_undecided(diag) -> bool:
    """True when nothing decided which side of the bar the base stands on.

    The anchor insertion axes said nothing (``ambiguous``), the document had no
    hand-picked tool sides or saved base to go by, and the user has not answered
    for this bar yet (``side_source`` empty) -- so the side is the open-ground
    guess and worth a look.
    """
    diag = diag or {}
    return bool(diag.get("ambiguous")) and not diag.get("side_source")


def _undecided_ids(placed):
    """Bar ids whose side is still the open-ground guess (see :func:`_side_undecided`)."""
    return sorted(ctx[0] for ctx in placed if _side_undecided(ctx[5]))


def _off_guide_ids(placed):
    """Bar ids whose base could not reach its guide line because the walkable ground ran out."""
    return sorted(ctx[0] for ctx in placed if float((ctx[5] or {}).get("off_guide_mm", 0.0)) > 1.0)


def _apply_tool_sides(placed, verbose=True, only=None):
    """Re-place each placed bar's tools so L/R matches where its base stands.

    Which end of a bar takes the LEFT tool follows from the approach, not from
    the bar -- see ``core.rhino_tool_place.assign_tool_sides_from_heading``, which
    also leaves hand-picked (RSJointEdit) sides alone. Run after every placement
    and every flip so the arms never end up crossed.

    ! Re-placing a tool DELETES the old instance and inserts a new one, so any
    tool oid captured before this call is dead. Every corrected bar therefore has
    its ctx refreshed in place -- without that, ``_solve_one`` later hands a stale
    guid to ``rs.coercerhinoobject`` and dies with "does not exist in ObjectTable".

    Args:
        placed (list): the placed contexts, updated in place.
        verbose (bool): print a line per correction.
        only (set | None): bar ids to look at; None = every placed bar.
    """
    n_fixed = 0
    for i, ctx in enumerate(placed):
        bar_id, curve, diag = ctx[0], ctx[1], (ctx[5] or {})
        if diag.get("heading") is None or (only is not None and bar_id not in only):
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


def _clear_base_preview():
    """Delete whatever is left on the two base preview layers (guides, markers, circles).

    This command bakes none of its own; the layers can still hold leftovers of
    an aborted single-bar run. Idempotent and never raises: it runs from a
    ``finally``, where an exception would mask whatever actually went wrong.
    """
    try:
        base_guide_viz.clear_base_guides()
        base_frame_viz.clear_base_frames()
    except Exception as exc:  # noqa: BLE001
        session_log.caught(exc)
        print(f"RSIKKeyframeAll: failed to clear the base preview layers ({exc}); continuing.")


def _review_marks(ctx, standoff_mm):
    """The review marks of one placed bar, for ``dynamic_preview.MarkerConduit.set_markers``.

    A labelled dot at the placed base with a line along its heading (where the
    robot faces), and a dot where the base would stand after a Flip: the standoff
    on the other side of the bar's ground point (not snapped to the ground; it
    only shows which side that is).

    Returns:
        tuple: ``(markers, lines)`` in document units.
    """
    bar_id, base_frame, diag = ctx[0], np.asarray(ctx[2], dtype=float), (ctx[5] or {})
    scale = 1.0 / doc_unit_scale_to_mm()

    def point(xyz_mm):
        p = np.asarray(xyz_mm, dtype=float) * scale
        return Rhino.Geometry.Point3d(float(p[0]), float(p[1]), float(p[2]))

    origin, heading = base_frame[:3, 3], base_frame[:3, 0]
    markers = [(point(origin), f"{bar_id} base", _REVIEW_BASE_COLOR)]
    lines = [(point(origin), point(origin + heading * _REVIEW_ARROW_MM), _REVIEW_BASE_COLOR)]
    ground_point = diag.get("ground_point")
    if ground_point is not None:
        other = np.asarray(ground_point, dtype=float) + heading * float(standoff_mm)
        markers.append((point(other), "other side", _REVIEW_OTHER_COLOR))
    return markers, lines


def _show_bar_for_review(ctx, standoff_mm, conduit, viewport):
    """Select the bar (centre line + tube), mark its base, and zoom the viewport onto both."""
    bar_id, curve = ctx[0], ctx[1]
    rs.UnselectAllObjects()
    targets = [curve]
    tube = _find_existing_tube(curve)
    if tube is not None:
        targets.append(tube)
    rs.SelectObjects(targets)
    markers, lines = _review_marks(ctx, standoff_mm)
    conduit.set_markers(markers, lines)
    if viewport is not None:
        box = rs.coercecurve(curve).GetBoundingBox(True)
        for point, _label, _rgb in markers:
            box.Union(point)
        box.Inflate(0.25 * max(box.Diagonal.Length, 1.0))
        viewport.ZoomBoundingBox(box)
    sc.doc.Views.Redraw()
    print(f"RSIKKeyframeAll: {bar_id}: {(ctx[5] or {}).get('detail', 'side undecided')}")


def _ask_keep_or_flip(bar_id, index, total):
    """One bar's question in the walk-through: ``"keep"`` / ``"flip"`` / ``"keep_rest"``, None on Esc."""
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(f"{bar_id} ({index}/{total}): base side undecided, standing on the more open side "
                        "(selected, base marked in red). Keep this side or Flip?")
    keep_idx = go.AddOption("Keep")
    flip_idx = go.AddOption("Flip")
    rest_idx = go.AddOption("KeepRest")
    go.SetCommandPromptDefault("Keep")
    go.AcceptNothing(True)
    while True:
        result = session_log.ask(go)
        if result == Rhino.Input.GetResult.Nothing:
            return "keep"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == keep_idx:
                return "keep"
            if chosen == flip_idx:
                return "flip"
            if chosen == rest_idx:
                return "keep_rest"
            continue
        return None


def _review_undecided(rwg, placed, flipped, standoff, grounds, soup_cache):
    """Walk through the bars whose side nothing decided, one by one.

    Each bar in turn is selected and zoomed to, its base and the other side are
    marked by a display conduit (nothing baked), and the command line asks Keep /
    Flip / KeepRest. Flip re-places the base on the other side (``_flip_bars``,
    which also re-places the tools). KeepRest keeps every remaining bar as placed;
    Esc stops the walk-through and leaves the remaining bars undecided, so the
    summary still names them. The viewport is put back afterwards.

    Returns:
        list: the bar ids answered (kept or flipped).
    """
    todo = [ctx[0] for ctx in placed if _side_undecided(ctx[5])]
    if not todo:
        return []
    print(f"RSIKKeyframeAll: {len(todo)} bar(s) have no side to go by (anchors insert vertically or cancel): "
          f"{', '.join(todo)}. Walking through them; Enter keeps the placed side.")
    view = sc.doc.Views.ActiveView
    viewport = view.ActiveViewport if view is not None else None
    if viewport is not None:
        viewport.PushViewProjection()  # the zooms below are undone at the end
    answered = []
    try:
        with ikf.dynamic_preview.marker_viz() as conduit:
            for index, bar_id in enumerate(todo, start=1):
                position = next(i for i, c in enumerate(placed) if c[0] == bar_id)
                _show_bar_for_review(placed[position], standoff, conduit, viewport)
                answer = _ask_keep_or_flip(bar_id, index, len(todo))
                if answer is None:
                    print(f"RSIKKeyframeAll: walk-through stopped; {len(todo) - index + 1} bar(s) left undecided.")
                    break
                if answer == "flip":
                    _flip_bars(rwg, placed, flipped, {bar_id}, standoff, grounds, soup_cache)
                    _apply_tool_sides(placed, only={bar_id})
                    _show_bar_for_review(placed[position], standoff, conduit, viewport)
                else:
                    placed[position][5]["side_source"] = "your answer in the walk-through"
                answered.append(bar_id)
                if answer == "keep_rest":
                    for other in todo[index:]:
                        ctx = next(c for c in placed if c[0] == other)
                        ctx[5]["side_source"] = "your answer in the walk-through"
                        answered.append(other)
                    break
    finally:
        rs.UnselectAllObjects()
        if viewport is not None:
            viewport.PopViewProjection()
        sc.doc.Views.Redraw()
    return answered


def _ask_base_decision(note=""):
    """Off-ramp prompt with the Flip options; mirrors ``_ask_save_base_or_continue``.

    Returns ``"continue"`` / ``"save_and_exit"`` / ``"cancel"`` (the same strings
    the single-bar command uses) or ``"flip_all"`` / ``"flip_one"``, which the
    caller handles before re-prompting.

    Args:
        note (str): text shown after "Bases placed", e.g. how many bars are still undecided.
    """
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
    undecided = _undecided_ids(placed)
    if undecided:
        lines.append("")
        lines.append(f"Side undecided -- VERIFY THE SIDE: {len(undecided)}  {', '.join(undecided)}")
        lines.append("  (their anchor joints say nothing about a side and the walk-through did not "
                     "answer for them; they stand on the more open side. FlipOne, or RSIKKeyframe > Flip.)")
    off_guide = _off_guide_ids(placed)
    if off_guide:
        lines.append("")
        lines.append(f"Base off its guide line (the ground ran out before the standoff): {len(off_guide)}"
                     f"  {', '.join(off_guide)}")
    # Sides that came from somewhere other than the automatic rule, grouped by what decided them.
    by_source = {}
    for ctx in placed:
        source = (ctx[5] or {}).get("side_source")
        if source:
            by_source.setdefault(source, []).append(ctx[0])
    if by_source:
        lines.append("")
        for source, ids in sorted(by_source.items()):
            lines.append(f"Side decided by {source}: {len(ids)}  {', '.join(sorted(ids))}")
    if skipped:
        lines.append("")
        lines.append("Skipped:")
        lines.extend(f"  {bid}: {reason}" for bid, reason in skipped)
    return "\n".join(lines)


def main():
    global base_frame_viz, base_guide_viz, rhino_tool_place

    ikf._reload_runtime_modules()
    # Pick up the extended default_base_frame_for_bar(center_mm=, standoff_mm=) /
    # derive_seed_base signatures without a Rhino restart (numpy half first).
    importlib.reload(_wg_np_module)
    rwg = importlib.reload(_rwg_module)
    base_frame_viz = importlib.reload(_base_frame_viz_module)
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

    # REQ2: standoff distance is user-typeable. The default is the OUTER base-guide
    # offset (625 mm), so the auto-placed base lands on the outer guide line and
    # 375 / 500 read as the closer alternatives drawn in front of it.
    standoff = rs.GetReal(
        "Robot base standoff from bar (mm)",
        float(ikf.config.IK_BASE_STANDOFF_MM),
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
    flipped = {}  # bar_id -> is this base on the other side from the automatic one?

    # ! Nothing below bakes preview geometry any more, but the two preview layers
    # are still cleared on every exit path (solved, ESC at any prompt,
    # SaveAndExit, or an exception), so leftovers of an aborted single-bar run
    # never survive a batch: neither RSUpdatePreview nor RSClearColorPreview
    # touches those layers.
    try:
        _clear_base_preview()
        for bar_id, curve in to_process:
            ctx, reason = _place_base(rwg, curve, bar_id, grounds, standoff,
                                      soup_cache=soup_cache)
            if ctx is None:
                skipped.append((bar_id, reason))
                continue
            placed.append(ctx)
            flipped[bar_id] = bool(ctx[5].get("flipped"))  # the side _place_base settled on

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

        print(f"RSIKKeyframeAll: placed base on {len(placed)} bar(s) "
              f"(standoff {standoff:.0f} mm).")

        # The bars whose side nothing decided: look at each one in turn.
        _review_undecided(rwg, placed, flipped, standoff, grounds, soup_cache)

        # Off-ramp: solve now, flip a base onto the other side of its bar, or keep
        # the placed bases and stop. Loops until the user continues or leaves.
        while True:
            undecided = _undecided_ids(placed)
            note = f" [{len(undecided)} still undecided: {', '.join(undecided)}]" if undecided else ""
            decision = _ask_base_decision(note)
            if decision == "flip_all":
                _flip_bars(rwg, placed, flipped, {c[0] for c in placed},
                           standoff, grounds, soup_cache)
            elif decision == "flip_one":
                # pick_bar_among logs its prompt through session_log.ask.
                bar_id = pick_bar_among(
                    [c[0] for c in placed],
                    "Pick the bar whose base should flip to its other side (centre line or tube)",
                )
                if not bar_id:
                    print("RSIKKeyframeAll: no bar picked; nothing flipped.")
                    continue
                _flip_bars(rwg, placed, flipped, {bar_id},
                           standoff, grounds, soup_cache)
            else:
                break
            # A flip re-decides the approach, so the tool sides follow it.
            _apply_tool_sides(placed)

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

        # One collision-option prompt for the whole batch, showing the process's
        # body count. The process is built once, after the bases and tool sides
        # are settled (they are in it).
        process = ikf._core_process("RSIKKeyframeAll")
        opts = ikf._ask_collision_options(env_count=len(process.bodies), offer_base_search=True)
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
            ok = _solve_one(process, ctx, include_self, include_env,
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

        # Calculation finished: clear the preview layers BEFORE the modal summary
        # so the per-bar solved/failed colors are the only thing left on screen.
        # (The finally below repeats this for every other exit path; both are
        # idempotent.) The saved base frames stay.
        _clear_base_preview()

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
        _clear_base_preview()
        # The base search's DEBUG trace (BaseSearch=Optimised) of the last bar searched.
        ikf.base_search_viz.hide()


if __name__ == "__main__":
    session_log.run(__file__, main)
