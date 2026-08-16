#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSJointEdit - Re-edit previously placed joint pairs.

Two modes, chosen at the first prompt:

**FlipJoint** (the default, and everything described below) - flip the
orientation of a joint half by clicking it.

**MoveJoint** - slide a joint along the bar it is attached to, carrying its own
bar and that bar's other joint with it.  Pick the joint, pick which of its two
bars should move, then give the slide as TWO points: what to align, and what to
align it with.  Both are free picks, so snap them to whatever you are lining up
by -- a joint face, a bar end, an intersection.  Only the component along the
bar is applied, because that is a joint's one degree of freedom; the across-bar
part is reported, not silently dropped.

The two joints on the moving bar keep their current separation, so moving one
forces the other to slide along its own bar; both joints' angles change, and
both pairs are rebuilt with their original ids so their tools survive.

The move is measured from each joint's half on its STATIONARY bar, whose block
origin already lies on that bar's centre-line -- see :func:`_anchor_point` for
why measuring from the half on the moving bar instead collapses the ~36-40 mm
gap between the two bar axes and leaves the pair unable to mate.

While picking, the three bars involved are colored (mover blue, its two hosts
green, every other bar grey) and the points being moved are marked, so what the
click will do is visible before it happens.  Esc at any prompt puts the bar,
both joint pairs and both tools back exactly where they were; Accept keeps the
move, and the whole command is one undo record afterwards.

Only bars carrying exactly two joints can be moved.  A bar with more would
carry the extra joints away from their mates -- which is also why a vertical or
horizontal bar full of joints is refused rather than dragged.

Click any placed female or male joint block anywhere in the document.  The
clicked block's side (female = le_rev, male = ln_rev) is toggled instantly
and the joint pair is re-placed — no confirmation step required.

  - **Click the female block** to toggle female joint orientation (le_rev).
  - **Click the male block** to toggle male joint orientation (ln_rev).
  - Click repeatedly to cycle back and forth.
  - Press **Escape** to exit the command.

Requires that the joint was placed by a recent version of RSJointPlace that
stores ``le_rev``, ``ln_rev``, ``variant_index``, ``joint_pair_name``,
``female_parent_bar``, and ``male_parent_bar`` as user-text keys on the
block instances.
"""

import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Shared placement primitives live in core.joint_placement; this command
# only owns the click loop + per-block-flip orchestration.
import rs_joint_place as _rjp  # for _reload_runtime_modules() side-effects

from core import config
from core import dynamic_preview
from core import geometry as _geometry_module

# Reload BEFORE the from-import below.  `from X import name` binds against the
# copy of X already in sys.modules, so the first run after a new helper lands in
# core.geometry would raise ImportError right here -- before ``main`` and its
# reload ever run, curable only by restarting Rhino.  dynamic_preview is
# reloaded for the same reason: a stale copy has no ``marker_viz``.
importlib.reload(_geometry_module)
importlib.reload(dynamic_preview)

from core.geometry import points_on_line_at_distance  # noqa: E402 -- after reload
from core.ground_placement import (
    GROUND_INSTANCES_LAYER,
    fk_ground_block_frame,
    place_ground_block,
    remove_placed_ground,
)
from core.joint_pair import get_joint_pair, load_joint_registry
from core.joint_pick_helpers import block_instance_frame
from core.joint_placement import (
    FEMALE_INSTANCES_LAYER,
    MALE_INSTANCES_LAYER,
    _numpy_to_rhino_transform,
    compute_variant_with_recovery,
    place_joint_blocks,
)
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    SEQ_COLOR_ACTIVE,
    SEQ_COLOR_BUILT,
    SEQ_COLOR_UNBUILT,
    TUBE_BAR_ID_KEY,
    ensure_bar_preview,
    get_all_bars,
    paint_bar,
    repair_on_entry,
    reset_bar_color,
)
from core.rhino_block_import import require_block_definition
from core.rhino_helpers import curve_endpoints
from core.transforms import align_vectors
from core.rhino_tool_place import (
    cycle_tool_at_tool_instance,
    get_tool_name_for_joint,
    place_tool_by_name_at_male_joint,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _placed_joint_filter(rhino_object, geometry, component_index):
    """Geometry filter -- accept any placed joint block (female/male/ground)
    OR robotic tool instance."""
    layer = rs.ObjectLayer(rhino_object.Id)
    return layer in (
        FEMALE_INSTANCES_LAYER,
        MALE_INSTANCES_LAYER,
        GROUND_INSTANCES_LAYER,
        config.LAYER_TOOL_INSTANCES,
    )


def _find_bar_curve(bar_id):
    """Return the Rhino object ID of the bar curve with the given bar_id, or None."""
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_ID_KEY) == bar_id:
            return oid
    return None


def _remove_placed_joint(joint_id):
    """Delete every placed female/male block instance for *joint_id*.

    Both callers here delete-then-re-place, so a miss does not fail loudly: the
    re-place simply inserts a SECOND pair and the old one stays behind.  That is
    where duplicate blocks on a bar come from -- two instances sharing one
    ``joint_id``, which then makes RSIKKeyframe count three "joints" on a bar
    that visibly has two.

    So this looks blocks up by their ``joint_id`` USER TEXT, which is written to
    every half at placement time and is what the rest of this command already
    trusts.  The object name (``{joint_id}_female`` / ``_male``) is still
    checked, but only as a second net: it is a convenience set at placement
    time, and anything that re-creates an object without carrying the name over
    silently breaks a name-only lookup.

    Returns the number of blocks deleted, so a caller can notice a miss.
    """
    to_delete = set()
    for suffix in ("_female", "_male"):
        for oid in rs.ObjectsByName(f"{joint_id}{suffix}") or []:
            to_delete.add(oid)
    for layer in (FEMALE_INSTANCES_LAYER, MALE_INSTANCES_LAYER):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            if rs.GetUserText(oid, "joint_id") == joint_id:
                to_delete.add(oid)
    if to_delete:
        rs.DeleteObjects(list(to_delete))
    return len(to_delete)


# ---------------------------------------------------------------------------
# MoveJoint -- slide a joint, dragging its bar and the bar's far joint with it
# ---------------------------------------------------------------------------


def _joints_touching_bar(bar_id):
    """Return ``{joint_id: block_oid}`` for every joint with a half on *bar_id*.

    One entry per joint, and the block returned is the half sitting ON *bar_id*
    -- which is the half whose world position moves when that bar moves.
    """
    found = {}
    for layer in (FEMALE_INSTANCES_LAYER, MALE_INSTANCES_LAYER):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            joint_id = rs.GetUserText(oid, "joint_id")
            if joint_id and rs.GetUserText(oid, "parent_bar_id") == bar_id:
                found[joint_id] = oid
    return found


def _block_world_point(block_id):
    """World position of a joint block instance, as a numpy 3-vector.

    ``None`` in gives ``None`` back: callers look blocks up by joint id after a
    re-place, and a missing id is a real (reported) outcome, not a crash --
    ``rs.BlockInstanceInsertPoint(None)`` would raise instead of returning None.
    """
    if block_id is None:
        return None
    pt = rs.BlockInstanceInsertPoint(block_id)
    if pt is None:
        return None
    return np.array([float(pt[0]), float(pt[1]), float(pt[2])], dtype=float)


def _rigid_move_matrix(from_a, from_b, to_a, to_b):
    """4x4 that carries segment *from_a*-*from_b* onto *to_a*-*to_b*.

    Both segments are the same length -- that is what
    :func:`_points_on_line_at_distance` guarantees -- so this is a pure
    rotate-and-shift with no scaling.  The spin about the segment's own axis is
    unconstrained; the minimal rotation is used, which is fine because the bar
    is a tube (spinning it about its axis changes nothing visible) and the
    joints are re-solved afterwards regardless.
    """
    rotation = align_vectors(
        np.asarray(from_b, dtype=float) - np.asarray(from_a, dtype=float),
        np.asarray(to_b, dtype=float) - np.asarray(to_a, dtype=float),
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (
        np.asarray(to_a, dtype=float) - rotation @ np.asarray(from_a, dtype=float)
    )
    return matrix


def _replace_joint_pair(joint_id, le_bar_id, ln_bar_id, pair, le_rev, ln_rev):
    """Re-solve and re-place one joint pair, keeping its id and its tool.

    Same round-trip the flip path uses: remember the tool by name, delete the
    blocks, re-solve, re-place, put the tool back.  ``place_joint_blocks``
    rebuilds the joint id from the two bar ids, so it comes back identical as
    long as neither bar was renamed -- checked rather than assumed, because
    every tool is tagged to that id.

    Returns ``(ok, le_rev, ln_rev)``.  The flags come back because the solver
    may auto-flip one side to escape a degenerate minimum; a repick that
    re-seeded from the flags passed IN would undo that recovery every round.
    """
    le_id = _find_bar_curve(le_bar_id)
    ln_id = _find_bar_curve(ln_bar_id)
    if le_id is None or ln_id is None:
        print(f"RSJointEdit: {joint_id}: bar curve missing; not re-placed.")
        return False, le_rev, ln_rev

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)
    variant, _rec, le_rev, ln_rev = compute_variant_with_recovery(
        le_start, le_end, ln_start, ln_end, le_rev, ln_rev,
        pair=pair, recover_side="female", log_prefix="RSJointEdit",
    )
    prev_tool_name = get_tool_name_for_joint(joint_id)
    n_removed = _remove_placed_joint(joint_id)
    if n_removed != 2:
        # 0 means the re-place below ADDS a pair rather than replacing one;
        # more than 2 means duplicates were already there and have just been
        # cleaned up.  Either way the user should hear about it.
        print(
            f"RSJointEdit: NOTE - removed {n_removed} block(s) for {joint_id}, "
            "expected 2 (one female + one male)."
        )
    _, male_id, new_joint_id = place_joint_blocks(
        variant, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
    )
    if new_joint_id != joint_id:
        print(
            f"RSJointEdit: WARNING - joint id changed {joint_id} -> {new_joint_id}; "
            "its tool tag no longer matches and will need re-placing."
        )
    place_tool_by_name_at_male_joint(male_id, new_joint_id, pair, prev_tool_name)
    return True, le_rev, ln_rev


# Marker colors for the MoveJoint preview.  Deliberately unlike the bar
# blue/green/grey so a marker never reads as a bar that changed color.
MARKER_ANCHOR = (255, 230, 0)  # yellow -- the point that lands on your click
MARKER_FOLLOW = (255, 150, 0)  # orange -- the far joint, dragged along
MARKER_CENTRE = (0, 200, 255)  # cyan   -- the joint's screw centre


def _pt3d(point):
    """numpy 3-vector -> Rhino Point3d (conduits and pickers want the latter)."""
    return Rhino.Geometry.Point3d(
        float(point[0]), float(point[1]), float(point[2])
    )


def _anchor_point(joint_id, host_bar_id):
    """The point MoveJoint slides: *joint_id*'s block origin on *host_bar_id*.

    A joint has two halves, one per bar, and each half's block origin lies on
    ITS OWN bar's centre-line (docs/coordinate_conventions.md: "the origin lies
    on the bar centre-line, at the midpoint of the hole that sandwiches the
    bar").  The two bars themselves stay ``contact_distance_mm`` apart, ~36-40
    mm, so the two halves' origins are NOT the same point.

    Which half is picked decides whether the move is right or wrong:

    * half on the STATIONARY bar -- its origin is already on the curve the user
      picks a point on, so sliding it to that point is a pure slide along that
      bar and the bar-to-bar offset survives untouched.  This is the one to use.
    * half on the MOVING bar -- its origin is on the moving bar instead, so
      sending it to a point on the stationary bar's centre-line drags the two
      bar axes into each other: the offset collapses to zero, the bars
      interpenetrate, and the re-solve can no longer close the mate.
    """
    return _block_world_point(_joints_touching_bar(host_bar_id).get(joint_id))


def _screw_centre(joint_id, pair):
    """World origin of a joint's screw frame -- its visual centre, or None.

    Shown to the user, never used to drive the move.  Unlike either block
    origin this sits BETWEEN the two bar axes, so it reads as the middle of the
    joint; it is also the point the solver makes the two halves agree on, which
    is why both halves report it identically when the pair is properly mated.

    Same composition as ``rhino_joint_refresh._screw_frame``.
    """
    ids = rs.ObjectsByName(f"{joint_id}_female")
    if not ids:
        return None
    try:
        block_world, _name = block_instance_frame(ids[0])
    except ValueError:
        return None
    frame = np.asarray(block_world, dtype=float) @ np.asarray(
        pair.female.M_screw_from_block, dtype=float
    )
    return frame[:3, 3]


def _paint_move_context(moving_bar_id, stationary_bar_ids):
    """Grey out the bars that take no part; blue the mover, green the hosts.

    Same blue/green/grey as the RSIKKeyframe sequence preview, imported rather
    than re-declared so the colors keep meaning the same thing across commands.
    Returns the painted curve ids for the caller's ``finally``.
    """
    painted = []
    rs.EnableRedraw(False)
    try:
        for bar_id, oid in get_all_bars().items():
            if bar_id == moving_bar_id:
                color = SEQ_COLOR_ACTIVE
            elif bar_id in stationary_bar_ids:
                color = SEQ_COLOR_BUILT
            else:
                color = SEQ_COLOR_UNBUILT
            paint_bar(oid, color)
            painted.append(oid)
    finally:
        rs.EnableRedraw(True)
    return painted


def _pick_bar_to_move(bar_a_id, bar_b_id):
    """Click the bar that should move.  Returns its bar id, or None on Esc.

    Restricted to the two bars the selected joint connects -- either their
    centerline or their tube -- so an unrelated bar simply is not selectable
    rather than being accepted and rejected afterwards.
    """
    allowed = {bar_a_id, bar_b_id}

    def _filter(rhino_object, geometry, component_index):
        oid = rhino_object.Id
        if rs.GetUserText(oid, BAR_ID_KEY) in allowed:
            return True
        # Tubes carry their bar id under their own key, not BAR_ID_KEY.
        return rs.GetUserText(oid, TUBE_BAR_ID_KEY) in allowed

    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt(f"Select the bar that should move ({bar_a_id} or {bar_b_id})")
    go.EnablePreSelect(False, False)
    go.SetCustomGeometryFilter(_filter)
    if go.Get() != Rhino.Input.GetResult.Object:
        return None
    oid = go.Object(0).ObjectId
    return rs.GetUserText(oid, BAR_ID_KEY) or rs.GetUserText(oid, TUBE_BAR_ID_KEY)


def _run_move_joint():
    """Slide one joint along its bar, carrying its bar and the bar's far joint."""
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select the joint to move")
    go.EnablePreSelect(False, False)
    go.SetCustomGeometryFilter(_placed_joint_filter)
    if go.Get() != Rhino.Input.GetResult.Object:
        return
    clicked_id = go.Object(0).ObjectId
    if rs.ObjectLayer(clicked_id) not in (FEMALE_INSTANCES_LAYER, MALE_INSTANCES_LAYER):
        print("RSJointEdit: MoveJoint works on female/male joint blocks only.")
        return

    near_joint_id = rs.GetUserText(clicked_id, "joint_id")
    pair_name = rs.GetUserText(clicked_id, "joint_pair_name")
    le_bar_id = rs.GetUserText(clicked_id, "female_parent_bar")
    ln_bar_id = rs.GetUserText(clicked_id, "male_parent_bar")
    le_rev = rs.GetUserText(clicked_id, "le_rev") == "True"
    ln_rev = rs.GetUserText(clicked_id, "ln_rev") == "True"
    if not (near_joint_id and pair_name and le_bar_id and ln_bar_id):
        print("RSJointEdit: joint metadata incomplete; re-place it with RSJointPlace.")
        return

    moving_bar_id = _pick_bar_to_move(le_bar_id, ln_bar_id)
    if moving_bar_id is None:
        return
    sliding_bar_id = ln_bar_id if moving_bar_id == le_bar_id else le_bar_id

    # The moving bar must carry EXACTLY the two joints this operation keeps
    # consistent.  A third joint would be carried along while its mate stayed
    # behind -- silently breaking a joint the user is not looking at.  This is
    # also what stops a vertical or horizontal bar being moved: those carry
    # joints all along them, so the check catches them without needing to know
    # anything about a bar's orientation.
    on_moving = _joints_touching_bar(moving_bar_id)
    if near_joint_id not in on_moving:
        print(f"RSJointEdit: {near_joint_id} has no half on {moving_bar_id}.")
        return
    if len(on_moving) != 2:
        others = ", ".join(sorted(j for j in on_moving if j != near_joint_id)) or "none"
        print(
            f"RSJointEdit: {moving_bar_id} carries {len(on_moving)} joint(s) "
            f"-- {near_joint_id}, plus: {others}.  MoveJoint handles exactly 2.\n"
            "  Moving it would carry the extra joint(s) away from their mates."
        )
        return
    far_joint_id = next(j for j in on_moving if j != near_joint_id)

    far_block = on_moving[far_joint_id]

    # The far joint's OTHER bar -- the line it has to stay on while it slides.
    far_le = rs.GetUserText(far_block, "female_parent_bar")
    far_ln = rs.GetUserText(far_block, "male_parent_bar")
    far_pair_name = rs.GetUserText(far_block, "joint_pair_name")
    far_le_rev = rs.GetUserText(far_block, "le_rev") == "True"
    far_ln_rev = rs.GetUserText(far_block, "ln_rev") == "True"
    far_host_bar_id = far_ln if far_le == moving_bar_id else far_le

    moving_curve = _find_bar_curve(moving_bar_id)
    sliding_curve = _find_bar_curve(sliding_bar_id)
    far_host_curve = _find_bar_curve(far_host_bar_id)
    if None in (moving_curve, sliding_curve, far_host_curve):
        print("RSJointEdit: could not resolve all three bar curves.")
        return

    try:
        pair = get_joint_pair(pair_name)
        far_pair = get_joint_pair(far_pair_name)
    except KeyError as exc:
        print(f"RSJointEdit: joint pair no longer registered: {exc}")
        return

    far_host_start, far_host_end = curve_endpoints(far_host_curve)

    # The one axis the near joint may move along.  Fixed for the whole command:
    # the bar it slides on is the one that does NOT move.
    slide_start, slide_end = curve_endpoints(sliding_curve)
    slide_dir = slide_end - slide_start
    slide_len = float(np.linalg.norm(slide_dir))
    if slide_len <= 1e-9:
        print(f"RSJointEdit: {sliding_bar_id} has zero length; cannot slide along it.")
        return
    slide_dir = slide_dir / slide_len

    if _anchor_point(near_joint_id, sliding_bar_id) is None or (
        _anchor_point(far_joint_id, far_host_bar_id) is None
    ):
        print(
            "RSJointEdit: a joint has no half on its stationary bar -- its "
            "parent_bar_id tags disagree with the bars.  Re-place it with "
            "RSJointPlace (or run RSUpdatePreview to see the broken links)."
        )
        return

    print(
        f"RSJointEdit: moving {moving_bar_id}.  {near_joint_id} slides along "
        f"{sliding_bar_id}; {far_joint_id} follows along {far_host_bar_id}."
    )

    # Everything needed to put the model back, captured before the first edit.
    orig_start, orig_end = curve_endpoints(moving_curve)
    orig_revs = {
        near_joint_id: (le_rev, ln_rev),
        far_joint_id: (far_le_rev, far_ln_rev),
    }
    revs = dict(orig_revs)
    joint_bars = {
        near_joint_id: (le_bar_id, ln_bar_id),
        far_joint_id: (far_le, far_ln),
    }
    joint_pairs = {near_joint_id: pair, far_joint_id: far_pair}

    def _replace_both():
        """Re-solve and re-place both pairs off the bars' current positions."""
        ok_all = True
        for jid in (near_joint_id, far_joint_id):
            le_b, ln_b = joint_bars[jid]
            ok, new_le, new_ln = _replace_joint_pair(
                jid, le_b, ln_b, joint_pairs[jid], *revs[jid]
            )
            revs[jid] = (new_le, new_ln)
            ok_all = ok_all and ok
        return ok_all

    def _restore():
        """Undo every edit this command made, without relying on Ctrl+Z.

        The bar is a line, so the rigid transform carrying its current
        endpoints back onto the ones captured above restores it exactly (its
        spin about its own axis is unobservable).  The joints and tools are
        then rebuilt from the restored bars with the flags they started with.
        """
        cur_start, cur_end = curve_endpoints(moving_curve)
        rs.TransformObject(
            moving_curve,
            _numpy_to_rhino_transform(
                _rigid_move_matrix(cur_start, cur_end, orig_start, orig_end)
            ),
        )
        ensure_bar_preview(moving_curve, float(config.BAR_RADIUS))
        paint_bar(moving_curve, SEQ_COLOR_ACTIVE)  # the tube was rebuilt
        revs.update(orig_revs)
        _replace_both()
        rs.Redraw()
        print("RSJointEdit: MoveJoint cancelled; everything is back where it was.")

    def _pick_alignment_pair():
        """Two free points: what to line up, and what to line it up with.

        Deliberately NOT constrained to the sliding bar.  The joint's own
        origin is an invisible point inside the block, so picking a target for
        it is guesswork; snapping both ends of a measurement to real features
        -- a joint face, a bar end, an intersection -- is how you actually
        align something.  Only the component along the bar is usable (a joint
        has one degree of freedom, along its bar), which the caller resolves;
        this returns the raw pair so that decision lives in one place.

        Returns ``(from_point, to_point, outcome)`` with outcome "ok",
        "cancel" (Esc -- the caller reverts) or "done" (Enter -- keep as is).
        """
        def _outcome(result):
            return "cancel" if result == Rhino.Input.GetResult.Cancel else "done"

        first = Rhino.Input.Custom.GetPoint()
        first.SetCommandPrompt(
            f"Pick the point to align FROM, on or near {near_joint_id} "
            "(Esc cancels the whole move)"
        )
        result = first.Get()
        if result != Rhino.Input.GetResult.Point:
            return None, None, _outcome(result)
        base = first.Point()

        second = Rhino.Input.Custom.GetPoint()
        second.SetCommandPrompt(f"Pick the point to align TO ({near_joint_id} slides there)")
        second.SetBasePoint(base, True)
        second.DrawLineFromPoint(base, True)
        result = second.Get()
        if result != Rhino.Input.GetResult.Point:
            return None, None, _outcome(result)
        return base, second.Point(), "ok"

    def _show_markers(markers):
        """Draw the point that will move, the one that follows, and both centres."""
        near_anchor = _anchor_point(near_joint_id, sliding_bar_id)
        far_anchor = _anchor_point(far_joint_id, far_host_bar_id)
        near_centre = _screw_centre(near_joint_id, pair)
        far_centre = _screw_centre(far_joint_id, far_pair)
        marks, lines = [], []
        if near_anchor is not None:
            marks.append((_pt3d(near_anchor), f"{near_joint_id} moves", MARKER_ANCHOR))
        if far_anchor is not None:
            marks.append((_pt3d(far_anchor), f"{far_joint_id} follows", MARKER_FOLLOW))
        # The screw centre is what the joint LOOKS centred on; the anchor is
        # what the move is measured from.  Draw the offset between them so it
        # is obvious they are not the same point.
        for anchor, centre in ((near_anchor, near_centre), (far_anchor, far_centre)):
            if centre is None:
                continue
            marks.append((_pt3d(centre), "", MARKER_CENTRE))
            if anchor is not None:
                lines.append((_pt3d(anchor), _pt3d(centre), MARKER_CENTRE))
        markers.set_markers(marks, lines)

    painted = _paint_move_context(
        moving_bar_id, {sliding_bar_id, far_host_bar_id}
    )
    # The undo record opens AFTER the coloring so a later Ctrl+Z replays the
    # geometry edits only -- the colors are already restored by the finally.
    undo = sc.doc.BeginUndoRecord("RSJointEdit MoveJoint")
    moved = False
    try:
        with dynamic_preview.marker_viz() as markers:
            while True:
                near_anchor = _anchor_point(near_joint_id, sliding_bar_id)
                far_anchor = _anchor_point(far_joint_id, far_host_bar_id)
                if near_anchor is None or far_anchor is None:
                    print("RSJointEdit: lost track of the joints after the move.")
                    return
                span = float(np.linalg.norm(far_anchor - near_anchor))
                _show_markers(markers)

                from_pt, to_pt, outcome = _pick_alignment_pair()
                if outcome == "cancel":
                    if moved:
                        _restore()
                    return
                if outcome != "ok":
                    print("RSJointEdit: MoveJoint finished.")
                    return

                # Split the pick into the part the joint CAN do and the part it
                # cannot.  Sliding along the bar is one degree of freedom, so
                # only the along-bar component survives; report the rest rather
                # than silently under-moving and looking broken.
                delta = np.array(
                    [to_pt.X - from_pt.X, to_pt.Y - from_pt.Y, to_pt.Z - from_pt.Z],
                    dtype=float,
                )
                along = float(np.dot(delta, slide_dir))
                across = float(np.linalg.norm(delta - along * slide_dir))
                if abs(along) < 1e-6:
                    print(
                        f"RSJointEdit: those two points are level along "
                        f"{sliding_bar_id} -- nothing to slide.  Pick again."
                    )
                    continue
                if across > 0.05:
                    print(
                        f"RSJointEdit: sliding {along:.1f} mm along "
                        f"{sliding_bar_id}; the {across:.1f} mm across the bar is "
                        "ignored -- a joint only slides along its bar."
                    )
                target_pt = near_anchor + along * slide_dir

                landings = points_on_line_at_distance(
                    target_pt, span, far_host_start, far_host_end
                )
                if not landings:
                    print(
                        f"RSJointEdit: {moving_bar_id} cannot reach that far -- "
                        f"{far_joint_id} would have to leave {far_host_bar_id} to "
                        f"stay {span:.1f} mm away.  Nothing changed; pick a "
                        "closer point."
                    )
                    continue
                # Two landings is the NORMAL case: a sphere cuts a line twice,
                # one each side of the perpendicular foot.  Take the one nearest
                # where the far joint already is, so the bar keeps leaning the
                # way it leans instead of flipping end-over-end.
                landing = min(
                    landings, key=lambda p: float(np.linalg.norm(p - far_anchor))
                )
                if len(landings) > 1:
                    print(
                        f"RSJointEdit: two placements fit; using the one nearer "
                        f"{far_joint_id}'s current position.  Repick to try the "
                        "other."
                    )

                # Both ends of this transform run stationary-bar-point ->
                # stationary-bar-point, so the moving bar slides along its hosts
                # instead of being dragged onto their centre-lines.
                rs.TransformObject(
                    moving_curve,
                    _numpy_to_rhino_transform(
                        _rigid_move_matrix(
                            near_anchor, far_anchor, target_pt, landing
                        )
                    ),
                )
                ensure_bar_preview(moving_curve, float(config.BAR_RADIUS))
                paint_bar(moving_curve, SEQ_COLOR_ACTIVE)  # the tube was rebuilt
                moved = True

                if not _replace_both():
                    print("RSJointEdit: a joint failed to re-place; undoing.")
                    _restore()
                    return
                rs.Redraw()
                _show_markers(markers)

                accept = Rhino.Input.Custom.GetOption()
                accept.SetCommandPrompt("Accept this position?")
                accept_idx = accept.AddOption("Accept")
                accept.AddOption("Repick")
                accept.SetCommandPromptDefault("Accept")
                accept.AcceptNothing(True)
                res = accept.Get()
                if res == Rhino.Input.GetResult.Cancel:
                    _restore()
                    return
                if res != Rhino.Input.GetResult.Option or (
                    accept.OptionIndex() == accept_idx
                ):
                    print(
                        f"RSJointEdit: {moving_bar_id} moved; both joints re-placed."
                    )
                    return
                # Repick: loop round and re-read the anchors, because the blocks
                # were deleted and re-made.
    finally:
        sc.doc.EndUndoRecord(undo)
        for oid in painted:
            reset_bar_color(oid)
        rs.Redraw()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _flip_ground_block(clicked_id):
    """Flip a placed ground-joint block's X axis along the bar.

    Reads ``ground_joint_name`` / ``parent_bar_id`` / ``position_mm`` /
    ``rotation_deg`` / ``flipped`` from the clicked instance, deletes it,
    and re-bakes via :func:`core.ground_placement.place_ground_block`
    with ``flipped`` toggled.  ``jr`` is preserved (the flip post-multi-
    plies ``M_block_from_bar`` by ``R_y(pi)``, which keeps block-local
    +Y -- so world-up alignment is preserved and ``jr`` does not change).
    """
    import math
    import numpy as np

    joint_id = rs.GetUserText(clicked_id, "joint_id")
    ground_name = rs.GetUserText(clicked_id, "ground_joint_name")
    bar_id = rs.GetUserText(clicked_id, "parent_bar_id")
    jp_text = rs.GetUserText(clicked_id, "position_mm")
    jr_text = rs.GetUserText(clicked_id, "rotation_deg")
    flipped_text = rs.GetUserText(clicked_id, "flipped")
    if not joint_id or not ground_name or not bar_id or not jp_text or not jr_text:
        print(
            "RSJointEdit: Could not read ground-joint metadata from the selected block.\n"
            "  This block may have been placed by an older RSGroundPlace.\n"
            "  Re-place it with RSGroundPlace to enable re-editing."
        )
        return

    try:
        registry = load_joint_registry()
        ground = registry.ground_joints[ground_name]
    except KeyError:
        print(
            f"RSJointEdit: Ground joint '{ground_name}' is no longer registered."
        )
        return

    bar_curve_id = _find_bar_curve(bar_id)
    if bar_curve_id is None:
        print(f"RSJointEdit: Could not find bar curve {bar_id}.")
        return

    bar_start, bar_end = curve_endpoints(bar_curve_id)
    bar_start = np.asarray(bar_start, dtype=float)
    bar_end = np.asarray(bar_end, dtype=float)
    jp = float(jp_text)
    jr = math.radians(float(jr_text))
    # Older bakes (pre-flipped-flag) have no UserText -> default False.
    flipped = (flipped_text == "True") if flipped_text else False
    flipped = not flipped

    remove_placed_ground(joint_id)
    ground_oid, _ = place_ground_block(
        ground=ground,
        bar_id=bar_id,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=jp,
        jr=jr,
        flipped=flipped,
        joint_id=joint_id,
    )

    # Re-place the robotic tool so it follows the flipped ground frame.
    # Preserve whichever tool the user previously had attached to this joint.
    from core.rhino_tool_place import (  # noqa: PLC0415
        get_tool_name_for_joint,
        place_tool_by_name_at_ground_block,
    )
    prev_tool = get_tool_name_for_joint(joint_id)
    place_tool_by_name_at_ground_block(ground_oid, joint_id, prev_tool)

    print(f"RSJointEdit: ground {joint_id} flipped (flipped now {flipped}).")


def _ask_mode():
    """FlipJoint (the historical behaviour, and the Enter default) or MoveJoint."""
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(
        "Flip joint orientations, or move a joint along its bar"
    )
    flip_idx = go.AddOption("FlipJoint")
    move_idx = go.AddOption("MoveJoint")
    go.SetCommandPromptDefault("FlipJoint")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return "flip"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == flip_idx:
                return "flip"
            if chosen == move_idx:
                return "move"
            continue
        return None


def main():
    _rjp._reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSJointEdit")

    mode = _ask_mode()
    if mode is None:
        return
    if mode == "move":
        _run_move_joint()
        return

    # Continuous pick loop — no accept/confirm step.  Each click on a joint
    # block immediately flips that block's side and re-places the pair.
    # Press Escape to exit the command.
    while True:
        go = Rhino.Input.Custom.GetObject()
        go.SetCommandPrompt(
            "Click a joint block to flip it, or a tool to cycle it  (Escape to exit)"
        )
        go.EnablePreSelect(False, False)
        go.SetCustomGeometryFilter(_placed_joint_filter)
        result = go.Get()
        if result != Rhino.Input.GetResult.Object:
            print("RSJointEdit: Done.")
            return

        clicked_id = go.Object(0).ObjectId
        clicked_layer = rs.ObjectLayer(clicked_id)

        # Tool instance: cycle to next tool in registry, no joint changes.
        if clicked_layer == config.LAYER_TOOL_INSTANCES:
            cycle_tool_at_tool_instance(clicked_id)
            continue

        # Ground-joint instance: flip jr by 180 deg and re-bake at the
        # same (jp) along the same bar.  No mate-side recovery needed.
        if clicked_layer == GROUND_INSTANCES_LAYER:
            _flip_ground_block(clicked_id)
            continue

        # Read stored metadata before the block is deleted.
        joint_id = rs.GetUserText(clicked_id, "joint_id")
        joint_pair_name = rs.GetUserText(clicked_id, "joint_pair_name")
        le_bar_id = rs.GetUserText(clicked_id, "female_parent_bar")
        ln_bar_id = rs.GetUserText(clicked_id, "male_parent_bar")
        le_rev = rs.GetUserText(clicked_id, "le_rev") == "True"
        ln_rev = rs.GetUserText(clicked_id, "ln_rev") == "True"

        if not joint_id or not le_bar_id or not ln_bar_id or not joint_pair_name:
            print(
                "RSJointEdit: Could not read joint metadata from the selected block.\n"
                "  This joint may have been placed by an older version of RSJointPlace.\n"
                "  Re-place it with RSJointPlace to enable re-editing."
            )
            continue

        try:
            pair = get_joint_pair(joint_pair_name)
        except KeyError:
            print(
                f"RSJointEdit: Joint pair '{joint_pair_name}' is no longer registered."
            )
            continue

        # Validate block definitions for this pair.
        try:
            require_block_definition(
                pair.female.block_name, asset_path=pair.female.asset_path()
            )
            require_block_definition(
                pair.male.block_name, asset_path=pair.male.asset_path()
            )
        except RuntimeError as exc:
            print(f"RSJointEdit: {exc}")
            continue

        # Toggle the side that was clicked, and remember which side it was
        # so the auto-recovery (when the chosen orientation lands on a bad
        # local minimum) can flip the OTHER side.
        clicked_side = None
        if clicked_layer == FEMALE_INSTANCES_LAYER:
            le_rev = not le_rev
            clicked_side = "female"
        elif clicked_layer == MALE_INSTANCES_LAYER:
            ln_rev = not ln_rev
            clicked_side = "male"

        # Find the underlying bar curves.
        le_id = _find_bar_curve(le_bar_id)
        ln_id = _find_bar_curve(ln_bar_id)
        if le_id is None or ln_id is None:
            missing = [
                b for b, i in [(le_bar_id, le_id), (ln_bar_id, ln_id)] if i is None
            ]
            print(f"RSJointEdit: Could not find bar curve(s): {', '.join(missing)}.")
            continue

        # Compute only the one variant we need, then swap the blocks.
        # If recovery is needed (bad local minimum on this orientation),
        # flip the OPPOSITE side from whichever the user just clicked.
        recover_side = "male" if clicked_side == "female" else "female"
        le_start, le_end = curve_endpoints(le_id)
        ln_start, ln_end = curve_endpoints(ln_id)
        new_variant, _recovered, le_rev, ln_rev = compute_variant_with_recovery(
            le_start, le_end, ln_start, ln_end, le_rev, ln_rev,
            pair=pair, recover_side=recover_side,
            log_prefix="RSJointEdit",
        )
        n_removed = _remove_placed_joint(joint_id)
        if n_removed != 2:
            print(
                f"RSJointEdit: NOTE - removed {n_removed} block(s) for {joint_id}, "
                "expected 2 (one female + one male)."
            )
        # Preserve the previously-chosen tool for this joint, so flipping
        # the male/female block doesn't reset the tool back to the doc default.
        prev_tool_name = get_tool_name_for_joint(joint_id)
        _, male_id, new_joint_id = place_joint_blocks(
            new_variant, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
        )
        place_tool_by_name_at_male_joint(male_id, new_joint_id, pair, prev_tool_name)
        print(f"RSJointEdit: {joint_id} flipped (pair '{pair.name}').")


if __name__ == "__main__":
    main()
