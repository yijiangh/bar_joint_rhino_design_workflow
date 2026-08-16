#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSTempPlaceFemaleJoint - TEMPORARY: give every unmated male / ground joint a
female half plus a 100 mm stub bar to carry it.

Why it exists: a male half placed by RSJointPlace mates with a female that lives
on the *other* bar.  When that other bar does not exist -- a mock-up, a
hand-built staging rig, a mocap import -- the male has nothing to mate with.
This command manufactures the missing side.  For every male / ground joint whose
``joint_id`` no female carries, it

1. computes where the mating female block must sit.  No solver run is needed:
   the mate is fully determined by the male's actual pose in the document,
   ``female_block = male_screw_frame @ inv(female.M_screw_from_block)``;
2. creates a 100 mm bar centred on the bar-frame origin that female pose
   implies, along the bar direction it implies, and registers it (bar id + tube
   preview) exactly like an RSCreateBar bar;
3. inserts the female block on the female-instances layer under the **same**
   ``joint_id`` as its male, so every downstream pair check (RSUpdatePreview's
   mate report, RSJointEdit) sees one complete joint rather than two orphans.

**Ground joints.**  A ``GroundJointDef`` anchors a bar to the world and has no
``M_screw_from_block`` -- the frame a ground block presents to a mate is simply
not in ``joint_pairs.json``.  So for ground joints this command *borrows* the
male half of a joint pair you choose at the prompt: it assumes the ground
block's plug sits where that pair's male plug sits, relative to the block
origin.  Both blocks are authored on the same bar-frame convention, which is
what makes the assumption plausible -- but it is an assumption.  If the females
land wrong at ground joints and right at male joints, this is the line to
revisit (:func:`_mate_frame`).

**Temporary.**  Written for the mock-up workflow, not for the design pipeline.
The stub bars are ordinary registered bars, so they WILL appear in fabrication
export -- delete them when the mock-up is done, or mark them with
``RSBarEdit > FakeBar`` to keep them on screen but out of the export.
"""

import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.joint_pair import (
    canonical_bar_frame_from_line,
    fk_half_from_bar_frame,
    load_joint_registry,
)
from core.joint_pick_helpers import block_instance_frame
from core.joint_placement import (
    FEMALE_INSTANCES_LAYER,
    block_orientation_tag,
    insert_block_instance,
    write_joint_user_text,
)
from core.rhino_bar_pick import (
    require_pair_names,
    resolve_default_pair_index,
    set_default_pair_name,
)
from core.rhino_bar_registry import (
    SEQ_COLOR_FAKE,
    ensure_bar_id,
    ensure_bar_preview,
    paint_bar,
    repair_bar_sequences,
    repair_on_entry,
    set_fake_bar,
)
from core.rhino_block_import import require_block_definition
from core.rhino_helpers import suspend_redraw


# Command name used in every command-line message + dialog title.
CMD = "RSTempPlace"

#: Length of the stub bar created under each new female, in mm.  One number for
#: every case, so the command stays one click.
#:
#: 200 rather than 100 so a stub survives being flipped in RSJointEdit.  A flip
#: mirrors a half's mounting, which moves the mate about 60 mm along this bar --
#: up to 30 mm either side of centre (see docs/Su_note.md section 27).  At 100 mm
#: that leaves only ~20 mm of bar past the block; at 200 mm it leaves ~70 mm,
#: while still reading as a stub rather than a real member.
TEMP_BAR_LENGTH_MM = 200.0

#: How far the reconstructed female pose may drift from the placed one before
#: it is reported (mm / radians, on the raw 4x4).  Anything above this means the
#: (jp, jr) written to UserText does not describe the block that was inserted.
_FK_ROUNDTRIP_TOL = 1e-6


# ---------------------------------------------------------------------------
# Scanning
# ---------------------------------------------------------------------------


def _block_instances(layer):
    """Block instances on *layer*, or an empty list if the layer is absent."""
    if not rs.IsLayer(layer):
        return []
    return [oid for oid in rs.ObjectsByLayer(layer) or [] if rs.IsBlockInstance(oid)]


def _placed_female_joint_ids() -> set:
    """``joint_id`` of every female block already in the document."""
    ids = set()
    for oid in _block_instances(config.LAYER_JOINT_FEMALE_INSTANCES):
        joint_id = rs.GetUserText(oid, "joint_id")
        if joint_id:
            ids.add(joint_id)
    return ids


def _unmated_sources() -> list:
    """Male / ground joints that no female mates yet.

    Returns:
        list[tuple]: ``(oid, kind, joint_id, block_name)`` where *kind* is
        ``"male"`` or ``"ground"``.  Re-running the command after a successful
        pass therefore finds nothing to do.
    """
    have_female = _placed_female_joint_ids()
    out = []
    for kind, layer in (
        ("male", config.LAYER_JOINT_MALE_INSTANCES),
        ("ground", config.LAYER_JOINT_GROUND_INSTANCES),
    ):
        for oid in _block_instances(layer):
            joint_id = rs.GetUserText(oid, "joint_id") or ""
            if not joint_id:
                print(f"{CMD}: a {kind} block carries no joint_id; skipped.")
                continue
            if joint_id in have_female:
                continue
            out.append((oid, kind, joint_id, rs.BlockInstanceName(oid) or ""))
    return out


# ---------------------------------------------------------------------------
# Which joint pair does a source joint belong to?
# ---------------------------------------------------------------------------


def _pair_for_male(oid, block_name, mates):
    """Resolve the ``JointPairDef`` a placed male block belongs to, or ``None``.

    The ``joint_pair_name`` written at placement time is authoritative.  The
    fallback -- the block name appearing in exactly one mate -- exists for males
    that predate that key; a block used by several mates (``T20_Male`` is used
    by three) is genuinely ambiguous and is reported rather than guessed at.
    """
    name = rs.GetUserText(oid, "joint_pair_name") or ""
    if name in mates:
        return mates[name]
    candidates = [p for p in mates.values() if p.male.block_name == block_name]
    if len(candidates) == 1:
        return candidates[0]
    return None


def _ask_mode():
    """PlaceFemaleJointBar or PlaceBar.  Returns ``"joint"`` / ``"bar"`` / None.

    No Enter default -- neither mode is the obvious one, so you pick.  Same
    must-choose shape as ``rs_reorder_bar_id._ask_operation``.

    Both modes put the stub bar in exactly the same place: the bar position is
    derived from where the female WOULD go either way.  PlaceBar just stops short
    of inserting the block.
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("Place the female half and its stub bar, or the bar only")
    joint_idx = go.AddOption("PlaceFemaleJointBar")
    bar_idx = go.AddOption("PlaceBar")
    go.AcceptNothing(False)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == joint_idx:
                return "joint"
            if chosen == bar_idx:
                return "bar"
        else:
            return None


def _ask_ground_pair(mates):
    """Ask which pair's male plug the ground blocks should be treated as.

    Returns the ``JointPairDef``, or ``None`` if the user cancelled.  See the
    module docstring for why ground joints need this at all.
    """
    names = require_pair_names(CMD)
    if names is None:
        return None
    index = resolve_default_pair_index(names, None)

    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt("pair the orphan Male and Ground joints")
    go.AcceptNothing(True)
    list_idx = go.AddOptionList("Pair", names, index)
    res = go.Get()
    if res == Rhino.Input.GetResult.Option:
        opt = go.Option()
        if opt is not None and opt.Index == list_idx:
            index = int(opt.CurrentListOptionIndex)
    elif res != Rhino.Input.GetResult.Nothing:
        return None  # Esc
    set_default_pair_name(names[index])
    return mates[names[index]]


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def _mate_frame(source_oid, pair) -> np.ndarray:
    """World frame the female must mate against.

    ``block_world @ male.M_screw_from_block`` -- the same composition
    ``core.joint_pair.fk_half_from_bar_frame`` uses, but driven by the block's
    ACTUAL transform in the document, so the female lands on where the male
    really is rather than where a re-solve would put it.

    For a **ground** source, ``pair`` is the borrowed pair (module docstring):
    the ground block has no screw transform of its own, so the chosen pair's
    male one is applied to the ground block's world frame.
    """
    block_world, _block_name = block_instance_frame(source_oid)
    return np.asarray(block_world, dtype=float) @ np.asarray(
        pair.male.M_screw_from_block, dtype=float
    )


def _female_block_frame(mate_frame, pair) -> np.ndarray:
    """Invert the female's screw transform to get its block frame.

    At a solved mate the two screw frames coincide, so taking the male's screw
    frame as the female's and undoing ``M_screw_from_block`` is exact -- the
    roll about the shared Z is unobservable (the solver never pins it either),
    and here it is inherited from the male, which is as good a choice as any.
    """
    return mate_frame @ np.linalg.inv(
        np.asarray(pair.female.M_screw_from_block, dtype=float)
    )


def _bar_axis_frame(female_block_frame, pair) -> np.ndarray:
    """Undo ``M_block_from_bar`` -- i.e. return ``bar_frame @ T_z(jp) @ R_z(jr)``.

    Its origin is a point ON the bar axis and its +Z is the bar direction:
    ``T_z`` only slides along Z and ``R_z`` only spins about it, so neither can
    move the axis.  That is all the stub bar needs.
    """
    return female_block_frame @ np.linalg.inv(
        np.asarray(pair.female.M_block_from_bar, dtype=float)
    )


def _stub_endpoints(axis_frame, female_block_frame):
    """The stub bar's two endpoints: half its length each side of the female.

    Centred on the female BLOCK, not on ``axis_frame``'s origin.  Both lie on the
    bar axis, but ``M_block_from_bar`` separates them along it (20 mm for
    ``T20_Female``, 30 mm for the subfloor females) -- about the block's own
    half-width, so centring on the origin puts the bar's midpoint on the joint's
    face instead of through its middle.

    Centred rather than running one way: nothing in the model says which
    direction counts as "outward", so a centred stub avoids guessing.
    """
    origin = np.asarray(axis_frame[:3, 3], dtype=float)
    direction = np.asarray(axis_frame[:3, 2], dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm <= 0.0:
        raise ValueError("degenerate bar axis")
    direction = direction / norm
    # Where the block sits, measured along the bar, then put back on the axis --
    # so a half mounted slightly off-axis cannot drag the bar off it too.
    block_origin = np.asarray(female_block_frame[:3, 3], dtype=float)
    centre = origin + direction * float(np.dot(block_origin - origin, direction))
    half = 0.5 * TEMP_BAR_LENGTH_MM
    return centre - direction * half, centre + direction * half, direction


def _jp_jr(bar_start, bar_end, axis_frame):
    """Recover the female's ``(jp, jr)`` on the stub bar, for its UserText.

    ``inv(bar_frame) @ axis_frame`` is ``T_z(jp) @ R_z(jr)`` by construction, so
    ``jp`` is its Z translation and ``jr`` its rotation about Z.
    """
    bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
    delta = np.linalg.inv(bar_frame) @ axis_frame
    jp = float(delta[2, 3])
    jr = float(np.arctan2(delta[1, 0], delta[0, 0]))
    return bar_frame, jp, jr


# ---------------------------------------------------------------------------
# Placement
# ---------------------------------------------------------------------------


def _copy_variant_flags(source_oid):
    """Carry the male's variant bookkeeping onto the female it now mates with.

    Those three keys are what RSJointEdit re-opens a joint with; a female whose
    flags disagree with its male would re-solve to a different variant.  Missing
    or unparsable values fall back to variant 0, which is what a joint placed
    before the keys existed reads as anyway.
    """
    try:
        var_idx = int(rs.GetUserText(source_oid, "variant_index") or 0)
    except (TypeError, ValueError):
        var_idx = 0
    le_rev = (rs.GetUserText(source_oid, "le_rev") or "") == "True"
    ln_rev = (rs.GetUserText(source_oid, "ln_rev") or "") == "True"
    return le_rev, ln_rev, var_idx


def _place_one(source_oid, kind, joint_id, pair, bar_radius, place_female=True):
    """Create one stub bar for *source_oid*, and its female block unless told not to.

    The bar is identical either way: its length and position come from where the
    female WOULD sit, so PlaceBar and PlaceFemaleJointBar put the bar in exactly
    the same place.  With *place_female* False the block is simply not inserted --
    leaving a registered bar with nothing on it, which RSUpdatePreview will list
    as a bare bar.

    Returns ``(bar_id, female_oid_or_None)``, or ``None`` when the source's
    geometry is degenerate (reported, not raised -- one bad joint must not abort
    the pass).
    """
    female_frame = _female_block_frame(_mate_frame(source_oid, pair), pair)
    axis_frame = _bar_axis_frame(female_frame, pair)
    try:
        p0, p1, direction = _stub_endpoints(axis_frame, female_frame)
    except ValueError:
        print(f"{CMD}: {joint_id}: degenerate bar axis from the block frame; skipped.")
        return None

    curve_id = rs.AddLine(
        (float(p0[0]), float(p0[1]), float(p0[2])),
        (float(p1[0]), float(p1[1]), float(p1[2])),
    )
    if curve_id is None:
        print(f"{CMD}: {joint_id}: could not create the stub bar; skipped.")
        return None
    bar_id = ensure_bar_id(curve_id)
    ensure_bar_preview(curve_id, bar_radius, bar_id=bar_id)
    # Staging, not fabrication.  A fake bar still sequences and still carries
    # joints the IK solves against; it is only kept out of the prefab / bar-action
    # / robot-cell exports.  That is exactly what a mock-up stub is, so mark it
    # here rather than leaving it to be remembered by hand later.  Its female
    # inherits the exemption -- the exports filter by parent bar, so there is no
    # separate per-joint flag to set.
    set_fake_bar(curve_id, True)
    paint_bar(curve_id, SEQ_COLOR_FAKE)

    if not place_female:
        print(
            f"{CMD}: {joint_id} ({kind}) -> stub bar {bar_id} only "
            f"({TEMP_BAR_LENGTH_MM:.0f} mm, no female placed)."
        )
        return bar_id, None

    female_block_name = require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    female_oid = insert_block_instance(
        female_block_name, female_frame, layer_name=FEMALE_INSTANCES_LAYER
    )
    rs.ObjectName(female_oid, f"{joint_id}_female")

    bar_frame, jp, jr = _jp_jr(p0, p1, axis_frame)
    # Self-check: the (jp, jr) about to be written must reproduce the block that
    # was actually inserted, or the UserText describes a different joint.
    rebuilt = fk_half_from_bar_frame(bar_frame, jp, jr, pair.female)["block_frame"]
    err = float(np.max(np.abs(np.asarray(rebuilt) - female_frame)))
    if err > _FK_ROUNDTRIP_TOL:
        print(
            f"{CMD}: {joint_id}: WARNING - recovered (jp={jp:.3f}, "
            f"jr={np.degrees(jr):.3f}deg) rebuilds the female {err:.2e} off its "
            "placed pose; the block is correct, its UserText may not be."
        )

    female_type, _, female_subtype = female_block_name.partition("_")
    source_bar = rs.GetUserText(source_oid, "parent_bar_id") or ""
    le_rev, ln_rev, var_idx = _copy_variant_flags(source_oid)
    write_joint_user_text(
        female_oid,
        joint_id=joint_id,
        block_type=female_type,
        block_subtype=female_subtype,
        pair_name=pair.name,
        parent_bar=bar_id,
        connected_bar=source_bar,
        le_bar_id=bar_id,
        ln_bar_id=source_bar,
        pos_mm=jp,
        rot_rad=jr,
        ori=block_orientation_tag(female_frame[:3, 0], direction),
        le_rev=le_rev,
        ln_rev=ln_rev,
        var_idx=var_idx,
    )
    print(
        f"{CMD}: {joint_id} ({kind}) -> female on new {bar_id} "
        f"({TEMP_BAR_LENGTH_MM:.0f} mm stub, pair {pair.name})."
    )
    return bar_id, female_oid


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _run_place(place_female):
    """One pass over every unmated male / ground joint.

    Shared by both modes: the stub bar is identical either way, and
    *place_female* only decides whether the block goes on it.
    """
    mates = load_joint_registry().mates
    if not mates:
        rs.MessageBox(
            "No joint pairs are registered yet.  Run RSDefineJointMate first.",
            0,
            CMD,
        )
        return

    sources = _unmated_sources()
    if not sources:
        print(f"{CMD}: every male / ground joint already has a female.  Nothing to do.")
        return

    n_male = sum(1 for _oid, kind, _jid, _bn in sources if kind == "male")
    n_ground = len(sources) - n_male
    print(f"{CMD}: {n_male} male + {n_ground} ground joint(s) without a female.")

    ground_pair = None
    if n_ground:
        ground_pair = _ask_ground_pair(mates)
        if ground_pair is None:
            print(f"{CMD}: cancelled.")
            return
        print(
            f"{CMD}: ground joints will be mated as {ground_pair.name} "
            f"({ground_pair.male.block_name} plug) -- see the script docstring."
        )

    bar_radius = float(config.BAR_RADIUS)
    made = []
    with suspend_redraw():
        for source_oid, kind, joint_id, block_name in sources:
            pair = ground_pair if kind == "ground" else _pair_for_male(
                source_oid, block_name, mates
            )
            if pair is None:
                print(
                    f"{CMD}: {joint_id}: cannot tell which joint pair "
                    f"{block_name!r} belongs to - no joint_pair_name user text, "
                    "and the block is used by several mates; skipped."
                )
                continue
            if kind == "male" and pair.male.block_name != block_name:
                print(
                    f"{CMD}: {joint_id}: pair {pair.name!r} expects male block "
                    f"{pair.male.block_name!r} but the placed block is "
                    f"{block_name!r}; skipped."
                )
                continue
            placed = _place_one(source_oid, kind, joint_id, pair, bar_radius)
            if placed is not None:
                made.append(placed)

    changed = repair_bar_sequences()
    if changed:
        print(f"{CMD}: sequence repaired ({len(changed)} bar(s)).")
    print(
        f"{CMD}: placed {len(made)} female joint(s) on {len(made)} new "
        f"{TEMP_BAR_LENGTH_MM:.0f} mm stub bar(s)."
    )
    if made:
        print(
            f"{CMD}: these bars are REGISTERED and will be exported for "
            "fabrication - delete them, or mark them with RSBarEdit > FakeBar, "
            "when the mock-up is done."
        )
    rs.Redraw()


def main() -> None:
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), CMD)

    mode = _ask_mode()
    if mode is None:
        return
    if mode == "bar":
        _run_place(place_female=False)
        return
    _run_place(place_female=True)


if __name__ == "__main__":
    main()
