#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSJointPlace - Place connector blocks on one bar pair, or one joint on one bar.

Three modes, chosen at the first prompt:

**JointPairAndTool** (the default, and everything below): confirm the active
joint pair (Enter accepts the last-used pair, or pick a different one from the
registered list), then select any two registered bars.  When the pair's Type
also has a MoCap block (``T20_MoCap`` beside ``T20_Female``), a ``Receiver``
prompt chooses which one goes on the receiving bar; Enter keeps the Female.

The bar with the lower assembly sequence number is automatically assigned as the
receiving bar (Le); the bar assembled later receives the male joint (Ln).  Both
prompts say so, because the pick ORDER does not decide it.  That rule is not
cosmetic: ``rs_ik_keyframe._resolve_arm_tools_on_bar`` requires the bar being
assembled to carry exactly two male/ground halves (they are what hold the
grippers), and "earlier seq -> receiver" is what guarantees it.

**ToolOnly**: pick one male or ground joint block and put a tool on it, with
Flip to swap left/right.  No joint blocks are created and no variant is solved.
This is the repair path for a joint pair that survived but lost its tool -- the
normal flow would build a whole new pair, which is not what is wanted.

**JointOnly**: place ONE single-sided joint -- a Ground joint or a standalone
MoCap joint -- on one bar.  Pick the bar, pick a point on it, pick the joint
(``T20_Ground`` / ``T20_MoCap``; skipped when only one is registered), then:

  - Ground: ``Accept | Flip``.  Its angle about the bar is automatic -- the
    foot faces the floor.  Clicking the preview flips it end-for-end.
  - MoCap: ``Accept | Rotate``.  It starts with its marker plate facing up
    (world +Z), or +X on a bar within 15 deg of vertical.  Rotate turns it
    about the bar by the angle you type; clicking the preview repeats it.

No tool is placed.  A Ground joint carries one, so the command reminds you to
add it with ToolOnly (RSUpdatePreview also adds the default tool to any Ground
joint still without one); a MoCap joint never carries a tool.

The optimizer solves all 4 endpoint-reversal variants.  After an initial
placement you can refine interactively:

  - **Click the receiver block** to toggle the receiver orientation
    (switches between variant pairs 0<->1 and 2<->3).
  - **Click the male block** to toggle the male joint orientation
    (switches between variant groups 0<->2 and 1<->3).
  - Press **Enter** or click **Accept** to confirm and write the final blocks.
  - Press **Escape** to cancel and remove the preview.

Headless / shared placement logic (variant solver, interface metrics,
auto-recovery, block insertion + UserText writing) lives in
:mod:`core.joint_placement`.  This file owns the interactive session
and the ``main()`` entry point only.
"""

import importlib
import os
import sys

import numpy as np
import Rhino
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core import joint_name_conventions as jnc
from core import joint_pair as _joint_pair_module
from core import joint_pair_solver as _joint_pair_solver_module
from core import joint_placement as _joint_placement_module
from core import single_sided_placement as _single_sided_module
from core.rhino_helpers import (
    curve_endpoints,
    delete_objects,
    suspend_redraw,
)
from core.rhino_bar_registry import (
    ensure_bar_id,
    get_bar_seq_map,
    repair_on_entry,
)
from core.rhino_block_import import require_block_definition
from core.rhino_bar_pick import (
    pick_bar,
    pick_bar_with_pair_option,
    pick_point_on_bar,
)
from core import robotic_tool as _robotic_tool
from core.rhino_tool_place import (
    get_tool_name_for_joint,
    place_tool_at_block_instance,
)


# ---------------------------------------------------------------------------
# Module reload
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    """Reload shared core modules so edits take effect without a Rhino restart."""
    global config, joint_pair_module, joint_placement, single_sided
    global compute_variant, interface_metrics, is_variant_acceptable
    global place_joint_blocks, insert_block_instance
    global PREVIEW_COLORS

    config = importlib.reload(_config_module)
    joint_pair_module = importlib.reload(_joint_pair_module)
    importlib.reload(_joint_pair_solver_module)
    joint_placement = importlib.reload(_joint_placement_module)
    single_sided = importlib.reload(_single_sided_module)

    compute_variant = joint_placement.compute_variant
    interface_metrics = joint_placement.interface_metrics
    is_variant_acceptable = joint_placement.is_variant_acceptable
    place_joint_blocks = joint_placement.place_joint_blocks
    insert_block_instance = joint_placement.insert_block_instance
    PREVIEW_COLORS = joint_placement.PREVIEW_COLORS


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Interactive session
# ---------------------------------------------------------------------------


class _JointSession:
    """Tracks interactive state for one joint-placement run.

    The four solver variants map to a 2-bit state:

    - ``le_rev`` (bit 0): receiver (female or mocap) orientation -- toggled
      by clicking the receiver block.
    - ``ln_rev`` (bit 1): male joint orientation -- toggled by clicking
      the male block.

    variant_index = (2 if ln_rev else 0) + (1 if le_rev else 0)

    Variants are computed on demand (lazy) and cached so toggling back
    to a previously-seen orientation does not re-run the solver.
    """

    def __init__(
        self,
        le_start,
        le_end,
        ln_start,
        ln_end,
        receiver_block_name,
        male_block_name,
        *,
        pair,
        le_rev=False,
        ln_rev=False,
    ):
        self.le_start = le_start
        self.le_end = le_end
        self.ln_start = ln_start
        self.ln_end = ln_end
        self.receiver_block_name = receiver_block_name
        self.male_block_name = male_block_name
        self.pair = pair
        self.le_rev = le_rev
        self.ln_rev = ln_rev
        self._cache = {}  # (le_rev, ln_rev) -> variant dict
        self.receiver_id = None
        self.male_id = None

    # ----- variant access -----------------------------------------------

    def _get_variant(self, le_rev, ln_rev):
        key = (le_rev, ln_rev)
        if key not in self._cache:
            self._cache[key] = compute_variant(
                self.le_start,
                self.le_end,
                self.ln_start,
                self.ln_end,
                le_rev,
                ln_rev,
                pair=self.pair,
            )
        return self._cache[key]

    @property
    def current_idx(self):
        return (2 if self.ln_rev else 0) + (1 if self.le_rev else 0)

    @property
    def current_variant(self):
        return self._get_variant(self.le_rev, self.ln_rev)

    # ----- preview blocks -----------------------------------------------

    def subtype_of(self, oid):
        """The Subtype a preview block was tagged with, or ``None``."""
        return rs.GetUserText(oid, jnc.UT_PREVIEW_SUBTYPE)

    def show_current(self):
        """Delete the existing preview, then insert the current variant."""
        existing = [i for i in (self.receiver_id, self.male_id) if i is not None]
        with suspend_redraw():
            delete_objects(existing)
            var = self.current_variant
            color = PREVIEW_COLORS[self.current_idx % len(PREVIEW_COLORS)]
            self.receiver_id = insert_block_instance(
                self.receiver_block_name,
                var["female_frame"],
                color=color,
                # The half's real Subtype, so a MoCap preview is not tagged Female.
                subtype=self.pair.receiver_subtype,
            )
            self.male_id = insert_block_instance(
                self.male_block_name,
                var["male_frame"],
                color=color,
                subtype=jnc.MALE,
            )
        _print_variant_info(var)

    def cleanup(self):
        delete_objects([i for i in (self.receiver_id, self.male_id) if i is not None])
        self.receiver_id = None
        self.male_id = None

    # ----- toggles + auto-recovery --------------------------------------

    def _maybe_recover(self, recover_side: str, context: str) -> None:
        """If the current variant has too-large interface error, flip the
        opposite side once.  Mirrors :func:`core.joint_placement.compute_variant_with_recovery`.
        """
        if is_variant_acceptable(self.current_variant):
            return
        origin_err, z_err = interface_metrics(self.current_variant)
        print(
            f"RSJointPlace: {context} variant interface error too large "
            f"(origin={origin_err:.4f} mm, z={np.degrees(z_err):.4f}deg); "
            f"auto-flipping {recover_side} side."
        )
        if recover_side == "receiver":
            self.le_rev = not self.le_rev
        else:
            self.ln_rev = not self.ln_rev

    def click_receiver(self):
        """Toggle receiver orientation; recover by flipping male if needed."""
        self.le_rev = not self.le_rev
        self._maybe_recover(recover_side="male", context="receiver-flip")
        self.show_current()

    def click_male(self):
        """Toggle male orientation; recover by flipping receiver if needed."""
        self.ln_rev = not self.ln_rev
        self._maybe_recover(recover_side="receiver", context="male-flip")
        self.show_current()


# ---------------------------------------------------------------------------
# Interactive cycling loop
# ---------------------------------------------------------------------------


def _joint_preview_filter(rhino_object, geometry, component_index):
    """Geometry filter -- accept only the current interactive preview blocks."""
    subtype = rs.GetUserText(rhino_object.Id, jnc.UT_PREVIEW_SUBTYPE)
    return subtype in jnc.PAIRED_SUBTYPES


def _interactive_click_loop(session: _JointSession):
    """Let the user click receiver/male preview blocks to toggle joint orientations.

    Returns the chosen variant dict, or ``None`` if cancelled.
    """
    session.show_current()
    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                "Click receiver block to flip receiver, click male block to flip male"
                "  (Enter = Accept)"
            )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(_joint_preview_filter)
            go.AddOption("Accept")

            result = go.Get()

            if result == Rhino.Input.GetResult.Cancel:
                return None
            if result == Rhino.Input.GetResult.Nothing:
                return session.current_variant
            if result == Rhino.Input.GetResult.Option:
                if go.Option().EnglishName == "Accept":
                    return session.current_variant
            if result == Rhino.Input.GetResult.Object:
                clicked_id = go.Object(0).ObjectId
                subtype = session.subtype_of(clicked_id)
                if subtype in jnc.RECEIVER_SUBTYPES:
                    session.click_receiver()
                elif subtype == jnc.MALE:
                    session.click_male()
    finally:
        session.cleanup()


# ---------------------------------------------------------------------------
# ToolOnly mode -- put a tool back on a joint that already exists
# ---------------------------------------------------------------------------


def _tool_bearing_joint_filter(rhino_object, geometry, component_index):
    """Accept only male / ground joint blocks -- the halves that carry a tool.

    A tool attaches to the male (or ground) half and is tagged with that half's
    ``joint_id``; the receiver half plays no part in placing it.  Filtering the
    pick means a receiver half simply is not selectable, rather than being
    accepted and then rejected with an error.

    Matched by LAYER -- the authority for a placed block's Subtype.  The
    preview tag is written only on interactive preview blocks.
    """
    return rs.ObjectLayer(rhino_object.Id) in jnc.TOOL_BEARING_LAYERS


def _pick_tool_bearing_joint():
    """Pick one male/ground joint block.  Returns ``(block_id, joint_id)`` or None."""
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Select the male or ground joint block to put a tool on")
    go.EnablePreSelect(False, False)
    go.SetCustomGeometryFilter(_tool_bearing_joint_filter)
    if go.Get() != Rhino.Input.GetResult.Object:
        return None
    block_id = go.Object(0).ObjectId
    joint_id = rs.GetUserText(block_id, jnc.UT_JOINT_ID)
    if not joint_id:
        print(
            "RSJointPlace: that joint block has no 'joint_id' user text, so a tool "
            "cannot be tagged to it.  Re-create the joint pair."
        )
        return None
    return block_id, joint_id


def _run_tool_only():
    """Place (or replace) the tool on one existing joint -- no new joint blocks.

    For the case where a joint pair survives but its tool was deleted.  The
    normal flow would build a whole new pair, which is not what is wanted.

    ``place_tool_at_block_instance`` is idempotent -- it calls
    ``remove_tool_for_joint`` first -- so Flip and a re-run both replace rather
    than stack up duplicates.
    """
    picked = _pick_tool_bearing_joint()
    if picked is None:
        return
    block_id, joint_id = picked

    try:
        active = _robotic_tool.get_active_pair()
    except (RuntimeError, ValueError) as exc:
        print(f"RSJointPlace: no active tool pair, cannot place a tool: {exc}")
        return

    # Start on the side the joint already had, so re-tooling a joint whose tool
    # was deleted restores the bar's original L/R layout without a Flip.
    previous = get_tool_name_for_joint(joint_id)
    side = _robotic_tool.arm_side_from_tool_name(previous or "") or "left"

    while True:
        tool = active[side]
        if place_tool_at_block_instance(block_id, joint_id, tool) is None:
            return  # place_tool_at_block_instance already printed why
        print(f"RSJointPlace: {joint_id} -> {side} tool '{tool.name}'.")
        rs.Redraw()

        go = Rhino.Input.Custom.GetOption()
        go.SetCommandPrompt(
            f"Tool '{tool.name}' placed on {joint_id}  (Enter = Accept)"
        )
        accept_idx = go.AddOption("Accept")
        flip_idx = go.AddOption("Flip")
        go.SetCommandPromptDefault("Accept")
        go.AcceptNothing(True)

        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == accept_idx:
                return
            if chosen == flip_idx:
                side = "right" if side == "left" else "left"
                continue
        # Esc -- the tool stays as last placed; it is a valid tool either way,
        # and deleting it would leave the joint bare again.
        print("RSJointPlace: left the tool as last placed.")
        return


# ---------------------------------------------------------------------------
# JointOnly mode -- one Ground or standalone MoCap joint on one bar
# ---------------------------------------------------------------------------


#: Default step for MoCap's Rotate, in degrees.
MOCAP_ROTATE_STEP_DEG = 90.0


class _SingleSidedSession:
    """Live preview of one single-sided joint while it is being aimed.

    Ground can only be flipped (its angle is set by the floor); MoCap can only
    be rotated about the bar (it has no floor to face).
    """

    def __init__(self, *, definition, bar_id, bar_start, bar_end, jp, jr, flipped=False):
        self.definition = definition
        self.subtype = single_sided.subtype_of(definition)
        self.bar_id = bar_id
        self.bar_start = bar_start
        self.bar_end = bar_end
        self.jp = float(jp)
        self.jr = float(jr)
        self.flipped = bool(flipped)
        self.rotate_step_deg = MOCAP_ROTATE_STEP_DEG
        self.preview_id = None

    def _frame(self):
        return single_sided.fk_single_block_frame(
            self.bar_start, self.bar_end, self.jp, self.jr, self.definition,
            flipped=self.flipped,
        )

    def show(self):
        """(Re-)insert the preview at the current angle / flip."""
        with suspend_redraw():
            self.cleanup()
            self.preview_id = single_sided.insert_single_sided_preview(
                self.definition, self._frame()
            )
        print(
            f"RSJointPlace: {self.definition.block_name} jp={self.jp:.2f} mm, "
            f"jr={np.degrees(self.jr):.1f} deg"
            + (f", flipped={self.flipped}" if self.subtype == jnc.GROUND else "")
        )

    def flip(self):
        self.flipped = not self.flipped
        self.show()

    def rotate(self, degrees):
        """Turn about the bar by *degrees*; jr is kept in (-180, 180]."""
        angle = self.jr + np.radians(float(degrees))
        self.jr = float(np.arctan2(np.sin(angle), np.cos(angle)))
        self.show()

    def cleanup(self):
        if self.preview_id is not None:
            delete_objects([self.preview_id])
            self.preview_id = None


def _single_sided_preview_loop(session):
    """``Accept | Flip`` (Ground) or ``Accept | Rotate`` (MoCap).  True on accept.

    Clicking the preview block repeats the one adjustment the joint has.
    Also used by RSJointEdit to re-aim a placed joint.
    """
    is_ground = session.subtype == jnc.GROUND

    def _preview_filter(rhino_object, geometry, component_index):
        return rs.GetUserText(rhino_object.Id, jnc.UT_PREVIEW_SUBTYPE) == session.subtype

    session.show()
    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            if is_ground:
                go.SetCommandPrompt(
                    "Click the joint to flip it along the bar (Enter = Accept, Esc = Cancel)"
                )
            else:
                go.SetCommandPrompt(
                    f"Click the joint to rotate it {session.rotate_step_deg:g} deg about "
                    "the bar (Enter = Accept, Esc = Cancel)"
                )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(_preview_filter)
            go.AddOption("Accept")
            go.AddOption("Flip" if is_ground else "Rotate")

            result = go.Get()
            if result == Rhino.Input.GetResult.Cancel:
                return False
            if result == Rhino.Input.GetResult.Nothing:
                return True
            if result == Rhino.Input.GetResult.Option:
                name = go.Option().EnglishName if go.Option() is not None else ""
                if name == "Accept":
                    return True
                if name == "Flip":
                    session.flip()
                elif name == "Rotate":
                    step = rs.GetReal(
                        "Rotate about the bar (degrees)", session.rotate_step_deg
                    )
                    if step is not None:
                        session.rotate_step_deg = float(step)
                        session.rotate(step)
                continue
            if result == Rhino.Input.GetResult.Object:
                if is_ground:
                    session.flip()
                else:
                    session.rotate(session.rotate_step_deg)
    finally:
        session.cleanup()


def _pick_single_sided_definition():
    """The Ground / MoCap definition to place, or ``None``."""
    registry = joint_pair_module.load_joint_registry()
    definitions = single_sided.single_sided_definitions(registry)
    if not definitions:
        rs.MessageBox(
            "No Ground or MoCap joint is registered.\n"
            "Define one with RSDefineJointHalf (block named <Type>_Ground or "
            "<Type>_MoCap).",
            0,
            "RSJointPlace",
        )
        return None
    if len(definitions) == 1:
        return definitions[0]
    names = [d.block_name for d in definitions]
    chosen = rs.ListBox(names, "Joint to place on the bar", "RSJointPlace")
    if not chosen:
        return None
    return definitions[names.index(chosen)]


def _run_joint_only():
    """Place one Ground or standalone MoCap joint on one bar.  No tool."""
    bar_curve = pick_bar("Select the bar for the Ground / MoCap joint")
    if bar_curve is None:
        return
    bar_id = ensure_bar_id(bar_curve)
    bar_start, bar_end = curve_endpoints(bar_curve)
    bar_start = np.asarray(bar_start, dtype=float)
    bar_end = np.asarray(bar_end, dtype=float)
    bar_vec = bar_end - bar_start
    bar_len = float(np.linalg.norm(bar_vec))
    if bar_len <= 0.0:
        print(f"RSJointPlace: {bar_id} has zero length; cannot place a joint on it.")
        return

    point = pick_point_on_bar(bar_curve, "Pick the point on the bar for the joint")
    if point is None:
        return
    jp = float(np.dot(point - bar_start, bar_vec / bar_len))

    definition = _pick_single_sided_definition()
    if definition is None:
        return
    try:
        require_block_definition(
            definition.block_name, asset_path=definition.asset_path()
        )
    except RuntimeError as exc:
        rs.MessageBox(str(exc), 0, "RSJointPlace")
        return

    session = _SingleSidedSession(
        definition=definition,
        bar_id=bar_id,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=jp,
        jr=single_sided.default_jr(bar_start, bar_end, definition),
    )
    if not _single_sided_preview_loop(session):
        print("RSJointPlace: Cancelled.")
        return

    _oid, joint_id = single_sided.place_single_sided_block(
        definition=definition,
        bar_id=bar_id,
        bar_start=bar_start,
        bar_end=bar_end,
        jp=session.jp,
        jr=session.jr,
        flipped=session.flipped,
    )
    if session.subtype == jnc.GROUND:
        print(
            f"RSJointPlace: {joint_id} has no tool yet -- run RSJointPlace > "
            "ToolOnly to place it (and choose its side), or the next "
            "RSUpdatePreview adds the default tool."
        )


def _print_variant_info(variant):
    origin_err, z_err = interface_metrics(variant)
    print(
        f"RSJointPlace: Showing variant {variant['variant_index'] + 1}/4 - "
        f"{variant.get('variant_label', '?')} | "
        f"residual={variant['residual']:.6f}, "
        f"origin err={origin_err:.4f} mm, "
        f"z err={np.degrees(z_err):.4f}deg"
    )


# ---------------------------------------------------------------------------
# Bar pair role assignment
# ---------------------------------------------------------------------------


def _assign_receiver_male_by_seq(bar_a_id, bar_a_bid, bar_b_id, bar_b_bid):
    """Auto-assign Le (receiver) / Ln (male) by assembly sequence number.

    Returns ``(le_id, le_bar_id, ln_id, ln_bar_id, le_seq, ln_seq)``.
    """
    seq_map = get_bar_seq_map()
    seq_a = seq_map.get(bar_a_bid, (None, 9999))[1]
    seq_b = seq_map.get(bar_b_bid, (None, 9999))[1]
    if seq_a <= seq_b:
        le_id, le_bar_id = bar_a_id, bar_a_bid
        ln_id, ln_bar_id = bar_b_id, bar_b_bid
    else:
        le_id, le_bar_id = bar_b_id, bar_b_bid
        ln_id, ln_bar_id = bar_a_id, bar_a_bid
    return (
        le_id,
        le_bar_id,
        ln_id,
        ln_bar_id,
        seq_map.get(le_bar_id, (None, "?"))[1],
        seq_map.get(ln_bar_id, (None, "?"))[1],
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _ask_place_mode():
    """JointPairAndTool / ToolOnly / JointOnly.

    Returns ``"pair"`` / ``"tool"`` / ``"single"``, or ``None`` on Esc.  Enter
    keeps the historical behaviour (a joint pair with its tool).
    """
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(
        "Place a joint pair with its tool, put a tool on an existing joint, or "
        "place one Ground / MoCap joint on a bar"
    )
    pair_idx = go.AddOption("JointPairAndTool")
    tool_idx = go.AddOption("ToolOnly")
    single_idx = go.AddOption("JointOnly")
    go.SetCommandPromptDefault("JointPairAndTool")
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return "pair"
        if result == Rhino.Input.GetResult.Option:
            chosen = go.OptionIndex()
            if chosen == pair_idx:
                return "pair"
            if chosen == tool_idx:
                return "tool"
            if chosen == single_idx:
                return "single"
            continue
        return None


def _ask_receiver(pair):
    """Female or MoCap on the receiving bar, when the pair's Type has both.

    Returns the pair to place -- unchanged when there is no choice -- or
    ``None`` on Esc.  Enter keeps the Female, so the usual placement costs one
    keystroke.  Asked every run rather than remembered, so a MoCap joint is
    never placed by accident.
    """
    registry = joint_pair_module.load_joint_registry()
    choices = joint_pair_module.receiver_subtypes(pair, registry.halves)
    if len(choices) < 2:
        return pair
    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(f"Receiving joint for '{pair.name}'")
    indices = {go.AddOption(subtype): subtype for subtype in choices}
    go.SetCommandPromptDefault(choices[0])
    go.AcceptNothing(True)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return pair
        if result == Rhino.Input.GetResult.Option and go.OptionIndex() in indices:
            subtype = indices[go.OptionIndex()]
            return joint_pair_module.with_receiver(pair, subtype, registry.halves)
        if result != Rhino.Input.GetResult.Option:
            return None


# Both bar prompts say the whole rule, because which bar becomes the receiver is
# decided by assembly sequence, NOT by the order you click -- and that is
# invisible otherwise.  It is not a free choice: RSIKKeyframe requires the bar
# being assembled to carry both male halves (they hold the grippers), which is
# exactly what "earlier seq -> receiver" guarantees.
_SEQ_RULE_NOTE = (
    "(the earlier seq becomes the RECEIVING joint / the later seq becomes MALE joint)"
)


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSJointPlace")

    rs.UnselectAllObjects()

    mode = _ask_place_mode()
    if mode is None:
        return
    if mode == "tool":
        _run_tool_only()
        return
    if mode == "single":
        _run_joint_only()
        return

    bar_a_id, pair = pick_bar_with_pair_option(
        f"Select first bar of the joint pair {_SEQ_RULE_NOTE}",
        command_name="RSJointPlace",
    )
    if bar_a_id is None or pair is None:
        return
    pair = _ask_receiver(pair)
    if pair is None:
        return
    print(
        f"RSJointPlace: using pair '{pair.name}' "
        f"({pair.receiver_subtype}='{pair.receiver.block_name}', "
        f"male='{pair.male.block_name}', "
        f"contact={pair.contact_distance_mm:.4f} mm)"
    )

    bar_b_id = pick_bar(f"Select second bar of the joint pair {_SEQ_RULE_NOTE}")
    if bar_b_id is None:
        return

    bar_a_bid = ensure_bar_id(bar_a_id)
    bar_b_bid = ensure_bar_id(bar_b_id)

    le_id, le_bar_id, ln_id, ln_bar_id, le_seq, ln_seq = _assign_receiver_male_by_seq(
        bar_a_id, bar_a_bid, bar_b_id, bar_b_bid
    )
    print(
        f"RSJointPlace: {le_bar_id} (seq {le_seq}) \u2192 {pair.receiver_subtype},  "
        f"{ln_bar_id} (seq {ln_seq}) \u2192 male"
    )

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)

    # Validate block definitions before solving.
    receiver_block_name = require_block_definition(
        pair.receiver.block_name, asset_path=pair.receiver.asset_path()
    )
    male_block_name = require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    session = _JointSession(
        le_start,
        le_end,
        ln_start,
        ln_end,
        receiver_block_name,
        male_block_name,
        pair=pair,
    )
    # Apply auto-recovery on the very first variant too, so special joints
    # (with only two valid variants out of the nominal four) don't open
    # with a visibly broken interface.
    session._maybe_recover(recover_side="receiver", context="initial")

    chosen = _interactive_click_loop(session)
    if chosen is None:
        print("RSJointPlace: Cancelled.")
        return

    _, male_id, joint_id = place_joint_blocks(
        chosen, le_id, ln_id, le_bar_id, ln_bar_id, pair=pair
    )
    from core.rhino_tool_place import auto_place_tool_at_male_joint
    auto_place_tool_at_male_joint(male_id, joint_id, pair)


if __name__ == "__main__":
    main()
