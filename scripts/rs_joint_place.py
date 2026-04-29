#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSJointPlace - Place connector blocks on one bar pair.

The user is first prompted to confirm the active joint pair (Enter accepts
the last-used pair, or pick a different one from the registered list).
Then select any two registered bars.  The bar with the lower assembly
sequence number is automatically assigned as the female-joint bar (Le);
the bar assembled later receives the male joint (Ln).

The optimizer solves all 4 endpoint-reversal variants.  After an initial
placement you can refine interactively:

  - **Click the female block** to toggle the female joint orientation
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
from core import joint_pair as _joint_pair_module
from core import joint_pair_solver as _joint_pair_solver_module
from core import joint_placement as _joint_placement_module
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
from core.rhino_bar_pick import pick_bar, pick_bar_with_pair_option


# ---------------------------------------------------------------------------
# Module reload
# ---------------------------------------------------------------------------


def _reload_runtime_modules():
    """Reload shared core modules so edits take effect without a Rhino restart."""
    global config, joint_pair_module, joint_placement
    global compute_variant, interface_metrics, is_variant_acceptable
    global place_joint_blocks, insert_block_instance
    global FEMALE_INSTANCES_LAYER, MALE_INSTANCES_LAYER
    global PREVIEW_COLORS, JOINT_ROLE_KEY, ROLE_FEMALE, ROLE_MALE

    config = importlib.reload(_config_module)
    joint_pair_module = importlib.reload(_joint_pair_module)
    importlib.reload(_joint_pair_solver_module)
    joint_placement = importlib.reload(_joint_placement_module)

    compute_variant = joint_placement.compute_variant
    interface_metrics = joint_placement.interface_metrics
    is_variant_acceptable = joint_placement.is_variant_acceptable
    place_joint_blocks = joint_placement.place_joint_blocks
    insert_block_instance = joint_placement.insert_block_instance
    FEMALE_INSTANCES_LAYER = joint_placement.FEMALE_INSTANCES_LAYER
    MALE_INSTANCES_LAYER = joint_placement.MALE_INSTANCES_LAYER
    PREVIEW_COLORS = joint_placement.PREVIEW_COLORS
    JOINT_ROLE_KEY = joint_placement.JOINT_ROLE_KEY
    ROLE_FEMALE = joint_placement.ROLE_FEMALE
    ROLE_MALE = joint_placement.ROLE_MALE


_reload_runtime_modules()


# ---------------------------------------------------------------------------
# Interactive session
# ---------------------------------------------------------------------------


class _JointSession:
    """Tracks interactive state for one joint-placement run.

    The four solver variants map to a 2-bit state:

    - ``le_rev`` (bit 0): female joint orientation -- toggled by clicking
      the female block.
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
        female_block_name,
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
        self.female_block_name = female_block_name
        self.male_block_name = male_block_name
        self.pair = pair
        self.le_rev = le_rev
        self.ln_rev = ln_rev
        self._cache = {}  # (le_rev, ln_rev) -> variant dict
        self.female_id = None
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

    def role_of(self, oid):
        return rs.GetUserText(oid, JOINT_ROLE_KEY)

    def show_current(self):
        """Delete the existing preview, then insert the current variant."""
        existing = [i for i in (self.female_id, self.male_id) if i is not None]
        with suspend_redraw():
            delete_objects(existing)
            var = self.current_variant
            color = PREVIEW_COLORS[self.current_idx % len(PREVIEW_COLORS)]
            self.female_id = insert_block_instance(
                self.female_block_name,
                var["female_frame"],
                color=color,
                role=ROLE_FEMALE,
            )
            self.male_id = insert_block_instance(
                self.male_block_name,
                var["male_frame"],
                color=color,
                role=ROLE_MALE,
            )
        _print_variant_info(var)

    def cleanup(self):
        delete_objects([i for i in (self.female_id, self.male_id) if i is not None])
        self.female_id = None
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
        if recover_side == "female":
            self.le_rev = not self.le_rev
        else:
            self.ln_rev = not self.ln_rev

    def click_female(self):
        """Toggle female orientation; recover by flipping male if needed."""
        self.le_rev = not self.le_rev
        self._maybe_recover(recover_side="male", context="female-flip")
        self.show_current()

    def click_male(self):
        """Toggle male orientation; recover by flipping female if needed."""
        self.ln_rev = not self.ln_rev
        self._maybe_recover(recover_side="female", context="male-flip")
        self.show_current()


# ---------------------------------------------------------------------------
# Interactive cycling loop
# ---------------------------------------------------------------------------


def _joint_role_filter(rhino_object, geometry, component_index):
    """Geometry filter -- accept only the current interactive preview blocks."""
    return rs.GetUserText(rhino_object.Id, JOINT_ROLE_KEY) in (ROLE_FEMALE, ROLE_MALE)


def _interactive_click_loop(session: _JointSession):
    """Let the user click female/male preview blocks to toggle joint orientations.

    Returns the chosen variant dict, or ``None`` if cancelled.
    """
    session.show_current()
    try:
        while True:
            go = Rhino.Input.Custom.GetObject()
            go.SetCommandPrompt(
                "Click female block to flip female, click male block to flip male"
                "  (Enter = Accept)"
            )
            go.EnablePreSelect(False, False)
            go.AcceptNothing(True)
            go.SetCustomGeometryFilter(_joint_role_filter)
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
                role = session.role_of(clicked_id)
                if role == ROLE_FEMALE:
                    session.click_female()
                elif role == ROLE_MALE:
                    session.click_male()
    finally:
        session.cleanup()


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


def _assign_female_male_by_seq(bar_a_id, bar_a_bid, bar_b_id, bar_b_bid):
    """Auto-assign Le (female) / Ln (male) by assembly sequence number.

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


def main():
    _reload_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSJointPlace")

    rs.UnselectAllObjects()

    bar_a_id, pair = pick_bar_with_pair_option(
        "Select first bar of the joint pair", command_name="RSJointPlace"
    )
    if bar_a_id is None or pair is None:
        return
    print(
        f"RSJointPlace: using pair '{pair.name}' "
        f"(female='{pair.female.block_name}', male='{pair.male.block_name}', "
        f"contact={pair.contact_distance_mm:.4f} mm)"
    )

    bar_b_id = pick_bar("Select second bar of the joint pair")
    if bar_b_id is None:
        return

    bar_a_bid = ensure_bar_id(bar_a_id)
    bar_b_bid = ensure_bar_id(bar_b_id)

    le_id, le_bar_id, ln_id, ln_bar_id, le_seq, ln_seq = _assign_female_male_by_seq(
        bar_a_id, bar_a_bid, bar_b_id, bar_b_bid
    )
    print(
        f"RSJointPlace: {le_bar_id} (seq {le_seq}) \u2192 female,  "
        f"{ln_bar_id} (seq {ln_seq}) \u2192 male"
    )

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)

    # Validate block definitions before solving.
    female_block_name = require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    male_block_name = require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    session = _JointSession(
        le_start,
        le_end,
        ln_start,
        ln_end,
        female_block_name,
        male_block_name,
        pair=pair,
    )
    # Apply auto-recovery on the very first variant too, so special joints
    # (with only two valid variants out of the nominal four) don't open
    # with a visibly broken interface.
    session._maybe_recover(recover_side="female", context="initial")

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
