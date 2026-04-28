"""Shared auto-placement of a joint pair between two existing bars.

``rs_bar_snap``, ``rs_bar_brace``, and ``rs_bar_subfloor`` all need the
same default-orientation joint placement after they've finalized a new
bar's geometry: the existing bar acts as the female (Le), the new bar
as the male (Ln), and we always pick the canonical variant
``(le_rev=False, ln_rev=False)``.  The user can refine each joint later
with ``RSJointEdit``.
"""

from core.joint_placement import (
    compute_variant_with_recovery,
    place_joint_blocks,
)
from core.rhino_bar_registry import ensure_bar_id
from core.rhino_block_import import require_block_definition
from core.rhino_helpers import curve_endpoints


def auto_place_joint_pair(le_curve_id, ln_curve_id, pair):
    """Place the canonical (variant 0) joint pair between two bars.

    Parameters
    ----------
    le_curve_id, ln_curve_id : Rhino object ids
        Centerline curves of the existing (female) and new (male) bars.
    pair : core.joint_pair.JointPairDef
        The active joint-pair definition.
    """
    le_bar_id = ensure_bar_id(le_curve_id)
    ln_bar_id = ensure_bar_id(ln_curve_id)
    le_start, le_end = curve_endpoints(le_curve_id)
    ln_start, ln_end = curve_endpoints(ln_curve_id)

    require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    # Auto-place: start at the canonical (le_rev=False, ln_rev=False)
    # variant.  If its interface error is too large (special joints with
    # only two valid variants), automatically flip the female side once.
    result, _recovered, _le_rev, _ln_rev = compute_variant_with_recovery(
        le_start, le_end, ln_start, ln_end, False, False,
        pair=pair, recover_side="female",
        log_prefix="auto_place_joint_pair",
    )
    _, male_id, joint_id = place_joint_blocks(
        result, le_curve_id, ln_curve_id, le_bar_id, ln_bar_id, pair=pair
    )

    from core.rhino_tool_place import auto_place_tool_at_male_joint
    auto_place_tool_at_male_joint(male_id, joint_id, pair)
