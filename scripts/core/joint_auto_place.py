"""Shared auto-placement of a joint pair between two existing bars.

Both ``rs_bar_snap`` and ``rs_bar_brace`` need the same default-orientation
joint placement after they've finalized a new bar's geometry: the existing
bar acts as the female (Le), the new bar as the male (Ln), and we always
pick the canonical variant ``(le_rev=False, ln_rev=False)``.  The user can
refine each joint later with ``RSJointEdit``.

This module lazy-imports ``rs_joint_place`` so the registry doesn't pull it
in at import time (and to keep this file Rhino-only at call time, not
import time).
"""

from core.rhino_bar_registry import ensure_bar_id
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
    import rs_joint_place as _rjp  # noqa: PLC0415  (lazy: needs Rhino runtime)

    le_bar_id = ensure_bar_id(le_curve_id)
    ln_bar_id = ensure_bar_id(ln_curve_id)
    le_start, le_end = curve_endpoints(le_curve_id)
    ln_start, ln_end = curve_endpoints(ln_curve_id)

    _rjp._require_block_definition(
        pair.female.block_name, asset_path=pair.female.asset_path()
    )
    _rjp._require_block_definition(
        pair.male.block_name, asset_path=pair.male.asset_path()
    )

    result = _rjp._compute_variant(
        le_start, le_end, ln_start, ln_end, False, False, pair=pair
    )
    _, male_id, joint_id = _rjp._place_joint_blocks(
        result, le_curve_id, ln_curve_id, le_bar_id, ln_bar_id, pair=pair
    )

    from core.rhino_tool_place import auto_place_tool_at_male_joint
    auto_place_tool_at_male_joint(male_id, joint_id, pair)
