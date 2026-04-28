#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarSnap - Snap a new bar onto an existing bar at a joint pair's contact distance.

The user is first prompted to confirm the active joint pair (Enter accepts
the last-used pair).  Then they pick an existing bar (Le) and a new bar (Ln);
the script translates Ln so that the shortest distance between Le and Ln
equals the chosen pair's ``contact_distance_mm``.
"""

import importlib
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import geometry
from core.rhino_helpers import curve_endpoints
from core.rhino_bar_registry import (
    ensure_bar_id,
    ensure_bar_preview,
    pick_bar,
    repair_on_entry,
)
from core.rhino_pair_selector import pick_bar_with_pair_option


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


def _auto_place_joints(le_curve_id, ln_curve_id, pair):
    """Place a default-orientation joint pair between an existing bar (female)
    and the newly snapped bar (male).  Reuses ``rs_joint_place`` internals
    and always picks ``variant_index=0`` (le_rev=False, ln_rev=False); the
    user can refine later with RSJointEdit.
    """
    import rs_joint_place as _rjp

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
    _rjp._place_joint_blocks(
        result, le_curve_id, ln_curve_id, le_bar_id, ln_bar_id, pair=pair
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    _refresh_runtime_modules()
    repair_on_entry(float(config.BAR_RADIUS), "RSBarSnap")
    rs.UnselectAllObjects()

    le_id, pair = pick_bar_with_pair_option(
        "Select existing bar (Le)", command_name="RSBarSnap"
    )
    if le_id is None or pair is None:
        return
    print(
        f"RSBarSnap: using pair '{pair.name}' "
        f"(contact distance {pair.contact_distance_mm:.4f} mm)"
    )

    ln_id = pick_bar("Select new bar (Ln) - will be repositioned")
    if ln_id is None:
        return

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)
    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start
    target_distance = float(pair.contact_distance_mm)

    t_e, t_n = geometry.closest_params_infinite_lines(
        le_start,
        le_direction,
        ln_start,
        ln_direction,
    )
    p_e = le_start + t_e * le_direction
    p_n = ln_start + t_n * ln_direction

    segment = p_e - p_n
    current_distance = np.linalg.norm(segment)
    if current_distance <= 1e-9:
        rs.MessageBox(
            "Error: the selected bars are coincident, so no unique contact normal exists."
        )
        return

    translation = (segment / current_distance) * (current_distance - target_distance)

    line_id = rs.MoveObject(ln_id, translation.tolist())
    if line_id is None:
        rs.MessageBox("Error: failed to move the selected new bar.")
        return

    # Register bars and create/update tube previews
    ensure_bar_id(le_id)
    ensure_bar_id(line_id)
    ensure_bar_preview(le_id, float(config.BAR_RADIUS))
    ensure_bar_preview(line_id, float(config.BAR_RADIUS))
    print(
        f"RSBarSnap: Bar placed at distance {target_distance:.2f} mm from Le "
        f"(pair '{pair.name}')"
    )

    # Auto-place a joint between Le (female) and the new bar (male).  Uses a
    # consistent default variant; refine later with RSJointEdit if needed.
    _auto_place_joints(le_id, line_id, pair)


if __name__ == "__main__":
    main()
