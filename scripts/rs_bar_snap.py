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
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import geometry
from core.rhino_helpers import curve_endpoints
from core.rhino_bar_registry import (
    ensure_bar_id,
    ensure_bar_preview,
    paint_bar,
    repair_on_entry,
    reset_bar_color,
)
from core.rhino_bar_pick import pick_bar, pick_bar_with_pair_option
from core.joint_auto_place import auto_place_joint_pair


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _refresh_runtime_modules():
    importlib.reload(config)
    importlib.reload(geometry)


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

    ln_id = None
    line_id = None
    try:
        # Visual feedback: highlight the existing bar (Le) the user just picked.
        ensure_bar_id(le_id)
        ensure_bar_preview(le_id, float(config.BAR_RADIUS))
        paint_bar(le_id, config.SELECTED_BAR_COLOR)

        ln_id = pick_bar("Select new bar (Ln) - will be repositioned")
        if ln_id is None:
            return
        ensure_bar_id(ln_id)
        ensure_bar_preview(ln_id, float(config.BAR_RADIUS))
        paint_bar(ln_id, config.SELECTED_BAR_COLOR)

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

        translation = (segment / current_distance) * (
            current_distance - target_distance
        )

        line_id = rs.MoveObject(ln_id, translation.tolist())
        if line_id is None:
            rs.MessageBox("Error: failed to move the selected new bar.")
            return

        # Refresh tube preview at the moved location and re-apply highlight.
        ensure_bar_id(line_id)
        ensure_bar_preview(line_id, float(config.BAR_RADIUS))
        paint_bar(line_id, config.SELECTED_BAR_COLOR)
        print(
            f"RSBarSnap: Bar placed at distance {target_distance:.2f} mm from Le "
            f"(pair '{pair.name}')"
        )

        # Auto-place a joint between Le (female) and the new bar (male).
        auto_place_joint_pair(le_id, line_id, pair)

    finally:
        reset_bar_color(le_id)
        if ln_id is not None and ln_id != line_id:
            reset_bar_color(ln_id)
        reset_bar_color(line_id)
        sc.doc.Views.Redraw()


if __name__ == "__main__":
    main()
