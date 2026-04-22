#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""RSBarSnap - Snap a new bar onto an existing bar at contact distance.

Pick an existing bar (Le) and a new bar (Ln). The script translates Ln so that
the shortest distance between Le and Ln equals BAR_CONTACT_DISTANCE.
"""

import contextlib
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
from core.rhino_helpers import (
    apply_object_display,
    as_object_id_list,
    curve_endpoints,
    ensure_layer,
    point_to_array,
    suspend_redraw,
)
from core.rhino_bar_registry import (
    ensure_bar_id,
    ensure_bar_preview,
    pick_bar,
    repair_on_entry,
)


_REFERENCE_SEGMENT_PRINT_WIDTH = 0.8
_BAR_AXIS_LAYER = "Bar Axis Lines"
_CONTACT_SEGMENT_LAYER = "Contact Segments"
_TUBE_LAYER = "Tube preview"
_S1_EXISTING_BAR_COLOR = (120, 120, 120)
_DEFAULT_TUBE_COLOR = (205, 150, 60)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Generic helpers (point_to_array, curve_endpoints, as_object_id_list,
# ensure_layer, apply_object_display, suspend_redraw) are now imported
# from core.rhino_helpers.


def _place_axis_line(curve_id, *, color=None, label=None):
    if curve_id is None or not rs.IsObject(curve_id):
        return None
    ensure_layer(_BAR_AXIS_LAYER)
    rs.ObjectLayer(curve_id, _BAR_AXIS_LAYER)
    if color is not None:
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(curve_id, 1)
        rs.ObjectColor(curve_id, color)
    if label:
        rs.SetUserText(curve_id, "axis_label", label)
    return curve_id


def _bake_reference_segment(start_point, end_point, label, color):
    start_xyz = point_to_array(start_point)
    end_xyz = point_to_array(end_point)
    line_id = rs.AddLine(start_xyz.tolist(), end_xyz.tolist())
    if line_id is None:
        return None
    apply_object_display(line_id, label, color=color, layer_name=_CONTACT_SEGMENT_LAYER)
    if hasattr(rs, "ObjectPrintWidthSource"):
        rs.ObjectPrintWidthSource(line_id, 1)
    if hasattr(rs, "ObjectPrintWidth"):
        rs.ObjectPrintWidth(line_id, _REFERENCE_SEGMENT_PRINT_WIDTH)
    return line_id


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

    le_id = pick_bar("Select existing bar (Le)")
    if le_id is None:
        return
    ln_id = pick_bar("Select new bar (Ln) - will be repositioned")
    if ln_id is None:
        return

    le_start, le_end = curve_endpoints(le_id)
    ln_start, ln_end = curve_endpoints(ln_id)
    le_direction = le_end - le_start
    ln_direction = ln_end - ln_start
    target_distance = float(config.BAR_CONTACT_DISTANCE)

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
    shortest_segment_start = p_n + translation
    shortest_segment_end = p_e

    line_id = rs.MoveObject(ln_id, translation.tolist())
    if line_id is None:
        rs.MessageBox("Error: failed to move the selected new bar.")
        return

    _bake_reference_segment(
        shortest_segment_start,
        shortest_segment_end,
        "RSBarSnap_shortest_segment",
        (180, 0, 180),
    )
    _place_axis_line(line_id, label="RSBarSnap_Ln")

    # Register bars and create/update tube previews
    ensure_bar_id(le_id)
    ensure_bar_id(line_id)
    ensure_bar_preview(le_id, float(config.BAR_RADIUS), color=_S1_EXISTING_BAR_COLOR)
    ensure_bar_preview(line_id, float(config.BAR_RADIUS), color=_DEFAULT_TUBE_COLOR)
    rs.SelectObject(line_id)
    print(f"RSBarSnap: Bar placed at distance {target_distance:.2f} from Le")


if __name__ == "__main__":
    main()
