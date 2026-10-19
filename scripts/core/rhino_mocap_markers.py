"""Label the designed MoCap markers of an exported process in Rhino.

RSExportProcess calls :func:`label_markers_in_rhino` after a successful write.  It
redraws the layer ``config.LAYER_MOCAP_MARKER_LABELS``: a text dot at every
designed marker, ``B20 #3 M1`` (bar, the marker's number on the bar, label),
named after the monitor's marker (``joints/J20-21_mocap/M1``), and a line
through each half's markers.  Positions, names and numbers come from the pure
:mod:`core.process_markers`, so the dots show exactly what the monitor fits.

Rhino-runtime only.
"""

from __future__ import annotations

import rhinoscriptsyntax as rs

from core import config
from core import joint_name_conventions as jnc
from core.process_markers import marker_labels
from core.rhino_helpers import (
    delete_objects,
    doc_unit_scale_to_mm,
    ensure_layer,
    objects_on_layers,
    suspend_redraw,
)


def label_markers_in_rhino(process) -> int:
    """Redraw the marker labels of *process*; returns how many markers were labelled.

    With no marker in the process the layer is only cleared.

    Args:
        process: The process as written (``scaffolding_assembly_core`` ``Process``).
    """
    layer = config.LAYER_MOCAP_MARKER_LABELS
    from_m = 1000.0 / doc_unit_scale_to_mm()
    labels = marker_labels(process)
    count = 0
    with suspend_redraw():
        delete_objects(objects_on_layers(layer))
        if not labels:
            return 0
        ensure_layer(layer, color=config.MOCAP_PREVIEW_COLOR)
        points_by_half: dict = {}
        for label in labels:
            doc_point = [float(c) * from_m for c in label.world]
            # The half's id is the name without its label: joints/J20-21_mocap/M1 -> joints/J20-21_mocap.
            points_by_half.setdefault(label.name.rpartition("/")[0], []).append(doc_point)
            dot = rs.AddTextDot(label.text, doc_point)
            if dot is None:
                continue
            rs.ObjectLayer(dot, layer)
            rs.ObjectName(dot, label.name)
            rs.SetUserText(dot, jnc.UT_PARENT_BAR, label.bar.rpartition("/")[2])
            rs.SetUserText(dot, "mocap_marker_index", str(label.number))
            count += 1
        for half_id, points in points_by_half.items():
            if len(points) < 2:
                continue
            line = rs.AddPolyline(points) if len(points) > 2 else rs.AddLine(*points)
            if line is not None:
                rs.ObjectLayer(line, layer)
                rs.ObjectName(line, half_id)
    return count
