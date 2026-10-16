"""Write ``MoCapMarkers.json`` beside ``RobotCell.json`` and label the markers.

Called by RSExportRobotCell and RSExportAllBarActions right after they write
the cell, with the cell's collision bodies (what
``robot_cell.ensure_assembly_cell`` returns).  The payload itself is built by
the pure :mod:`core.mocap_markers`; this module only gathers its Rhino inputs
-- each bar's centre line, the assembly order, the registry's marker points --
writes the file, and draws a numbered text dot at every designed marker plus a
line through each joint's markers on ``config.LAYER_MOCAP_MARKER_LABELS``.
The dots are what you read when naming the markers in Motive
(``J20-21_mocap_M1``, ...).

Rhino-runtime only.
"""

from __future__ import annotations

import json
import os

import rhinoscriptsyntax as rs
import scriptcontext as sc

from core import config
from core import joint_name_conventions as jnc
from core import mocap_markers
from core.joint_pair import load_joint_registry
from core.rhino_bar_registry import get_bar_seq_map
from core.rhino_helpers import (
    curve_endpoints,
    delete_objects,
    doc_unit_scale_to_mm,
    ensure_layer,
    objects_on_layers,
    suspend_redraw,
)


def _bar_lines_mm(bodies: dict) -> dict:
    """``{bar_id: (start_mm, end_mm)}`` for every bar body of the cell."""
    scale = doc_unit_scale_to_mm()
    lines = {}
    for info in bodies.values():
        if info.get("kind") != "bar":
            continue
        oid = info.get("source_oid")
        if oid is None or not rs.IsObject(oid):
            continue
        start, end = curve_endpoints(oid)
        lines[info["parent_bar_id"]] = (start * scale, end * scale)
    return lines


def _assembly_order() -> list:
    seq_map = get_bar_seq_map()
    return [
        bar_id
        for bar_id, (_oid, seq) in sorted(
            seq_map.items(),
            key=lambda kv: (kv[1][1] is None, kv[1][1] or 0, jnc.bar_sort_key(kv[0])),
        )
    ]


def label_markers_in_rhino(payload: dict) -> int:
    """Redraw the marker labels from *payload*; returns how many markers.

    One text dot per marker, ``B20 #3 M1`` (bar, number on the bar, label),
    at its designed position, with the full name (``J20-21_mocap_M1``) as the
    dot's object name; one line through each joint's markers (its pair line).
    """
    layer = config.LAYER_MOCAP_MARKER_LABELS
    from_mm = 1.0 / doc_unit_scale_to_mm()
    count = 0
    with suspend_redraw():
        delete_objects(objects_on_layers(layer))
        ensure_layer(layer, color=config.MOCAP_PREVIEW_COLOR)
        for bar in payload["bars"]:
            doc_point = {m["index"]: [c * from_mm for c in m["world_mm"]] for m in bar["markers"]}
            for marker in bar["markers"]:
                dot = rs.AddTextDot(
                    f"{bar['bar_id']} #{marker['index']} {marker['label']}",
                    doc_point[marker["index"]],
                )
                if dot is None:
                    continue
                rs.ObjectLayer(dot, layer)
                rs.ObjectName(dot, marker["name"])
                rs.SetUserText(dot, jnc.UT_PARENT_BAR, bar["bar_id"])
                rs.SetUserText(dot, "mocap_marker_index", str(marker["index"]))
                count += 1
            for joint in bar["joints"]:
                points = [doc_point[i] for i in joint["marker_indices"]]
                if len(points) < 2:
                    continue
                line = rs.AddPolyline(points) if len(points) > 2 else rs.AddLine(*points)
                if line is not None:
                    rs.ObjectLayer(line, layer)
                    rs.ObjectName(line, joint["name"])
    return count


def export_mocap_markers(bodies: dict, root: str, caller: str):
    """Build ``MoCapMarkers.json`` from the cell's *bodies*, write it to *root*,
    and label the markers in Rhino.

    Returns the payload, or ``None`` when no MoCap marker is placed (the file is
    then not written, and the labels are cleared).
    """
    marker_points = {
        name: half.marker_points_mm
        for name, half in load_joint_registry().halves.items()
        if half.marker_points_mm
    }
    payload = mocap_markers.designed_markers(
        bodies, marker_points, _bar_lines_mm(bodies), _assembly_order(),
        source=sc.doc.Name or "",
    )
    for warning in payload["warnings"]:
        print(f"{caller}: MoCap NOTE - {warning}")

    path = os.path.join(root, mocap_markers.FILE_NAME)
    if not mocap_markers.marker_count(payload):
        with suspend_redraw():
            delete_objects(objects_on_layers(config.LAYER_MOCAP_MARKER_LABELS))
        stale = " An older one is still in the folder." if os.path.exists(path) else ""
        print(f"{caller}: no MoCap markers placed; {mocap_markers.FILE_NAME} not written.{stale}")
        return None

    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    n_labels = label_markers_in_rhino(payload)
    print(
        f"{caller}: saved {path} ({mocap_markers.summary(payload)}); "
        f"{n_labels} markers labelled on layer '{config.LAYER_MOCAP_MARKER_LABELS}'."
    )
    return payload
