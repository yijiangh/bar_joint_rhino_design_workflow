"""Write ``MoCapMarkers.json`` beside ``RobotCell.json`` and label the markers.

Called by RSExportRobotCell and RSExportAllBarActions with the cell's
collision bodies (what ``robot_cell.ensure_assembly_cell`` returns, cut the
same way as the cell being written).  The payload itself is built by the pure
:mod:`core.mocap_markers`; this module only gathers its Rhino inputs -- the
registry's halves and the assembly order -- writes the file, and draws a
numbered text dot at every designed marker plus a line through each joint's
markers on ``config.LAYER_MOCAP_MARKER_LABELS``.  The dots are what you read
when naming the markers in Motive (``J20-21_mocap_M1``, ...).

Build first, write later: :func:`build_mocap_sidecar` runs the layout check
and raises before the caller has written anything, so a refused export leaves
the folder untouched.

Rhino-runtime only.
"""

# TODO(core-merge R3/R4): its callers (RSExportRobotCell, RSExportAllBarActions) are retired. RSExportProcess carries
#  the markers in process.json; the layout check and the label dots wait for a new caller.

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
    delete_objects,
    doc_unit_scale_to_mm,
    ensure_layer,
    objects_on_layers,
    suspend_redraw,
)


def _assembly_seq() -> dict:
    """``{bar_id: sequence number or None}`` from the bar registry."""
    return {bar_id: seq for bar_id, (_oid, seq) in get_bar_seq_map().items()}


def build_mocap_sidecar(bodies: dict, caller: str) -> dict:
    """The ``MoCapMarkers.json`` payload for *bodies*; nothing is written.

    Raises ``MoCapExportError`` / ``MoCapLayoutError`` (both ``RuntimeError``)
    with a message naming the joint or bar at fault; the caller shows it and
    stops before writing any file.
    """
    payload = mocap_markers.build_payload(
        bodies,
        load_joint_registry().halves,
        _assembly_seq(),
        source=sc.doc.Name or "",
        tol_m=float(config.MOCAP_LAYOUT_TOL_MM) / 1000.0,
    )
    for note in mocap_markers.layout_notes(payload):
        print(f"{caller}: MoCap NOTE - {note}")
    return payload


def _clear_labels() -> None:
    with suspend_redraw():
        delete_objects(objects_on_layers(config.LAYER_MOCAP_MARKER_LABELS))


def label_markers_in_rhino(payload: dict, bodies: dict) -> int:
    """Redraw the marker labels from *payload*; returns how many markers.

    One text dot per marker, ``B20 #3 M1`` (bar, number on the bar, label), at
    its designed position, with the full name (``J20-21_mocap_M1``) as the
    dot's object name; one line through each joint's markers (its pair line).
    Positions are the file's ``bar_m`` moved by the bar body's frame, so the
    dots show exactly what the file says.
    """
    layer = config.LAYER_MOCAP_MARKER_LABELS
    from_mm = 1.0 / doc_unit_scale_to_mm()
    world_mm = mocap_markers.world_points_mm(payload, bodies)
    count = 0
    with suspend_redraw():
        delete_objects(objects_on_layers(layer))
        ensure_layer(layer, color=config.MOCAP_PREVIEW_COLOR)
        for bar in payload["bars"].values():
            points_by_joint: dict = {}
            for number, marker in enumerate(bar["markers"], 1):
                doc_point = [c * from_mm for c in world_mm[marker["name"]]]
                points_by_joint.setdefault(marker["joint"], []).append(doc_point)
                dot = rs.AddTextDot(f"{bar['bar_id']} #{number} {marker['label']}", doc_point)
                if dot is None:
                    continue
                rs.ObjectLayer(dot, layer)
                rs.ObjectName(dot, marker["name"])
                rs.SetUserText(dot, jnc.UT_PARENT_BAR, bar["bar_id"])
                rs.SetUserText(dot, "mocap_marker_index", str(number))
                count += 1
            for joint_key, points in points_by_joint.items():
                if len(points) < 2:
                    continue
                line = rs.AddPolyline(points) if len(points) > 2 else rs.AddLine(*points)
                if line is not None:
                    rs.ObjectLayer(line, layer)
                    # The joint's object name: joint_J20-21_mocap -> J20-21_mocap.
                    rs.ObjectName(line, joint_key[len("joint_"):] if joint_key.startswith("joint_") else joint_key)
    return count


def write_mocap_sidecar(payload: dict, bodies: dict, root: str, caller: str):
    """Write *payload* to ``<root>/MoCapMarkers.json`` and label the markers.

    With no marker in the payload the file is NOT written, a stale one in the
    folder is deleted (the monitor must not read last week's markers), and the
    label layer is cleared.  Returns the written path, or ``None``.
    """
    path = os.path.join(root, mocap_markers.FILE_NAME)
    if not mocap_markers.marker_count(payload):
        _clear_labels()
        if os.path.exists(path):
            os.remove(path)
            print(f"{caller}: no MoCap markers placed; removed the stale {mocap_markers.FILE_NAME}.")
        else:
            print(f"{caller}: no MoCap markers placed; {mocap_markers.FILE_NAME} not written.")
        return None

    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    n_labels = label_markers_in_rhino(payload, bodies)
    print(
        f"{caller}: saved {path} ({mocap_markers.summary(payload)}); "
        f"{n_labels} markers labelled on layer '{config.LAYER_MOCAP_MARKER_LABELS}'."
    )
    return path
