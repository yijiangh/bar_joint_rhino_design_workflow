"""Designed MoCap marker positions of an assembly -- ``MoCapMarkers.json``.

RSExportRobotCell (and RSExportAllBarActions) write this file next to
``RobotCell.json``.  The cell carries every MoCap joint as a mesh but no pose,
and the marker spheres are in neither the cell nor the BarActions: they live
only in the joint registry, block-local (``JointHalfDef.marker_points_mm``,
recorded by RSDefineJointHalf).  This module puts the two together, so the
teleop monitor can compare live Motive markers with the design without
reading the registry or Rhino.

What the file says, per bar (every real bar, in assembly order):

* the designed centre line, ``start_mm`` -> ``end_mm`` (world, mm);
* each MoCap joint on the bar, ordered along the bar from its start;
* each marker sphere, numbered ``1..N`` per bar -- joints along the bar, and
  within a joint its labels in order (``M1``, ``M2``) -- with its name
  (``J20-21_mocap_M1``, :func:`core.joint_name_conventions.marker_name`), its
  world position and its position in the bar frame (origin at the bar start,
  z along the bar: ``joint_pair.canonical_bar_frame_from_line``, the frame the
  RobotCell bar bodies use).

A joint's markers, in order, are its designed marker-pair line.  Any number of
MoCap joints per bar works; a bar with none is listed with empty lists.

Pure numpy, no Rhino: the Rhino side (``core.rhino_mocap_markers``) passes in
the cell's collision bodies.
"""

from __future__ import annotations

import re

import numpy as np

from core import joint_name_conventions as jnc
from core.joint_pair import canonical_bar_frame_from_line
from core.marker_points import markers_in_frame_mm

FILE_NAME = "MoCapMarkers.json"
SCHEMA_VERSION = 1

_LABEL_RE = re.compile(r"^(\D*)(\d+)$")


def label_sort_key(label: str):
    """``M2`` before ``M10``: labels compared by prefix, then number."""
    match = _LABEL_RE.match(str(label))
    if match:
        return (match.group(1), int(match.group(2)), "")
    return (str(label), -1, str(label))


def _round_matrix(matrix) -> list:
    return [[round(float(v), 6) for v in row] for row in np.asarray(matrix, dtype=float)]


def _round_point(point) -> list:
    return [round(float(c), 2) for c in point]


def mocap_joints(bodies: dict) -> list:
    """``[(joint_id, body_info), ...]`` for every MoCap joint in *bodies*.

    *bodies* is the assembly cell's ``collision_bodies`` (``{key: body_info}``);
    the joint id is read back from the key (``joint_J20-21_mocap``).
    """
    out = []
    for key, info in bodies.items():
        if info.get("kind") != "joint":
            continue
        parts = jnc.split_joint_key(key)
        if parts is None or parts[1] != jnc.MOCAP:
            continue
        out.append((parts[0], info))
    return out


def designed_markers(
    bodies: dict,
    marker_points_by_block: dict,
    bar_lines: dict,
    assembly_order=None,
    *,
    source: str = "",
) -> dict:
    """The ``MoCapMarkers.json`` payload.

    Args:
        bodies: ``{key: body_info}`` of the assembly cell; a joint's
            ``body_info`` holds ``frame_world_mm`` (its block's world
            transform, mm), ``block_name`` and ``parent_bar_id``.
        marker_points_by_block: ``{block_name: {label: (x, y, z)}}``,
            block-local mm (the registry's ``marker_points_mm``).
        bar_lines: ``{bar_id: (start_mm, end_mm)}`` for every real bar.
        assembly_order: bar ids in assembly order; bars not in it follow,
            sorted by bar number.
        source: the Rhino document name, recorded in the file.
    """
    order = [b for b in (assembly_order or []) if b in bar_lines]
    order += sorted((b for b in bar_lines if b not in order), key=jnc.bar_sort_key)

    warnings = []
    joints_by_bar = {bar_id: [] for bar_id in order}
    for jid, info in mocap_joints(bodies):
        bar_id = info.get("parent_bar_id")
        if bar_id not in joints_by_bar:
            warnings.append(f"{jnc.object_name(jid, jnc.MOCAP)}: parent bar "
                            f"{bar_id or '<none>'} is not exported; skipped.")
            continue
        points = marker_points_by_block.get(info.get("block_name")) or {}
        if not points:
            warnings.append(f"{jnc.object_name(jid, jnc.MOCAP)}: block "
                            f"{info.get('block_name')!r} has no marker points "
                            "(record them with RSDefineJointHalf); skipped.")
            continue
        joints_by_bar[bar_id].append((jid, info, points))

    bars = []
    for seq, bar_id in enumerate(order, 1):
        start, end = (np.asarray(p, dtype=float) for p in bar_lines[bar_id])
        bar_frame = canonical_bar_frame_from_line(start, end)
        entries = []
        for jid, info, points in joints_by_bar[bar_id]:
            world = np.asarray(info["frame_world_mm"], dtype=float)
            position = float((world[:3, 3] - start) @ bar_frame[:3, 2])
            entries.append((position, jid, info, points, world))
        entries.sort(key=lambda e: (e[0], e[1]))

        joints, markers = [], []
        for position, jid, info, points, world in entries:
            labels = sorted(points, key=label_sort_key)
            in_world = markers_in_frame_mm(world, points, np.eye(4))
            in_bar = markers_in_frame_mm(world, points, bar_frame)
            indices = []
            for label in labels:
                index = len(markers) + 1
                indices.append(index)
                markers.append({
                    "index": index,
                    "name": jnc.marker_name(jid, jnc.MOCAP, label),
                    "joint_id": jid,
                    "label": label,
                    "world_mm": in_world[label],
                    "bar_mm": in_bar[label],
                })
            joints.append({
                "joint_id": jid,
                "name": jnc.object_name(jid, jnc.MOCAP),
                "paired": jnc.is_paired_half(jnc.MOCAP, jid),
                "block_name": info.get("block_name"),
                "position_mm": round(position, 2),
                "frame_world_mm": _round_matrix(world),
                "marker_indices": indices,
            })

        bars.append({
            "bar_id": bar_id,
            "assembly_seq": seq,
            "start_mm": _round_point(start),
            "end_mm": _round_point(end),
            "length_mm": round(float(np.linalg.norm(end - start)), 2),
            "joints": joints,
            "markers": markers,
        })

    return {
        "schema_version": SCHEMA_VERSION,
        "source": source,
        "units": "mm",
        "world": "rhino",
        "bar_frame": "origin at the bar start, z along the bar (as RobotCell bar bodies)",
        "marker_name": "<joint object name>_<label>, e.g. J20-21_mocap_M1",
        "bars": bars,
        "warnings": warnings,
    }


def marker_count(payload: dict) -> int:
    return sum(len(bar["markers"]) for bar in payload["bars"])


def summary(payload: dict) -> str:
    """``"B20: 8 markers on 4 joints; B21: 8 markers on 4 joints"``."""
    parts = [
        f"{bar['bar_id']}: {len(bar['markers'])} markers on {len(bar['joints'])} joints"
        for bar in payload["bars"]
        if bar["markers"]
    ]
    return "; ".join(parts)


__all__ = [
    "FILE_NAME",
    "SCHEMA_VERSION",
    "designed_markers",
    "label_sort_key",
    "marker_count",
    "mocap_joints",
    "summary",
]
