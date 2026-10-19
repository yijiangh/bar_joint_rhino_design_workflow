"""Designed MoCap marker positions of an assembly -- ``MoCapMarkers.json`` (schema 2).

RSExportRobotCell and RSExportAllBarActions write this file next to
``RobotCell.json``.  It carries ONLY what neither the cell nor the BarActions
have: where each marker sphere sits on its bar.

* ``RobotCell.json`` has every bar and joint as a collision body, no poses.
* The BarAction states have every body's pose per movement (metres, world).
* The marker sphere centres exist only in the joint registry, block-local
  (``JointHalfDef.marker_points_mm``, recorded by RSDefineJointHalf).

So, per bar, the file lists each marker in the BAR BODY'S OWN FRAME, in metres:
the frame the bar body uses in the cell and in every BarAction state
(``RigidBodyState.frame``; origin at the bar start, z along the bar).  The
teleop monitor fits the measured Motive markers onto these points, and the
resulting transform IS the measured ``T_world_from_bar``, directly comparable
to the designed state frame.  No world positions, bar centre lines or joint
transforms are repeated here.

Everything comes from the cached cell snapshot (the ``bodies`` of
``robot_cell.ensure_assembly_cell``), the SAME source as ``RobotCell.json``, so
the two files cannot disagree; a range export passes the same cut body set.

Before anything is written, every bar with markers must pass
``core.mocap_layout_check``: a layout that could sit in two poses that look the
same to Motive is refused with a message naming the bar (no silent fallback).
Likewise a MoCap block with no recorded markers or diameter, or a MoCap joint
whose bar is not in the export, stops the export instead of being skipped.

Pure numpy, no Rhino: ``core.rhino_mocap_markers`` gathers the Rhino inputs.
"""

from __future__ import annotations

import re

import numpy as np

from core import joint_name_conventions as jnc
from core.marker_points import markers_in_frame_mm
from core.mocap_layout_check import DEFAULT_TOL_M, require_unambiguous
from core.transforms import local_transform, transform_point

FILE_NAME = "MoCapMarkers.json"
SCHEMA_VERSION = 2

#: Shown with the refusal when a bar's layout is ambiguous.
LAYOUT_HINT = (
    "Rule of thumb: at least three MoCap joints per tracked bar, not all facing the same "
    "way, not mirror-symmetric about the bar's middle. Add a MoCap joint turned about the "
    "bar (RSJointPlace > JointOnly > MoCap, Rotate), or move one, rebuild the cell, export again."
)

_LABEL_RE = re.compile(r"^(\D*)(\d+)$")


class MoCapExportError(RuntimeError):
    """The MoCap marker file cannot be built from this cell; nothing is written."""


def label_sort_key(label: str):
    """``M2`` before ``M10``: labels compared by prefix, then number."""
    match = _LABEL_RE.match(str(label))
    if match:
        return (match.group(1), int(match.group(2)), "")
    return (str(label), -1, str(label))


def _round_m(point_mm) -> list:
    """mm -> metres, rounded to 0.01 mm (5 decimals)."""
    return [round(float(c) / 1000.0, 5) for c in point_mm]


# * ---------------------------------------------------------------------------
# * Reading the cell snapshot
# * ---------------------------------------------------------------------------


def mocap_joints(bodies: dict) -> list:
    """``[(joint_id, body_key, body_info), ...]`` for every MoCap joint in *bodies*.

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
        out.append((parts[0], key, info))
    return out


def bar_bodies(bodies: dict) -> dict:
    """``{bar_id: (body_key, body_info)}`` for every bar body in *bodies*."""
    return {
        info["parent_bar_id"]: (key, info)
        for key, info in bodies.items()
        if info.get("kind") == "bar" and info.get("parent_bar_id")
    }


def _bar_order(bar_ids, bar_seq: dict) -> list:
    """Bar ids in assembly order; bars without a sequence number follow, by bar number."""
    return sorted(
        bar_ids,
        key=lambda b: (bar_seq.get(b) is None, bar_seq.get(b) or 0, jnc.bar_sort_key(b)),
    )


# * ---------------------------------------------------------------------------
# * Building the file
# * ---------------------------------------------------------------------------


def build_payload(
    bodies: dict,
    halves: dict,
    bar_seq: dict,
    *,
    source: str = "",
    tol_m: float = DEFAULT_TOL_M,
    cell_file: str = "RobotCell.json",
) -> dict:
    """The ``MoCapMarkers.json`` payload (schema 2).

    Args:
        bodies: ``{key: body_info}`` of the assembly cell, already cut to what
            ``RobotCell.json`` holds.  A bar body has ``frame_world_mm`` (the
            bar frame, mm) and ``parent_bar_id``; a joint body additionally
            ``block_name``.
        halves: ``{block_name: JointHalfDef}`` -- the registry's halves, read
            for ``marker_points_mm`` (block-local mm) and ``marker_diameter_mm``.
        bar_seq: ``{bar_id: assembly sequence number or None}``.
        source: the Rhino document name, recorded in the file.
        tol_m: the layout-ambiguity tolerance in metres.
        cell_file: the cell file this sidecar belongs to.

    Raises:
        MoCapExportError: a MoCap block without recorded markers or diameter,
            or a MoCap joint whose bar is not in *bodies*.
        MoCapLayoutError: a bar whose marker layout is ambiguous.
    """
    bars = bar_bodies(bodies)

    # Group the MoCap joints by bar, refusing anything incomplete.
    joints_by_bar = {bar_id: [] for bar_id in bars}
    blocks_used = {}
    for jid, key, info in mocap_joints(bodies):
        bar_id = info.get("parent_bar_id")
        if bar_id not in bars:
            raise MoCapExportError(
                f"{key}: its bar {bar_id or '<none>'} has no bar body in this export; "
                "rebuild the cell (RSRebuildRobotCell) or check the joint's parent_bar_id."
            )
        block_name = info.get("block_name") or ""
        half = halves.get(block_name)
        if half is None or not half.marker_points_mm:
            raise MoCapExportError(
                f"{key}: block {block_name!r} has no recorded marker points; "
                "run RSDefineJointHalf on it and pick the marker spheres."
            )
        if not half.marker_diameter_mm:
            raise MoCapExportError(
                f"{key}: block {block_name!r} has no recorded marker diameter; "
                "run RSDefineJointHalf on it and pick the marker spheres again."
            )
        blocks_used[block_name] = half
        joints_by_bar[bar_id].append((jid, key, info, half))

    bars_out = {}
    bars_without = []
    for bar_id in _bar_order(bars, bar_seq):
        bar_key, bar_info = bars[bar_id]
        if not joints_by_bar[bar_id]:
            bars_without.append(bar_key)
            continue
        bar_frame = np.asarray(bar_info["frame_world_mm"], dtype=float)

        # Joints along the bar from its start (z of the joint origin in the bar frame).
        entries = []
        for jid, key, info, half in joints_by_bar[bar_id]:
            joint_world = np.asarray(info["frame_world_mm"], dtype=float)
            along_mm = float(local_transform(bar_frame, joint_world)[2, 3])
            entries.append((along_mm, jid, key, joint_world, half))
        entries.sort(key=lambda e: (e[0], e[1]))

        markers = []
        for _along, jid, key, joint_world, half in entries:
            # inv(bar frame) @ joint frame @ block-local point, in mm, then metres.
            in_bar_mm = markers_in_frame_mm(joint_world, half.marker_points_mm, bar_frame)
            for label in sorted(in_bar_mm, key=label_sort_key):
                markers.append({
                    "name": jnc.marker_name(jid, jnc.MOCAP, label),
                    "joint": key,
                    "label": label,
                    "block": half.block_name,
                    "bar_m": _round_m(in_bar_mm[label]),
                })

        # ! The layout check: refuse a bar Motive could see in two poses.
        points_m = np.array([m["bar_m"] for m in markers], dtype=float)
        report = require_unambiguous(points_m, tol_m, name=f"bar {bar_id}", hint=LAYOUT_HINT)

        bars_out[bar_key] = {
            "bar_id": bar_id,
            "assembly_seq": bar_seq.get(bar_id),
            "layout": {
                "n_markers": report.n_markers,
                "nearest_miss_m": (
                    None if report.nearest_miss_m is None else round(report.nearest_miss_m, 5)
                ),
            },
            "markers": markers,
        }

    return {
        "schema_version": SCHEMA_VERSION,
        "source": source,
        "units": "m",
        "cell_file": cell_file,
        "bar_frame": (
            "frame of the bar body of the same name in the cell / BarAction states: "
            "origin at the bar start, z along the bar; world = bar_state_frame @ bar_m"
        ),
        "marker_name": "<joint object name>_<label>, e.g. J20-21_mocap_M1 (name the Motive marker this)",
        "layout_tolerance_m": tol_m,
        "marker_blocks": {
            name: {
                "marker_diameter_m": round(half.marker_diameter_mm / 1000.0, 5),
                "points_block_m": {
                    label: _round_m(point)
                    for label, point in sorted(half.marker_points_mm.items(), key=lambda kv: label_sort_key(kv[0]))
                },
            }
            for name, half in sorted(blocks_used.items())
        },
        "bars": bars_out,
        "bars_without_markers": bars_without,
    }


# * ---------------------------------------------------------------------------
# * Reading the payload back
# * ---------------------------------------------------------------------------


def marker_count(payload: dict) -> int:
    return sum(len(bar["markers"]) for bar in payload["bars"].values())


def summary(payload: dict) -> str:
    """``"B20: 8 markers on 4 joints; B21: 8 markers on 4 joints"``."""
    parts = []
    for bar in payload["bars"].values():
        joints = {m["joint"] for m in bar["markers"]}
        parts.append(f"{bar['bar_id']}: {len(bar['markers'])} markers on {len(joints)} joints")
    return "; ".join(parts)


def layout_notes(payload: dict, factor: float = 3.0) -> list:
    """Bars whose layout passed only narrowly: nearest miss below ``factor * tol``."""
    tol = float(payload["layout_tolerance_m"])
    notes = []
    for bar in payload["bars"].values():
        miss = bar["layout"].get("nearest_miss_m")
        if miss is not None and miss < factor * tol:
            notes.append(
                f"{bar['bar_id']}: another pose fits its markers within {miss * 1000:.1f} mm "
                f"(tolerance {tol * 1000:.1f} mm) -- fine, but with little margin."
            )
    return notes


def world_points_mm(payload: dict, bodies: dict) -> dict:
    """``{marker name: world xyz (mm)}`` -- the bar body's frame applied to ``bar_m``.

    Used for the Rhino labels, so the dots come from the same numbers as the file.
    """
    bars = bar_bodies(bodies)
    out = {}
    for bar in payload["bars"].values():
        bar_frame = np.asarray(bars[bar["bar_id"]][1]["frame_world_mm"], dtype=float)
        for marker in bar["markers"]:
            point_mm = np.asarray(marker["bar_m"], dtype=float) * 1000.0
            out[marker["name"]] = np.asarray(transform_point(bar_frame, point_mm), dtype=float)
    return out


__all__ = [
    "FILE_NAME",
    "LAYOUT_HINT",
    "SCHEMA_VERSION",
    "MoCapExportError",
    "bar_bodies",
    "build_payload",
    "label_sort_key",
    "layout_notes",
    "marker_count",
    "mocap_joints",
    "summary",
    "world_points_mm",
]
