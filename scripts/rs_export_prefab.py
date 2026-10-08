#! python 3
# venv: scaffolding_env
"""RSExportPrefab - Export bar prefabrication data as JSON.

Scans all placed joint block instances, reads flat user text keys written
by RSJointPlace, and writes a JSON file compatible with the joint jig
controller.

A MoCap joint's entry additionally carries::

    "mocap": {"paired": true,                       # J... id; false for a standalone M...
              "markers_mm": {"M1": [x, y, z], ...}}  # sphere centres in the bar frame

The bar frame is the one ``position_mm`` / ``rotation_deg`` are measured in,
``joint_pair.canonical_bar_frame_from_line``: origin at the bar start, z along
the bar, x = world Z x bar (world X x bar for a near-vertical bar).  Coordinates are document units, taken as mm like the rest
of this export.  ``markers_mm`` is empty until the markers are recorded on the
block with RSDefineJointHalf.
"""

import importlib
import json
import math
import os
import sys

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import joint_name_conventions as jnc
from core import marker_points
from core.joint_pair import canonical_bar_frame_from_line, load_joint_registry
from core.joint_placement import block_orientation_tag
from core.rhino_helpers import curve_endpoints
from core.rhino_bar_registry import (
    get_all_bars,
    is_fake_bar,
    repair_on_entry,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _bar_endpoints(curve_id):
    start, end = curve_endpoints(curve_id)
    return np.asarray(start, dtype=float), np.asarray(end, dtype=float)


def _bar_length(curve_id):
    start, end = _bar_endpoints(curve_id)
    return round(float(np.linalg.norm(end - start)), 2)


def _block_world(obj_id) -> np.ndarray:
    """A block instance's world transform (document units, taken as mm)."""
    xf = rs.BlockInstanceXform(obj_id)
    return np.array([[xf[r, c] for c in range(4)] for r in range(4)], dtype=float)


def _rotation_deg(joint_z, bar_frame) -> float:
    """Angle from bar X to joint Z projected onto the bar XY plane, about bar Z.

    At rotation_deg=0 the joint assembly axis (Z) aligns with the bar X-axis.
    Positive rotation is CCW about bar Z (right-hand rule).
    """
    bar_x, bar_z = bar_frame[:3, 0], bar_frame[:3, 2]
    joint_z = joint_z / np.linalg.norm(joint_z)
    proj = joint_z - float(joint_z @ bar_z) * bar_z
    proj_len = float(np.linalg.norm(proj))
    if proj_len < 1e-9:
        return 0.0  # joint Z is parallel to bar Z; rotation is undefined
    proj = proj / proj_len
    cos_a = max(-1.0, min(1.0, float(bar_x @ proj)))
    sin_a = float(np.cross(bar_x, proj) @ bar_z)
    return math.degrees(math.atan2(sin_a, cos_a))


def _collect_joint_blocks():
    """Return list of (obj_id, flat_data_dict) for all placed joint blocks."""
    results = []
    for layer in jnc.JOINT_LAYERS:
        objs = rs.ObjectsByLayer(layer) if rs.IsLayer(layer) else []
        if not objs:
            continue
        for obj_id in objs:
            joint_id = rs.GetUserText(obj_id, jnc.UT_JOINT_ID)
            if not joint_id:
                continue
            # Every joint, Ground included, exports its block's Type and
            # Subtype: T20_Ground -> "T20" / "Ground", like T20_Female.
            data = {
                "obj_id": obj_id,
                "layer": layer,
                "joint_id": joint_id,
                "type": rs.GetUserText(obj_id, jnc.UT_JOINT_TYPE) or "",
                "subtype": rs.GetUserText(obj_id, jnc.UT_JOINT_SUBTYPE) or "",
                "bar_id": rs.GetUserText(obj_id, jnc.UT_PARENT_BAR) or "",
            }
            results.append(data)
    return results


def _mocap_entry(obj_id, joint_id, halves, block_world, bar_frame):
    """``{"paired": ..., "markers_mm": {...}}`` for one placed MoCap block."""
    half = halves.get(rs.BlockInstanceName(obj_id) or "")
    markers = {}
    if half is not None and half.marker_points_mm:
        markers = marker_points.markers_in_frame_mm(
            block_world, half.marker_points_mm, bar_frame
        )
    return {
        "paired": jnc.is_paired_half(jnc.MOCAP, joint_id),
        "markers_mm": markers,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    importlib.reload(config)
    repair_on_entry(float(config.BAR_RADIUS), "RSExportPrefab")
    # 1. Collect bars
    bars = get_all_bars()
    if not bars:
        print("RSExportPrefab: No registered bars found.")
        return

    # Staging bars are not fabricated -- they are put up by hand so the robot
    # has something to mate against, and exporting them would have the workshop
    # build parts nobody needs.  Reported, never silently dropped: a short
    # export that says nothing is indistinguishable from a broken one.
    bars_by_id = {}  # bar_id -> curve_guid
    skipped_fake = []
    for bar_id, curve_id in bars.items():
        if is_fake_bar(curve_id):
            skipped_fake.append(bar_id)
            continue
        bars_by_id[bar_id] = curve_id
    if skipped_fake:
        print(
            f"RSExportPrefab: skipping {len(skipped_fake)} fake bar(s) -- "
            f"{', '.join(sorted(skipped_fake))} (RSBarEdit > FakeBar to change)."
        )
    if not bars_by_id:
        print("RSExportPrefab: every registered bar is marked fake; nothing to export.")
        return

    # 2. Collect joint blocks
    all_joints = _collect_joint_blocks()
    if not all_joints:
        print("RSExportPrefab: No joint block instances found.")
        return

    # 3. Group by bar_id
    joints_per_bar = {}
    for data in all_joints:
        bid = data["bar_id"]
        if not bid:
            continue
        joints_per_bar.setdefault(bid, []).append(data)

    # 4. Build export
    errors = []
    bar_entries = []
    halves = load_joint_registry().halves  # for MoCap marker points

    for bid in sorted(joints_per_bar.keys(), key=jnc.bar_sort_key):
        if bid not in bars_by_id:
            errors.append(f"Bar {bid} referenced by joints but not found in document")
            continue

        bar_curve_id = bars_by_id[bid]
        bar_start, bar_end = _bar_endpoints(bar_curve_id)
        bar_frame = canonical_bar_frame_from_line(bar_start, bar_end)
        bar_dir = bar_frame[:3, 2]
        bar_length = round(float(np.linalg.norm(bar_end - bar_start)), 2)
        joint_entries = []

        for data in joints_per_bar[bid]:
            obj_id = data["obj_id"]
            joint_id = data["joint_id"]

            # All three exported quantities derived from world geometry
            try:
                block_world = _block_world(obj_id)
            except Exception as exc:
                errors.append(
                    f"Joint {joint_id} on {bid}: could not read transform ({exc})"
                )
                continue

            # position_mm: signed projection of (origin - bar_start) onto bar_dir
            pos = round(float((block_world[:3, 3] - bar_start) @ bar_dir), 2)
            # ori: P if block x-axis points toward bar end, N toward start
            ori = block_orientation_tag(block_world[:3, 0], bar_dir)
            # rotation_deg: angle from bar X-axis to joint Z-axis about bar Z
            rot = round(_rotation_deg(block_world[:3, 2], bar_frame), 2)

            entry = {
                "joint_id": joint_id,
                "type": data["type"],
                "subtype": data["subtype"],
                "ori": ori,
                "position_mm": pos,
                "rotation_deg": rot,
            }
            if jnc.subtype_of_layer(data["layer"]) == jnc.MOCAP:
                entry["mocap"] = _mocap_entry(
                    obj_id, joint_id, halves, block_world, bar_frame
                )
            joint_entries.append(entry)

        joint_entries.sort(key=lambda j: j["position_mm"])
        bar_entries.append(
            {
                "bar_id": bid,
                "length_mm": bar_length,
                "joints": joint_entries,
            }
        )

    # Also include bars with no joints
    for bid in sorted(bars_by_id.keys(), key=jnc.bar_sort_key):
        if bid not in joints_per_bar:
            bar_entries.append(
                {
                    "bar_id": bid,
                    "length_mm": _bar_length(bars_by_id[bid]),
                    "joints": [],
                }
            )
    bar_entries.sort(key=lambda b: jnc.bar_sort_key(b["bar_id"]))

    # Project ID from document name
    doc_path = sc.doc.Path or ""
    doc_name = (
        os.path.splitext(os.path.basename(doc_path))[0] if doc_path else "untitled"
    )

    export_data = {
        "schema_version": 1,
        "project_id": doc_name,
        "bars": bar_entries,
    }

    # 5. Print summary
    total_joints = sum(len(b["joints"]) for b in bar_entries)
    print(f"RSExportPrefab: {len(bar_entries)} bars, {total_joints} joints")
    if errors:
        print(f"  {len(errors)} error(s):")
        for e in errors:
            print(f"    - {e}")

    # Bill of materials
    print("\n--- Bill of Materials ---")

    # Bar lengths: round to 0.1 mm
    from collections import Counter

    bar_lengths = [round(b["length_mm"] / 0.1) * 0.1 for b in bar_entries]
    length_counts = Counter(bar_lengths)
    sorted_lengths = sorted(length_counts.keys())
    print("\nBar lengths:")
    for L in sorted_lengths:
        print(f"  {L:.1f} mm  x{length_counts[L]}")
    total_bar_length = sum(L * cnt for L, cnt in length_counts.items())
    print(f"Total bar length: {total_bar_length:.1f} mm")

    # Joint instances
    joint_keys = Counter(
        (j["type"], j["subtype"]) for b in bar_entries for j in b["joints"]
    )
    if joint_keys:
        print("\nJoint instances:")
        for (jtype, jsubtype), cnt in sorted(joint_keys.items()):
            label = f"{jtype}/{jsubtype}" if jsubtype else jtype
            print(f"  {label}  x{cnt}")

    print("--- End of BOM ---")

    # 6. Save file
    doc_dir = os.path.dirname(doc_path) if doc_path else os.getcwd()
    save_path = rs.SaveFileName(
        "Save prefab JSON",
        "JSON files (*.json)|*.json||",
        folder=doc_dir,
        filename=f"{doc_name}_prefab.json",
    )
    if not save_path:
        print("RSExportPrefab: Cancelled.")
        return

    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(export_data, f, indent=2)

    print(f"RSExportPrefab: Saved to {save_path}")


if __name__ == "__main__":
    main()
