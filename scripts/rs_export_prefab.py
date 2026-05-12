#! python 3
# venv: scaffolding_env
"""RSExportPrefab - Export bar prefabrication data as JSON.

Scans all placed joint block instances, reads flat user text keys written
by RSJointPlace, and writes a JSON file compatible with the joint jig
controller.
"""

import importlib
import json
import math
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.rhino_helpers import curve_endpoints
from core.rhino_bar_registry import get_all_bars, BAR_ID_KEY, repair_on_entry

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_FEMALE_INSTANCES_LAYER = config.LAYER_JOINT_FEMALE_INSTANCES
_MALE_INSTANCES_LAYER = config.LAYER_JOINT_MALE_INSTANCES
_GROUND_INSTANCES_LAYER = config.LAYER_JOINT_GROUND_INSTANCES


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _bar_length(curve_id):
    start, end = curve_endpoints(curve_id)
    vec = [e - s for e, s in zip(end, start)]
    return round((sum(v * v for v in vec) ** 0.5), 2)


def _bar_start_and_dir(curve_id):
    """Return (start_list, unit_dir_list) for a bar curve."""
    start, end = curve_endpoints(curve_id)
    vec = [e - s for e, s in zip(end, start)]
    length = sum(v * v for v in vec) ** 0.5
    direction = [v / length for v in vec]
    return list(start), direction


def _block_origin_and_x(obj_id):
    """Return (origin_list, x_axis_list) from a block instance transform."""
    xf = rs.BlockInstanceXform(obj_id)
    origin = [xf[0, 3], xf[1, 3], xf[2, 3]]
    x_axis = [xf[0, 0], xf[1, 0], xf[2, 0]]
    return origin, x_axis


def _block_z_axis(obj_id):
    """Return joint Z-axis (assembly direction) from a block instance transform."""
    xf = rs.BlockInstanceXform(obj_id)
    z = [xf[0, 2], xf[1, 2], xf[2, 2]]
    n = (z[0] ** 2 + z[1] ** 2 + z[2] ** 2) ** 0.5
    return [v / n for v in z]


def _cross3(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _bar_x_axis(bar_dir):
    """Stable X-axis of the bar frame, perpendicular to bar_dir.

    Uses world_Z x bar_dir, falling back to world_X x bar_dir when the bar
    is nearly vertical (|dot(bar_dir, world_Z)| >= 0.95).
    """
    world_z = [0.0, 0.0, 1.0]
    world_x = [1.0, 0.0, 0.0]
    if abs(_dot3(world_z, bar_dir)) < 0.95:
        candidate = _cross3(world_z, bar_dir)
    else:
        candidate = _cross3(world_x, bar_dir)
    n = (candidate[0] ** 2 + candidate[1] ** 2 + candidate[2] ** 2) ** 0.5
    return [v / n for v in candidate]


def _compute_rotation_deg(joint_z, bar_x, bar_z):
    """Angle from bar_x to joint_z projected onto the bar XY plane, about bar_z.

    At rotation_deg=0 the joint assembly axis (Z) aligns with the bar X-axis.
    Positive rotation is CCW about bar Z (right-hand rule).
    """
    # Project joint_z onto bar XY plane
    d = _dot3(joint_z, bar_z)
    proj = [joint_z[i] - d * bar_z[i] for i in range(3)]
    proj_len = (proj[0] ** 2 + proj[1] ** 2 + proj[2] ** 2) ** 0.5
    if proj_len < 1e-9:
        return 0.0  # joint Z is parallel to bar Z; rotation is undefined
    proj = [v / proj_len for v in proj]
    cos_a = max(-1.0, min(1.0, _dot3(bar_x, proj)))
    sin_a = _dot3(_cross3(bar_x, proj), bar_z)
    return math.degrees(math.atan2(sin_a, cos_a))


def _collect_joint_blocks():
    """Return list of (obj_id, flat_data_dict) for all placed joint blocks."""
    results = []
    for layer in (_FEMALE_INSTANCES_LAYER, _MALE_INSTANCES_LAYER, _GROUND_INSTANCES_LAYER):
        objs = rs.ObjectsByLayer(layer) if rs.IsLayer(layer) else []
        if not objs:
            continue
        for obj_id in objs:
            joint_id = rs.GetUserText(obj_id, "joint_id")
            if not joint_id:
                continue
            data = {
                "obj_id": obj_id,
                "joint_id": joint_id,
                "type": rs.GetUserText(obj_id, "joint_type") or "",
                "subtype": rs.GetUserText(obj_id, "joint_subtype") or "",
                "bar_id": rs.GetUserText(obj_id, "parent_bar_id") or "",
                "connected_bar_id": rs.GetUserText(obj_id, "connected_bar_id") or "",
            }
            results.append(data)
    return results


def _bar_sort_key(bar_id):
    num = bar_id.lstrip("B")
    try:
        return int(num)
    except ValueError:
        return float("inf")


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

    bars_by_id = {}  # bar_id -> curve_guid
    for bar_id, curve_id in bars.items():
        bars_by_id[bar_id] = curve_id

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

    for bid in sorted(joints_per_bar.keys(), key=_bar_sort_key):
        if bid not in bars_by_id:
            errors.append(f"Bar {bid} referenced by joints but not found in document")
            continue

        bar_curve_id = bars_by_id[bid]
        bar_start, bar_dir = _bar_start_and_dir(bar_curve_id)
        bar_length = _bar_length(bar_curve_id)
        joint_entries = []

        bar_x = _bar_x_axis(bar_dir)

        for data in joints_per_bar[bid]:
            obj_id = data["obj_id"]
            joint_id = data["joint_id"]

            # All three exported quantities derived from world geometry
            try:
                origin, x_axis = _block_origin_and_x(obj_id)
                joint_z = _block_z_axis(obj_id)
            except Exception as exc:
                errors.append(
                    f"Joint {joint_id} on {bid}: could not read transform ({exc})"
                )
                continue

            # position_mm: signed projection of (origin - bar_start) onto bar_dir
            v = [origin[i] - bar_start[i] for i in range(3)]
            pos = round(sum(v[i] * bar_dir[i] for i in range(3)), 2)

            # ori: P if block x-axis points toward bar end, N toward start
            ori = "P" if _dot3(x_axis, bar_dir) > 0 else "N"

            # rotation_deg: angle from bar X-axis to joint Z-axis about bar Z
            rot = round(_compute_rotation_deg(joint_z, bar_x, bar_dir), 2)

            joint_entries.append(
                {
                    "joint_id": joint_id,
                    "type": data["type"],
                    "subtype": data["subtype"],
                    "ori": ori,
                    "position_mm": pos,
                    "rotation_deg": rot,
                }
            )

        joint_entries.sort(key=lambda j: j["position_mm"])
        bar_entries.append(
            {
                "bar_id": bid,
                "length_mm": bar_length,
                "joints": joint_entries,
            }
        )

    # Also include bars with no joints
    for bid in sorted(bars_by_id.keys(), key=_bar_sort_key):
        if bid not in joints_per_bar:
            bar_entries.append(
                {
                    "bar_id": bid,
                    "length_mm": _bar_length(bars_by_id[bid]),
                    "joints": [],
                }
            )
    bar_entries.sort(key=lambda b: _bar_sort_key(b["bar_id"]))

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
    default_path = os.path.join(doc_dir, f"{doc_name}_prefab.json")
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
