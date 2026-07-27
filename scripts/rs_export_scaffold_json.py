#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSExportScaffoldJSON - write the Rhino bars back out as scaffolding JSON.

The inverse of RSImportScaffoldJSON: reads every registered bar, rebuilds
``node_list`` / ``rod_list`` / ``coupler_list`` from the *current* curve
geometry, and saves a file in the same schema as the imported one.

What survives the round trip
----------------------------

``rod_id``, ``layer_id``, ``grounded`` and any unmodelled rod fields come
straight back off the bar's UserText, so a bar keeps its identity even
after RSBarSnap has moved it or RSReorderBarID has renumbered ``bar_id``.
``coupler_list`` comes from the document UserText written at import,
merged with any *additional* pairs discovered from joint blocks the
designer has placed since -- so joints added in Rhino show up as new
couplers instead of being lost.

What necessarily changes
------------------------

Node positions.  In the source file the rods meet exactly at shared
nodes; a buildable model cannot, because each joint holds its two bars
apart by the pair's contact distance.  So a source node whose incident
bar ends have been pulled apart is split into one node per distinct
position (the original ``node_id`` stays with the lowest-numbered rod,
the others get fresh ids past the top of the original range).  Rod-to-rod
connectivity is unaffected -- it lives in ``coupler_list``, not in shared
node ids.

Bars drawn in Rhino without an imported ``rod_id`` (e.g. a brace added by
hand) are exported too, with freshly minted rod ids, ``layer_id`` 0 and
``grounded`` false; they are listed in the summary so the defaults are
visible.
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
from core import scaffold_json as sj
from core.rhino_bar_registry import (
    BAR_ID_KEY,
    BAR_TYPE_KEY,
    BAR_TYPE_VALUE,
    repair_on_entry,
)
from core.rhino_helpers import curve_endpoints

#: Endpoints closer than this (mm, per axis) are the same node.  Bars that
#: have not been moved keep bit-identical endpoints, and RSBarSnap offsets
#: are tens of millimetres, so the exact value is not delicate.
NODE_MERGE_TOL_MM = 1e-6

#: Two bars whose centerlines come closer than this are physically
#: interfering (their tubes overlap).  Reported for pairs that are *not*
#: coupled, as a "is this model good to export" check.
_INTERFERENCE_TOL_MM = 2.0 * float(config.BAR_RADIUS)

#: Above this many bars the all-pairs interference scan is skipped -- it is
#: O(n^2) and only meant as a quick sanity check.
_INTERFERENCE_SCAN_MAX_BARS = 400


# ---------------------------------------------------------------------------
# Reading bars back out of the document
# ---------------------------------------------------------------------------


def _collect_bars():
    """Return one record per registered bar, sorted so rod ids stay stable.

    Each record is ``{"oid", "bar_id", "rod_id", "imported", "layer_id",
    "grounded", "source_node_ids", "source_start", "source_end", "extra",
    "start", "end"}`` with ``start`` / ``end`` the *current* curve
    endpoints.  Bars imported from JSON come first (ordered by rod id),
    then any bar drawn in Rhino, ordered by its numeric bar id and given
    rod ids past the top of the imported range.
    """
    records = []
    for oid in rs.AllObjects():
        if rs.GetUserText(oid, BAR_TYPE_KEY) != BAR_TYPE_VALUE:
            continue
        start, end = curve_endpoints(oid)
        bar_id = rs.GetUserText(oid, BAR_ID_KEY) or ""
        records.append(
            {
                "oid": oid,
                "bar_id": bar_id,
                "rod_id": sj.parse_int(rs.GetUserText(oid, sj.KEY_ROD_ID)),
                "layer_id": sj.parse_int(rs.GetUserText(oid, sj.KEY_ROD_LAYER_ID), 0),
                "grounded": sj.parse_bool(rs.GetUserText(oid, sj.KEY_ROD_GROUNDED)),
                "source_node_ids": sj.parse_id_list(
                    rs.GetUserText(oid, sj.KEY_ROD_NODE_IDS)
                ),
                "source_start": sj.parse_xyz(rs.GetUserText(oid, sj.KEY_ROD_START_XYZ)),
                "source_end": sj.parse_xyz(rs.GetUserText(oid, sj.KEY_ROD_END_XYZ)),
                "extra": sj.loads_or_default(rs.GetUserText(oid, sj.KEY_ROD_EXTRA), {}),
                "start": tuple(float(v) for v in start),
                "end": tuple(float(v) for v in end),
            }
        )

    def _bar_number(record):
        digits = record["bar_id"].lstrip("B")
        try:
            return int(digits)
        except ValueError:
            return float("inf")

    for record in records:
        record["imported"] = record["rod_id"] is not None
    imported = sorted((r for r in records if r["imported"]), key=lambda r: r["rod_id"])
    fresh = sorted((r for r in records if not r["imported"]), key=_bar_number)
    next_rod_id = (imported[-1]["rod_id"] + 1) if imported else 0
    for record in fresh:
        record["rod_id"] = next_rod_id
        next_rod_id += 1
    return imported + fresh


def _endpoint_source_ids(record):
    """Map the bar's current (start, end) onto its source node ids.

    A curve the designer has reversed in Rhino would otherwise hand node
    ids to the wrong physical ends, so the two pairings are scored against
    the endpoint positions captured at import time and the better one wins.
    Returns ``(start_node_id, end_node_id)``, either entry possibly None.
    """
    node_ids = record["source_node_ids"]
    if len(node_ids) != 2:
        return (None, None)
    source_start, source_end = record["source_start"], record["source_end"]
    if source_start is None or source_end is None:
        return (node_ids[0], node_ids[1])

    def _dist(a, b):
        return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))

    forward = _dist(record["start"], source_start) + _dist(record["end"], source_end)
    reversed_ = _dist(record["start"], source_end) + _dist(record["end"], source_start)
    if reversed_ < forward:
        return (node_ids[1], node_ids[0])
    return (node_ids[0], node_ids[1])


# ---------------------------------------------------------------------------
# Couplers
# ---------------------------------------------------------------------------


def _stored_couplers(known_rod_ids):
    """Coupler pairs saved in document UserText at import time."""
    raw = sj.loads_or_default(rs.GetDocumentUserText(sj.DOC_KEY_COUPLERS), [])
    pairs = []
    dropped = []
    for entry in raw:
        try:
            a, b = int(entry[0]), int(entry[1])
        except (TypeError, ValueError, IndexError):
            continue
        if a in known_rod_ids and b in known_rod_ids:
            pairs.append((a, b))
        else:
            dropped.append((a, b))
    return pairs, dropped


def _joint_couplers(bar_id_to_rod_id):
    """Coupler pairs implied by the joint block instances in the document.

    Each placed pair writes ``parent_bar_id`` / ``connected_bar_id`` on both
    halves, which is exactly a rod-to-rod coupling.  Ground joints are
    skipped -- they anchor a bar to the ground, not to another rod.
    """
    pairs = set()
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            parent = bar_id_to_rod_id.get(rs.GetUserText(oid, "parent_bar_id"))
            connected = bar_id_to_rod_id.get(rs.GetUserText(oid, "connected_bar_id"))
            if parent is None or connected is None or parent == connected:
                continue
            pairs.add((min(parent, connected), max(parent, connected)))
    return sorted(pairs)


# ---------------------------------------------------------------------------
# Pre-export sanity report
# ---------------------------------------------------------------------------


def _segment_distance(record_a, record_b):
    """Shortest distance between two bar centerlines (finite segments)."""
    t_a, t_b = geometry.closest_params_finite_segments(
        record_a["start"], record_a["end"], record_b["start"], record_b["end"]
    )
    start_a, end_a = np.asarray(record_a["start"]), np.asarray(record_a["end"])
    start_b, end_b = np.asarray(record_b["start"]), np.asarray(record_b["end"])
    point_a = (1.0 - t_a) * start_a + t_a * end_a
    point_b = (1.0 - t_b) * start_b + t_b * end_b
    return float(np.linalg.norm(point_a - point_b))


def _report_geometry(records_by_rod, couplers):
    """Print how far apart the coupled bars actually are, plus interferences.

    Coupled bars still at ~0 mm have not been snapped out to a joint
    contact distance yet; non-coupled bars closer than two bar radii are
    physically overlapping.  Both are things to fix before exporting.
    """
    unsnapped = []
    histogram = {}
    for rod_a, rod_b in couplers:
        record_a, record_b = records_by_rod.get(rod_a), records_by_rod.get(rod_b)
        if record_a is None or record_b is None:
            continue
        try:
            distance = _segment_distance(record_a, record_b)
        except ValueError:
            continue  # degenerate (zero-length) bar; reported elsewhere
        key = round(distance, 1)
        histogram[key] = histogram.get(key, 0) + 1
        if distance < 1.0:
            unsnapped.append((rod_a, rod_b))

    print("  coupled-pair centerline gaps (mm -> count):")
    for distance, count in sorted(histogram.items())[:12]:
        print(f"    {distance:8.1f} -> {count}")
    if len(histogram) > 12:
        print(f"    ... {len(histogram) - 12} more distinct gap value(s)")
    if unsnapped:
        preview = ", ".join(f"{a}-{b}" for a, b in unsnapped[:10])
        print(
            f"  NOTE: {len(unsnapped)} coupled pair(s) are still touching "
            f"(gap < 1 mm) -- no joint fits there yet: {preview}"
            + (" ..." if len(unsnapped) > 10 else "")
        )

    if len(records_by_rod) > _INTERFERENCE_SCAN_MAX_BARS:
        print(
            f"  (interference scan skipped: {len(records_by_rod)} bars exceeds "
            f"the {_INTERFERENCE_SCAN_MAX_BARS}-bar limit)"
        )
        return
    coupled_set = {(min(a, b), max(a, b)) for a, b in couplers}
    rod_ids = sorted(records_by_rod)
    interfering = []
    for i, rod_a in enumerate(rod_ids):
        for rod_b in rod_ids[i + 1:]:
            if (rod_a, rod_b) in coupled_set:
                continue
            try:
                distance = _segment_distance(records_by_rod[rod_a], records_by_rod[rod_b])
            except ValueError:
                continue
            if distance < _INTERFERENCE_TOL_MM:
                interfering.append((rod_a, rod_b, distance))
    if interfering:
        interfering.sort(key=lambda item: item[2])
        preview = ", ".join(f"{a}-{b} ({d:.1f} mm)" for a, b, d in interfering[:10])
        print(
            f"  NOTE: {len(interfering)} uncoupled bar pair(s) are within "
            f"{_INTERFERENCE_TOL_MM:.0f} mm (tubes overlap): {preview}"
            + (" ..." if len(interfering) > 10 else "")
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    importlib.reload(config)
    importlib.reload(sj)
    repair_on_entry(float(config.BAR_RADIUS), "RSExportScaffoldJSON")

    records = _collect_bars()
    if not records:
        print("RSExportScaffoldJSON: no registered bars found; nothing to export.")
        return

    fresh = [r for r in records if not r["imported"]]
    records_by_rod = {r["rod_id"]: r for r in records}

    # ---- nodes ------------------------------------------------------------
    endpoint_records = []
    for record in records:
        start_node_id, end_node_id = _endpoint_source_ids(record)
        endpoint_records.append(
            {
                "rod_id": record["rod_id"],
                "index": 0,
                "point": record["start"],
                "source_node_id": start_node_id,
            }
        )
        endpoint_records.append(
            {
                "rod_id": record["rod_id"],
                "index": 1,
                "point": record["end"],
                "source_node_id": end_node_id,
            }
        )

    node_extra = {
        int(node_id): extra
        for node_id, extra in sj.loads_or_default(
            rs.GetDocumentUserText(sj.DOC_KEY_NODE_EXTRA), {}
        ).items()
    }
    built = sj.build_node_list(
        endpoint_records, tol=NODE_MERGE_TOL_MM, node_extra=node_extra
    )

    # ---- rods -------------------------------------------------------------
    rods = []
    for record in records:
        rod_id = record["rod_id"]
        rods.append(
            {
                "rod_id": rod_id,
                "end_node_ids": (
                    built["rod_end_node_ids"][(rod_id, 0)],
                    built["rod_end_node_ids"][(rod_id, 1)],
                ),
                "layer_id": record["layer_id"],
                "grounded": record["grounded"],
                "extra": record["extra"],
            }
        )

    # ---- couplers ---------------------------------------------------------
    stored, dropped = _stored_couplers(set(records_by_rod))
    bar_id_to_rod_id = {r["bar_id"]: r["rod_id"] for r in records if r["bar_id"]}
    discovered = _joint_couplers(bar_id_to_rod_id)
    couplers, added = sj.merge_couplers(stored, discovered)

    document = sj.build_document(
        built["nodes"],
        rods,
        couplers,
        top_extra=sj.loads_or_default(
            rs.GetDocumentUserText(sj.DOC_KEY_TOP_EXTRA), {}
        ),
    )

    # ---- report -----------------------------------------------------------
    print(
        f"RSExportScaffoldJSON: {len(rods)} rod(s), {len(built['nodes'])} node(s), "
        f"{len(couplers)} coupler(s)."
    )
    if built["split_source_ids"]:
        print(
            f"  {len(built['split_source_ids'])} source node(s) split into separate "
            "positions (bars pulled apart to their joint contact distance): "
            + ", ".join(str(n) for n in built["split_source_ids"][:15])
            + (" ..." if len(built["split_source_ids"]) > 15 else "")
        )
    if built["merged_clusters"]:
        print(
            f"  {len(built['merged_clusters'])} position(s) now shared by originally "
            "distinct nodes; the lowest source id was kept."
        )
    if fresh:
        print(
            f"  {len(fresh)} bar(s) had no imported rod data and were exported with "
            "new rod ids, layer_id 0, grounded false: "
            + ", ".join(f"{r['bar_id']}->rod {r['rod_id']}" for r in fresh[:15])
            + (" ..." if len(fresh) > 15 else "")
        )
    if added:
        print(
            f"  {len(added)} coupler(s) added from joints placed in Rhino: "
            + ", ".join(f"{a}-{b}" for a, b in added[:15])
            + (" ..." if len(added) > 15 else "")
        )
    if dropped:
        print(
            f"  {len(dropped)} stored coupler(s) dropped -- their rod(s) are no longer "
            "in the document: "
            + ", ".join(f"{a}-{b}" for a, b in dropped[:15])
            + (" ..." if len(dropped) > 15 else "")
        )
    _report_geometry(records_by_rod, couplers)

    # ---- save -------------------------------------------------------------
    source_path = rs.GetDocumentUserText(sj.DOC_KEY_SOURCE) or ""
    if source_path:
        folder = os.path.dirname(source_path)
        stem = os.path.splitext(os.path.basename(source_path))[0]
    else:
        doc_path = sc.doc.Path or ""
        folder = os.path.dirname(doc_path) if doc_path else config.REPO_ROOT
        stem = (
            os.path.splitext(os.path.basename(doc_path))[0] if doc_path else "scaffold"
        )
    default_name = f"{stem}_rhino.json"

    save_path = rs.SaveFileName(
        "Save scaffolding JSON",
        "JSON files (*.json)|*.json||",
        folder=folder,
        filename=default_name,
    )
    if not save_path:
        print("RSExportScaffoldJSON: Cancelled (nothing written).")
        return
    try:
        sj.write_document(save_path, document)
    except OSError as exc:
        rs.MessageBox(f"Could not write the JSON file:\n\n{exc}", 0, "RSExportScaffoldJSON")
        print(f"RSExportScaffoldJSON: failed to write {save_path}: {exc}")
        return
    print(f"RSExportScaffoldJSON: saved to {save_path}")


if __name__ == "__main__":
    main()
