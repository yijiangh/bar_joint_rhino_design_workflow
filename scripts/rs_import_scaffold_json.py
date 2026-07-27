#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
"""RSImportScaffoldJSON - build registered bars from a scaffolding JSON file.

Reads a ``node_list`` / ``rod_list`` / ``coupler_list`` document (the
format produced by the upstream layout generator, e.g.
``large_scaffold.json``) and bakes one bar per rod:

* the centerline is the line between the rod's two ``end_node_ids``,
* the bar is registered exactly like RSCreateBar does it -- ``bar_type`` /
  ``bar_id`` / ``bar_guid`` / ``bar_seq`` UserText, name, centerline
  layer, tube preview -- with ``bar_id`` set to ``B<rod_id>`` so the Rhino
  id reads the same as the rod id,
* every field of the source file is written back onto the document so
  nothing is lost by the trip through Rhino: the per-rod fields go on the
  bar curve as UserText, and ``coupler_list`` (plus any node/top-level
  fields this schema does not model) goes into document UserText.

Assembly sequence numbers follow ``layer_id`` then ``rod_id``, i.e. the
model is built bottom-up.  RSJointPlace gives the earlier bar of a pair
the female half, so this ordering is what decides joint roles later.

After importing, the normal design loop applies: RSBarSnap / RSBarBrace to
push bars out to their joint contact distance, joint placement, then
RSExportScaffoldJSON to write the same schema back out.
"""

import importlib
import math
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import scaffold_json as sj
from core.rhino_bar_registry import (
    BAR_GUID_KEY,
    BAR_ID_KEY,
    BAR_SEQ_KEY,
    BAR_TYPE_KEY,
    BAR_TYPE_VALUE,
    BAR_CENTERLINE_LAYER,
    TUBE_AXIS_GUID_KEY,
    ensure_bar_preview,
    repair_on_entry,
)
from core.rhino_helpers import delete_objects, ensure_layer, suspend_redraw


#: Rhino unit-system code for millimetres.  The whole toolchain (bar radius,
#: joint contact distances, screw offsets) is authored in mm, and so is the
#: scaffolding JSON, so anything else is worth a warning.
_UNIT_SYSTEM_MILLIMETERS = 2

_MSGBOX_YESNO = 4
_MSGBOX_YES = 6

#: Rods shorter than this are degenerate -- Rhino cannot make a line out of
#: them and no bar could be fabricated at that length anyway.
_MIN_ROD_LENGTH_MM = 1e-6


def _length(start_xyz, end_xyz):
    """Straight-line distance between two ``(x, y, z)`` tuples."""
    return math.sqrt(sum((e - s) ** 2 for s, e in zip(start_xyz, end_xyz)))


# ---------------------------------------------------------------------------
# Existing-import detection / cleanup
# ---------------------------------------------------------------------------


def _imported_bar_ids():
    """Return the object ids of every bar that already carries a ``rod_id``."""
    return [
        oid
        for oid in rs.AllObjects()
        if rs.GetUserText(oid, BAR_TYPE_KEY) == BAR_TYPE_VALUE
        and rs.GetUserText(oid, sj.KEY_ROD_ID)
    ]


def _purge_previous_import(bar_oids):
    """Delete previously imported bars and everything hanging off them.

    Removes, in this order: the joint block instances parented to those
    bars (and the robotic tools placed on those joints), the tube previews
    pointing at those centerlines, and finally the centerlines.  Returns a
    ``{"bars": n, "joints": n, "tools": n, "tubes": n}`` count dict.
    """
    bar_ids = {rs.GetUserText(oid, BAR_ID_KEY) for oid in bar_oids}
    bar_guids = {str(rs.coerceguid(oid)) for oid in bar_oids}

    joint_oids = []
    joint_ids = set()
    for layer in (
        config.LAYER_JOINT_FEMALE_INSTANCES,
        config.LAYER_JOINT_MALE_INSTANCES,
        config.LAYER_JOINT_GROUND_INSTANCES,
    ):
        if not rs.IsLayer(layer):
            continue
        for oid in rs.ObjectsByLayer(layer) or []:
            if rs.GetUserText(oid, "parent_bar_id") in bar_ids:
                joint_oids.append(oid)
                joint_id = rs.GetUserText(oid, "joint_id")
                if joint_id:
                    joint_ids.add(joint_id)

    tool_oids = []
    if rs.IsLayer(config.LAYER_TOOL_INSTANCES):
        tool_oids = [
            oid
            for oid in rs.ObjectsByLayer(config.LAYER_TOOL_INSTANCES) or []
            if rs.GetUserText(oid, "joint_id") in joint_ids
        ]

    tube_oids = []
    if rs.IsLayer(config.LAYER_BAR_TUBE_PREVIEWS):
        tube_oids = [
            oid
            for oid in rs.ObjectsByLayer(config.LAYER_BAR_TUBE_PREVIEWS) or []
            if rs.GetUserText(oid, TUBE_AXIS_GUID_KEY) in bar_guids
        ]

    with suspend_redraw():
        delete_objects(tool_oids)
        delete_objects(joint_oids)
        delete_objects(tube_oids)
        delete_objects(list(bar_oids))
    return {
        "bars": len(bar_oids),
        "joints": len(joint_oids),
        "tools": len(tool_oids),
        "tubes": len(tube_oids),
    }


# ---------------------------------------------------------------------------
# Baking
# ---------------------------------------------------------------------------


def _register_bar(curve_id, bar_id, seq):
    """Write the bar-registry UserText that RSCreateBar would have written.

    Set directly rather than through ``ensure_bar_id`` because that helper
    always mints the next free ``B<n>``; here the id has to be ``B<rod_id>``
    so the Rhino bar id and the JSON rod id read the same.
    """
    rs.SetUserText(curve_id, BAR_TYPE_KEY, BAR_TYPE_VALUE)
    rs.SetUserText(curve_id, BAR_ID_KEY, bar_id)
    rs.SetUserText(curve_id, BAR_GUID_KEY, str(rs.coerceguid(curve_id)))
    rs.SetUserText(curve_id, BAR_SEQ_KEY, str(seq))
    rs.ObjectName(curve_id, bar_id)
    rs.ObjectLayer(curve_id, BAR_CENTERLINE_LAYER)


def _write_rod_user_text(curve_id, rod, start_xyz, end_xyz, coupled_rod_ids):
    """Persist the rod's schema fields on the bar centerline."""
    rs.SetUserText(curve_id, sj.KEY_ROD_ID, str(rod["rod_id"]))
    rs.SetUserText(curve_id, sj.KEY_ROD_LAYER_ID, str(rod["layer_id"]))
    rs.SetUserText(curve_id, sj.KEY_ROD_GROUNDED, str(bool(rod["grounded"])))
    rs.SetUserText(
        curve_id,
        sj.KEY_ROD_NODE_IDS,
        f"{rod['end_node_ids'][0]},{rod['end_node_ids'][1]}",
    )
    rs.SetUserText(curve_id, sj.KEY_ROD_START_XYZ, sj.format_xyz(start_xyz))
    rs.SetUserText(curve_id, sj.KEY_ROD_END_XYZ, sj.format_xyz(end_xyz))
    if coupled_rod_ids:
        rs.SetUserText(curve_id, sj.KEY_ROD_COUPLED, sj.format_id_list(coupled_rod_ids))
    if rod["extra"]:
        rs.SetUserText(curve_id, sj.KEY_ROD_EXTRA, sj.dumps_compact(rod["extra"]))


def _store_document_data(source_path, parsed):
    """Park the non-per-bar parts of the source file in document UserText.

    ``coupler_list`` is the model's connectivity and must survive the round
    trip even though it has no Rhino geometry of its own; node-level and
    top-level fields outside this schema are kept for the same reason.
    """
    rs.SetDocumentUserText(sj.DOC_KEY_SOURCE, source_path)
    rs.SetDocumentUserText(
        sj.DOC_KEY_COUPLERS,
        sj.dumps_compact([list(c["rod_ids"]) for c in parsed["couplers"]]),
    )
    node_extra = {
        str(node_id): data["extra"]
        for node_id, data in parsed["nodes"].items()
        if data["extra"]
    }
    # Always write these two, deleting the key when the new file has nothing
    # for it (the 1-argument form removes it) -- otherwise a re-import of a
    # leaner file would leave the previous file's extras behind for export.
    for key, value in (
        (sj.DOC_KEY_NODE_EXTRA, node_extra),
        (sj.DOC_KEY_TOP_EXTRA, parsed["top_extra"]),
    ):
        if value:
            rs.SetDocumentUserText(key, sj.dumps_compact(value))
        else:
            rs.SetDocumentUserText(key)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    importlib.reload(config)
    importlib.reload(sj)
    repair_on_entry(float(config.BAR_RADIUS), "RSImportScaffoldJSON")

    if rs.UnitSystem() != _UNIT_SYSTEM_MILLIMETERS:
        print(
            "RSImportScaffoldJSON (warning): the document is not in millimetres. "
            "The scaffolding JSON and every joint/bar constant in this toolchain "
            "are mm, so the imported bars will be the wrong size."
        )

    source_path = rs.OpenFileName(
        "Select scaffolding JSON",
        "JSON files (*.json)|*.json||",
        folder=config.REPO_ROOT,
    )
    if not source_path:
        print("RSImportScaffoldJSON: Cancelled.")
        return

    try:
        parsed = sj.load_scaffold(source_path)
    except (OSError, ValueError) as exc:
        rs.MessageBox(f"Could not read the JSON file:\n\n{exc}", 0, "RSImportScaffoldJSON")
        print(f"RSImportScaffoldJSON: failed to read {source_path}: {exc}")
        return

    for warning in parsed["warnings"]:
        print(f"RSImportScaffoldJSON (warning): {warning}")

    rods = parsed["rods"]
    if not rods:
        print("RSImportScaffoldJSON: the file contains no usable rods; nothing to do.")
        return

    existing = _imported_bar_ids()
    if existing:
        answer = rs.MessageBox(
            f"This document already contains {len(existing)} imported bar(s).\n\n"
            "Yes  - delete them (with their joints and tools) and re-import.\n"
            "No   - cancel and leave the document untouched.",
            _MSGBOX_YESNO,
            "RSImportScaffoldJSON",
        )
        if answer != _MSGBOX_YES:
            print("RSImportScaffoldJSON: Cancelled (document left untouched).")
            return
        counts = _purge_previous_import(existing)
        print(
            "RSImportScaffoldJSON: purged previous import -- "
            f"{counts['bars']} bar(s), {counts['tubes']} tube(s), "
            f"{counts['joints']} joint(s), {counts['tools']} tool(s)."
        )

    ensure_layer(BAR_CENTERLINE_LAYER)
    nodes = parsed["nodes"]
    coupled = sj.coupled_rods_map(parsed["couplers"])
    ordered = sj.assembly_order(rods)

    created = 0
    skipped = []
    with suspend_redraw():
        for seq, rod in enumerate(ordered, start=1):
            start_id, end_id = rod["end_node_ids"]
            start_xyz = nodes[start_id]["point"]
            end_xyz = nodes[end_id]["point"]
            if _length(start_xyz, end_xyz) <= _MIN_ROD_LENGTH_MM:
                skipped.append(
                    f"rod {rod['rod_id']} (nodes {start_id}/{end_id} are coincident)"
                )
                continue
            curve_id = rs.AddLine(start_xyz, end_xyz)
            if curve_id is None:
                skipped.append(f"rod {rod['rod_id']} (Rhino refused to add the line)")
                continue
            bar_id = f"B{rod['rod_id']}"
            _register_bar(curve_id, bar_id, seq)
            _write_rod_user_text(
                curve_id, rod, start_xyz, end_xyz, coupled.get(rod["rod_id"], [])
            )
            ensure_bar_preview(curve_id, float(config.BAR_RADIUS), bar_id=bar_id)
            created += 1

    _store_document_data(source_path, parsed)

    # ---- summary ----------------------------------------------------------
    layers = {}
    grounded = 0
    for rod in rods:
        layers[rod["layer_id"]] = layers.get(rod["layer_id"], 0) + 1
        if rod["grounded"]:
            grounded += 1
    print(f"RSImportScaffoldJSON: imported {os.path.basename(source_path)}")
    print(
        f"  {created} bar(s) created from {len(rods)} rod(s), "
        f"{len(nodes)} node(s), {len(parsed['couplers'])} coupler(s)."
    )
    print(
        "  bars per layer_id: "
        + ", ".join(f"{layer_id}->{count}" for layer_id, count in sorted(layers.items()))
        + f" | grounded: {grounded}"
    )
    print(
        f"  bar ids B{ordered[0]['rod_id']}.. (bar_id = B<rod_id>), "
        f"assembly sequence 1..{created} ordered by (layer_id, rod_id)."
    )
    if skipped:
        print(f"  {len(skipped)} rod(s) skipped: " + "; ".join(skipped))
    print(
        "  coupler_list and any unmodelled fields are stored in document user text; "
        "RSExportScaffoldJSON writes them back out."
    )


if __name__ == "__main__":
    main()
