#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
"""RSSetRodLayer - assign ``layer_id`` (and the ``grounded`` flag it implies).

``layer_id`` is the storey index the stability simulation reads out of
``rod_list``, and ``grounded`` says whether the rod stands on the ground.
Both are written onto the bar curve by RSImportScaffoldJSON and read straight
back by RSExportScaffoldJSON, so without this command the only way to change
them is to edit the JSON and re-import.  Here you pick the bars instead:

    layer_id 0        -> grounded true
    layer_id 1, 2, …  -> grounded false

Select as many bars as you like -- centerline curves or their tube previews,
mixed freely, since ``bar_or_tube_filter`` accepts both and
``resolve_picked_to_bar_curve`` maps a tube back to its curve -- then type the
layer id.  A bar drawn in Rhino that was never imported can be assigned too:
it exports with a freshly minted ``rod_id`` but keeps the ``layer_id`` set
here instead of defaulting to 0.

Those two user-text keys are the only thing that changes.  No geometry moves,
no bar is renumbered, and ``bar_seq`` is untouched: import seeds the sequence
from the layers once, but from then on it belongs to RSSequenceEdit and is
independent of ``layer_id``.  There is no ``repair_on_entry`` pass either --
nothing here depends on the previews being current, and regenerating a tube
would drop it out of a selection made before the command started.
"""

from __future__ import annotations

import importlib
import os
import sys

import rhinoscriptsyntax as rs

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import rhino_bar_pick as bar_pick
from core import rhino_bar_registry as registry
from core import scaffold_json as sj

# Command name used in every command-line message.
CMD = "RSSetRodLayer"


def _current_layer_id(curve_id):
    """The bar's stored ``layer_id``, defaulting to 0 exactly as export does."""
    return sj.parse_int(rs.GetUserText(curve_id, sj.KEY_ROD_LAYER_ID), 0)


def _format_bar_ids(bar_ids):
    """Comma-separated bar ids, abbreviated past the first 15 like the
    RSImportScaffoldJSON / RSExportScaffoldJSON summaries do."""
    shown = ", ".join(bar_ids[:15])
    return shown + (f" ... (+{len(bar_ids) - 15})" if len(bar_ids) > 15 else "")


def main():
    importlib.reload(sj)

    picked = rs.GetObjects(
        "Select bars to assign a layer_id (centerlines or tube previews)",
        preselect=True,
        select=False,
        custom_filter=bar_pick.bar_or_tube_filter,
    )
    if not picked:
        print(f"{CMD}: cancelled.")
        return

    # Whichever geometry was clicked, work on the centerline: that is where the
    # rod user text lives.  Keying by bar id also folds a bar picked twice (its
    # curve AND its tube) into one target.
    targets = {}
    for oid in picked:
        curve_id = bar_pick.resolve_picked_to_bar_curve(oid)
        if curve_id is None:
            continue  # stale tube whose bar has been deleted
        bar_id = rs.GetUserText(curve_id, registry.BAR_ID_KEY)
        if bar_id:
            targets[bar_id] = curve_id
    if not targets:
        print(f"{CMD}: nothing to do -- no registered bar in the selection.")
        return
    bar_ids = sorted(targets, key=lambda b: registry._parse_bar_number(b) or 0)

    # Pre-fill the prompt with the picked bars' layer when they all agree on one.
    layers_now = {_current_layer_id(targets[bar_id]) for bar_id in bar_ids}
    layer_id = rs.GetInteger(
        f"layer_id for {len(bar_ids)} bar(s)  [0 = grounded, 1,2,... = not grounded]",
        number=layers_now.pop() if len(layers_now) == 1 else 0,
        minimum=0,
    )
    if layer_id is None:
        print(f"{CMD}: cancelled.")
        return
    layer_id = int(layer_id)
    grounded = sj.grounded_for_layer(layer_id)

    unchanged = 0
    not_imported = []
    for bar_id in bar_ids:
        curve_id = targets[bar_id]
        if _current_layer_id(curve_id) == layer_id:
            unchanged += 1
        rs.SetUserText(curve_id, sj.KEY_ROD_LAYER_ID, str(layer_id))
        rs.SetUserText(curve_id, sj.KEY_ROD_GROUNDED, str(grounded))
        if not rs.GetUserText(curve_id, sj.KEY_ROD_ID):
            not_imported.append(bar_id)

    # ---- summary ----------------------------------------------------------
    print(
        f"{CMD}: layer_id {layer_id}, grounded {str(grounded).lower()} -> "
        f"{len(bar_ids)} bar(s): {_format_bar_ids(bar_ids)}"
    )
    if unchanged:
        print(f"  {unchanged} of them were already on layer {layer_id}.")
    if not_imported:
        print(
            f"  {len(not_imported)} bar(s) carry no rod_id (never imported from JSON): "
            f"{_format_bar_ids(not_imported)} -- they export with a new rod_id, but "
            "now keep this layer_id instead of defaulting to 0."
        )

    counts = {}
    grounded_total = 0
    for curve_id in registry.get_all_bars().values():
        layer = _current_layer_id(curve_id)
        counts[layer] = counts.get(layer, 0) + 1
        if sj.parse_bool(rs.GetUserText(curve_id, sj.KEY_ROD_GROUNDED)):
            grounded_total += 1
    print(
        "  document now: "
        + ", ".join(f"layer {layer}->{count} bar(s)" for layer, count in sorted(counts.items()))
        + f" | grounded: {grounded_total}"
    )
    print("  RSExportScaffoldJSON writes the new layer_id/grounded into rod_list.")


if __name__ == "__main__":
    main()