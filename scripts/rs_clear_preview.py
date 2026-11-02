#! python 3
# venv: scaffolding_env
"""RSClearPreview - delete every baked robot preview, so the .3dm is small again.

The IK and plan previews (RSIKKeyframe, RSShowBarActionPlan, RSShowAssemblyPlan,
RSLoadSolvedBarAction, RSAssignAndShowWalkableGround) draw the robots, their
tools and every bar and joint as real mesh objects on the layers under
"MANAGED Scaffolding > IK Cache".  Until the fix in ``core.ik_viz`` (Rhino's
plain delete refuses hidden objects), every clear left the hidden part of a
preview set behind, so older files carry thousands of hidden meshes: 27,013 in
``261008_2ModulesDemo.3dm``.  This command removes them, hidden or not.

What it deletes:

* everything on "IK Cache" and its sub-layers, and the preview cache that
  points at it (the next preview bakes once, as after opening a file);
* the base markers and base guides ("Base Frame Preview", "Base Guide
  Preview"), and the groups they leave empty;
* the old preview layers "IKSupportPreview" and "IKPineapplePreview".

What it keeps: bars, joints, tools, walkable ground, obstacles, the MoCap
marker labels, and colours (RSClearColorPreview resets those).  Nothing here
is a design object; collision checks never use these meshes (PyBullet has its
own copy).  The deletion is not recorded for Undo, so the removed meshes do not
stay in memory.  Save afterwards to make the file smaller.
"""

import importlib
import os
import sys

import rhinoscriptsyntax as rs
import scriptcontext as sc

SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core import ik_viz as _ik_viz_module
from core import rhino_helpers as _rhino_helpers_module

CALLER = "RSClearPreview"

#: Layers from older versions of the preview code; cleared if a file still has them.
_LEGACY_PREVIEW_LAYERS = ("IKPineapplePreview",)


def _preview_layers(helpers):
    """Every preview layer that exists in this document, sub-layers included."""
    roots = (
        config.LAYER_IK_CACHE,
        config.LAYER_BASE_FRAME_PREVIEW,
        config.LAYER_BASE_GUIDE_PREVIEW,
        getattr(config, "SUPPORT_PREVIEW_LAYER", None),
        *_LEGACY_PREVIEW_LAYERS,
    )
    out = []
    for root in roots:
        if root:
            out.extend(helpers.layer_tree(root))
    return out


def _count(layers):
    """``(objects, hidden)`` on *layers*."""
    total = hidden = 0
    for layer in layers:
        for oid in rs.ObjectsByLayer(layer) or []:
            total += 1
            hidden += int(rs.IsObjectHidden(oid))
    return total, hidden


def _groups_of(oids) -> set:
    """Names of every group any of *oids* belongs to."""
    names = set()
    for oid in oids:
        names.update(rs.ObjectGroups(oid) or [])
    return names


def _delete_emptied_groups(names) -> int:
    """Delete those of *names* -- the preview objects' groups -- that are now empty.

    Only these: an empty group elsewhere in the document is not ours to remove.
    """
    groups = sc.doc.Groups
    n = 0
    for name in names:
        group = groups.FindName(name)
        if group is None or group.IsDeleted or groups.GroupObjectCount(group.Index):
            continue
        groups.Delete(group.Index)
        n += 1
    return n


def main():
    importlib.reload(config)
    helpers = importlib.reload(_rhino_helpers_module)
    ik_viz = importlib.reload(_ik_viz_module)

    layers = _preview_layers(helpers)
    before, hidden = _count(layers)
    print(f"{CALLER}: {before} preview object(s) on {len(layers)} layer(s), {hidden} of them hidden.")
    # Read before anything is deleted: discard_cache below empties the IK Cache
    # layers, and a deleted object no longer says which group it was in.
    preview_groups = _groups_of(
        oid for layer in layers for oid in (rs.ObjectsByLayer(layer) or [])
    )

    undo_was = sc.doc.UndoRecordingEnabled
    try:
        sc.doc.UndoRecordingEnabled = False
        # Ends any preview session left open (restores the document layers it
        # hid) and forgets the cached preview sets, so nothing points at the
        # meshes about to go.
        ik_viz.end_session()
        ik_viz.discard_cache()
        # discard_cache clears the IK Cache layers; this pass takes the rest
        # (base markers and guides, old layers).
        oids = [oid for layer in layers for oid in (rs.ObjectsByLayer(layer) or [])]
        with helpers.suspend_redraw():
            helpers.delete_objects_any_state(oids)
        n_groups = _delete_emptied_groups(preview_groups)
    finally:
        sc.doc.UndoRecordingEnabled = undo_was

    left, _ = _count(_preview_layers(helpers))
    print(
        f"{CALLER}: deleted {before - left} preview object(s) and "
        f"{n_groups} empty group(s); {left} left."
    )
    if before - left:
        print(f"{CALLER}: save the file to make it smaller.")
    sc.doc.Views.Redraw()


if __name__ == "__main__":
    main()
