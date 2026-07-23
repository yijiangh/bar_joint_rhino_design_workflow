"""Shared Rhino block-definition import helpers.

Both joint placement and robotic-tool placement need the same logic to
load an InstanceDefinition by name from an asset .3dm file.  This module
hosts the canonical implementation; both paths import from here.

Rhino-runtime-only at call time; safe to import from non-Rhino contexts
since the Rhino imports are deferred into each function body.
"""

from __future__ import annotations

import os


def has_block_definition(block_name: str) -> bool:
    """Return True if *block_name* is currently a non-deleted InstanceDefinition."""
    import scriptcontext as sc  # noqa: PLC0415

    for instance_def in sc.doc.InstanceDefinitions:
        if (
            instance_def is not None
            and not instance_def.IsDeleted
            and instance_def.Name == block_name
        ):
            return True
    return False


def _resolve_visible_layer_index(layer_name: str):
    """Ensure *layer_name* exists + is visible; return its doc layer index.

    Returns ``None`` if the layer cannot be resolved (so callers leave the
    imported attributes' layer untouched).
    """
    import scriptcontext as sc  # noqa: PLC0415
    from core.rhino_helpers import ensure_layer  # noqa: PLC0415

    ensure_layer(layer_name)
    index = sc.doc.Layers.FindByFullPath(layer_name, -1)
    return index if index is not None and index >= 0 else None


def import_block_definition_from_3dm(
    block_name: str, asset_path: str, *, layer_name: str | None = None
) -> bool:
    """Import a single block definition named *block_name* from *asset_path*.

    Strategy: read the .3dm via :class:`Rhino.FileIO.File3dm`, gather every
    geometry/attribute pair, and register them as one InstanceDefinition
    under *block_name*.  Falls back to Rhino's ``_-Insert`` command if the
    File3dm path fails.  Returns True on success.

    Args:
        block_name (str): the InstanceDefinition name to create.
        asset_path (str): the .3dm file to import the definition from.
        layer_name (str | None): if given, pin EVERY imported sub-object onto
            this (visible) layer.  ``InstanceDefinitions.Add`` does NOT import
            the source file's layer table, so each object's
            ``Attributes.LayerIndex`` would otherwise index the CURRENT doc's
            layers by the source file's numbering -- landing the geometry on an
            unrelated / hidden layer, so the inserted block shows nothing.
            Pinning keeps the whole block visible (used by tool placement to
            force parts onto the "Robotic Tool Instances" layer).
    """
    import Rhino  # noqa: PLC0415
    import rhinoscriptsyntax as rs  # noqa: PLC0415
    import scriptcontext as sc  # noqa: PLC0415

    asset_path = os.path.normpath(asset_path)
    if not os.path.isfile(asset_path):
        print(f"  [import_block] asset file not found: {asset_path}")
        return False

    target_layer_index = (
        _resolve_visible_layer_index(layer_name) if layer_name else None
    )

    # ---- Attempt 1: RhinoCommon File3dm direct read --------------------
    try:
        file3dm = Rhino.FileIO.File3dm.Read(asset_path)
        if file3dm is None:
            print(f"  [import_block] File3dm.Read returned None for {asset_path}")
        else:
            geometries = []
            attributes = []
            for obj in file3dm.Objects:
                if obj is None or obj.Geometry is None:
                    continue
                geometries.append(obj.Geometry.Duplicate())
                attr = (
                    obj.Attributes.Duplicate()
                    if obj.Attributes
                    else Rhino.DocObjects.ObjectAttributes()
                )
                # Force the sub-object onto a known visible layer (see docstring):
                # otherwise the source file's layer index makes the part invisible.
                if target_layer_index is not None:
                    attr.LayerIndex = target_layer_index
                attributes.append(attr)
            if geometries:
                idef_index = sc.doc.InstanceDefinitions.Add(
                    block_name,
                    f"Imported from {os.path.basename(asset_path)}",
                    Rhino.Geometry.Point3d.Origin,
                    geometries,
                    attributes,
                )
                if idef_index >= 0:
                    print(
                        f"  [import_block] imported '{block_name}' "
                        f"({len(geometries)} object(s)) from {asset_path}"
                    )
                    sc.doc.Views.Redraw()
                    return True
                print(f"  [import_block] InstanceDefinitions.Add returned {idef_index}")
            else:
                print(f"  [import_block] no geometry found in {asset_path}")
    except Exception as exc:
        print(f"  [import_block] File3dm path raised: {exc!r}")

    # ---- Attempt 2: _-Insert command -----------------------------------
    cmd_path = asset_path.replace("/", "\\")
    cmd = (
        '_-Insert _File _Yes "{path}" _Block _Enter '
        '0,0,0 _Enter 1 _Enter 0 _Enter'
    ).format(path=cmd_path)
    print(f"  [import_block] running: {cmd}")
    ok_cmd = rs.Command(cmd, echo=False)
    if ok_cmd and has_block_definition(block_name):
        # The _-Insert also placed an instance at the origin -- the caller
        # will insert at the proper location, so unselect it.
        rs.UnselectAllObjects()
        return True
    return False


def require_block_definition(
    block_name: str, *, asset_path: str | None = None, layer_name: str | None = None
) -> str:
    """Ensure *block_name* is loaded; import from *asset_path* if missing.

    *layer_name* (if given) pins imported sub-objects onto that visible layer --
    see :func:`import_block_definition_from_3dm`.
    """
    if has_block_definition(block_name):
        return block_name
    if asset_path and import_block_definition_from_3dm(
        block_name, asset_path, layer_name=layer_name
    ):
        return block_name
    raise RuntimeError(
        f"Missing required Rhino block definition '{block_name}'."
        + (f"  (Tried to import from {asset_path}.)" if asset_path else "")
    )


def refresh_block_definition(
    block_name: str, asset_path: str, *, layer_name: str | None = None
) -> None:
    """Force-reload *block_name* from *asset_path*, replacing any in-doc copy.

    ``require_block_definition`` skips the import when a definition of that
    name already exists, so edits to a block's asset file would silently keep
    the stale in-doc geometry.  This helper deletes the existing
    InstanceDefinition first (together with any instances still referencing
    it -- callers like the tool swap remove their instances beforehand anyway)
    and then re-imports from the asset.

    Args:
        block_name (str): the InstanceDefinition name to reload.
        asset_path (str): the .3dm file to import the definition from.
        layer_name (str | None): if given, pin imported sub-objects onto that
            visible layer -- see :func:`import_block_definition_from_3dm`.

    Raises:
        RuntimeError: if the asset file is missing, the delete fails, or the
            re-import does not produce the definition.
    """
    import scriptcontext as sc  # noqa: PLC0415

    asset_path = os.path.normpath(asset_path)
    if not os.path.isfile(asset_path):
        raise RuntimeError(
            f"Cannot refresh block '{block_name}': asset file not found at "
            f"{asset_path}. Re-run RSDefineRoboticTool (AssemblyTool mode) to "
            "export it."
        )

    # Delete the stale in-doc definition (and any leftover references) so the
    # import below starts clean instead of being rejected as a duplicate name.
    for idef in sc.doc.InstanceDefinitions:
        if idef is None or idef.IsDeleted or idef.Name != block_name:
            continue
        ok = sc.doc.InstanceDefinitions.Delete(idef.Index, True, True)
        if not ok:
            raise RuntimeError(
                f"Failed to delete existing block definition '{block_name}' "
                "before re-import. Check for locked instances and retry."
            )
        break

    if not import_block_definition_from_3dm(block_name, asset_path, layer_name=layer_name):
        raise RuntimeError(
            f"Failed to re-import block '{block_name}' from {asset_path}."
        )
    if not has_block_definition(block_name):
        raise RuntimeError(
            f"Block '{block_name}' still missing after re-import from "
            f"{asset_path}."
        )
    print(f"  [refresh_block] reloaded '{block_name}' from {asset_path}")
