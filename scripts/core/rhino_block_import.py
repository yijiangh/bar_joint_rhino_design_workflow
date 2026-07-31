"""Shared Rhino block-definition import helpers.

Both joint placement and robotic-tool placement need the same logic to
load an InstanceDefinition by name from an asset .3dm file.  This module
hosts the canonical implementation; both paths import from here.

Rhino-runtime-only at call time; safe to import from non-Rhino contexts
since the Rhino imports are deferred into each function body.
"""

from __future__ import annotations

import os


# Marker embedded in an InstanceDefinition's Description so a later run can tell
# whether the source asset file has changed since the block was imported.  See
# :func:`asset_stamp` / :func:`block_asset_stamp`.
_STAMP_PREFIX = "asset_stamp="


def asset_stamp(asset_path: str) -> str:
    """Return a cheap change-marker for *asset_path*: ``"<mtime_ns>:<size>"``.

    Used instead of hashing the .3dm: re-exporting a block always bumps the
    modification time, and comparing two short strings costs nothing, so
    RSUpdatePreview can skip the (destructive) re-import of blocks whose asset
    file has not moved.

    Args:
        asset_path (str): the .3dm the block definition was imported from.

    Returns:
        str: the stamp, or ``""`` when the file is missing / unreadable.
    """
    try:
        stat = os.stat(os.path.normpath(asset_path))
    except OSError:
        return ""
    return f"{stat.st_mtime_ns}:{stat.st_size}"


def block_asset_stamp(block_name: str) -> str | None:
    """Return the stamp recorded on *block_name*'s definition, if any.

    Args:
        block_name (str): the InstanceDefinition name to inspect.

    Returns:
        str | None: the stamp written by :func:`import_block_definition_from_3dm`,
        or ``None`` when the block is absent or carries no stamp (a document
        whose blocks were imported before stamping existed -- callers treat that
        as "unknown, refresh once").
    """
    import scriptcontext as sc  # noqa: PLC0415

    for instance_def in sc.doc.InstanceDefinitions:
        if (
            instance_def is None
            or instance_def.IsDeleted
            or instance_def.Name != block_name
        ):
            continue
        description = instance_def.Description or ""
        _, _, stamp = description.partition(_STAMP_PREFIX)
        return stamp.strip() or None
    return None


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


def _read_asset_geometry(asset_path: str, layer_name: str | None):
    """Read every geometry/attribute pair out of *asset_path*.

    Shared by the import path (which registers them as a new InstanceDefinition)
    and the in-place update path (which swaps them into an existing one).

    Args:
        asset_path (str): a .3dm file, already normalised and known to exist.
        layer_name (str | None): if given, pin EVERY sub-object onto this
            (visible) layer -- see :func:`import_block_definition_from_3dm`.

    Returns:
        tuple[list, list] | None: ``(geometries, attributes)``, or ``None`` when
        the file cannot be read or holds no geometry (the reason is printed).
    """
    import Rhino  # noqa: PLC0415

    target_layer_index = (
        _resolve_visible_layer_index(layer_name) if layer_name else None
    )
    try:
        file3dm = Rhino.FileIO.File3dm.Read(asset_path)
    except Exception as exc:  # noqa: BLE001 - any read failure falls back
        print(f"  [import_block] File3dm path raised: {exc!r}")
        return None
    if file3dm is None:
        print(f"  [import_block] File3dm.Read returned None for {asset_path}")
        return None

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
        # Force the sub-object onto a known visible layer (see the caller's
        # docstring): otherwise the source file's layer index makes it invisible.
        if target_layer_index is not None:
            attr.LayerIndex = target_layer_index
        attributes.append(attr)

    if not geometries:
        print(f"  [import_block] no geometry found in {asset_path}")
        return None
    return geometries, attributes


def update_block_definition_geometry(
    block_name: str, asset_path: str, *, layer_name: str | None = None
) -> bool:
    """Replace *block_name*'s geometry IN PLACE from *asset_path*.

    The safe way to pick up an edited asset in an open document.  Unlike
    :func:`refresh_block_definition`, nothing is deleted: RhinoCommon's
    ``InstanceDefinitions.ModifyGeometry`` swaps the definition's geometry and
    re-points every existing instance at it, so instance ids, world transforms,
    object names and user text all survive by construction -- and the
    ``_-Insert`` fallback (which strands an extra instance at the world origin)
    is never reached.

    Args:
        block_name (str): an InstanceDefinition that must already exist.
        asset_path (str): the .3dm to take the new geometry from.
        layer_name (str | None): pin imported sub-objects onto this layer.

    Returns:
        bool: True when the definition was updated.  False (with a printed
        reason) when the block is absent, the asset is unreadable, or Rhino
        rejects the modification -- callers treat that as "leave it alone".
    """
    import scriptcontext as sc  # noqa: PLC0415

    asset_path = os.path.normpath(asset_path)
    if not os.path.isfile(asset_path):
        print(f"  [update_block] asset file not found: {asset_path}")
        return False

    target = None
    for instance_def in sc.doc.InstanceDefinitions:
        if (
            instance_def is not None
            and not instance_def.IsDeleted
            and instance_def.Name == block_name
        ):
            target = instance_def
            break
    if target is None:
        print(f"  [update_block] block '{block_name}' is not in this document.")
        return False

    parts = _read_asset_geometry(asset_path, layer_name)
    if parts is None:
        return False
    geometries, attributes = parts

    if not sc.doc.InstanceDefinitions.ModifyGeometry(
        target.Index, geometries, attributes
    ):
        print(f"  [update_block] ModifyGeometry rejected '{block_name}'.")
        return False

    # Re-stamp so the next staleness check sees the file we just read in.
    _write_asset_stamp(block_name, asset_path)
    sc.doc.Views.Redraw()
    print(
        f"  [update_block] updated '{block_name}' in place "
        f"({len(geometries)} object(s)) from {asset_path}"
    )
    return True


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

    # ---- Attempt 1: RhinoCommon File3dm direct read --------------------
    parts = _read_asset_geometry(asset_path, layer_name)
    if parts is not None:
        geometries, attributes = parts
        try:
            # The description doubles as the staleness record: RSUpdatePreview
            # compares the embedded stamp against the file on disk to decide
            # whether this definition needs reloading.
            idef_index = sc.doc.InstanceDefinitions.Add(
                block_name,
                f"Imported from {os.path.basename(asset_path)} "
                f"| {_STAMP_PREFIX}{asset_stamp(asset_path)}",
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
        except Exception as exc:  # noqa: BLE001 - fall through to _-Insert
            print(f"  [import_block] InstanceDefinitions.Add raised: {exc!r}")

    # ---- Attempt 2: _-Insert command -----------------------------------
    cmd_path = asset_path.replace("/", "\\")
    cmd = (
        '_-Insert _File _Yes "{path}" _Block _Enter '
        '0,0,0 _Enter 1 _Enter 0 _Enter'
    ).format(path=cmd_path)
    print(f"  [import_block] running: {cmd}")
    ok_cmd = rs.Command(cmd, echo=False)
    if ok_cmd and has_block_definition(block_name):
        # `_-Insert` also PLACES an instance at the origin. The caller inserts at
        # the proper location itself, so that one has to go -- merely unselecting
        # it (as this did originally) strands an untagged block at 0,0,0 on every
        # fallback import, which then shows up as a phantom joint.
        for stray in rs.SelectedObjects() or []:
            if rs.IsBlockInstance(stray) and rs.BlockInstanceName(stray) == block_name:
                rs.DeleteObject(stray)
        rs.UnselectAllObjects()
        # `_-Insert` writes its own description, so stamp the definition here
        # too; without it the block would read as "unknown" and be re-imported
        # on every RSUpdatePreview.
        _write_asset_stamp(block_name, asset_path)
        return True
    return False


def _write_asset_stamp(block_name: str, asset_path: str) -> None:
    """Record *asset_path*'s stamp on an already-created block definition.

    Only needed for the ``_-Insert`` fallback above; the File3dm path writes the
    stamp as part of ``InstanceDefinitions.Add``.  Failures are non-fatal -- the
    block is usable, it will just be treated as stale once more.
    """
    import scriptcontext as sc  # noqa: PLC0415

    stamp = asset_stamp(asset_path)
    description = (
        f"Imported from {os.path.basename(asset_path)} | {_STAMP_PREFIX}{stamp}"
    )
    for instance_def in sc.doc.InstanceDefinitions:
        if (
            instance_def is None
            or instance_def.IsDeleted
            or instance_def.Name != block_name
        ):
            continue
        try:
            sc.doc.InstanceDefinitions.Modify(
                instance_def, block_name, description, True
            )
        except Exception as exc:  # noqa: BLE001 - cosmetic bookkeeping only
            print(f"  [import_block] could not stamp '{block_name}': {exc!r}")
        return


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
            f"{asset_path}. Re-run the command that exports it "
            "(RSDefineRoboticTool in AssemblyTool mode for a tool, "
            "RSDefineJointHalf for a joint half)."
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
