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


def import_block_definition_from_3dm(block_name: str, asset_path: str) -> bool:
    """Import a single block definition named *block_name* from *asset_path*.

    Strategy: read the .3dm via :class:`Rhino.FileIO.File3dm`, gather every
    geometry/attribute pair, and register them as one InstanceDefinition
    under *block_name*.  Falls back to Rhino's ``_-Insert`` command if the
    File3dm path fails.  Returns True on success.
    """
    import Rhino  # noqa: PLC0415
    import rhinoscriptsyntax as rs  # noqa: PLC0415
    import scriptcontext as sc  # noqa: PLC0415

    asset_path = os.path.normpath(asset_path)
    if not os.path.isfile(asset_path):
        print(f"  [import_block] asset file not found: {asset_path}")
        return False

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
                attributes.append(obj.Attributes.Duplicate() if obj.Attributes else None)
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


def require_block_definition(block_name: str, *, asset_path: str | None = None) -> str:
    """Ensure *block_name* is loaded; import from *asset_path* if missing."""
    if has_block_definition(block_name):
        return block_name
    if asset_path and import_block_definition_from_3dm(block_name, asset_path):
        return block_name
    raise RuntimeError(
        f"Missing required Rhino block definition '{block_name}'."
        + (f"  (Tried to import from {asset_path}.)" if asset_path else "")
    )
