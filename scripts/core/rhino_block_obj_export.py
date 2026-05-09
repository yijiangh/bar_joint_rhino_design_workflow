"""Export a Rhino block definition's renderable geometry as a single .obj
mesh in MILLIMETERS, suitable for use as a `compas_fab.robots.RigidBody`
collision mesh.

The OBJ origin coincides with the block definition's local frame, so the
mesh can be attached at the block's pose without any extra transform.

Mirrors the approach used by `rs_export_pineapple_obj.py`, but writes
millimetres (not metres) because joint-half collision meshes live in the
same unit system as the joint registry (`scripts/core/joint_pairs.json`).
"""

from __future__ import annotations

import os


def export_block_definition_to_obj_mm(block_name: str, output_path: str) -> bool:
    """Write a single-mesh OBJ of the given block definition (mm).

    Returns True on success.  Verbose progress is printed.
    """
    import Rhino  # noqa: PLC0415
    import scriptcontext as sc  # noqa: PLC0415

    from core.rhino_frame_io import doc_unit_scale_to_mm  # noqa: PLC0415

    output_path = os.path.normpath(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    block_def = None
    for idef in sc.doc.InstanceDefinitions:
        if idef is not None and not idef.IsDeleted and idef.Name == block_name:
            block_def = idef
            break
    if block_def is None:
        print(f"    [export_obj] block definition '{block_name}' not found.")
        return False

    combined = Rhino.Geometry.Mesh()
    mp = Rhino.Geometry.MeshingParameters.Default
    nested = False
    for rh_obj in block_def.GetObjects():
        geom = rh_obj.Geometry
        if isinstance(geom, Rhino.Geometry.Mesh):
            combined.Append(geom)
        elif isinstance(geom, Rhino.Geometry.Brep):
            for m in (Rhino.Geometry.Mesh.CreateFromBrep(geom, mp) or []):
                combined.Append(m)
        elif isinstance(geom, Rhino.Geometry.Extrusion):
            brep = geom.ToBrep(False)
            for m in (Rhino.Geometry.Mesh.CreateFromBrep(brep, mp) or []):
                combined.Append(m)
        elif isinstance(geom, Rhino.Geometry.SubD):
            sub = Rhino.Geometry.Mesh.CreateFromSubD(geom, 1)
            if sub:
                combined.Append(sub)
        elif isinstance(geom, Rhino.Geometry.InstanceReferenceGeometry):
            nested = True

    if nested:
        print(f"    [export_obj] warning: nested block instances inside '{block_name}' were skipped.")
    if combined.Vertices.Count == 0:
        print(f"    [export_obj] block '{block_name}' has no mesh-able geometry.")
        return False

    scale_to_mm = doc_unit_scale_to_mm()
    if abs(scale_to_mm - 1.0) > 1e-12:
        sf = Rhino.Geometry.Transform.Scale(Rhino.Geometry.Point3d.Origin, scale_to_mm)
        combined.Transform(sf)

    from compas.datastructures import Mesh  # noqa: PLC0415

    vertices = [(float(v.X), float(v.Y), float(v.Z)) for v in combined.Vertices]
    faces = []
    for i in range(combined.Faces.Count):
        f = combined.Faces[i]
        if f.IsTriangle:
            faces.append([f.A, f.B, f.C])
        else:
            faces.append([f.A, f.B, f.C, f.D])
    cmesh = Mesh.from_vertices_and_faces(vertices, faces)
    cmesh.to_obj(output_path)
    print(
        f"    [export_obj] '{block_name}' -> {output_path} "
        f"(verts={cmesh.number_of_vertices()}, faces={cmesh.number_of_faces()}, scale_to_mm={scale_to_mm:g})"
    )
    return True
