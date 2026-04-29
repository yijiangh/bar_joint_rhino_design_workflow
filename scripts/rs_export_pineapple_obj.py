#! python 3
# venv: scaffolding_env
# r: numpy==1.24.4
# r: scipy==1.13.1
# r: compas==2.13.0
# r: compas_robots==0.6.0
"""RSExportPineappleOBJ - Export pineapple block definitions as OBJ in meters.

Walks the two block definitions named in `core.config`
(`LEFT_PINEAPPLE_BLOCK`, `RIGHT_PINEAPPLE_BLOCK`), combines all renderable
geometry inside each into a single mesh, scales it from the active Rhino
doc unit to METERS (compas convention), and writes the result to
`LEFT_PINEAPPLE_TOOL_MESH` / `RIGHT_PINEAPPLE_TOOL_MESH`.

If a target file already exists, the user is prompted per file: Reuse
the existing OBJ, or re-export and overwrite. Prompt is per file so a
half-finished session (e.g. only left was re-exported last time) doesn't
force a redo of both.

Used by the IK keyframe collision check (Phase A of the collision plan):
the OBJs become `ToolModel`s attached to AL / AR in `robot_cell`.
"""

from __future__ import annotations

import importlib
import os
import sys

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config as _config_module
from core.rhino_frame_io import doc_unit_scale_to_mm


def _reload():
    global config
    config = importlib.reload(_config_module)


_reload()


def _doc_units_to_meters_scale() -> float:
    """Multiplier from active doc units to meters."""
    return doc_unit_scale_to_mm() / 1000.0


def _find_block_def(name):
    for idef in sc.doc.InstanceDefinitions:
        if idef is not None and not idef.IsDeleted and idef.Name == name:
            return idef
    return None


def _block_def_render_mesh(block_def) -> Rhino.Geometry.Mesh | None:
    """Combine all renderable geometry inside a block definition into one mesh."""
    combined = Rhino.Geometry.Mesh()
    mp = Rhino.Geometry.MeshingParameters.Default

    nested_block = False
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
            sub_mesh = Rhino.Geometry.Mesh.CreateFromSubD(geom, 1)
            if sub_mesh:
                combined.Append(sub_mesh)
        elif isinstance(geom, Rhino.Geometry.InstanceReferenceGeometry):
            nested_block = True
        # silently skip curves, annotations, etc.

    if nested_block:
        print(
            "RSExportPineappleOBJ: warning - nested block instance(s) detected; "
            "they were skipped. Explode the pineapple block once if its visuals "
            "are split across sub-blocks."
        )
    if combined.Vertices.Count == 0:
        return None
    return combined


def _rhino_mesh_to_compas(rmesh: Rhino.Geometry.Mesh):
    """Convert a Rhino.Geometry.Mesh into a compas Mesh."""
    from compas.datastructures import Mesh

    vertices = [(float(v.X), float(v.Y), float(v.Z)) for v in rmesh.Vertices]
    faces = []
    for i in range(rmesh.Faces.Count):
        f = rmesh.Faces[i]
        if f.IsTriangle:
            faces.append([f.A, f.B, f.C])
        else:
            faces.append([f.A, f.B, f.C, f.D])
    return Mesh.from_vertices_and_faces(vertices, faces)


def _ask_reuse_or_reexport(existing_path: str) -> str | None:
    """Return 'reuse', 'reexport', or None if the user cancels."""
    answer = rs.GetString(
        f"'{os.path.basename(existing_path)}' already exists. Reuse or Reexport?",
        "Reuse",
        ["Reuse", "Reexport"],
    )
    if answer is None:
        return None
    a = answer.strip().lower()
    if a.startswith("reu"):
        return "reuse"
    if a.startswith("ree"):
        return "reexport"
    return None


def _export_one(block_name: str, output_path: str, scale: float) -> bool:
    if os.path.isfile(output_path):
        choice = _ask_reuse_or_reexport(output_path)
        if choice is None:
            print(f"RSExportPineappleOBJ: cancelled at {block_name}.")
            return False
        if choice == "reuse":
            print(f"RSExportPineappleOBJ: reusing existing {output_path}")
            return True

    block_def = _find_block_def(block_name)
    if block_def is None:
        rs.MessageBox(
            f"Block definition '{block_name}' not found in this Rhino document. "
            "Paste the pineapple block from the asset 3dm, then re-run.",
            0, "Export Pineapple OBJ",
        )
        return False

    combined = _block_def_render_mesh(block_def)
    if combined is None:
        rs.MessageBox(
            f"Block definition '{block_name}' has no renderable mesh-able geometry.",
            0, "Export Pineapple OBJ",
        )
        return False

    if abs(scale - 1.0) > 1e-12:
        sf = Rhino.Geometry.Transform.Scale(Rhino.Geometry.Point3d.Origin, scale)
        combined.Transform(sf)

    cmesh = _rhino_mesh_to_compas(combined)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cmesh.to_obj(output_path)
    print(
        f"RSExportPineappleOBJ: '{block_name}' -> {output_path} "
        f"(verts={cmesh.number_of_vertices()}, faces={cmesh.number_of_faces()}, scale={scale:g})"
    )
    return True


def main() -> None:
    _reload()
    scale = _doc_units_to_meters_scale()
    targets = [
        (config.LEFT_PINEAPPLE_BLOCK, config.LEFT_PINEAPPLE_TOOL_MESH),
        (config.RIGHT_PINEAPPLE_BLOCK, config.RIGHT_PINEAPPLE_TOOL_MESH),
    ]
    successes = 0
    for block_name, out_path in targets:
        if _export_one(block_name, out_path, scale):
            successes += 1
    print(f"RSExportPineappleOBJ: {successes}/{len(targets)} export(s) OK.")


if __name__ == "__main__":
    main()
