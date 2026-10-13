#! python 3
# Python packages: requirements-rhino.txt, installed and put on sys.path by `rhino_env.activate()` below.
"""Manual benchmark: ways to turn a robot's visual meshes into Rhino meshes (`core.ik_viz._rhino_mesh`).

The first time a robot is drawn, `ik_viz` builds its Rhino meshes with one Python -> Rhino call per vertex and per
face; a support robot has about 0.7 million vertices. This times that against the alternatives, on one robot of the
open document's design, and prints the results (they also land in the session log). Nothing is added to the
document.

Run in Rhino with a design document open: `_-ScriptEditor _R "_dev_mesh_bake_benchmark.py"`.
"""

from __future__ import annotations

import os
import sys
import tempfile
import time

import Rhino
import scriptcontext as sc

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rhino_env  # noqa: E402  (before any package import: requirements-rhino.txt)

rhino_env.activate()

import numpy as np  # noqa: E402

import session_log  # noqa: E402
from core import core_lib  # noqa: E402, F401  (puts bar_assembly_core on sys.path)
from core import design_source, ik_viz  # noqa: E402

ROBOT = "robots/alice"


def _arrays(robot, scale):
    """(vertices, faces) per link of the robot's visual shapes, in document units."""
    from bar_assembly_core.geometry import shape_mesh

    out = []
    for geometry in robot.model.link_geometry.values():
        for shape in geometry.visual or ():
            triangles = shape_mesh(shape)
            out.append((np.asarray(triangles.vertices, dtype=float) * scale, np.asarray(triangles.faces, dtype=int)))
    return out


def _new_mesh():
    mesh = Rhino.Geometry.Mesh()
    mesh.Vertices.UseDoublePrecisionVertices = True
    return mesh


def per_vertex(arrays):
    """What ik_viz does now."""
    meshes = []
    for vertices, faces in arrays:
        mesh = _new_mesh()
        for x, y, z in vertices.tolist():
            mesh.Vertices.Add(x, y, z)
        for a, b, c in faces.tolist():
            mesh.Faces.AddFace(a, b, c)
        mesh.Normals.ComputeNormals()
        mesh.Compact()
        meshes.append(mesh)
    return meshes


def welded(arrays):
    """Equal vertices merged with numpy first: fewer calls, but smooth shading across hard edges."""
    meshes = []
    for vertices, faces in arrays:
        unique, inverse = np.unique(np.round(vertices, 6), axis=0, return_inverse=True)
        mesh = _new_mesh()
        for x, y, z in unique.tolist():
            mesh.Vertices.Add(x, y, z)
        for a, b, c in inverse.reshape(-1)[faces].tolist():
            mesh.Faces.AddFace(a, b, c)
        mesh.Normals.ComputeNormals()
        mesh.Compact()
        meshes.append(mesh)
    return meshes


def bulk_lists(arrays):
    """One AddVertices / AddFaces call per mesh, on lists of Point3d / MeshFace built in Python."""
    meshes = []
    for vertices, faces in arrays:
        mesh = _new_mesh()
        mesh.Vertices.AddVertices([Rhino.Geometry.Point3d(x, y, z) for x, y, z in vertices.tolist()])
        mesh.Faces.AddFaces([Rhino.Geometry.MeshFace(a, b, c) for a, b, c in faces.tolist()])
        mesh.Normals.ComputeNormals()
        mesh.Compact()
        meshes.append(mesh)
    return meshes


def _count(meshes):
    return sum(m.Vertices.Count for m in meshes), sum(m.Faces.Count for m in meshes)


def main() -> None:
    result = design_source.RhinoDesignSource("MeshBakeBenchmark", design_source.geometry_cache(sc.sticky)).build()
    design = result.design
    movement = next((m for _action, m in design.movements()), None)
    if movement is None or ROBOT not in design.robots:
        print(f"MeshBakeBenchmark: the design has no movement or no {ROBOT}.")
        return
    robot = design.scene_at(movement).robots[ROBOT]
    with session_log.step("read robot meshes"):
        arrays = _arrays(robot, ik_viz._doc_units_per_metre())
    n_vertices, n_faces = sum(len(v) for v, _f in arrays), sum(len(f) for _v, f in arrays)
    print(f"MeshBakeBenchmark: {ROBOT}: {len(arrays)} meshes, {n_vertices:,} vertices, {n_faces:,} faces")

    built = None
    for name, method in (("per vertex (now)", per_vertex), ("welded", welded), ("bulk lists", bulk_lists)):
        started = time.perf_counter()
        try:
            with session_log.step(name):
                meshes = method(arrays)
        except Exception as exc:  # noqa: BLE001 -- a method Rhino does not support must not stop the others
            session_log.caught(exc)
            print(f"MeshBakeBenchmark: {name:18} FAILED: {type(exc).__name__}: {exc}")
            continue
        vertices, faces = _count(meshes)
        print(f"MeshBakeBenchmark: {name:18} {time.perf_counter() - started:6.2f} s   "
              f"-> {vertices:,} vertices, {faces:,} faces")
        if method is per_vertex:
            built = meshes

    if built:  # a mesh file per robot, written once and read on later sessions
        path = os.path.join(tempfile.gettempdir(), "bar_joint_mesh_benchmark.3dm")
        try:
            started = time.perf_counter()
            with session_log.step("write .3dm cache"):
                file = Rhino.FileIO.File3dm()
                for mesh in built:
                    file.Objects.AddMesh(mesh)
                file.Write(path, 8)
            wrote = time.perf_counter() - started
            started = time.perf_counter()
            with session_log.step("read .3dm cache"):
                read = [obj.Geometry for obj in Rhino.FileIO.File3dm.Read(path).Objects]
            vertices, faces = _count(read)
            print(f"MeshBakeBenchmark: {'.3dm cache':18} write {wrote:.2f} s, read {time.perf_counter() - started:.2f} s "
                  f"({os.path.getsize(path) / 1e6:.1f} MB)   -> {vertices:,} vertices, {faces:,} faces")
        except Exception as exc:  # noqa: BLE001
            session_log.caught(exc)
            print(f"MeshBakeBenchmark: .3dm cache FAILED: {type(exc).__name__}: {exc}")
        finally:
            if os.path.isfile(path):
                os.remove(path)


if __name__ == "__main__":
    session_log.run(__file__, main)
