"""A tiny fake of the Rhino API (`rhinoscriptsyntax`, `scriptcontext`, `Rhino`): just what `core.design_source` calls.

`install(doc)` puts the fake modules into `sys.modules`; `doc` holds the objects. Units are mm. Walkable ground is a
horizontal rectangle at z = 0.
"""

from __future__ import annotations

import math
import sys
import types
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class Point3d:
    X: float
    Y: float
    Z: float

    def DistanceTo(self, other) -> float:  # noqa: N802 (RhinoCommon name)
        return math.dist((self.X, self.Y, self.Z), (other.X, other.Y, other.Z))


@dataclass
class Vector3d:
    X: float
    Y: float
    Z: float


class Transform:
    """A 4x4 transform indexed as `xform[row, column]`."""

    def __init__(self, matrix):
        self.matrix = np.asarray(matrix, dtype=float)

    def __getitem__(self, key):
        return self.matrix[key]


class InstanceObject:
    def __init__(self, xform):
        self.InstanceXform = Transform(xform)


@dataclass
class Face:
    A: int
    B: int
    C: int
    D: int


class Mesh:
    def __init__(self, vertices=(), faces=()):
        self.Vertices = _Items(Point3d(*v) for v in vertices)
        self.Faces = _Items(Face(*f) for f in faces)

    def Append(self, other) -> None:  # noqa: N802
        offset = self.Vertices.Count
        self.Vertices.extend(other.Vertices)
        self.Faces.extend(Face(f.A + offset, f.B + offset, f.C + offset, f.D + offset) for f in other.Faces)

    @staticmethod
    def CreateFromBrep(brep, _parameters):  # noqa: N802
        x0, x1, y0, y1 = brep.extent
        return [Mesh([(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)], [(0, 1, 2, 3)])]


class _Items(list):
    @property
    def Count(self) -> int:  # noqa: N802
        return len(self)


class Brep:
    """A horizontal rectangle at z = 0."""

    def __init__(self, extent):
        self.extent = extent

    def ClosestPoint(self, point, maximum_distance=None):  # noqa: N802
        x0, x1, y0, y1 = self.extent
        closest = Point3d(min(max(point.X, x0), x1), min(max(point.Y, y0), y1), 0.0)
        if maximum_distance is None:
            return closest
        return True, closest, None, 0.0, 0.0, Vector3d(0.0, 0.0, 1.0)


@dataclass
class Obj:
    """One document object: a curve (`start`, `end`), a block instance (`block`, `xform`) or a brep (`brep`)."""

    layer: str
    text: Dict[str, str] = field(default_factory=dict)
    name: str = ""
    start: Optional[tuple] = None
    end: Optional[tuple] = None
    block: str = ""
    xform: Optional[np.ndarray] = None
    brep: Optional[Brep] = None


class Doc:
    def __init__(self):
        self.objects: Dict[str, Obj] = {}

    def add(self, obj: Obj) -> str:
        oid = f"oid{len(self.objects)}"
        self.objects[oid] = obj
        return oid


def install(doc: Doc) -> None:
    """Put fake `rhinoscriptsyntax`, `scriptcontext` and `Rhino` modules over `doc` into `sys.modules`."""
    rs = types.ModuleType("rhinoscriptsyntax")
    rs.AllObjects = lambda: list(doc.objects)
    rs.GetUserText = lambda oid, key: doc.objects[oid].text.get(key)
    rs.ObjectsByLayer = lambda layer: [oid for oid, o in doc.objects.items() if o.layer == layer]
    rs.IsLayer = lambda layer: any(o.layer == layer for o in doc.objects.values())
    rs.ObjectName = lambda oid: doc.objects[oid].name
    rs.BlockInstanceName = lambda oid: doc.objects[oid].block
    rs.CurveStartPoint = lambda oid: Point3d(*doc.objects[oid].start)
    rs.CurveEndPoint = lambda oid: Point3d(*doc.objects[oid].end)
    rs.coerceguid = lambda oid: oid
    rs.coercerhinoobject = lambda oid, *args: InstanceObject(doc.objects[oid].xform) if doc.objects[oid].block \
        else doc.objects[oid]
    rs.coercebrep = lambda oid: doc.objects[oid].brep
    rs.IsMesh = lambda oid: False
    sc = types.ModuleType("scriptcontext")
    sc.sticky = {}
    sc.doc = types.SimpleNamespace(ModelUnitSystem="mm")
    rhino = types.ModuleType("Rhino")
    rhino.RhinoMath = types.SimpleNamespace(UnitScale=lambda _from, _to: 1.0)
    rhino.UnitSystem = types.SimpleNamespace(Millimeters="mm")
    rhino.DocObjects = types.SimpleNamespace(InstanceObject=InstanceObject)
    rhino.Geometry = types.SimpleNamespace(Point3d=Point3d, Mesh=Mesh, Extrusion=type("Extrusion", (), {}),
                                           MeshingParameters=types.SimpleNamespace(Coarse=None))
    sys.modules.update({"rhinoscriptsyntax": rs, "scriptcontext": sc, "Rhino": rhino})


def uninstall() -> List[str]:
    """Remove the fake modules and every `core` module that imported them."""
    names = ["rhinoscriptsyntax", "scriptcontext", "Rhino"] + [
        n for n in sys.modules if n.startswith("core.rhino") or n in ("core.design_source",)]
    for name in names:
        sys.modules.pop(name, None)
    return names


def doc_from_3dm(path) -> Doc:
    """A real .3dm file's curves, block instances and user text, read with rhino3dm (`pip install rhino3dm`).

    ? Breps are not read: walkable ground and obstacles come out empty.
    """
    import rhino3dm

    model = rhino3dm.File3dm.Read(str(path))
    layers = {i: layer.FullPath for i, layer in enumerate(model.Layers)}
    blocks = {str(d.Id): d.Name for d in model.InstanceDefinitions}
    doc = Doc()
    for o in model.Objects:
        attributes, geometry = o.Attributes, o.Geometry
        obj = Obj(layers.get(attributes.LayerIndex, ""), dict(attributes.GetUserStrings()), name=attributes.Name or "")
        if isinstance(geometry, rhino3dm.InstanceReference):
            obj.block = blocks.get(str(geometry.ParentIdefId), "")
            obj.xform = np.array(geometry.Xform.ToFloatArray(True), dtype=float).reshape(4, 4)
        elif isinstance(geometry, rhino3dm.Curve):
            start, end = geometry.PointAtStart, geometry.PointAtEnd
            obj.start, obj.end = (start.X, start.Y, start.Z), (end.X, end.Y, end.Z)
        doc.objects[str(attributes.Id)] = obj
    return doc
