"""`core.ik_viz` on a fake Rhino canvas: parts baked once by id, moved by the change in their transform, hidden, rebaked.

The canvas fakes the few `rhinoscriptsyntax`, `scriptcontext` and `Rhino` calls the cache makes: layers with
visibility, mesh objects with a name, a layer, visibility and vertices that transforms really move.
"""

import importlib
import sys
import types

import numpy as np
import pytest

from test_process_source import source  # noqa: F401  (fixture: the fake document's process_source module)


class _Vertices(list):
    UseDoublePrecisionVertices = False

    @property
    def Count(self):  # noqa: N802 (RhinoCommon name)
        return len(self)

    def Add(self, x, y, z):  # noqa: N802
        self.append((x, y, z))


class _Faces(list):
    def AddFace(self, a, b, c):  # noqa: N802
        self.append((a, b, c))


class Mesh:
    def __init__(self):
        self.Vertices, self.Faces = _Vertices(), _Faces()
        self.Normals = types.SimpleNamespace(ComputeNormals=lambda: None)

    def Compact(self):  # noqa: N802
        pass

    def DuplicateMesh(self):  # noqa: N802
        copy = Mesh()
        copy.Vertices.extend(self.Vertices)
        copy.Faces.extend(self.Faces)
        return copy

    def Transform(self, xform):  # noqa: N802
        points = np.c_[np.asarray(self.Vertices, dtype=float), np.ones(len(self.Vertices))] @ xform.m.T
        self.Vertices[:] = [tuple(p) for p in points[:, :3]]

    @property
    def points(self):
        return np.asarray(self.Vertices, dtype=float)


class Transform:
    def __init__(self, diagonal):
        self.m = np.eye(4) * diagonal

    def __setitem__(self, key, value):
        self.m[key] = value


class Canvas:
    """Layers and mesh objects; what the cache draws."""

    def __init__(self):
        self.layers = {}  # path -> visible
        self.objects = {}  # guid -> {"mesh", "layer", "name", "visible"}
        self.added = 0

    def add_layer(self, path):
        self.layers.setdefault(path, True)

    def add_mesh(self, mesh, attributes):
        guid = f"m{self.added}"
        self.added += 1
        layer = next(path for i, path in enumerate(self.layers) if i == attributes.LayerIndex)
        self.objects[guid] = {"mesh": mesh.DuplicateMesh(), "layer": layer, "name": attributes.Name, "visible": True}
        return guid

    def named(self, name):
        return [o for o in self.objects.values() if o["name"] == name]

    def shown(self, obj):
        """Whether an object is on screen: visible itself and every layer above it."""
        parts = obj["layer"].split("::")
        return obj["visible"] and all(self.layers["::".join(parts[:i + 1])] for i in range(len(parts)))


@pytest.fixture
def canvas(source):  # noqa: F811
    """The fake document's process, then its fake modules with a drawing canvas added; `core.ik_viz` imported fresh."""
    canvas = Canvas()
    canvas.process = source.RhinoProcessSource("x", source.geometry_cache({})).build().process
    rs, sc, rhino = (sys.modules[n] for n in ("rhinoscriptsyntax", "scriptcontext", "Rhino"))
    rs.IsLayer = lambda path: path in canvas.layers
    rs.AddLayer = canvas.add_layer
    rs.LayerVisible = lambda path, visible=None: canvas.layers[path] if visible is None \
        else canvas.layers.__setitem__(path, bool(visible))
    rs.LayerChildren = lambda path: [p for p in canvas.layers if p.rsplit("::", 1)[0] == path and p != path]
    rs.ObjectsByLayer = lambda path: [g for g, o in canvas.objects.items() if o["layer"] == path]
    # ik_viz deletes through RhinoCommon (hidden objects too): FindId, then Delete(obj, quiet, ignoreModes).
    sc.doc.Objects.FindId = lambda guid: guid if guid in canvas.objects else None
    sc.doc.Objects.Delete = lambda guid, _quiet, _ignore_modes: canvas.objects.pop(guid, None) is not None
    rs.HideObjects = lambda guids: [canvas.objects[g].__setitem__("visible", False) for g in guids]
    rs.ShowObjects = lambda guids: [canvas.objects[g].__setitem__("visible", True) for g in guids]

    def transform(guids, xform):
        for guid in guids:
            canvas.objects[guid]["mesh"].Transform(xform)
        return list(guids)

    rs.TransformObjects = transform
    sc.doc.RuntimeSerialNumber = 1
    sc.doc.Views = types.SimpleNamespace(RedrawEnabled=True, Redraw=lambda: None)
    sc.doc.Layers = types.SimpleNamespace(FindByFullPath=lambda path, _missing: list(canvas.layers).index(path))
    sc.doc.Objects.AddMesh = canvas.add_mesh
    rhino.Geometry.Mesh, rhino.Geometry.Transform = Mesh, Transform
    rhino.DocObjects.ObjectAttributes = lambda: types.SimpleNamespace(LayerIndex=-1, Name="")
    for name in ("core.ik_viz", "core.rhino_helpers"):
        sys.modules.pop(name, None)
    canvas.ik_viz = importlib.import_module("core.ik_viz")
    canvas.sticky = sc.sticky
    yield canvas
    for name in ("core.ik_viz", "core.rhino_helpers"):
        sys.modules.pop(name, None)


def _movement(process, movement_id):
    return next(m for a in process.actions.values() for m in a.movements if m.id == movement_id)


def _expected(geometry, pose, mode="visual"):
    """A part's mesh points where its pose puts them, millimetres (the fake document's unit)."""
    from scaffolding_assembly_core.geometry import shape_mesh

    shapes = geometry.visual if mode == "visual" else geometry.collision
    points = np.vstack([shape_mesh(shape).vertices for shape in shapes])
    matrix = pose.matrix()
    return (points @ matrix[:3, :3].T + matrix[:3, 3]) * 1000.0


def test_update_state_bakes_moves_hides_and_rebakes(canvas):
    from scaffolding_assembly_core.geometry import Geometry, Pose
    from scaffolding_assembly_core.scene import scene_parts
    from core import config, process_display

    ik_viz = canvas.ik_viz
    process = canvas.process
    scene = process.scene_at(_movement(process, "B1_J_insert"))
    process_display.place(scene, "robots/cindy", Pose((1.0, 0.5, 0.0)), {"left_ur_arm_elbow_joint": 0.4})

    ik_viz.begin_session(active_mesh_mode=ik_viz.MESH_MODE_VISUAL, layer_key="Assembly", hide_doc_layers=())
    ik_viz.update_state(scene, robot_id="robots/cindy", layer_key="Assembly")
    wrist = "robots/cindy/left_ur_arm_wrist_3_link"
    parts = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(scene)}
    count = len(canvas.objects)
    for part_id in (wrist, "bars/B1", "ground/WG0"):
        (visual,) = [o for o in canvas.named(part_id) if o["layer"].endswith("::Visual")]
        assert visual["layer"] == f"{config.LAYER_IK_CACHE}::Assembly::Visual" and canvas.shown(visual)
        assert np.allclose(visual["mesh"].points, _expected(*parts[part_id]), atol=1e-6)
    assert not canvas.named("robots/alice/ur_arm_wrist_3_link")  # absent: never baked

    # Moving the robot moves its links and the bar it holds, by the change in their transform; nothing is baked.
    process_display.place(scene, "robots/cindy", Pose((2.0, 0.0, 0.0), (0.0, 0.0, 0.7071068, 0.7071068)),
                         {"left_ur_arm_elbow_joint": -0.8})
    ik_viz.update_state(scene, robot_id="robots/cindy", layer_key="Assembly")
    assert len(canvas.objects) == count
    parts = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(scene)}
    for part_id in (wrist, "bars/B1"):
        visual = next(o for o in canvas.named(part_id) if o["layer"].endswith("::Visual"))
        assert np.allclose(visual["mesh"].points, _expected(*parts[part_id]), atol=1e-6)

    # Mesh mode: a sub-layer flip.
    ik_viz.set_active_mesh_mode("Assembly", ik_viz.MESH_MODE_COLLISION)
    collision = next(o for o in canvas.named(wrist) if o["layer"].endswith("::Collision"))
    assert canvas.shown(collision) and not canvas.shown(next(o for o in canvas.named(wrist)
                                                                if o["layer"].endswith("::Visual")))
    assert np.allclose(collision["mesh"].points, _expected(*parts[wrist], mode="collision"), atol=1e-6)
    assert ik_viz.objects_of(["robots/cindy/left_ur_arm_wrist_3_link"], "Assembly", "collision") == \
        [g for g, o in canvas.objects.items() if o is collision]
    whole = ik_viz.objects_of(["robots/cindy"], "Assembly", "collision")
    assert len(whole) == sum(1 for o in canvas.objects.values()
                             if o["layer"].endswith("::Collision") and o["name"].startswith(("robots/cindy/", "tools/")))

    # Taken out of the scene: hidden, not deleted. A disabled body too.
    process_display.place(scene, "robots/cindy", enabled=False)
    scene.bodies["ground/WG0"].enabled = False
    ik_viz.update_state(scene, robot_id="robots/cindy", layer_key="Assembly")
    assert len(canvas.objects) == count
    assert not any(canvas.shown(o) for name in (wrist, "bars/B1", "ground/WG0") for o in canvas.named(name))

    # A new Geometry object is baked again (once); a part whose id leaves the scene is deleted.
    process_display.place(scene, "robots/cindy")
    old = scene.bodies["bars/B1"].geometry
    scene.bodies["bars/B1"].geometry = Geometry(old.visual, old.collision)
    del scene.bodies["ground/WG0"]
    before = [g for g, o in canvas.objects.items() if o["name"] == "bars/B1"]
    ik_viz.update_state(scene, robot_id="robots/cindy", layer_key="Assembly")
    after = [g for g, o in canvas.objects.items() if o["name"] == "bars/B1"]
    assert len(after) == len(before) and not set(before) & set(after) and not canvas.named("ground/WG0")
    parts = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(scene)}
    visual = next(o for o in canvas.named("bars/B1") if o["layer"].endswith("::Visual"))
    assert canvas.shown(visual) is False  # the collision set is the one shown
    assert np.allclose(visual["mesh"].points, _expected(*parts["bars/B1"]), atol=1e-6)

    # Another preview layer: this one is hidden while it shows.
    ik_viz.update_state(scene, robot_id="robots/cindy", layer_key="SupportSolve")
    assert not canvas.layers[f"{config.LAYER_IK_CACHE}::Assembly"]
    assert canvas.layers[f"{config.LAYER_IK_CACHE}::SupportSolve"]

    ik_viz.discard_cache()
    assert not canvas.objects


def test_robot_meshes_are_copies_at_the_robot(canvas):
    from scaffolding_assembly_core.scene import robot_parts
    from core import process_display

    ik_viz = canvas.ik_viz
    process = canvas.process
    ghost = process_display.ghost_scene(process.scene_at(_movement(process, "B1_J_insert")), "robots/alice")
    alice = ghost.robots["robots/alice"]
    meshes = ik_viz.robot_meshes(alice)
    expected = [_expected(g, p) if g.visual else _expected(g, p, "collision") for _i, g, p in robot_parts(alice)
                if g.visual or g.collision]
    assert len(meshes) == len(expected) and not canvas.objects
    assert all(np.allclose(m.points, e, atol=1e-6) for m, e in zip(meshes, expected))


def test_update_state_colours_parts_by_id_and_by_robot(canvas):
    """A part takes its own id's colour, else its robot's; without one it goes back to its layer's."""
    ik_viz = canvas.ik_viz
    rs = sys.modules["rhinoscriptsyntax"]
    calls = []

    def object_color(guids, color):
        calls.append(("color", tuple(color), len(guids)))
        for guid in guids:
            canvas.objects[guid]["color"] = tuple(color)

    def object_color_source(guids, source):
        guids = guids if isinstance(guids, list) else [guids]
        calls.append(("source", source, len(guids)))
        for guid in guids:
            canvas.objects[guid].pop("color", None)

    rs.ObjectColor, rs.ObjectColorSource, rs.IsObject = object_color, object_color_source, lambda g: g in canvas.objects
    process = canvas.process
    from scaffolding_assembly_core.geometry import Pose
    from core import process_display

    scene = process.scene_at(_movement(process, "B1_J_insert"))
    robot, bar = "robots/cindy", "bars/B1"
    process_display.place(scene, robot, Pose((1.0, 0.5, 0.0)), {"left_ur_arm_elbow_joint": 0.4})
    green, yellow = (60, 179, 60), (225, 185, 40)

    ik_viz.update_state(scene, robot_id=robot, layer_key="Assembly", colors={bar: green, robot: yellow})
    assert all(o["color"] == green for o in canvas.named(bar))
    robot_objects = [o for o in canvas.objects.values() if o["name"].startswith(robot + "/")]
    assert robot_objects and all(o["color"] == yellow for o in robot_objects)
    assert canvas.named("ground/WG0") and all("color" not in o for o in canvas.named("ground/WG0"))

    # Unchanged colours are not painted again; a caller's highlight is undone by `restore_colors`.
    calls.clear()
    ik_viz.update_state(scene, robot_id=robot, layer_key="Assembly", colors={bar: green, robot: yellow})
    assert calls == []
    for guid, obj in canvas.objects.items():
        obj.pop("color", None)
    ik_viz.restore_colors("Assembly")
    assert all(o["color"] == green for o in canvas.named(bar))

    # No colours: every part goes back to its layer's.
    ik_viz.update_state(scene, robot_id=robot, layer_key="Assembly")
    assert all("color" not in o for o in canvas.objects.values())
