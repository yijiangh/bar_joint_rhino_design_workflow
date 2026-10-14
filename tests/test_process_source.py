"""`core.process_source` on a fake Rhino document (`fake_rhino`): three bars, a hold, a fake bar, one walkable ground.

Same layout as `test_process_builder.py`, but read through the Rhino API as RSExportProcess does, in mm.
"""

import importlib

import numpy as np
import pytest

import fake_rhino
from fake_rhino import Brep, Doc, Obj
from test_process_builder import CINDY_URDF, PKG

pytestmark = pytest.mark.skipif(not CINDY_URDF.is_file(), reason="asset/husky_urdf submodule not checked out")

DOWN = np.diag([1.0, -1.0, -1.0])     # z axis pointing down: 180° about x
TURN = np.diag([-1.0, -1.0, 1.0])     # the T20/Ground seat: 180° about z


def _xform(rotation, position) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, position
    return matrix


@pytest.fixture
def source():
    """A fake document installed as the Rhino API; yields the `core.process_source` module over it."""
    from core import config, joint_name_conventions as jnc
    from core.robotic_tool import get_active_pair

    doc = Doc()
    for bar, y, z, seq, extra in (("B1", 0, 50, 1, {}), ("B2", 300, 50, 2, {"supported_until": "B3"}),
                                  ("B3", 150, 100, 3, {}), ("B4", 600, 50, 4, {"scaffolding.fake_bar": "1"})):
        doc.add(Obj(config.LAYER_BAR_CENTERLINES, {"bar_type": "scaffolding_bar", "bar_id": bar, "bar_seq": str(seq),
                                                   config.KEY_BAR_WALKABLE_GROUND_IDS: "WG0", **extra},
                    start=(0, y, z), end=(1000, y, z)))
    doc.add(Obj(config.LAYER_WALKABLE_GROUND, {config.KEY_WALKABLE_GROUND_ID: "WG0"}, brep=Brep((-2500, 2500, -2500,
                                                                                                  2500))))
    tcp = {side: tool.M_tcp_from_block for side, tool in get_active_pair().items()}
    names = {side: tool.name for side, tool in get_active_pair().items()}
    halves = [("G1-T20-0", jnc.GROUND, "T20_Ground", "B1", (200, 0, 0), "left"),
              ("G1-T20-1", jnc.GROUND, "T20_Ground", "B1", (800, 0, 0), "right"),
              ("G2-T20-0", jnc.GROUND, "T20_Ground", "B2", (800, 300, 0), "right"),
              ("J1-2", jnc.FEMALE, "T20_Female", "B1", (500, 0, 50), None),
              ("J1-2", jnc.MALE, "T20_Male", "B2", (200, 300, 50), "left"),
              ("J1-3", jnc.FEMALE, "T20_Female", "B1", (300, 0, 50), None),
              ("J1-3", jnc.MALE, "T20_Male", "B3", (300, 50, 100), "left"),
              ("J2-3", jnc.FEMALE, "T20_Female", "B2", (600, 300, 50), None),
              ("J2-3", jnc.MALE, "T20_Male", "B3", (600, 250, 100), "right"),
              ("J4-3", jnc.FEMALE, "T20_Female", "B4", (900, 600, 50), None)]
    for jid, subtype, block, bar, position, side in halves:
        xform = _xform(DOWN, position)
        doc.add(Obj(jnc.joint_layer(subtype), {jnc.UT_JOINT_ID: jid, jnc.UT_PARENT_BAR: bar},
                    name=jnc.object_name(jid, subtype), block=block, xform=xform))
        if side:
            seat = _xform(TURN if subtype == jnc.GROUND else np.eye(3), (0, 0, 0))
            tool0 = xform @ np.linalg.inv(tcp[side] @ seat)
            doc.add(Obj(config.LAYER_TOOL_INSTANCES, {jnc.UT_JOINT_ID: jid, jnc.UT_TOOL_NAME: names[side]},
                        block="tool", xform=tool0))
    grasp = _xform(np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float), (500, 300, 50))
    b2 = next(oid for oid, o in doc.objects.items() if o.text.get("bar_id") == "B2")
    doc.objects[b2].text[config.KEY_SUPPORT_GRASP_FRAME] = str(grasp.tolist())
    fake_rhino.install(doc)
    try:
        yield importlib.import_module("core.process_source")  # * fresh: uninstall() drops it from sys.modules
    finally:
        fake_rhino.uninstall()


def test_document_becomes_a_valid_process(source, tmp_path):
    """Bodies, mates, robots and actions; written and read back with no A or B errors."""
    from scaffolding_assembly_core.process import write
    from scaffolding_assembly_core.process.plan_check import check_plan

    result = source.RhinoProcessSource("RSExportProcess").build()
    process = write(result.process, tmp_path / "process", package_dirs=[PKG])
    assert process.schedule == ("B1_J", "B1_R", "B2_J", "B2_H", "B2_R", "B3_J", "B3_R", "B2_HR")
    assert "bars/B4" not in process.bodies and "joints/J4-3_female" not in process.bodies
    assert any("fake bars" in p for p in result.problems)
    assert ("joints/J1-2_female", "joints/J1-2_male") in process.mates
    assert ("ground/WG0", "joints/G1-T20-0_ground") in process.mates
    assert process.bodies["joints/J1-2_male"].part == "T20/Male" and process.bodies["joints/J1-2_male"].mount == \
        "bars/B2"
    bar = process.bodies["bars/B1"]
    assert np.allclose(bar.assembly_pose.position, (0.5, 0.0, 0.05)) and bar.geometry.collision[0].height == pytest.approx(1)
    assert process.producer.command == "RSExportProcess"
    insert = process.actions["B1_J"].movements[-1]
    assert np.allclose(insert.line["robots/cindy/left_ur_arm_tool0"].direction, (0, 0, -1))
    report = check_plan(process)
    assert report.ok, report.errors


def test_keyframe_groups_merge():
    """Rhino's per-arm keyframe record becomes one joint map; a support record is one group."""
    from core.process_source import joints_from_groups

    groups = {"left": {"joint_names": ["a", "b"], "joint_values": [1, 2]},
              "right": {"joint_names": ["c"], "joint_values": [3]}}
    assert joints_from_groups(groups) == {"a": 1.0, "b": 2.0, "c": 3.0}
    assert joints_from_groups({"joint_names": ["x"], "joint_values": [0.5]}) == {"x": 0.5}
    assert joints_from_groups(None) is None


@pytest.mark.parametrize("name, actions", [("YH_ik_test.3dm", 5), ("examples/Example4_two_box.3dm", 45)])
def test_real_documents_write(name, actions, tmp_path):
    """Real documents (read with rhino3dm) become processes that pass the file checks; bars without tools are named."""
    pytest.importorskip("rhino3dm")
    from pathlib import Path
    from scaffolding_assembly_core.process import write

    fake_rhino.install(fake_rhino.doc_from_3dm(Path(__file__).resolve().parents[1] / name))
    try:
        result = importlib.import_module("core.process_source").RhinoProcessSource("RSExportProcess").build()
    finally:
        fake_rhino.uninstall()
    process = write(result.process, tmp_path / "process", package_dirs=[PKG])
    assert len(process.actions) == actions
    assert any("needs a left and a right tool" in p for p in result.problems)


def test_geometry_cache_keeps_objects(source):
    """Two builds with one cache share every Geometry, Tool and robot model; a resized bar gets a new Geometry."""
    from scaffolding_assembly_core.process.scenes import process_model

    store = {}
    first = source.RhinoProcessSource("x", source.geometry_cache(store)).build().process
    second = source.RhinoProcessSource("x", source.geometry_cache(store)).build().process
    assert all(second.bodies[b].geometry is first.bodies[b].geometry for b in first.bodies)
    assert len({id(body.geometry) for body in first.bodies.values()}) == len(first.bodies)  # never shared
    assert all(second.tools[t] is first.tools[t] for t in first.tools)
    assert process_model(second, "robots/cindy") is process_model(first, "robots/cindy")
    rs = __import__("rhinoscriptsyntax")
    b1 = next(oid for oid in rs.AllObjects() if rs.GetUserText(oid, "bar_id") == "B1")
    rs.coercerhinoobject(b1).end = (900, 0, 50)
    third = source.RhinoProcessSource("x", source.geometry_cache(store)).build().process
    assert third.bodies["bars/B1"].geometry is not first.bodies["bars/B1"].geometry
    assert third.bodies["bars/B2"].geometry is first.bodies["bars/B2"].geometry
