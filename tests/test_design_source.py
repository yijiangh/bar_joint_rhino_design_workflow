"""`core.design_source` on a fake Rhino document (`fake_rhino`): three bars, a hold, a fake bar, one walkable ground.

Same layout as `test_design_builder.py`, but read through the Rhino API as RSExportDesign does, in mm.
"""

import importlib

import numpy as np
import pytest

import fake_rhino
from fake_rhino import Brep, Doc, Obj
from test_design_builder import CINDY_URDF, PKG

pytestmark = pytest.mark.skipif(not CINDY_URDF.is_file(), reason="asset/husky_urdf submodule not checked out")

DOWN = np.diag([1.0, -1.0, -1.0])     # z axis pointing down: 180° about x
TURN = np.diag([-1.0, -1.0, 1.0])     # the T20/Ground seat: 180° about z


def _xform(rotation, position) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, position
    return matrix


@pytest.fixture
def source():
    """A fake document installed as the Rhino API; yields the `core.design_source` module over it."""
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
        from core import design_source
        yield importlib.reload(design_source)
    finally:
        fake_rhino.uninstall()


def test_document_becomes_a_valid_design(source, tmp_path):
    """Bodies, mates, robots and actions; written and read back with no A or B errors."""
    from bar_assembly_core.design import write
    from bar_assembly_core.design.plan_check import check_plan

    result = source.RhinoDesignSource("RSExportDesign").build()
    design = write(result.design, tmp_path / "design", package_dirs=[PKG])
    assert design.schedule == ("B1_J", "B1_R", "B2_J", "B2_H", "B2_R", "B3_J", "B3_R", "B2_HR")
    assert "bars/B4" not in design.bodies and "joints/J4-3_female" not in design.bodies
    assert any("fake bars" in p for p in result.problems)
    assert ("joints/J1-2_female", "joints/J1-2_male") in design.mates
    assert ("ground/WG0", "joints/G1-T20-0_ground") in design.mates
    assert design.bodies["joints/J1-2_male"].part == "T20/Male" and design.bodies["joints/J1-2_male"].mount == \
        "bars/B2"
    bar = design.bodies["bars/B1"]
    assert np.allclose(bar.pose.position, (0.5, 0.0, 0.05)) and bar.geometry.collision[0].height == pytest.approx(1)
    assert design.producer.command == "RSExportDesign"
    insert = design.actions["B1_J"].movements[-1]
    assert np.allclose(insert.line["robots/cindy/left_ur_arm_tool0"].direction, (0, 0, -1))
    report = check_plan(design)
    assert report.ok, report.errors


def test_keyframe_groups_merge():
    """Rhino's per-arm keyframe record becomes one joint map; a support record is one group."""
    from core.design_source import joints_from_groups

    groups = {"left": {"joint_names": ["a", "b"], "joint_values": [1, 2]},
              "right": {"joint_names": ["c"], "joint_values": [3]}}
    assert joints_from_groups(groups) == {"a": 1.0, "b": 2.0, "c": 3.0}
    assert joints_from_groups({"joint_names": ["x"], "joint_values": [0.5]}) == {"x": 0.5}
    assert joints_from_groups(None) is None
