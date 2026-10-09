"""`core.design_display`: the core's drawing cells for a design read from a (fake) Rhino document.

No PyBullet: drawing cells are compas_fab cells with visual meshes, built by the core from the design's scenes.
"""

from importlib.util import find_spec

import numpy as np
import pytest

from test_design_source import source  # noqa: F401  (fixture: the fake document's design_source module)

pytestmark = pytest.mark.skipif(find_spec("compas_fab") is None, reason="compas_fab not installed")
if find_spec("compas_fab") is not None:
    # ! Before the fake Rhino is installed: compas checks for a `Rhino` module once, on its first import.
    from core import core_lib  # noqa: F401
    import bar_assembly_core.mirrors.cell  # noqa: F401


def _movement(design, movement_id):
    return next(m for a in design.actions.values() for m in a.movements if m.id == movement_id)


def test_drawing_cell_and_state(source):  # noqa: F811
    """Cindy's drawing cell has visual meshes and every design body; absent robots are parked; the cell is kept."""
    from core import design_display

    store = {}
    result = source.RhinoDesignSource("x", source.geometry_cache(store)).build()
    design = result.design
    assert result.objects["bars/B1"] and result.objects["joints/J1-2_male"] and result.objects["ground/WG0"]
    insert = design.scene_at(_movement(design, "B1_J_insert"))
    cell, state = design_display.cell_and_state(store, insert, "robots/cindy")
    assert any(link.visual for link in cell.robot_model.links)
    assert set(cell.rigid_body_models) == set(design.bodies)
    assert state.rigid_body_states["bars/B1"].attached_to_link == "left_ur_arm_tool0"
    assert list(state.tool_states["robots/alice"].frame.point) == [50.0, 50.0, 0.0]  # Alice not there yet
    again = source.RhinoDesignSource("x", source.geometry_cache(store)).build().design
    cell2, _ = design_display.cell_and_state(store, again.scene_at(_movement(again, "B1_R_home")), "robots/cindy")
    assert cell2 is cell


def test_place_and_ghost(source):  # noqa: F811
    """`place` moves a robot and what it holds; a ghost scene keeps only that robot, at the origin."""
    from bar_assembly_core.geometry import Pose
    from core import design_display

    store = {}
    design = source.RhinoDesignSource("x", source.geometry_cache(store)).build().design
    scene = design.scene_at(_movement(design, "B1_J_insert"))  # Cindy holds B1, her base unknown (unsolved)
    assert not scene.robots["robots/cindy"].base_tracked
    design_display.place(scene, "robots/cindy", Pose((1.0, 0.0, 0.0)), {"left_ur_arm_elbow_joint": 0.3})
    robot = scene.robots["robots/cindy"]
    assert robot.base_tracked and robot.joints["left_ur_arm_elbow_joint"] == pytest.approx(0.3)
    held = scene.world_poses["bars/B1"]
    design_display.place(scene, "robots/cindy", Pose((2.0, 0.0, 0.0)))
    assert np.allclose(np.subtract(scene.world_poses["bars/B1"].position, held.position), (1.0, 0.0, 0.0))

    ghost = design_display.ghost_scene(scene, "robots/alice")
    assert ghost.robots["robots/alice"].enabled and not ghost.robots["robots/cindy"].enabled
    cell, state = design_display.cell_and_state(store, ghost, "robots/alice")
    assert list(state.robot_base_frame.point) == [0.0, 0.0, 0.0]
    assert all(v == 0.0 for v in state.robot_configuration.joint_values)
    assert list(state.tool_states["robots/cindy"].frame.point) == [50.0, 50.0, 0.0]
    assert scene.robots["robots/cindy"].enabled  # the original scene is untouched


def test_robot_scene_shares_the_cell(source):  # noqa: F811
    """A robot alone (no action needed) draws on the same cell as the design's scenes; bodies hidden."""
    from core import design_display

    store = {}
    design = source.RhinoDesignSource("x", source.geometry_cache(store)).build().design
    cell, _ = design_display.cell_and_state(store, design.scene_at(_movement(design, "B1_J_insert")), "robots/cindy")
    alone = design_display.robot_scene(design, "robots/cindy", {"left_ur_arm_elbow_joint": 1.0})
    cell2, state = design_display.cell_and_state(store, alone, "robots/cindy")
    assert cell2 is cell
    assert all(body.is_hidden for body in state.rigid_body_states.values())
    assert state.robot_configuration["left_ur_arm_elbow_joint"] == pytest.approx(1.0)
    assert list(state.robot_base_frame.point) == [0.0, 0.0, 0.0]
