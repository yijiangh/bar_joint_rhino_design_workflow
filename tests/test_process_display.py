"""`core.process_display`: what `ik_viz` draws of a process's scenes, read from a (fake) Rhino document.

No PyBullet and no compas: parts come from the core's `scene_parts` (bodies, robot links, mounted tools).
"""

import numpy as np
import pytest

from test_process_source import source  # noqa: F401  (fixture: the fake document's process_source module)


def _movement(process, movement_id):
    return next(m for a in process.actions.values() for m in a.movements if m.id == movement_id)


def test_drawn_parts(source):  # noqa: F811
    """Every body and robot part by id; absent robots and robots with unknown bases are hidden, with what they hold."""
    from bar_assembly_core.scene import scene_parts
    from core import process_display

    store = {}
    result = source.RhinoProcessSource("x", source.geometry_cache(store)).build()
    process = result.process
    insert = process.scene_at(_movement(process, "B1_J_insert"))  # Cindy holds B1, her base unknown (unsolved)
    parts = process_display.drawn_parts(insert, ("robots/cindy",))
    assert set(process.bodies) <= set(parts)
    assert "robots/cindy/left_ur_arm_wrist_3_link" in parts and "robots/alice/ur_arm_wrist_3_link" in parts
    assert parts["robots/cindy/left_ur_arm_wrist_3_link"].owner == "robots/cindy"
    assert all(parts[tool.id].owner == "robots/cindy" for tool in insert.robots["robots/cindy"].model.tools.values())
    # Cindy's base is not solved: she, her tools and the bar she holds are hidden; Alice is absent.
    assert process_display.parked(insert, ("robots/cindy",)) == {"robots/cindy", "robots/alice", "robots/belle"}
    assert parts["bars/B1"].pose is None and parts["robots/cindy/left_ur_arm_wrist_3_link"].pose is None
    assert parts["ground/WG0"].pose is not None

    from bar_assembly_core.geometry import Pose
    process_display.place(insert, "robots/cindy", Pose((1.0, 0.0, 0.0)))
    parts = process_display.drawn_parts(insert, ("robots/cindy",))
    shown = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(insert)
             if not part_id.startswith(("robots/alice", "robots/belle"))}
    for part_id, (geometry, pose) in shown.items():
        assert parts[part_id].geometry is geometry
        assert parts[part_id].pose is not None and np.allclose(parts[part_id].pose.matrix(), pose.matrix())
    assert parts["bars/B1"].pose is not None
    # Without `acting`, a robot with unknown joints is hidden; `hide_bodies` hides every body.
    insert.robots["robots/cindy"].unmeasured = frozenset({"left_ur_arm_elbow_joint"})
    assert process_display.drawn_parts(insert)["robots/cindy/left_ur_arm_wrist_3_link"].pose is None
    assert all(parts[b].pose is None for b in process.bodies
               for parts in [process_display.drawn_parts(insert, ("robots/cindy",), hide_bodies=True)])


def test_place_and_ghost(source):  # noqa: F811
    """`place` moves a robot and what it holds; a ghost scene keeps only that robot, at the origin."""
    from bar_assembly_core.geometry import Pose
    from core import process_display

    store = {}
    process = source.RhinoProcessSource("x", source.geometry_cache(store)).build().process
    scene = process.scene_at(_movement(process, "B1_J_insert"))
    assert not scene.robots["robots/cindy"].base_tracked
    process_display.place(scene, "robots/cindy", Pose((1.0, 0.0, 0.0)), {"left_ur_arm_elbow_joint": 0.3})
    robot = scene.robots["robots/cindy"]
    assert robot.base_tracked and robot.joints["left_ur_arm_elbow_joint"] == pytest.approx(0.3)
    held = scene.world_poses["bars/B1"]
    process_display.place(scene, "robots/cindy", Pose((2.0, 0.0, 0.0)))
    assert np.allclose(np.subtract(scene.world_poses["bars/B1"].position, held.position), (1.0, 0.0, 0.0))

    ghost = process_display.ghost_scene(scene, "robots/alice")
    assert ghost.robots["robots/alice"].enabled and not ghost.robots["robots/cindy"].enabled
    alice = ghost.robots["robots/alice"]
    assert alice.base.position == (0.0, 0.0, 0.0) and all(v == 0.0 for v in alice.joints.values())
    assert process_display.parked(ghost) == {"robots/cindy", "robots/belle"}
    assert scene.robots["robots/cindy"].enabled  # the original scene is untouched


def test_robot_scene(source):  # noqa: F811
    """A robot alone (no action needed), at the origin, every body hidden, with the process's models."""
    from core import process_display

    process = source.RhinoProcessSource("x", source.geometry_cache({})).build().process
    insert = process.scene_at(_movement(process, "B1_J_insert"))
    alone = process_display.robot_scene(process, "robots/cindy", {"left_ur_arm_elbow_joint": 1.0})
    assert alone.robots["robots/cindy"].model is insert.robots["robots/cindy"].model
    parts = process_display.drawn_parts(alone, ("robots/cindy",))
    assert all(parts[b].pose is None for b in process.bodies)
    assert parts["robots/cindy/left_ur_arm_wrist_3_link"].pose is not None
    assert alone.robots["robots/cindy"].joints["left_ur_arm_elbow_joint"] == pytest.approx(1.0)
