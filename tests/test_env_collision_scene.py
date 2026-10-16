"""Headless tests for the scene pieces of ``core.env_collision`` (issues D2, D3, D6).

- one body naming scheme for every cell (``bar_<id>``, ``joint_<jid>_<sub>``,
  ``ground_<id>``);
- the "built before this step" filter over the full body set;
- the floor slab made from a flat walkable-ground surface.

Run with the tamp venv (has compas + compas_fab):
    external\\husky_assembly_tamp\\.venv\\Scripts\\python.exe -m pytest tests/test_env_collision_scene.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

from core import env_collision
from core.env_collision import (
    bar_body_name,
    collect_built_geometry,
    floor_body_name,
    floor_slab_mesh_data,
    joint_body_name,
)


# * ---------------------------------------------------------------- names


def test_names():
    assert bar_body_name("B3") == "bar_B3"
    assert joint_body_name("J1-3", "Male") == "joint_J1-3_male"
    assert joint_body_name("G1-T20Ground-0", "ground") == "joint_G1-T20Ground-0_ground"
    assert floor_body_name("WG0") == "ground_WG0"
    # Every managed name starts with one of the managed prefixes.
    for name in ("bar_B3", "joint_J1-3_male", "obstacle_table", "ground_WG0"):
        assert name.startswith(env_collision.MANAGED_BODY_PREFIXES)


# * ---------------------------------------------------------------- built filter


BAR_MAP = {"B1": ("oid1", 1), "B3": ("oid3", 2), "B4": ("oid4", 3), "B5": ("oid5", 4)}
ALL_GEOM = {
    "bar_B1": {"kind": "bar", "parent_bar_id": "B1"},
    "bar_B3": {"kind": "bar", "parent_bar_id": "B3"},
    "joint_J1-3_male": {"kind": "joint", "parent_bar_id": "B3"},
    "bar_B4": {"kind": "bar", "parent_bar_id": "B4"},
    "bar_B5": {"kind": "bar", "parent_bar_id": "B5"},
    "obstacle_table": {"kind": "environment"},
    "ground_WG0": {"kind": "floor"},
}


def test_built_before_step():
    """Before B4: B1 and B3 (+ its joint); static bodies are never 'built'."""
    out = collect_built_geometry("B4", BAR_MAP, all_geom=ALL_GEOM)
    assert sorted(out) == ["bar_B1", "bar_B3", "joint_J1-3_male"]


def test_built_including_the_step_and_excluding_a_bar():
    out = collect_built_geometry("B4", BAR_MAP, include_active=True, exclude_bar_ids=["B3"],
                                 all_geom=ALL_GEOM)
    assert sorted(out) == ["bar_B1", "bar_B4"]


def test_unknown_step_is_empty():
    assert collect_built_geometry("B99", BAR_MAP, all_geom=ALL_GEOM) == {}


# * ---------------------------------------------------------------- floor slab


SQUARE = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 3.0, 0.0), (0.0, 3.0, 0.0)]


@pytest.mark.parametrize("face", [[0, 1, 2, 3], [3, 2, 1, 0]])
def test_slab_under_a_flat_square(face):
    """Top stays on the surface, bottom 0.05 m below, whatever the face winding."""
    vertices, faces, up = floor_slab_mesh_data(SQUARE, [face], 0.05)
    points = np.asarray(vertices)
    assert len(points) == 8
    assert np.allclose(points[:4, 2], 0.0)
    assert np.allclose(points[4:, 2], -0.05)
    assert np.allclose(up, [0.0, 0.0, 1.0])
    # Top + bottom + the four outline walls.
    assert len(faces) == 6


def test_slab_from_two_triangles_has_four_walls_only():
    """The shared diagonal is not on the outline, so it gets no wall."""
    faces = [[0, 1, 2], [0, 2, 3]]
    _vertices, slab_faces, _up = floor_slab_mesh_data(SQUARE, faces, 0.05)
    assert len(slab_faces) == 2 + 2 + 4


def test_stepped_surface_is_refused():
    """Two faces at an angle: not flat -> split it into flat pieces."""
    vertices = SQUARE + [(2.0, 3.0, 1.0), (0.0, 3.0, 1.0)]
    faces = [[0, 1, 2, 3], [3, 2, 4, 5]]
    with pytest.raises(ValueError, match="not flat"):
        floor_slab_mesh_data(vertices, faces, 0.05)


def test_wall_is_refused():
    """A vertical surface (like the reference problem's WG2) is not a floor."""
    wall = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 0.0, 3.0), (0.0, 0.0, 3.0)]
    with pytest.raises(ValueError, match="a wall is not a floor"):
        floor_slab_mesh_data(wall, [[0, 1, 2, 3]], 0.05)


def test_zero_area_is_refused():
    with pytest.raises(ValueError, match="no area"):
        floor_slab_mesh_data([(0, 0, 0), (1, 0, 0), (2, 0, 0)], [[0, 1, 2]], 0.05)
