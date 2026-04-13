"""Headless PyBullet tests for the generated URDF kinematic chain."""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pybullet as p
import pybullet_data
import pytest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from generate_urdf import DEFAULT_URDF_PATH, SCALE, generate_urdf


URDF_PATH = DEFAULT_URDF_PATH
TOL = 1e-4
ANGLE_TOL = 1e-3


@pytest.fixture(scope="module")
def robot(viz_enabled):
    generate_urdf(URDF_PATH)
    if viz_enabled:
        client_id = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8,
            cameraYaw=45.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[0.0, 0.0, 0.15],
        )
    else:
        client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(URDF_PATH, basePosition=[0.0, 0.0, 0.0], useFixedBase=True)

    joint_map: dict[str, int] = {}
    link_map: dict[str, int] = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_map[joint_info[1].decode("utf-8")] = joint_index
        link_map[joint_info[12].decode("utf-8")] = joint_index

    yield {"id": robot_id, "joints": joint_map, "links": link_map, "client": client_id, "viz": viz_enabled}
    p.disconnect(client_id)


def set_joints(robot, values_mm_deg: dict[str, float]) -> None:
    for joint_name, value in values_mm_deg.items():
        joint_index = robot["joints"][joint_name]
        joint_info = p.getJointInfo(robot["id"], joint_index)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_PRISMATIC:
            p.resetJointState(robot["id"], joint_index, value * SCALE)
        else:
            p.resetJointState(robot["id"], joint_index, math.radians(value))


def get_link_pos(robot, link_name: str) -> np.ndarray:
    link_index = robot["links"][link_name]
    state = p.getLinkState(robot["id"], link_index, computeForwardKinematics=True)
    return np.array(state[0], dtype=float)


def get_link_orn_matrix(robot, link_name: str) -> np.ndarray:
    link_index = robot["links"][link_name]
    state = p.getLinkState(robot["id"], link_index, computeForwardKinematics=True)
    return np.array(p.getMatrixFromQuaternion(state[1]), dtype=float).reshape(3, 3)


def _pause_if_viz(robot, title=""):
    if robot.get("viz"):
        p.addUserDebugText(title, [0.0, 0.0, 0.5], textColorRGB=[0.0, 0.0, 0.0], textSize=1.5)
        try:
            input(f"  [VIZ] {title} - press Enter to continue...")
        except EOFError:
            pass


class TestURDFStructure:
    def test_has_5_movable_joints(self, robot):
        movable = []
        for joint_index in range(p.getNumJoints(robot["id"])):
            joint_info = p.getJointInfo(robot["id"], joint_index)
            if joint_info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable.append(joint_info[1].decode("utf-8"))
        assert set(movable) == {"fjp_joint", "fjr_joint", "jjr_joint", "mjr_joint", "mjp_joint"}

    def test_joint_types(self, robot):
        expected_types = {
            "fjp_joint": p.JOINT_PRISMATIC,
            "fjr_joint": p.JOINT_REVOLUTE,
            "jjr_joint": p.JOINT_REVOLUTE,
            "mjr_joint": p.JOINT_REVOLUTE,
            "mjp_joint": p.JOINT_PRISMATIC,
        }
        for joint_name, expected_type in expected_types.items():
            joint_index = robot["joints"][joint_name]
            joint_info = p.getJointInfo(robot["id"], joint_index)
            assert joint_info[2] == expected_type

    def test_has_expected_links(self, robot):
        expected = {"fjp_link", "fjr_link", "female_link", "male_link", "mjr_link", "mjr_out_link", "ln_bar"}
        assert expected.issubset(set(robot["links"]))


class TestFJP:
    def test_fjp_moves_female_along_le(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 100.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos1 = get_link_pos(robot, "female_link")

        delta = pos1 - pos0
        assert abs(delta[2] - 0.1) < TOL
        assert abs(delta[0]) < TOL
        assert abs(delta[1]) < TOL
        _pause_if_viz(robot, "test_fjp_moves_female_along_le")

    def test_fjp_moves_ln_bar_equally(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "ln_bar").copy()

        set_joints(robot, {"fjp_joint": 50.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos1 = get_link_pos(robot, "ln_bar")

        delta = pos1 - pos0
        assert abs(delta[2] - 0.05) < TOL
        _pause_if_viz(robot, "test_fjp_moves_ln_bar_equally")


class TestFJR:
    def test_fjr_rotates_female_around_le(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 90.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos90 = get_link_pos(robot, "female_link")

        radial = config.FEMALE_RADIAL_OFFSET * SCALE
        assert abs(pos0[0] - radial) < TOL
        assert abs(pos0[1]) < TOL
        assert abs(pos90[0]) < TOL
        assert abs(pos90[1] - radial) < TOL
        _pause_if_viz(robot, "test_fjr_rotates_female_around_le")

    def test_fjr_does_not_change_z(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        z0 = get_link_pos(robot, "female_link")[2]

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 45.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        z45 = get_link_pos(robot, "female_link")[2]

        assert abs(z45 - z0) < TOL
        _pause_if_viz(robot, "test_fjr_does_not_change_z")


class TestJJR:
    def test_jjr_does_not_move_female(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 90.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos90 = get_link_pos(robot, "female_link")

        np.testing.assert_allclose(pos0, pos90, atol=TOL)
        _pause_if_viz(robot, "test_jjr_does_not_move_female")

    def test_jjr_changes_ln_direction(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        ln_dir_0 = get_link_orn_matrix(robot, "ln_bar")[:, 2]

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 90.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        ln_dir_90 = get_link_orn_matrix(robot, "ln_bar")[:, 2]

        cos_angle = np.clip(np.dot(ln_dir_0, ln_dir_90), -1.0, 1.0)
        angle = math.degrees(math.acos(abs(cos_angle)))
        assert abs(angle - 90.0) < 2.0
        _pause_if_viz(robot, "test_jjr_changes_ln_direction")


class TestMJP:
    def test_mjp_moves_ln_along_its_axis(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "ln_bar").copy()
        ln_dir = get_link_orn_matrix(robot, "ln_bar")[:, 2]

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 100.0})
        pos1 = get_link_pos(robot, "ln_bar")

        delta = pos1 - pos0
        displacement_along = np.dot(delta, ln_dir)
        displacement_perp = np.linalg.norm(delta - displacement_along * ln_dir)
        assert abs(displacement_along - 0.1) < TOL
        assert displacement_perp < TOL
        _pause_if_viz(robot, "test_mjp_moves_ln_along_its_axis")

    def test_mjp_does_not_move_female(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 100.0})
        pos1 = get_link_pos(robot, "female_link")

        np.testing.assert_allclose(pos0, pos1, atol=TOL)
        _pause_if_viz(robot, "test_mjp_does_not_move_female")


class TestMJR:
    def test_mjr_does_not_change_ln_position(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "ln_bar").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 90.0, "mjp_joint": 0.0})
        pos90 = get_link_pos(robot, "ln_bar")

        np.testing.assert_allclose(pos0, pos90, atol=TOL)
        _pause_if_viz(robot, "test_mjr_does_not_change_ln_position")

    def test_mjr_rotates_ln_axes(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        R0 = get_link_orn_matrix(robot, "ln_bar")

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 90.0, "mjp_joint": 0.0})
        R90 = get_link_orn_matrix(robot, "ln_bar")

        np.testing.assert_allclose(R0[:, 2], R90[:, 2], atol=ANGLE_TOL)
        assert abs(np.dot(R0[:, 0], R90[:, 0])) < 0.1
        _pause_if_viz(robot, "test_mjr_rotates_ln_axes")


class TestRadialOffset:
    def test_female_offset_from_bar_axis(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        female_pos = get_link_pos(robot, "female_link")
        radial_dist = math.sqrt(female_pos[0] ** 2 + female_pos[1] ** 2)
        expected = config.FEMALE_RADIAL_OFFSET * SCALE
        assert abs(radial_dist - expected) < TOL
        _pause_if_viz(robot, "test_female_offset_from_bar_axis")

    def test_total_bar_separation(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        ln_pos = get_link_pos(robot, "ln_bar")
        radial_dist = math.sqrt(ln_pos[0] ** 2 + ln_pos[1] ** 2)
        expected = (config.FEMALE_RADIAL_OFFSET + config.MALE_RADIAL_OFFSET) * SCALE
        assert abs(radial_dist - expected) < TOL
        _pause_if_viz(robot, "test_total_bar_separation")
