"""Headless PyBullet tests for the generated URDF kinematic chain."""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
import pybullet_data
import pytest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.geometry import distance_infinite_lines
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
    if link_name == "le_bar_link":
        return np.array(p.getBasePositionAndOrientation(robot["id"])[0], dtype=float)
    link_index = robot["links"][link_name]
    state = p.getLinkState(robot["id"], link_index, computeForwardKinematics=True)
    return np.array(state[0], dtype=float)


def get_link_orn_matrix(robot, link_name: str) -> np.ndarray:
    if link_name == "le_bar_link":
        quaternion = p.getBasePositionAndOrientation(robot["id"])[1]
        return np.array(p.getMatrixFromQuaternion(quaternion), dtype=float).reshape(3, 3)
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

    def test_has_expected_links(self, robot):
        expected = {
            "fjp_link",
            "fjr_link",
            "female_link",
            "female_screw_hole_link",
            "male_screw_hole_link",
            "male_link",
            "mjr_link",
            "mjp_link",
            "ln_bar_link",
        }
        assert expected.issubset(set(robot["links"]))

    def test_mesh_visuals_are_used_for_connector_links(self):
        tree = ET.parse(URDF_PATH)
        root = tree.getroot()
        mesh_attrs = {
            link.attrib["name"]: link.find("./visual/geometry/mesh").attrib
            for link in root.findall("link")
            if link.find("./visual/geometry/mesh") is not None
        }
        assert mesh_attrs["female_link"]["filename"] == "female_joint_mesh_mm.obj"
        assert mesh_attrs["male_link"]["filename"] == "male_joint_mesh_mm.obj"
        assert mesh_attrs["female_link"]["scale"] == "0.001 0.001 0.001"
        assert mesh_attrs["male_link"]["scale"] == "0.001 0.001 0.001"


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
        pos0 = get_link_pos(robot, "ln_bar_link").copy()

        set_joints(robot, {"fjp_joint": 50.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos1 = get_link_pos(robot, "ln_bar_link")

        delta = pos1 - pos0
        assert abs(delta[2] - 0.05) < TOL
        _pause_if_viz(robot, "test_fjp_moves_ln_bar_equally")


class TestFJR:
    def test_fjr_keeps_female_origin_on_bar_axis(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "female_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 90.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos90 = get_link_pos(robot, "female_link")

        np.testing.assert_allclose(pos0, pos90, atol=TOL)
        _pause_if_viz(robot, "test_fjr_keeps_female_origin_on_bar_axis")

    def test_fjr_rotates_female_screw_hole_around_le(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        female_pos_0 = get_link_pos(robot, "female_link").copy()
        screw_pos_0 = get_link_pos(robot, "female_screw_hole_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 90.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        female_pos_90 = get_link_pos(robot, "female_link").copy()
        screw_pos_90 = get_link_pos(robot, "female_screw_hole_link")

        offset_0 = screw_pos_0 - female_pos_0
        offset_90 = screw_pos_90 - female_pos_90
        radius = float(np.linalg.norm(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM[:3, 3])) * SCALE
        assert abs(np.linalg.norm(offset_0) - radius) < TOL
        assert abs(np.linalg.norm(offset_90) - radius) < TOL
        assert abs(np.dot(offset_0, offset_90)) < TOL
        _pause_if_viz(robot, "test_fjr_rotates_female_screw_hole_around_le")


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
        ln_dir_0 = get_link_orn_matrix(robot, "ln_bar_link")[:, 2]

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 90.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        ln_dir_90 = get_link_orn_matrix(robot, "ln_bar_link")[:, 2]

        cos_angle = np.clip(np.dot(ln_dir_0, ln_dir_90), -1.0, 1.0)
        angle = math.degrees(math.acos(abs(cos_angle)))
        assert abs(angle - 90.0) < 2.0
        _pause_if_viz(robot, "test_jjr_changes_ln_direction")


class TestMJP:
    def test_mjp_moves_ln_along_its_axis(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        pos0 = get_link_pos(robot, "ln_bar_link").copy()
        ln_dir = get_link_orn_matrix(robot, "ln_bar_link")[:, 2]

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 100.0})
        pos1 = get_link_pos(robot, "ln_bar_link")

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
        pos0 = get_link_pos(robot, "ln_bar_link").copy()

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 90.0, "mjp_joint": 0.0})
        pos90 = get_link_pos(robot, "ln_bar_link")

        np.testing.assert_allclose(pos0, pos90, atol=TOL)
        _pause_if_viz(robot, "test_mjr_does_not_change_ln_position")

    def test_mjr_rotates_ln_axes(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        R0 = get_link_orn_matrix(robot, "ln_bar_link")

        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 90.0, "mjp_joint": 0.0})
        R90 = get_link_orn_matrix(robot, "ln_bar_link")

        np.testing.assert_allclose(R0[:, 2], R90[:, 2], atol=ANGLE_TOL)
        assert abs(np.dot(R0[:, 0], R90[:, 0])) < 0.1
        _pause_if_viz(robot, "test_mjr_rotates_ln_axes")


class TestZeroPose:
    def test_zero_pose_bar_separation_matches_contact_distance(self, robot):
        set_joints(robot, {"fjp_joint": 0.0, "fjr_joint": 0.0, "jjr_joint": 0.0, "mjr_joint": 0.0, "mjp_joint": 0.0})
        le_pos = get_link_pos(robot, "le_bar_link")
        ln_pos = get_link_pos(robot, "ln_bar_link")
        le_axis = get_link_orn_matrix(robot, "le_bar_link")[:, 2]
        ln_axis = get_link_orn_matrix(robot, "ln_bar_link")[:, 2]
        distance = abs(distance_infinite_lines(le_pos, le_axis, ln_pos, ln_axis))
        assert abs(distance - config.BAR_CONTACT_DISTANCE * SCALE) < TOL
