"""Interactive PyBullet viewer for the generated T20-5 URDF chain."""

from __future__ import annotations

import math
import os
import sys
import time

import pybullet as p
import pybullet_data


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from generate_urdf import SCALE, DEFAULT_URDF_PATH, generate_urdf


def _degrees(bounds_rad: tuple[float, float]) -> tuple[float, float]:
    return tuple(math.degrees(value) for value in bounds_rad)


def main() -> None:
    urdf_path = generate_urdf(DEFAULT_URDF_PATH)
    physics_client = p.connect(p.GUI)

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(urdf_path, basePosition=[0.0, 0.0, 0.3], useFixedBase=True)

        joint_map: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                joint_map[joint_name] = joint_index

        slider_specs = [
            ("fjp_joint", "FJP (mm)", config.FJP_RANGE[0], config.FJP_RANGE[1], 0.0, "prismatic"),
            ("fjr_joint", "FJR (deg)", *_degrees(config.FJR_RANGE), 0.0, "revolute"),
            ("jjr_joint", "JJR (deg)", *_degrees(config.JJR_RANGE), 0.0, "revolute"),
            ("mjr_joint", "MJR (deg)", *_degrees(config.MJR_RANGE), 0.0, "revolute"),
            ("mjp_joint", "MJP (mm)", config.MJP_RANGE[0], config.MJP_RANGE[1], 0.0, "prismatic"),
        ]

        sliders: dict[str, tuple[int, str]] = {}
        for joint_name, label, lower, upper, default, joint_kind in slider_specs:
            sliders[joint_name] = (p.addUserDebugParameter(label, lower, upper, default), joint_kind)

        print("Movable joints:", sorted(joint_map))
        print("Use the PyBullet sliders to inspect the 5-DOF chain.")

        p.resetDebugVisualizerCamera(
            cameraDistance=0.8,
            cameraYaw=45.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[0.0, 0.0, 0.3],
        )

        while p.isConnected(physics_client):
            for joint_name, (slider_id, joint_kind) in sliders.items():
                slider_value = p.readUserDebugParameter(slider_id)
                joint_value = slider_value * SCALE if joint_kind == "prismatic" else math.radians(slider_value)
                if joint_name in joint_map:
                    p.resetJointState(robot_id, joint_map[joint_name], joint_value)
            time.sleep(0.02)
    finally:
        if p.isConnected(physics_client):
            p.disconnect(physics_client)


if __name__ == "__main__":
    main()
