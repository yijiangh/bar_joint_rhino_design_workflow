from __future__ import annotations

from collections import defaultdict

import math
import os
import sys

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_planning as pp


HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from generate_urdf import DEFAULT_URDF_PATH, generate_urdf


BASE_POSITION = [0.0, 0.0, 0.3]
BASE_YAW_RAD = -0.5 * math.pi
ANNOTATED_LINK_NAMES = (
    "le_bar_link",
    "female_link",
    "female_screw_hole_link",
    "male_screw_hole_link",
    "male_link",
    "ln_bar_link",
)
_LINK_ALPHA = {
    "le_bar_link": 0.2,
    "fjp_link": 0.1,
    "fjr_link": 0.1,
    "female_screw_hole_link": 0.35,
    "male_screw_hole_link": 0.35,
    "mjr_link": 0.1,
    "mjp_link": 0.1,
    "ln_bar_link": 0.25,
    "female_link": 0.15,
    "male_link": 0.15,
}


def _set_link_transparency(robot: int) -> None:
    for link_name, alpha in _LINK_ALPHA.items():
        try:
            link = pp.link_from_name(robot, link_name)
        except ValueError:
            continue
        pp.set_color(robot, pp.apply_alpha(pp.GREY, alpha), link=link)


def _draw_all_link_annotations(robot: int, pose_length: float = 0.03) -> None:
    label_counts: dict[tuple[float, float, float], int] = defaultdict(int)
    stacked_label_offset = np.array([0.0, 0.0, 0.012], dtype=float)

    for link_name in ANNOTATED_LINK_NAMES:
        link = pp.link_from_name(robot, link_name)
        link_pose = pp.get_link_pose(robot, link)
        pp.draw_pose(link_pose, length=pose_length if "bar" not in link_name else 0.01)
        link_position = np.asarray(link_pose[0], dtype=float)
        pose_key = tuple(np.round(link_position, 4))
        label_index = label_counts[pose_key]
        label_counts[pose_key] += 1
        label_position = link_position + label_index * stacked_label_offset
        pp.add_text(link_name, position=tuple(label_position))


def main():
    pp.connect(use_gui=True)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    robot_urdf = DEFAULT_URDF_PATH
    HERE = os.path.dirname(os.path.abspath(__file__))
    pp.create_obj(os.path.join(HERE, 'female_joint_mesh_mm.obj'), scale=0.001)
    pp.create_obj(os.path.join(HERE, 'male_joint_mesh_mm.obj'), scale=0.001)
    robot_urdf = generate_urdf(DEFAULT_URDF_PATH)
    assert os.path.exists(robot_urdf)

    robot = pp.load_pybullet(robot_urdf, fixed_base=True)
    pp.set_pose(robot, pp.Pose(point=BASE_POSITION, euler=pp.Euler(yaw=BASE_YAW_RAD)))
    _set_link_transparency(robot)
    pp.camera_focus_on_body(robot)
    _draw_all_link_annotations(robot)

    pp.wait_if_gui()


if __name__ == "__main__":
    main()
