"""Generate a PyBullet-friendly URDF for the T20-5 connector chain."""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config


SCALE = 0.001
LE_BAR_LENGTH = 440.0
LN_BAR_LENGTH = 420.0
FEMALE_BOX_SIZE = (46.0, 28.0, 18.0)
MALE_BOX_SIZE = (40.0, 24.0, 16.0)
FIXED_JOINT_RPY = (math.pi, -math.pi / 2.0, 0.0)
DEFAULT_URDF_PATH = os.path.join(SCRIPT_DIR, "T20_5_chain.urdf")


def _format_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def _add_inertial(link: ET.Element, mass: float, inertia: float) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": f"{mass:.9g}"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": f"{inertia:.9g}",
            "iyy": f"{inertia:.9g}",
            "izz": f"{inertia:.9g}",
            "ixy": "0",
            "ixz": "0",
            "iyz": "0",
        },
    )


def _add_visual(link: ET.Element, geometry_tag: str, geometry_attrs: dict[str, str], rgba: tuple[float, float, float, float]) -> None:
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, geometry_tag, geometry_attrs)
    material = ET.SubElement(visual, "material", {"name": f"{link.attrib['name']}_material"})
    ET.SubElement(material, "color", {"rgba": " ".join(f"{value:.9g}" for value in rgba)})


def _add_link(
    robot: ET.Element,
    name: str,
    *,
    mass: float,
    inertia: float,
    geometry_tag: str | None = None,
    geometry_attrs: dict[str, str] | None = None,
    rgba: tuple[float, float, float, float] | None = None,
) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    if geometry_tag is not None and geometry_attrs is not None and rgba is not None:
        _add_visual(link, geometry_tag, geometry_attrs, rgba)
    _add_inertial(link, mass, inertia)


def _add_joint(
    robot: ET.Element,
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    origin_xyz: tuple[float, float, float],
    origin_rpy: tuple[float, float, float],
    *,
    axis_xyz: tuple[float, float, float] | None = None,
    limits: tuple[float, float] | None = None,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": _format_triplet(origin_xyz), "rpy": _format_triplet(origin_rpy)})
    if axis_xyz is not None:
        ET.SubElement(joint, "axis", {"xyz": _format_triplet(axis_xyz)})
    if limits is not None:
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": f"{limits[0]:.9g}",
                "upper": f"{limits[1]:.9g}",
                "effort": "100",
                "velocity": "1",
            },
        )


def build_urdf_tree() -> ET.ElementTree:
    s = SCALE
    robot = ET.Element("robot", {"name": "t20_5_connector_chain"})

    _add_link(
        robot,
        "le_bar",
        mass=0.1,
        inertia=0.001,
        geometry_tag="cylinder",
        geometry_attrs={"length": f"{LE_BAR_LENGTH * s:.9g}", "radius": f"{config.BAR_RADIUS * s:.9g}"},
        rgba=(0.5, 0.5, 0.5, 1.0),
    )
    _add_link(robot, "fjp_link", mass=0.01, inertia=0.0001)
    _add_link(robot, "fjr_link", mass=0.01, inertia=0.0001)
    _add_link(
        robot,
        "female_link",
        mass=0.05,
        inertia=0.0001,
        geometry_tag="box",
        geometry_attrs={"size": _format_triplet(tuple(value * s for value in FEMALE_BOX_SIZE))},
        rgba=(0.9, 0.5, 0.55, 1.0),
    )
    _add_link(
        robot,
        "male_link",
        mass=0.05,
        inertia=0.0001,
        geometry_tag="box",
        geometry_attrs={"size": _format_triplet(tuple(value * s for value in MALE_BOX_SIZE))},
        rgba=(0.44, 0.85, 0.55, 1.0),
    )
    _add_link(robot, "mjr_link", mass=0.01, inertia=0.0001)
    _add_link(robot, "mjr_out_link", mass=0.01, inertia=0.0001)
    _add_link(
        robot,
        "ln_bar",
        mass=0.1,
        inertia=0.001,
        geometry_tag="cylinder",
        geometry_attrs={"length": f"{LN_BAR_LENGTH * s:.9g}", "radius": f"{config.BAR_RADIUS * s:.9g}"},
        rgba=(0.85, 0.63, 0.4, 1.0),
    )

    _add_joint(
        robot,
        "fjp_joint",
        "prismatic",
        "le_bar",
        "fjp_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=(config.FJP_RANGE[0] * s, config.FJP_RANGE[1] * s),
    )
    _add_joint(
        robot,
        "fjr_joint",
        "revolute",
        "fjp_link",
        "fjr_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=config.FJR_RANGE,
    )
    _add_joint(
        robot,
        "female_offset_joint",
        "fixed",
        "fjr_link",
        "female_link",
        (config.FEMALE_RADIAL_OFFSET * s, 0.0, config.FEMALE_AXIAL_OFFSET * s),
        FIXED_JOINT_RPY,
    )
    _add_joint(
        robot,
        "jjr_joint",
        "revolute",
        "female_link",
        "male_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=config.JJR_RANGE,
    )
    _add_joint(
        robot,
        "male_offset_joint",
        "fixed",
        "male_link",
        "mjr_link",
        (config.MALE_AXIAL_OFFSET * s, 0.0, config.MALE_RADIAL_OFFSET * s),
        FIXED_JOINT_RPY,
    )
    _add_joint(
        robot,
        "mjr_joint",
        "revolute",
        "mjr_link",
        "mjr_out_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=config.MJR_RANGE,
    )
    _add_joint(
        robot,
        "mjp_joint",
        "prismatic",
        "mjr_out_link",
        "ln_bar",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=(config.MJP_RANGE[0] * s, config.MJP_RANGE[1] * s),
    )

    return ET.ElementTree(robot)


def generate_urdf(output_path: str = DEFAULT_URDF_PATH) -> str:
    tree = build_urdf_tree()
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def main() -> None:
    output_path = generate_urdf()
    print(f"URDF written to {output_path}")


if __name__ == "__main__":
    main()
