"""Generate a PyBullet-friendly URDF for the CAD-backed connector chain."""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import config
from core.transforms import transform_to_xyz_rpy


SCALE = 0.001
LE_BAR_LENGTH = 440.0
LN_BAR_LENGTH = 420.0
DEFAULT_URDF_PATH = os.path.join(SCRIPT_DIR, "T20_5_chain.urdf")
ORIGIN_CLAMP_EPS = 1e-7
URDF_MESH_SCALE = (SCALE, SCALE, SCALE)
FEMALE_URDF_MESH_FILENAME = "female_joint_mesh_mm.obj"
MALE_URDF_MESH_FILENAME = "male_joint_mesh_mm.obj"


def _format_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def _format_origin_triplet(values: tuple[float, float, float]) -> str:
    clamped = tuple(0.0 if abs(value) < ORIGIN_CLAMP_EPS else value for value in values)
    return _format_triplet(clamped)


def _scaled_xyz(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(float(value) * SCALE for value in values)


def _origin_from_transform(transform) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xyz_mm, rpy = transform_to_xyz_rpy(transform)
    xyz = tuple(float(component) * SCALE for component in xyz_mm)
    return xyz, tuple(float(value) for value in rpy)


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


def _add_visual(
    link: ET.Element,
    geometry_tag: str,
    geometry_attrs: dict[str, str],
    rgba: tuple[float, float, float, float] | None = None,
    origin_xyz: tuple[float, float, float] | None = None,
    origin_rpy: tuple[float, float, float] | None = None,
) -> None:
    visual = ET.SubElement(link, "visual")
    if origin_xyz is not None or origin_rpy is not None:
        ET.SubElement(
            visual,
            "origin",
            {
                "xyz": _format_origin_triplet(origin_xyz or (0.0, 0.0, 0.0)),
                "rpy": _format_origin_triplet(origin_rpy or (0.0, 0.0, 0.0)),
            },
        )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, geometry_tag, geometry_attrs)
    if rgba is not None:
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
    visual_origin_xyz: tuple[float, float, float] | None = None,
    visual_origin_rpy: tuple[float, float, float] | None = None,
) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    if geometry_tag is not None and geometry_attrs is not None:
        _add_visual(
            link,
            geometry_tag,
            geometry_attrs,
            rgba,
            origin_xyz=visual_origin_xyz,
            origin_rpy=visual_origin_rpy,
        )
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
    ET.SubElement(
        joint,
        "origin",
        {"xyz": _format_origin_triplet(origin_xyz), "rpy": _format_origin_triplet(origin_rpy)},
    )
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
    robot = ET.Element("robot", {"name": "t20_5_connector_chain"})

    _add_link(
        robot,
        "le_bar_link",
        mass=0.1,
        inertia=0.001,
        geometry_tag="cylinder",
        geometry_attrs={"length": f"{LE_BAR_LENGTH * SCALE:.9g}", "radius": f"{config.BAR_RADIUS * SCALE:.9g}"},
        rgba=(0.5, 0.5, 0.5, 1.0),
        visual_origin_xyz=(0.0, 0.0, 0.5 * LE_BAR_LENGTH * SCALE),
    )
    _add_link(robot, "fjp_link", mass=0.01, inertia=0.0001)
    _add_link(robot, "fjr_link", mass=0.01, inertia=0.0001)
    _add_link(
        robot,
        "female_link",
        mass=0.05,
        inertia=0.0001,
        geometry_tag="mesh",
        geometry_attrs={
            "filename": FEMALE_URDF_MESH_FILENAME,
            "scale": _format_triplet(URDF_MESH_SCALE),
        },
    )
    _add_link(robot, "female_screw_hole_link", mass=0.01, inertia=0.0001)
    _add_link(robot, "male_screw_hole_link", mass=0.01, inertia=0.0001)
    _add_link(
        robot,
        "male_link",
        mass=0.05,
        inertia=0.0001,
        geometry_tag="mesh",
        geometry_attrs={
            "filename": MALE_URDF_MESH_FILENAME,
            "scale": _format_triplet(URDF_MESH_SCALE),
        },
    )
    _add_link(robot, "mjr_link", mass=0.01, inertia=0.0001)
    _add_link(robot, "mjp_link", mass=0.01, inertia=0.0001)
    _add_link(
        robot,
        "ln_bar_link",
        mass=0.1,
        inertia=0.001,
        geometry_tag="cylinder",
        geometry_attrs={"length": f"{LN_BAR_LENGTH * SCALE:.9g}", "radius": f"{config.BAR_RADIUS * SCALE:.9g}"},
        rgba=(0.85, 0.63, 0.4, 1.0),
        visual_origin_xyz=(0.0, 0.0, 0.5 * LN_BAR_LENGTH * SCALE),
    )

    female_fixed_xyz, female_fixed_rpy = _origin_from_transform(config.FEMALE_FIXED_ROT_FROM_BAR_TRANSFORM)
    female_gap_xyz, female_gap_rpy = _origin_from_transform(config.FEMALE_MALE_GAP_OFFSET_TRANSFORM)
    jjr_zero_xyz, jjr_zero_rpy = _origin_from_transform(config.JJR_ZERO_TRANSFORM)
    male_offset_xyz, male_offset_rpy = _origin_from_transform(config.MALE_SCREW_HOLE_OFFSET_TRANSFORM)
    male_fixed_xyz, male_fixed_rpy = _origin_from_transform(config.MALE_FIXED_ROT_TO_BAR_TRANSFORM)

    _add_joint(
        robot,
        "fjp_joint",
        "prismatic",
        "le_bar_link",
        "fjp_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=(config.FJP_RANGE[0] * SCALE, config.FJP_RANGE[1] * SCALE),
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
        "female_fixed_rot_from_bar",
        "fixed",
        "fjr_link",
        "female_link",
        female_fixed_xyz,
        female_fixed_rpy,
    )
    _add_joint(
        robot,
        "female_male_gap_offset",
        "fixed",
        "female_link",
        "female_screw_hole_link",
        female_gap_xyz,
        female_gap_rpy,
    )
    _add_joint(
        robot,
        "jjr_joint",
        "revolute",
        "female_screw_hole_link",
        "male_screw_hole_link",
        jjr_zero_xyz,
        jjr_zero_rpy,
        axis_xyz=(0.0, 0.0, 1.0),
        limits=config.JJR_RANGE,
    )
    _add_joint(
        robot,
        "male_screw_hole_offset",
        "fixed",
        "male_screw_hole_link",
        "male_link",
        male_offset_xyz,
        male_offset_rpy,
    )
    _add_joint(
        robot,
        "male_fixed_rot_to_bar",
        "fixed",
        "male_link",
        "mjr_link",
        male_fixed_xyz,
        male_fixed_rpy,
    )
    _add_joint(
        robot,
        "mjr_joint",
        "revolute",
        "mjr_link",
        "mjp_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=config.MJR_RANGE,
    )
    _add_joint(
        robot,
        "mjp_joint",
        "prismatic",
        "mjp_link",
        "ln_bar_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        axis_xyz=(0.0, 0.0, 1.0),
        limits=(config.MJP_RANGE[0] * SCALE, config.MJP_RANGE[1] * SCALE),
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
