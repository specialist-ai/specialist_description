"""Compile urdf/<variant>.urdf with MuJoCo to mjcf/<variant>.xml, and add what the URDF cannot state."""

import argparse
import json
import xml.etree.ElementTree as ET
from collections import Counter

import mujoco

from specialist_description import ROBOTS_DIR

# A gear is rigid, but the default equality of MuJoCo yields in proportion to the inertia of the
# joints that it couples. These values hold the fingers to 0.004 rad at the 3 Nm stall of the servo.
GEAR_SOLREF = [0.004, 1.0]
GEAR_SOLIMP = [0.95, 0.99, 0.001, 0.5, 2.0]


def convert(variant: str) -> str:
    robot_dir = ROBOTS_DIR / variant
    urdf = ET.parse(robot_dir / "urdf" / f"{variant}.urdf").getroot()
    # Keep the visual geoms and the bodies of the fixed joints.
    mujoco_tag = ET.SubElement(urdf, "mujoco")
    ET.SubElement(mujoco_tag, "compiler", discardvisual="false", fusestatic="false")
    spec = mujoco.MjSpec.from_string(ET.tostring(urdf, encoding="unicode"))
    # The mesh paths are relative to urdf/, so they resolve the same way from mjcf/.
    spec.modelfiledir = str(robot_dir / "mjcf")
    properties = json.loads((robot_dir / "cad" / "joint_properties.json").read_text())

    # MuJoCo does not filter the contacts of a parent and a child if the parent is welded to the
    # world. The base is welded to the world, so without an exclude it pushes on the fingers.
    for joint in urdf.iter("joint"):
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        spec.add_exclude(bodyname1=parent, bodyname2=child)
    # The URDF import makes a joint equality of each <mimic>.
    for gear in spec.equalities:
        gear.solref = GEAR_SOLREF
        gear.solimp = GEAR_SOLIMP

    uses = Counter(geom.meshname for geom in spec.geoms)
    index = Counter()
    for body in spec.bodies:
        for geom in body.geoms:
            key = geom.meshname or f"{body.name}_collider"
            geom.name = key if uses[key] == 1 else f"{key}_{index[key]}"
            index[key] += 1
            # A pinch makes one contact on each side, and only torsional friction stops the object
            # from turning about the line between the two contacts.
            if geom.group == 0 and geom.meshname:
                geom.condim = 4
            # A primitive is larger than its part, so two primitives must not collide.
            elif geom.group == 0:
                geom.contype, geom.conaffinity = 2, 1

    for name, values in properties.items():
        if "armature" in values:
            spec.joint(name).armature = values["armature"]

    mjcf = ET.fromstring(spec.to_xml())
    ET.indent(mjcf, "  ")
    return ET.tostring(mjcf, encoding="unicode") + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", help="a directory in specialist_description/robots")
    variant = parser.parse_args().variant
    path = ROBOTS_DIR / variant / "mjcf" / f"{variant}.xml"
    path.parent.mkdir(exist_ok=True)
    path.write_text(convert(variant))


if __name__ == "__main__":
    main()
