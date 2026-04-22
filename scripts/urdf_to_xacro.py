"""Wrap urdf/<variant>.urdf in xacro/<variant>.macro.xacro, a macro that mounts it on a host robot.

The macro prefixes each link and joint name, so that one robot can carry two grippers.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from specialist_description import ROBOTS_DIR


def convert(variant: str) -> str:
    path = ROBOTS_DIR / variant / "urdf" / f"{variant}.urdf"
    urdf = ET.parse(path, ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))).getroot()
    children = {joint.find("child").get("link") for joint in urdf.iter("joint")}
    (root,) = [link.get("name") for link in urdf.iter("link") if link.get("name") not in children]
    for element in [*urdf.iter("link"), *urdf.iter("joint")]:
        element.set("name", "${prefix}" + element.get("name"))
    for tag, attribute in (("parent", "link"), ("child", "link"), ("mimic", "joint")):
        for element in urdf.iter(tag):
            element.set(attribute, "${prefix}" + element.get(attribute))
    for mesh in urdf.iter("mesh"):
        mesh.set("filename", "${mesh_root}/" + Path(mesh.get("filename")).name)

    robot = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    mesh_root = f"package://specialist_description/robots/{variant}/meshes"
    params = f"prefix parent *origin mesh_root:={mesh_root}"
    macro = ET.SubElement(robot, "xacro:macro", name=variant, params=params)
    mount = ET.SubElement(macro, "joint", name=f"${{prefix}}{root}_mount", type="fixed")
    ET.SubElement(mount, "parent", link="${parent}")
    ET.SubElement(mount, "child", link="${prefix}" + root)
    ET.SubElement(mount, "xacro:insert_block", name="origin")
    macro.extend(urdf)
    ET.indent(robot, "  ")
    return ET.tostring(robot, encoding="unicode") + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", help="a directory in specialist_description/robots")
    variant = parser.parse_args().variant
    path = ROBOTS_DIR / variant / "xacro" / f"{variant}.macro.xacro"
    path.parent.mkdir(exist_ok=True)
    path.write_text(convert(variant))


if __name__ == "__main__":
    main()
