"""Convert the onshape-to-robot export in cad/ to urdf/<variant>.urdf and meshes/.

The onshape-to-robot app exports cad/robot.pkl and the part meshes in cad/assets/. This script gives
each part the collider that cad/colliders.json names, and writes the URDF.
"""

import argparse
import json
import os
import pickle
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from onshape_to_robot.config import Config
from onshape_to_robot.exporter_urdf import ExporterURDF
from onshape_to_robot.processor_scad import ProcessorScad

from specialist_description import ROBOTS_DIR


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", help="a directory in specialist_description/robots")
    variant = parser.parse_args().variant
    robot_dir = ROBOTS_DIR / variant
    # The mesh paths in robot.pkl are relative to cad/.
    os.chdir(robot_dir / "cad")
    config = Config("config.json")
    robot = pickle.loads(Path("robot.pkl").read_bytes())
    policies = json.loads(Path("colliders.json").read_text()).items()

    scad = ProcessorScad(config)
    for link in robot.links:
        for part in link.parts:
            policy = next(
                policy for pattern, policy in policies if re.fullmatch(pattern, part.name)
            )
            if policy == "mesh":
                continue
            (mesh,) = part.meshes
            mesh.collision = False
            if policy == "scad":
                part.shapes += scad.parse_scad(f"scad/{part.name}.scad", mesh.color)
            elif policy != "none":
                raise ValueError(f"Part {part.name} has the unknown collider policy {policy!r}.")

    xml = ExporterURDF(config).get_xml(robot)
    urdf = ET.fromstring(xml, ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    meshes = robot_dir / "meshes"
    shutil.rmtree(meshes, ignore_errors=True)
    meshes.mkdir()
    for mesh in urdf.iter("mesh"):
        source = Path(mesh.get("filename").removeprefix("package://"))
        shutil.copy(source, meshes)
        mesh.set("filename", f"../meshes/{source.name}")

    joints = {joint.get("name"): joint for joint in urdf.iter("joint")}
    properties = json.loads(Path("joint_properties.json").read_text())
    for name, values in properties.items():
        if "friction" in values:
            ET.SubElement(joints[name], "dynamics", friction=str(values["friction"]))
        # The CAD can put the gear on either finger. Move each <mimic> to the joint that follows.
        if "mimic" in values:
            driven = joints[values["mimic"]]
            mimic = driven.find("mimic")
            if mimic is not None:
                driven.remove(mimic)
                ratio = str(1 / float(mimic.get("multiplier")))
                ET.SubElement(joints[name], "mimic", joint=values["mimic"], multiplier=ratio)

    ET.indent(urdf, "  ")
    (robot_dir / "urdf").mkdir(exist_ok=True)
    (robot_dir / "urdf" / f"{variant}.urdf").write_text(
        ET.tostring(urdf, encoding="unicode") + "\n"
    )


if __name__ == "__main__":
    main()
