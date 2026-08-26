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

import numpy as np
import trimesh
from onshape_to_robot.config import Config
from onshape_to_robot.exporter_urdf import ExporterURDF
from onshape_to_robot.geometry import Mesh
from onshape_to_robot.processor_convex_decomposition import ProcessorConvexDecomposition
from onshape_to_robot.processor_scad import ProcessorScad

from specialist_description import ROBOTS_DIR

# A left part is the mirror image of its right part across the yz plane of the part frame.
MIRROR = np.array([-1.0, 1.0, 1.0])


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
    mirrors = []
    for link in robot.links:
        for part in link.parts:
            policy = next(
                policy for pattern, policy in policies if re.fullmatch(pattern, part.name)
            )
            if policy == "mesh":
                continue
            (mesh,) = part.meshes
            mesh.collision = False
            match policy:
                case "scad":
                    part.shapes += scad.parse_scad(f"scad/{part.name}.scad", mesh.color)
                case {"mirror": source_name}:
                    mirrors.append((part, source_name))
                case "none":
                    pass
                case _:
                    raise ValueError(
                        f"Part {part.name} has the unknown collider policy {policy!r}."
                    )
    ProcessorConvexDecomposition(config).process(robot)

    # Onshape triangulates a part and its mirror image differently, and CoACD cuts the two meshes
    # into different hulls. Thus a mirror part takes the mirror images of the hulls of its source.
    parts = {part.name: part for link in robot.links for part in link.parts}
    for part, source_name in mirrors:
        (mesh,) = part.meshes
        (source_mesh,) = [other for other in parts[source_name].meshes if other.visual]
        vertices = np.unique(trimesh.load(mesh.filename).vertices, axis=0)
        mirrored = np.unique(trimesh.load(source_mesh.filename).vertices * MIRROR, axis=0)
        if vertices.shape != mirrored.shape or not np.allclose(vertices, mirrored, atol=1e-6):
            raise RuntimeError(f"Part {part.name} is not the mirror image of {source_name}.")
        hulls = [other.filename for other in parts[source_name].meshes if other.collision]
        for index, path in enumerate(hulls):
            hull = trimesh.load(path)
            filename = config.asset_path(f"convex_decomposition/{part.name}_{index:05d}.stl")
            # A mirror turns each face inside out, so the vertex order of each face reverses.
            trimesh.Trimesh(hull.vertices * MIRROR, hull.faces[:, ::-1]).export(filename)
            part.meshes.append(Mesh(filename, mesh.color, visual=False, collision=True))

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
