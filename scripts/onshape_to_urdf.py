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

import coacd
import numpy as np
import pymeshlab
import trimesh
from onshape_to_robot.config import Config
from onshape_to_robot.exporter_urdf import ExporterURDF
from onshape_to_robot.geometry import Mesh
from onshape_to_robot.processor_scad import ProcessorScad

from specialist_description import ROBOTS_DIR

# A fused part touches its target face to face, and the union of two such solids keeps slivers
# of zero volume. Thus the part grows by FUSE_GROWTH, so that the two solids overlap. The union then
# has edges a few nanometers long, which trimesh collapses into holes. Thus vertices closer than
# FUSE_WELD merge. Both values are in meters. The union is not robust: on the current tips, a growth
# of 35 um gives an open mesh, and 20 um gives faces that intersect. main() stops on such a mesh.
FUSE_GROWTH = 4e-5
FUSE_WELD = 1e-6

# A left part is the mirror image of its right part across the yz plane of the part frame.
MIRROR = np.array([-1.0, 1.0, 1.0])


def fuse(target_path: str, part_path: str, T_target_part: np.ndarray) -> trimesh.Trimesh:
    """Return the union of two STL solids, in the frame of the first."""
    target = trimesh.load(target_path)
    part = trimesh.load(part_path)
    part.apply_transform(T_target_part)
    # Round to float32, as STL does, so that two equal parts fuse to equal bytes, and CoACD cuts them
    # into equal hulls.
    part.vertices = part.vertices.astype(np.float32)
    meshes = pymeshlab.MeshSet()
    meshes.add_mesh(pymeshlab.Mesh(target.vertices, target.faces))
    meshes.add_mesh(pymeshlab.Mesh(part.vertices + part.vertex_normals * FUSE_GROWTH, part.faces))
    meshes.generate_boolean_union(first_mesh=0, second_mesh=1)
    meshes.meshing_merge_close_vertices(threshold=pymeshlab.PureValue(FUSE_WELD))
    union = meshes.current_mesh()
    # The union gives its faces in a different order on each call. Thus each face starts at its lowest
    # vertex, which keeps its winding, and the faces go in order, so that equal parts give equal bytes.
    faces = union.face_matrix()
    faces = np.take_along_axis(faces, (faces.argmin(axis=1)[:, None] + np.arange(3)) % 3, axis=1)
    return trimesh.Trimesh(union.vertex_matrix(), faces[np.lexsort(faces.T[::-1])])


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
        # The parts fuse in the order of their names, because the union changes with the order.
        for part in sorted(link.parts, key=lambda part: part.name):
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
                case {"fuse": target_name}:
                    target = next(other for other in link.parts if other.name == target_name)
                    # A second part fuses into the union of the first.
                    (target_mesh,) = [other for other in target.meshes if other.collision]
                    filename = config.asset_path(f"{target.name}_fused.stl")
                    T_target_part = np.linalg.inv(target.T_world_part) @ part.T_world_part
                    fuse(target_mesh.filename, mesh.filename, T_target_part).export(filename)
                    target_mesh.collision = False
                    target.meshes.append(
                        Mesh(filename, target_mesh.color, visual=False, collision=True)
                    )
                case {"mirror": source_name}:
                    mirrors.append((part, source_name))
                case "none":
                    pass
                case _:
                    raise ValueError(
                        f"Part {part.name} has the unknown collider policy {policy!r}."
                    )

    # CoACD remeshes a mesh that fails its manifold check on a 50-voxel grid, which grows the hulls by
    # up to 0.5 mm. A fused mesh fails that check, so CoACD cuts each mesh as it is. This needs a
    # closed mesh that does not intersect itself.
    os.makedirs(config.asset_path("convex_decomposition"), exist_ok=True)
    for link in robot.links:
        for part in link.parts:
            for mesh in [other for other in part.meshes if other.collision]:
                solid = trimesh.load(mesh.filename)
                check = pymeshlab.MeshSet()
                check.add_mesh(pymeshlab.Mesh(solid.vertices, solid.faces))
                check.compute_selection_by_self_intersections_per_face()
                if not solid.is_watertight or check.current_mesh().selected_face_number():
                    raise RuntimeError(
                        f"The collision mesh {mesh.filename} is open or intersects itself."
                    )
                hulls = coacd.run_coacd(
                    coacd.Mesh(solid.vertices, solid.faces),
                    max_convex_hull=16,
                    preprocess_mode="off",
                )
                mesh.collision = False
                for index, (vertices, faces) in enumerate(hulls):
                    filename = config.asset_path(
                        f"convex_decomposition/{part.name}_{index:05d}.stl"
                    )
                    trimesh.Trimesh(vertices, faces).export(filename)
                    part.meshes.append(Mesh(filename, mesh.color, visual=False, collision=True))

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
