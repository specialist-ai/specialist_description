"""Open the scad collider of a part in OpenSCAD, with the meshes of its link as a reference."""

import argparse
import pickle
import subprocess

import numpy as np
import trimesh

from specialist_description import ROBOTS_DIR


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("variant", help="a directory in specialist_description/robots")
    parser.add_argument("part", help="a part name in cad/colliders.json")
    args = parser.parse_args()
    cad = ROBOTS_DIR / args.variant / "cad"
    robot = pickle.loads((cad / "robot.pkl").read_bytes())
    parts = {part.name: (link, part) for link in robot.links for part in link.parts}
    link, part = parts[args.part]

    # The scad file imports this mesh in the part frame.
    T_part_world = np.linalg.inv(part.T_world_part)
    meshes = []
    for other in link.parts:
        (mesh,) = other.meshes
        meshes.append(
            trimesh.load(cad / mesh.filename).apply_transform(T_part_world @ other.T_world_part)
        )
    trimesh.util.concatenate(meshes).export(cad / "scad" / f"{link.name}_context.stl")
    subprocess.run(["openscad", f"{args.part}.scad"], cwd=cad / "scad", check=True)


if __name__ == "__main__":
    main()
