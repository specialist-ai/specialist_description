"""Draw docs/preview.png: the visual meshes and the colliders of each variant."""

import os
import struct
import sys
import zlib
from pathlib import Path

# EGL renders without a display on Linux, and macOS does not have EGL. On macOS, MuJoCo uses CGL,
# which also needs no display. MuJoCo reads this variable on import.
if sys.platform == "linux":
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from specialist_description import VARIANTS, get_mjcf_path  # noqa: E402

OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "preview.png"
HULL_COLORS = [
    (0.85, 0.33, 0.24, 1),
    (0.95, 0.62, 0.21, 1),
    (0.36, 0.63, 0.35, 1),
    (0.27, 0.51, 0.78, 1),
]
PRIMITIVE_COLOR = (0.55, 0.58, 0.62, 0.5)
MARGIN = 16


def render(variant: str) -> list[np.ndarray]:
    """Return an image of the visual geoms and an image of the collision geoms of a variant."""
    spec = mujoco.MjSpec.from_file(str(get_mjcf_path(variant)))
    sky = spec.add_texture(
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT
    )
    sky.rgb1 = sky.rgb2 = [1, 1, 1]
    sky.width = sky.height = 8
    model = spec.compile()
    model.vis.headlight.ambient = model.vis.headlight.diffuse = [0.5, 0.5, 0.5]
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    meshes = np.flatnonzero(
        (model.geom_group == 0) & (model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)
    )
    model.geom_rgba[model.geom_group == 0] = PRIMITIVE_COLOR
    model.geom_rgba[meshes] = [HULL_COLORS[i % len(HULL_COLORS)] for i in range(len(meshes))]

    low = (data.geom_xpos - model.geom_rbound[:, None]).min(axis=0)
    high = (data.geom_xpos + model.geom_rbound[:, None]).max(axis=0)
    camera = mujoco.MjvCamera()
    camera.lookat, camera.distance = (low + high) / 2, 0.9 * np.linalg.norm(high - low)
    camera.azimuth, camera.elevation = 120, -22

    images = []
    with mujoco.Renderer(model, 480, 640) as renderer:
        for group in (1, 0):
            option = mujoco.MjvOption()
            option.geomgroup[:] = 0
            option.geomgroup[group] = 1
            renderer.update_scene(data, camera, option)
            images.append(renderer.render())
    return images


def main() -> None:
    panels = np.array([render(variant) for variant in VARIANTS])
    # One crop box for all panels, so that the rows and the columns line up.
    ink = panels.min(axis=-1) < 250
    rows = np.flatnonzero(ink.any(axis=(0, 1, 3)))
    columns = np.flatnonzero(ink.any(axis=(0, 1, 2)))
    panels = panels[
        :, :, rows[0] - MARGIN : rows[-1] + MARGIN, columns[0] - MARGIN : columns[-1] + MARGIN
    ]
    sheet = np.vstack([np.hstack(row) for row in panels])
    height, width, _ = sheet.shape
    scanlines = b"".join(b"\0" + row.tobytes() for row in sheet)

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (
        b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(scanlines, 9))
    )
    OUTPUT.write_bytes(png + chunk(b"IEND", b""))


if __name__ == "__main__":
    main()
