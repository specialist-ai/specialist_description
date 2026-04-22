"""Paths to the generated descriptions of the Specialist grippers."""

from pathlib import Path

ROBOTS_DIR = Path(__file__).parent / "robots"
VARIANTS = ("specialist_handheld", "specialist_actuated")


def get_urdf_path(variant: str) -> Path:
    return ROBOTS_DIR / variant / "urdf" / f"{variant}.urdf"


def get_mjcf_path(variant: str) -> Path:
    return ROBOTS_DIR / variant / "mjcf" / f"{variant}.xml"


def get_xacro_path(variant: str) -> Path:
    return ROBOTS_DIR / variant / "xacro" / f"{variant}.macro.xacro"
