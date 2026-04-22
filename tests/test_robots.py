import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import urdf_to_mjcf
import urdf_to_xacro
import xacro

from specialist_description import VARIANTS, get_mjcf_path, get_urdf_path, get_xacro_path

TWO_GRIPPERS = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arms">
  <xacro:include filename="{macro}"/>
  <link name="left_wrist"/>
  <link name="right_wrist"/>
  <xacro:{variant} prefix="left_" parent="left_wrist"><origin xyz="0 0 0"/></xacro:{variant}>
  <xacro:{variant} prefix="right_" parent="right_wrist"><origin xyz="0 0 0"/></xacro:{variant}>
</robot>
"""


@pytest.mark.parametrize("variant", VARIANTS)
def test_generated_files_match_the_urdf(variant):
    assert urdf_to_mjcf.convert(variant) == get_mjcf_path(variant).read_text()
    assert urdf_to_xacro.convert(variant) == get_xacro_path(variant).read_text()


@pytest.mark.parametrize("variant", VARIANTS)
def test_gear_drives_left_finger(variant):
    urdf = ET.parse(get_urdf_path(variant))
    mimics = {
        joint.get("name"): mimic.get("joint")
        for joint in urdf.iter("joint")
        for mimic in joint.iter("mimic")
    }
    assert mimics == {"left_finger": "right_finger"}
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
    assert [model.joint(model.eq_obj1id[0]).name, model.joint(model.eq_obj2id[0]).name] == [
        "left_finger",
        "right_finger",
    ]


@pytest.mark.parametrize("variant", VARIANTS)
def test_open_fingers_do_not_touch(variant):
    # A URDF has no collision filter, so a simulator with self-collision collides the two fingers.
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
    data = mujoco.MjData(model)
    colliders = np.flatnonzero(model.geom_group == 0)
    left, right = (
        colliders[model.geom_bodyid[colliders] == model.body(body).id]
        for body in ("left_finger", "right_finger")
    )
    for angle in np.linspace(model.joint("right_finger").range[0], 0, 21):
        data.joint("right_finger").qpos = angle
        data.joint("left_finger").qpos = -angle
        mujoco.mj_kinematics(model, data)
        assert all(
            mujoco.mj_geomDistance(model, data, a, b, 0.01, None) > 0 for a in left for b in right
        )


@pytest.mark.parametrize("variant", VARIANTS)
def test_every_geom_has_a_name(variant):
    model = mujoco.MjModel.from_xml_path(str(get_mjcf_path(variant)))
    assert all(model.geom(i).name for i in range(model.ngeom))


@pytest.mark.parametrize("variant", VARIANTS)
def test_two_macros_on_one_robot(variant, tmp_path):
    robot = tmp_path / "arms.urdf.xacro"
    robot.write_text(TWO_GRIPPERS.format(macro=get_xacro_path(variant), variant=variant))
    urdf = ET.fromstring(xacro.process_file(str(robot)).toxml())
    for tag in ("link", "joint"):
        names = [element.get("name") for element in urdf.iter(tag)]
        assert len(names) == len(set(names))
    assert sorted(mimic.get("joint") for mimic in urdf.iter("mimic")) == [
        "left_right_finger",
        "right_right_finger",
    ]
