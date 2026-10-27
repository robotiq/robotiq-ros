# Copyright (c) 2026 Robotiq
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Expand the Hand-E xacro at both strokes and check what each URDF promises.

The 0.050 m description is the original and must not change; the 0.100 m one
has its own base and fingers, built from the CAD (meshes/*/hand_e_100).
"""

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PKG_DIR = Path(__file__).parents[1]
URDF_DIR = PKG_DIR / "urdf"
MODEL = "robotiq_hand_e_gripper.urdf.xacro"
MESH_DIRS = {0.050: "hand_e", 0.100: "hand_e_100"}

LEFT = "hande_left_finger_joint"
RIGHT = "hande_right_finger_joint"

requires_xacro = pytest.mark.skipif(
    shutil.which("xacro") is None, reason="xacro not on PATH"
)


def expand(*args, check=True):
    return subprocess.run(
        ["xacro", str(URDF_DIR / MODEL), *args],
        capture_output=True,
        text=True,
        check=check,
    )


def robot_for(stroke):
    return ET.fromstring(
        expand(f"hand_e_stroke:={stroke:.3f}", "use_fake_hardware:=false").stdout
    )


def floats(text):
    return [float(v) for v in text.split()]


def mesh_files(robot):
    prefix = "package://robotiq_description/"
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename")
        assert filename.startswith(prefix)
        yield PKG_DIR / filename[len(prefix) :]


@requires_xacro
def test_the_default_is_the_0_050_description():
    assert expand().stdout == expand("hand_e_stroke:=0.050").stdout
    assert expand().stdout == expand("hand_e_stroke:=0.05").stdout


@requires_xacro
@pytest.mark.parametrize("stroke", sorted(MESH_DIRS))
def test_each_finger_travels_half_the_stroke_and_the_pads_meet_when_closed(stroke):
    robot = robot_for(stroke)

    def pad_x(name, travel):
        joint = robot.find(f"joint[@name='{name}']")
        assert joint.get("type") == "prismatic"
        limit = joint.find("limit")
        assert float(limit.get("lower")) == 0.0
        assert float(limit.get("upper")) == pytest.approx(stroke / 2)
        return floats(joint.find("origin").get("xyz"))[0] + (
            floats(joint.find("axis").get("xyz"))[0] * travel
        )

    assert pad_x(RIGHT, 0.0) - pad_x(LEFT, 0.0) == pytest.approx(stroke)
    assert pad_x(RIGHT, stroke / 2) - pad_x(LEFT, stroke / 2) == pytest.approx(0.0)
    mimic = robot.find(f"joint[@name='{RIGHT}']/mimic")
    assert mimic.get("joint") == LEFT


@requires_xacro
@pytest.mark.parametrize("stroke", sorted(MESH_DIRS))
def test_the_driver_closes_at_one_finger_s_travel(stroke):
    robot = robot_for(stroke)
    params = {p.get("name"): p.text for p in robot.iter("param")}
    assert float(params["gripper_closed_position"]) == pytest.approx(stroke / 2)
    assert params["gripper_profile"] == "hand_e"
    assert float(params["gripper_max_speed"]) == 0.075
    assert float(params["gripper_max_force"]) == 185.0


WRAPPER = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="wrapper">
    <xacro:include filename="$(find robotiq_description)/urdf/robotiq_hand_e_macro.urdf.xacro" />
    <link name="world" />
    <xacro:robotiq_gripper name="G" prefix="" parent="world" use_fake_hardware="false"
        stroke="{stroke}" gripper_closed_position="{closed}">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
</robot>
"""


@requires_xacro
@pytest.mark.parametrize("stroke", sorted(MESH_DIRS))
def test_an_explicit_closed_position_overrides_the_computed_one(stroke, tmp_path):
    """The shipped wrapper does not forward gripper_closed_position, so a macro call of its own does."""
    wrapper = tmp_path / "wrapper.urdf.xacro"
    wrapper.write_text(WRAPPER.format(stroke=f"{stroke:.3f}", closed="0.04"))
    robot = ET.fromstring(
        subprocess.run(
            ["xacro", str(wrapper)], capture_output=True, text=True, check=True
        ).stdout
    )
    params = {p.get("name"): p.text for p in robot.iter("param")}
    assert float(params["gripper_closed_position"]) == pytest.approx(0.04)
    assert float(params["gripper_closed_position"]) != pytest.approx(stroke / 2)


@requires_xacro
@pytest.mark.parametrize("stroke", sorted(MESH_DIRS))
def test_the_meshes_come_from_the_strokes_own_folder_and_exist(stroke):
    robot = robot_for(stroke)
    files = list(mesh_files(robot))
    assert len(files) == 6
    for path in files:
        assert path.parent.name == MESH_DIRS[stroke]
        assert path.is_file(), path


@requires_xacro
def test_the_0_100_base_inertia_is_physical():
    robot = robot_for(0.100)
    inertial = robot.find("link[@name='hand_e_base_link']/inertial")
    mass = float(inertial.find("mass").get("value"))
    # The 50 mm base's vendor mass scaled by the datasheet weights, 1.1 kg / 1.0 kg.
    assert mass == pytest.approx(0.86387 * 1.1, abs=1e-5)
    i = inertial.find("inertia").attrib
    matrix = np.array(
        [
            [i["ixx"], i["ixy"], i["ixz"]],
            [i["ixy"], i["iyy"], i["iyz"]],
            [i["ixz"], i["iyz"], i["izz"]],
        ],
        dtype=float,
    )
    assert np.all(np.linalg.eigvalsh(matrix) > 0)


@requires_xacro
@pytest.mark.parametrize("stroke", ["0.070", "0.025", "0.200"])
def test_an_unsupported_stroke_fails(stroke):
    result = expand(f"hand_e_stroke:={stroke}", check=False)
    assert result.returncode != 0
    assert "stroke must be 0.050 or 0.100" in result.stderr
