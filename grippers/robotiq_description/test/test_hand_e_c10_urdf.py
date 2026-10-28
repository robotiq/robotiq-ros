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

"""Expand the Hand-E C10 xacro and check what its URDF promises.

The C10 has three fingers 120 degrees apart that slide radially toward the axis. Nothing drives it yet, so
the description has no control support: links, joints, meshes and inertia, and no ros2_control block.
"""

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PKG_DIR = Path(__file__).parents[1]
URDF_DIR = PKG_DIR / "urdf"
MODEL = "robotiq_hand_e_c10_gripper.urdf.xacro"

GRIPPER_MASS = 1.7  # kg, datasheet weight

# The `robot` fixture skips when xacro is missing; this test expands the xacro itself.
requires_xacro = pytest.mark.skipif(
    shutil.which("xacro") is None, reason="xacro not on PATH"
)


def expand(*args, model=MODEL):
    result = subprocess.run(
        ["xacro", str(URDF_DIR / model), *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return ET.fromstring(result.stdout)


def floats(text):
    return [float(v) for v in text.split()]


@pytest.fixture(scope="module")
def robot():
    if shutil.which("xacro") is None:
        pytest.skip("xacro not on PATH")
    return expand()


def finger_joints(robot):
    return [j for j in robot.findall("joint") if "hande_c10_finger_" in j.get("name")]


def test_three_prismatic_fingers_under_one_base(robot):
    joints = finger_joints(robot)
    assert [j.get("name") for j in joints] == [
        "hande_c10_finger_1_joint",
        "hande_c10_finger_2_joint",
        "hande_c10_finger_3_joint",
    ]
    for j in joints:
        assert j.get("type") == "prismatic"
        assert j.find("parent").get("link") == "hand_e_c10_base_link"
        assert floats(j.find("axis").get("xyz")) == [-1.0, 0.0, 0.0]


def test_fingers_are_120_degrees_apart(robot):
    angles = [floats(j.find("origin").get("rpy"))[2] for j in finger_joints(robot)]
    assert angles == pytest.approx([0.0, 2 * math.pi / 3, 4 * math.pi / 3])


def test_finger_1_drives_and_the_others_mimic_it(robot):
    first, second, third = finger_joints(robot)
    assert first.find("mimic") is None
    for j in (second, third):
        mimic = j.find("mimic")
        assert mimic is not None
        assert mimic.get("joint") == "hande_c10_finger_1_joint"
        assert float(mimic.get("multiplier", "1")) == 1.0
        assert float(mimic.get("offset", "0")) == 0.0


def test_the_three_fingers_share_one_valid_limit(robot):
    limits = [
        tuple(
            float(j.find("limit").get(k))
            for k in ("lower", "upper", "effort", "velocity")
        )
        for j in finger_joints(robot)
    ]
    assert len(set(limits)) == 1, limits
    lower, upper, effort, velocity = limits[0]
    assert lower == 0.0 < upper
    assert effort > 0.0
    assert velocity > 0.0


def test_total_mass_is_the_datasheet_weight(robot):
    total = sum(
        float(link.find("inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("inertial") is not None
    )
    assert total == pytest.approx(GRIPPER_MASS, abs=1e-4)


def test_inertias_are_physically_valid(robot):
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        i = inertial.find("inertia").attrib
        tensor = np.array(
            [
                [float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                [float(i["ixz"]), float(i["iyz"]), float(i["izz"])],
            ]
        )
        principal = np.linalg.eigvalsh(tensor)
        assert (
            principal.min() > 0
        ), f"{link.get('name')} inertia is not positive definite"
        # The moments of a real body satisfy the triangle inequality.
        a, b, c = principal
        assert a + b >= c * (1 - 1e-6), f"{link.get('name')} inertia is not realisable"


def test_every_mesh_exists(robot):
    meshes = {m.get("filename") for m in robot.iter("mesh")}
    assert meshes, "no meshes in the description"
    prefix = "package://robotiq_description/"
    for filename in meshes:
        assert filename.startswith(prefix), filename
        assert (PKG_DIR / filename[len(prefix) :]).is_file(), filename


def test_declares_no_ros2_control_block(robot):
    # No driver or controller supports the C10 yet; a block here would claim otherwise.
    assert robot.find("ros2_control") is None


@requires_xacro
def test_the_shared_top_level_selects_it_with_gripper_model():
    robot = expand("gripper_model:=hand_e_c10", model="robotiq_gripper.urdf.xacro")
    names = {j.get("name") for j in robot.findall("joint")}
    assert "hande_c10_finger_1_joint" in names
