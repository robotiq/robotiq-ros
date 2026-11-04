#!/usr/bin/env python3
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

"""Expand released descriptions that include robotiq_description's macros.

usage: check_dependents.py <package>...

Each package is a released dependent the job installed; the script fails if
one is missing, or if any of its descriptions does not expand.
"""

import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

CLEARPATH = """<?xml version="1.0"?>
<robot name="clearpath" xmlns:xacro="http://ros.org/wiki/xacro">
  <link name="world"/>
  <xacro:include filename="$(find clearpath_manipulators_description)/urdf/gripper/robotiq_2f_85.urdf.xacro"/>
  <xacro:robotiq_2f_85 name="arm_0_gripper" parent_link="world">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_2f_85>
</robot>
"""

HUSARION = """<?xml version="1.0"?>
<robot name="husarion" xmlns:xacro="http://ros.org/wiki/xacro">
  <link name="world"/>
  <xacro:include filename="$(find husarion_components_description)/urdf/robotiq.urdf.xacro"/>
  <xacro:robotiq parent_link="world"/>
</robot>
"""

REAL = "robotiq_driver/RobotiqGripperHardwareInterface"
MOCK = "mock_components/GenericSystem"
GAZEBO = "gz_ros2_control/GazeboSimSystem"


def kinova_cases():
    robot = os.path.join(
        get_package_share_directory("kortex_description"), "robots/kinova.urdf.xacro"
    )
    for gripper in ["robotiq_2f_85", "robotiq_2f_140"]:
        for mode, plugin in [
            ([], REAL),
            (["use_fake_hardware:=true"], MOCK),
            (["sim_gazebo:=true"], GAZEBO),
        ]:
            arguments = ["name:=kinova_arm", "arm:=gen3", f"gripper:={gripper}", *mode]
            yield [robot, *arguments], plugin


def wrapper_cases(template, modes):
    with tempfile.NamedTemporaryFile("w", suffix=".urdf.xacro") as wrapper:
        wrapper.write(template)
        wrapper.flush()
        for mode, plugin in modes:
            yield [wrapper.name, *mode], plugin


def gripper_plugin(urdf):
    for ros2_control in ET.fromstring(urdf).iter("ros2_control"):
        joints = [joint.get("name") for joint in ros2_control.iter("joint")]
        if any(j.endswith(("left_knuckle_joint", "finger_joint")) for j in joints):
            return ros2_control.findtext("hardware/plugin")
    return None


CASES = {
    "kortex_description": kinova_cases,
    "clearpath_manipulators_description": lambda: wrapper_cases(
        CLEARPATH,
        [
            (["use_manipulation_controllers:=true"], REAL),
            (["use_manipulation_controllers:=true", "use_fake_hardware:=true"], MOCK),
            (["use_manipulation_controllers:=true", "is_sim:=true"], GAZEBO),
        ],
    ),
    "husarion_components_description": lambda: wrapper_cases(HUSARION, [([], GAZEBO)]),
}


def main(packages):
    share = get_package_share_directory("robotiq_description")
    print("robotiq_description:", share)
    if share.startswith("/opt/ros/"):
        print("FAIL robotiq_description resolves to the released package")
        return 1
    failed = 0
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            print(f"FAIL {package}: not installed")
            failed += 1
            continue
        for args, expected in CASES[package]():
            result = subprocess.run(["xacro", *args], capture_output=True, text=True)
            label = f"{package} {' '.join(args[1:]) or '(default)'}"
            if result.returncode != 0:
                print(f"FAIL {label}")
                print(result.stderr.strip())
                failed += 1
            elif (plugin := gripper_plugin(result.stdout)) != expected:
                print(f"FAIL {label}: gripper plugin {plugin}, expected {expected}")
                failed += 1
            else:
                print(f"pass {label}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
