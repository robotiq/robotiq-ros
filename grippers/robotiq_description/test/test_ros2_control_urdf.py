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

"""Expand the gripper xacro and check the ros2_control interfaces it declares.

The two hardware plugins need different interface sets, and getting it wrong is
silent until a controller fails to activate at runtime:

* The real driver exports set_gripper_max_effort / set_gripper_max_velocity
  itself rather than from the URDF, and its on_init rejects a
  joint that declares more than one command interface.
* Mock hardware exposes only what the URDF declares, so those two interfaces have
  to be declared there or parallel_gripper_action_controller cannot claim them and
  never activates under use_fake_hardware:=true.
* motor_current, object_status and the two fault fields are the reverse case:
  only the real driver writes them.
"""

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

PKG_DIR = Path(__file__).parents[1]
URDF_DIR = PKG_DIR / "urdf"
CONFIG = PKG_DIR / "config" / "robotiq_controllers.yaml"
TOPIC_BASED_CONFIG = PKG_DIR / "config" / "robotiq_controllers.topic_based.yaml"

# Top-level xacro -> the joint carrying the gripper's position command.
MODELS = {
    "robotiq_2f_85_gripper.urdf.xacro": "robotiq_85_left_knuckle_joint",
    "robotiq_2f_140_gripper.urdf.xacro": "finger_joint",
    "robotiq_hand_e_gripper.urdf.xacro": "hande_left_finger_joint",
    "robotiq_hand_e_c10_gripper.urdf.xacro": "hande_c10_finger_1_joint",
}

MOCK_PLUGIN = "mock_components/GenericSystem"
REAL_PLUGIN = "robotiq_driver/RobotiqGripperHardwareInterface"
TOPIC_BASED_PLUGIN = "topic_based_ros2_control/TopicBasedSystem"
TOPIC_BASED_ARG = "sim_topic_based:=true"

# Joints that follow the driven one through <mimic>: every other 2F finger
# joint, and the Hand-E's right finger.
MIMIC_JOINT_COUNTS = {
    "robotiq_2f_85_gripper.urdf.xacro": 5,
    "robotiq_2f_140_gripper.urdf.xacro": 5,
    "robotiq_hand_e_gripper.urdf.xacro": 1,
    # None under sim_topic_based: the C10 declares its followers for Gazebo only (#118).
    "robotiq_hand_e_c10_gripper.urdf.xacro": 0,
}

# Exported by the driver at runtime; needed from the URDF under mock hardware.
EXTRA_MOCK_COMMAND_INTERFACES = {"set_gripper_max_velocity", "set_gripper_max_effort"}

HARDWARE_ONLY_STATE_INTERFACES = {
    "motor_current",
    "object_status",
    "gripper_fault",
    "gripper_fault_severity",
}

requires_xacro = pytest.mark.skipif(
    shutil.which("xacro") is None, reason="xacro not on PATH"
)


def expand(model, use_fake_hardware, *extra_args):
    """Run xacro on `model` and return the parsed <ros2_control> element."""
    result = subprocess.run(
        [
            "xacro",
            str(URDF_DIR / model),
            f"use_fake_hardware:={str(use_fake_hardware).lower()}",
            *extra_args,
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    ros2_control = ET.fromstring(result.stdout).find("ros2_control")
    assert ros2_control is not None, f"{model} declares no <ros2_control> block"
    return ros2_control


def plugin_of(ros2_control):
    return ros2_control.find("hardware/plugin").text.strip()


def hardware_param(ros2_control, name):
    param = ros2_control.find(f"hardware/param[@name='{name}']")
    return None if param is None else param.text.strip()


def command_interfaces_of(ros2_control, joint_name):
    joint = ros2_control.find(f"joint[@name='{joint_name}']")
    assert joint is not None, f"joint {joint_name} missing from <ros2_control>"
    return {ci.get("name") for ci in joint.findall("command_interface")}


def state_interfaces_of(ros2_control, joint_name):
    joint = ros2_control.find(f"joint[@name='{joint_name}']")
    assert joint is not None, f"joint {joint_name} missing from <ros2_control>"
    return {si.get("name") for si in joint.findall("state_interface")}


def joints_of(ros2_control):
    return {j.get("name") for j in ros2_control.findall("joint")}


@requires_xacro
@pytest.mark.parametrize(
    "gripper_model,joint",
    [
        ("2f_140", "finger_joint"),
        ("hand_e", "hande_left_finger_joint"),
        ("hand_e_c10", "hande_c10_finger_1_joint"),
    ],
)
def test_gripper_model_selects_the_description_from_the_launch_default(
    gripper_model, joint
):
    # The launch loads robotiq_gripper.urdf.xacro and passes only gripper_model.
    ros2_control = expand(
        "robotiq_gripper.urdf.xacro", True, f"gripper_model:={gripper_model}"
    )
    assert joints_of(ros2_control) == {joint}


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_mock_declares_the_interfaces_the_gripper_controller_claims(model, joint):
    ros2_control = expand(model, use_fake_hardware=True)
    assert plugin_of(ros2_control) == MOCK_PLUGIN
    assert (
        command_interfaces_of(ros2_control, joint)
        == {"position"} | EXTRA_MOCK_COMMAND_INTERFACES
    )


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_real_hardware_declares_exactly_one_command_interface(model, joint):
    # RobotiqGripperHardwareInterface::on_init fails the component outright if the
    # joint declares anything other than a single position command interface.
    ros2_control = expand(model, use_fake_hardware=False)
    assert plugin_of(ros2_control) == REAL_PLUGIN
    assert command_interfaces_of(ros2_control, joint) == {"position"}


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_mock_publishes_the_same_joints_as_hardware(model, joint):
    # The mimic joints must NOT be declared under mock: mock_components would then
    # own them and publish them in /joint_states, which Humble reports as 0.0 while
    # Jazzy and Lyrical apply the URDF multipliers. Leaving them out makes
    # robot_state_publisher derive them from the <mimic> tags on every distro,
    # exactly as it already does for real hardware.
    # Humble EOL: simplify this comment; the mimic joints stay out either way.
    mock = joints_of(expand(model, use_fake_hardware=True))
    real = joints_of(expand(model, use_fake_hardware=False))
    assert mock == real == {joint}


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
@pytest.mark.parametrize(
    "use_fake_hardware,extra_args",
    [(False, ()), (True, ()), (False, (TOPIC_BASED_ARG,))],
)
def test_every_plugin_gets_the_joint_scale(model, use_fake_hardware, extra_args):
    ros2_control = expand(model, use_fake_hardware, *extra_args)
    assert hardware_param(ros2_control, "gripper_closed_position") is not None
    assert hardware_param(ros2_control, "gripper_max_speed") is not None
    assert hardware_param(ros2_control, "gripper_max_force") is not None


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_gripper_controller_config_interfaces_exist_under_mock(model, joint):
    # Ties config/robotiq_controllers.yaml to the mock URDF: these two parameters
    # name "<joint>/<interface>" pairs the controller claims, and an unmatched name
    # is why the controller used to fail to activate with use_fake_hardware:=true.
    params = yaml.safe_load(CONFIG.read_text())["robotiq_gripper_controller"][
        "ros__parameters"
    ]
    ros2_control = expand(model, use_fake_hardware=True)
    claimed = {
        params[k].replace("$(var gripper_joint)", joint)
        for k in ("max_effort_interface", "max_velocity_interface")
    }
    available = {
        f"{joint}/{name}" for name in command_interfaces_of(ros2_control, joint)
    }

    assert (
        claimed <= available
    ), f"controller claims {claimed - available}, which the mock URDF does not declare"


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_only_real_hardware_declares_the_status_block_interfaces(model, joint):
    real = state_interfaces_of(expand(model, use_fake_hardware=False), joint)
    mock = state_interfaces_of(expand(model, use_fake_hardware=True), joint)

    assert HARDWARE_ONLY_STATE_INTERFACES <= real
    assert HARDWARE_ONLY_STATE_INTERFACES.isdisjoint(mock)
    assert real - HARDWARE_ONLY_STATE_INTERFACES == mock == {"position", "velocity"}


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
def test_baudrate_reaches_the_driver(model):
    # The launch argument crosses three xacro files to get here, and a half
    # updated chain fails at runtime with `Invalid parameter "baudrate"` rather
    # than at expansion.
    ros2_control = expand(model, False, "baudrate:=57600")
    assert hardware_param(ros2_control, "baudrate") == "57600"

    assert hardware_param(expand(model, False), "baudrate") == "115200"
    assert hardware_param(expand(model, True), "baudrate") is None


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
def test_slave_address_reaches_the_driver(model):
    ros2_control = expand(model, False, "slave_address:=2")
    assert hardware_param(ros2_control, "slave_address") == "2"

    ros2_control_hex = expand(model, False, "slave_address:=0x0A")
    assert hardware_param(ros2_control_hex, "slave_address") == "0x0A"

    assert hardware_param(expand(model, False), "slave_address") == "9"
    assert hardware_param(expand(model, True), "slave_address") is None


@requires_xacro
@pytest.mark.parametrize(
    "model,profile",
    [
        ("robotiq_2f_85_gripper.urdf.xacro", "2f_85"),
        ("robotiq_2f_140_gripper.urdf.xacro", "2f_140"),
        ("robotiq_hand_e_gripper.urdf.xacro", "hand_e"),
        ("robotiq_hand_e_c10_gripper.urdf.xacro", "hand_e_c10"),
    ],
)
def test_every_plugin_gets_the_models_sdk_profile(model, profile):
    assert hardware_param(expand(model, False), "gripper_profile") == profile
    assert hardware_param(expand(model, True), "gripper_profile") == profile


@requires_xacro
def test_the_hand_e_driver_closes_at_one_finger_s_travel():
    ros2_control = expand("robotiq_hand_e_gripper.urdf.xacro", False)
    assert float(hardware_param(ros2_control, "gripper_closed_position")) == 0.025


@requires_xacro
def test_the_hand_e_c10_driver_closes_at_one_finger_s_travel():
    ros2_control = expand("robotiq_hand_e_c10_gripper.urdf.xacro", False)
    assert float(hardware_param(ros2_control, "gripper_closed_position")) == 0.0192


def expand_robot(model):
    """The whole description of `model` under the real driver, parsed."""
    result = subprocess.run(
        ["xacro", str(URDF_DIR / model), "use_fake_hardware:=false"],
        capture_output=True,
        text=True,
        check=True,
    )
    return ET.fromstring(result.stdout)


@requires_xacro
@pytest.mark.parametrize(
    "model,joint",
    [
        ("robotiq_hand_e_gripper.urdf.xacro", "hande_left_finger_joint"),
        ("robotiq_hand_e_c10_gripper.urdf.xacro", "hande_c10_finger_1_joint"),
    ],
)
def test_the_prismatic_driver_s_full_speed_is_the_finger_joint_s_limit(model, joint):
    robot = expand_robot(model)
    limit = robot.find(f"joint[@name='{joint}']/limit")
    max_speed = hardware_param(robot.find("ros2_control"), "gripper_max_speed")
    assert float(max_speed) == float(limit.get("velocity"))


@requires_xacro
@pytest.mark.parametrize(
    "model,floor",
    [
        ("robotiq_2f_85_gripper.urdf.xacro", 0.020),
        ("robotiq_2f_140_gripper.urdf.xacro", 0.030),
        ("robotiq_hand_e_gripper.urdf.xacro", 0.010),
        ("robotiq_hand_e_c10_gripper.urdf.xacro", 0.007),
    ],
)
def test_each_model_sets_its_speed_floor(model, floor):
    ros2_control = expand_robot(model).find("ros2_control")
    assert float(hardware_param(ros2_control, "gripper_min_speed")) == floor


@requires_xacro
def test_the_hand_e_c10_driver_s_full_force_is_the_finger_joint_s_limit():
    robot = expand_robot("robotiq_hand_e_c10_gripper.urdf.xacro")
    limit = robot.find("joint[@name='hande_c10_finger_1_joint']/limit")
    max_force = hardware_param(robot.find("ros2_control"), "gripper_max_force")
    assert float(max_force) == float(limit.get("effort"))


@requires_xacro
def test_the_hand_e_joint_is_0_open_and_closes_as_it_grows():
    result = subprocess.run(
        ["xacro", str(URDF_DIR / "robotiq_hand_e_gripper.urdf.xacro")],
        capture_output=True,
        text=True,
        check=True,
    )
    robot = ET.fromstring(result.stdout)
    right_mimic = robot.find("joint[@name='hande_right_finger_joint']/mimic")
    assert right_mimic.get("joint") == "hande_left_finger_joint"
    assert float(right_mimic.get("multiplier", "1")) == 1.0
    assert float(right_mimic.get("offset", "0")) == 0.0

    def finger_x(name, travel):
        joint = robot.find(f"joint[@name='{name}']")
        origin_x = float(joint.find("origin").get("xyz").split()[0])
        axis_x = float(joint.find("axis").get("xyz").split()[0])
        return origin_x + axis_x * travel

    def pad_gap(travel):
        return finger_x("hande_right_finger_joint", travel) - finger_x(
            "hande_left_finger_joint", travel
        )

    assert pad_gap(0.0) == pytest.approx(0.050)
    assert pad_gap(0.025) == pytest.approx(0.0)


GAZEBO_C10 = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="gazebo_c10">
    <xacro:include filename="{include}" />
    <link name="world" />
    <xacro:robotiq_gripper name="Gripper" prefix="" parent="world" sim_gazebo="true"
        use_fake_hardware="false">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
</robot>
"""


LIMIT_ARGUMENTS_C10 = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="limits_c10">
    <xacro:include filename="{include}" />
    <link name="world" />
    <xacro:robotiq_gripper name="Gripper" prefix="" parent="world"
        use_fake_hardware="false" gripper_max_speed="0.05" gripper_max_force="100.0"
        gripper_closed_position="0.01">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
</robot>
"""


@requires_xacro
def test_the_hand_e_c10_macro_takes_the_driver_limit_arguments_like_the_other_models(
    tmp_path,
):
    # A user xacro that passes them to any other model must work with this one too.
    description = tmp_path / "limits.urdf.xacro"
    description.write_text(
        LIMIT_ARGUMENTS_C10.format(
            include=URDF_DIR / "robotiq_hand_e_c10_macro.urdf.xacro"
        )
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    ros2_control = ET.fromstring(result.stdout).find("ros2_control")
    assert hardware_param(ros2_control, "gripper_max_speed") == "0.05"
    assert hardware_param(ros2_control, "gripper_max_force") == "100.0"
    assert hardware_param(ros2_control, "gripper_closed_position") == "0.01"


@requires_xacro
def test_a_zero_speed_floor_reaches_the_hand_e_c10_driver(tmp_path):
    description = tmp_path / "floor.urdf.xacro"
    description.write_text(
        LIMIT_ARGUMENTS_C10.replace(
            'gripper_max_speed="0.05"',
            'gripper_min_speed="0.0" gripper_max_speed="0.05"',
        ).format(include=URDF_DIR / "robotiq_hand_e_c10_macro.urdf.xacro")
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    ros2_control = ET.fromstring(result.stdout).find("ros2_control")
    assert float(hardware_param(ros2_control, "gripper_min_speed")) == 0.0


@requires_xacro
def test_the_hand_e_c10_followers_are_declared_for_gazebo_only(tmp_path):
    # Declared mimic joints under sim_topic_based make the simulator's command name more
    # joints than it gives positions for (#118), so only Gazebo, which derives nothing
    # itself, gets them; /joint_states carries them from <mimic> everywhere else.
    followers = {"hande_c10_finger_2_joint", "hande_c10_finger_3_joint"}
    model = "robotiq_hand_e_c10_gripper.urdf.xacro"
    assert followers.isdisjoint(joints_of(expand(model, False, TOPIC_BASED_ARG)))
    assert followers.isdisjoint(joints_of(expand(model, False)))
    assert followers.isdisjoint(joints_of(expand(model, True)))

    description = tmp_path / "gazebo.urdf.xacro"
    description.write_text(
        GAZEBO_C10.format(include=URDF_DIR / "robotiq_hand_e_c10_macro.urdf.xacro")
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    ros2_control = ET.fromstring(result.stdout).find("ros2_control")
    assert followers <= joints_of(ros2_control)


def _link_pose(robot, link, driven_joint, position):
    joints = {
        joint.find("child").get("link"): joint for joint in robot.findall("joint")
    }
    pose = np.eye(4)
    while link in joints:
        joint = joints[link]
        origin = joint.find("origin")
        step = np.eye(4)
        # URDF's rpy turns about the fixed axes, scipy's lowercase "xyz".
        step[:3, :3] = Rotation.from_euler(
            "xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]
        ).as_matrix()
        step[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
        if joint.get("type") == "revolute":
            mimic = joint.find("mimic")
            angle = position
            if joint.get("name") != driven_joint:
                assert (
                    mimic is not None
                ), f"{joint.get('name')} is neither driven nor a mimic"
                angle = position * float(mimic.get("multiplier", "1")) + float(
                    mimic.get("offset", "0")
                )
            turn = np.eye(4)
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            turn[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            step = step @ turn
        pose = step @ pose
        link = joint.find("parent").get("link")
    return pose


@pytest.fixture(scope="module")
def robot_2f_140():
    result = subprocess.run(
        [
            "xacro",
            str(URDF_DIR / "robotiq_2f_140_gripper.urdf.xacro"),
            "use_fake_hardware:=false",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return ET.fromstring(result.stdout)


def _pad_poses(robot, position):
    base = np.linalg.inv(
        _link_pose(robot, "robotiq_140_base_link", "finger_joint", position)
    )
    return (
        base @ _link_pose(robot, "left_inner_finger_pad", "finger_joint", position),
        base @ _link_pose(robot, "right_inner_finger_pad", "finger_joint", position),
    )


@requires_xacro
def test_the_2f_140_pads_span_the_stroke_up_to_the_driver_s_closed_position(
    robot_2f_140,
):
    robot = robot_2f_140
    closed = float(
        hardware_param(robot.find("ros2_control"), "gripper_closed_position")
    )
    upper = float(robot.find("joint[@name='finger_joint']/limit").get("upper"))

    # The pad frames sit on the contact faces, z pointing away from the other pad.
    def pad_gap(position):
        left, right = _pad_poses(robot, position)
        return np.dot(right[:3, 3] - left[:3, 3], -left[:3, 2])

    assert closed <= upper
    assert pad_gap(0.0) == pytest.approx(0.140, abs=0.001)
    # The closed position is fitted to a real 2F-140, whose pads meet a little
    # later in the stroke than the model's.
    assert -0.0015 < pad_gap(closed) <= 0.0


@requires_xacro
@pytest.mark.parametrize("position", [0.0, 0.385, 0.77])
def test_the_2f_140_pads_mirror_each_other(robot_2f_140, position):
    left, right = _pad_poses(robot_2f_140, position)
    assert right[:3, 3] == pytest.approx(left[:3, 3] * [1, -1, 1], abs=1e-9)
    assert right[:3, :3] == pytest.approx(np.diag([-1, -1, 1]) @ left[:3, :3], abs=1e-9)


def _stl_vertices(path):
    """The vertices of a binary STL, one row each."""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    facets = np.frombuffer(
        data[84 : 84 + 50 * count],
        dtype=np.dtype([("normal", "<3f4"), ("vertices", "<9f4"), ("attr", "<u2")]),
    )
    return facets["vertices"].reshape(-1, 3)


@requires_xacro
@pytest.mark.parametrize("side", ["left", "right"])
def test_the_2f_140_pad_inertia_lies_along_its_mesh(robot_2f_140, side):
    # A box of the pad's extents has its moments in the same order as the
    # pad's own: the smallest about its long side, along the finger.
    link = robot_2f_140.find(f"link[@name='{side}_inner_finger_pad']")
    inertia = link.find("inertial/inertia")
    moments = [float(inertia.get(axis)) for axis in ("ixx", "iyy", "izz")]
    mesh = link.find("collision/geometry/mesh").get("filename")
    vertices = _stl_vertices(
        PKG_DIR / mesh.removeprefix("package://robotiq_description/")
    )
    x, y, z = vertices.max(axis=0) - vertices.min(axis=0)
    box = [y**2 + z**2, x**2 + z**2, x**2 + y**2]
    assert list(np.argsort(moments)) == list(np.argsort(box))


# How robotiq_2f_140_macro.urdf.xacro called the finger macros in 1.3.
FINGERS_1_3_CALL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="fingers_1_3">
    <xacro:include filename="{include}" />
    <link name="robotiq_140_base_link" />
    <xacro:finger_links prefix="" fingerprefix="left" stroke="140"/>
    <xacro:finger_links prefix="" fingerprefix="right" stroke="140"/>
    <xacro:finger_joint prefix=""/>
    <xacro:right_outer_knuckle_joint prefix=""/>
</robot>
"""


@requires_xacro
def test_the_2f_140_finger_macros_take_a_1_3_call_and_warn(tmp_path):
    description = tmp_path / "fingers.urdf.xacro"
    description.write_text(
        FINGERS_1_3_CALL.format(include=URDF_DIR / "robotiq_2f_140.xacro")
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    assert "finger_joint" in joints_of(ET.fromstring(result.stdout))
    assert "stroke argument of finger_links is deprecated" in result.stderr


# The 2F-140 meshes 1.1 to 1.3 installed under names the description no longer
# uses, which descriptions copied from those releases may still point at.
MESHES_1_3_2F_140 = [
    f"meshes/{kind}/2f_140/robotiq_2f_140_{part}.stl"
    for kind in ("visual", "collision")
    for part in (
        "base_link",
        "coupling",
        "inner_finger",
        "inner_knuckle",
        "outer_finger",
        "outer_knuckle",
    )
]


def test_every_2f_140_mesh_1_3_installed_is_still_shipped():
    # CMakeLists.txt installs meshes/ as it is, so the source tree is what ships.
    missing = [name for name in MESHES_1_3_2F_140 if not (PKG_DIR / name).is_file()]
    assert missing == []


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_sim_plugin_declares_only_the_position_command(model, joint):
    # The topic_based plugin does not know the driver's set_gripper_max_* interfaces;
    # config/robotiq_controllers.topic_based.yaml therefore claims none, and the URDF must
    # not declare them either or the plugin rejects the joint.
    ros2_control = expand(model, False, TOPIC_BASED_ARG)
    assert plugin_of(ros2_control) == TOPIC_BASED_PLUGIN
    assert command_interfaces_of(ros2_control, joint) == {"position"}


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_sim_plugin_declares_no_reactivate_gpio(model, joint):
    # robotiq_control.launch.py skips robotiq_activation_controller under sim_topic_based;
    # this is the URDF side of that agreement.
    assert expand(model, False, TOPIC_BASED_ARG).find("gpio") is None
    assert expand(model, False).find("gpio[@name='reactivate_gripper']") is not None
    assert expand(model, True).find("gpio[@name='reactivate_gripper']") is not None


@requires_xacro
def test_the_deprecated_2f_gripper_file_expands_like_robotiq_gripper():
    def expanded(name):
        result = subprocess.run(
            ["xacro", str(URDF_DIR / name)], capture_output=True, text=True, check=True
        )
        return ET.tostring(ET.fromstring(result.stdout))

    assert expanded("robotiq_2f_gripper.urdf.xacro") == expanded(
        "robotiq_gripper.urdf.xacro"
    )


HARDWARE_MACRO_CALL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="calls">
    <xacro:include filename="{include}" />
    <ros2_control name="Gripper" type="system">
        <xacro:{hardware} sim_gazebo="false" sim_topic_based="true"
            sim_joint_commands_topic="/commands" sim_joint_states_topic="/states"
            use_fake_hardware="false" mock_sensor_commands="false"
            com_port="/dev/ttyUSB0" baudrate="115200"
            gripper_speed_multiplier="1.0" gripper_force_multiplier="0.5"
            gripper_max_speed="0.150" gripper_max_force="235.0"
            gripper_closed_position="0.7929" />
        <xacro:{mimic} name="mimic_joint" sim_gazebo="false" />
    </ros2_control>
</robot>
"""


@requires_xacro
def test_the_deprecated_2f_hardware_macros_expand_like_the_shared_ones(tmp_path):
    def expanded(include, hardware, mimic):
        description = tmp_path / f"{hardware}.urdf.xacro"
        description.write_text(
            HARDWARE_MACRO_CALL.format(
                include=URDF_DIR / include, hardware=hardware, mimic=mimic
            )
        )
        result = subprocess.run(
            ["xacro", str(description)], capture_output=True, text=True, check=True
        )
        return ET.tostring(ET.fromstring(result.stdout).find("ros2_control"))

    assert expanded(
        "2f_common.ros2_control.xacro",
        "robotiq_2f_hardware",
        "robotiq_2f_sim_mimic_joint",
    ) == expanded(
        "common.ros2_control.xacro", "robotiq_hardware", "robotiq_sim_mimic_joint"
    )


TWO_GRIPPERS = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="two_grippers">
    <xacro:include filename="{macro}" />
    <link name="world" />
    <xacro:robotiq_gripper name="LeftGripper" prefix="left_" parent="world" {args}>
        <origin xyz="0 0.5 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
    <xacro:robotiq_gripper name="RightGripper" prefix="right_" parent="world" {args}>
        <origin xyz="0 -0.5 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
</robot>
"""


def interface_names(ros2_control):
    """Every `<component>/<interface>` the block declares, as ros2_control keys
    them, paired with its kind: a joint's position is both a command and a state."""
    return [
        (interface.tag, f"{component.get('name')}/{interface.get('name')}")
        for component in ros2_control
        if component.tag in ("joint", "gpio")
        for interface in component
        if interface.tag in ("command_interface", "state_interface")
    ]


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
@pytest.mark.parametrize("use_fake_hardware", [False, True])
def test_two_prefixed_grippers_declare_distinct_interfaces(
    tmp_path, model, use_fake_hardware
):
    # Issue #131: the reactivate_gripper GPIO used to ignore the prefix, so a
    # second gripper redeclared the first one's interfaces and the resource
    # manager refused the description.
    macro = URDF_DIR / model.replace("_gripper.urdf.xacro", "_macro.urdf.xacro")
    args = f'use_fake_hardware="{str(use_fake_hardware).lower()}"'
    description = tmp_path / "two_grippers.urdf.xacro"
    description.write_text(TWO_GRIPPERS.format(macro=macro, args=args))

    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    blocks = ET.fromstring(result.stdout).findall("ros2_control")
    assert [block.get("name") for block in blocks] == ["LeftGripper", "RightGripper"]

    names = [name for block in blocks for name in interface_names(block)]
    assert len(names) == len(set(names))
    for block, prefix in zip(blocks, ["left_", "right_"]):
        assert block.find(f"gpio[@name='{prefix}reactivate_gripper']") is not None


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_sim_topic_based_declares_the_mimic_joints_as_state_only(model, joint):
    # TopicBasedSystem reports only the joints declared here, and the simulator
    # owns the mimic joints, so they must be declared for /joint_states to carry
    # them, but with no command interface: only the knuckle is driven.
    ros2_control = expand(model, False, "sim_topic_based:=true")
    mimic = joints_of(ros2_control) - {joint}
    assert len(mimic) == MIMIC_JOINT_COUNTS[model]
    for name in mimic:
        assert command_interfaces_of(ros2_control, name) == set()
        assert state_interfaces_of(ros2_control, name) == {"position", "velocity"}


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
def test_sim_topics_reach_the_plugin(model):
    # Same three-file chain as baudrate: a half-forwarded argument only shows up
    # as a simulator that never hears a command.
    ros2_control = expand(
        model,
        False,
        "sim_topic_based:=true",
        "sim_joint_commands_topic:=/sim/cmd",
        "sim_joint_states_topic:=/sim/state",
    )
    assert hardware_param(ros2_control, "joint_commands_topic") == "/sim/cmd"
    assert hardware_param(ros2_control, "joint_states_topic") == "/sim/state"

    default = expand(model, False, "sim_topic_based:=true")
    assert hardware_param(default, "joint_commands_topic") == "/sim/joint_commands"
    assert hardware_param(default, "joint_states_topic") == "/sim/joint_states"


def xacro_stderr(model, *extra_args):
    return subprocess.run(
        ["xacro", str(URDF_DIR / model), *extra_args],
        capture_output=True,
        text=True,
        check=True,
    ).stderr


@requires_xacro
@pytest.mark.parametrize("model", MODELS)
def test_deprecated_isaac_arguments_still_select_the_plugin_and_warn(model):
    # PickNik's 1.1.0 names: same plugin, the /isaac_* topic defaults they had,
    # an explicit isaac_* topic still wins, and xacro says so on stderr.
    ros2_control = expand(model, False, "sim_isaac:=true")
    assert plugin_of(ros2_control) == TOPIC_BASED_PLUGIN
    assert (
        hardware_param(ros2_control, "joint_commands_topic") == "/isaac_joint_commands"
    )
    assert hardware_param(ros2_control, "joint_states_topic") == "/isaac_joint_states"

    overridden = expand(model, False, "sim_isaac:=true", "isaac_joint_commands:=/x")
    assert hardware_param(overridden, "joint_commands_topic") == "/x"

    assert "sim_isaac" in xacro_stderr(model, "sim_isaac:=true")
    assert "deprecated" in xacro_stderr(model, "sim_isaac:=true")
    assert "deprecated" not in xacro_stderr(model, TOPIC_BASED_ARG)


MACRO_CALL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="macro_call">
    <xacro:include filename="{include}" />
    <link name="world" />
    <xacro:robotiq_gripper name="Gripper" prefix="" parent="world" {arguments}>
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>
</robot>
"""


def expand_macro(tmp_path, macro, arguments):
    description = tmp_path / "macro_call.urdf.xacro"
    description.write_text(
        MACRO_CALL.format(include=URDF_DIR / macro, arguments=arguments)
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    block = ET.fromstring(result.stdout).find("ros2_control")
    return ET.tostring(block), result.stderr


@requires_xacro
@pytest.mark.parametrize(
    "macro", ["robotiq_2f_85_macro.urdf.xacro", "robotiq_2f_140_macro.urdf.xacro"]
)
@pytest.mark.parametrize(
    "deprecated,current",
    [
        ('sim_ignition="true"', 'sim_gazebo="true"'),
        (
            'use_fake_hardware="true" fake_sensor_commands="true"',
            'use_fake_hardware="true" mock_sensor_commands="true"',
        ),
    ],
)
def test_picknik_0_0_1_arguments_expand_like_their_current_names_and_warn(
    tmp_path, macro, deprecated, current
):
    # Clearpath, Husarion and Kinova's Humble release call the macro with these.
    deprecated_block, deprecated_stderr = expand_macro(tmp_path, macro, deprecated)
    current_block, current_stderr = expand_macro(tmp_path, macro, current)
    assert deprecated_block == current_block
    assert "deprecated" in deprecated_stderr
    assert "deprecated" not in current_stderr


@requires_xacro
@pytest.mark.parametrize("model,joint", MODELS.items())
def test_sim_controller_config_claims_only_what_the_sim_urdf_declares(model, joint):
    # The sim_topic_based counterpart of test_gripper_controller_config_interfaces_exist_under_mock.
    params = yaml.safe_load(TOPIC_BASED_CONFIG.read_text())[
        "robotiq_gripper_controller"
    ]["ros__parameters"]
    ros2_control = expand(model, False, TOPIC_BASED_ARG)

    assert params["joint"] == "$(var gripper_joint)"
    assert "max_effort_interface" not in params
    assert "max_velocity_interface" not in params
    assert set(params["state_interfaces"]) <= state_interfaces_of(ros2_control, joint)


# Every file PickNik's 0.0.1 release installs, which descriptions written
# against it may still include or point at.
PICKNIK_0_0_1_FILES = [
    "launch/view_gripper.launch.py",
    "rviz/view_urdf.rviz",
    "urdf/robotiq_2f_85_gripper.urdf.xacro",
    "urdf/robotiq_2f_85_macro.urdf.xacro",
    "urdf/robotiq_gripper.ros2_control.xacro",
    "urdf/ur_to_robotiq_adapter.urdf.xacro",
    *[
        f"meshes/{kind}/{part}.{extension}"
        for kind, extension in [("visual", "dae"), ("collision", "stl")]
        for part in [
            "left_finger",
            "left_finger_tip",
            "left_inner_knuckle",
            "left_knuckle",
            "right_finger",
            "right_finger_tip",
            "right_inner_knuckle",
            "right_knuckle",
            "robotiq_base",
            "ur_to_robotiq_adapter",
        ]
    ],
]


def test_every_file_picknik_0_0_1_installs_is_still_installed():
    share = Path(get_package_share_directory("robotiq_description"))
    # Not PickNik's own release, which would pass trivially.
    assert (share / "package.xml").read_text() == (PKG_DIR / "package.xml").read_text()
    missing = [name for name in PICKNIK_0_0_1_FILES if not (share / name).is_file()]
    assert missing == []


ROS2_CONTROL_CALL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ros2_control_call">
    <xacro:include filename="{include}" />
    <xacro:robotiq_gripper_ros2_control name="Gripper" prefix="" {arguments} />
</robot>
"""


def expand_ros2_control(tmp_path, include, arguments):
    description = tmp_path / "ros2_control_call.urdf.xacro"
    description.write_text(
        ROS2_CONTROL_CALL.format(include=URDF_DIR / include, arguments=arguments)
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    block = ET.fromstring(result.stdout).find("ros2_control")
    return ET.tostring(block), result.stderr


OLD_SIM_WARNING = "sim_ignition and fake_sensor_commands are deprecated"
ISAAC_WARNING = "sim_isaac, isaac_joint_commands and isaac_joint_states are deprecated"
ISAAC_TOPIC_BASED = (
    'sim_topic_based="true" sim_joint_commands_topic="/isaac_joint_commands"'
    ' sim_joint_states_topic="/isaac_joint_states"'
)


@requires_xacro
@pytest.mark.parametrize(
    "deprecated,current,warning",
    [
        ("", "", None),
        ('sim_ignition="true"', 'sim_gazebo="true"', OLD_SIM_WARNING),
        ('sim_isaac="true"', ISAAC_TOPIC_BASED, ISAAC_WARNING),
        (
            'use_fake_hardware="true" fake_sensor_commands="true"',
            'use_fake_hardware="true" mock_sensor_commands="true"',
            OLD_SIM_WARNING,
        ),
    ],
)
def test_the_picknik_0_0_1_ros2_control_file_and_names_expand_like_the_current_ones(
    tmp_path, deprecated, current, warning
):
    deprecated_block, deprecated_stderr = expand_ros2_control(
        tmp_path, "robotiq_gripper.ros2_control.xacro", deprecated
    )
    current_block, current_stderr = expand_ros2_control(
        tmp_path, "2f_85.ros2_control.xacro", current
    )
    assert deprecated_block == current_block
    assert "robotiq_gripper.ros2_control.xacro is deprecated" in deprecated_stderr
    if warning:
        assert warning in deprecated_stderr
    assert "deprecated" not in current_stderr


@requires_xacro
@pytest.mark.parametrize(
    "include", ["2f_85.ros2_control.xacro", "2f_140.ros2_control.xacro"]
)
@pytest.mark.parametrize(
    "deprecated,current",
    [
        ('sim_isaac="true"', ISAAC_TOPIC_BASED),
        (
            'sim_isaac="true" isaac_joint_commands="/c" isaac_joint_states="/s"',
            'sim_topic_based="true" sim_joint_commands_topic="/c"'
            ' sim_joint_states_topic="/s"',
        ),
        (
            'use_fake_hardware="true" isaac_joint_commands="/c"',
            'use_fake_hardware="true" sim_joint_commands_topic="/c"',
        ),
    ],
)
def test_picknik_main_isaac_arguments_expand_like_their_current_names_and_warn(
    tmp_path, include, deprecated, current
):
    deprecated_block, deprecated_stderr = expand_ros2_control(
        tmp_path, include, deprecated
    )
    current_block, current_stderr = expand_ros2_control(tmp_path, include, current)
    assert deprecated_block == current_block
    assert ISAAC_WARNING in deprecated_stderr
    assert "deprecated" not in current_stderr


ADAPTER_CALL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="adapter_call">
    <xacro:include filename="{include}" />
    <link name="tool0" />
    <xacro:ur_to_robotiq prefix="arm_" {arguments} />
</robot>
"""


def expand_adapter(tmp_path, arguments):
    description = tmp_path / "adapter_call.urdf.xacro"
    description.write_text(
        ADAPTER_CALL.format(
            include=URDF_DIR / "ur_to_robotiq_adapter.urdf.xacro", arguments=arguments
        )
    )
    result = subprocess.run(
        ["xacro", str(description)], capture_output=True, text=True, check=True
    )
    return ET.tostring(ET.fromstring(result.stdout)), result.stderr


@requires_xacro
def test_the_picknik_0_0_1_adapter_call_expands_like_the_current_one(tmp_path):
    deprecated_robot, deprecated_stderr = expand_adapter(
        tmp_path, 'connected_to="tool0"'
    )
    current_robot, current_stderr = expand_adapter(
        tmp_path, 'parent="tool0" child="arm_gripper_mount_link"'
    )
    assert deprecated_robot == current_robot
    assert "connected_to is deprecated" in deprecated_stderr
    assert "deprecated" not in current_stderr
