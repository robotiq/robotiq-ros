# Copyright (c) 2022 PickNik, Inc.
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
#    * Neither the name of the {copyright_holder} nor the names of its
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

import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch_ros
import importlib.util
import os


def load_gripper_models():
    """launch/gripper_models.py, next to this file in the source and the install."""
    path = os.path.join(os.path.dirname(__file__), "gripper_models.py")
    spec = importlib.util.spec_from_file_location("robotiq_gripper_models", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def xacro_command(gripper_models):
    # Empty gripper_model leaves the description's own, so `model:=` alone renders that file.
    gripper_model_arg = {"": ""} | {
        model: f" gripper_model:={model}" for model in gripper_models.GRIPPER_JOINTS
    }
    return Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            gripper_models.PerGripperModel(
                gripper_model_arg, LaunchConfiguration("gripper_model")
            ),
        ]
    )


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robotiq_description"
    ).find("robotiq_description")
    gripper_models = load_gripper_models()
    gripper_model = LaunchConfiguration("gripper_model")

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="gripper_model",
            default_value="",
            choices=["", *sorted(gripper_models.GRIPPER_JOINTS)],
            description="Gripper whose description and RViz view to load; empty: the "
            "2F-85, or the gripper `model` names",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=[
                os.path.join(pkg_share, "urdf", "robotiq_"),
                gripper_models.PerGripperModel(
                    {"": "2f_85"}
                    | {model: model for model in gripper_models.GRIPPER_JOINTS},
                    gripper_model,
                ),
                "_gripper.urdf.xacro",
            ],
            description="Absolute path to gripper URDF file; defaults from gripper_model",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=PathJoinSubstitution(
                [
                    pkg_share,
                    "rviz",
                    gripper_models.PerGripperModel(
                        {"": "view_urdf.rviz"} | gripper_models.GRIPPER_RVIZ_CONFIGS,
                        gripper_model,
                    ),
                ]
            ),
            description="Absolute path to rviz config file; defaults from gripper_model",
        )
    )

    robot_description_content = xacro_command(gripper_models)
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
