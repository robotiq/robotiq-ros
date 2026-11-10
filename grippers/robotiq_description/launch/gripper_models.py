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

"""Per-model values shared by robotiq_control.launch.py and view_gripper.launch.py."""

from launch.substitution import Substitution

# Joint carrying the position command, per gripper_model. The launch argument
# is restricted to these keys, so an unknown model fails at launch instead of
# as a controller that never activates.
GRIPPER_JOINTS = {
    "2f_85": "robotiq_85_left_knuckle_joint",
    "2f_140": "finger_joint",
    "hand_e": "hande_left_finger_joint",
    "hand_e_c10": "hande_c10_finger_1_joint",
}

# goal_tolerance, in the driven joint's unit: radians on the 2F models, metres
# on the Hand-E and the C10. The stock check ends a goal once inside it even
# when the object status decides, so it has to be small against the stroke.
GRIPPER_GOAL_TOLERANCES = {
    "2f_85": "0.02",
    "2f_140": "0.02",
    "hand_e": "0.0005",
    "hand_e_c10": "0.0005",
}

# stall_velocity_threshold for the topic-based simulation, in the driven joint's
# unit per second, well under the slowest finger speed: about 0.2 rad/s on the
# 2F models, 0.010 m/s on the Hand-E, 0.007 m/s on the C10.
GRIPPER_STALL_VELOCITY_THRESHOLDS = {
    "2f_85": "0.05",
    "2f_140": "0.05",
    "hand_e": "0.002",
    "hand_e_c10": "0.002",
}

# The RViz config under rviz/ that frames each gripper.
GRIPPER_RVIZ_CONFIGS = {
    "2f_85": "view_urdf.rviz",
    "2f_140": "view_urdf_2f_140.rviz",
    "hand_e": "view_urdf.rviz",
    "hand_e_c10": "view_urdf.rviz",
}


class PerGripperModel(Substitution):
    """The value `table` gives the gripper_model launch configuration."""

    def __init__(self, table, gripper_model):
        super().__init__()
        self.table = table
        self.gripper_model = gripper_model

    def describe(self):
        values = ", ".join(
            f"{model}: {value}" for model, value in self.table.items() if model
        )
        return f"by {self.gripper_model.describe()} ({values})"

    def perform(self, context):
        return self.table[self.gripper_model.perform(context)]
