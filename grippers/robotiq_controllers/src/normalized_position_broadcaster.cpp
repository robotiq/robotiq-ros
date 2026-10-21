// Copyright (c) 2026 Robotiq
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_controllers/normalized_position_broadcaster.hpp"

#include <string>

#include "robotiq_controllers/joint_scale.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"
#include "robotiq_driver/gripper_scaling.hpp"

namespace robotiq_controllers {
namespace {
constexpr const char* kJointParameter = "joint";
constexpr const char* kNormalizedJointParameter = "normalized_joint";
} // namespace

controller_interface::InterfaceConfiguration NormalizedPositionBroadcaster::command_interface_configuration() const
{
   return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration NormalizedPositionBroadcaster::state_interface_configuration() const
{
   return {controller_interface::interface_configuration_type::INDIVIDUAL, {joint_ + "/position"}};
}

controller_interface::return_type NormalizedPositionBroadcaster::update(const rclcpp::Time& time,
                                                                        const rclcpp::Duration& /*period*/)
{
   const std::optional<double> position = compat::getValue(state_interfaces_.at(0));
   const std::optional<double> normalized =
      position ? robotiq_driver::normalizedFromJointPosition(*position, closed_position_, profile_) : std::nullopt;
   if(normalized)
   {
      message_.header.stamp = time;
      message_.position.at(0) = *normalized;
      compat::tryPublish(*realtime_publisher_, message_);
   }
   return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizedPositionBroadcaster::on_init()
{
   try
   {
      auto_declare<std::string>(kJointParameter, "");
      auto_declare<std::string>(kNormalizedJointParameter, "robotiq_normalized_joint");
      joint_scale::declareParameters(*this);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare the parameters: %s", e.what());
      return CallbackReturn::ERROR;
   }
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizedPositionBroadcaster::on_configure(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   joint_ = get_node()->get_parameter(kJointParameter).as_string();
   const std::optional<double> closed_position = joint_scale::closedPosition(*this, joint_);
   const std::optional<Robotiq::DeviceProfile> device_profile = joint_scale::profile(*this, joint_);
   if(!closed_position || !device_profile)
   {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint '%s' has no usable %s and %s.",
                   joint_.c_str(),
                   robotiq_driver::kClosedPositionParam,
                   robotiq_driver::kProfileParam);
      return CallbackReturn::ERROR;
   }
   closed_position_ = *closed_position;
   profile_ = *device_profile;

   message_ = sensor_msgs::msg::JointState();
   message_.name = {get_node()->get_parameter(kNormalizedJointParameter).as_string()};
   message_.position = {0.0};
   publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
   realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(publisher_);
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NormalizedPositionBroadcaster::on_cleanup(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   realtime_publisher_.reset();
   publisher_.reset();
   return CallbackReturn::SUCCESS;
}
} // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::NormalizedPositionBroadcaster, controller_interface::ControllerInterface)
