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

//! Publishes the gripper joint's position as the normalized position, 0 = open
//! to 1 = closed, on joint_states, whatever the hardware plugin.

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Robotiq/gripper/device_profile.hpp>

namespace robotiq_controllers {

class NormalizedPositionBroadcaster : public controller_interface::ControllerInterface
{
public:
   controller_interface::InterfaceConfiguration command_interface_configuration() const override;

   controller_interface::InterfaceConfiguration state_interface_configuration() const override;

   controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   CallbackReturn on_init() override;

   CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

   CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
   std::string joint_;
   double closed_position_ = 0.0;
   Robotiq::DeviceProfile profile_;
   sensor_msgs::msg::JointState message_;
   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
   std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_publisher_;
};
} // namespace robotiq_controllers
