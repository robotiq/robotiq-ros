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

//! The joint's scale, gripper_closed_position and gripper_profile, as the
//! controllers find it: from the URDF from Jazzy on, from their own parameters
//! on Humble.

#pragma once

#include <limits>
#include <optional>
#include <string>

#include <Robotiq/gripper/device_profile.hpp>

#include "robotiq_controllers/object_status_goal.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"
#include "robotiq_driver/gripper_scaling.hpp"
#include "robotiq_driver/hardware_parameters.hpp"

namespace robotiq_controllers::joint_scale {

// Humble EOL: delete this function and the two parameter branches below.
template <class ControllerT>
void declareParameters(ControllerT& controller)
{
   if constexpr(!compat::detail::HasRobotDescription<ControllerT>::value)
   {
      controller.template auto_declare<double>(robotiq_driver::kClosedPositionParam,
                                               std::numeric_limits<double>::quiet_NaN());
      controller.template auto_declare<std::string>(robotiq_driver::kProfileParam, "");
   }
}

template <class ControllerT>
std::optional<double> closedPosition(ControllerT& controller, const std::string& joint)
{
   if constexpr(compat::detail::HasRobotDescription<ControllerT>::value)
   {
      return object_status_goal::closedPositionFromUrdf(controller.get_robot_description(), joint);
   }
   else
   {
      const double closed_position =
         controller.get_node()->get_parameter(robotiq_driver::kClosedPositionParam).as_double();
      return robotiq_driver::isValidClosedPosition(closed_position) ? std::optional(closed_position) : std::nullopt;
   }
}

template <class ControllerT>
std::optional<Robotiq::DeviceProfile> profile(ControllerT& controller, const std::string& joint)
{
   if constexpr(compat::detail::HasRobotDescription<ControllerT>::value)
   {
      return object_status_goal::profileFromUrdf(controller.get_robot_description(), joint);
   }
   else
   {
      const std::string name = controller.get_node()->get_parameter(robotiq_driver::kProfileParam).as_string();
      if(name.empty())
      {
         RCLCPP_WARN(controller.get_node()->get_logger(),
                     "No %s: assuming the 2F-85's. Deprecated since 1.4.0; it is required from the next major "
                     "release.",
                     robotiq_driver::kProfileParam);
         return Robotiq::profiles::k2F85;
      }
      return robotiq_driver::profileNamed(name);
   }
}

// The driver's full-scale speed or force, which \p from_urdf reads. Humble EOL:
// delete the Humble branch; its controller commands neither.
template <class ControllerT>
double fullScale([[maybe_unused]] ControllerT& controller,
                 [[maybe_unused]] const std::string& joint,
                 [[maybe_unused]] double (*from_urdf)(const std::string&, const std::string&),
                 [[maybe_unused]] double fallback)
{
   if constexpr(compat::detail::HasRobotDescription<ControllerT>::value)
   {
      return from_urdf(controller.get_robot_description(), joint);
   }
   else
   {
      return fallback;
   }
}
} // namespace robotiq_controllers::joint_scale
