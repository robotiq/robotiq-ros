// Copyright (c) 2022 PickNik, Inc.
// Copyright (c) 2026 Robotiq, Inc.
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

//! \brief Conversions between ros2_control's SI joint quantities and the
//!        gripper's 0..255 register counts.
//! Pulled out of the hardware interface so the arithmetic — the part that
//! silently mis-commands a gripper when it is wrong — is unit-testable
//! without a gripper, a serial port, or a controller manager.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>

#include <Robotiq/gripper/connection_state.hpp>
#include <Robotiq/gripper/device_profile.hpp>
#include <Robotiq/gripper/fault_status.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/status.hpp>
#include <Robotiq/gripper/units.hpp>

namespace robotiq_driver {

// The URDF hardware parameter carrying the closed_position the conversions below take.
inline constexpr const char* kClosedPositionParam = "gripper_closed_position";

// Whether the conversions below can take \p closed_position: zero and
// non-finite are the two they cannot divide by. Negative is fine, and is how a
// joint that closes in the negative direction is described.
[[nodiscard]] inline bool isValidClosedPosition(double closed_position)
{
   return std::isfinite(closed_position) && closed_position != 0.0;
}

// The URDF hardware parameter naming the SDK profile the conversions below take;
// a description without it gets the 2F-85's.
inline constexpr const char* kProfileParam = "gripper_profile";

// The SDK profile \p name selects, or nothing for a name it does not know.
// Surrounding whitespace is ignored, as a formatted URDF parameter carries it.
[[nodiscard]] inline std::optional<Robotiq::DeviceProfile> profileNamed(std::string_view name)
{
   constexpr std::string_view kWhitespace = " \t\n\r";
   name.remove_prefix(std::min(name.find_first_not_of(kWhitespace), name.size()));
   name.remove_suffix(name.size() - std::min(name.find_last_not_of(kWhitespace) + 1, name.size()));
   if(name == "2f_85")
   {
      return Robotiq::profiles::k2F85;
   }
   if(name == "hand_e")
   {
      return Robotiq::profiles::kHandE;
   }
   if(name == "hand_e_c10")
   {
      return Robotiq::profiles::kHandEC10;
   }
   return std::nullopt;
}

//! Register count (gPO: 0 open .. 255 closed) -> joint position, linear over
//! \p profile's travel band. \p closed_position is the joint value at a fully
//! closed gripper and sets the unit: the mapping carries whatever the URDF
//! uses. The shipped 2F descriptions model the knuckle as a revolute joint,
//! so there it is radians.
[[nodiscard]] inline double jointPositionFromRegister(uint8_t position,
                                                      double closed_position,
                                                      const Robotiq::DeviceProfile& profile)
{
   return closed_position * (position - profile.openPosition) / profile.registerPositionRange();
}

//! Joint position -> register count (rPR), in the same unit as
//! \p closed_position. Counts outside the byte are clamped. Nothing when the
//! arithmetic cannot produce a count — a NaN command, or a
//! \p closed_position of zero: there is no defensible register value, and
//! commanding a made-up one moves a gripper that may be holding something.
//!
//! Rounds, so a register round-trips through the joint value.
[[nodiscard]] inline std::optional<uint8_t> registerFromJointPosition(double joint_position,
                                                                      double closed_position,
                                                                      const Robotiq::DeviceProfile& profile)
{
   const double counts = (joint_position / closed_position) * profile.registerPositionRange() + profile.openPosition;
   if(!std::isfinite(counts))
   {
      return std::nullopt;
   }
   return static_cast<uint8_t>(std::lround(std::clamp(counts, 0.0, 255.0)));
}

//! Normalized position (0 open .. 1 closed) -> the joint position commanding the same register.
[[nodiscard]] inline std::optional<double> jointPositionFromNormalized(
   double normalized,
   double closed_position,
   const Robotiq::DeviceProfile& profile,
   const std::shared_ptr<Robotiq::Logger>& logger = nullptr)
{
   const std::optional<uint8_t> registerValue = Robotiq::units::normalizedToRegister(normalized, logger);
   if(!registerValue || !isValidClosedPosition(closed_position))
   {
      return std::nullopt;
   }
   return jointPositionFromRegister(*registerValue, closed_position, profile);
}

//! Joint position -> normalized position (0 open .. 1 closed), through its register.
[[nodiscard]] inline std::optional<double> normalizedFromJointPosition(double joint_position,
                                                                       double closed_position,
                                                                       const Robotiq::DeviceProfile& profile)
{
   const std::optional<uint8_t> position = registerFromJointPosition(joint_position, closed_position, profile);
   if(!position)
   {
      return std::nullopt;
   }
   return Robotiq::units::normalizedFromRegister(*position);
}

// Deprecated since 1.4.0 and removed in the next major release. They assume
// a 2F-85.
[[deprecated("use Robotiq::profiles::k2F85.openPosition")]] inline constexpr uint8_t kGripperMinPos =
   Robotiq::profiles::k2F85.openPosition;
[[deprecated("use Robotiq::profiles::k2F85.closedPosition")]] inline constexpr uint8_t kGripperMaxPos =
   Robotiq::profiles::k2F85.closedPosition;
[[deprecated("use Robotiq::profiles::k2F85.registerPositionRange()")]] inline constexpr uint8_t kGripperRange =
   Robotiq::profiles::k2F85.closedPosition - Robotiq::profiles::k2F85.openPosition;

[[deprecated("pass the gripper's Robotiq::DeviceProfile")]] [[nodiscard]] inline double jointPositionFromRegister(
   uint8_t position,
   double closed_position)
{
   return jointPositionFromRegister(position, closed_position, Robotiq::profiles::k2F85);
}

[[deprecated("pass the gripper's Robotiq::DeviceProfile")]] [[nodiscard]] inline std::optional<uint8_t>
registerFromJointPosition(double joint_position, double closed_position)
{
   return registerFromJointPosition(joint_position, closed_position, Robotiq::profiles::k2F85);
}

//! An SI speed or force command -> its 0x00..0xFF register (rSP / rFR), as the
//! fraction of \p maximum it represents, rounded. Nothing unless both sides
//! are finite: \p value from zero up, \p maximum above it.
[[nodiscard]] inline std::optional<uint8_t> registerFromFractionOf(double value, double maximum)
{
   return Robotiq::units::registerFromSpan(value, 0.0, maximum);
}

inline constexpr double kAmperesPerCurrentCount = 0.010;

[[nodiscard]] inline constexpr double motorCurrentFromRegister(uint8_t register_value)
{
   return kAmperesPerCurrentCount * register_value;
}

// gFLT alone: the fault byte's high nibble is kFLT, a fault of the controller
// rather than of the gripper, and nothing exports it.
[[nodiscard]] inline double gripperFaultFromRegister(Robotiq::FaultStatus fault)
{
   return static_cast<double>(fault.gripperFault());
}

// The SDK ranks a code it does not recognize as Major, so an undocumented
// fault is never reported as harmless.
[[nodiscard]] inline double gripperFaultSeverityFromRegister(Robotiq::FaultStatus fault)
{
   return static_cast<double>(Robotiq::severity(fault.gripperFault()));
}

[[nodiscard]] inline double objectStatusValue(Robotiq::ObjectDetection detection, Robotiq::ConnectionState connection)
{
   return connection == Robotiq::ConnectionState::Operational ? static_cast<double>(detection)
                                                              : std::numeric_limits<double>::quiet_NaN();
   //            \__when the connection state is not operational, we must avoid reporting a stale state
}

} // namespace robotiq_driver
