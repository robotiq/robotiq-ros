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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include <Robotiq/gripper/to_string.hpp>

#include <robotiq_driver/gripper_scaling.hpp>

namespace robotiq_driver::test {
namespace {
// The 2F-85's fully-closed joint angle, as the shipped description sets it.
constexpr double kClosedPosition = 0.7929;
constexpr const Robotiq::DeviceProfile& k2F85 = Robotiq::profiles::k2F85;
// A user's own profile for a 2F-85 whose stops measure at other counts.
constexpr Robotiq::DeviceProfile kRemeasured2F85 = [] {
   Robotiq::DeviceProfile profile = Robotiq::profiles::k2F85;
   profile.openPosition = 5;
   profile.closedPosition = 226;
   return profile;
}();
constexpr uint8_t kOpenCount = k2F85.openPosition;
constexpr uint8_t kClosedCount = k2F85.closedPosition;
} // namespace

TEST(GripperScaling, TravelEndpointsMapToTheJointLimits)
{
   EXPECT_DOUBLE_EQ(0.0, jointPositionFromRegister(kOpenCount, kClosedPosition, k2F85));
   EXPECT_DOUBLE_EQ(kClosedPosition, jointPositionFromRegister(kClosedCount, kClosedPosition, k2F85));
}

TEST(GripperScaling, JointLimitsMapBackToTheTravelEndpoints)
{
   EXPECT_EQ(kOpenCount, registerFromJointPosition(0.0, kClosedPosition, k2F85));
   EXPECT_EQ(kClosedCount, registerFromJointPosition(kClosedPosition, kClosedPosition, k2F85));
}

TEST(GripperScaling, TheProfileSetsTheTravelBand)
{
   EXPECT_DOUBLE_EQ(kClosedPosition,
                    jointPositionFromRegister(kRemeasured2F85.closedPosition, kClosedPosition, kRemeasured2F85));
   EXPECT_EQ(kRemeasured2F85.closedPosition,
             registerFromJointPosition(kClosedPosition, kClosedPosition, kRemeasured2F85));
   EXPECT_EQ(kRemeasured2F85.openPosition, registerFromJointPosition(0.0, kClosedPosition, kRemeasured2F85));
}

TEST(GripperScaling, RegisterRoundTripIsExact)
{
   for(int counts = 0; counts <= 0xFF; ++counts)
   {
      const auto raw = static_cast<uint8_t>(counts);
      EXPECT_EQ(
         raw,
         registerFromJointPosition(jointPositionFromRegister(raw, kClosedPosition, k2F85), kClosedPosition, k2F85))
         << "at " << counts << " counts";
   }
}

TEST(GripperScaling, TheNormalizedEndsAreTheByteEnds)
{
   EXPECT_EQ(0,
             registerFromJointPosition(jointPositionFromNormalized(0.0, kClosedPosition, k2F85).value(),
                                       kClosedPosition,
                                       k2F85));
   EXPECT_EQ(255,
             registerFromJointPosition(jointPositionFromNormalized(1.0, kClosedPosition, k2F85).value(),
                                       kClosedPosition,
                                       k2F85));
   EXPECT_LT(jointPositionFromNormalized(0.0, kClosedPosition, k2F85).value(), 0.0);
   EXPECT_GT(jointPositionFromNormalized(1.0, kClosedPosition, k2F85).value(), kClosedPosition);
}

TEST(GripperScaling, ANormalizedPositionCommandEqualsItselfAfterAConversionRoundTrip)
{
   for(int counts = 0; counts <= 0xFF; ++counts)
   {
      const double normalized = Robotiq::units::normalizedFromRegister(static_cast<uint8_t>(counts));
      const std::optional<double> joint = jointPositionFromNormalized(normalized, kClosedPosition, kRemeasured2F85);
      ASSERT_TRUE(joint.has_value());
      EXPECT_EQ(counts, registerFromJointPosition(*joint, kClosedPosition, kRemeasured2F85)) << "at " << counts;
      EXPECT_DOUBLE_EQ(normalized, normalizedFromJointPosition(*joint, kClosedPosition, kRemeasured2F85).value());
   }
}

TEST(GripperScaling, ANormalizedPositionWithNoRegisterValueCommandsNothing)
{
   EXPECT_FALSE(jointPositionFromNormalized(std::numeric_limits<double>::quiet_NaN(), kClosedPosition, k2F85));
   EXPECT_FALSE(jointPositionFromNormalized(std::numeric_limits<double>::infinity(), kClosedPosition, k2F85));
   EXPECT_FALSE(jointPositionFromNormalized(0.5, 0.0, k2F85));
   EXPECT_FALSE(normalizedFromJointPosition(std::numeric_limits<double>::quiet_NaN(), kClosedPosition, k2F85));
}

TEST(GripperScaling, AClampedNormalizedPositionWarnsOnTheGivenLogger)
{
   struct Collecting : Robotiq::Logger
   {
      void log(Level level, std::string_view) override { levels.push_back(level); }
      std::vector<Level> levels;
   };
   const auto logger = std::make_shared<Collecting>();
   std::ignore = jointPositionFromNormalized(0.5, kClosedPosition, k2F85, logger);
   EXPECT_TRUE(logger->levels.empty());
   std::ignore = jointPositionFromNormalized(1.5, kClosedPosition, k2F85, logger);
   EXPECT_EQ(logger->levels, std::vector<Robotiq::Logger::Level>{Robotiq::Logger::Level::Warn});
}

TEST(GripperScaling, ANormalizedPositionOutsideTheUnitSpanClamps)
{
   EXPECT_EQ(jointPositionFromNormalized(-0.5, kClosedPosition, k2F85),
             jointPositionFromNormalized(0.0, kClosedPosition, k2F85));
   EXPECT_EQ(jointPositionFromNormalized(1.5, kClosedPosition, k2F85),
             jointPositionFromNormalized(1.0, kClosedPosition, k2F85));
}

TEST(GripperScaling, PositionsOutsideTheJointRangeClampIntoTheByte)
{
   EXPECT_EQ(0, registerFromJointPosition(-1.0, kClosedPosition, k2F85));
   EXPECT_EQ(255, registerFromJointPosition(10.0 * kClosedPosition, kClosedPosition, k2F85));
}

TEST(GripperScaling, PositionsBelowTheTravelBandStayReachable)
{
   // The band starts at 3 counts, but 0..2 are still commandable: clamping is
   // to the byte, not to the band.
   EXPECT_LT(registerFromJointPosition(-0.005, kClosedPosition, k2F85), kOpenCount);
}

TEST(GripperScaling, AnUnusableClosedPositionCommandsNothing)
{
   // 0/0 and x/0 have no register to land on. Answering "nothing" keeps the
   // caller from moving the fingers on a made-up count.
   EXPECT_FALSE(registerFromJointPosition(0.0, 0.0, k2F85).has_value());
   EXPECT_FALSE(registerFromJointPosition(0.5, 0.0, k2F85).has_value());
   EXPECT_FALSE(
      registerFromJointPosition(std::numeric_limits<double>::quiet_NaN(), kClosedPosition, k2F85).has_value());
   EXPECT_FALSE(registerFromJointPosition(0.5, std::numeric_limits<double>::quiet_NaN(), k2F85).has_value());
}

TEST(GripperScaling, AClosedPositionThatClosesNegativeStillMaps)
{
   // A joint whose closed direction is negative is a supported description:
   // the ratio inverts and the mapping still lands in the travel band.
   EXPECT_EQ(kOpenCount, registerFromJointPosition(0.0, -kClosedPosition, k2F85));
   EXPECT_EQ(kClosedCount, registerFromJointPosition(-kClosedPosition, -kClosedPosition, k2F85));
}

TEST(GripperScaling, AFractionOutsideTheRangeCommandsNothing)
{
   // As above: no fraction, so no register, rather than a guessed speed or
   // force. A negative speed and a maximum that is not positive are both broken
   // descriptions, not directions the way a negative closed position is.
   EXPECT_FALSE(registerFromFractionOf(0.1, 0.0).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, -0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(-0.1, 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(std::numeric_limits<double>::quiet_NaN(), 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, std::numeric_limits<double>::quiet_NaN()).has_value());
   EXPECT_FALSE(registerFromFractionOf(std::numeric_limits<double>::infinity(), 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, std::numeric_limits<double>::infinity()).has_value());
}

TEST(GripperScaling, FractionOfSpansTheWholeByte)
{
   EXPECT_EQ(0, registerFromFractionOf(0.0, 0.15));
   EXPECT_EQ(255, registerFromFractionOf(0.15, 0.15));
   EXPECT_EQ(128, registerFromFractionOf(0.075, 0.15));
   EXPECT_EQ(127, registerFromFractionOf(0.0749, 0.15));
}

TEST(GripperScaling, FractionOfSaturatesRatherThanWrapping)
{
   // A request beyond full scale must pin at 0xFF, not wrap to a near-zero
   // speed or force.
   EXPECT_EQ(255, registerFromFractionOf(10.0, 0.15));
}

TEST(GripperScaling, whenTheMaximumIsNearZero_theRegisterIsTheMaxAllowedValue)
{
   EXPECT_EQ(255, registerFromFractionOf(0.15, std::numeric_limits<double>::denorm_min()));
}

TEST(GripperScaling, ANormalizedSpeedOrForceCommandsItsRegister)
{
   for(const double maximum : {0.150, 0.075, 235.0, 185.0, 0.1})
   {
      for(int count = 0; count <= 255; ++count)
      {
         const std::optional<double> value = speedOrForceFromNormalized(count / 255.0, maximum);
         ASSERT_TRUE(value.has_value());
         EXPECT_EQ(count, registerFromFractionOf(*value, maximum)) << "maximum " << maximum;
      }
   }
}

TEST(GripperScaling, ANormalizedSpeedOrForceRoundsToTheNearestRegister)
{
   EXPECT_EQ(128, registerFromFractionOf(speedOrForceFromNormalized(0.5, 0.15).value(), 0.15));
}

TEST(GripperScaling, ANormalizedSpeedOrForceAboveOneSaturates)
{
   EXPECT_DOUBLE_EQ(0.15, speedOrForceFromNormalized(1.5, 0.15).value());
}

TEST(GripperScaling, ANormalizedSpeedOrForceOutsideTheRangeCommandsNothing)
{
   EXPECT_FALSE(speedOrForceFromNormalized(-0.1, 0.15).has_value());
   EXPECT_FALSE(speedOrForceFromNormalized(std::numeric_limits<double>::quiet_NaN(), 0.15).has_value());
   EXPECT_FALSE(speedOrForceFromNormalized(std::numeric_limits<double>::infinity(), 0.15).has_value());
   EXPECT_FALSE(speedOrForceFromNormalized(0.5, 0.0).has_value());
   EXPECT_FALSE(speedOrForceFromNormalized(0.5, -0.15).has_value());
   EXPECT_FALSE(speedOrForceFromNormalized(0.5, std::numeric_limits<double>::quiet_NaN()).has_value());
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST(GripperScaling, TheDeprecatedNamesKeepThe2F85Band)
{
   EXPECT_EQ(kOpenCount, kGripperMinPos);
   EXPECT_EQ(kClosedCount, kGripperMaxPos);
   EXPECT_EQ(k2F85.registerPositionRange(), kGripperRange);
   EXPECT_DOUBLE_EQ(jointPositionFromRegister(128, kClosedPosition, k2F85),
                    jointPositionFromRegister(128, kClosedPosition));
   EXPECT_EQ(registerFromJointPosition(0.4, kClosedPosition, k2F85), registerFromJointPosition(0.4, kClosedPosition));
}
#pragma GCC diagnostic pop

TEST(GripperScaling, MotorCurrentIsTenMilliampsPerCount)
{
   EXPECT_DOUBLE_EQ(0.0, motorCurrentFromRegister(0));
   EXPECT_DOUBLE_EQ(0.01, motorCurrentFromRegister(1));
   EXPECT_DOUBLE_EQ(1.27, motorCurrentFromRegister(127));
   EXPECT_DOUBLE_EQ(2.55, motorCurrentFromRegister(255));
}

TEST(GripperScaling, MotorCurrentTakesNoScaleFromTheDescription)
{
   // Scaling gCU by a description parameter would make one motor current read
   // as two numbers on two robots.
   static_assert(std::is_invocable_r_v<double, decltype(motorCurrentFromRegister), uint8_t>);
   EXPECT_DOUBLE_EQ(1.0, motorCurrentFromRegister(100));
}

TEST(GripperScaling, TheFaultInterfacesReadTheGripperFaultAndItsOwnSeverity)
{
   const Robotiq::FaultStatus fault = Robotiq::FaultStatus::fromRaw(0x5C);
   //            \__ gripper fault: 0xC Internal; controller fault: 0x5 NoDeviceDetected

   const auto expectedGripperFault = Robotiq::GripperFault::InternalFault;
   EXPECT_EQ(static_cast<double>(expectedGripperFault), gripperFaultFromRegister(fault));
   EXPECT_EQ(static_cast<double>(Robotiq::severity(expectedGripperFault)), gripperFaultSeverityFromRegister(fault));
}

TEST(GripperScaling, ObjectStatusIsNaNUnlessTheLinkIsOperational)
{
   EXPECT_EQ(static_cast<double>(Robotiq::ObjectDetection::DetectedWhileClosing),
             objectStatusValue(Robotiq::ObjectDetection::DetectedWhileClosing, Robotiq::ConnectionState::Operational));
   for(const Robotiq::ConnectionState connection : {Robotiq::ConnectionState::Disconnected,
                                                    Robotiq::ConnectionState::Connecting,
                                                    Robotiq::ConnectionState::Faulted})
   {
      EXPECT_TRUE(std::isnan(objectStatusValue(Robotiq::ObjectDetection::DetectedWhileClosing, connection)))
         << Robotiq::toString(connection);
   }
}

} // namespace robotiq_driver::test
