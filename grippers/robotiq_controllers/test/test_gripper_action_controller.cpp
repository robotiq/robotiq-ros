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

#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/parallel_gripper_command.hpp>

#include <robotiq_controllers/gripper_action_controller.hpp>
#include <robotiq_driver/hardware_parameters.hpp>

#include "object_status_controller_test.hpp"

namespace robotiq_controllers::test {
namespace {
struct ParallelGripperCommandTraits
{
   using Controller = GripperActionController;
   using Action = control_msgs::action::ParallelGripperCommand;
   static constexpr const char* kAction = "ParallelGripperCommand";
   static constexpr const char* kName = "test_gripper_action_controller";

   static Action::Goal goal(const std::string& joint, double position)
   {
      Action::Goal goal;
      goal.command.name = {joint};
      goal.command.position = {position};
      return goal;
   }
   static double position(const Action::Result& result) { return result.state.position[0]; }
};

INSTANTIATE_TYPED_TEST_SUITE_P(GripperActionController,
                               ObjectStatusControllerTest,
                               ::testing::Types<ParallelGripperCommandTraits>,
                               NamedByAction);

using NormalizedSpeedAndForce = ObjectStatusControllerTest<ParallelGripperCommandTraits>;

constexpr double kNonDefaultMaxSpeed = 0.075;
constexpr double kNonDefaultMaxForce = 185.0;

ParallelGripperCommandTraits::Action::Goal goal(double position, std::vector<double> speed, std::vector<double> force)
{
   ParallelGripperCommandTraits::Action::Goal goal = ParallelGripperCommandTraits::goal(kJoint, position);
   goal.command.velocity = std::move(speed);
   goal.command.effort = std::move(force);
   return goal;
}

TEST_F(NormalizedSpeedAndForce, ForwardsTheValuesCommandingTheirRegisters)
{
   bringUp(Config::driver().withFullScales(kNonDefaultMaxSpeed, kNonDefaultMaxForce));
   sendNormalizedGoal(goal(0.5, {0.5}, {0.25}));

   const auto& command = controller_->activeGoal()->gh_->get_goal()->command;
   ASSERT_EQ(1u, command.velocity.size());
   ASSERT_EQ(1u, command.effort.size());
   EXPECT_EQ(128, robotiq_driver::registerFromFractionOf(command.velocity[0], kNonDefaultMaxSpeed));
   EXPECT_EQ(64, robotiq_driver::registerFromFractionOf(command.effort[0], kNonDefaultMaxForce));
}

TEST_F(NormalizedSpeedAndForce, ScalesByTheDriverDefaultsWithoutFullScales)
{
   bringUp();
   sendNormalizedGoal(goal(0.5, {0.0}, {1.0}));

   const auto& command = controller_->activeGoal()->gh_->get_goal()->command;
   ASSERT_EQ(1u, command.velocity.size());
   ASSERT_EQ(1u, command.effort.size());
   EXPECT_EQ(0, robotiq_driver::registerFromFractionOf(command.velocity[0], robotiq_driver::kMaxSpeedDefault));
   EXPECT_EQ(255, robotiq_driver::registerFromFractionOf(command.effort[0], robotiq_driver::kMaxForceDefault));
}

TEST_F(NormalizedSpeedAndForce, LeavesAnUnsetSpeedAndForceToGripperCmd)
{
   bringUp();
   sendNormalizedGoal(goal(0.5, {}, {}));

   const auto& command = controller_->activeGoal()->gh_->get_goal()->command;
   EXPECT_TRUE(command.velocity.empty());
   EXPECT_TRUE(command.effort.empty());
}

TEST_F(NormalizedSpeedAndForce, RejectsANegativeOrNonFiniteValue)
{
   bringUp();
   EXPECT_FALSE(acceptsNormalizedGoal(goal(0.5, {-0.1}, {})));
   EXPECT_FALSE(acceptsNormalizedGoal(goal(0.5, {}, {kNaN})));
   EXPECT_FALSE(controller_->holdsGoal());
}
} // namespace
} // namespace robotiq_controllers::test

// Not gtest_main: the tests create nodes, so rclcpp must be up before the first
// and down after the last.
int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   rclcpp::init(argc, argv);
   const int result = RUN_ALL_TESTS();
   rclcpp::shutdown();
   return result;
}
