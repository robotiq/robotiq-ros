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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <robotiq_controllers/normalized_position_broadcaster.hpp>
#include <robotiq_driver/gripper_scaling.hpp>

#include "controller_test_compat.hpp"
#include "gripper_urdf.hpp"

namespace robotiq_controllers::test {
namespace {
constexpr const char* kControllerName = "test_normalized_position_broadcaster";
constexpr const char* kJoint = "robotiq_85_left_knuckle_joint";
constexpr unsigned int kUpdateRate = 100;
constexpr double kClosedPosition = 0.7929;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
} // namespace

class NormalizedPositionBroadcasterTest : public ::testing::Test
{
protected:
   using JointState = sensor_msgs::msg::JointState;

   void SetUp() override
   {
      subscriber_ = std::make_shared<rclcpp::Node>("joint_state_subscriber");
      subscription_ =
         subscriber_->create_subscription<JointState>("/joint_states",
                                                      rclcpp::SystemDefaultsQoS(),
                                                      [this](JointState::SharedPtr message) { received_ = *message; });
      executor_.add_node(subscriber_);
   }

   controller_interface::return_type init(std::optional<double> closed_position,
                                          std::vector<rclcpp::Parameter> parameters = {})
   {
      broadcaster_ = std::make_unique<NormalizedPositionBroadcaster>();
      parameters.emplace_back("joint", kJoint);
      // Humble EOL: delete; the robot description below supplies them.
      parameters.emplace_back("gripper_closed_position", closed_position.value_or(kNaN));
      parameters.emplace_back("gripper_profile", "2f_85");
      const std::string hardware =
         closed_position ? R"(<param name="gripper_closed_position">)" + std::to_string(*closed_position) + "</param>"
                         : std::string{};
      return test_compat::init(*broadcaster_, kControllerName, kUpdateRate, parameters, gripperUrdf(kJoint, hardware));
   }

   void bringUp()
   {
      ASSERT_EQ(controller_interface::return_type::OK, init(kClosedPosition));
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->configure().id());
      owned_.push_back(test_compat::bindState(kJoint, "position", &position_));
      std::vector<hardware_interface::LoanedStateInterface> loans;
      loans.push_back(test_compat::loan(owned_.back().interface));
      broadcaster_->assign_interfaces({}, std::move(loans));
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, broadcaster_->get_node()->activate().id());
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
      while(subscription_->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
      {
         executor_.spin_once(std::chrono::milliseconds{10});
      }
      ASSERT_GT(subscription_->get_publisher_count(), 0u);
   }

   std::optional<JointState> receive(const std::function<bool(const JointState&)>& accept)
   {
      received_.reset();
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
      while(!(received_ && accept(*received_)) && std::chrono::steady_clock::now() < deadline)
      {
         for(const auto& state : owned_)
         {
            test_compat::sync(state);
         }
         EXPECT_EQ(controller_interface::return_type::OK,
                   broadcaster_->update(subscriber_->now(), rclcpp::Duration::from_seconds(1.0 / kUpdateRate)));
         executor_.spin_once(std::chrono::milliseconds{10});
      }
      return received_;
   }

   std::unique_ptr<NormalizedPositionBroadcaster> broadcaster_;
   std::vector<test_compat::BoundInterface<hardware_interface::StateInterface>> owned_;
   rclcpp::Node::SharedPtr subscriber_;
   rclcpp::Subscription<JointState>::SharedPtr subscription_;
   rclcpp::executors::SingleThreadedExecutor executor_;
   std::optional<JointState> received_;
   double position_ = kNaN;
};

TEST_F(NormalizedPositionBroadcasterTest, claims_only_the_joint_position)
{
   ASSERT_EQ(controller_interface::return_type::OK, init(kClosedPosition));
   ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->configure().id());
   EXPECT_EQ(std::vector<std::string>{std::string{kJoint} + "/position"},
             broadcaster_->state_interface_configuration().names);
   EXPECT_EQ(controller_interface::interface_configuration_type::NONE,
             broadcaster_->command_interface_configuration().type);
}

TEST_F(NormalizedPositionBroadcasterTest, publishes_the_register_the_joint_position_commands)
{
   bringUp();
   position_ = robotiq_driver::jointPositionFromRegister(128, kClosedPosition, Robotiq::profiles::k2F85);

   const auto state = receive([](const JointState& message) { return !message.position.empty(); });
   ASSERT_TRUE(state.has_value());
   EXPECT_EQ(std::vector<std::string>{"robotiq_normalized_joint"}, state->name);
   EXPECT_DOUBLE_EQ(128.0 / 255.0, state->position.at(0));
}

TEST_F(NormalizedPositionBroadcasterTest, publishes_under_the_configured_name)
{
   ASSERT_EQ(controller_interface::return_type::OK,
             init(kClosedPosition, {rclcpp::Parameter("normalized_joint", "left_robotiq_normalized_joint")}));
   ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->configure().id());
   owned_.push_back(test_compat::bindState(kJoint, "position", &position_));
   std::vector<hardware_interface::LoanedStateInterface> loans;
   loans.push_back(test_compat::loan(owned_.back().interface));
   broadcaster_->assign_interfaces({}, std::move(loans));
   ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, broadcaster_->get_node()->activate().id());
   position_ = 0.0;

   const auto state = receive([](const JointState& message) { return !message.name.empty(); });
   ASSERT_TRUE(state.has_value());
   EXPECT_EQ(std::vector<std::string>{"left_robotiq_normalized_joint"}, state->name);
}

TEST_F(NormalizedPositionBroadcasterTest, refuses_to_configure_without_a_closed_position)
{
   ASSERT_EQ(controller_interface::return_type::OK, init(std::nullopt));
   EXPECT_NE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->configure().id());
}
} // namespace robotiq_controllers::test

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   rclcpp::init(argc, argv);
   const int result = RUN_ALL_TESTS();
   rclcpp::shutdown();
   return result;
}
