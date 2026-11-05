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

//! A stock gripper action controller deciding stall and goal states from the
//! gripper's own object detection instead of the velocity stall timeout:
//! parallel_gripper_controller's from Jazzy on, gripper_controllers' on Humble.
//! Humble EOL: simplify — one base remains, so this can become a plain class.
//! Also serves normalized_gripper_cmd (0 = open, 1 = closed), forwarded to gripper_cmd.
//! From Jazzy on, its speed and force run from 0 = slowest, weakest to 1.

#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotiq_controllers/gripper_status.hpp"
#include "robotiq_controllers/joint_scale.hpp"
#include "robotiq_controllers/object_status_goal.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"
#include "robotiq_driver/gripper_scaling.hpp"
#include "robotiq_driver/rclcpp_logger.hpp"

namespace robotiq_controllers {

inline constexpr const char* kNormalizedAction = "normalized_gripper_cmd";

template <class Base>
class ObjectStatusController : public Base
{
public:
   controller_interface::InterfaceConfiguration state_interface_configuration() const override
   {
      controller_interface::InterfaceConfiguration configuration = Base::state_interface_configuration();
      if(use_object_status_)
      {
         configuration.names.push_back(object_status_goal::interfaceName(this->params_.joint));
      }
      return configuration;
   }

   controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override
   {
      if(object_status_)
      {
         // Reasserted every cycle rather than once at activation: only the gripper
         // may call a stall, and a base that refreshed params_ would bring the
         // velocity one back.
         this->params_.stall_timeout = std::numeric_limits<double>::infinity();
         decideFromObjectStatus(time);
      }
      // A goal decided above is no longer active, so the stock check finds nothing to do.
      return Base::update(time, period);
   }

   controller_interface::CallbackReturn on_init() override
   {
      if(Base::on_init() != controller_interface::CallbackReturn::SUCCESS)
      {
         return controller_interface::CallbackReturn::ERROR;
      }
      try
      {
         use_object_status_ = this->template auto_declare<bool>(object_status_goal::kUseParameter, false);
         object_status_timeout_ = this->template auto_declare<double>(object_status_goal::kTimeoutParameter,
                                                                      object_status_goal::kDefaultTimeout);
         joint_scale::declareParameters(*this);
      }
      catch(const std::exception& e)
      {
         RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to declare the object status parameters: %s", e.what());
         return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
   {
      const controller_interface::CallbackReturn result = Base::on_configure(previous_state);
      if(result != controller_interface::CallbackReturn::SUCCESS)
      {
         return result;
      }
      use_object_status_ = this->get_node()->get_parameter(object_status_goal::kUseParameter).as_bool();
      object_status_timeout_ = this->get_node()->get_parameter(object_status_goal::kTimeoutParameter).as_double();
      if(!(object_status_timeout_ > 0.0))
      {
         RCLCPP_ERROR(this->get_node()->get_logger(),
                      "%s must be positive, got %g.",
                      object_status_goal::kTimeoutParameter,
                      object_status_timeout_);
         return controller_interface::CallbackReturn::ERROR;
      }
      closed_position_ = joint_scale::closedPosition(*this, this->params_.joint);
      profile_ = joint_scale::profile(*this, this->params_.joint);
      max_speed_ = joint_scale::fullScale(*this,
                                          this->params_.joint,
                                          object_status_goal::maxSpeedFromUrdf,
                                          robotiq_driver::kMaxSpeedDefault);
      max_force_ = joint_scale::fullScale(*this,
                                          this->params_.joint,
                                          object_status_goal::maxForceFromUrdf,
                                          robotiq_driver::kMaxForceDefault);
      if(use_object_status_ && closed_position_ && !profile_)
      {
         RCLCPP_WARN(this->get_node()->get_logger(),
                     "Unknown %s for joint '%s': a goal repeating the last one waits for %s to change.",
                     robotiq_driver::kProfileParam,
                     this->params_.joint.c_str(),
                     object_status_goal::kTimeoutParameter);
      }
      if(use_object_status_ && !closed_position_)
      {
         RCLCPP_WARN(this->get_node()->get_logger(),
                     "No %s for joint '%s': a goal repeating the last one waits for %s to change.",
                     robotiq_driver::kClosedPositionParam,
                     this->params_.joint.c_str(),
                     object_status_goal::kTimeoutParameter);
      }
      startNormalizedAction();
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
   {
      const controller_interface::CallbackReturn result = Base::on_activate(previous_state);
      if(result != controller_interface::CallbackReturn::SUCCESS || !use_object_status_)
      {
         return result;
      }

      object_status_ = object_status_goal::findInterface(this->state_interfaces_,
                                                         this->params_.joint,
                                                         this->get_node()->get_logger());
      if(!object_status_)
      {
         return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
   {
      object_status_.reset();
      tracked_goal_.reset();
      verdict_ = object_status_goal::Verdict();
      return Base::on_deactivate(previous_state);
   }

protected:
   using RealtimeGoalHandlePtr = typename Base::RealtimeGoalHandlePtr;

private:
   using Action = typename Base::GripperCommandAction;
   using NormalizedGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
   using ForwardedGoalHandle = rclcpp_action::ClientGoalHandle<Action>;

   void startNormalizedAction()
   {
      normalized_server_.reset();
      joint_client_.reset();
      if(!closed_position_ || !profile_)
      {
         RCLCPP_WARN(this->get_node()->get_logger(),
                     "No %s: joint '%s' has no usable %s and %s.",
                     kNormalizedAction,
                     this->params_.joint.c_str(),
                     robotiq_driver::kClosedPositionParam,
                     robotiq_driver::kProfileParam);
         return;
      }
      sdk_logger_ = std::make_shared<robotiq_driver::RclcppLogger>(this->get_node()->get_logger());
      joint_client_ = rclcpp_action::create_client<Action>(this->get_node(), "~/gripper_cmd");
      normalized_server_ = rclcpp_action::create_server<Action>(
         this->get_node(),
         std::string{"~/"} + kNormalizedAction,
         [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal> goal) {
            return onNormalizedGoal(*goal);
         },
         [this](std::shared_ptr<NormalizedGoalHandle> goal) { return onNormalizedCancel(goal); },
         [this](std::shared_ptr<NormalizedGoalHandle> goal) { relay(goal); });
   }

   // Called once per goal, so a clamp warns once.
   std::optional<double> jointPosition(const typename Action::Goal& goal) const
   {
      const std::optional<double> normalized = compat::goalPosition(goal);
      if(!normalized)
      {
         return std::nullopt;
      }
      return robotiq_driver::jointPositionFromNormalized(*normalized, *closed_position_, *profile_, sdk_logger_);
   }

   bool speedAndForceAreValid(const typename Action::Goal& goal) const
   {
      bool valid = true;
      compat::visitSpeedAndForce(
         goal,
         [&](double speed) { valid = valid && robotiq_driver::speedOrForceFromNormalized(speed, max_speed_); },
         [&](double force) { valid = valid && robotiq_driver::speedOrForceFromNormalized(force, max_force_); });
      return valid;
   }

   rclcpp_action::GoalResponse onNormalizedGoal(const typename Action::Goal& goal)
   {
      const std::optional<double> normalized = compat::goalPosition(goal);
      if(!normalized || !std::isfinite(*normalized))
      {
         RCLCPP_ERROR(this->get_node()->get_logger(), "Rejected a %s goal with no finite position.", kNormalizedAction);
         return rclcpp_action::GoalResponse::REJECT;
      }
      if(!speedAndForceAreValid(goal))
      {
         RCLCPP_ERROR(this->get_node()->get_logger(),
                      "Rejected a %s goal with a negative or non-finite speed or force.",
                      kNormalizedAction);
         return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
   }

   rclcpp_action::CancelResponse onNormalizedCancel(const std::shared_ptr<NormalizedGoalHandle>& goal)
   {
      const std::lock_guard<std::mutex> lock(forwarded_mutex_);
      Forwarded& forwarded = forwarded_[goal->get_goal_id()];
      if(forwarded.handle)
      {
         joint_client_->async_cancel_goal(forwarded.handle);
      }
      else
      {
         // Cancelled once gripper_cmd acknowledges it.
         forwarded.cancel_requested = true;
      }
      return rclcpp_action::CancelResponse::ACCEPT;
   }

   void relay(const std::shared_ptr<NormalizedGoalHandle>& goal)
   {
      if(!joint_client_->action_server_is_ready())
      {
         RCLCPP_ERROR(this->get_node()->get_logger(),
                      "%s is not up; aborting the %s goal.",
                      "gripper_cmd",
                      kNormalizedAction);
         forget(goal->get_goal_id());
         goal->abort(std::make_shared<typename Action::Result>());
         return;
      }
      typename Action::Goal joint_goal = *goal->get_goal();
      compat::setGoalPosition(joint_goal, jointPosition(joint_goal).value());
      // To the values the driver turns into the same rSP and rFR.
      compat::visitSpeedAndForce(
         joint_goal,
         [this](double& speed) { speed = robotiq_driver::speedOrForceFromNormalized(speed, max_speed_).value(); },
         [this](double& force) { force = robotiq_driver::speedOrForceFromNormalized(force, max_force_).value(); });

      typename rclcpp_action::Client<Action>::SendGoalOptions options;
      options.goal_response_callback = [this, goal](typename ForwardedGoalHandle::SharedPtr forwarded) {
         if(!forwarded)
         {
            forget(goal->get_goal_id());
            goal->abort(std::make_shared<typename Action::Result>());
            return;
         }
         const std::lock_guard<std::mutex> lock(forwarded_mutex_);
         Forwarded& entry = forwarded_[goal->get_goal_id()];
         entry.handle = forwarded;
         if(entry.cancel_requested)
         {
            joint_client_->async_cancel_goal(forwarded);
         }
      };
      options.result_callback = [this, goal](const typename ForwardedGoalHandle::WrappedResult& forwarded) {
         forget(goal->get_goal_id());
         auto result = forwarded.result ? std::make_shared<typename Action::Result>(*forwarded.result)
                                        : std::make_shared<typename Action::Result>();
         compat::convertReportedPosition(*result, [this](double position) { return normalizedPosition(position); });
         if(forwarded.code == rclcpp_action::ResultCode::SUCCEEDED)
         {
            goal->succeed(result);
         }
         else if(forwarded.code == rclcpp_action::ResultCode::CANCELED && goal->is_canceling())
         {
            goal->canceled(result);
         }
         else
         {
            // Preempted by a newer goal on either action.
            goal->abort(result);
         }
      };
      joint_client_->async_send_goal(joint_goal, options);
   }

   void forget(const rclcpp_action::GoalUUID& goal)
   {
      const std::lock_guard<std::mutex> lock(forwarded_mutex_);
      forwarded_.erase(goal);
   }

   double normalizedPosition(double joint_position) const
   {
      return robotiq_driver::normalizedFromJointPosition(joint_position, *closed_position_, *profile_)
         .value_or(std::numeric_limits<double>::quiet_NaN());
   }

   void decideFromObjectStatus(const rclcpp::Time& time)
   {
      const std::optional<RealtimeGoalHandlePtr> goal = compat::tryGet<RealtimeGoalHandlePtr>(this->rt_active_goal_);
      if(!goal || !*goal)
      {
         return;
      }

      const std::optional<Robotiq::ObjectDetection> detection =
         gripper_status::toObjectDetection(compat::getValue(object_status_->get()));
      if(*goal != tracked_goal_)
      {
         tracked_goal_ = *goal;
         verdict_.reset(time, detection, object_status_timeout_, positionRequest(**goal));
         return;
      }
      if(const std::optional<object_status_goal::Outcome> outcome = verdict_.decide(time, detection))
      {
         finish(*goal, outcome.value());
      }
   }

   // The register count the driver sends for the goal's position.
   template <class RealtimeGoalHandle>
   std::optional<uint8_t> positionRequest(const RealtimeGoalHandle& goal) const
   {
      const std::optional<double> position = compat::goalPosition(*goal.gh_->get_goal());
      if(!position || !closed_position_ || !profile_)
      {
         return std::nullopt;
      }
      return robotiq_driver::registerFromJointPosition(*position, *closed_position_, *profile_);
   }

   void finish(const RealtimeGoalHandlePtr& goal, const object_status_goal::Outcome& outcome)
   {
      const std::optional<double> position = compat::getValue(this->joint_position_state_interface_->get());
      if(!position)
      {
         return;
      }
      compat::setResult(*this->pre_alloc_result_, position.value(), this->computed_command_);
      this->pre_alloc_result_->reached_goal = outcome.reached_goal;
      this->pre_alloc_result_->stalled = outcome.stalled;
      if(outcome.reached_goal || (outcome.stalled && this->params_.allow_stalling))
      {
         goal->setSucceeded(this->pre_alloc_result_);
      }
      else
      {
         goal->setAborted(this->pre_alloc_result_);
      }
      compat::set(this->rt_active_goal_, RealtimeGoalHandlePtr());
   }

   bool use_object_status_ = false;
   double object_status_timeout_ = 0.0;
   std::optional<double> closed_position_;
   std::optional<Robotiq::DeviceProfile> profile_;
   double max_speed_ = robotiq_driver::kMaxSpeedDefault;
   double max_force_ = robotiq_driver::kMaxForceDefault;
   std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> object_status_;
   RealtimeGoalHandlePtr tracked_goal_;
   object_status_goal::Verdict verdict_;

   struct Forwarded
   {
      typename ForwardedGoalHandle::SharedPtr handle;
      bool cancel_requested = false;
   };
   // The action callbacks run on the executor.
   std::mutex forwarded_mutex_;
   std::map<rclcpp_action::GoalUUID, Forwarded> forwarded_;
   std::shared_ptr<Robotiq::Logger> sdk_logger_;
   // Last, so destroyed first: no callback outlives the members above.
   typename rclcpp_action::Client<Action>::SharedPtr joint_client_;
   typename rclcpp_action::Server<Action>::SharedPtr normalized_server_;
};
} // namespace robotiq_controllers
