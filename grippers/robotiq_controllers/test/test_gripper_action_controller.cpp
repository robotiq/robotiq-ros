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

#include <control_msgs/action/parallel_gripper_command.hpp>

#include <robotiq_controllers/gripper_action_controller.hpp>

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
} // namespace
} // namespace robotiq_controllers::test
