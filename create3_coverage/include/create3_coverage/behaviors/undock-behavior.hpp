// Copyright 2021 iRobot Corporation. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "create3_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_coverage {

class UndockBehavior : public Behavior
{
public:
    using UndockAction = irobot_create_msgs::action::Undock;
    using GoalHandleUndock = rclcpp_action::ClientGoalHandle<UndockAction>;

    UndockBehavior(
        rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
        rclcpp::Logger logger);

    ~UndockBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::UNDOCK; }

    void cleanup() override;

private:
    bool m_undock_action_sent {false};
    GoalHandleUndock::SharedPtr m_undock_goal_handle;
    bool m_undock_goal_handle_ready {false};
    GoalHandleUndock::WrappedResult m_undock_result;
    bool m_undock_result_ready {false};

    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;
    rclcpp::Logger m_logger;
};

} // namespace create3_coverage
