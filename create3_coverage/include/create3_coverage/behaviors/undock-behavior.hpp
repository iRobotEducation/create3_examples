// Copyright 2021 iRobot Corporation. All Rights Reserved.

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
