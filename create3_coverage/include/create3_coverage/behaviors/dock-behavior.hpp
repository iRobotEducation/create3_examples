// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_coverage {

class DockBehavior : public Behavior
{
public:
    using DockAction = irobot_create_msgs::action::Dock;
    using GoalHandleDock = rclcpp_action::ClientGoalHandle<DockAction>;

    DockBehavior(
        rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
        rclcpp::Logger logger);

    ~DockBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::DOCK; }

    void cleanup() override;

private:
    bool m_dock_action_sent {false};
    GoalHandleDock::SharedPtr m_dock_goal_handle;
    bool m_dock_goal_handle_ready {false};
    GoalHandleDock::WrappedResult m_dock_result;
    bool m_dock_result_ready {false};

    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;
    rclcpp::Logger m_logger;
};

} // namespace create3_coverage
