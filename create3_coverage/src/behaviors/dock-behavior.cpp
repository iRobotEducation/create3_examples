// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_coverage/behaviors/dock-behavior.hpp"

namespace create3_coverage {

DockBehavior::DockBehavior(
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp::Logger logger)
: m_dock_action_client(dock_action_client), m_logger(logger)
{

}

State DockBehavior::execute(const Data & data)
{
    // Make sure we are not docked
    if (!m_dock_action_sent && data.dock.is_docked) {
        RCLCPP_ERROR(m_logger, "Robot is already docked!");
        return State::FAILURE;
    }

    // We can't dock until we discover the docking action server
    if (!m_dock_action_client->action_server_is_ready()) {
        RCLCPP_DEBUG(m_logger, "Waiting for dock action server");
        return State::RUNNING;
    }

    // Send dock command if not already sent and if we are not waiting for result
    if (!m_dock_action_sent) {
        RCLCPP_INFO(m_logger, "Sending docking goal!");
        auto goal_msg = DockAction::Goal();

        auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleDock::SharedPtr & goal_handle){
            m_dock_goal_handle_ready = true;
            m_dock_goal_handle = goal_handle;
        };
        send_goal_options.result_callback = [this](const GoalHandleDock::WrappedResult & result){
            m_dock_result_ready = true;
            m_dock_result = result;
        };

        m_dock_action_client->async_send_goal(goal_msg, send_goal_options);
        m_dock_action_sent = true;

        return State::RUNNING;
    }

    if (m_dock_goal_handle_ready && !m_dock_goal_handle) {
        RCLCPP_ERROR(m_logger, "Docking goal was rejected by server");
        return State::FAILURE;
    }

    if (m_dock_result_ready) {
        if (m_dock_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Docking succeeded!");
            return State::SUCCESS;
        } else {
            RCLCPP_ERROR(m_logger, "Docking failed!");
            return State::FAILURE;
        }
    }

    return State::RUNNING;
}

void DockBehavior::cleanup()
{
    // This behavior is being cancelled, so send a cancel request to dock action server if it's running
    if (!m_dock_result_ready && m_dock_goal_handle_ready && m_dock_goal_handle) {
        m_dock_action_client->async_cancel_goal(m_dock_goal_handle);
    }
}

} // namespace create3_coverage
