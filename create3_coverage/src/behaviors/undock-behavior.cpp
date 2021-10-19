// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_coverage/behaviors/undock-behavior.hpp"

namespace create3_coverage {

UndockBehavior::UndockBehavior(
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Logger logger)
: m_undock_action_client(undock_action_client), m_logger(logger)
{

}

State UndockBehavior::execute(const Data & data)
{
    // Make sure we are not docked
    if (!m_undock_action_sent && !data.dock.is_docked) {
        RCLCPP_ERROR(m_logger, "Robot is already undocked!");
        return State::FAILURE;
    }

    // We can't undock until we discover the undocking action server
    if (!m_undock_action_client->action_server_is_ready()) {
        RCLCPP_DEBUG(m_logger, "Waiting for undock action server");
        return State::RUNNING;
    }

    // Send undock command if not already sent and if we are not waiting for result
    if (!m_undock_action_sent) {
        RCLCPP_INFO(m_logger, "Sending undocking goal!");
        auto goal_msg = UndockAction::Goal();

        auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleUndock::SharedPtr & goal_handle){
            m_undock_goal_handle_ready = true;
            m_undock_goal_handle = goal_handle;
        };
        send_goal_options.result_callback = [this](const GoalHandleUndock::WrappedResult & result){
            m_undock_result_ready = true;
            m_undock_result = result;
        };

        m_undock_action_client->async_send_goal(goal_msg, send_goal_options);
        m_undock_action_sent = true;

        return State::RUNNING;
    }

    if (m_undock_goal_handle_ready && !m_undock_goal_handle) {
        RCLCPP_ERROR(m_logger, "Undock goal was rejected by server");
        return State::FAILURE;
    }

    if (m_undock_result_ready) {
        if (m_undock_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Undocking succeeded!");
            return State::SUCCESS;
        } else {
            RCLCPP_ERROR(m_logger, "Undocking failed!");
            return State::FAILURE;
        }
    }

    return State::RUNNING;
}

void UndockBehavior::cleanup()
{
    // This behavior is being cancelled, so send a cancel request to dock action server if it's running
    if (!m_undock_result_ready && m_undock_goal_handle_ready && m_undock_goal_handle) {
        m_undock_action_client->async_cancel_goal(m_undock_goal_handle);
    }
}

} // namespace create3_coverage
