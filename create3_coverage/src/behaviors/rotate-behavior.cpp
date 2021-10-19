// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <cmath>

#include "create3_coverage/behaviors/rotate-behavior.hpp"
#include "tf2/utils.h"

#include "utils.hpp"

namespace create3_coverage {

RotateBehavior::RotateBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_clock = clock;
    m_config = config;

    m_first_run = false;
    m_start_time = m_clock->now();
    m_hazards_count = 0;
}

State RotateBehavior::execute(const Data & data)
{
    if (m_hazards_count > m_config.max_hazard_retries) {
        RCLCPP_INFO(m_logger, "Failed to clear hazard! Too many trials");
        return State::FAILURE;
    }

    // Use reflexes to handle hazards if we have hazard detections
    State reflex_state = handle_hazards(data);
    // Handle RUNNING and FAILURE reflex states.
    if (reflex_state != State::SUCCESS) {
        return reflex_state;
    }
    m_reflex_behavior.reset();

    if (!m_first_run) {
        // After reflex returns SUCCESS, we are ready to start the behavior
        m_first_run = true;
        tf2::convert(data.pose.orientation, m_initial_orientation);

        RCLCPP_DEBUG(m_logger, "Rotation initial yaw: %f", tf2::getYaw(m_initial_orientation));
    }

    tf2::Quaternion current_orientation;
    tf2::convert(data.pose.orientation, current_orientation);
    tf2::Quaternion relative_orientation = current_orientation * m_initial_orientation.inverse();

    double relative_yaw = tf2::getYaw(relative_orientation);
    if (std::abs(relative_yaw) >= std::abs(m_config.target_rotation)) {
        RCLCPP_INFO(m_logger, "Rotation completed: from %f to %f",
            tf2::getYaw(m_initial_orientation), tf2::getYaw(current_orientation));
        return State::SUCCESS;
    }


    RCLCPP_DEBUG(m_logger, "Rotating: current orientation %f progress %f/%f",
        tf2::getYaw(current_orientation), relative_yaw, m_config.target_rotation);

    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->angular.z = std::copysign(m_config.angular_vel, m_config.target_rotation);
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

State RotateBehavior::handle_hazards(const Data & data)
{
    if (!is_front_hazard_active(data.hazards) && !m_reflex_behavior) {
        return State::SUCCESS;
    }

    if (m_config.robot_has_reflexes) {
        // Wait for reflexes to clear the hazards.
        // Making sure that they do not take too much time.
        if (m_clock->now() - m_start_time > m_config.clear_hazard_time) {
            RCLCPP_INFO(m_logger, "Aborting ROTATE because initial hazard is not getting cleared");
            return State::FAILURE;
        }

        // Return SUCCESS or RUNNING depending on whether the hazards have been cleared
        if (!is_front_hazard_active(data.hazards)) {
            return State::SUCCESS;
        } else {
            return State::RUNNING;
        }
    } else {
        // Run a reflex behavior to clear the hazard because the robot does not have
        // active reflexes.

        // Initialize the reflex behavior if necessary
        if (!m_reflex_behavior) {
            m_hazards_count++;
            RCLCPP_INFO(m_logger, "Starting reflex behavior to clear hazard");

            auto config = ReflexBehavior::Config();
            config.clear_hazard_time = m_config.clear_hazard_time;
            config.backup_distance = 0.05;
            config.linear_vel = 0.1;
            m_reflex_behavior = std::make_unique<ReflexBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
        }

        // Run the reflex behavior
        auto reflex_state = m_reflex_behavior->execute(data);
        return reflex_state;
    }
}

} // namespace create3_coverage
