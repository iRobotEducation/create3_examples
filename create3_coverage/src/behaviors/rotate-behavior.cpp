#include <cmath>

#include "create3_coverage/behaviors/rotate-behavior.hpp"
#include "tf2/utils.h"

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
}

State RotateBehavior::execute(const Data & data)
{
    if (!m_first_run) {

        // The reflexes are taking too much time to clear the hazards
        if (m_clock->now() - m_start_time > m_config.clear_hazard_time) {
            RCLCPP_INFO(m_logger, "Aborting ROTATE because initial hazard is not getting cleared");
            return State::FAILURE;
        }

        if (!data.hazards.detections.empty()) {
            return State::RUNNING;
        }

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

    // Found hazard while rotating!
    if (!data.hazards.detections.empty()) {
        RCLCPP_INFO(m_logger, "Found hazard during rotation!");
        return State::FAILURE;
    }

    RCLCPP_DEBUG(m_logger, "Rotating: current orientation %f progress %f/%f",
        tf2::getYaw(current_orientation), relative_yaw, m_config.target_rotation);

    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->angular.z = std::copysign(m_config.angular_vel, m_config.target_rotation);
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_coverage
