// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_coverage/behaviors/spiral-behavior.hpp"
#include "utils.hpp"

namespace create3_coverage {

SpiralBehavior::SpiralBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_config = config;
    m_clock = clock;

    m_start_time = m_clock->now();
    m_last_radius_update_time = m_start_time;
    m_radius = m_config.initial_radius;
}

State SpiralBehavior::execute(const Data & data)
{
    auto now = m_clock->now();
    rclcpp::Duration spiral_time = now - m_start_time;
    if (spiral_time > m_config.spiral_duration) {
        RCLCPP_INFO(m_logger, "Spiral completed!");
        return State::SUCCESS;
    }

    bool driving_towards_dock = is_driving_towards_dock(data.opcodes);
    bool hazards_detected = is_front_hazard_active(data.hazards);
    // Pointing towards dock or found hazard
    if (driving_towards_dock || hazards_detected) {
        RCLCPP_INFO(m_logger, "Stop spiraling: time spent %f/%f hazards %ld dock %d",
        spiral_time.seconds(), m_config.spiral_duration.seconds(),
        data.hazards.detections.size(), driving_towards_dock);
        return State::FAILURE;
    }

    if (now - m_last_radius_update_time > m_config.radius_increment_interval) {
        m_radius += m_config.radius_increment;
        m_last_radius_update_time = now;
    }

    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = m_config.linear_vel;
    twist_msg->angular.z = m_config.linear_vel / m_radius;

    RCLCPP_DEBUG(m_logger, "Spiral velocities: linear %f angular %f",
        twist_msg->linear.x, twist_msg->angular.z);

    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_coverage
