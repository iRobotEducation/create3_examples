#include "create3_coverage/behaviors/spiral-behavior.hpp"

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
    // Pointing towards dock during spiral
    auto dock_front_detection = std::find_if(data.opcodes.begin(), data.opcodes.end(), [](const OpCodeMsg& msg){
        return (msg.sensor == OpCodeMsg::SENSOR_DIRECTIONAL_FRONT);
    });
    bool driving_towards_dock = dock_front_detection != data.opcodes.end();
    bool hazards_detected = !data.hazards.detections.empty();

    // Pointing towards dock or found hazard
    if (driving_towards_dock || hazards_detected) {
        RCLCPP_INFO(m_logger, "Can't keep spiraling: hazard %d dock %d",
        hazards_detected, driving_towards_dock);
        return State::FAILURE;
    }

    auto now = m_clock->now();
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

    if (now - m_start_time > m_config.spiral_duration) {
        RCLCPP_INFO(m_logger, "Spiral completed!");
        return State::SUCCESS;
    }

    return State::RUNNING;
}

} // namespace create3_coverage
