#include "create3_coverage/behaviors/reflex-behavior.hpp"
#include "create3_coverage/utils.hpp"

namespace create3_coverage {

ReflexBehavior::ReflexBehavior(
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

State ReflexBehavior::execute(const Data & data)
{
    if (!m_first_run) {
        m_first_run = true;
        m_initial_position = data.pose.position;
    }

    // The reflexes are taking too much time to clear the hazards
    if (m_clock->now() - m_start_time > m_config.clear_hazard_time) {
        RCLCPP_INFO(m_logger, "Aborting reflexes because hazard is not getting cleared");
        return State::FAILURE;
    }

    bool hazards_detected = !data.hazards.detections.empty();
    //bool backup_limit_reached
    if (!hazards_detected) {
        RCLCPP_INFO(m_logger, "Reflex successfully cleared hazard");
        return State::SUCCESS; 
    }

    // Command a negative velocity to backup from hazard
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = - m_config.linear_vel;
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_coverage
