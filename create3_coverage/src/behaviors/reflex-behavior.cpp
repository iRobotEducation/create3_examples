// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_coverage/behaviors/reflex-behavior.hpp"
#include "utils.hpp"

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

        if (!is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "No need to run reflex");
            return State::SUCCESS;
        }
    }

    bool timeout = m_clock->now() - m_start_time > m_config.clear_hazard_time;
    bool moved_enough = get_distance(data.pose.position, m_initial_position) > m_config.backup_distance;
    bool limit_reached = backup_limit_reached(data.hazards);

    if (timeout || moved_enough || limit_reached) {
        if (is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "Reflex failed: was not able to clear hazard (timeout %d distance %d backup %d)",
                timeout, moved_enough, limit_reached);
            return State::FAILURE;
        } else {
            RCLCPP_INFO(m_logger, "Reflex successfully cleared hazard");
            return State::SUCCESS;
        }
    }

    // Command a negative velocity to backup from hazard
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = - m_config.linear_vel;
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

bool ReflexBehavior::backup_limit_reached(const irobot_create_msgs::msg::HazardDetectionVector& hazards)
{
    auto limit_hazard = std::find_if(hazards.detections.begin(), hazards.detections.end(),
        [](const irobot_create_msgs::msg::HazardDetection& detection){
            return (detection.type == irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT);
        });
    
    return limit_hazard != hazards.detections.end();
}

} // namespace create3_coverage
