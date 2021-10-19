// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace create3_coverage {

class SpiralBehavior : public Behavior
{
public:
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;
    using TwistMsg = geometry_msgs::msg::Twist;

    struct Config
    {
        rclcpp::Duration spiral_duration {rclcpp::Duration(std::chrono::seconds(30))};
        double linear_vel {0.4};
        double initial_radius {0.25};
        double radius_increment {0.25};
        rclcpp::Duration radius_increment_interval {rclcpp::Duration(std::chrono::seconds(5))};
    };

    SpiralBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    ~SpiralBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::SPIRAL; }

private:
    rclcpp::Time m_start_time;
    rclcpp::Time m_last_radius_update_time;
    double m_radius;

    Config m_config;

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
};

} // namespace create3_coverage
