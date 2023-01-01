// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace create3_coverage {

class DriveStraightBehavior : public Behavior
{
public:
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;
    using TwistMsg = geometry_msgs::msg::Twist;

    struct Config
    {
        double max_distance {5};
        double min_distance {0.25};
        double linear_vel {0.25};
    };

    DriveStraightBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    ~DriveStraightBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::DRIVE_STRAIGHT; }

private:
    Config m_config;

    bool m_first_run;
    rclcpp::Time m_start_time;
    geometry_msgs::msg::Point m_initial_position;

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
};

} // namespace create3_coverage
