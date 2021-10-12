// Copyright 2021 iRobot Corporation. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "create3_coverage/behaviors/behavior.hpp"
#include "create3_coverage/behaviors/reflex-behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace create3_coverage {

class RotateBehavior : public Behavior
{
public:
    using TwistMsg = geometry_msgs::msg::Twist;

    struct Config
    {
        double target_rotation {0.785398};
        double angular_vel {0.6};
        bool robot_has_reflexes {false};
        rclcpp::Duration clear_hazard_time {rclcpp::Duration(std::chrono::seconds(2))};
    };

    RotateBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    ~RotateBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::ROTATE; }

    double get_rotation_amount() { return m_rotation_amount; }

private:
    State handle_hazards(const Data & data);

    Config m_config;
    double m_rotation_amount;

    bool m_first_run;
    rclcpp::Time m_start_time;
    tf2::Quaternion m_initial_orientation;

    std::unique_ptr<ReflexBehavior> m_reflex_behavior;

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
};

} // namespace create3_coverage
