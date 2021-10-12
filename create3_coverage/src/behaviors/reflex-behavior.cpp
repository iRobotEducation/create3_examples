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
