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

#include "geometry_msgs/msg/point.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"

namespace create3_coverage {

/**
 * @brief Computes Euclidean distance between two points
 */
double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

/**
 * @brief Inspects received opcodes to determine if robot is driving
 * towards the dock
 */
bool is_driving_towards_dock(const std::vector<irobot_create_msgs::msg::IrOpcode>& opcodes);

/**
 * @brief Inspects hazard detections to determine if any hazard is active.
 * This excludes BACKUP_LIMIT hazards.
 */
bool is_front_hazard_active(const irobot_create_msgs::msg::HazardDetectionVector& hazards);

} // namespace create3_coverage
