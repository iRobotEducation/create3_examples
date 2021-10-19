// Copyright 2021 iRobot Corporation. All Rights Reserved.

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
