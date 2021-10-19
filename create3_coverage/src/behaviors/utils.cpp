// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <math.h>

#include "utils.hpp"

#include <iostream>
namespace create3_coverage {

double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

bool is_driving_towards_dock(const std::vector<irobot_create_msgs::msg::IrOpcode>& opcodes)
{
    using irobot_create_msgs::msg::IrOpcode;

    // We consider the robot to be driving towards the dock if both IR receivers are detecting it.
    // The directional sensor will tell us that we are pointing towards it.
    // The omni sensors will tell us that we are close to it.

    bool dir_dock_detection = false;
    bool omni_dock_detection = false;

    for (const IrOpcode& msg : opcodes) {
        if (msg.sensor == IrOpcode::SENSOR_DIRECTIONAL_FRONT && msg.opcode != IrOpcode::CODE_IR_VIRTUAL_WALL) {
            dir_dock_detection = true;
            continue;
        }
        if (msg.sensor == IrOpcode::SENSOR_OMNI && msg.opcode != IrOpcode::CODE_IR_VIRTUAL_WALL) {
            omni_dock_detection = true;
            continue;
        }
    }

    return dir_dock_detection && omni_dock_detection;    
}

bool is_front_hazard_active(const irobot_create_msgs::msg::HazardDetectionVector& hazards)
{
    using irobot_create_msgs::msg::HazardDetection;

    for (const HazardDetection& detection : hazards.detections) {
        if (detection.type != HazardDetection::BACKUP_LIMIT) {
            return true;
        }
    }

    return false;
}

} // namespace create3_coverage
