// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include <vector>

#include "create3_coverage/state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"

namespace create3_coverage {

class Behavior
{
public:
    struct Data {
        geometry_msgs::msg::Pose pose;
        irobot_create_msgs::msg::HazardDetectionVector hazards;
        irobot_create_msgs::msg::DockStatus dock;
        std::vector<irobot_create_msgs::msg::IrOpcode> opcodes;
    };

    Behavior() = default;

    virtual ~Behavior() = default;

    virtual State execute(const Data & data) = 0;

    virtual int32_t get_id() const = 0;

    virtual void cleanup() {}
};

} // namespace create3_coverage
