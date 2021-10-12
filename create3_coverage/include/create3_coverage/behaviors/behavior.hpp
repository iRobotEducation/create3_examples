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

#include <vector>

#include "create3_coverage/state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "irobot_create_msgs/msg/dock.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"

namespace create3_coverage {

class Behavior
{
public:
    struct Data {
        geometry_msgs::msg::Pose pose;
        irobot_create_msgs::msg::HazardDetectionVector hazards;
        irobot_create_msgs::msg::Dock dock;
        std::vector<irobot_create_msgs::msg::IrOpcode> opcodes;
    };

    Behavior() = default;

    virtual ~Behavior() = default;

    virtual State execute(const Data & data) = 0;

    virtual int32_t get_id() const = 0;

    virtual void cleanup() {}
};

} // namespace create3_coverage
