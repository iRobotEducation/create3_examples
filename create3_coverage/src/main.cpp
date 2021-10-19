// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <memory>

#include "create3_coverage/create3_coverage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<create3_coverage::Create3CoverageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
