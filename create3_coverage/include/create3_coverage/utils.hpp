#pragma once

#include "geometry_msgs/msg/point.hpp"

namespace create3_coverage {

double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

} // namespace create3_coverage
