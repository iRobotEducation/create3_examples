
#include <math.h>

#include "create3_coverage/utils.hpp"

namespace create3_coverage {

double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

} // namespace create3_coverage
