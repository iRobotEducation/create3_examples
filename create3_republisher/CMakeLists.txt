cmake_minimum_required(VERSION 3.5)
project(create3_republisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(create3_republisher)
target_sources(create3_republisher PRIVATE republisher.cpp)
target_link_libraries(
    create3_republisher
    PRIVATE
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
        ${irobot_create_msgs_TARGETS}
)

install(TARGETS create3_republisher
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bringup DESTINATION share/${PROJECT_NAME})
install(DIRECTORY dds-config DESTINATION share/${PROJECT_NAME})

ament_package()
