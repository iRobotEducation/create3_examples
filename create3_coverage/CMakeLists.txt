cmake_minimum_required(VERSION 3.5)
project(create3_coverage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(create3_examples_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(executable_name create3_coverage)
set(library_name ${executable_name}_core)

set(dependencies
    create3_examples_msgs
    geometry_msgs
    irobot_create_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    tf2_geometry_msgs
)

add_library(${library_name} SHARED
    src/behaviors/dock-behavior.cpp
    src/behaviors/drive-straight-behavior.cpp
    src/behaviors/reflex-behavior.cpp
    src/behaviors/rotate-behavior.cpp
    src/behaviors/spiral-behavior.cpp
    src/behaviors/undock-behavior.cpp
    src/behaviors/utils.cpp
    src/coverage_state_machine.cpp
    src/create3_coverage_node.cpp
)
target_include_directories(${library_name} PUBLIC include)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(${executable_name} src/main.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name})

install(TARGETS ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
