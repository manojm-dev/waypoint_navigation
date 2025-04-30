cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

#Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add include directory
include_directories(include)

# Common dependencies
set(COMMON_DEPENDENCIES
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  nlohmann_json
  tf2
  tf2_ros
)

add_executable(waypoint_navigator src/waypoint_navigator_node.cpp)
ament_target_dependencies(waypoint_navigator ${COMMON_DEPENDENCIES})

install(
  TARGETS waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
