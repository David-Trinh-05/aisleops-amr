cmake_minimum_required(VERSION 3.16)
project(aisleops_localization)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_map_monitor src/scan_map_monitor.cpp)
target_compile_features(scan_map_monitor PRIVATE cxx_std_17)
target_compile_options(scan_map_monitor PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(scan_map_monitor
  diagnostic_msgs geometry_msgs message_filters nav_msgs rcl_interfaces rclcpp sensor_msgs tf2
  tf2_geometry_msgs tf2_ros
)

install(TARGETS scan_map_monitor DESTINATION lib/${PROJECT_NAME})
ament_package()
