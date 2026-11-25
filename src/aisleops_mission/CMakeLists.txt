cmake_minimum_required(VERSION 3.16)
project(aisleops_mission)

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(delivery_mission src/delivery_mission.cpp)
target_compile_features(delivery_mission PRIVATE cxx_std_17)
target_compile_options(delivery_mission PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(delivery_mission
  nav2_msgs nav_msgs rclcpp rclcpp_action std_msgs std_srvs
)

install(TARGETS delivery_mission DESTINATION lib/${PROJECT_NAME})
ament_package()
