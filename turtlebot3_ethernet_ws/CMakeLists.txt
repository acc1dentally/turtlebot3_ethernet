cmake_minimum_required(VERSION 3.8)
project(turtlebot3_ethernet_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(ethernet_bridge_node src/ethernet_bridge_node.cpp)
ament_target_dependencies(ethernet_bridge_node
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(ethernet_controller_node src/ethernet_controller_node.cpp)
ament_target_dependencies(ethernet_controller_node
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

install(TARGETS
 ethernet_bridge_node
  ethernet_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()