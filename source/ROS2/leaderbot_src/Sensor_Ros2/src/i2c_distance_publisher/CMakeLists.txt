cmake_minimum_required(VERSION 3.8)
project(i2c_distance_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(i2c_distance_node
  src/i2c_distance_node.cpp
)
target_include_directories(i2c_distance_node PRIVATE
  include
)
ament_target_dependencies(i2c_distance_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    i2c_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
