cmake_minimum_required(VERSION 3.8)
project(sub_robot_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mock_main_odom
  src/mock_main_odom.cpp
)

target_include_directories(mock_main_odom PUBLIC include)

ament_target_dependencies(mock_main_odom
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS mock_main_odom
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_package()
