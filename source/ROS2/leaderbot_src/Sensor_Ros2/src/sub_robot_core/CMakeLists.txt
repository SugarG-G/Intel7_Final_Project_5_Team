cmake_minimum_required(VERSION 3.8)
project(sub_robot_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_controller
  src/sub_controller.cpp
)

target_include_directories(${PROJECT_NAME}_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_controller
  rclcpp::rclcpp
)

ament_target_dependencies(${PROJECT_NAME}_controller
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  angles
)

add_executable(sub_controller_node
  src/sub_controller_node.cpp
)

target_link_libraries(sub_controller_node
  ${PROJECT_NAME}_controller
)

target_include_directories(sub_controller_node PUBLIC include)

ament_target_dependencies(sub_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  angles
)

install(TARGETS
  ${PROJECT_NAME}_controller
  sub_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_package()
