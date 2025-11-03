cmake_minimum_required(VERSION 3.8)
project(crop_task_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(crop_task_node_lib
  src/crop_task_node.cpp
)
target_include_directories(crop_task_node_lib PUBLIC
  include
)
ament_target_dependencies(crop_task_node_lib
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(crop_task_node_lib
  ${OpenCV_LIBRARIES}
)

add_executable(crop_task_node
  src/crop_task_main.cpp
)
target_link_libraries(crop_task_node
  crop_task_node_lib
)
ament_target_dependencies(crop_task_node
  rclcpp
)

add_executable(lidar_guard_node
  src/lidar_guard_node.cpp
  src/lidar_guard_main.cpp
)
target_include_directories(lidar_guard_node PUBLIC
  include
)
ament_target_dependencies(lidar_guard_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS
    crop_task_node_lib
    crop_task_node
    lidar_guard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(crop_task_node_lib)

ament_package()
