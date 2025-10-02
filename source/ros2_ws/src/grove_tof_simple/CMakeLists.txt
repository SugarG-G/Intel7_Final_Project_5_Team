cmake_minimum_required(VERSION 3.8)
project(grove_tof_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 실행 파일 추가
add_executable(grove_distance_publisher src/grove_distance_publisher.cpp)
add_executable(grove_distance_subscriber src/grove_distance_subscriber.cpp)

# 의존성 연결
ament_target_dependencies(grove_distance_publisher 
  rclcpp 
  std_msgs
)

ament_target_dependencies(grove_distance_subscriber 
  rclcpp 
  std_msgs
)

# 실행 파일 설치
install(TARGETS
  grove_distance_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  grove_distance_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 설정 파일 설치
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()