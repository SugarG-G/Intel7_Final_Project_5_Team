cmake_minimum_required(VERSION 3.8)
project(robot_arm)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # srv 빌드용

# 서비스 파일 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RobotArmCommand.srv"
)

# 실행 파일이 생성된 인터페이스를 사용할 수 있도록 런타임 의존성 익스포트
ament_export_dependencies(rosidl_default_runtime)

# 서버 노드
add_executable(robot_arm_server src/robot_arm_server.cpp)
ament_target_dependencies(robot_arm_server rclcpp)
# 동일 패키지에서 생성된 srv의 C++ 타입서포트에 연결
rosidl_target_interfaces(robot_arm_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
# 클라이언트 노드
add_executable(robot_arm_client src/robot_arm_client.cpp)
ament_target_dependencies(robot_arm_client rclcpp)
# 동일 패키지에서 생성된 srv의 C++ 타입서포트에 연결
rosidl_target_interfaces(robot_arm_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
# 설치
install(TARGETS
  robot_arm_server
  robot_arm_client
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트용 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
