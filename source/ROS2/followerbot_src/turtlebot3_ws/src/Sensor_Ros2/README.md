# Sensor_Ros2 ROS 2 C++ MVP

메인 로봇 없이도 서브 로봇의 핵심 상태 루프를 검증하기 위한 ROS 2 실습 프로젝트입니다.

```
Follow → Task-Wait → Formation Align → Follow → Stop
```

## 현재 디렉터리 구조

```
Sensor_Ros2/
├── README.md                     # 프로젝트 개요 및 실습 가이드(현재 파일)
└── src/
    ├── sub_robot_core/           # 서브 로봇 FSM 및 제어 노드(C++)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/sub_robot_core/sub_controller.hpp
    │   ├── src/sub_controller.cpp        # FSM 제어 로직 골격 (TODO: 제어식 구현)
    │   ├── src/sub_controller_node.cpp   # ROS 2 노드 래퍼 (구독/발행, 타이머)
    │   └── params/controller.yaml        # 기본 파라미터 세트
    ├── sub_robot_sim/            # 메인 로봇 더미 오돔 발행 노드(C++)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/mock_main_odom.cpp        # 직선→원 궤적 오돔 퍼블리셔 구현
    │   └── params/mock_main_odom.yaml    # 궤적 파라미터
    └── sub_robot_bringup/        # 실행/테스트용 런치 & 파라미터
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/controller_params.yaml # controller.yaml 복사본 (런치에서 사용)
        ├── config/mock_main_odom.yaml
        └── launch/sub_robot_mvp.launch.py
```

## 각 패키지가 하는 역할

### sub_robot_core
- 노드 이름: `sub_controller_node`
- 구독: `/main/odom`(nav_msgs/Odometry, SensorDataQoS) / `/main/task_simple`(std_msgs/UInt8)
- 발행: `/sub/cmd_vel`(geometry_msgs/Twist)
- FSM 상태: `STOP`, `FOLLOW`, `TASK_WAIT`, `FORMATION_ALIGN`
- 제어 파라미터: target_distance, align_offset_x/y, 허용오차, 속도 제한, 타임아웃 등
- **실습 포인트**: `src/sub_controller.cpp` 안의 `compute_follow`, `compute_task_wait`, `compute_alignment` 함수에 실제 제어식을 작성해 FSM을 완성합니다.

### sub_robot_sim
- 노드 이름: `mock_main_odom`
- 20Hz 타이머로 직선→원 궤적을 생성해 `/main/odom` 주행 데이터를 발행합니다.
- 파라미터를 수정해 속도, 회전 반경, 직선 주행 시간 등을 조정할 수 있습니다.

### sub_robot_bringup
- 런치 파일 `sub_robot_mvp.launch.py`가 위 두 노드를 한 번에 실행합니다.
- `config/` 폴더에 기본 파라미터 YAML을 배치하여 런치 시 자동 로드합니다.

## 실습 목표 및 가능한 기능

1. **FSM 제어 로직 완성**
   - Follow: 메인 로봇의 바디 좌표 기준으로 target_distance 뒤, align_offset_y 옆 위치를 유지하도록 선/각속도를 계산합니다.
   - Task-Wait: 명령 값 2 시 즉시 정지 후 0속도를 유지합니다.
   - Formation Align: align_offset_x/y 목표 지점에 들어오면 FOLLOW로 복귀합니다.
   - Stop: 명령 값 0 또는 오돔 타임아웃 시 즉시 정지합니다.

2. **테스트 시나리오 검증**
   - `/main/task_simple` 값을 0/1/2/3으로 토글하여 Follow → Task-Wait → Formation Align → Follow → Stop 루프가 정상 동작하는지 확인합니다.
   - mock_main_odom 노드를 일시 중지하거나 파라미터를 조정해 타임아웃과 거리 유지 기능을 실험합니다.

3. **파라미터 튜닝**
   - controller_params.yaml에서 거리, 오프셋, 게인(k_lin, k_lat, k_yaw), 속도 제한을 조정하면서 제어 응답을 최적화합니다.

4. **추가 확장 아이디어**
   - 실제 센서(/sub/sensors/distance)와 연동하거나, ROS 2 액션 서버(Align/Corner/Follow)로 FSM을 구조화.
   - SLAM/Nav2, 하드웨어 통신 노드, Gazebo 시뮬레이션 등을 결합.

## 빌드 및 실행 방법

```bash
# 작업 공간 루트에서
colcon build --packages-select sub_robot_core sub_robot_sim sub_robot_bringup
source install/setup.bash

# 런치 실행
ros2 launch sub_robot_bringup sub_robot_mvp.launch.py
```

다른 터미널에서 명령 토픽을 퍼블리시하여 상태 전이를 확인할 수 있습니다.

```bash
source install/setup.bash
ros2 topic pub /main/task_simple std_msgs/UInt8 "data: 1"  # Follow
ros2 topic pub /main/task_simple std_msgs/UInt8 "data: 2"  # Task-Wait
ros2 topic pub /main/task_simple std_msgs/UInt8 "data: 3"  # Formation Align
ros2 topic pub /main/task_simple std_msgs/UInt8 "data: 0"  # Stop
```

## 참고 사항
- `package.xml`의 maintainer/라이선스는 실제 사용에 맞게 수정하세요.
- C++ 제어 로직은 skeleton 상태이므로, 실습 중 자유롭게 구현하고 테스트 데이터를 취합해 보세요.
- 모든 패키지는 `ament_cmake` 기반이며, 추가 의존성이 필요하면 CMakeLists와 package.xml에 `find_package`/`depend`를 추가하면 됩니다.

## 실습 일지 / 진행 기록

- **워크스페이스 재정비**: 기존 임베디드 템플릿 파일을 제거하고 ROS 2 Humble 기준 `sub_robot_core`, `sub_robot_sim`, `sub_robot_bringup` 세 패키지로 구조를 재구성했습니다. README와 launch, 파라미터 템플릿을 정리해 기초 환경을 마련했습니다.
- **메인/서브 통신 파이프라인 구축**: PC에서 `mock_main_odom`으로 더미 메인 로봇 오돔을 발행하고, 서브 FSM 노드가 `/main/task_simple` 명령을 받아 `/sub/cmd_vel`을 내보내도록 연결했습니다. 상태 전이가 로그(INFO/DEBUG)로 드러나도록 디버깅 출력도 추가했습니다.
- **제어 로직 초안 작성**: `sub_controller.cpp`에 내부 포즈 추정을 두고 Follow/Align에서 비례 제어(선속·각속)에 기반한 출력이 나오도록 구현했습니다. Task-Wait에서는 감속 후 정지, Align 시에는 align_offset_x/y를 만족하면 Follow로 복귀하도록 했습니다.
- **속도/파라미터 튜닝**: 테스트 결과 속도를 낮출 필요가 있어 `controller_params.yaml`에서 `max_lin`, `max_ang`, 게인을 조정해 약 0.6배 속도로 동작하도록 맞췄습니다. 이후에도 파라미터 수정으로 응답 특성을 계속 확인했습니다.
- **라즈베리 파이 TurtleBot 배포**: PC에서 `scp`로 Sensor_Ros2를 Pi의 `~/ros2_ws/src/`에 전송하고, Pi에서 `colcon build` 후 `sub_controller_node`를 `/cmd_vel`과 remap해서 실행했습니다. `/main/task_simple` 명령을 PC에서 보내 실제 TurtleBot이 Follow→Task-Wait→Alignment→Stop 시나리오를 수행하는지 검증했습니다.
- **테스트 절차 정리**: Follow 3초 이동→Task-Wait→Formation Align→Follow→Stop 등 명령 시퀀스를 문서로 정리했고, `/sub/cmd_vel`, `/main/odom`을 `ros2 topic echo`, `rqt_plot`, RViz로 모니터링하는 방법을 제안했습니다.
- **다음 단계 구상**: 센서 없는 단순 상대 제어에서 출발해, 추후 SLAM/Nav2·센서 토픽 연동·액션 서버 확장을 통해 실제 자율주행으로 이어가야 한다는 가이드라인을 마련했습니다.
- **2025-10-07 1차 실습 (벽따라 주행)**: 워크스페이스에서 `colcon build --packages-select wall_follower` 후 `source install/setup.bash`로 환경을 준비하고, 지도 `/home/ubuntu/map.yaml`을 사용하는 `ros2 launch wall_follower wall_follow.launch.py`를 실행해 벽과 20cm 간격으로 주행을 수행했다. 전용 터미널에서 `P`/`S`로 정지·재개를 검증하며 `ros2 topic echo wall_follow/status`, `ros2 topic echo /cmd_vel`, `ros2 topic echo /map_metadata --once`로 상태·속도·지도 로딩을 확인했다.
- **2025-10-10 단일 터틀봇 ‘ㄷ’ 경로 실습 준비**: `turtlebot3_follower`를 Nav2 `FollowPath` 기반에서 `cmd_vel` 직접 제어 구조로 리팩터링하고, YAML 기반 waypoint 로더·P제어 루프·TF 조회 유틸을 추가했다. `/home/ubuntu/map.yaml`을 기준으로 벽에서 0.25 m 떨어진 ‘ㄷ’자 4포인트를 산출해 `param/single_d_path_waypoints.yaml`로 정리하고, map_server 라이프사이클 활성화와 `map↔odom` 정적 TF, `use_sim_time` 동기화 절차를 문서화했다. Gazebo·RViz 환경에서 좌표 추출 및 TF 경고(TF_OLD_DATA, use_sim_time 중복 선언) 문제를 점검하며 재현 절차와 해결책(파라미터 중복 제거, 시간 동기, 정밀 좌표 재측정 스크립트)을 기록했다.
- **2025-10-12 TurtleBot3 웨이포인트 자율주행**: `/odom`에서 point1(0,0), point2(1.8573,-0.1178, yaw≈-92.5°), point3(1.7554,-1.9248, yaw≈178.9°), point4(-0.0971,-1.9213, yaw≈90.8°)을 재측정해 `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml`에 반영했다. `turtlebot3_patrol_server.py`의 `linear_kp=0.2`, `angular_kp=1.0`, `max_linear_speed=0.07`, `max_angular_speed=0.4`, `heading_slowdown_gain=0.85`로 속도를 약 3배 낮춰 부드러운 순환 주행을 구현했고, Gazebo 빈 월드 실행→spawn→서버/클라이언트 구동→RViz 모니터링까지 테스트 절차와 `/odom`·`tf2_echo`를 통한 좌표 검증 방법을 정리했다.

## 벽따라 주행 1차 실습

- 버거 기준으로 벽과 20cm 간격을 유지하며 직선/코너를 부드럽게 주행합니다.
- `/home/ubuntu/map.yaml`과 `map.pgm`을 로드해 현재 지형 정보를 참조합니다.
- 전용 터미널 노드에서 현재 상태, 위치, 속도를 한글로 확인하고 `P`/`S` 키로 정지/재개합니다.
- 코너 4회를 통과하면 초기 자세로 복귀하고 주행을 종료합니다.

## 벽따라 주행 2차 실습 (2025-10-12 TurtleBot3 웨이포인트 자율주행)

- SLAM으로 확보한 실제 맵에서 TurtleBot3가 point1 → point2 → point3 → point4 → point1_return 순환 경로를 따라가도록 `turtlebot3_example` 패키지를 커스터마이징했습니다.
- `/odom` 기준으로 네 포인트의 위치와 yaw를 재측정해 `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml`에 반영했습니다.  
  - point1: (0.0000, 0.0000, yaw ≈ 0°)  
  - point2: (1.8573, -0.1178, yaw ≈ -92.5°)  
  - point3: (1.7554, -1.9248, yaw ≈ 178.9°)  
  - point4: (-0.0971, -1.9213, yaw ≈ 90.8°)
- `turtlebot3_patrol_server.py`의 제어 파라미터를 느린 주행에 맞춰 조정했습니다.  
  - `linear_kp` 0.2 → 선속도 응답 완만화  
  - `angular_kp` 1.0  
  - `max_linear_speed` 0.07 m/s, `max_angular_speed` 0.4 rad/s → 기존보다 약 3배 느린 주행  
  - `heading_slowdown_gain` 0.85 → 방향 오차가 클 때 선속 감속 강화
- Gazebo + RViz 환경에서 아래 순서로 실습을 반복했습니다.  
  1. `colcon build --packages-select turtlebot3_example --allow-overriding turtlebot3_example`  
  2. `source install/setup.bash` & `export TURTLEBOT3_MODEL=burger`  
  3. `ros2 launch turtlebot3_gazebo empty_world.launch.py`로 빈 월드를 실행  
  4. 필요 시 `ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x:=0.0 y:=0.0 yaw:=0.0`로 로봇 스폰  
  5. `ros2 run turtlebot3_example turtlebot3_patrol_server`로 웨이포인트 서버 실행  
  6. `ros2 run turtlebot3_example turtlebot3_patrol_client` → `mode(s, t, c): c`, `patrol_count: 1`로 순환 주행  
  7. `ros2 run rviz2 rviz2`에서 Fixed Frame을 `odom`으로 설정하고 `RobotModel`, `TF`, `Odometry`를 확인
- `/odom`을 `ros2 topic echo /odom`으로 모니터링하고, `ros2 run tf2_ros tf2_echo odom base_footprint`로 yaw를 확인하여 좌표가 의도대로 이동하는지 검증했습니다.
- 실제 TurtleBot3에서도 동일한 순서를 적용할 수 있도록 파라미터·순서·검증 방법을 README에 정리했습니다.

### 패키지 구성

```
wall_follower/
├── launch/wall_follow.launch.py        # 지도 로드 + 컨트롤러 + 터미널 실행
├── config/wall_follow_params.yaml      # 벽 간격, 속도, 게인 파라미터 (기본 20cm)
├── wall_follower/wall_follow_controller.py  # 라이다 기반 벽따라 제어 FSM
└── wall_follower/wall_follow_terminal.py    # 상태 모니터 & 키보드 명령 (P/S/R/Q)
```

### 실행 방법

```bash
# 패키지 빌드
colcon build --packages-select wall_follower
source install/setup.bash

# 기본 지도(/home/ubuntu/map.yaml)를 사용해 실행
ros2 launch wall_follower wall_follow.launch.py
```

- 다른 지도를 사용하려면 `map:=<다른_yaml>` 인자를 넣어 실행합니다.
- `wall_follow_params.yaml`에서 원하는 거리(`desired_distance`), 속도, 코너 판단 임계값을 조정할 수 있습니다.
- 터미널 창에서 출력되는 상태 메시지를 통해 위치·속도·지도 정보를 확인할 수 있습니다.
- `P`: 즉시 정지, `S`: 재개, `R`: 리셋(시작 위치부터 다시 주행), `Q`: 노드 종료.
