# 인텔 7기 최종 프로젝트 깃허브 #
## 중요!!!! 자료 및 코드는 본인 브렌치에서 작성 ##

---

## 2025-10-12 작업 기록

### 목표
- Gazebo/RViz 환경에서 TurtleBot3를 SLAM으로 확보한 실제 좌표에 맞춰 사각 경로로 자율 주행
- 주행 속도와 게인을 조정해 실기 환경에서도 안정적으로 반복 순환하도록 개선

### 진행 내용
- `/odom` 기준으로 4개 포인트의 좌표/자세 측정  
  - point1: (0.0000, 0.0000, yaw ≈ 0°)  
  - point2: (1.8573, -0.1178, yaw ≈ -92.5°)  
  - point3: (1.7554, -1.9248, yaw ≈ 178.9°)  
  - point4: (-0.0971, -1.9213, yaw ≈ 90.8°)
- `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml`을 위 좌표로 갱신하여 point1 → point2 → point3 → point4 → point1_return 순환 경로 구성
- `turtlebot3_patrol_server.py` 파라미터를 느린 주행에 맞게 조정  
  - `linear_kp` 0.2 → 부드러운 선속도 제어  
  - `angular_kp` 1.0  
  - `max_linear_speed` 0.07 m/s, `max_angular_speed` 0.4 rad/s  
  - `heading_slowdown_gain` 0.85로 헤딩 오차 시 감속 강화
- Gazebo 빈 월드 + `spawn_turtlebot3`로 로봇 초기화 후, `turtlebot3_patrol_client` 모드 `c`(waypoint loop)로 주행 테스트
- RViz에서 Fixed Frame `odom`으로 설정해 로봇 모델과 경로 확인, `/odom` 에코/`tf2_echo` 명령으로 실시간 상태 점검

### 실행 순서 메모
1. `colcon build --packages-select turtlebot3_example --allow-overriding turtlebot3_example`
2. `source /home/ubuntu/turtlebot3_ws/install/setup.bash` & `export TURTLEBOT3_MODEL=burger`
3. Gazebo 빈 월드: `ros2 launch turtlebot3_gazebo empty_world.launch.py`
4. (필요 시) 로봇 스폰: `ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x:=0.0 y:=0.0 yaw:=0.0`
5. 서버 실행: `ros2 run turtlebot3_example turtlebot3_patrol_server`
6. 클라이언트 실행: `ros2 run turtlebot3_example turtlebot3_patrol_client` → `mode(s, t, c): c`, `patrol_count: 1`
7. RViz 시각화: `ros2 run rviz2 rviz2` → Fixed Frame `odom`, `RobotModel`/`TF` 추가

### 향후 체크포인트
- 맵 좌표 오차 발생 시 `waypoints.yaml` 위치/자세 재측정 및 재빌드
- 실기에서 미끄러짐이 크면 `position_tolerance`, `yaw_tolerance_deg`, `max_linear_speed`를 추가로 조정
- 필요 시 속도 파라미터를 실시간으로 `ros2 param set /turtlebot3_patrol_server ...` 방식으로 재튜닝
