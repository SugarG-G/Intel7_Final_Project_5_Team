# Grove ToF Simple

STM32와 Grove ToF 센서에서 거리 데이터를 받아 ROS2 토픽으로 발행하는 간단한 패키지입니다. 현재 미완성.

## 사용법

### 1. 빌드
```bash
cd ~/ros2_ws
source install/setup.bash
colcon build --packages-select grove_tof_simple
```

### 2. 실행
```bash
source install/setup.bash
ros2 run grove_tof_simple grove_distance_publisher
ros2 run grove_tof_simple grove_distance_subscriber
