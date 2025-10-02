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
# 직접 실행
ros2 run grove_tof_simple tof_node

# 런치 파일로 실행
ros2 launch grove_tof_simple tof_sensor.launch.py

# 파라미터 변경해서 실행
ros2 run grove_tof_simple tof_node --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=9600
```

### 3. 토픽 확인
```bash
# 토픽 리스트 확인
ros2 topic list

# 거리 데이터 확인
ros2 topic echo /tof_range

# 토픽 정보 확인
ros2 topic info /tof_range
```

## STM32 코드 예시

STM32에서 다음 형태로 데이터를 전송해야 합니다: 합의 예정

```c
// UART로 전송
sprintf(buffer, "DIST:%d\n", distance_mm);
HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
```

## 파라미터

- `serial_port`: 시리얼 포트 경로 (기본값: "/dev/ttyUSB0")
- `baud_rate`: 보드레이트 (기본값: 115200)
- `publish_rate`: 발행 주기 Hz (기본값: 10.0)
- `frame_id`: TF 프레임 ID (기본값: "tof_link")

## 토픽

### Published Topics
- `/tof_range` (sensor_msgs/Range): 거리 측정 데이터

## 의존성

- rclcpp
- sensor_msgs
