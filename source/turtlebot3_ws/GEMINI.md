# Changes made by Gemini

I have modified the following file to fix the build issue:

*   `/home/ubuntu/turtlebot3_ws/src/turtlebot3_applications/turtlebot3_panorama/CMakeLists.txt`

The change was to add the `stitching` and `highgui` components to the `find_package(OpenCV ...)` call.

## Diff

```diff
--- a/src/turtlebot3_applications/turtlebot3_panorama/CMakeLists.txt
+++ b/src/turtlebot3_applications/turtlebot3_panorama/CMakeLists.txt
@@ -26,7 +26,7 @@
 find_package(geometry_msgs REQUIRED)
 find_package(image_transport REQUIRED)
 find_package(nav_msgs REQUIRED)
-find_package(OpenCV REQUIRED)
+find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc stitching)
 find_package(rclcpp REQUIRED)
 find_package(sensor_msgs REQUIRED)
 find_package(tf2 REQUIRED)

```

---

# Follower 로직 변경 및 설정 요약 (Side-by-Side Following)

2대의 로봇(리더 1, 팔로워 1)이 시뮬레이션에 생성되며, 팔로워가 리더의 오른쪽에서 나란히 따라가도록 설정을 변경했습니다.

## 1. Gazebo 시뮬레이션 설정

### `src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_robot.launch.py`

- **역할**: Gazebo 시뮬레이션을 실행하고 지정된 수의 로봇을 스폰합니다.
- **주요 변경 변수**:
    - `number_of_robots = 2`: 시뮬레이션에 생성할 총 로봇 수를 2대(리더 1, 팔로워 1)로 설정했습니다.
    - `pose = [[-0.8, -0.5], [-0.8, -1.0], ... ]`: 팔로워 로봇(두 번째 로봇)이 리더 로봇(첫 번째 로봇)의 오른쪽에서 시작하도록 초기 스폰 위치를 수정했습니다.

## 2. Follower 애플리케이션 설정

### `src/turtlebot3_applications/turtlebot3_follower/launch/turtlebot3_follower.launch.py`

- **역할**: Follower 핵심 노드를 실행하고, 관련된 `Nav2` 노드들을 관리합니다.
- **주요 변경 변수**:
    - `number_of_follower = 1`: 제어할 팔로워 로봇의 수를 1대로 설정했습니다.

### `src/turtlebot3_applications/turtlebot3_follower/src/follower.cpp`

- **역할**: 팔로워 로봇의 실제 주행 로직을 담당합니다.
- **주요 변경 로직**:
    - `send_path()` 함수 내 목표 지점 계산 방식 변경:
        - 기존: 리더의 위치를 그대로 목표로 설정 (`x`, `y`).
        - 변경: 리더의 위치에서 오른쪽으로 0.5m 떨어진 지점을 목표로 설정 (`x`, `y - 0.5`).
    - `const double lateral_offset = 0.5;` 변수를 통해 측면 거리 오프셋을 설정하여 평행 주행을 구현했습니다.

Comment: 되진 않음.
