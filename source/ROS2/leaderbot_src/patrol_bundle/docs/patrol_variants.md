# TurtleBot3 Patrol Variants – Design Staging

본 문서는 실제 코드를 수정하기 전에 세 가지 패트롤 시나리오를 정리해 둔 초안입니다.  
각 섹션에는 필요한 파일과 코드 조각을 모두 포함했으므로, 나중에 구현할 때 그대로 옮겨 사용할 수 있습니다.

---

## 1. 4-웨이포인트 단순 순환 (Python 유지)

### 적용 대상
- `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml` – 좌표 4개(복귀 포함 5개)만 사용

### 제안 YAML
```yaml
waypoints:
  - name: point1
    position: {x: 0.0, y: 0.0}
    yaw_deg: 0.0
  - name: point2
    position: {x: 1.8, y: 0.0}
    yaw_deg: -90.0
  - name: point3
    position: {x: 1.8, y: -1.9}
    yaw_deg: 180.0
  - name: point4
    position: {x: 0.0, y: -1.9}
    yaw_deg: 90.0
  - name: point1_return
    position: {x: 0.0, y: 0.0}
    yaw_deg: 0.0
```

### 주요 포인트
- `/odom` 기준 상대 좌표로 입력되어 있으므로 실행 전 로봇 시작 자세가 동일해야 합니다.
- 제자리 회전(코너 정렬)이 필요 없다면 `turtlebot3_patrol_server.py` 의 `drive_to_waypoint()` 내 `reached_position` 분기에서 최소 선속도를 유지하도록 추가 수정이 필요합니다.

---

## 2. 16-웨이포인트 곡선 보간 순환 (Python 유지)

### 적용 대상
- `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml` – 코너마다 3~4개 웨이포인트 추가

### 제안 YAML
> yaw 값은 각 구간에서 자연스럽게 회전하도록 15~30° 단위로 변화시켰습니다.  
> 필요하면 실제 지도에 맞춰 수치만 조정하세요.

```yaml
waypoints:
  - name: p01_start
    position: {x: 0.0, y: 0.0}
    yaw_deg: 0.0
  - name: p02_pre_corner
    position: {x: 0.5, y: -0.02}
    yaw_deg: -15.0
  - name: p03_corner_blend
    position: {x: 1.0, y: -0.08}
    yaw_deg: -40.0
  - name: p04_right_entry
    position: {x: 1.5, y: -0.20}
    yaw_deg: -75.0
  - name: p05_right_mid_1
    position: {x: 1.8, y: -0.40}
    yaw_deg: -90.0
  - name: p06_right_mid_2
    position: {x: 1.8, y: -0.80}
    yaw_deg: -95.0
  - name: p07_right_exit
    position: {x: 1.75, y: -1.20}
    yaw_deg: -110.0
  - name: p08_lower_entry
    position: {x: 1.60, y: -1.55}
    yaw_deg: -135.0
  - name: p09_lower_mid_1
    position: {x: 1.20, y: -1.85}
    yaw_deg: -170.0
  - name: p10_lower_mid_2
    position: {x: 0.80, y: -1.90}
    yaw_deg: 180.0
  - name: p11_lower_exit
    position: {x: 0.40, y: -1.85}
    yaw_deg: 150.0
  - name: p12_left_entry
    position: {x: 0.10, y: -1.70}
    yaw_deg: 120.0
  - name: p13_left_mid_1
    position: {x: 0.00, y: -1.30}
    yaw_deg: 90.0
  - name: p14_left_mid_2
    position: {x: 0.00, y: -0.90}
    yaw_deg: 60.0
  - name: p15_left_exit
    position: {x: 0.00, y: -0.45}
    yaw_deg: 25.0
  - name: p16_home
    position: {x: 0.00, y: 0.00}
    yaw_deg: 0.0
```

### 주요 포인트
- 기존 `drive_to_waypoint()` 로직을 그대로 사용해도 웨이포인트 간 간격이 짧아져 곡선처럼 보입니다.
- I2C 거리 센서 기반 벽 추종을 추가하려면 `heading_error` 계산 이후에 벽거리 오차 보정 항을 더하면 됩니다.

---

## 3. 8-웨이포인트 0.45m 순환

### 적용 대상
- `turtlebot3_example/turtlebot3_patrol/config/waypoints.yaml` – 8개 포인트로 간단한 코너 보간
- 시각 참고: `docs/patrol_waypoints_10points.png` (앞쪽 보간 포인트를 줄여도 전체 궤적은 동일)

### 제안 YAML
```yaml
waypoints:
  - name: point1_start
    position: {x: 0.0, y: 0.0}
    yaw_deg: 0.0
  - name: point2
    position: {x: 1.35, y: 0.0}
    yaw_deg: 0.0
  - name: point3
    position: {x: 1.8, y: -0.45}
    yaw_deg: -45.0
  - name: point4
    position: {x: 1.8, y: -1.35}
    yaw_deg: -90.0
  - name: point5
    position: {x: 1.35, y: -1.8}
    yaw_deg: -135.0
  - name: point6
    position: {x: 0.45, y: -1.8}
    yaw_deg: 180.0
  - name: point7
    position: {x: 0.0, y: -1.35}
    yaw_deg: 135.0
  - name: point8_finish
    position: {x: 0.0, y: 0.0}
    yaw_deg: 0.0
```

### 주요 포인트
- 전방 예열(0.45 m)와 상단 복귀(0 m, -0.45 m)를 제거해 꼭 필요한 지점 8개만 남겼습니다.
- 반복 주행 시 마지막 포인트를 유지하면 한 바퀴 후 시작 자세로 복귀합니다. 여러 바퀴를 돌려면 `point8_finish` 를 제거하거나 로직에서 스킵하면 됩니다.

---

## 4. C++ 기반 구현 (ROS 2 Foxy/Humble 호환)

> Python 대신 C++로 액션 서버/클라이언트를 재작성한 예시입니다.  
> 코드 구조는 기존 Python 버전을 최대한 유지하면서 ROS 2 C++ 스타일로 옮긴 것입니다.

### 4.1 소스 파일

#### `src/patrol_server.cpp`
```cpp
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlebot3_msgs/action/patrol.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

struct Waypoint
{
  std::string name;
  double x;
  double y;
  double yaw;
};
}  // namespace

class Turtlebot3PatrolServer : public rclcpp::Node
{
public:
  using Patrol = turtlebot3_msgs::action::Patrol;
  using GoalHandlePatrol = rclcpp_action::ServerGoalHandle<Patrol>;

  Turtlebot3PatrolServer()
  : Node("turtlebot3_patrol_server_cpp")
  {
    declare_parameter<std::string>("waypoint_yaml", "config/waypoints.yaml");
    declare_parameter<double>("position_tolerance", 0.05);
    declare_parameter<double>("yaw_tolerance_deg", 5.0);
    declare_parameter<double>("linear_kp", 0.2);
    declare_parameter<double>("angular_kp", 1.0);
    declare_parameter<double>("max_linear_speed", 0.07);
    declare_parameter<double>("max_angular_speed", 0.4);
    declare_parameter<double>("heading_slowdown_gain", 0.85);
    declare_parameter<double>("waypoint_timeout", 35.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 20, std::bind(&Turtlebot3PatrolServer::odom_callback, this, std::placeholders::_1));

    load_waypoints();

    action_server_ = rclcpp_action::create_server<Patrol>(
      this,
      "turtlebot3",
      std::bind(&Turtlebot3PatrolServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Turtlebot3PatrolServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Turtlebot3PatrolServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Patrol::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePatrol> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Canceling patrol...");
    goal_handle->canceled(std::make_shared<Patrol::Result>());
    init_twist();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePatrol> goal_handle)
  {
    std::thread{std::bind(&Turtlebot3PatrolServer::execute, this, std::placeholders::_1), goal_handle}
      .detach();
  }

  void execute(const std::shared_ptr<GoalHandlePatrol> goal_handle)
  {
    auto feedback = std::make_shared<Patrol::Feedback>();
    auto result = std::make_shared<Patrol::Result>();
    const auto goal = goal_handle->get_goal();

    int shape_mode = static_cast<int>(goal->goal.x);
    double travel_distance = goal->goal.y;
    int iteration = static_cast<int>(goal->goal.z);
    if (iteration <= 0) {
      iteration = 1;
    }

    if (shape_mode == 3 && waypoints_.empty()) {
      feedback->state = "no waypoint data available";
      goal_handle->publish_feedback(feedback);
      result->result = feedback->state;
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing patrol mode %d for %d loop(s).", shape_mode, iteration);
    rclcpp::Rate loop_rate(20.0);

    bool keep_running = true;
    while (keep_running && rclcpp::ok()) {
      switch (shape_mode) {
        case 1:
          keep_running = false;
          for (int i = 0; i < iteration && rclcpp::ok(); ++i) {
            feedback->state = "square patrol loop " + std::to_string(i + 1);
            goal_handle->publish_feedback(feedback);
            run_square_pattern(travel_distance);
          }
          feedback->state = "square patrol complete!!";
          break;
        case 2:
          keep_running = false;
          for (int i = 0; i < iteration && rclcpp::ok(); ++i) {
            feedback->state = "triangle patrol loop " + std::to_string(i + 1);
            goal_handle->publish_feedback(feedback);
            run_triangle_pattern(travel_distance);
          }
          feedback->state = "triangle patrol complete!!";
          break;
        case 3:
          for (int loop_idx = 0; loop_idx < iteration && rclcpp::ok(); ++loop_idx) {
            for (std::size_t idx = 0; idx < waypoints_.size() && rclcpp::ok(); ++idx) {
              feedback->state = "loop " + std::to_string(loop_idx + 1) + "/" + std::to_string(iteration) +
                                " - waypoint " + std::to_string(idx + 1) + "/" +
                                std::to_string(waypoints_.size()) + " (" + waypoints_[idx].name + ")";
              goal_handle->publish_feedback(feedback);
              if (!drive_to_waypoint(waypoints_[idx])) {
                keep_running = false;
                break;
              }
            }
            if (!keep_running) {
              break;
            }
          }
          feedback->state = "custom waypoint patrol complete!!";
          keep_running = false;
          break;
        default:
          feedback->state = "unsupported mode";
          keep_running = false;
          break;
      }

      loop_rate.sleep();
    }

    init_twist();
    result->result = feedback->state;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Patrol complete.");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_ = *msg;
    if (!initial_pose_) {
      const auto & p = odom_.pose.pose.position;
      const auto & q = odom_.pose.pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      initial_pose_ = std::make_tuple(p.x, p.y, yaw);
      RCLCPP_INFO(get_logger(), "Initial pose recorded: x=%.3f y=%.3f yaw=%.1f°",
                  p.x, p.y, yaw * 180.0 / M_PI);
    }
  }

  void load_waypoints()
  {
    const auto yaml_path = get_parameter("waypoint_yaml").get_parameter_value().get<std::string>();
    try {
      YAML::Node root = YAML::LoadFile(yaml_path);
      if (!root["waypoints"]) {
        RCLCPP_WARN(get_logger(), "Waypoint file has no 'waypoints' key: %s", yaml_path.c_str());
        return;
      }
      for (const auto & entry : root["waypoints"]) {
        Waypoint wp;
        wp.name = entry["name"] ? entry["name"].as<std::string>() : "wp_" + std::to_string(waypoints_.size() + 1);
        wp.x = entry["position"]["x"].as<double>();
        wp.y = entry["position"]["y"].as<double>();
        const double yaw_deg = entry["yaw_deg"] ? entry["yaw_deg"].as<double>() : 0.0;
        wp.yaw = yaw_deg * M_PI / 180.0;
        waypoints_.push_back(wp);
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu waypoint(s) from %s", waypoints_.size(), yaml_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse waypoint YAML: %s", e.what());
    }
  }

  bool drive_to_waypoint(const Waypoint & waypoint)
  {
    if (!rclcpp::ok()) {
      return false;
    }

    while (!initial_pose_ && rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for initial pose...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!initial_pose_) {
      RCLCPP_WARN(get_logger(), "Initial pose unavailable. Abort waypoint (%s).", waypoint.name.c_str());
      return false;
    }

    const auto [init_x, init_y, init_yaw] = *initial_pose_;
    const double target_x = init_x + waypoint.x;
    const double target_y = init_y + waypoint.y;
    const double target_yaw = normalize_angle(init_yaw + waypoint.yaw);

    const double pos_tol = get_parameter("position_tolerance").as_double();
    const double yaw_tol = get_parameter("yaw_tolerance_deg").as_double() * M_PI / 180.0;
    const double lin_kp = get_parameter("linear_kp").as_double();
    const double ang_kp = get_parameter("angular_kp").as_double();
    const double max_lin = get_parameter("max_linear_speed").as_double();
    const double max_ang = get_parameter("max_angular_speed").as_double();
    const double heading_gain = get_parameter("heading_slowdown_gain").as_double();
    const double timeout = get_parameter("waypoint_timeout").as_double();

    RCLCPP_INFO(get_logger(), "Moving to '%s' target=(%.2f, %.2f, %.1f°)",
                waypoint.name.c_str(), target_x, target_y, target_yaw * 180.0 / M_PI);

    auto deadline = now() + rclcpp::Duration::from_seconds(timeout);
    bool reached_position = false;
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok()) {
      nav_msgs::msg::Odometry current;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current = odom_;
      }

      const auto & pose = current.pose.pose;
      const double current_yaw = std::atan2(
        2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

      const double dx = target_x - pose.position.x;
      const double dy = target_y - pose.position.y;
      const double distance = std::hypot(dx, dy);

      geometry_msgs::msg::Twist cmd;

      if (!reached_position) {
        if (distance < pos_tol) {
          reached_position = true;
          cmd_pub_->publish(cmd);
          rate.sleep();
          continue;
        }

        const double heading = std::atan2(dy, dx);
        const double heading_error = normalize_angle(heading - current_yaw);

        double linear = std::clamp(lin_kp * distance, 0.0, max_lin);
        double angular = std::clamp(ang_kp * heading_error, -max_ang, max_ang);

        const double heading_scale = std::max(0.1, 1.0 - heading_gain * std::abs(heading_error));
        linear *= heading_scale;

        cmd.linear.x = linear;
        cmd.angular.z = angular;
      } else {
        const double yaw_error = normalize_angle(target_yaw - current_yaw);
        if (std::abs(yaw_error) < yaw_tol) {
          cmd_pub_->publish(geometry_msgs::msg::Twist{});
          break;
        }
        cmd.angular.z = std::clamp(ang_kp * yaw_error, -max_ang, max_ang);
      }

      cmd_pub_->publish(cmd);

      if (now() > deadline) {
        RCLCPP_WARN(get_logger(), "Timed out on waypoint '%s'.", waypoint.name.c_str());
        init_twist();
        return false;
      }

      rate.sleep();
    }

    init_twist();
    return true;
  }

  void run_square_pattern(double length)
  {
    execute_straight(length, 0.2);
    execute_turn(90.0);
    execute_straight(length, 0.2);
    execute_turn(90.0);
    execute_straight(length, 0.2);
    execute_turn(90.0);
    execute_straight(length, 0.2);
    execute_turn(90.0);
  }

  void run_triangle_pattern(double length)
  {
    execute_straight(length, 0.2);
    execute_turn(120.0);
    execute_straight(length, 0.2);
    execute_turn(120.0);
    execute_straight(length, 0.2);
    execute_turn(120.0);
  }

  void execute_straight(double length, double speed)
  {
    const auto start_time = now();
    double distance = 0.0;
    rclcpp::Rate rate(10.0);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;

    while (distance < length && rclcpp::ok()) {
      cmd_pub_->publish(cmd);
      distance = (now() - start_time).seconds() * speed;
      rate.sleep();
    }
    init_twist();
  }

  void execute_turn(double target_angle_deg)
  {
    const double angular_speed = 0.4;
    const double target_angle_rad = target_angle_deg * M_PI / 180.0;
    double rotated = 0.0;
    rclcpp::Rate rate(20.0);
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = angular_speed;

    while (rotated < target_angle_rad && rclcpp::ok()) {
      cmd_pub_->publish(cmd);
      rotated += std::abs(cmd.angular.z) * (1.0 / 20.0);
      rate.sleep();
    }
    init_twist();
  }

  void init_twist()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  rclcpp_action::Server<Patrol>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::vector<Waypoint> waypoints_;
  nav_msgs::msg::Odometry odom_;
  std::optional<std::tuple<double, double, double>> initial_pose_;
  std::mutex odom_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Turtlebot3PatrolServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

#### `src/patrol_client.cpp`
```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlebot3_msgs/action/patrol.hpp>

class Turtlebot3PatrolClient : public rclcpp::Node
{
public:
  using Patrol = turtlebot3_msgs::action::Patrol;
  using GoalHandlePatrol = rclcpp_action::ClientGoalHandle<Patrol>;

  Turtlebot3PatrolClient()
  : Node("turtlebot3_patrol_client_cpp")
  {
    action_client_ = rclcpp_action::create_client<Patrol>(this, "turtlebot3");
  }

  void send_goal_interactive()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "Patrol action server not available.");
      return;
    }

    std::cout << "mode: s(square), t(triangle), c(custom waypoint), x(cancel)\n";
    std::cout << "mode(s, t, c, x): ";
    std::string mode;
    std::getline(std::cin, mode);

    if (mode.empty()) {
      RCLCPP_WARN(get_logger(), "Empty mode input.");
      return;
    }

    Patrol::Goal goal_msg;
    if (mode == "s") {
      goal_msg.goal.x = 1.0;
      std::cout << "travel_distance (m): ";
      std::cin >> goal_msg.goal.y;
      std::cout << "patrol_count: ";
      std::cin >> goal_msg.goal.z;
    } else if (mode == "t") {
      goal_msg.goal.x = 2.0;
      std::cout << "travel_distance (m): ";
      std::cin >> goal_msg.goal.y;
      std::cout << "patrol_count: ";
      std::cin >> goal_msg.goal.z;
    } else if (mode == "c") {
      goal_msg.goal.x = 3.0;
      goal_msg.goal.y = 0.0;
      std::cout << "patrol_count (default 1): ";
      std::string count_str;
      std::getline(std::cin >> std::ws, count_str);
      goal_msg.goal.z = count_str.empty() ? 1.0 : std::stod(count_str);
    } else if (mode == "x") {
      RCLCPP_INFO(get_logger(), "Client canceled by user.");
      return;
    } else {
      RCLCPP_WARN(get_logger(), "Unsupported mode: %s", mode.c_str());
      return;
    }

    auto send_goal_options = rclcpp_action::Client<Patrol>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandlePatrol::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal rejected.");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted.");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandlePatrol::SharedPtr,
             const std::shared_ptr<const Patrol::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Feedback: %s", feedback->state.c_str());
      };

    send_goal_options.result_callback =
      [this](const GoalHandlePatrol::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Result: %s", result.result->result.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal was canceled.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted: %s", result.result->result.c_str());
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
            break;
        }
        rclcpp::shutdown();
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Patrol>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Turtlebot3PatrolClient>();
  node->send_goal_interactive();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 4.2 `CMakeLists.txt` 추가 예시
```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(turtlebot3_patrol_server_cpp src/patrol_server.cpp)
ament_target_dependencies(turtlebot3_patrol_server_cpp
  rclcpp rclcpp_action geometry_msgs nav_msgs turtlebot3_msgs yaml-cpp)

add_executable(turtlebot3_patrol_client_cpp src/patrol_client.cpp)
ament_target_dependencies(turtlebot3_patrol_client_cpp
  rclcpp rclcpp_action turtlebot3_msgs)

install(TARGETS
  turtlebot3_patrol_server_cpp
  turtlebot3_patrol_client_cpp
  DESTINATION lib/${PROJECT_NAME})
```

### 4.3 `package.xml` 의존성 추가
```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>turtlebot3_msgs</depend>
<depend>yaml-cpp</depend>
```

### 4.4 실행 예시
```bash
colcon build --packages-select turtlebot3_example
. install/setup.bash
ros2 run turtlebot3_example turtlebot3_patrol_server_cpp
ros2 run turtlebot3_example turtlebot3_patrol_client_cpp
```

---

## 마무리 메모
- 상기 코드 열람 후 실제 구현에 들어갈 때는 Markdown 내용을 각 파일에 그대로 반영하면 됩니다.
- 벽과의 일정 거리 유지 제어(I2C 레이저)는 C++/Python 어느 쪽에서도 `drive_to_waypoint` 루프 내에서 거리 오차를 각속도 제어에 혼합하는 방식으로 추가할 수 있습니다.
