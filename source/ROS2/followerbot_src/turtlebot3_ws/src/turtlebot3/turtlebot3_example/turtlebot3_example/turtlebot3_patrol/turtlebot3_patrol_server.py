#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

import math
import threading
import time
from pathlib import Path
from typing import List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

from turtlebot3_msgs.action import Patrol


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Waypoint:
    def __init__(self, name: str, x: float, y: float, yaw: float) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        print('TurtleBot3 Patrol Server')
        print('----------------------------------------------')

        self._action_server = ActionServer(
            self,
            Patrol,
            'turtlebot3',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback)

        self.goal_msg = Patrol.Goal()
        self.twist = Twist()
        self.odom = Odometry()
        self.position = Point()
        self.rotation = 0.0

        self.linear_x = 1.0
        self.angular_z = 4.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )
        self.stop_sub = self.create_subscription(
            Bool, 'patrol/stop', self._on_stop_signal, qos
        )

        # stop 자동해제/쿨다운 기능 없음(원래 동작으로 복원)

        # sup_order 제어: follower(tb3_2)만 활성화
        self._sup_order_enabled = (self.get_namespace() == '/tb3_2')
        self._sup_last_order: Optional[bool] = None
        if self._sup_order_enabled:
            self._sup_stop_pub = self.create_publisher(Bool, 'patrol/stop', 10)
            self.create_subscription(Bool, 'sup_order', self._on_sup_order, qos)
            self.get_logger().info("[SupOrder] 활성화: 'sup_order' 토픽으로 순찰 일시정지 제어")

        # 팔로워 내장 가드(별도 노드 없이 사용): 리더가 제자리 회전/정지 시 잠시 일시정지
        # 기본: /tb3_2 네임스페이스에서는 자동 활성화, 그 외 비활성화
        self.declare_parameter('guard_enable', None)  # None이면 NS 기준 자동
        self.declare_parameter('guard_leader_odom', '/tb3_1/odom')
        self.declare_parameter('guard_leader_cmd', '/tb3_1/cmd_vel')
        self.declare_parameter('guard_window_sec', 0.7)
        self.declare_parameter('guard_dist_thresh', 0.03)
        self.declare_parameter('guard_vx_thresh', 0.03)
        self.declare_parameter('guard_wz_spin_thresh', 0.25)
        self.declare_parameter('guard_hold_min_sec', 0.5)
        self.declare_parameter('guard_resume_min_sec', 0.5)

        # NS 자동 판단
        _ns = self.get_namespace()
        #_auto_enable = (_ns == '/tb3_2')
        _auto_enable = (_ns == '')  # IGNORE
        _param_enable = self.get_parameter('guard_enable').get_parameter_value().bool_value if self.get_parameter('guard_enable').type_ != rclpy.parameter.Parameter.Type.NOT_SET else None
        self._guard_enable = (_param_enable if _param_enable is not None else _auto_enable)

        if self._guard_enable:
            self._guard_odom_topic = self.get_parameter('guard_leader_odom').get_parameter_value().string_value
            self._guard_cmd_topic = self.get_parameter('guard_leader_cmd').get_parameter_value().string_value
            self._guard_window_sec = float(self.get_parameter('guard_window_sec').value)
            self._guard_dist_thresh = float(self.get_parameter('guard_dist_thresh').value)
            self._guard_vx_thresh = float(self.get_parameter('guard_vx_thresh').value)
            self._guard_wz_spin_thresh = float(self.get_parameter('guard_wz_spin_thresh').value)
            self._guard_hold_min_sec = float(self.get_parameter('guard_hold_min_sec').value)
            self._guard_resume_min_sec = float(self.get_parameter('guard_resume_min_sec').value)

            # 내부 퍼블리셔(자기 자신 /patrol/stop으로 송신해 콜백을 재사용)
            self._guard_stop_pub = self.create_publisher(Bool, 'patrol/stop', 10)

            # 입력 구독
            self._guard_trace = []  # list[(t,x,y)]
            self._guard_last_cmd = (0.0, 0.0)
            self._guard_state = None  # None/True/False
            self._guard_changed_at = time.monotonic()
            self.create_subscription(Odometry, self._guard_odom_topic, self._guard_on_odom, qos)
            self.create_subscription(Twist, self._guard_cmd_topic, self._guard_on_cmd, qos)
            self.create_timer(0.05, self._guard_tick)
            self.get_logger().info(f"[Guard] 활성: odom={self._guard_odom_topic}, cmd={self._guard_cmd_topic} → patrol/stop")
        

        # 팔로워 속도 부스트(간단한 가속 창) 기능
        # 기본: /tb3_2 네임스페이스에서 자동 활성화. 지정 인덱스 웨이포인트 통과 후 N초간 선속도에 배수 적용
        self.declare_parameter('boost_enable', None)  # None이면 NS 기준 자동
        self.declare_parameter('boost_waypoint_indices', [3, 5, 7])
        self.declare_parameter('boost_scale', 1.0)  # multiply
        self.declare_parameter('boost_duration', 2.0) # seconds

        _param_boost_enable = self.get_parameter('boost_enable').get_parameter_value().bool_value if self.get_parameter('boost_enable').type_ != rclpy.parameter.Parameter.Type.NOT_SET else None
        self._boost_enable = (_param_boost_enable if _param_boost_enable is not None else _auto_enable)
        _boost_indices_param = self.get_parameter('boost_waypoint_indices').get_parameter_value().integer_array_value
        self._boost_indices = set(int(i) for i in list(_boost_indices_param))
        self._boost_scale = float(self.get_parameter('boost_scale').value)
        self._boost_duration = float(self.get_parameter('boost_duration').value)
        self._boost_until = 0.0  # epoch sec
        if self._boost_enable:
            self.get_logger().info(
                f"[Boost] 활성: idx={sorted(self._boost_indices)}, scale={self._boost_scale}, dur={self._boost_duration}s"
            )

        default_yaml = Path(
            get_package_share_directory('turtlebot3_example')
        ) / 'turtlebot3_patrol' / 'config' / 'waypoints.yaml'

        print("MY NAMESPACE: " + self.get_namespace())

        if(self.get_namespace() == '/tb3_2'):
            default_yaml = Path(
                get_package_share_directory('turtlebot3_example')
            ) / 'turtlebot3_patrol' / 'config' / 'waypoints_follower.yaml'


        self.declare_parameter('waypoint_yaml', str(default_yaml))
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('linear_kp', 0.2)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('heading_slowdown_gain', 0.85)
        self.declare_parameter('waypoint_timeout', 35.0)
        self.declare_parameter('final_position_tolerance', 0.03)
        self.declare_parameter('final_yaw_tolerance_deg', 2.0)
        self.declare_parameter('final_timeout', 10.0)

        waypoint_path = Path(
            self.get_parameter('waypoint_yaml').get_parameter_value().string_value
        ).expanduser()
        self.waypoints: List[Waypoint] = self.load_waypoints(waypoint_path)
        if not self.waypoints:
            self.get_logger().warn(
                f'No waypoints loaded from {waypoint_path}. Custom patrol will be disabled.'
            )

        self.initial_pose: Optional[tuple[float, float, float]] = None
        self.stop_requested = False
        self.get_logger().info('순찰 서버 초기화 완료. stop 신호는 /patrol/stop 토픽에서 수신합니다.')

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg):
        self.odom = msg
        if self.initial_pose is None:
            yaw = self.get_yaw()
            self.initial_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw
            )
            self.get_logger().info(
                f'Initial pose recorded at x={self.initial_pose[0]:.3f}, '
                f'y={self.initial_pose[1]:.3f}, yaw={math.degrees(self.initial_pose[2]):.1f}°'
            )

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def load_waypoints(self, yaml_path: Path) -> List[Waypoint]:
        if not yaml_path.exists():
            self.get_logger().error(f'Waypoint file not found: {yaml_path}')
            return []
        try:
            data = yaml.safe_load(yaml_path.read_text())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to parse waypoint YAML: {exc}')
            return []

        result: List[Waypoint] = []
        for idx, entry in enumerate(data.get('waypoints', [])):
            try:
                name = str(entry.get('name', f'waypoint_{idx + 1}'))
                x = float(entry['position']['x'])
                y = float(entry['position']['y'])
                yaw_deg = float(entry.get('yaw_deg', 0.0))
                result.append(Waypoint(name, x, y, math.radians(yaw_deg)))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Skipping waypoint index {idx}: {exc}')
        if result:
            self.get_logger().info(
                f"[순찰] 웨이포인트 {len(result)}개 로드 완료 ({yaml_path})"
            )
        return result

    def drive_to_waypoint(self, waypoint: Waypoint, waypoint_index: Optional[int] = None):
        if not rclpy.ok() or self.stop_requested:
            return
        if self.initial_pose is None:
            self.get_logger().info('Waiting for initial pose before starting waypoint drive...')
            while rclpy.ok() and self.initial_pose is None:
                if self.stop_requested:
                    self.get_logger().warn('순찰 중지 요청으로 웨이포인트 이동을 취소합니다.')
                    return
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.initial_pose is None:
                self.get_logger().warn('Initial pose not available. Aborting waypoint drive.')
                return

        target_x = self.initial_pose[0] + waypoint.x
        target_y = self.initial_pose[1] + waypoint.y
        target_yaw = normalize_angle(self.initial_pose[2] + waypoint.yaw)

        pos_tol = float(self.get_parameter('position_tolerance').value)
        yaw_tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        lin_kp = float(self.get_parameter('linear_kp').value)
        ang_kp = float(self.get_parameter('angular_kp').value)
        max_lin = float(self.get_parameter('max_linear_speed').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)
        heading_gain = float(self.get_parameter('heading_slowdown_gain').value)
        timeout = float(self.get_parameter('waypoint_timeout').value)

        deadline = time.time() + timeout
        reached_position = False

        self.get_logger().info(
            f"[순찰] '{waypoint.name}' 지점으로 이동 시작 "
            f"(목표 x={target_x:.2f}, y={target_y:.2f}, yaw={math.degrees(target_yaw):.1f}°)"
        )

        while rclpy.ok():
            if self.stop_requested:
                if not self._wait_for_resume(f"웨이포인트 '{waypoint.name}' 이동"):
                    return
                deadline = time.time() + timeout
                continue
            rclpy.spin_once(self, timeout_sec=0.05)

            current_pose = self.odom.pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            current_yaw = self.get_yaw()

            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)

            if not reached_position:
                if distance < pos_tol:
                    self.get_logger().info(
                        f"[순찰] '{waypoint.name}' 위치 도달. 자세 정렬 단계로 전환합니다."
                    )
                    reached_position = True
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                    continue

                heading = math.atan2(dy, dx)
                heading_error = normalize_angle(heading - current_yaw)

                linear = max(0.0, min(max_lin, lin_kp * distance))
                angular = max(-max_ang, min(max_ang, ang_kp * heading_error))

                heading_scale = max(0.1, 1.0 - heading_gain * abs(heading_error))
                linear *= heading_scale

                # 부스트 창이 켜져 있으면 선속도만 배수 적용
                if self._boost_enable and (time.time() < self._boost_until):
                    linear = min(linear * self._boost_scale, max_lin * self._boost_scale)

                self.twist.linear.x = linear
                self.twist.angular.z = angular
            else:
                yaw_error = normalize_angle(target_yaw - current_yaw)
                if abs(yaw_error) < yaw_tol:
                    self.get_logger().info(
                        f"[순찰] '{waypoint.name}' yaw 정렬 완료."
                    )
                    break
                angular = max(-max_ang, min(max_ang, ang_kp * yaw_error))
                self.twist.linear.x = 0.0
                self.twist.angular.z = angular

            self.cmd_vel_pub.publish(self.twist)

            if time.time() > deadline:
                self.get_logger().warn(
                    f"Timed out while driving to waypoint '{waypoint.name}'."
                )
                break

            time.sleep(0.05)

        self.init_twist()

        

        # 웨이포인트를 정상적으로 정렬 완료하고 나왔으면(타임아웃이 아닌 경우) 해당 인덱스에 따라 부스트 창 오픈
        if self._boost_enable and waypoint_index is not None:
            # 도달/정렬 완료 조건에서만 루프 탈출했을 때만 부스트 트리거되도록, yaw 정렬 완료 로그가 있었는지 힌트를 사용
            # 엄격한 플래그는 두지 않고, 타임아웃/중단 시에는 상단에서 return/break 되어 여기로 오더라도 부스터는 무시될 수 있도록 인덱스로만 제어
            if (waypoint_index in self._boost_indices):
                self._boost_until = time.time() + self._boost_duration
                self.get_logger().info(
                    f"[Boost] WP#{waypoint_index} 통과 → {self._boost_duration:.1f}s 동안 선속도 x{self._boost_scale}")

    def go_front(self, position, length):
        while True:
            if self.stop_requested:
                if not self._wait_for_resume('go_front 전진'):
                    break
                continue
            position += self.twist.linear.x
            if position >= length:
                break
            self.twist.linear.x = self.linear_x
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            time.sleep(1)
        self.init_twist()

    def turn(self, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.stop_requested:
                if not self._wait_for_resume('turn 회전'):
                    break
                continue

            current_yaw = self.get_yaw()
            yaw_diff = abs(
                math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw)
                )
            )

            if yaw_diff < 0.01:
                break

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_z
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Patrol.Feedback()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)
        if iteration <= 0:
            iteration = 1

        self.stop_requested = False
        self.get_logger().info(
            f"[순찰] 목표 실행 시작 (모드={self.goal_msg.goal.x}, 반복={iteration})"
        )

        while True:
            if self.stop_requested:
                feedback_msg.state = 'stop 신호 수신. 재개 신호 대기 중...'
                goal_handle.publish_feedback(feedback_msg)
                if not self._wait_for_resume('순찰 실행 루프'):
                    feedback_msg.state = 'stop 신호로 순찰 중단'
                    self.get_logger().warn('Stop 신호 수신: 순찰을 종료합니다.')
                    break
                feedback_msg.state = '순찰 재개'
                goal_handle.publish_feedback(feedback_msg)
                continue
            if self.goal_msg.goal.x == 1:
                for count in range(iteration):
                    self.square(feedback_msg, goal_handle, length)
                    if self.stop_requested:
                        break
                feedback_msg.state = 'square patrol complete!!'
                break
            elif self.goal_msg.goal.x == 2:
                for count in range(iteration):
                    self.triangle(feedback_msg, goal_handle, length)
                    if self.stop_requested:
                        break
                feedback_msg.state = 'triangle patrol complete!!'
                break
            elif self.goal_msg.goal.x == 3:
                if not self.waypoints:
                    feedback_msg.state = 'no waypoint data available'
                    self.get_logger().error('Custom waypoint list is empty.')
                    break
                loops = iteration
                for loop_idx in range(loops):
                    for idx, waypoint in enumerate(self.waypoints):
                        if self.stop_requested:
                            break
                        feedback_msg.state = (
                            f"loop {loop_idx + 1}/{loops} - waypoint {idx + 1}/{len(self.waypoints)} "
                            f"({waypoint.name})"
                        )
                        goal_handle.publish_feedback(feedback_msg)
                        # 사용자 관점의 번호는 1-base로 통일
                        self.drive_to_waypoint(waypoint, waypoint_index=idx + 1)
                    if self.stop_requested:
                        break
                feedback_msg.state = 'custom waypoint patrol complete!!'
                break

        goal_handle.succeed()
        result = Patrol.Result()
        result.result = feedback_msg.state

        self._final_align_to_start()

        self.init_twist()
        self.get_logger().info('Patrol complete.')
        threading.Timer(0.1, rclpy.shutdown).start()

        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 13 * (90.0 / 180.0) * math.pi / 100.0

        for i in range(4):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(90.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self.init_twist()

    def _final_align_to_start(self) -> None:
        if not rclpy.ok() or self.initial_pose is None:
            return

        if self.stop_requested:
            self.get_logger().warn('Stop 상태이므로 최종 정렬 단계를 건너뜁니다.')
            return

        target_x, target_y, target_yaw = self.initial_pose
        pos_tol = float(self.get_parameter('final_position_tolerance').value)
        yaw_tol = math.radians(float(self.get_parameter('final_yaw_tolerance_deg').value))
        timeout = float(self.get_parameter('final_timeout').value)

        lin_kp = float(self.get_parameter('linear_kp').value)
        ang_kp = float(self.get_parameter('angular_kp').value)
        max_lin = float(self.get_parameter('max_linear_speed').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)
        heading_gain = float(self.get_parameter('heading_slowdown_gain').value)

        deadline = time.time() + timeout
        self.get_logger().info('[순찰] 시작 위치/자세로 최종 정렬을 수행합니다...')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            current_pose = self.odom.pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            current_yaw = self.get_yaw()

            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)
            yaw_error = normalize_angle(target_yaw - current_yaw)

            if distance < pos_tol and abs(yaw_error) < yaw_tol:
                self.get_logger().info('[순찰] 최종 정렬 완료.')
                break

            heading = math.atan2(dy, dx)
            heading_error = normalize_angle(heading - current_yaw)

            linear = max(0.0, min(max_lin, lin_kp * distance))
            angular = max(-max_ang, min(max_ang, ang_kp * heading_error))

            heading_scale = max(0.1, 1.0 - heading_gain * abs(heading_error))
            linear *= heading_scale

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)

            if time.time() > deadline:
                self.get_logger().warn('Final alignment timeout reached.')
                break

        self.init_twist()

    def _on_stop_signal(self, msg: Bool) -> None:
        self.stop_requested = msg.data
        if msg.data:
            self.get_logger().warn('Stop 신호 수신: 순찰을 일시 정지합니다.')
            self.init_twist()
        else:
            self.get_logger().info('Stop 해제 신호 수신: 순찰을 다시 진행합니다.')

    # ---------- 내장 가드 로직 ----------
    def _guard_on_odom(self, msg: Odometry) -> None:
        if not self._guard_enable:
            return
        t = time.monotonic()
        x = float(msg.pose.pose.position.x); y = float(msg.pose.pose.position.y)
        self._guard_trace.append((t, x, y))
        cutoff = t - self._guard_window_sec
        while self._guard_trace and self._guard_trace[0][0] < cutoff:
            self._guard_trace.pop(0)

    def _guard_on_cmd(self, msg: Twist) -> None:
        if not self._guard_enable:
            return
        self._guard_last_cmd = (float(msg.linear.x), float(msg.angular.z))

    def _guard_tick(self) -> None:
        if not self._guard_enable or not self._guard_trace:
            return
        vx, wz = self._guard_last_cmd
        t0, x0, y0 = self._guard_trace[0]
        t1, x1, y1 = self._guard_trace[-1]
        disp = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        # 더 보수적으로: "정말 정지(idle)"이거나 "명확한 제자리 회전(spinning)"일 때만 hold
        idle = (abs(vx) < self._guard_vx_thresh) and (abs(wz) < self._guard_wz_spin_thresh)
        spinning = abs(wz) > self._guard_wz_spin_thresh
        should_hold = (disp < self._guard_dist_thresh) and (idle or spinning)

        now = time.monotonic()
        if self._guard_state is None:
            self._guard_set(should_hold); return
        if should_hold and not self._guard_state and now - self._guard_changed_at >= self._guard_resume_min_sec:
            self._guard_set(True)
        elif (not should_hold) and self._guard_state and now - self._guard_changed_at >= self._guard_hold_min_sec:
            self._guard_set(False)

    def _guard_set(self, hold: bool) -> None:
        self._guard_state = hold
        self._guard_changed_at = time.monotonic()
        msg = Bool(); msg.data = hold
        # 자기 자신 /patrol/stop으로 발행 → 기존 정지 로직 재사용
        self._guard_stop_pub.publish(msg)
        self.get_logger().info(f"[Guard] follower stop={hold}")

    def _wait_for_resume(self, context: str) -> bool:
        self.init_twist()
        self.get_logger().warn(f'[{context}] 정지 상태입니다. 재개 신호를 기다립니다...')
        while rclpy.ok() and self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        return rclpy.ok()

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 8 * (120.0 / 180.0) * math.pi / 100.0

        for i in range(3):
            self.position.x = 0.0
            self.angle = 0.0

            self.go_front(self.position.x, length)
            self.turn(120.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.init_twist()

    def _on_stop_signal(self, msg: Bool) -> None:
        self.stop_requested = msg.data
        if msg.data:
            self.get_logger().warn('Stop 신호 수신: 순찰을 일시 정지합니다.')
            self.init_twist()
        else:
            self.get_logger().info('Stop 해제 신호 수신: 순찰을 다시 진행합니다.')

    def _on_sup_order(self, msg: Bool) -> None:
        if not self._sup_order_enabled:
            return
        allow_motion = bool(msg.data)
        if self._sup_last_order is not None and allow_motion == self._sup_last_order:
            return
        self._sup_last_order = allow_motion
        out = Bool()
        out.data = not allow_motion
        if allow_motion:
            self.get_logger().info("[SupOrder] TRUE → 순찰 재개 요청 전송")
        else:
            self.get_logger().warn("[SupOrder] FALSE → 순찰 일시 정지 요청 전송")
        self._sup_stop_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_server = Turtlebot3PatrolServer()

    rclpy.spin(turtlebot3_patrol_server)


if __name__ == '__main__':
    main()
