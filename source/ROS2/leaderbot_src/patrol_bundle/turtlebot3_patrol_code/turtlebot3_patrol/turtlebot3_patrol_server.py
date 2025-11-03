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

        default_yaml = Path(
            get_package_share_directory('turtlebot3_example')
        ) / 'turtlebot3_patrol' / 'config' / 'waypoints.yaml'

        self.declare_parameter('waypoint_yaml', str(default_yaml))
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('linear_kp', 0.2)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('heading_slowdown_gain', 0.85)
        self.declare_parameter('waypoint_timeout', 35.0)

        waypoint_path = Path(
            self.get_parameter('waypoint_yaml').get_parameter_value().string_value
        ).expanduser()
        self.waypoints: List[Waypoint] = self.load_waypoints(waypoint_path)
        if not self.waypoints:
            self.get_logger().warn(
                f'No waypoints loaded from {waypoint_path}. Custom patrol will be disabled.'
            )

        self.initial_pose: Optional[tuple[float, float, float]] = None

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
                f'Loaded {len(result)} waypoint(s) from {yaml_path}'
            )
        return result

    def drive_to_waypoint(self, waypoint: Waypoint):
        if not rclpy.ok():
            return
        if self.initial_pose is None:
            self.get_logger().info('Waiting for initial pose before starting waypoint drive...')
            while rclpy.ok() and self.initial_pose is None:
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
            f"Moving to '{waypoint.name}' "
            f"(target: x={target_x:.2f}, y={target_y:.2f}, yaw={math.degrees(target_yaw):.1f}°)"
        )

        while rclpy.ok():
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

                self.twist.linear.x = linear
                self.twist.angular.z = angular
            else:
                yaw_error = normalize_angle(target_yaw - current_yaw)
                if abs(yaw_error) < yaw_tol:
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

    def go_front(self, position, length):
        while True:
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

        while True:
            if self.goal_msg.goal.x == 1:
                for count in range(iteration):
                    self.square(feedback_msg, goal_handle, length)
                feedback_msg.state = 'square patrol complete!!'
                break
            elif self.goal_msg.goal.x == 2:
                for count in range(iteration):
                    self.triangle(feedback_msg, goal_handle, length)
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
                        feedback_msg.state = (
                            f"loop {loop_idx + 1}/{loops} - waypoint {idx + 1}/{len(self.waypoints)} "
                            f"({waypoint.name})"
                        )
                        goal_handle.publish_feedback(feedback_msg)
                        self.drive_to_waypoint(waypoint)
                feedback_msg.state = 'custom waypoint patrol complete!!'
                break

        goal_handle.succeed()
        result = Patrol.Result()
        result.result = feedback_msg.state

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


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_server = Turtlebot3PatrolServer()

    rclpy.spin(turtlebot3_patrol_server)


if __name__ == '__main__':
    main()
