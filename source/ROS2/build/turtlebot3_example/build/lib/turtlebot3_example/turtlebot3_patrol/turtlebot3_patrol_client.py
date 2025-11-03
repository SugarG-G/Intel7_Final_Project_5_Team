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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolClient(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_client')

        print('TurtleBot3 Patrol Client')
        print('----------------------------------------------')
        print('Input below / 입력 안내')
        print('mode: s(사각형), t(삼각형), c(웨이포인트 순환)')
        print('travel_distance (unit: m)  *사각형/삼각형에서만 사용')
        print('patrol_count (반복 횟수)')
        print('----------------------------------------------')

        self._action_client = ActionClient(self, Patrol, 'turtlebot3')

        self.mode = 1.0
        self.travel_distance = 1.0
        self.patrol_count = 1

        self.mode, self.travel_distance, self.patrol_count = self.get_key()
        if rclpy.ok() and self.mode != 0:
            self.send_goal()

    def get_key(self):
        mode_input = str(input('mode(s, t, c): ')).strip().lower()

        if mode_input == 's':
            travel_distance = float(input('travel_distance: '))
            patrol_count = int(input('patrol_count: '))
            self.get_logger().info(f"[클라이언트] 사각형 모드 선택 (거리={travel_distance}m, 반복={patrol_count})")
            return 1, travel_distance, patrol_count

        if mode_input == 't':
            travel_distance = float(input('travel_distance: '))
            patrol_count = int(input('patrol_count: '))
            self.get_logger().info(f"[클라이언트] 삼각형 모드 선택 (거리={travel_distance}m, 반복={patrol_count})")
            return 2, travel_distance, patrol_count

        if mode_input == 'c':
            patrol_count_str = input('patrol_count (기본값 1): ').strip()
            patrol_count = int(patrol_count_str) if patrol_count_str else 1
            self.get_logger().info(f"[클라이언트] 커스텀 웨이포인트 모드 선택 (반복={patrol_count})")
            return 3, 0.0, patrol_count

        if mode_input == 'x':
            rclpy.shutdown()
            return 0, 0.0, 0

        self.get_logger().warn('[클라이언트] 지원하지 않는 모드가 입력되었습니다.')
        rclpy.shutdown()
        return 0, 0.0, 0

    def send_goal(self):
        goal_msg = Patrol.Goal()
        goal_msg.goal.x = float(self.mode)
        goal_msg.goal.y = float(self.travel_distance)
        goal_msg.goal.z = float(self.patrol_count)

        self._action_client.wait_for_server()

        self._send_goal_future = \
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[클라이언트] 목표가 거절되었습니다.')
            return

        self.get_logger().info('[클라이언트] 목표가 수락되었습니다. 순찰 시작을 기다립니다.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[클라이언트] 순찰 종료 결과: {result.result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[클라이언트] 진행 상황: {feedback.state}')


def main(args=None):
    rclpy.init(args=args)

    turtlebot3_patrol_client = Turtlebot3PatrolClient()

    if not rclpy.ok() or getattr(turtlebot3_patrol_client, 'mode', 0) == 0:
        turtlebot3_patrol_client.destroy_node()
        return

    try:
        rclpy.spin(turtlebot3_patrol_client)
    finally:
        if rclpy.ok():
            turtlebot3_patrol_client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
