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
from robot_arm.srv import RobotArmCommand
from std_msgs.msg import String as StringMsg
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

        # 공유 서비스(서브로봇)에 웨이포인트 계획을 전달하기 위한 클라이언트
        # 기본값: 비활성화(False). 필요 시 실행 시 파라미터로 활성화/대상 서비스명 설정
        self.declare_parameter('share_enabled', False)
        self.declare_parameter('share_service_name', '/tb3_2/patrol/share_plan')
        self.declare_parameter('share_progress_service_name', '/tb3_2/patrol/share_progress')
        # 토픽 기반 공유(서브로봇이 주행하지 않고 정보만 받는 경우 간편)
        self.declare_parameter('share_topic_enabled', True)
        self.declare_parameter('share_plan_topic', '/tb3_2/patrol/plan')
        self.declare_parameter('share_progress_topic', '/tb3_2/patrol/current_waypoint')
        self._share_enabled = bool(self.get_parameter('share_enabled').value)
        self._share_service_name = str(self.get_parameter('share_service_name').value)
        self._share_prog_service_name = str(self.get_parameter('share_progress_service_name').value)
        self._share_topic_enabled = bool(self.get_parameter('share_topic_enabled').value)
        self._share_plan_topic = str(self.get_parameter('share_plan_topic').value)
        self._share_progress_topic = str(self.get_parameter('share_progress_topic').value)
        if self._share_enabled:
            self._share_client = self.create_client(RobotArmCommand, self._share_service_name)
            self.get_logger().info(f"[클라이언트] 웨이포인트 공유 서비스 연결 대기: {self._share_service_name}")
            self._share_client.wait_for_service()
            self._share_prog_client = self.create_client(RobotArmCommand, self._share_prog_service_name)
            self.get_logger().info(f"[클라이언트] 진행 공유 서비스 연결 대기: {self._share_prog_service_name}")
            self._share_prog_client.wait_for_service()
        # 토픽 퍼블리셔 준비(서브가 토픽만 구독해도 되도록)
        if self._share_topic_enabled:
            self._plan_pub = self.create_publisher(StringMsg, self._share_plan_topic, 10)
            self._progress_pub = self.create_publisher(StringMsg, self._share_progress_topic, 10)

        self.mode = 1.0
        self.travel_distance = 1.0
        self.patrol_count = 1

        self.mode, self.travel_distance, self.patrol_count = self.get_key()
        if rclpy.ok() and self.mode != 0:
            # 선택: 서브로봇에 현재 계획 공유
            if self._share_enabled:
                try:
                    payload = f"plan:mode={int(self.mode)},dist={float(self.travel_distance):.3f},count={int(self.patrol_count)}"
                    req = RobotArmCommand.Request()
                    req.command = payload
                    fut = self._share_client.call_async(req)
                    fut.add_done_callback(lambda f: self.get_logger().info(
                        f"[클라이언트] 공유 응답: {getattr(f.result(), 'result', '')}"))
                except Exception as e:
                    self.get_logger().error(f"[클라이언트] 공유 전송 실패: {e}")
            # 토픽으로도 계획 방송
            if self._share_topic_enabled:
                msg = StringMsg()
                msg.data = f"plan:mode={int(self.mode)},dist={float(self.travel_distance):.3f},count={int(self.patrol_count)}"
                self._plan_pub.publish(msg)
                self.get_logger().info(f"[클라이언트] 계획 토픽 송신: {msg.data} -> {self._share_plan_topic}")
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
        # 진행 상황을 서브로봇에 전달(옵션)
        if getattr(self, '_share_enabled', False):
            try:
                req = RobotArmCommand.Request()
                req.command = f"progress:{feedback.state}"
                self._share_prog_client.call_async(req)
            except Exception as e:
                self.get_logger().warn(f"[클라이언트] 진행 공유 실패: {e}")
        if getattr(self, '_share_topic_enabled', False):
            try:
                msg = StringMsg(); msg.data = f"progress:{feedback.state}"
                self._progress_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"[클라이언트] 진행 토픽 송신 실패: {e}")


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
