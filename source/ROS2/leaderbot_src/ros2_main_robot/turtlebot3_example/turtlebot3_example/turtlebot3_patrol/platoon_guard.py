#!/usr/bin/env python3
import time
from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class PlatoonGuard(Node):
    """팔로워 안전 가드: 리더가 제자리 회전/정지 시 팔로워를 일시정지.

    - 입력: 리더 /odom, /cmd_vel
    - 출력: 팔로워 /patrol/stop (Bool)
    - 기준: 창길이 내 변위 < dist_thresh 이고 (|vx| < vx_thresh 또는 |wz| > wz_spin_thresh)
    - 디바운스: hold/resume 각각 최소 지속 시간 유지
    """

    def __init__(self) -> None:
        super().__init__('platoon_guard')

        # Parameters
        self.declare_parameter('leader_odom_topic', '/tb3_1/odom')
        self.declare_parameter('leader_cmd_topic', '/tb3_1/cmd_vel')
        self.declare_parameter('follower_stop_topic', '/tb3_2/patrol/stop')
        self.declare_parameter('dist_window_sec', 0.7)
        self.declare_parameter('dist_thresh', 0.03)
        self.declare_parameter('vx_thresh', 0.03)
        self.declare_parameter('wz_spin_thresh', 0.25)
        self.declare_parameter('hold_min_sec', 0.5)
        self.declare_parameter('resume_min_sec', 0.5)

        self.leader_odom_topic = self.get_parameter('leader_odom_topic').get_parameter_value().string_value
        self.leader_cmd_topic = self.get_parameter('leader_cmd_topic').get_parameter_value().string_value
        self.follower_stop_topic = self.get_parameter('follower_stop_topic').get_parameter_value().string_value
        self.dist_window_sec = float(self.get_parameter('dist_window_sec').value)
        self.dist_thresh = float(self.get_parameter('dist_thresh').value)
        self.vx_thresh = float(self.get_parameter('vx_thresh').value)
        self.wz_spin_thresh = float(self.get_parameter('wz_spin_thresh').value)
        self.hold_min_sec = float(self.get_parameter('hold_min_sec').value)
        self.resume_min_sec = float(self.get_parameter('resume_min_sec').value)

        qos = QoSProfile(depth=10)
        self.stop_pub = self.create_publisher(Bool, self.follower_stop_topic, qos)
        self.create_subscription(Odometry, self.leader_odom_topic, self._on_odom, qos)
        self.create_subscription(Twist, self.leader_cmd_topic, self._on_cmd, qos)

        self._trace: Deque[Tuple[float, float, float]] = deque()  # (t, x, y)
        self._last_cmd: Tuple[float, float] = (0.0, 0.0)  # (vx, wz)
        self._state_hold = None  # None/True/False
        self._state_changed_at = time.monotonic()

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'PlatoonGuard watching odom={self.leader_odom_topic}, cmd={self.leader_cmd_topic} -> stop={self.follower_stop_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = time.monotonic()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._trace.append((t, x, y))
        # drop old
        cutoff = t - self.dist_window_sec
        while self._trace and self._trace[0][0] < cutoff:
            self._trace.popleft()

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = (float(msg.linear.x), float(msg.angular.z))

    def _tick(self) -> None:
        if not self._trace:
            return
        vx, wz = self._last_cmd
        t_now = time.monotonic()
        t0, x0, y0 = self._trace[0]
        t1, x1, y1 = self._trace[-1]
        # 변위
        dx = x1 - x0
        dy = y1 - y0
        disp = (dx * dx + dy * dy) ** 0.5

        # 기존: 정지(idle) 또는 회전(spinning)이며 창 내 변위가 작을 때 hold
        idle = (abs(vx) < self.vx_thresh) and (abs(wz) < self.wz_spin_thresh)
        spinning = abs(wz) > self.wz_spin_thresh
        should_hold = (disp < self.dist_thresh) and (idle or spinning)

        # 디바운스/토글
        if self._state_hold is None:
            self._set_state(should_hold)
            return
        if should_hold and not self._state_hold:
            if t_now - self._state_changed_at >= self.resume_min_sec:
                self._set_state(True)
        elif (not should_hold) and self._state_hold:
            if t_now - self._state_changed_at >= self.hold_min_sec:
                self._set_state(False)

    def _set_state(self, hold: bool) -> None:
        self._state_hold = hold
        self._state_changed_at = time.monotonic()
        msg = Bool(); msg.data = hold
        self.stop_pub.publish(msg)
        self.get_logger().info(f'[PlatoonGuard] follower stop={hold} (publish: {self.follower_stop_topic})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatoonGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
