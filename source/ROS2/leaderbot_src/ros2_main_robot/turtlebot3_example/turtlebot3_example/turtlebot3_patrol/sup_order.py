#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import Range


class SupOrder(Node):
    """Rear-range-based follower Stop/Go commander.

    - Subscribes: rear range (`rear_topic`)
    - Publishes: follower stop Bool (`follower_stop_topic`)
    - Policy:
        * distance < near_stop_thresh → Stop (True)
        * no data OR distance >= far_go_thresh → Go (False)
        * otherwise (in band) → Go (False)
    """

    def __init__(self) -> None:
        super().__init__('sup_order')

        # Parameters
        self.declare_parameter('rear_topic', '/vl53_rear_range')
        # follower 제어 토픽 (True=Go/Resume, False=Stop/Pause)
        self.declare_parameter('follower_sup_topic', '/tb3_2/sup_order')
        self.declare_parameter('near_stop_thresh', 0.07)
        self.declare_parameter('far_go_thresh', 0.09)
        self.declare_parameter('no_data_timeout', 0.7)
        self.declare_parameter('command_cooldown', 0.2)

        self.rear_topic = str(self.get_parameter('rear_topic').value)
        self.follower_sup_topic = str(self.get_parameter('follower_sup_topic').value)
        self.near_stop_thresh = float(self.get_parameter('near_stop_thresh').value)
        self.far_go_thresh = float(self.get_parameter('far_go_thresh').value)
        self.no_data_timeout = float(self.get_parameter('no_data_timeout').value)
        self.command_cooldown = float(self.get_parameter('command_cooldown').value)

        # State
        self.last_range: Optional[float] = None
        self.last_update: float = 0.0
        self.last_cmd: Optional[bool] = None  # True=Stop, False=Go
        self.last_cmd_time: float = 0.0

        # IO
        self.pub = self.create_publisher(Bool, self.follower_sup_topic, 10)
        self.create_subscription(Range, self.rear_topic, self._on_range, qos_profile_sensor_data)

        # Timer for decision
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"sup_order: rear={self.rear_topic} -> sup={self.follower_sup_topic} "
            f"(near<{self.near_stop_thresh:.3f}m → False(Stop), else True(Go); no-data → True)"
        )

    def _on_range(self, msg: Range) -> None:
        d = float(msg.range)
        if d >= msg.min_range and d <= msg.max_range:
            self.last_range = d
            self.last_update = time.monotonic()
        else:
            self.last_range = None
            self.last_update = time.monotonic()

    def _publish_cmd(self, go: bool) -> None:
        now = time.monotonic()
        if self.last_cmd is not None and go == self.last_cmd and (now - self.last_cmd_time) < self.command_cooldown:
            return
        self.last_cmd = go
        self.last_cmd_time = now
        self.pub.publish(Bool(data=go))
        self.get_logger().info(f"[SupOrder] follower={'Go' if go else 'Stop'} (rear={self.last_range if self.last_range is not None else 'None'})")

    def _tick(self) -> None:
        now = time.monotonic()
        no_data = (self.last_update == 0.0) or ((now - self.last_update) > self.no_data_timeout)
        if no_data:
            # 데이터 없음 → 멀다고 가정 → Go(True)
            self._publish_cmd(True)
            return
        assert self.last_range is not None
        d = self.last_range
        if d < self.near_stop_thresh:
            # 너무 가까움 → Stop(False)
            self._publish_cmd(False)
        else:
            # 이상적/멀음 → Go(True)
            self._publish_cmd(True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupOrder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
