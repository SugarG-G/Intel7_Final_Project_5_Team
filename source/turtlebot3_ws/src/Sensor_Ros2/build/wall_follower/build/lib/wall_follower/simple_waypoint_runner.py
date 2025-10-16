from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float  # radians


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SimpleWaypointRunner(Node):
    """Lightweight waypoint follower that relies only on /odom feedback."""

    def __init__(self) -> None:
        super().__init__('simple_waypoint_runner')

        default_yaml = (
            Path(__file__).resolve().parent.parent / 'config' / 'waypoints.yaml'
        )

        self.declare_parameter('waypoint_yaml_path', str(default_yaml))
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', math.radians(5.0))
        self.declare_parameter('final_yaw_tolerance', math.radians(5.0))
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('angular_kp', 1.8)

        waypoint_path = Path(
            self.get_parameter('waypoint_yaml_path').get_parameter_value().string_value
        ).expanduser().resolve()
        self.waypoints = self._load_waypoints(waypoint_path)
        if not self.waypoints:
            raise RuntimeError(f'No waypoints found in {waypoint_path}')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.timer = self.create_timer(0.05, self._on_timer)

        self.current_pose: Optional[tuple[float, float, float]] = None
        self.current_index = 0
        self.state = 'WAIT_ODOM'
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints. Waiting for odometry...'
        )

    def _load_waypoints(self, yaml_path: Path) -> List[Waypoint]:
        if not yaml_path.exists():
            self.get_logger().error(f'Waypoint file not found: {yaml_path}')
            return []
        try:
            data = yaml.safe_load(yaml_path.read_text())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read waypoint YAML: {exc}')
            return []

        waypoints: List[Waypoint] = []
        for idx, entry in enumerate(data.get('waypoints', [])):
            try:
                pos = entry['position']
                yaw_deg = float(entry.get('yaw_deg', 0.0))
                waypoints.append(
                    Waypoint(
                        x=float(pos['x']),
                        y=float(pos['y']),
                        yaw=math.radians(yaw_deg),
                    )
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Skipping waypoint {idx}: {exc}')
        return waypoints

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _on_timer(self) -> None:
        if self.current_pose is None:
            return

        if self.current_index >= len(self.waypoints):
            self._publish_twist(0.0, 0.0)
            return

        target = self.waypoints[self.current_index]
        x, y, yaw = self.current_pose
        dx = target.x - x
        dy = target.y - y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        yaw_error = normalize_angle(heading - yaw)
        final_yaw_error = normalize_angle(target.yaw - yaw)

        pos_tol = self.get_parameter('position_tolerance').get_parameter_value().double_value
        heading_tol = self.get_parameter('heading_tolerance').get_parameter_value().double_value
        final_yaw_tol = self.get_parameter('final_yaw_tolerance').get_parameter_value().double_value
        max_lin = self.get_parameter('linear_speed').get_parameter_value().double_value
        max_ang = self.get_parameter('angular_speed').get_parameter_value().double_value
        lin_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        ang_kp = self.get_parameter('angular_kp').get_parameter_value().double_value

        if self.state == 'WAIT_ODOM':
            self.get_logger().info('Odometry received. Starting waypoint loop.')
            self.state = 'ROTATE_TO_HEADING'

        if self.state == 'ROTATE_TO_HEADING':
            if distance < pos_tol:
                self.state = 'ALIGN_FINAL'
            elif abs(yaw_error) < heading_tol:
                self.state = 'DRIVE'
            else:
                angular = max(-max_ang, min(max_ang, ang_kp * yaw_error))
                self._publish_twist(0.0, angular)
                return

        if self.state == 'DRIVE':
            if distance < pos_tol:
                self.state = 'ALIGN_FINAL'
                self._publish_twist(0.0, 0.0)
            else:
                linear = max(-max_lin, min(max_lin, lin_kp * distance))
                angular = max(-max_ang, min(max_ang, ang_kp * yaw_error))
                self._publish_twist(linear, angular)
                return

        if self.state == 'ALIGN_FINAL':
            if abs(final_yaw_error) < final_yaw_tol:
                self._publish_twist(0.0, 0.0)
                self.get_logger().info(
                    f'Waypoint {self.current_index + 1}/{len(self.waypoints)} '
                    f'completed at ({target.x:.3f}, {target.y:.3f}).'
                )
                self.current_index += 1
                self.state = 'ROTATE_TO_HEADING'
            else:
                angular = max(-max_ang, min(max_ang, ang_kp * final_yaw_error))
                self._publish_twist(0.0, angular)

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimpleWaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simple waypoint runner interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
