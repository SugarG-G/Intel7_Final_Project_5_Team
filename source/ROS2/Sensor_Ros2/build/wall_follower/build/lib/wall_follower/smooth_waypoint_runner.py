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


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float  # radians


class SmoothWaypointRunner(Node):
    """Minimal waypoint follower that keeps motion continuous without Nav2."""

    def __init__(self) -> None:
        super().__init__('smooth_waypoint_runner')

        default_yaml = (
            Path(__file__).resolve().parent.parent / 'config' / 'waypoints.yaml'
        )

        self.declare_parameter('waypoint_yaml_path', str(default_yaml))
        self.declare_parameter('close_loop', True)
        self.declare_parameter('position_tolerance', 0.06)
        self.declare_parameter('final_yaw_tolerance', math.radians(4.0))
        self.declare_parameter('linear_kp', 0.9)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('angular_kp', 2.2)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('slowdown_distance', 0.35)
        self.declare_parameter('heading_slowdown_gain', 0.7)
        self.declare_parameter('final_align_kp', 2.0)

        waypoint_path = Path(
            self.get_parameter('waypoint_yaml_path').get_parameter_value().string_value
        ).expanduser().resolve()
        self.waypoints = self._load_waypoints(waypoint_path)
        if not self.waypoints:
            raise RuntimeError(f'No waypoints found in {waypoint_path}')

        if (
            self.get_parameter('close_loop').get_parameter_value().bool_value
            and (
                self.waypoints[0].x != self.waypoints[-1].x
                or self.waypoints[0].y != self.waypoints[-1].y
                or self.waypoints[0].yaw != self.waypoints[-1].yaw
            )
        ):
            first = self.waypoints[0]
            self.waypoints.append(
                Waypoint(
                    name=f'{first.name}_return',
                    x=first.x,
                    y=first.y,
                    yaw=first.yaw,
                )
            )
            self.get_logger().info(
                'close_loop enabled: appended start waypoint to the end for final alignment.'
            )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.timer = self.create_timer(0.05, self._on_timer)

        self.current_pose: Optional[tuple[float, float, float]] = None
        self.current_index = 0
        self.finished = False
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
                name = str(entry.get('name', f'wp_{idx + 1}'))
                pos = entry['position']
                yaw_deg = float(entry.get('yaw_deg', 0.0))
                waypoints.append(
                    Waypoint(
                        name=name,
                        x=float(pos['x']),
                        y=float(pos['y']),
                        yaw=math.radians(yaw_deg),
                    )
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Skipping waypoint index {idx}: {exc}')
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
        if self.current_pose is None or self.finished:
            if self.finished:
                self._publish_twist(0.0, 0.0)
            return

        x, y, yaw = self.current_pose

        if self.current_index >= len(self.waypoints):
            self.finished = True
            self._publish_twist(0.0, 0.0)
            self.get_logger().info('Waypoint loop completed.')
            return

        target = self.waypoints[self.current_index]
        dx = target.x - x
        dy = target.y - y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)
        final_yaw_error = normalize_angle(target.yaw - yaw)

        pos_tol = self.get_parameter('position_tolerance').get_parameter_value().double_value
        final_yaw_tol = self.get_parameter('final_yaw_tolerance').get_parameter_value().double_value
        max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        min_lin = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        lin_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        ang_kp = self.get_parameter('angular_kp').get_parameter_value().double_value
        slowdown_distance = self.get_parameter('slowdown_distance').get_parameter_value().double_value
        heading_slowdown_gain = self.get_parameter('heading_slowdown_gain').get_parameter_value().double_value
        final_align_kp = self.get_parameter('final_align_kp').get_parameter_value().double_value

        if distance < pos_tol:
            if self.current_index == len(self.waypoints) - 1:
                if abs(final_yaw_error) < final_yaw_tol:
                    self.finished = True
                    self._publish_twist(0.0, 0.0)
                    self.get_logger().info(
                        'Final waypoint reached. Pose aligned, stopping controller.'
                    )
                else:
                    angular = max(-max_ang, min(max_ang, final_align_kp * final_yaw_error))
                    self._publish_twist(0.0, angular)
                return

            self.get_logger().info(
                f"Waypoint {self.current_index + 1}/{len(self.waypoints)} "
                f"('{target.name}') reached. Advancing to next."
            )
            self.current_index += 1
            return

        linear = max(min_lin, min(max_lin, lin_kp * distance))
        if distance < slowdown_distance:
            ratio = distance / max(slowdown_distance, 1e-6)
            linear *= max(0.1, ratio)

        heading_scale = max(0.0, 1.0 - heading_slowdown_gain * abs(heading_error))
        linear *= max(0.1, heading_scale)

        angular = max(-max_ang, min(max_ang, ang_kp * heading_error))

        self._publish_twist(linear, angular)

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SmoothWaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Smooth waypoint runner interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
