import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float  # radians


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw (rad) to geometry_msgs/Quaternion."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class WaypointRunner(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_runner')

        default_waypoint_path = Path(get_package_share_directory('wall_follower')) / 'config/waypoints.yaml'

        self.declare_parameter(
            'waypoint_yaml_path',
            str(default_waypoint_path)
        )
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_timeout', 120.0)

        waypoint_path = Path(
            self.get_parameter('waypoint_yaml_path').value
        ).expanduser().resolve()
        self.waypoints = self._load_waypoints(waypoint_path)
        if not self.waypoints:
            raise RuntimeError(f'No waypoints found in {waypoint_path}')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._current_index = 0
        self._goal_handle = None
        self._shutdown_requested = False

        self._start_timer = self.create_timer(1.0, self._try_start)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {waypoint_path}. Waiting for action server...'
        )

    def _load_waypoints(self, yaml_path: Path) -> List[Waypoint]:
        if not yaml_path.exists():
            self.get_logger().error(f'Waypoint YAML not found: {yaml_path}')
            return []
        try:
            data = yaml.safe_load(yaml_path.read_text())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to parse waypoint YAML: {exc}')
            return []
        entries = data.get('waypoints', [])
        waypoints: List[Waypoint] = []
        for idx, entry in enumerate(entries):
            try:
                name = entry.get('name') or f'wp_{idx + 1}'
                pos = entry['position']
                yaw_deg = float(entry.get('yaw_deg', entry.get('yaw_deg', 0.0)))
                yaw = math.radians(yaw_deg)
                waypoints.append(
                    Waypoint(
                        name=name,
                        x=float(pos['x']),
                        y=float(pos['y']),
                        yaw=yaw
                    )
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Skipping waypoint index {idx}: {exc}')
        return waypoints

    def _try_start(self) -> None:
        if self._shutdown_requested:
            return
        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            return
        self._start_timer.cancel()
        self.get_logger().info('Action server ready. Starting waypoint sequence.')
        self._send_next_waypoint()

    def _build_goal(self, waypoint: Waypoint) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation = yaw_to_quaternion(waypoint.yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _send_next_waypoint(self) -> None:
        if self._current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed. Shutting down.')
            self._shutdown_requested = True
            rclpy.shutdown()
            return

        waypoint = self.waypoints[self._current_index]
        goal = self._build_goal(waypoint)

        self.get_logger().info(
            f"[{self._current_index + 1}/{len(self.waypoints)}] Sending waypoint '{waypoint.name}' "
            f"(x={waypoint.x:.3f}, y={waypoint.y:.3f}, yaw={math.degrees(waypoint.yaw):.1f}°)"
        )

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._handle_feedback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to send goal: {exc}')
            self._shutdown_requested = True
            if rclpy.ok():
                rclpy.shutdown()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            self._shutdown_requested = True
            if rclpy.ok():
                rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Goal failed: {exc}')
            self._shutdown_requested = True
            if rclpy.ok():
                rclpy.shutdown()
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached successfully.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Waypoint aborted by navigation stack.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Waypoint goal was cancelled.')
        else:
            self.get_logger().warn(f'Waypoint result status code: {status}')

        self._current_index += 1
        self._send_next_waypoint()

    def _handle_feedback(self, feedback_msg: NavigateToPose.Feedback) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Current pose -> x={feedback.current_pose.pose.position.x:.2f}, "
            f"y={feedback.current_pose.pose.position.y:.2f}"
        )

    def destroy_node(self) -> bool:
        self._shutdown_requested = True
        if self._goal_handle is not None:
            try:
                if not self._goal_handle.is_done:
                    self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint runner interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
