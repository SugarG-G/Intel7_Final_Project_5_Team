import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import yaml


class WallFollowController(Node):
    def __init__(self) -> None:
        super().__init__('wall_follow_controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_distance', 0.20),             # meters
                ('corner_front_distance', 0.35),        # meters threshold to start turn
                ('corner_exit_distance', 0.55),         # meters threshold to finish turn
                ('straight_linear_speed', 0.08),        # m/s
                ('turn_linear_speed', 0.05),
                ('turn_angular_speed', 0.6),            # rad/s
                ('max_angular_speed', 0.8),
                ('kp_distance', 1.6),
                ('kd_distance', 0.3),
                ('side_angle_center_deg', 95.0),
                ('side_angle_width_deg', 20.0),
                ('front_angle_width_deg', 30.0),
                ('corner_target_count', 4),
                ('return_kp_linear', 0.4),
                ('return_kp_angular', 1.2),
                ('return_linear_limit', 0.10),
                ('return_angular_limit', 0.6),
                ('home_tolerance', 0.10),
                ('home_yaw_tolerance', 0.20),
                ('home_trigger_distance', 0.35),
                ('status_publish_period', 0.5),         # seconds
                ('map_yaml_path', '/home/ubuntu/map.yaml')
            ]
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_cmd = QoSProfile(depth=10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
        self.status_pub = self.create_publisher(String, 'wall_follow/status', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._handle_scan,
            qos_sensor
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._handle_odom,
            10
        )
        self.cmd_sub = self.create_subscription(
            String,
            'wall_follow/cmd',
            self._handle_command,
            10
        )

        self.timer = self.create_timer(0.05, self._on_timer)
        self.status_timer = self.create_timer(self.get_parameter('status_publish_period').value, self._publish_status)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.home_pose: Optional[tuple[float, float, float]] = None
        self.prev_state: Optional[str] = None
        self.prev_error: Optional[float] = None
        self.prev_error_time: Optional[float] = None
        self.corner_count = 0
        self.state = 'INIT'
        self.last_cmd = Twist()

        self.map_info = self._load_map_info()

        self.get_logger().info('벽따라 주행 컨트롤러 초기화 완료')

    def _load_map_info(self) -> Optional[dict]:
        yaml_path = Path(self.get_parameter('map_yaml_path').value)
        if not yaml_path.exists():
            self.get_logger().warn(f'지도 YAML 파일을 찾을 수 없습니다: {yaml_path}')
            return None
        try:
            data = yaml.safe_load(yaml_path.read_text())
            image_path = yaml_path.parent / data.get('image', '')
            info = {
                'image': str(image_path.resolve()),
                'resolution': float(data.get('resolution', 0.05)),
                'origin': data.get('origin', [0.0, 0.0, 0.0])
            }
            self.get_logger().info(
                f"지도 정보 로드 완료 (이미지: {info['image']}, 해상도: {info['resolution']:.3f}m, "
                f"원점: {info['origin']})"
            )
            return info
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'지도 YAML 읽기 실패: {exc}')
            return None

    def _handle_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _handle_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.home_pose is None:
            self.home_pose = (*self._pose_from_odom(msg),)
            self.get_logger().info(
                f'초기 자세 저장: x={self.home_pose[0]:.3f}, y={self.home_pose[1]:.3f}, yaw={self.home_pose[2]:.3f}'
            )

    def _handle_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        self.get_logger().info(f'터미널 명령 수신: {command}')
        if command == 'pause':
            if self.state != 'PAUSE':
                self.prev_state = self.state
                self.state = 'PAUSE'
                self._stop_motion()
        elif command == 'resume':
            if self.state == 'PAUSE':
                self.state = self.prev_state or 'FOLLOW'
                self.prev_state = None
        elif command == 'reset':
            self._reset_controller()
        elif command == 'quit':
            self.get_logger().info('종료 명령 수신, 노드 종료를 시도합니다.')
            rclpy.shutdown()
        elif command == 'stop':
            self.state = 'COMPLETE'
            self._stop_motion()

    def _reset_controller(self) -> None:
        self.corner_count = 0
        self.prev_error = None
        self.prev_error_time = None
        self.prev_state = None
        self.state = 'INIT'
        self.home_pose = None
        self.get_logger().info('컨트롤러가 리셋되었습니다.')
        self._stop_motion()

    def _on_timer(self) -> None:
        if self.latest_odom is None or self.latest_scan is None:
            self._stop_motion()
            return

        if self.state == 'INIT':
            if self.latest_scan is not None and self.latest_odom is not None:
                self.state = 'FOLLOW'
                self.get_logger().info('주행 상태로 전환 (INIT -> FOLLOW)')
            self._stop_motion()
            return

        if self.state == 'PAUSE':
            self._stop_motion()
            return

        if self.state == 'COMPLETE':
            self._stop_motion()
            return

        if self.state == 'RETURN_HOME':
            self._execute_return_home()
            return

        current_time = time.monotonic()
        scan = self.latest_scan
        side_distance = self._sector_average(scan, self._deg_to_rad(self.get_parameter('side_angle_center_deg').value), self._deg_to_rad(self.get_parameter('side_angle_width_deg').value))
        front_distance = self._sector_average(scan, 0.0, self._deg_to_rad(self.get_parameter('front_angle_width_deg').value))

        if front_distance is None:
            front_distance = float('inf')
        if side_distance is None:
            side_distance = float('inf')

        if self.state == 'FOLLOW':
            if front_distance < self.get_parameter('corner_front_distance').value:
                self.state = 'TURN'
                self.prev_error = None
                self.prev_error_time = None
                self.get_logger().info('코너 감지 -> 곡선 주행 상태 전환')
                self._command_motion(
                    linear=self.get_parameter('turn_linear_speed').value,
                    angular=self.get_parameter('turn_angular_speed').value
                )
                return
            self._execute_follow(side_distance, current_time)
            self._maybe_trigger_return()
            return

        if self.state == 'TURN':
            self._command_motion(
                linear=self.get_parameter('turn_linear_speed').value,
                angular=self.get_parameter('turn_angular_speed').value
            )
            if (
                front_distance > self.get_parameter('corner_exit_distance').value
                and side_distance < self.get_parameter('desired_distance').value * 1.8
            ):
                self.corner_count += 1
                self.get_logger().info(f'코너 주행 완료, 누적 횟수: {self.corner_count}')
                self.state = 'FOLLOW'
                self.prev_error = None
                self.prev_error_time = None
            self._maybe_trigger_return()
            return

    def _maybe_trigger_return(self) -> None:
        if self.home_pose is None:
            return
        if self.corner_count < self.get_parameter('corner_target_count').value:
            return
        current_pose = self._pose_from_odom(self.latest_odom)
        distance_to_home = math.hypot(current_pose[0] - self.home_pose[0], current_pose[1] - self.home_pose[1])
        if distance_to_home < self.get_parameter('home_trigger_distance').value:
            self.get_logger().info('기본 자세 복귀 절차 시작')
            self.state = 'RETURN_HOME'

    def _execute_follow(self, side_distance: float, current_time: float) -> None:
        desired = self.get_parameter('desired_distance').value
        error = desired - side_distance
        derivative = 0.0
        if self.prev_error is not None and self.prev_error_time is not None:
            dt = current_time - self.prev_error_time
            if dt > 1e-3:
                derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_error_time = current_time

        kp = self.get_parameter('kp_distance').value
        kd = self.get_parameter('kd_distance').value
        angular = kp * error + kd * derivative
        angular = self._clamp(angular, -self.get_parameter('max_angular_speed').value, self.get_parameter('max_angular_speed').value)
        linear = self.get_parameter('straight_linear_speed').value
        self._command_motion(linear=linear, angular=angular)

    def _execute_return_home(self) -> None:
        if self.home_pose is None or self.latest_odom is None:
            self._stop_motion()
            return
        current_pose = self._pose_from_odom(self.latest_odom)
        dx = self.home_pose[0] - current_pose[0]
        dy = self.home_pose[1] - current_pose[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_heading - current_pose[2])

        lin_gain = self.get_parameter('return_kp_linear').value
        ang_gain = self.get_parameter('return_kp_angular').value

        linear = lin_gain * distance
        angular = ang_gain * yaw_error

        linear = self._clamp(linear, -self.get_parameter('return_linear_limit').value, self.get_parameter('return_linear_limit').value)
        angular = self._clamp(angular, -self.get_parameter('return_angular_limit').value, self.get_parameter('return_angular_limit').value)

        if distance < self.get_parameter('home_tolerance').value and abs(yaw_error) < self.get_parameter('home_yaw_tolerance').value:
            self.get_logger().info('초기 자세 복귀 완료, 작업 종료')
            self.state = 'COMPLETE'
            self._stop_motion()
            return

        self._command_motion(linear=linear, angular=angular)

    def _publish_status(self) -> None:
        if self.latest_odom is None:
            return
        pose = self._pose_from_odom(self.latest_odom)
        status_text = self._build_status_text(pose)
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)

    def _build_status_text(self, pose: tuple[float, float, float]) -> str:
        state_text = self.state
        if state_text == 'FOLLOW':
            state_text = '직선 주행'
        elif state_text == 'TURN':
            state_text = '코너 주행'
        elif state_text == 'PAUSE':
            state_text = '일시 정지'
        elif state_text == 'RETURN_HOME':
            state_text = '기준 자세 복귀'
        elif state_text == 'COMPLETE':
            state_text = '완료'
        elif state_text == 'INIT':
            state_text = '준비 중'

        map_summary = '지도 정보 없음'
        if self.map_info is not None:
            map_summary = (
                f"지도: 해상도 {self.map_info['resolution']:.3f}m, "
                f"원점 ({self.map_info['origin'][0]:.2f}, {self.map_info['origin'][1]:.2f})"
            )

        status = (
            f"상태: {state_text} | 코너: {self.corner_count}/{self.get_parameter('corner_target_count').value}"
            f" | 위치 x={pose[0]:.2f}m, y={pose[1]:.2f}m, yaw={pose[2]:.2f}rad"
            f" | 선속도={self.last_cmd.linear.x:.2f}m/s, 각속도={self.last_cmd.angular.z:.2f}rad/s"
            f" | {map_summary}"
        )
        return status

    def _command_motion(self, *, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
        self.last_cmd = twist

    def _stop_motion(self) -> None:
        if self.last_cmd.linear.x == 0.0 and self.last_cmd.angular.z == 0.0:
            return
        self._command_motion(linear=0.0, angular=0.0)

    @staticmethod
    def _pose_from_odom(odom: Odometry) -> tuple[float, float, float]:
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(min(value, maximum), minimum)

    def _sector_average(self, scan: LaserScan, center: float, width: float) -> Optional[float]:
        if scan is None:
            return None
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        inc = scan.angle_increment
        min_angle = center - width / 2.0
        max_angle = center + width / 2.0
        if max_angle < angle_min or min_angle > angle_max:
            return None
        start_index = max(0, int((min_angle - angle_min) / inc))
        end_index = min(len(scan.ranges) - 1, int((max_angle - angle_min) / inc))
        if end_index <= start_index:
            return None
        values = [
            scan.ranges[i]
            for i in range(start_index, end_index + 1)
            if math.isfinite(scan.ranges[i])
        ]
        if not values:
            return None
        return sum(values) / len(values)

    @staticmethod
    def _deg_to_rad(angle_deg: float) -> float:
        return angle_deg * math.pi / 180.0


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WallFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('컨트롤러 노드가 Ctrl+C로 종료되었습니다.')
    finally:
        node._stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
