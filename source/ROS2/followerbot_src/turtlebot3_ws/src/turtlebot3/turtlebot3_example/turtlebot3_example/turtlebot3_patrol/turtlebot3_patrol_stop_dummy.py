#!/usr/bin/env python3
import math
import threading
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String
from std_msgs.msg import Empty as EmptyMsg

# 로봇팔 서비스 타입
from robot_arm.srv import RobotArmCommand
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
try:
    # 팔로워 정렬 액션 메시지(패키지 존재 시 사용)
    from turtlebot3_applications_msgs.action import Handover
except Exception:  # 패키지 미설치 환경에서도 파일 로드는 되도록
    Handover = None


class AutoSequenceState(Enum):
    IDLE = auto()
    STOPPING = auto()
    WAIT_BEFORE_TASK = auto()
    TASK_RUNNING = auto()
    POST_TASK = auto()


class PatrolStopDummy(Node):
    """콘솔 입력과 이벤트·레이저 정보를 동시에 확인 + 로봇팔 호출을 포함한 Stop 더미 노드."""

    def __init__(self) -> None:
        super().__init__('patrol_stop_dummy')

        # 퍼블리셔/구독자
        self.publisher = self.create_publisher(Bool, 'patrol/stop', 10)
        self.event_sub = self.create_subscription(
            String, 'crop_task/events', self._on_event, 10
        )

        # 레이저/자동 시퀀스 파라미터
        self.laser_topic = self.declare_parameter('laser_topic', '/scan').value
        self.laser_topic_type = self.declare_parameter('laser_topic_type', 'scan').value.lower()
        self.detection_distance = float(self.declare_parameter('laser_detection_distance', 0.12).value)
        self.auto_sequence_enabled = self.declare_parameter('auto_sequence_enabled', True).value
        self.auto_stop_min = float(self.declare_parameter('auto_stop_min', 0.10).value)
        self.auto_stop_max = float(self.declare_parameter('auto_stop_max', 0.12).value)
        self.auto_pre_task_delay = float(self.declare_parameter('auto_pre_task_delay', 2.0).value)
        self.auto_task_duration = float(self.declare_parameter('auto_task_duration', 10.0).value)  # 팔 호출로 대체
        self.auto_post_task_delay = float(self.declare_parameter('auto_post_task_delay', 2.0).value)

        # 재트리거 방지 파라미터
        self.retrigger_cooldown = float(self.declare_parameter('retrigger_cooldown', 5.0).value)
        self.require_exit_before_retrigger = bool(self.declare_parameter('require_exit_before_retrigger', True).value)

        # 로봇팔 서비스 파라미터
        self.arm_service_name = self.declare_parameter('arm_service_name', '/robot_arm_control').value
        self.arm_command = self.declare_parameter('arm_command', 'detect').value
        self.arm_call_timeout = float(self.declare_parameter('arm_call_timeout', 60.0).value)
        self.arm_wait_ready_timeout = float(self.declare_parameter('arm_wait_ready_timeout', 5.0).value)

        # 팔로우 로봇 트리거 설정(토픽/액션/서비스)
        self.follower_trigger_enabled = bool(self.declare_parameter('follower_trigger_enabled', True).value)
        self.follower_trigger_when = str(self.declare_parameter('follower_trigger_when', 'on_detection').value)
        self.follower_trigger_topic = str(self.declare_parameter('follower_trigger_topic', '/tb3_2/follow/trigger').value)
        self.follower_trigger_latch = bool(self.declare_parameter('follower_trigger_latch', True).value)
        # 액션(정렬 시작)·서비스(적재 완료) 엔드포인트
        self.follower_handover_action = str(self.declare_parameter('follower_handover_action', '/tb3_2/handover').value)
        self.follower_loading_done_service = str(self.declare_parameter('follower_loading_done_service', '/tb3_2/loading_complete').value)
        self.handover_wait_ready_timeout = float(self.declare_parameter('handover_wait_ready_timeout', 3.0).value)
        self.loading_done_wait_ready_timeout = float(self.declare_parameter('loading_done_wait_ready_timeout', 3.0).value)

        # 내부 상태
        self.last_laser_distance: Optional[float] = None
        self.last_update_time: Optional[float] = None
        self.data_timeout = float(self.declare_parameter('laser_data_timeout', 1.0).value)
        self.auto_sequence_active = False
        self._auto_state = AutoSequenceState.IDLE
        self._auto_timer = None
        self._arm_future = None
        self._arm_watchdog = None
        self._in_window = False
        self._retrigger_block_until = 0.0

        # Steady 타이머 정확도 향상을 위한 Steady Clock
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        # 로봇팔 서비스 클라이언트
        self.arm_client = self.create_client(RobotArmCommand, self.arm_service_name)
        start_wait = time.monotonic()
        while not self.arm_client.wait_for_service(timeout_sec=0.5):
            if time.monotonic() - start_wait > self.arm_wait_ready_timeout:
                self.get_logger().warn(f"로봇팔 서비스 준비 대기 시간 초과: {self.arm_service_name}")
                break
            self.get_logger().info(f'로봇팔 서비스 대기 중: {self.arm_service_name}')

        # 레이저 구독
        if self.laser_topic_type == 'range':
            self.laser_sub = self.create_subscription(Range, self.laser_topic, self._on_range, qos_profile_sensor_data)
        else:
            self.laser_sub = self.create_subscription(LaserScan, self.laser_topic, self._on_scan, qos_profile_sensor_data)

        # 팔로우 로봇 트리거 퍼블리셔
        if self.follower_trigger_enabled:
            qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if self.follower_trigger_latch else DurabilityPolicy.VOLATILE)
            self.follower_trigger_pub = self.create_publisher(EmptyMsg, self.follower_trigger_topic, qos)
            self.get_logger().info(f"팔로우 트리거 활성화: topic={self.follower_trigger_topic}, when={self.follower_trigger_when}")

        # 팔로워 트리거(액션/서비스) 클라이언트 준비
        if Handover is not None:
            self.handover_ac = ActionClient(self, Handover, self.follower_handover_action)
            self.get_logger().info(f"팔로워 액션 대기: {self.follower_handover_action}")
            self.handover_ac.wait_for_server(timeout_sec=self.handover_wait_ready_timeout)
        else:
            self.handover_ac = None
            self.get_logger().warn('turtlebot3_applications_msgs가 없어 handover 액션을 비활성화합니다.')

        self.loading_done_cli = self.create_client(Trigger, self.follower_loading_done_service)
        self.get_logger().info(f"팔로워 완료 서비스 대기: {self.follower_loading_done_service}")
        self.loading_done_cli.wait_for_service(timeout_sec=self.loading_done_wait_ready_timeout)

        # 실시간 모니터 스레드
        self._monitoring = False
        self._monitor_thread = None
        self._monitor_stop = threading.Event()

        self.get_logger().info("Stop 더미 준비. 's'=정지, 'r'=재개, 'l'=1회확인, 'c'=실시간, 'q'=종료")
        self.get_logger().info(
            f"이벤트 'crop_task/events' 구독, 레이저 '{self.laser_topic}'({self.laser_topic_type}) 감시 "
            f"(임계 {self.detection_distance:.2f}m)."
        )

    # 기본 퍼블리시/유틸
    def publish_stop(self, flag: bool) -> None:
        msg = Bool()
        msg.data = flag
        self.publisher.publish(msg)
        if flag:
            self.get_logger().warn('정지 신호 전송')
        else:
            self.get_logger().info('정지 해제 신호 전송')

    def report_laser(self) -> None:
        if (self.last_update_time is not None and time.monotonic() - self.last_update_time > self.data_timeout):
            self._mark_no_data()
        if self.last_laser_distance is None:
            print('레이저 데이터 수신 대기 중입니다.')
        # 추가 모니터 없이 기본 레이저만 출력
        if self.last_laser_distance is None:
            return
        if self.last_laser_distance <= self.detection_distance:
            print(f'물체 탐지! 최소 거리 {self.last_laser_distance:.2f} m (임계 {self.detection_distance:.2f} m)')
        else:
            print(f'안전: 최소 거리 {self.last_laser_distance:.2f} m (임계 {self.detection_distance:.2f} m)')

    # 입력/센서 콜백
    def _on_event(self, msg: String) -> None:
        self.get_logger().info(f'[event] {msg.data}')

    def _on_scan(self, msg: LaserScan) -> None:
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            self._set_laser_distance(min(valid_ranges))
        else:
            self._mark_no_data()

    def _on_range(self, msg: Range) -> None:
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            self._set_laser_distance(msg.range)
        else:
            self._mark_no_data()


    def _set_laser_distance(self, distance: float) -> None:
        self.last_laser_distance = distance
        self.last_update_time = time.monotonic()
        if self._monitoring and self.last_laser_distance is not None:
            self._latest_monitor_text = (
                f'실시간 최소 거리 {self.last_laser_distance:.2f} m (임계 {self.detection_distance:.2f} m)'
            )
        elif self._monitoring:
            self._latest_monitor_text = '실시간 데이터 없음'
        if self.auto_sequence_enabled:
            self._check_auto_sequence(distance)

    def _mark_no_data(self) -> None:
        self.last_laser_distance = None
        self.last_update_time = None
        if self._monitoring:
            self._latest_monitor_text = '실시간 데이터 없음'

    # 자동 시퀀스 상태머신 + 재트리거 방지
    def _check_auto_sequence(self, distance: float) -> None:
        now = time.monotonic()
        in_window = (self.auto_stop_min <= distance <= self.auto_stop_max)

        # 쿨다운 중이면 무시
        if now < self._retrigger_block_until:
            self._in_window = in_window
            return

        # 상승엣지(창 밖→창 안)에서만 트리거
        trigger_ok = in_window and (not self._in_window if self.require_exit_before_retrigger else True)

        if trigger_ok and (not self.auto_sequence_active) and (self._auto_state == AutoSequenceState.IDLE):
            self._enter_auto_state(AutoSequenceState.STOPPING)

        # 마지막 상태 업데이트
        self._in_window = in_window

    def _enter_auto_state(self, state: AutoSequenceState) -> None:
        self._auto_state = state

        if state == AutoSequenceState.IDLE:
            self.auto_sequence_active = False
            self._cancel_auto_timer()
            self._cancel_arm_watchdog()
            self._arm_future = None
            # 재트리거 쿨다운 시작 + 엣지 초기화
            self._retrigger_block_until = time.monotonic() + self.retrigger_cooldown
            self._in_window = False
            return

        if state == AutoSequenceState.STOPPING:
            self.auto_sequence_active = True
            print('물체 탐지! 터틀봇을 정지합니다!')
            self.publish_stop(True)
            # 팔로우 트리거: 탐지 시점
            self._maybe_publish_follower_trigger('on_detection')
            # 팔로워 정렬 액션 트리거
            self._maybe_send_handover_action()
            if self.auto_pre_task_delay > 0:
                print(f'작업을 시작하기 전 {self.auto_pre_task_delay:.1f}초 기다립니다.')
            self._schedule_auto_timer(self.auto_pre_task_delay, AutoSequenceState.WAIT_BEFORE_TASK)

        elif state == AutoSequenceState.WAIT_BEFORE_TASK:
            # 로봇팔 시퀀스 시작
            self.get_logger().info('로봇팔 시퀀스 실행 요청')
            # 팔로우 트리거: 작업 시작 시점
            self._maybe_publish_follower_trigger('on_task_start')
            self._start_arm_task()

        elif state == AutoSequenceState.TASK_RUNNING:
            # 팔 호출 완료를 기다리는 중
            pass

        elif state == AutoSequenceState.POST_TASK:
            if self.auto_post_task_delay > 0:
                print(f'자세 정렬 중... {self.auto_post_task_delay:.1f}초 대기합니다.')
            # 팔로우 트리거: 작업 완료 시점
            self._maybe_publish_follower_trigger('on_task_done')
            self._schedule_auto_timer(self.auto_post_task_delay, AutoSequenceState.IDLE)

    def _schedule_auto_timer(self, duration: float, next_state: AutoSequenceState) -> None:
        self._cancel_auto_timer()
        if duration <= 0.0:
            if next_state == AutoSequenceState.IDLE:
                print('터틀봇을 다시 주행합니다.')
                self.publish_stop(False)
            self._enter_auto_state(next_state)
            return

        def timer_callback() -> None:
            if self._auto_timer is not None:
                self._auto_timer.cancel()
                self._auto_timer = None
            if next_state == AutoSequenceState.IDLE:
                print('터틀봇을 다시 주행합니다.')
                self.publish_stop(False)
            self._enter_auto_state(next_state)

        self._auto_timer = self.create_timer(duration, timer_callback, clock=self._steady_clock)

    def _cancel_auto_timer(self) -> None:
        if self._auto_timer is not None:
            self._auto_timer.cancel()
            self._auto_timer = None

    # 로봇팔 호출
    def _start_arm_task(self) -> None:
        # 상태 전환
        self._auto_state = AutoSequenceState.TASK_RUNNING

        # 서비스 준비 재확인(최대 arm_wait_ready_timeout)
        ready_start = time.monotonic()
        while not self.arm_client.service_is_ready():
            if time.monotonic() - ready_start > self.arm_wait_ready_timeout:
                self.get_logger().error('로봇팔 서비스 미준비: 타임아웃. 작업을 건너뜁니다.')
                self._enter_auto_state(AutoSequenceState.POST_TASK)
                return
            self.arm_client.wait_for_service(timeout_sec=0.5)

        # 요청 전송
        req = RobotArmCommand.Request()
        req.command = self.arm_command
        try:
            self._arm_future = self.arm_client.call_async(req)
        except Exception as e:
            self.get_logger().error(f'로봇팔 서비스 호출 예외: {e}')
            self._enter_auto_state(AutoSequenceState.POST_TASK)
            return

        # 완료 콜백
        self._arm_future.add_done_callback(self._on_arm_done)

        # 타임아웃 워치독
        self._cancel_arm_watchdog()

        def on_timeout():
            self._cancel_arm_watchdog()
            self.get_logger().error('로봇팔 서비스 호출 시간 초과')
            self._enter_auto_state(AutoSequenceState.POST_TASK)

        self._arm_watchdog = self.create_timer(self.arm_call_timeout, on_timeout, clock=self._steady_clock)

    def _on_arm_done(self, future) -> None:
        self._cancel_arm_watchdog()
        try:
            res = future.result()
            msg = getattr(res, 'result', '')
            self.get_logger().info(f'로봇팔 응답: {msg}')
        except Exception as e:
            self.get_logger().error(f'로봇팔 서비스 결과 예외: {e}')
        print('작업 완료')
        # 팔로워 적재 완료 트리거(서비스)
        self._maybe_call_loading_done()
        self._enter_auto_state(AutoSequenceState.POST_TASK)

    def _maybe_publish_follower_trigger(self, when: str) -> None:
        if not getattr(self, 'follower_trigger_enabled', False):
            return
        if self.follower_trigger_when != when:
            return
        try:
            self.follower_trigger_pub.publish(EmptyMsg())
            self.get_logger().info(f"팔로우 트리거 전송: when={when} -> {self.follower_trigger_topic}")
        except Exception as e:
            self.get_logger().error(f"팔로우 트리거 전송 실패: {e}")

    def _cancel_arm_watchdog(self) -> None:
        if self._arm_watchdog is not None:
            self._arm_watchdog.cancel()
            self._arm_watchdog = None

    def _maybe_send_handover_action(self) -> None:
        try:
            if self.handover_ac is None:
                return
            if not self.handover_ac.server_is_ready():
                self.get_logger().warn('팔로워 handover 액션 서버 미준비')
                return
            goal = Handover.Goal() if Handover is not None else None
            self.handover_ac.send_goal_async(goal)
            self.get_logger().info(f"팔로워 정렬 시작 트리거 전송: {self.follower_handover_action}")
        except Exception as e:
            self.get_logger().error(f"handover 액션 전송 실패: {e}")

    def _maybe_call_loading_done(self) -> None:
        try:
            if not self.loading_done_cli.service_is_ready():
                self.get_logger().warn('팔로워 완료 서비스 미준비')
                return
            req = Trigger.Request()
            self.loading_done_cli.call_async(req)
            self.get_logger().info(f"팔로워 적재 완료 트리거 전송: {self.follower_loading_done_service}")
        except Exception as e:
            self.get_logger().error(f"loading_complete 서비스 호출 실패: {e}")

    # 모니터 토글
    def toggle_monitor(self) -> None:
        if not self._monitoring:
            self._start_monitor()
        else:
            self._stop_monitor()

    def _start_monitor(self) -> None:
        if self._monitoring:
            return
        self._monitoring = True
        self._monitor_stop.clear()
        self._latest_monitor_text = '실시간 데이터 없음'

        def _loop():
            while not self._monitor_stop.is_set():
                if (self.last_update_time is not None and
                        time.monotonic() - self.last_update_time > self.data_timeout):
                    self._mark_no_data()
                if hasattr(self, '_latest_monitor_text'):
                    print(self._latest_monitor_text)
                time.sleep(0.5)

        self._monitor_thread = threading.Thread(target=_loop, daemon=True)
        self._monitor_thread.start()
        print("레이저 실시간 조회 시작 (다시 'c' 입력 시 종료)")

    def _stop_monitor(self) -> None:
        if not self._monitoring:
            return
        self._monitor_stop.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        self._monitoring = False
        print("레이저 실시간 조회 종료")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolStopDummy()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            cmd = input('[s]top / [r]esume / [l]idar check / [c] monitor / [q]uit > ').strip().lower()
            if cmd == 's':
                node.publish_stop(True)
            elif cmd == 'r':
                node.publish_stop(False)
            elif cmd == 'l':
                node.report_laser()
            elif cmd == 'c':
                node.toggle_monitor()
            elif cmd == 'q':
                break
            else:
                print('지원하지 않는 입력입니다.')
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_monitor()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
