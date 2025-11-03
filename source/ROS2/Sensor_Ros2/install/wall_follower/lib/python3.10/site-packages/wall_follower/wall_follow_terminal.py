import select
import sys
import termios
import threading
import tty
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TerminalInterface(Node):
    def __init__(self) -> None:
        super().__init__('wall_follow_terminal')

        self.status_sub = self.create_subscription(
            String,
            'wall_follow/status',
            self._handle_status,
            10
        )
        self.cmd_pub = self.create_publisher(String, 'wall_follow/cmd', 10)
        self.print_timer = self.create_timer(0.5, self._print_status)

        self._status_text = '상태 정보를 수신하는 중입니다...'
        self._status_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._keyboard_thread: Optional[threading.Thread] = None
        self._orig_term: Optional[list[int]] = None

        if sys.stdin.isatty():
            self._enable_keyboard()
        else:
            self.get_logger().warn('표준 입력이 터미널이 아니어서 키보드 제어를 비활성화합니다.')

        self.get_logger().info('터미널 준비 완료: P(정지) / S(재개) / R(리셋) / Q(종료)')

    def _enable_keyboard(self) -> None:
        fd = sys.stdin.fileno()
        self._orig_term = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

    def _keyboard_loop(self) -> None:
        fd = sys.stdin.fileno()
        while not self._stop_event.is_set() and rclpy.ok():
            ready, _, _ = select.select([fd], [], [], 0.1)
            if fd in ready:
                char = sys.stdin.read(1)
                if not char:
                    continue
                lower = char.lower()
                if lower == 'p':
                    self._publish_command('pause', '일시 정지를 요청합니다.')
                elif lower == 's':
                    self._publish_command('resume', '주행 재개를 요청합니다.')
                elif lower == 'r':
                    self._publish_command('reset', '시스템 리셋을 요청합니다.')
                elif lower == 'q':
                    self._publish_command('quit', '프로그램 종료를 요청합니다.')
                    self._stop_event.set()

    def _publish_command(self, command: str, log_text: str) -> None:
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(log_text)

    def _handle_status(self, msg: String) -> None:
        with self._status_lock:
            self._status_text = msg.data

    def _print_status(self) -> None:
        with self._status_lock:
            status = self._status_text
        sys.stdout.write('\r' + ' ' * 120 + '\r')
        sys.stdout.write(status + '\n')
        sys.stdout.flush()

    def destroy_node(self) -> None:
        self._stop_event.set()
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)
        if self._orig_term is not None and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TerminalInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('터미널 노드가 Ctrl+C로 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
