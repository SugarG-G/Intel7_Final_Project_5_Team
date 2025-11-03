#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PatrolStopDummy(Node):
    """간단한 Stop 신호 송신 노드."""

    def __init__(self) -> None:
        super().__init__('patrol_stop_dummy')
        self.publisher = self.create_publisher(Bool, 'patrol/stop', 10)
        self.get_logger().info(
            'Stop 더미 노드 준비 완료. '
            "'s' 입력 → 정지, 'r' 입력 → 재개, 'q' → 종료"
        )

    def publish_stop(self, flag: bool) -> None:
        msg = Bool()
        msg.data = flag
        self.publisher.publish(msg)
        self.get_logger().warn('정지 신호 전송') if flag else self.get_logger().info('정지 해제 신호 전송')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolStopDummy()

    try:
        while rclpy.ok():
            cmd = input('[s]top / [r]esume / [q]uit > ').strip().lower()
            if cmd == 's':
                node.publish_stop(True)
            elif cmd == 'r':
                node.publish_stop(False)
            elif cmd == 'q':
                break
            else:
                print('지원하지 않는 입력입니다.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
