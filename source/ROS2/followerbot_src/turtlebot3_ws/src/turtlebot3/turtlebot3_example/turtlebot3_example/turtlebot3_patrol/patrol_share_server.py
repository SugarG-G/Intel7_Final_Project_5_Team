#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_arm.srv import RobotArmCommand


class PatrolShareServer(Node):
    """서브로봇에서 패트롤 계획(모드/거리/반복)을 문자열로 받아 보관/로그하는 간단 서버.

    임시로 robot_arm/RobotArmCommand(string command) 서비스를 재활용합니다.
    payload 예: "plan:mode=3,dist=0.000,count=1"
    """

    def __init__(self) -> None:
        super().__init__('patrol_share_server')
        self._srv = self.create_service(RobotArmCommand, 'patrol/share_plan', self._on_share)
        self._srv_prog = self.create_service(RobotArmCommand, 'patrol/share_progress', self._on_progress)
        self._last_plan = None
        self._last_progress = None
        self.get_logger().info('patrol/share_plan, patrol/share_progress 서비스 준비 완료')

    def _on_share(self, req: RobotArmCommand.Request, res: RobotArmCommand.Response):
        payload = (req.command or '').strip()
        self._last_plan = payload
        self.get_logger().info(f'[share] 수신: {payload}')
        res.result = 'accepted'
        return res

    def _on_progress(self, req: RobotArmCommand.Request, res: RobotArmCommand.Response):
        payload = (req.command or '').strip()
        self._last_progress = payload
        self.get_logger().info(f'[progress] 수신: {payload}')
        res.result = 'accepted'
        return res


def main():
    rclpy.init()
    n = PatrolShareServer()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
