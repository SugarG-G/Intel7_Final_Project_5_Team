#!/usr/bin/env python3
"""Simple keyboard teleop overlay that scales patrol speeds via parameters."""

from __future__ import annotations

import os
import select
import sys
import threading
from typing import Optional

if os.name == 'nt':
    import msvcrt  # type: ignore
else:
    import termios
    import tty

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


class TeleopOverlay(Node):
    """Adjust max speeds on a patrol server using WASDX keys."""

    def __init__(self) -> None:
        super().__init__('teleop_overlay')

        self.declare_parameter('target_namespace', 'tb3_2')
        self.declare_parameter('server_node_name', 'turtlebot3_patrol_server')
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('min_scale', 0.2)
        self.declare_parameter('max_scale', 2.5)

        ns = self.get_parameter('target_namespace').get_parameter_value().string_value.strip('/')
        self._target_node = f'/{ns}/{self.get_parameter("server_node_name").value}'.rstrip('/')
        self._linear_step = float(self.get_parameter('linear_step').value)
        self._angular_step = float(self.get_parameter('angular_step').value)
        self._min_scale = float(self.get_parameter('min_scale').value)
        self._max_scale = float(self.get_parameter('max_scale').value)

        self._get_client = self.create_client(GetParameters, f'{self._target_node}/get_parameters')
        self._set_client = self.create_client(SetParameters, f'{self._target_node}/set_parameters')

        self._base_linear: Optional[float] = None
        self._base_angular: Optional[float] = None
        self._linear_scale = 1.0
        self._angular_scale = 1.0

        self._stop_event = threading.Event()
        self._keyboard_thread: Optional[threading.Thread] = None
        self._term_settings = None

        self._await_parameter_services()
        self._fetch_base_parameters()
        self._start_keyboard_listener()

        self.get_logger().info(
            f'teleop_overlay ready. Controlling {self._target_node}. '
            'Keys: w/x (+/- linear), a/d (+/- angular), s or space (reset).'
        )
        self._log_state()

    def _await_parameter_services(self) -> None:
        while rclpy.ok():
            if self._get_client.wait_for_service(timeout_sec=1.0) and \
               self._set_client.wait_for_service(timeout_sec=0.1):
                return
            self.get_logger().info(f'Waiting for parameter service on {self._target_node}...')
        raise RuntimeError('ROS shutdown before parameter services became available.')

    def _fetch_base_parameters(self) -> None:
        req = GetParameters.Request()
        req.names = ['max_linear_speed', 'max_angular_speed']
        future = self._get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError('Failed to read base patrol parameters.')

        values = {}
        for name, value in zip(req.names, future.result().values):
            values[name] = self._extract_value(value)

        self._base_linear = float(values.get('max_linear_speed', 0.07))
        self._base_angular = float(values.get('max_angular_speed', 0.4))
        self.get_logger().info(
            f'Base max speeds: linear={self._base_linear:.3f} m/s, angular={self._base_angular:.3f} rad/s'
        )

    def _start_keyboard_listener(self) -> None:
        if os.name == 'nt':
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop_windows, daemon=True)
        else:
            self._term_settings = termios.tcgetattr(sys.stdin)
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop_posix, daemon=True)
        self._keyboard_thread.start()

    # ------------------------------------------------------------------ keyboard loops
    def _keyboard_loop_posix(self) -> None:
        assert self._term_settings is not None
        try:
            while not self._stop_event.is_set():
                key = self._get_key_posix()
                if not key:
                    continue
                if key == '\x03':
                    raise KeyboardInterrupt
                self._handle_key(key)
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received, shutting down.')
            rclpy.shutdown()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
            self._stop_event.set()

    def _keyboard_loop_windows(self) -> None:
        try:
            while not self._stop_event.is_set():
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8').lower()
                    if key == '\x03':
                        raise KeyboardInterrupt
                    self._handle_key(key)
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received, shutting down.')
            rclpy.shutdown()
        finally:
            self._stop_event.set()

    def _get_key_posix(self) -> str:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        return key.lower()

    # ------------------------------------------------------------------ key handling
    def _handle_key(self, key: str) -> None:
        updated = False
        if key == 'w':
            self._linear_scale = self._clamp(self._linear_scale + self._linear_step, self._min_scale, self._max_scale)
            updated = True
        elif key == 'x':
            self._linear_scale = self._clamp(self._linear_scale - self._linear_step, self._min_scale, self._max_scale)
            updated = True
        elif key == 'a':
            self._angular_scale = self._clamp(self._angular_scale + self._angular_step, self._min_scale, self._max_scale)
            updated = True
        elif key == 'd':
            self._angular_scale = self._clamp(self._angular_scale - self._angular_step, self._min_scale, self._max_scale)
            updated = True
        elif key in ('s', ' '):
            self._linear_scale = 1.0
            self._angular_scale = 1.0
            updated = True

        if updated:
            self._apply_scales()
            self._log_state()

    # ------------------------------------------------------------------ parameter updates
    def _apply_scales(self) -> None:
        if self._base_linear is None or self._base_angular is None:
            return
        params = [
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, self._base_linear * self._linear_scale),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, self._base_angular * self._angular_scale),
        ]
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]
        future = self._set_client.call_async(req)
        future.add_done_callback(self._on_params_set)

    def _on_params_set(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to update patrol parameters: {exc}')
            return
        if not all(r.successful for r in result.results):
            self.get_logger().warn('One or more parameter updates were rejected.')

    # ------------------------------------------------------------------ helpers
    def _log_state(self) -> None:
        if self._base_linear is None or self._base_angular is None:
            return
        self.get_logger().info(
            f'Scales -> linear: {self._linear_scale:.2f} (max {self._base_linear * self._linear_scale:.3f} m/s), '
            f'angular: {self._angular_scale:.2f} (max {self._base_angular * self._angular_scale:.3f} rad/s)'
        )

    @staticmethod
    def _extract_value(param) -> float:
        if param.type == ParameterType.PARAMETER_DOUBLE:
            return param.double_value
        if param.type == ParameterType.PARAMETER_INTEGER:
            return float(param.integer_value)
        raise ValueError('Unsupported parameter type for patrol speed parameter.')

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=1.0)
        if os.name != 'nt' and self._term_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
