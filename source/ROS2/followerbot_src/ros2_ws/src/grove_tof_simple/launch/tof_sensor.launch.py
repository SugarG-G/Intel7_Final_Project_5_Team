#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grove_tof_simple',
            executable='tof_node',
            name='tof_sensor',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'publish_rate': 10.0,
                'frame_id': 'tof_link'
            }],
            output='screen'
        )
    ])