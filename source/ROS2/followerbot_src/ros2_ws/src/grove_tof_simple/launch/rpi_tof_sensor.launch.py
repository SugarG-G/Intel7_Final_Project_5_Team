#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 설정 파일 경로
    config_file = os.path.join(
        get_package_share_directory('grove_tof_simple'),
        'config',
        'rpi_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='grove_tof_simple',
            executable='tof_node',
            name='tof_sensor_rpi',
            parameters=[config_file],
            output='screen'
        )
    ])