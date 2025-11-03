#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    namespace = LaunchConfiguration('namespace', default='')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('coin_d4_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    setup_addresses = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            '/home/ubuntu/turtlebot3_ws/tools/vl53_i2c/setup_vl53_addresses.py'
        ],
        output='screen'
    )

    # VL53 퍼블리셔를 각 네임스페이스 하위로 퍼블리시하도록 상대 토픽을 사용합니다.
    # (PushRosNamespace(namespace)로 네임스페이스가 적용되므로 여기서 namespace 중복 지정은 제거합니다.)
    vl53_node = Node(
        package='vl53l0x_distance_cpp',
        executable='dual_vl53l0x_node',
        name='dual_vl53l0x_node',
        parameters=[{
            'front_address': 0x2A,
            'rear_address': 0x2B,
            'front_topic': 'vl53_front_range',
            'rear_topic': 'vl53_rear_range',
            'range_min': 0.001,
            'range_max': 4.0
        }],
        output='screen'
    )

    actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        PushRosNamespace(namespace),
        setup_addresses,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': namespace}],
            arguments=['-i', usb_port],
            output='screen'),
        RegisterEventHandler(
            OnProcessExit(
                target_action=setup_addresses,
                on_exit=[vl53_node]
            )
        ),
    ]

    return LaunchDescription(actions)
