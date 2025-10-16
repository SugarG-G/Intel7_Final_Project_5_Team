from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    core_params = os.path.join(
        get_package_share_directory('sub_robot_bringup'),
        'config',
        'controller_params.yaml')

    sim_params = os.path.join(
        get_package_share_directory('sub_robot_bringup'),
        'config',
        'mock_main_odom.yaml')

    controller_param_arg = DeclareLaunchArgument(
        'controller_params',
        default_value=core_params,
        description='Path to controller parameter file')

    sim_param_arg = DeclareLaunchArgument(
        'sim_params',
        default_value=sim_params,
        description='Path to mock odom parameter file')

    controller_node = Node(
        package='sub_robot_core',
        executable='sub_controller_node',
        name='sub_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('controller_params')]
    )

    mock_odom_node = Node(
        package='sub_robot_sim',
        executable='mock_main_odom',
        name='mock_main_odom',
        output='screen',
        parameters=[LaunchConfiguration('sim_params')]
    )

    return LaunchDescription([
        controller_param_arg,
        sim_param_arg,
        controller_node,
        mock_odom_node,
    ])
