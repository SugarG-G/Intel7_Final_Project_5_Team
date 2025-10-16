from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('wall_follower')
    params_default = PathJoinSubstitution([pkg_share, 'config', 'wall_follow_params.yaml'])

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/map.yaml',
        description='사용할 지도 YAML 파일 경로'
    )
    params_arg = DeclareLaunchArgument(
        'params',
        default_value=params_default,
        description='wall_follow_controller 파라미터 YAML'
    )

    controller_node = Node(
        package='wall_follower',
        executable='wall_follow_controller',
        name='wall_follow_controller',
        output='screen',
        parameters=[LaunchConfiguration('params')]
    )

    terminal_node = Node(
        package='wall_follower',
        executable='wall_follow_terminal',
        name='wall_follow_terminal',
        output='screen'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='static_map',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        map_server_node,
        controller_node,
        terminal_node
    ])
