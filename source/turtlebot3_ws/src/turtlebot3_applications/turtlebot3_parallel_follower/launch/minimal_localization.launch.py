import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    map_file_path = LaunchConfiguration('map_file_path')
    leader_namespace = LaunchConfiguration('leader_namespace')
    follower_namespace = LaunchConfiguration('follower_namespace')

    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path',
        default_value=os.path.join(os.getcwd(), 'cleaned_up_map.yaml'), # Assuming cleaned_up_map.yaml is in the workspace root
        description='Full path to map yaml file to load')

    declare_leader_namespace_cmd = DeclareLaunchArgument(
        'leader_namespace',
        default_value='tb3_1',
        description='Namespace for the leader robot')

    declare_follower_namespace_cmd = DeclareLaunchArgument(
        'follower_namespace',
        default_value='tb3_2',
        description='Namespace for the follower robot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Get AMCL parameters file
    amcl_params_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_parallel_follower'), 'param', 'amcl_params.yaml']
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}, {'use_sim_time': use_sim_time}],
    )

    # AMCL for Leader
    leader_amcl_group = GroupAction(
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                namespace=leader_namespace,
                parameters=[amcl_params_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/scan', 'scan'),
                    ('/odom', 'odom'),
                    ('/initialpose', 'initialpose'),
                    ('/amcl_pose', 'amcl_pose'),
                ]
            ),
        ]
    )

    # AMCL for Follower
    follower_amcl_group = GroupAction(
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                namespace=follower_namespace,
                parameters=[amcl_params_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/scan', 'scan'),
                    ('/odom', 'odom'),
                    ('/initialpose', 'initialpose'),
                    ('/amcl_pose', 'amcl_pose'),
                ]
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_leader_namespace_cmd)
    ld.add_action(declare_follower_namespace_cmd)

    ld.add_action(map_server_node)
    ld.add_action(leader_amcl_group)
    ld.add_action(follower_amcl_group)

    return ld
