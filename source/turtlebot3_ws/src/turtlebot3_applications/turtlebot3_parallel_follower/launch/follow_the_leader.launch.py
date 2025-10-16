
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_path = LaunchConfiguration('map', default=os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'))
    
    leader_namespace = 'tb3_1'
    follower_namespace = 'tb3_2'

    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_nav2_pkg_dir = get_package_share_directory('turtlebot3_navigation2')
    follower_pkg_dir = get_package_share_directory('turtlebot3_parallel_follower')
    
    # --- 로봇 기본 실행 (Bringup) ---
    # 각 로봇의 컴퓨터에서 직접 실행하는 경우 이 부분을 주석 처리하세요.
    # robot_bringup_leader = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
    #     ),
    #     launch_arguments={'namespace': leader_namespace, 'use_sim_time': use_sim_time}.items()
    # )
    # robot_bringup_follower = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
    #     ),
    #     launch_arguments={'namespace': follower_namespace, 'use_sim_time': use_sim_time}.items()
    # )
    # ------------------------------------

    # Nav2 전체 스택 실행 (Map Server, AMCL, Controller, Planner 등)
    # 리더 로봇용 Nav2
    nav2_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_nav2_pkg_dir, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_path,
            'namespace': leader_namespace,
            'params_file': os.path.join(turtlebot3_nav2_pkg_dir, 'param', 'waffle.yaml')
        }.items(),
    )

    # 팔로워 로봇용 Nav2
    nav2_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_nav2_pkg_dir, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_path,
            'namespace': follower_namespace,
            'params_file': os.path.join(turtlebot3_nav2_pkg_dir, 'param', 'waffle.yaml')
        }.items(),
    )

    # 팔로워 로직 실행
    follower_logic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_pkg_dir, 'launch', 'parallel_follower.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
            description='Full path to map file to load'),
        
        # --- 로봇 기본 실행 ---
        # 각 로봇에서 직접 실행 시 주석 처리
        # robot_bringup_leader,
        # robot_bringup_follower,
        # --------------------

        nav2_leader,
        nav2_follower,
        follower_logic,
    ])
