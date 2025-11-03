from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    leader_server = Node(
        package='turtlebot3_patrol',
        executable='turtlebot3_patrol_server',
        namespace='tb3_1',
        output='screen'
    )

    follower_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='turtlebot3_patrol',
                executable='turtlebot3_patrol_server',
                namespace='tb3_2',
                output='screen',
                remappings=[('patrol/stop', '/tb3_1/patrol/stop')]
            )
        ]
    )

    return LaunchDescription([
        leader_server,
        follower_server
    ])
