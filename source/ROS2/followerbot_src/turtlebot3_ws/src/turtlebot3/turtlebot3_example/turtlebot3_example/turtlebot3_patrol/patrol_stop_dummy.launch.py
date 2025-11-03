from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_example',
            executable='turtlebot3_patrol_stop_dummy',
            name='turtlebot3_patrol_stop_dummy',
            output='screen',
            parameters=[{
                'laser_topic': '/tb3_2/vl53_front_range',
                'laser_topic_type': 'range',
                'auto_sequence_enabled': True,
                'auto_stop_min': 0.17,
                'auto_stop_max': 0.20,
                'arm_service_name': '/robot_arm_control',
                'arm_command': 'detect',
                'arm_call_timeout': 60.0,
                'auto_pre_task_delay': 2.0,
                'auto_post_task_delay': 2.0,
                # follower trigger params
                'follower_trigger_enabled': True,
                # when: 'on_detection' | 'on_task_start' | 'on_task_done'
                'follower_trigger_when': 'on_detection',
                'follower_trigger_topic': '/tb3_2/follow/trigger',
                'follower_trigger_latch': True,
            }]
        )
    ])
