from setuptools import setup

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follow.launch.py']),
        ('share/' + package_name + '/config', [
            'config/wall_follow_params.yaml',
            'config/waypoints.yaml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intel7 Team',
    maintainer_email='user@example.com',
    description='Wall following controller and terminal interface for TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wall_follow_controller = wall_follower.wall_follow_controller:main',
            'wall_follow_terminal = wall_follower.wall_follow_terminal:main',
            'waypoint_runner = wall_follower.waypoint_runner:main',
            'simple_waypoint_runner = wall_follower.simple_waypoint_runner:main',
            'smooth_waypoint_runner = wall_follower.smooth_waypoint_runner:main'
        ],
    },
)
