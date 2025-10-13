from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_applications_msgs',
    version='1.0.1',
    packages=find_packages(
        include=('turtlebot3_applications_msgs', 'turtlebot3_applications_msgs.*')),
)
