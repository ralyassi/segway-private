# segwayrmp_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='segwayrmp',
            executable='SmartCar',
            name='SmartCar',
            output='screen'
        ),
        Node(
            package='segwayrmp',
            executable='ChassisResponseTest',
            name='ChassisResponseTest',
            output='screen'
        )
    ])
