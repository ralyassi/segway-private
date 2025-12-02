# robot_pose_ekf_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_pose_ekf',
            executable='robot_pose_ekf',
            name='robot_pose_ekf',
            parameters=[{
                'output_frame': 'odom',
                'freq': 30.0,
                'sensor_timeout': 1.0,
                'odom_used': True,
                'imu_used': True,
                'vo_used': False,
                'debug': False,
                'self_diagnose': False
            }]
        )
    ])
