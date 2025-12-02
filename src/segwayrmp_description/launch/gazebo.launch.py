from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([

        # 1. Launch Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Static Transform Publisher (footprint -> base_link), since footprint is needed by some nav pkgs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        # 3. Spawn robot model into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=[
                '-file', os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share/segwayrmp_description/urdf/segway_e1.urdf'),
                '-entity', 'segway_e1_description'
            ],
            output='screen'
        ),

        # 4. Publish "calibrated" topic once (ros2 topic pub)
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/Bool', 'data:true', '--once'],
            output='screen'
        ),
    ])

