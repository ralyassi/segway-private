from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Path to the robot model (URDF file)'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable joint_state_publisher GUI'
    )

    robot_description_path = os.path.join(
        get_package_share_directory('segwayrmp_description'),
        'urdf',
        'segway_e1.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('segwayrmp_description'),
        'urdf.rviz'
    )

    # Load URDF as a string for robot_description parameter
    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        model_arg,
        gui_arg,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',  # or 'joint_state_publisher_gui' if GUI is enabled
            name='joint_state_publisher',
            parameters=[{'use_gui': LaunchConfiguration('gui')}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])

