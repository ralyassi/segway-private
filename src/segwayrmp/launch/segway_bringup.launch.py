from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define arguments (default: false means don’t start it)
    launch_pkg2_arg = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='Launch livox_ros_driver2/MID360_launch.py'
    )
    launch_pkg3_arg = DeclareLaunchArgument(
        'zed', default_value='true',
        description='Launch zed_wrapper/zed_camera.launch.py'
    )
    launch_pkg4_arg = DeclareLaunchArgument(
        'localization', default_value='true',
        description='Launch segway_slam_toolbox/localization_launch.py'
    )


    # Paths
    pkg1_launch = os.path.join(get_package_share_directory('segwayrmp'), 'launch', 'segway.launch.py')  # SmartCar + state/joint pub
    pkg2_launch = os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'MID360_launch.py')  # Lidar
    pkg3_launch = os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')  # zed
    pkg4_launch = os.path.join(get_package_share_directory('segway_slam_toolbox'), 'launch', 'localization_launch.py')  # localization

    # Conditional includes
    include_pkg1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg1_launch),
    )
    include_pkg2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg2_launch),
        condition=IfCondition(LaunchConfiguration('lidar'))
    )
    include_pkg3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg3_launch),
        condition=IfCondition(LaunchConfiguration('zed'))
    )
    include_pkg4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg4_launch),
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    return LaunchDescription([
        launch_pkg2_arg,
        launch_pkg3_arg,
        launch_pkg4_arg,
        include_pkg1,
        include_pkg2,
        include_pkg3,
        include_pkg4,
    ])

