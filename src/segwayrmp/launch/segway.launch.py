from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('segwayrmp_description'),
        'urdf',
        'segway_e1.urdf'
    )
    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()
        
    return LaunchDescription([
        Node(
            package='segwayrmp',
            executable='SmartCar',
            name='SmartCar',
            output='screen'
        ),
        #Node(
        #    package='segwayrmp',
        #    executable='RobotOdoemtry',
        #    name='RobotOdoemtry',
        #    output='screen'
        #)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,         
            		 'use_sim_time': False
			}],
            output='screen'
        ),
         Node(
             package='joint_state_publisher',
             executable='joint_state_publisher',
             name='joint_state_publisher',
             output='screen'
         ),
          Node(
             package='cmd_vel_tools',
             executable='throttle_node',
             name='throttle_node',
             output='screen'
         ),

    ])
