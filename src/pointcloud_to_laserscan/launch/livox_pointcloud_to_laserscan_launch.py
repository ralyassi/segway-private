from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(
    		package='pointcloud_to_laserscan',
    		executable='pointcloud_to_laserscan_node',
    		name='pc2_to_scan',
   		 output='screen',
    		remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
	    parameters=[{
		'target_frame': 'lidar_link',
		'transform_tolerance': 0.01,
		'min_height': -0.20,
		'max_height': 0.30,
		'angle_min': -3.14159,
		'angle_max':  3.14159,
		'angle_increment': 0.0087,
		'scan_time': 0.1,
		'range_min': 0.2,
		'range_max': 20.0,
		'use_inf': False,
		'inf_epsilon': 1.0
	    }]
	)
    ])

