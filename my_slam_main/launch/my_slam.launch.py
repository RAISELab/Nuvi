#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	cartographer_prefix = get_package_share_directory('cartographer')
	tetra_prefix = get_package_share_directory('tetra')
	tetra_description_prefix = get_package_share_directory('tetra_description')
	sllidar_prefix = get_package_share_directory('sllidar_ros2')



	resolution = LaunchConfiguration('resolution', default='0.05')
	publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')



	rviz_config_dir = os.path.join(
            get_package_share_directory('my_slam_main'),
            'rviz',
            'slam.rviz')
 
	
	return LaunchDescription([
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([cartographer_prefix, '/launch','/cartographer.launch.py']),
			launch_arguments={'use_rviz': 'true', 'rviz_config_dir': rviz_config_dir}.items() 
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([tetra_prefix,'/launch','/tetra_configuration.launch.py']),
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([tetra_description_prefix,'/launch','/tetra_description.launch.py']),
			launch_arguments={'use_rviz': 'false'}.items()
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([sllidar_prefix, '/launch','/view_sllidar_a2m12_launch.py']),
			launch_arguments={'use_rviz': 'false'}.items()
    	)
	])
