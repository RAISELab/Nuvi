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

	tetra_prefix = get_package_share_directory('tetra')
	tetra_description_prefix = get_package_share_directory('tetra_description')
	sllidar_prefix = get_package_share_directory('sllidar_ros2')

	
	return LaunchDescription([
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


