#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	use_rviz = LaunchConfiguration('use_rviz', default='true')  

	rviz_config_dir = os.path.join(
		get_package_share_directory('cartographer'),
		'rviz', 'cartographer.rviz')

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='true',
			description='Use simulation (Gazebo) clock if true'),

		DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),

		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', rviz_config_dir],
			parameters=[{'use_sim_time': use_sim_time}],
			output='screen',
			condition=IfCondition(use_rviz)
		),
	])