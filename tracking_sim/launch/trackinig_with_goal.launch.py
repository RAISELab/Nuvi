from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracking_sim',
            executable='tracking_sim',
            name='tracking_sim'
        ),
        Node(
            package='tracking_sim',
            executable='goal_navigator',
            name='goal_navigator'
        )
    ])
