#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cafe_gui',
            executable='gui_node',
            name='cafe_gui_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])