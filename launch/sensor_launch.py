#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('sensor_communication')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'sensor_params.yaml')
    
    return LaunchDescription([
        # Start mock sensor server first
        Node(
            package='sensor_communication',
            executable='mock_sensor_server',
            name='mock_sensor_server',
            output='screen',
            emulate_tty=True,
        ),
        
        # Wait a bit for server to start, then start sensor node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='sensor_communication',
                    executable='sensor_node',
                    name='sensor_communication_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[config_file],
                ),
            ]
        ),
        
        # Start GUI after sensor node
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='sensor_communication',
                    executable='sensor_gui',
                    name='sensor_gui_node',
                    output='screen',
                    emulate_tty=True,
                ),
            ]
        ),
    ])
