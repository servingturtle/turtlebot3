#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            description='Run in simulation mode (without ROS1)'
        ),
        
        # Ultrasonic ROS1 Bridge Node
        Node(
            package='ultrasonic_sensor_bridge',
            executable='ultrasonic_ros1_bridge',
            name='ultrasonic_ros1_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=[
                '--simulation' if LaunchConfiguration('simulation_mode') == 'true' else '',
            ],
            remappings=[
                # ROS2 토픽 리매핑 (필요시)
                ('/ultrasonic/left', '/ultrasonic/left'),
                ('/ultrasonic/front', '/ultrasonic/front'),
                ('/ultrasonic/right', '/ultrasonic/right'),
            ],
        ),
    ])
