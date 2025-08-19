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
        
        # Ultrasonic Sensor Bridge Node
        Node(
            package='ultrasonic_sensor_bridge',
            executable='ultrasonic_sensor_bridge',
            name='ultrasonic_sensor_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                # ROS2 토픽 리매핑 (필요시)
                ('/ultrasonic/left', '/ultrasonic/left'),
                ('/ultrasonic/front', '/ultrasonic/front'),
                ('/ultrasonic/right', '/ultrasonic/right'),
            ],
        ),
    ])
