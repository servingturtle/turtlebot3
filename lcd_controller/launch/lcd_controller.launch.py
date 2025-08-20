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
            'i2c_address',
            default_value='0x27',
            description='I2C address (hex)'
        ),
        DeclareLaunchArgument(
            'i2c_port',
            default_value='1',
            description='I2C port'
        ),
        
        # LCD Controller Node
        Node(
            package='lcd_controller',
            executable='lcd_controller',
            name='lcd_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'i2c_port': LaunchConfiguration('i2c_port'),
            }],
            arguments=[
                '--i2c-address', LaunchConfiguration('i2c_address'),
                '--i2c-port', LaunchConfiguration('i2c_port'),
            ],
            remappings=[
                # 토픽 리매핑 (필요시)
            ],
        ),
    ])
