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
            'lcd_width',
            default_value='16',
            description='LCD character width'
        ),
        DeclareLaunchArgument(
            'lcd_height',
            default_value='2',
            description='LCD line height'
        ),
        DeclareLaunchArgument(
            'scroll_enabled',
            default_value='true',
            description='Enable text scrolling'
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
        DeclareLaunchArgument(
            'scroll_delay',
            default_value='0.5',
            description='Scroll delay (seconds)'
        ),
        
        # LCD Display Node
        Node(
            package='lcd_controller',
            executable='lcd_display',
            name='lcd_display',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'lcd_width': LaunchConfiguration('lcd_width'),
                'lcd_height': LaunchConfiguration('lcd_height'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'i2c_port': LaunchConfiguration('i2c_port'),
                'scroll_enabled': LaunchConfiguration('scroll_enabled'),
                'scroll_delay': LaunchConfiguration('scroll_delay'),
            }],
            arguments=[
                '--width', LaunchConfiguration('lcd_width'),
                '--height', LaunchConfiguration('lcd_height'),
                '--i2c-address', LaunchConfiguration('i2c_address'),
                '--i2c-port', LaunchConfiguration('i2c_port'),
                '--scroll-delay', LaunchConfiguration('scroll_delay'),
            ] + (['--scroll'] if LaunchConfiguration('scroll_enabled') == 'true' else []),
            remappings=[
                # 토픽 리매핑 (필요시)
                ('/lcd/display', '/lcd/display'),
                ('/lcd/clear', '/lcd/clear'),
                ('/lcd/backlight', '/lcd/backlight'),
            ],
        ),
    ])
