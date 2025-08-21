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
        
        # TurtleBot3 Robot Node (센서 데이터 소스)
        Node(
            package='turtlebot3_bringup',
            executable='turtlebot3_robot',
            name='turtlebot3_robot',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        
        # Sonar to Range Converter Node
        Node(
            package='ultrasonic_sensor_bridge',
            executable='sonar_to_range_converter',
            name='sonar_to_range_converter',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                # 토픽 리매핑 (필요시)
                ('/sensor_state', '/sensor_state'),
                ('/ultrasonic/left', '/ultrasonic/left'),
                ('/ultrasonic/front', '/ultrasonic/front'),
                ('/ultrasonic/right', '/ultrasonic/right'),
            ],
        ),
    ])
