#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # TurtleBot3 Robot Node (센서 데이터 소스)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
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
