#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 환경 변수 설정
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    # 파라미터 파일 경로
    param_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param',
        'humble',
        TURTLEBOT3_MODEL + '.yaml'
    )
    
    # RViz 설정 파일
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
        ),
        
        # SLAM (Cartographer) 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_cartographer'),
                '/launch/cartographer.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        
        # Navigation2 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'map': '',  # SLAM 모드에서는 맵 파일 불필요
            }.items(),
        ),
        
        # RViz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # 초음파 센서 브리지 실행
        Node(
            package='ultrasonic_sensor_bridge',
            executable='sonar_to_range_converter',
            name='sonar_to_range_converter',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'
        ),
    ])
