#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    namespace = LaunchConfiguration('namespace', default='')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('coin_d4_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # RFID parameters
    rfid_bus = LaunchConfiguration('rfid_bus', default='0')
    rfid_device = LaunchConfiguration('rfid_device', default='0')
    rfid_hold_ms = LaunchConfiguration('rfid_hold_ms', default='80')
    rfid_cooldown = LaunchConfiguration('rfid_cooldown', default='1.0')
    rfid_grace_ms = LaunchConfiguration('rfid_grace_ms', default='200')
    rfid_whitelist = LaunchConfiguration('rfid_whitelist', default='')
    rfid_rst_bcm = LaunchConfiguration('rfid_rst_bcm', default='24')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        # RFID parameters
        DeclareLaunchArgument(
            'rfid_bus',
            default_value=rfid_bus,
            description='SPI bus number for RFID reader'),

        DeclareLaunchArgument(
            'rfid_device',
            default_value=rfid_device,
            description='SPI device number for RFID reader'),

        DeclareLaunchArgument(
            'rfid_hold_ms',
            default_value=rfid_hold_ms,
            description='Hold time in milliseconds for RFID'),

        DeclareLaunchArgument(
            'rfid_cooldown',
            default_value=rfid_cooldown,
            description='Cooldown time in seconds for RFID'),

        DeclareLaunchArgument(
            'rfid_grace_ms',
            default_value=rfid_grace_ms,
            description='Grace period in milliseconds for RFID'),

        DeclareLaunchArgument(
            'rfid_whitelist',
            default_value=rfid_whitelist,
            description='Comma-separated list of whitelisted RFID UIDs'),

        DeclareLaunchArgument(
            'rfid_rst_bcm',
            default_value=rfid_rst_bcm,
            description='BCM line number for RFID reset pin'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': namespace}],
            arguments=['-i', usb_port],
            output='screen'),

        # RFID Tag Publisher Node
        Node(
            package='rfid_tag_publisher',
            executable='rfid_tag_publisher',
            name='rfid_tag_publisher',
            parameters=[{
                'bus': rfid_bus,
                'device': rfid_device,
                'hold_ms': rfid_hold_ms,
                'cooldown': rfid_cooldown,
                'grace_ms': rfid_grace_ms,
                'whitelist': rfid_whitelist,
                'rst_bcm': rfid_rst_bcm,
            }],
            output='screen'),
    ])
