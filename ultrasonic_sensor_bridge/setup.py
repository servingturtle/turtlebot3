from setuptools import setup

package_name = 'ultrasonic_sensor_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ultrasonic_sensor_bridge.launch.py',
            'launch/ultrasonic_ros1_bridge.launch.py',
            'launch/sonar_to_range_converter.launch.py',
            'launch/slam_navigation.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Ultrasonic sensor bridge for TurtleBot3 - converts sensor data to ROS2 Range messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_sensor_bridge = ultrasonic_sensor_bridge.ultrasonic_sensor_bridge:main',
            'ultrasonic_ros1_bridge = ultrasonic_sensor_bridge.ultrasonic_ros1_bridge:main',
            'sonar_to_range_converter = ultrasonic_sensor_bridge.sonar_to_range_converter:main',
        ],
    },
)
