#!/usr/bin/env python3
import os
import time
import argparse
from typing import Optional
os.environ["ROS_DOMAIN_ID"] = "10"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from turtlebot3_msgs.msg import SensorState


class SonarToRangeConverter(Node):
    def __init__(self):
        super().__init__("sonar_to_range_converter")
        
        # ROS2 퍼블리셔 생성
        self.ul_pub_left = self.create_publisher(Range, "/ultrasonic/left", 10)
        self.ul_pub_front = self.create_publisher(Range, "/ultrasonic/front", 10)
        self.ul_pub_right = self.create_publisher(Range, "/ultrasonic/right", 10)
        
        # turtlebot3_msgs/SensorState 서브스크라이버
        self.sensor_state_sub = self.create_subscription(
            SensorState,
            "/sensor_state",
            self.sensor_state_callback,
            10
        )
        
        # 센서 설정
        self.min_range = 0.02  # 2cm
        self.max_range = 4.0   # 4m
        self.field_of_view = 0.1  # 약 5.7도
        
        # 시뮬레이션된 센서 값 (실제로는 OpenCR에서 개별 센서 데이터를 받아야 함)
        self.left_distance = 1.5
        self.front_distance = 2.0
        self.right_distance = 1.8
        
        self.get_logger().info("Sonar to Range Converter가 시작되었습니다.")
        self.get_logger().info("turtlebot3_msgs/SensorState의 3개 초음파 센서 데이터를 sensor_msgs/Range로 변환합니다.")

    def sensor_state_callback(self, msg: SensorState):
        """SensorState 메시지 콜백"""
        current_time = self.get_clock().now()
        
        # 3개 초음파 센서 데이터 사용
        left_distance = msg.ultrasonic_left if hasattr(msg, 'ultrasonic_left') and msg.ultrasonic_left > 0 else self.left_distance
        front_distance = msg.ultrasonic_front if hasattr(msg, 'ultrasonic_front') and msg.ultrasonic_front > 0 else self.front_distance
        right_distance = msg.ultrasonic_right if hasattr(msg, 'ultrasonic_right') and msg.ultrasonic_right > 0 else self.right_distance
        
        # 왼쪽 센서 데이터
        left_msg = self.create_range_message(
            current_time, 
            "ultrasonic_left", 
            left_distance
        )
        self.ul_pub_left.publish(left_msg)
        
        # 앞쪽 센서 데이터
        front_msg = self.create_range_message(
            current_time, 
            "ultrasonic_front", 
            front_distance
        )
        self.ul_pub_front.publish(front_msg)
        
        # 오른쪽 센서 데이터
        right_msg = self.create_range_message(
            current_time, 
            "ultrasonic_right", 
            right_distance
        )
        self.ul_pub_right.publish(right_msg)
        
        self.get_logger().debug(f"Ultrasonic 변환: L={left_msg.range:.2f}m, F={front_msg.range:.2f}m, R={right_msg.range:.2f}m")

    def create_range_message(self, timestamp, frame_id: str, distance: float) -> Range:
        """Range 메시지 생성"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = frame_id
        
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        
        return msg

    def destroy_node(self):
        """노드 정리"""
        self.get_logger().info("Sonar to Range Converter를 종료합니다.")
        super().destroy_node()


def parse_args():
    ap = argparse.ArgumentParser(description="turtlebot3_msgs/SensorState의 sonar 데이터를 sensor_msgs/Range로 변환")
    return ap.parse_args()


def main():
    rclpy.init()
    node = SonarToRangeConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단되었습니다.")
    except Exception as e:
        node.get_logger().error(f"예상치 못한 오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
