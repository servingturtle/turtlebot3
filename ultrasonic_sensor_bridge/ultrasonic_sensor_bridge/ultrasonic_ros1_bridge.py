#!/usr/bin/env python3
import os
import time
import argparse
import threading
from typing import Optional
os.environ["ROS_DOMAIN_ID"] = "10"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header

# ROS1 관련 import (별도 설치 필요)
try:
    import rospy
    from sensor_msgs.msg import Range as ROS1Range
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    print("경고: ROS1이 설치되지 않았습니다. 시뮬레이션 모드로 실행됩니다.")


class UltrasonicROS1Bridge(Node):
    def __init__(self, simulation_mode=False):
        super().__init__("ultrasonic_ros1_bridge")
        
        # ROS2 퍼블리셔 생성
        self.ul_pub_left = self.create_publisher(Range, "/ultrasonic/left", 10)
        self.ul_pub_front = self.create_publisher(Range, "/ultrasonic/front", 10)
        self.ul_pub_right = self.create_publisher(Range, "/ultrasonic/right", 10)
        
        # 센서 설정
        self.min_range = 0.02  # 2cm
        self.max_range = 4.0   # 4m
        self.field_of_view = 0.1  # 약 5.7도
        
        self.simulation_mode = simulation_mode or not ROS1_AVAILABLE
        
        if self.simulation_mode:
            # 시뮬레이션 모드
            self.create_timer(0.1, self.publish_simulated_data)
            self.get_logger().info("시뮬레이션 모드로 초음파 센서 브리지가 시작되었습니다.")
        else:
            # ROS1 브리지 모드
            self.setup_ros1_bridge()
            self.get_logger().info("ROS1 브리지 모드로 초음파 센서 브리지가 시작되었습니다.")

    def setup_ros1_bridge(self):
        """ROS1 브리지 설정"""
        try:
            # ROS1 노드 초기화
            rospy.init_node('ultrasonic_bridge', anonymous=True)
            
            # ROS1 서브스크라이버 생성
            rospy.Subscriber("/ultrasonic/left", ROS1Range, self.left_callback)
            rospy.Subscriber("/ultrasonic/front", ROS1Range, self.front_callback)
            rospy.Subscriber("/ultrasonic/right", ROS1Range, self.right_callback)
            
            # ROS1 스핀을 별도 스레드에서 실행
            self.ros1_thread = threading.Thread(target=self.ros1_spin, daemon=True)
            self.ros1_thread.start()
            
            self.get_logger().info("ROS1 토픽을 구독하고 ROS2로 변환하여 발행합니다.")
            
        except Exception as e:
            self.get_logger().error(f"ROS1 브리지 설정 실패: {e}")
            self.simulation_mode = True
            self.create_timer(0.1, self.publish_simulated_data)

    def left_callback(self, msg: ROS1Range):
        """왼쪽 초음파 센서 데이터 콜백"""
        ros2_msg = self.convert_ros1_to_ros2(msg, "ultrasonic_left")
        self.ul_pub_left.publish(ros2_msg)
        self.get_logger().debug(f"왼쪽 센서: {msg.range:.3f}m")

    def front_callback(self, msg: ROS1Range):
        """앞쪽 초음파 센서 데이터 콜백"""
        ros2_msg = self.convert_ros1_to_ros2(msg, "ultrasonic_front")
        self.ul_pub_front.publish(ros2_msg)
        self.get_logger().debug(f"앞쪽 센서: {msg.range:.3f}m")

    def right_callback(self, msg: ROS1Range):
        """오른쪽 초음파 센서 데이터 콜백"""
        ros2_msg = self.convert_ros1_to_ros2(msg, "ultrasonic_right")
        self.ul_pub_right.publish(ros2_msg)
        self.get_logger().debug(f"오른쪽 센서: {msg.range:.3f}m")

    def convert_ros1_to_ros2(self, ros1_msg: ROS1Range, frame_id: str) -> Range:
        """ROS1 Range 메시지를 ROS2 Range 메시지로 변환"""
        ros2_msg = Range()
        ros2_msg.header = Header()
        ros2_msg.header.stamp = self.get_clock().now().to_msg()
        ros2_msg.header.frame_id = frame_id
        
        ros2_msg.radiation_type = ros1_msg.radiation_type
        ros2_msg.field_of_view = ros1_msg.field_of_view
        ros2_msg.min_range = ros1_msg.min_range
        ros2_msg.max_range = ros1_msg.max_range
        ros2_msg.range = ros1_msg.range
        
        return ros2_msg

    def ros1_spin(self):
        """ROS1 스핀 함수"""
        try:
            rospy.spin()
        except Exception as e:
            self.get_logger().error(f"ROS1 스핀 오류: {e}")

    def publish_simulated_data(self):
        """시뮬레이션된 초음파 센서 데이터를 발행"""
        current_time = self.get_clock().now()
        
        # 왼쪽 센서 데이터
        left_msg = self.create_range_message(
            current_time, 
            "ultrasonic_left", 
            1.5  # 시뮬레이션된 거리 (1.5m)
        )
        self.ul_pub_left.publish(left_msg)
        
        # 앞쪽 센서 데이터
        front_msg = self.create_range_message(
            current_time, 
            "ultrasonic_front", 
            2.0  # 시뮬레이션된 거리 (2.0m)
        )
        self.ul_pub_front.publish(front_msg)
        
        # 오른쪽 센서 데이터
        right_msg = self.create_range_message(
            current_time, 
            "ultrasonic_right", 
            1.8  # 시뮬레이션된 거리 (1.8m)
        )
        self.ul_pub_right.publish(right_msg)
        
        self.get_logger().debug(f"시뮬레이션 데이터 발행: L={left_msg.range:.2f}m, F={front_msg.range:.2f}m, R={right_msg.range:.2f}m")

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
        if not self.simulation_mode and ROS1_AVAILABLE:
            try:
                rospy.signal_shutdown("ROS2 노드 종료")
            except Exception:
                pass
        self.get_logger().info("초음파 센서 브리지를 종료합니다.")
        super().destroy_node()


def parse_args():
    ap = argparse.ArgumentParser(description="ROS1 초음파 센서 데이터를 ROS2로 변환하는 브리지")
    ap.add_argument("--simulation", action="store_true", 
                   help="시뮬레이션 모드로 실행 (ROS1 없이)")
    ap.add_argument("--frequency", type=float, default=10.0, 
                   help="센서 데이터 발행 주파수 (Hz, 기본값: 10)")
    return ap.parse_args()


def main():
    rclpy.init()
    args = parse_args()
    
    node = UltrasonicROS1Bridge(simulation_mode=args.simulation)
    
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
