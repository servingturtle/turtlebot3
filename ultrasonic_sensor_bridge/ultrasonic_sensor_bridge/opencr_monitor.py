#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlebot3_msgs.msg import SensorState
import time


class OpenCRMonitor(Node):
    def __init__(self):
        super().__init__('opencr_monitor')
        
        # OpenCR의 sensor_state 토픽 구독
        self.sensor_sub = self.create_subscription(
            SensorState, '/sensor_state', self.sensor_callback, 10)
        
        # 타이머 설정 (1초마다 출력)
        self.timer = self.create_timer(1.0, self.print_values)
        
        # 센서 값 저장
        self.sonar_value = 0.0
        self.ultrasonic_left = 0.0
        self.ultrasonic_front = 0.0
        self.ultrasonic_right = 0.0
        
        # 마지막 업데이트 시간
        self.last_update = 0
        
        self.get_logger().info('OpenCR Monitor started')
    
    def sensor_callback(self, msg):
        # OpenCR의 sensor_state에서 데이터 읽기
        self.sonar_value = msg.sonar
        # OpenCR의 제어 테이블에서 초음파 센서 값들
        # 이 값들은 turtlebot3_node에서 읽어와야 함
        self.last_update = time.time()
    
    def print_values(self):
        current_time = time.time()
        
        # 데이터 수신 상태 확인
        status = "OK" if (current_time - self.last_update) < 2.0 else "NO DATA"
        
        print(f"\n=== OpenCR Sensor Values ===")
        print(f"Sonar:           {self.sonar_value:.3f}m [{status}]")
        print(f"Time:            {time.strftime('%H:%M:%S')}")
        print("=" * 35)
        
        # 추가 정보 출력
        if status == "OK":
            print(f"Note: Sonar value from OpenCR control table")
            print(f"      Check if this value changes when moving objects")
        else:
            print(f"Warning: No data received from OpenCR")


def main(args=None):
    rclpy.init(args=args)
    
    monitor = OpenCRMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
