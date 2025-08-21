#!/usr/bin/env python3
from RPLCD.i2c import CharLCD

import rclpy
from rclpy.node import Node
from std_msgs.srv import String as StringSrv, Empty as EmptySrv, Bool as BoolSrv, UInt8

class LCDController(Node):
    def __init__(self, lcd):
        super().__init__('lcd_controller')
        
        # LCD 객체 저장
        self.lcd = lcd
        
        # 기본 서비스 생성
        self.display_srv = self.create_service(StringSrv, '/lcd/display', self.display_callback)
        self.clear_srv = self.create_service(EmptySrv, '/lcd/clear', self.clear_callback)
        self.backlight_srv = self.create_service(BoolSrv, '/lcd/backlight', self.backlight_callback)
        
        # 고급 서비스 생성
        self.set_cursor_srv = self.create_service(StringSrv, '/lcd/set_cursor', self.set_cursor_callback)
        self.write_srv = self.create_service(StringSrv, '/lcd/write', self.write_callback)
        self.clear_line_srv = self.create_service(UInt8, '/lcd/clear_line', self.clear_line_callback)
        self.cursor_mode_srv = self.create_service(StringSrv, '/lcd/cursor_mode', self.cursor_mode_callback)
        
        self.get_logger().info('LCD Controller started')
        
    def display_callback(self, request, response):
        """전체 화면 텍스트 표시 서비스"""
        try:
            self.lcd.clear()
            self.lcd.write_string(request.data)
            self.get_logger().info(f"Display: {request.data}")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Display error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def clear_callback(self, request, response):
        """전체 화면 지우기 서비스"""
        try:
            self.lcd.clear()
            self.get_logger().info("LCD cleared")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Clear error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def backlight_callback(self, request, response):
        """백라이트 제어 서비스"""
        try:
            self.lcd.backlight_enabled = request.data
            self.get_logger().info(f"Backlight: {'ON' if request.data else 'OFF'}")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Backlight error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def set_cursor_callback(self, request, response):
        """커서 위치 설정 서비스 (형식: "row,col")"""
        try:
            # "row,col" 형식 파싱
            parts = request.data.split(',')
            if len(parts) != 2:
                response.data = "Error: Invalid format. Use 'row,col'"
                return response
            
            row = int(parts[0])
            col = int(parts[1])
            self.lcd.cursor_pos = (row, col)
            self.get_logger().info(f"Cursor set to ({row},{col})")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Set cursor error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def write_callback(self, request, response):
        """현재 커서 위치에 텍스트 쓰기 서비스"""
        try:
            self.lcd.write_string(request.data)
            self.get_logger().info(f"Write: {request.data}")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def clear_line_callback(self, request, response):
        """특정 줄 지우기 서비스"""
        try:
            row = request.data
            self.lcd.cursor_pos = (row, 0)
            self.lcd.write_string(' ' * 16)  # 16자 공백으로 덮어쓰기
            self.get_logger().info(f"Line {row} cleared")
            response.data = "Success"
        except Exception as e:
            self.get_logger().error(f"Clear line error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def cursor_mode_callback(self, request, response):
        """커서 모드 설정 서비스 (hide/visible/blink)"""
        try:
            mode = request.data
            if mode in ['hide', 'visible', 'blink']:
                self.lcd.cursor_mode = mode
                self.get_logger().info(f"Cursor mode: {mode}")
                response.data = "Success"
            else:
                response.data = "Error: Invalid mode. Use 'hide', 'visible', or 'blink'"
        except Exception as e:
            self.get_logger().error(f"Cursor mode error: {e}")
            response.data = f"Error: {e}"
        return response
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        try:
            self.lcd.clear()
            self.lcd.close()
            self.get_logger().info("LCD hardware cleaned up")
        except Exception as e:
            self.get_logger().error(f"LCD cleanup error: {e}")
        
        self.get_logger().info("LCD Controller shutting down")
        super().destroy_node()

def main(args=None):
    # LCD 초기화
    try:
        lcd = CharLCD('PCF8574', 0x27)
        lcd.clear()
        lcd.write_string("LCD Ready")
        print("LCD initialized successfully")
    except Exception as e:
        print(f"Failed to initialize LCD: {e}")
        return
    
    rclpy.init(args=args)
    
    # 노드 생성
    lcd_controller = LCDController(lcd)
    
    try:
        rclpy.spin(lcd_controller)
    except KeyboardInterrupt:
        pass
    finally:
        lcd_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
