# LCD Controller

TurtleBot3용 LCD 컨트롤러 패키지입니다. ROS2 토픽으로 받은 텍스트를 LCD에 바로 표시합니다.

## 기능

- **텍스트 표시**: 토픽으로 받은 텍스트를 LCD에 표시
- **화면 지우기**: LCD 화면 전체 지우기
- **백라이트 제어**: 백라이트 켜기/끄기
- **실시간 제어**: ROS 토픽으로 즉시 LCD 제어

## 설치

### 의존성 설치
```bash
# RPLCD 라이브러리 설치
pip3 install RPLCD

# I2C 도구 설치
sudo apt install python3-smbus i2c-tools

# I2C 활성화
sudo raspi-config
# Interface Options → I2C → Enable
```

### 패키지 빌드
```bash
colcon build --packages-select lcd_controller
source install/setup.bash
```

## 사용법

### 1. LCD 컨트롤러 실행
```bash
# 기본 실행
ros2 run lcd_controller lcd_controller

# 파라미터 지정
ros2 run lcd_controller lcd_controller --i2c-address 0x27 --i2c-port 1 --width 16 --height 2
```

### 2. Launch 파일 사용
```bash
ros2 launch lcd_controller lcd_controller.launch.py
```

### 3. 테스트 실행
```bash
ros2 run lcd_controller lcd_test
```

## 토픽

### 입력 토픽 (Subscribers)
- `/lcd/display` (std_msgs/String): LCD에 표시할 텍스트
- `/lcd/clear` (std_msgs/Empty): LCD 화면 지우기
- `/lcd/backlight` (std_msgs/Bool): 백라이트 켜기/끄기

### 출력 토픽 (Publishers)
- `/lcd/status` (std_msgs/String): LCD 상태 정보

## 사용 예시

### 1. 텍스트 표시
```bash
ros2 topic pub /lcd/display std_msgs/String "data: 'Hello World!'"
```

### 2. 화면 지우기
```bash
ros2 topic pub /lcd/clear std_msgs/Empty
```

### 3. 백라이트 제어
```bash
# 백라이트 끄기
ros2 topic pub /lcd/backlight std_msgs/Bool "data: false"

# 백라이트 켜기
ros2 topic pub /lcd/backlight std_msgs/Bool "data: true"
```

### 4. 실시간 상태 표시
```bash
# 배터리 상태
ros2 topic pub /lcd/display std_msgs/String "data: 'Battery: 85%'"

# 시스템 상태
ros2 topic pub /lcd/display std_msgs/String "data: 'Status: Ready'"

# 속도 정보
ros2 topic pub /lcd/display std_msgs/String "data: 'Speed: 0.5m/s'"
```

### 5. Python 스크립트에서 사용
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LCDPublisher(Node):
    def __init__(self):
        super().__init__('lcd_publisher')
        self.lcd_pub = self.create_publisher(String, '/lcd/display', 10)
        
    def display_message(self, text):
        msg = String()
        msg.data = text
        self.lcd_pub.publish(msg)

# 사용 예시
lcd_pub = LCDPublisher()
lcd_pub.display_message("Hello from Python!")
```

## 파라미터

- `i2c_address` (int): I2C 주소 (기본값: 0x27)
- `i2c_port` (int): I2C 포트 (기본값: 1)
- `lcd_width` (int): LCD 너비 (기본값: 16)
- `lcd_height` (int): LCD 높이 (기본값: 2)

## I2C 설정 확인

```bash
# I2C 장치 스캔
i2cdetect -y 1

# I2C 활성화 확인
ls /dev/i2c*
```

## 문제 해결

### RPLCD 모듈 없음
```bash
# RPLCD 라이브러리 설치 (필수)
pip3 install RPLCD
```

### I2C 권한 문제
```bash
sudo usermod -a -G i2c $USER
# 재로그인 필요
```

### LCD가 응답하지 않음
1. I2C 주소 확인: `i2cdetect -y 1`
2. 배선 확인
3. 전원 공급 확인

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.
