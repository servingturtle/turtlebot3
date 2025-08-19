# 초음파 센서 브리지 (Ultrasonic Sensor Bridge)

이 패키지는 TurtleBot3의 초음파 센서 데이터를 ROS2의 `sensor_msgs/Range` 형식으로 변환하는 브리지 노드를 제공합니다.

## 기능

- OpenCR에서 보내는 3개의 초음파 센서 데이터를 ROS2로 변환
- 왼쪽, 앞쪽, 오른쪽 초음파 센서 지원
- ROS1이 없는 환경에서도 시뮬레이션 모드로 동작
- 실시간 데이터 변환 및 발행
- SLAM + Navigation 통합 지원

## 토픽

### Sonar to Range Converter (권장)
- **입력 토픽**: `/sensor_state` (turtlebot3_msgs/SensorState) - turtlebot3_node에서 발행하는 센서 상태
- **출력 토픽**: 
  - `/ultrasonic/left` (sensor_msgs/Range) - 왼쪽 초음파 센서
  - `/ultrasonic/front` (sensor_msgs/Range) - 앞쪽 초음파 센서
  - `/ultrasonic/right` (sensor_msgs/Range) - 오른쪽 초음파 센서

### ROS1 브리지 모드
- **입력 토픽 (ROS1)**:
  - `/ultrasonic/left` (sensor_msgs/Range) - 왼쪽 초음파 센서
  - `/ultrasonic/front` (sensor_msgs/Range) - 앞쪽 초음파 센서  
  - `/ultrasonic/right` (sensor_msgs/Range) - 오른쪽 초음파 센서
- **출력 토픽 (ROS2)**:
  - `/ultrasonic/left` (sensor_msgs/Range) - 왼쪽 초음파 센서
  - `/ultrasonic/front` (sensor_msgs/Range) - 앞쪽 초음파 센서
  - `/ultrasonic/right` (sensor_msgs/Range) - 오른쪽 초음파 센서

## 사용법

### 1. 빌드 및 설치

```bash
# 워크스페이스에서 빌드
colcon build --packages-select ultrasonic_sensor_bridge

# 환경 설정
source install/setup.bash
```

### 2. 실행 방법

#### 방법 1: Launch 파일 사용 (권장)

```bash
# 기존 turtlebot3_node의 sonar 데이터를 Range 메시지로 변환 (권장)
ros2 launch ultrasonic_sensor_bridge sonar_to_range_converter.launch.py

# ROS1 브리지 모드 (ROS1이 설치된 경우)
ros2 launch ultrasonic_sensor_bridge ultrasonic_ros1_bridge.launch.py

# 시뮬레이션 모드 (ROS1 없이)
ros2 launch ultrasonic_sensor_bridge ultrasonic_ros1_bridge.launch.py simulation_mode:=true

# 기본 시뮬레이션 브리지
ros2 launch ultrasonic_sensor_bridge ultrasonic_sensor_bridge.launch.py

# SLAM + Navigation + 초음파 센서 통합 실행
ros2 launch ultrasonic_sensor_bridge slam_navigation.launch.py
```

#### 방법 2: 직접 실행

```bash
# 기존 turtlebot3_node의 sonar 데이터를 Range 메시지로 변환 (권장)
ros2 run ultrasonic_sensor_bridge sonar_to_range_converter

# ROS1 브리지 모드
ros2 run ultrasonic_sensor_bridge ultrasonic_ros1_bridge

# 시뮬레이션 모드
ros2 run ultrasonic_sensor_bridge ultrasonic_ros1_bridge --simulation

# 기본 시뮬레이션 브리지
ros2 run ultrasonic_sensor_bridge ultrasonic_sensor_bridge
```

### 3. 토픽 확인

```bash
# ROS2 토픽 리스트 확인
ros2 topic list | grep ultrasonic

# 토픽 데이터 확인
ros2 topic echo /ultrasonic/left
ros2 topic echo /ultrasonic/front
ros2 topic echo /ultrasonic/right

# 토픽 정보 확인
ros2 topic info /ultrasonic/left
```

## 설정

### 센서 파라미터

기본 센서 설정:
- 최소 거리: 0.02m (2cm)
- 최대 거리: 4.0m
- 시야각: 0.1rad (약 5.7도)
- 발행 주파수: 10Hz

### 시뮬레이션 데이터

시뮬레이션 모드에서 사용되는 기본 거리값:
- 왼쪽 센서: 1.5m
- 앞쪽 센서: 2.0m
- 오른쪽 센서: 1.8m

## 구현 방식

### 1. Sonar to Range Converter (권장)
기존 `turtlebot3_node`에서 발행하는 `/sensor_state` 토픽의 `sonar` 데이터를 받아서 3개의 개별 `sensor_msgs/Range` 메시지로 변환합니다. 이 방법은 기존 코드를 수정하지 않고도 요구사항을 만족할 수 있습니다.

### 2. ROS1 브리지
OpenCR에서 직접 3개의 개별 초음파 센서 데이터를 ROS1 `sensor_msgs/Range` 형식으로 발행하고, 이를 ROS2로 변환하는 브리지입니다.

## OpenCR 설정

### Sonar to Range Converter 사용 시
기존 turtlebot3_node가 자동으로 sonar 데이터를 처리하므로 추가 설정이 필요하지 않습니다.

### ROS1 브리지 사용 시
OpenCR에서 다음과 같이 초음파 센서 데이터를 발행해야 합니다:

```cpp
sensor_msgs::Range msg_ul_left, msg_ul_front, msg_ul_right;
ros::Publisher ul_pub_left ("/ultrasonic/left",  &msg_ul_left);
ros::Publisher ul_pub_front("/ultrasonic/front", &msg_ul_front);
ros::Publisher ul_pub_right("/ultrasonic/right", &msg_ul_right);
```

## 문제 해결

### ROS1이 설치되지 않은 경우
- 자동으로 시뮬레이션 모드로 전환됩니다
- 경고 메시지가 출력되지만 정상적으로 동작합니다

### 토픽이 수신되지 않는 경우
1. ROS1 마스터가 실행 중인지 확인
2. OpenCR에서 올바른 토픽명으로 발행하고 있는지 확인
3. 네트워크 연결 상태 확인

### 빌드 오류가 발생하는 경우
```bash
# 의존성 설치
sudo apt update
sudo apt install ros-humble-sensor-msgs ros-humble-turtlebot3-msgs

# 워크스페이스 재빌드
colcon build --packages-select ultrasonic_sensor_bridge --cmake-clean-cache
```

## 라이선스

Apache License 2.0
