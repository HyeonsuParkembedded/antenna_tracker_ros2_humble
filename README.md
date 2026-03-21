# Antenna Tracker ROS2 Humble

ROS2 Humble 기반 안테나 추적 시스템. STM32H7A3ZI-Q Nucleo (micro-ROS) + RPi4B (Ubuntu 22.04 + PREEMPT_RT) 구성.

## 시스템 구성

```
ESP32 LoRa ──CAN──> STM32H7A3ZI-Q ──USB CDC──> RPi4B (ROS2 Humble)
(Target GPS)        (micro-ROS)                 (Controller + GCS)
```

### 하드웨어
| 구성요소 | 모델 | 인터페이스 |
|---------|------|-----------|
| MCU | STM32H7A3ZI-Q Nucleo | micro-ROS (USB CDC) |
| IMU | BMI270 | I2C1 (0x68) |
| Magnetometer | MLX90393 | I2C1 (0x0C) |
| GPS | NEO-6M | UART3 (9600 baud) |
| Encoder | Hall Sensor Quadrature x2 | TIM1 (Az), TIM8 (El) |
| Motor Driver | TB6600 x2 | PWM + GPIO |
| CAN Bridge | ESP32 LoRa | FDCAN1 (500kbps) |

### PID 파라미터 (MATLAB 튜닝 완료)
| 축 | Loop | Kp | Ki | Kd |
|----|------|-----|-----|-----|
| Azimuth | Position | 15.0 | 0.1 | 0.3 |
| Azimuth | Velocity | 4.0 | 0.05 | 0.1 |
| Elevation | Position | 8.0 | 1.0 | 0.2 |
| Elevation | Velocity | 5.0 | 0.15 | 0.08 |

## 패키지 구조

```
src/
├── antenna_tracker_msgs/      # 커스텀 메시지/서비스/액션
├── antenna_tracker_hardware/  # micro-ROS 펌웨어 + CAN 브릿지
├── antenna_tracker_controller/ # 센서 퓨전, 네비게이션, PID, 상태머신
├── antenna_tracker_simulation/ # Gazebo URDF 시뮬레이션
├── antenna_tracker_bringup/   # 런치 파일 + 파라미터
└── antenna_tracker_web/       # 웹 대시보드 (rosbridge)
```

## 토픽 구조

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `/imu/raw` | sensor_msgs/Imu | 100Hz | BMI270 raw data |
| `/magnetic_field` | sensor_msgs/MagneticField | 100Hz | MLX90393 |
| `/gps/fix` | sensor_msgs/NavSatFix | 1Hz | NEO-6M |
| `/antenna/encoder_feedback` | EncoderFeedback | 100Hz | Hall encoder |
| `/antenna/imu_fusion` | ImuFusion | 100Hz | Complementary + Kalman |
| `/antenna/target_gps` | TargetGPS | ~1Hz | CAN -> ESP32 LoRa |
| `/antenna/target_azimuth` | Float64 | ~1Hz | Haversine bearing |
| `/antenna/target_elevation` | Float64 | ~1Hz | Elevation angle |
| `/antenna/motor_cmd` | MotorCommand | 100Hz | TB6600 command |
| `/antenna/state` | AntennaState | 100Hz | Tracking state |
| `/antenna/diagnostics` | TrackerDiagnostics | 1Hz | System health |

## 빌드

```bash
# 1. ROS2 Humble 설치
./scripts/setup_ros2_humble.sh

# 2. 워크스페이스 빌드
cd antenna_tracker_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 하드웨어 (RPi4B)
```bash
# micro-ROS 펌웨어 플래시 (별도 Zephyr 빌드 필요)
# CAN 인터페이스 설정
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 런치
ros2 launch antenna_tracker_bringup hardware.launch.py
```

### 시뮬레이션
```bash
ros2 launch antenna_tracker_bringup sim.launch.py
```

### 웹 대시보드
```bash
# rosbridge 시작
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 웹 파일 서빙 (별도 터미널)
cd install/antenna_tracker_web/share/antenna_tracker_web/web
python3 -m http.server 8080
# 브라우저: http://localhost:8080
```

### 서비스 호출
```bash
# 모드 변경
ros2 service call /antenna/set_mode antenna_tracker_msgs/srv/SetMode "{mode: 0}"

# 수동 타겟 설정
ros2 service call /antenna/set_manual_target antenna_tracker_msgs/srv/SetManualTarget \
  "{azimuth_deg: 180.0, elevation_deg: 45.0}"

# 상태 조회
ros2 service call /antenna/get_status antenna_tracker_msgs/srv/GetStatus
```

### 액션 호출
```bash
ros2 action send_goal /antenna/track_target antenna_tracker_msgs/action/TrackTarget \
  "{target_azimuth_deg: 180.0, target_elevation_deg: 45.0, tolerance_deg: 1.0, timeout_sec: 30.0}"
```

## micro-ROS 펌웨어 빌드 (Zephyr)

```bash
# west 설치
pip3 install west

# 펌웨어 디렉토리로 이동
cd src/antenna_tracker_hardware/micro_ros_firmware

# Zephyr 워크스페이스 초기화
west init -l .
west update

# 빌드
west build -b nucleo_h7a3zi_q

# 플래시
west flash
```

## PREEMPT_RT 설정 (실시간 제어)

```bash
./scripts/install_preempt_rt.sh
# 재부팅 후 RT 스케줄러로 controller_node 실행:
sudo chrt -f 80 taskset -c 2,3 ros2 run antenna_tracker_controller controller_node
```
