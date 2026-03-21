# Antenna Tracker ROS2 - 에이전트 인수인계 문서

> **작성일**: 2026-03-21
> **프로젝트**: `antenna_tracker_ros` (ROS2 Humble + micro-ROS)
> **원본 프로젝트**: `../antenna_tracker_imu_encoder` (Zephyr RTOS 기반)
> **담당자**: Hyeonsu Park (MINI-UNIQ-STAR)

---

## 1. 프로젝트 배경

기존 `antenna_tracker_imu_encoder` 프로젝트(STM32H7 + Zephyr RTOS + Raspberry Pi 4B FastAPI)를 **ROS2 Humble + micro-ROS 풀 통합** 방식으로 재설계한 프로젝트.

### 핵심 설계 결정

| 항목 | 결정 |
|------|------|
| micro-ROS transport | USB CDC (`/dev/ttyACM0`) |
| RPi4B OS | Ubuntu 22.04 + PREEMPT_RT 커널 |
| 100Hz 제어 루프 위치 | RPi4B ROS2 (`controller_node`) 로 이전 |
| 인코더 방식 | Hall Sensor Quadrature (TIM1=Azimuth, TIM8=Elevation) |
| ESP32 LoRa 연결 | CAN 2.0B 유지 (500kbps, 기존 프로토콜 재사용) |
| Gazebo 버전 | Fortress (ROS2 Humble 공식 지원) |

### 전체 아키텍처

```
[STM32H7 Nucleo] ──USB CDC──────────────── [RPi4B Ubuntu 22.04 + PREEMPT_RT]
  Zephyr + micro-ROS Humble                  ROS2 Humble
  ├── BMI270 → /imu/raw (100Hz)               ├── /micro_ros_agent
  ├── MLX90393 → /magnetic_field (100Hz)      ├── sensor_fusion_node  (100Hz)
  ├── NEO-6M → /gps/fix (1Hz)                 ├── navigation_node
  ├── TIM1 Hall → /antenna/encoder_feedback   ├── controller_node     (100Hz PID)
  ├── TIM8 Hall → (위 동일 메시지)             ├── state_machine_node
  └── /antenna/motor_cmd → TB6600            └── can_bridge_node
                                               │
[ESP32 LoRa] ──CAN 2.0B─────────────────→ /antenna/target_gps
                                               │
[Gazebo Fortress] ──ros_gz_bridge──────── sim 노드들
```

---

## 2. 완료된 작업 (Phase 1)

### 생성된 파일 목록 (67개)

```
antenna_tracker_ros/
├── README.md
├── HANDOFF.md                          ← 이 파일
├── scripts/
│   ├── install_preempt_rt.sh           ← RPi4B PREEMPT_RT 커널 설치
│   ├── setup_ros2_humble.sh            ← ROS2 Humble + CycloneDDS 설치
│   └── setup_micro_ros.sh             ← micro-ROS agent 워크스페이스 설정
└── src/
    ├── antenna_tracker_msgs/           ← 커스텀 메시지/서비스/액션
    │   ├── msg/ImuFusion.msg
    │   ├── msg/EncoderFeedback.msg
    │   ├── msg/AntennaState.msg
    │   ├── msg/MotorCommand.msg
    │   ├── msg/TargetGPS.msg
    │   ├── msg/TrackerDiagnostics.msg
    │   ├── srv/SetMode.srv
    │   ├── srv/SetManualTarget.srv
    │   ├── srv/GetStatus.srv
    │   └── action/TrackTarget.action
    ├── antenna_tracker_hardware/
    │   ├── micro_ros_firmware/         ← STM32H7 Zephyr 펌웨어
    │   │   ├── src/main.c              ← micro-ROS 초기화, 100Hz 루프
    │   │   ├── src/bmi270_driver.c/h   ← IMU I2C 드라이버
    │   │   ├── src/mlx90393_driver.c/h ← 자력계 I2C 드라이버
    │   │   ├── src/gps_parser.c/h      ← NMEA GGA/RMC 파싱
    │   │   ├── src/hall_encoder.c/h    ← TIM1/TIM8 Hall 모드 카운터
    │   │   ├── src/motor_driver.c/h    ← TB6600 PWM+DIR 제어
    │   │   ├── prj.conf                ← Zephyr 커널 설정
    │   │   ├── west.yml                ← micro-ROS Humble 의존성
    │   │   ├── .clangd                 ← LSP false positive 억제
    │   │   └── boards/nucleo_h7a3zi_q.overlay
    │   └── src/can_bridge_node.cpp     ← SocketCAN → /antenna/target_gps
    ├── antenna_tracker_controller/
    │   ├── src/complementary_filter.cpp ← 원본 sensor_fusion.c에서 포팅 (alpha=0.98)
    │   ├── src/kalman_filter.cpp        ← 4-state Kalman (Q=0.001, R=2.0)
    │   ├── src/pid_controller.cpp       ← Cascade PID (config.h 수치 포팅)
    │   ├── src/sensor_fusion_node.cpp   ← 100Hz, /imu/raw + /magnetic_field → /antenna/imu_fusion
    │   ├── src/navigation_node.cpp      ← Haversine 방위각/고각 계산
    │   ├── src/controller_node.cpp      ← 100Hz PID 루프, TrackTarget 액션 서버
    │   └── src/state_machine_node.cpp   ← AUTO/MANUAL/STANDBY/EMERGENCY
    ├── antenna_tracker_simulation/
    │   ├── urdf/antenna_tracker.urdf.xacro  ← Pan-Tilt 안테나 모델
    │   ├── worlds/antenna_tracker.world     ← 서울 좌표 기반 Gazebo 세계
    │   ├── launch/sim.launch.py
    │   └── config/rviz_config.rviz
    ├── antenna_tracker_bringup/
    │   ├── launch/hardware.launch.py    ← 하드웨어 전체 런치
    │   ├── launch/sim.launch.py
    │   ├── config/hardware_params.yaml  ← config.h 수치 전부 포함
    │   └── config/sim_params.yaml
    └── antenna_tracker_web/
        ├── web/index.html              ← WebSocket 기반 GCS 대시보드
        └── web/app.js                  ← rosbridge 클라이언트
```

### 포팅된 알고리즘 수치 (원본 config.h 기준)

```yaml
# Cascade PID Gains (hardware_params.yaml)
azimuth:
  position_kp: 15.0  ki: 0.1  kd: 0.3
  velocity_kp: 4.0   ki: 0.05 kd: 0.1
elevation:
  position_kp: 8.0   ki: 1.0  kd: 0.2
  velocity_kp: 5.0   ki: 0.15 kd: 0.08

# Kalman Filter
process_noise_q: 0.001
measurement_noise_r: 2.0

# Complementary Filter
alpha: 0.98   # 98% gyro + 2% accel

# System
gear_ratio: 5.0
magnetic_declination_deg: -8.0  # 서울
motor_max_freq_hz: 1000.0
angle_limits: az=[0,360], el=[0,90]
```

### ROS2 토픽/서비스/액션 구조

```
Publishers (firmware → RPi4B):
  /imu/raw               sensor_msgs/Imu           100Hz
  /magnetic_field        sensor_msgs/MagneticField  100Hz
  /gps/fix               sensor_msgs/NavSatFix      1Hz
  /antenna/encoder_feedback  EncoderFeedback        100Hz

Publishers (RPi4B 내부):
  /antenna/imu_fusion    ImuFusion                  100Hz
  /antenna/motor_cmd     MotorCommand               100Hz
  /antenna/state         AntennaState               10Hz
  /antenna/target_gps    TargetGPS                  (CAN 수신 시)
  /antenna/diagnostics   TrackerDiagnostics         1Hz

Services:
  /antenna/set_mode          SetMode
  /antenna/set_manual_target SetManualTarget
  /antenna/get_status        GetStatus

Actions:
  /antenna/track_target      TrackTarget
```

---

## 3. 앞으로 해야 할 작업

### Phase 2 — 알고리즘 검증 + 단위 테스트 (우선순위 높음)

- [ ] `antenna_tracker_controller/test/` 디렉토리 생성
- [ ] `test_complementary_filter.cpp`: 원본 C 코드와 수치 동일성 검증
- [ ] `test_kalman_filter.cpp`: 수렴 테스트, 노이즈 주입 테스트
- [ ] `test_pid_controller.cpp`: 계단 응답, 적분 와인드업 방지 테스트
- [ ] `test_navigation.cpp`: Haversine 거리/방위각 기준값 테스트
- [ ] CMakeLists.txt에 gtest 추가
- [ ] `colcon test` 통과 확인

**검증 포인트**: 원본 `sensor_fusion.c`와 포팅된 C++ 코드의 동일 입력 대비 출력 오차 < 1e-6

### Phase 3 — micro-ROS 펌웨어 빌드 검증

- [ ] Zephyr SDK 설치 확인 (SDK 0.16.x for Humble)
- [ ] micro-ROS Humble 브랜치 west 초기화
- [ ] `west build -b nucleo_h7a3zi_q` 통과
- [ ] Hall Sensor overlay 핀 매핑 검증
  - TIM1_CH1=PA8, TIM1_CH2=PA9 (또는 실제 회로도 확인 필요)
  - TIM8_CH1=PC6, TIM8_CH2=PC7 (또는 실제 회로도 확인 필요)
- [ ] USB CDC transport 연결 테스트 (`/dev/ttyACM0`)
- [ ] micro-ROS agent ↔ 펌웨어 통신 smoke test

> ⚠️ **주의**: `boards/nucleo_h7a3zi_q.overlay`의 Hall 센서 핀 번호를 실제 회로도와 반드시 대조할 것.
> 현재는 STM32H7A3ZI-Q의 일반적인 타이머 핀을 사용했으나 변경될 수 있음.

### Phase 4 — RPi4B 환경 구축

- [ ] `scripts/install_preempt_rt.sh` 실행 (빌드 ~2시간 소요)
  - CPU 격리: `isolcpus=2,3` (controller_node 전용)
  - `uname -r`에 `-rt` 접미사 확인
- [ ] `scripts/setup_ros2_humble.sh` 실행
- [ ] `scripts/setup_micro_ros.sh` 실행
- [ ] CAN 인터페이스 설정: `ip link set can0 up type can bitrate 500000`
- [ ] `colcon build --packages-select antenna_tracker_msgs` 먼저 빌드
- [ ] 전체 `colcon build` 확인

### Phase 5 — 하드웨어 통합 테스트

- [ ] STM32H7 ↔ RPi4B USB CDC 연결
- [ ] `ros2 topic hz /imu/raw` → 100Hz 확인
- [ ] `ros2 topic hz /antenna/encoder_feedback` → 100Hz 확인
- [ ] GPS fix 수신 확인 (`/gps/fix`)
- [ ] CAN 브릿지: ESP32 LoRa target GPS 수신 확인
- [ ] 100Hz 제어 루프 지터 측정 (목표: < 2ms with PREEMPT_RT)
- [ ] PID 튜닝 (현장 테스트로 gain 조정)
- [ ] 비상 정지 테스트 (EMERGENCY 모드)

### Phase 6 — Gazebo 시뮬레이션 완성

- [ ] Gazebo Fortress 설치 확인 (`gz sim --version`)
- [ ] URDF → SDF 변환 확인 (`ros2 run xacro xacro`)
- [ ] Pan-Tilt 조인트 물리 파라미터 조정 (관성 텐서, 마찰)
- [ ] IMU 플러그인 → `/imu/raw` 토픽 연결 확인
- [ ] GPS 플러그인 → `/gps/fix` 토픽 연결 확인
- [ ] `sim_params.yaml` PID gain 튜닝 (Gazebo 마찰 모델에 맞게)
- [ ] `ros2 launch antenna_tracker_simulation sim.launch.py` 통과
- [ ] RViz2에서 안테나 방향 시각화 확인

### Phase 7 — 웹 대시보드 완성

- [ ] `rosbridge_suite` 설치: `sudo apt install ros-humble-rosbridge-suite`
- [ ] `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` 실행
- [ ] `index.html`에서 실시간 방위각/고각 표시 확인
- [ ] 모드 전환 버튼 동작 확인 (SetMode 서비스 호출)
- [ ] Target GPS 수신 시 지도 표시 (선택 사항)

### Phase 8 — CI/CD + 배포

- [ ] `.github/workflows/ros2_build.yml`: colcon build + test 자동화
- [ ] `docker/Dockerfile`: ROS2 Humble 기반 컨테이너 (선택 사항)
- [ ] `ansible/` 또는 deploy 스크립트: RPi4B 원클릭 배포

---

## 4. 미해결 이슈 및 주의사항

### 반드시 확인해야 할 항목

1. **Hall 센서 핀 매핑** — `nucleo_h7a3zi_q.overlay`에 설정된 TIM1/TIM8 핀이 실제 회로와 일치하는지 확인 필요. 원본 프로젝트 회로도(`antenna_tracker_imu_encoder/boards/`) 참조.

2. **Hall 센서 폴 수** — `hall_encoder.c`의 `PULSES_PER_REV` (현재 12)와 기어비(5.0)가 실제 모터 스펙과 일치하는지 확인 필요.

3. **micro-ROS Humble 브랜치** — `west.yml`에서 `humble` 브랜치를 사용하도록 설정되어 있음. Zephyr SDK 버전과의 호환성 확인 필요.

4. **100Hz 실시간성** — PREEMPT_RT 커널 없이는 jitter가 5~20ms까지 발생할 수 있음. `install_preempt_rt.sh` 실행 필수.

5. **CAN 인터페이스명** — `can_bridge_node.cpp`에서 `can0`를 사용. 실제 RPi4B의 CAN 인터페이스명 확인 필요 (`ip link` 또는 `ifconfig`).

### 알려진 미구현 사항

- LMPC 백엔드: 현재 Cascade PID만 구현. 원본의 LMPC(OSQP 기반) 이전은 Phase 2 이후로 미룸
- `antenna_tracker_web` 패키지의 WebSocket 브릿지 노드 미구현 (rosbridge_suite 활용으로 대체)
- Diagnostics 메시지의 CPU 온도 측정 미구현 (RPi4B `/sys/class/thermal/thermal_zone0/temp` 읽기 필요)

---

## 5. 빠른 시작 가이드

```bash
# 1. RPi4B 환경 설정
cd antenna_tracker_ros/scripts
bash setup_ros2_humble.sh
bash install_preempt_rt.sh   # 재부팅 필요
bash setup_micro_ros.sh

# 2. ROS2 워크스페이스 빌드
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. 하드웨어 실행
ros2 launch antenna_tracker_bringup hardware.launch.py

# 4. 시뮬레이션 실행
ros2 launch antenna_tracker_bringup sim.launch.py

# 5. 상태 확인
ros2 topic hz /imu/raw
ros2 topic echo /antenna/state
ros2 service call /antenna/set_mode antenna_tracker_msgs/srv/SetMode "{mode: 0}"
```

---

## 6. 참고 파일 경로

| 목적 | 경로 |
|------|------|
| 원본 PID 수치 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/control.h` |
| 원본 Kalman 코드 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/sensor_fusion.c` |
| 원본 CAN 프로토콜 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/protocol.h` |
| 원본 config 수치 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/config.h` |
| 원본 상태 머신 | `../antenna_tracker_imu_encoder/firmware/zephyr_app/src/state_machine.h` |
| 원본 GCS Python | `../antenna_tracker_imu_encoder/Raspberry_pi_4b/main.py` |
| 플래너 상세 계획 | `.omc/plans/antenna-tracker-ros2-migration.md` |
