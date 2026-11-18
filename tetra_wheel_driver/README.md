# 🤖 Tetra Wheel Driver (`tetra_wheel_driver`)

## 📖 개요
`tetra_wheel_driver` 패키지는 TETRA 로봇의 차륜 모터 구동 및 주행 제어 기능을 제공합니다. 이 패키지는 로봇의 이동 명령 수신, 오도메트리 연산 및 모터 상태/센서 데이터를 처리합니다.

## 의존성
- ament_cmake
- rclcpp
- std_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2
- tf2_ros
- interfaces
- tetra_serial (같은 레포의 RS232 공용 패키지)

## 설치 및 빌드 방법
1. 워크스페이스 루트에서 다음 명령어를 실행합니다:
   ```bash
   colcon build --symlink-install --packages-select tetra_wheel_driver
   ```

## 주요 파일 설명
RS232 포트 설정과 패킷 읽기·쓰기는 `tetra_serial` 패키지(`tetra_serial/rs232*.h`)를 씁니다.

### src/
- **`drive_module.c`**: 로봇의 구동 모듈 구현.
- **`dssp_rs232_drive_module.c`**: RS232 기반 구동 모듈 구현.
- **`tetra_wheel_driver.cpp`**: ROS2 노드의 메인 파일로, 로봇의 주행 동작을 제어합니다. 주요 기능:
  - `/cmd_vel_nav2`(Nav2 최종 출력), `/cmd_vel_manual`(웹 조그) 토픽을 구독하여 로봇의 속도를 제어.
  - `/odom` 토픽을 퍼블리시하여 로봇의 위치와 자세 정보를 제공합니다.
  - 다양한 서비스(`ParameterRead`, `ParameterWrite`, `SetMoveMode` 등)를 통해 로봇의 설정 및 동작을 제어.

### launch/
- **`tetra_wheel_driver_launch.xml`**: XML 형식의 Launch 파일.
- **`tetra_wheel_driver.launch.py`**: Python 형식의 Launch 파일로, ROS2 노드를 실행합니다.

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/ttyS0` | 구동 보드 시리얼 포트 |
| `robot_model` | `m` | 바퀴 반지름·간격을 고른다 (`m`, `h`, `edu`, `mh`, `s`, `sm`). **bringup 에서 꼭 넘겨야 합니다** |
| `wheel_radius`, `wheel_distance`, `tread_width` | 0 (모델 값 사용) | 0 보다 크면 모델 값 대신 사용 |
| `ekf_option` | `false` | (예전 이름 `m_bEKF_option` 도 읽지만 경고) `true` 면 odom→base_footprint TF 를 내보내지 않음 (EKF 가 담당) |
| `cmd_vel_timeout` | 0.5 | 이 시간(초) 동안 속도 명령이 없으면 정지. 0 이하면 끔 |
| `max_linear_velocity` | 2.0 | 선속도 상한 (m/s) |
| `max_angular_velocity` | 2.0 | 각속도 상한 (rad/s) |

## 안전 동작
- **모터 드라이버 에러**: 비상정지('1')·저전압('5')은 1초마다 리셋을 시도합니다. 그 밖의 에러(과부하, 엔코더 등)는 1초 간격으로 3번까지만
  리셋·서보 ON 을 시도하고, 그래도 남으면 서보를 끈 채 멈춥니다. `/servo_on` 1 또는 `servo_cmd` 1 로 다시 켜면 해제됩니다.
- **통신 끊김**: 구동 보드를 연속 15번(약 0.5초) 읽지 못하면 `/odom` 발행을 멈추고 2초마다 포트를 다시 엽니다.
- **전원 보드 끊김**: `tetra_power_driver` 가 `/power_status` 에 -10 을 보내면 서보를 끕니다.
- **종료**: 0 속도를 보낸 뒤 서보를 끕니다 (예외로 끝나도 같음).
- `/odom` 의 twist 는 `base_footprint` 기준입니다 (EKF 가 vx·vy 를 로봇 기준으로 받음).

## 하드웨어 없이 확인
`tools/board_sim/` 의 가상 구동 보드로 노드를 실제로 띄워 볼 수 있습니다 (레포 루트 `tools/board_sim/README.md`).

## 주요 토픽 및 서비스
### 퍼블리시 토픽
- **`/odom`** (타입: `nav_msgs/msg/Odometry`): 로봇의 위치와 자세 정보를 제공합니다.
- **`/bumper_data`** (타입: `std_msgs/msg/Int32`): 범퍼 상태를 퍼블리시합니다.
- **`/emg_state`** (타입: `std_msgs/msg/Int32`): 비상 정지 상태를 퍼블리시합니다.
- **`/left_error_code`** (타입: `std_msgs/msg/Int32`): 왼쪽 모터의 오류 코드를 퍼블리시합니다.
- **`/right_error_code`** (타입: `std_msgs/msg/Int32`): 오른쪽 모터의 오류 코드를 퍼블리시합니다.

### 구독 토픽
- **`/cmd_vel_nav2`** (타입: `geometry_msgs/msg/Twist`): Nav2 최종 명령. `cmd_vel_nav → velocity_smoother → collision_monitor` 를 거친 값이고, 도킹(docking_server)도 이 토픽으로 보냅니다.
- **`/cmd_vel_nav`** (타입: `geometry_msgs/msg/Twist`): 기본은 **구독하지 않습니다** (`use_raw_nav_cmd:=false`).
  평활화·충돌 감시 전의 컨트롤러 원본이라, 받으면 collision_monitor 가 0(정지)을 보낸 직후 원본 명령으로 다시 움직입니다.
  예전 동작(cmd_vel_nav2 가 0 이면 cmd_vel_nav 사용)으로 되돌릴 때만 `use_raw_nav_cmd:=true` 로 켭니다.
- **`/cmd_vel_manual`** (타입: `geometry_msgs/msg/Twist`): 수동 속도 명령을 수신합니다.
- **`/accel_vel`** (타입: `std_msgs/msg/Int32`): 가속도 데이터를 수신합니다.
- **`/pose_reset`** (타입: `std_msgs/msg/Int32`): 로봇의 위치를 초기화합니다.
- **`/servo_on`** (타입: `std_msgs/msg/Int32`): 서보 모터를 활성화합니다.
- **`/power_status`** (타입: `std_msgs/msg/Int32`): 전원 상태를 수신합니다.
- **`/joy`** (타입: `sensor_msgs/msg/Joy`): 조이스틱 입력을 수신합니다.

### 서비스
- **`param_read_cmd`** (타입: `interfaces/srv/ParameterRead`): 로봇의 매개변수를 읽습니다.
- **`param_write_cmd`** (타입: `interfaces/srv/ParameterWrite`): 로봇의 매개변수를 설정합니다.
- **`mode_change_cmd`** (타입: `interfaces/srv/SetMoveMode`): 로봇의 이동 모드를 변경합니다.
- **`linear_move_cmd`** (타입: `interfaces/srv/LinearPositionMove`): 선형 위치 이동 명령을 실행합니다.
- **`angular_move_cmd`** (타입: `interfaces/srv/AngularPositionMove`): 각도 위치 이동 명령을 실행합니다.
- **`servo_cmd`** (타입: `interfaces/srv/Servo`): 서보 모터를 On/Off 제어합니다.
- **`/tetra_wheel_driver/raw_command`** (타입: `interfaces/srv/BoardRawCommand`): 구동 보드에 임의 바이트를 보내고 응답을 그대로 받습니다 (관리자 "보드 명령 테스트"). 바퀴에 속도를 주는 중이면 거부합니다. 루트 `README.md` 의 "보드 명령 테스트" 참고.

## 사용 방법 예시
1. Launch 파일 실행:
   ```bash
   ros2 launch tetra_wheel_driver tetra_wheel_driver.launch.py
   ```
2. `/cmd_vel_manual` 토픽을 퍼블리시하여 로봇의 속도를 제어합니다 (0.5초 이상 끊기면 멈춤):
   ```bash
   ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
   ```
3. `param_read_cmd` 서비스를 호출하여 매개변수를 읽습니다:
   ```bash
   ros2 service call /param_read_cmd interfaces/srv/ParameterRead "{num: 1}"
   ```