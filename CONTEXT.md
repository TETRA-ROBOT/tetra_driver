# tetra_driver - AI Agent Context

## 개요
로봇의 저수준(Low-level) 하드웨어와 ROS2 사이를 연결하는 핵심 드라이버 패키지 그룹입니다. 
시리얼 통신(RS232)을 통해 모터 컨트롤러, 전원 관리 보드, 서보 모터를 직접 제어합니다.

## 패키지 맵

| 서브 패키지 | 주요 소스 | 역할 | 주요 통신 (Topic/Service) |
|---|---|---|---|
| `tetra_wheel_driver` | `tetra_wheel_driver.cpp`, `drive_module.c` | 좌/우 바퀴 모터 제어, Odometry 계산, TF 브로드캐스팅 | Sub: `cmd_vel_nav2`(Nav2 최종), `cmd_vel_manual`(조그) — `cmd_vel_nav` 는 기본 미구독<br>Pub: `/odom`, `/tf` |
| `tetra_power_driver` | `tetra_power_driver.cpp`, `power_module.c` | 배터리, 충전 전류, 전압, Sonar 초음파, LED 제어, 비상정지(EMG), 범퍼 | Pub: `/tetra_battery`, `/tetra_power`, `/gpio_status`<br>Srv: `power_outport_cmd`, `led_cmd`, `conveyor_auto_move_cmd`(CN, 리프트), `~/raw_command` |
| `tetra_serial` | `rs232.c`, `rs232_common.c`, `rs232_raw.c` | 두 드라이버가 같이 쓰는 RS232 포트 설정·패킷 읽기/쓰기 (정적 라이브러리), raw 명령 주고받기 | - |

## 아키텍처 다이어그램

```
  [ cmd_vel_nav2 / cmd_vel_manual ] ──> [ tetra_wheel_driver ] ──> RS232 ──> [ 모터 컨트롤러 (휠) ]
                          │
                          └──> [ /odom ], [ /tf ]
                          
  [ tetra_power_driver ] ──> RS232 ──> [ 전원 보드 (Power Board) ]
         │
         ├──> [ /tetra_battery ], [ /tetra_power ]
         ├──> [ /gpio_status ]
         └──> [ Sonar PointCloud2 ]
```

## AI 에이전트 주의사항
1. **시리얼 포트**: 코드 기본값은 `/dev/ttyS0` (휠), `/dev/ttyS1` (전원)입니다 (`port` 파라미터). 포트는 열 때 잠가(TIOCEXCL) root 가 아닌 다른 프로세스가 같은 포트를 열지 못하게 합니다.
2. **단위 표준**:
   - 선속도: `m/s`, 각속도: `rad/s`
   - 배터리: 퍼센트(`%`, 0~100)
   - 전압: `V`, 전류: `A`
3. **`tetra_wheel_driver/src/legacy_archive/`, `tetra_power_driver/src/legacy_archive/`는 빌드(CMakeLists)에서 쓰지 않습니다.** 예전 모델별 코드 참고용이니 수정 대상이 아닙니다.
4. **시리얼 코드는 `tetra_serial` 에만 있습니다.** 응답 읽기 함수는 모두 버퍼 크기를 받습니다 (`get_response(fd, buf, sizeof(buf))`). 보내기는 `send_packet` 으로 하세요 (이전 응답을 비우고 끝까지 보냄).
5. **실패를 성공으로 보고하지 않습니다.** 서비스의 `command_result` 는 보드 응답 결과이고, 읽기에 실패하면 값을 해석하지 않습니다. 포트 fd 는 -1 이 "없음"입니다 (0 이 아님).
6. **안전 동작**(모터 에러 재시도 제한, 통신 끊김 시 정지·재연결, 종료 시 0 속도 → 서보 OFF)은 각 패키지 README 의 "안전 동작"/"통신 오류 처리"를 보세요. 바꾸면 아래 가상 보드 테스트로 확인합니다.
7. **하드웨어 없이 확인**: `tools/board_sim/` 의 가상 구동·전원 보드에 실제 노드를 붙여 시나리오를 돌립니다 (`tools/board_sim/README.md`).
8. **HerkuleX 서보(Pan/Tilt)는 이 레포에 없습니다.** `tetra_dependency/herkulex_driver` 를 씁니다 (`herkulex/set_angle` 등). 예전 `herkulex` 패키지는 쓰는 곳이 없어 지웠습니다.
9. **구형 M 구동보드**는 velocity2('BV') 응답이 16바이트(속도 4바이트 없음)다. `drive_module.c` 가 짧은 프레임이 연속으로 오면 스스로 16바이트로 읽는다
   (20바이트를 기다리면 매 주기 0.2초 타임아웃이 걸려 제어 루프가 5Hz 로 떨어지고 조그에서 손을 떼도 한참 더 간다).
   이 보드는 비상정지 동안 주행 명령을 FLAG_FAIL(02 32 03)로 거부하므로, 비상정지 중의 FLAG_FAIL 은 `/drive_board_incompatible` 판정에 세지 않는다.
10. **`emg_releases_servo`** (기본 false, `tetra_bringup_m.launch.py` 는 구형 M 일 때 true): 비상정지 버튼을 누르면 서보를 풀고, 풀리면 에러를 지우고 다시 켠다.
    뒤쪽 서보 버튼이 없는 구형 M 용이다. **`publish_odom_covariance`** (기본 false, M 런치는 true): `/odom` 에 공분산을 싣는다. `ekf_m.yaml` 이 이 값에 맞춰져 있다
    (주행 중 회전 분산 1e-6 으로 바퀴가 방향을 잡고 IMU 는 바퀴가 끊길 때 대신한다. 근거는 `FillOdometryCovariance` 주석).
    구형 보드와 비상정지 동작은 `tools/board_sim/test_wheel_driver_legacy_m.py` 로 확인한다.
11. **`~/raw_command`** (`/tetra_wheel_driver/raw_command`, `/tetra_power_driver/raw_command`, `interfaces/srv/BoardRawCommand`): 관리자 "보드 명령 테스트"가 보드에 임의 바이트를 보낸다.
    폴링 루프와 같은 스레드(spin_some)에서 처리해야 패킷이 섞이지 않는다 — 드라이버를 멀티스레드 실행기로 바꾸면 시리얼 접근에 잠금을 따로 둬야 한다.
    응답을 기다리는 동안 폴링이 멈추므로 한도(256 바이트, 2초)를 늘리지 않는다. 자세한 내용은 `README.md` "보드 명령 테스트".
12. **이름 규칙**: ROS 서비스·토픽·파라미터 이름은 소문자 스네이크 케이스다. 전원 드라이버는 예전 대문자 서비스 이름(`Power_*_cmd`, `Conveyor_*_cmd`)도
    `legacy_services_` 로 같이 열고, 예전 파라미터 이름(`m_bConveyor_option`, `m_bUltrasonic_option`, `m_bEKF_option`)도 경고와 함께 읽는다
    (`tetra_serial/renamed_parameter.hpp`). 새 코드와 런치 파일은 새 이름을 쓴다. `m_bSingle_TF_option` 은 외부 패키지 iahrs_driver 것이라 그대로다.
13. **리프트와 컨베이어 명령**: 보드 CN(`conveyor_auto_move_cmd`)이 리프트, CC(`conveyor_manual_move_cmd`)가 컨베이어, LT(`turn_on_cmd`)는 LED 채널이다.
    서비스 이름의 "conveyor" 는 예전 이름을 따른 것이다.
14. **바퀴 오도메트리 보정 배율**: 오도메트리(x, y, θ)는 구동보드가 계산한다. 드라이버는 매 주기 보드 이동량에 `odom_linear_scale`·`odom_angular_scale`
    (0.8~1.2, 기본 1)을 곱해 다시 적분해 `/odom` 으로 낸다 (`tetra_wheel_driver/src/odom_scale.hpp`, 테스트 `test/test_odom_scale.cpp`).
    배율 1 이면 보드 값을 그대로 낸다. 파라미터를 바꾸면 바로 쓰고 `~/.config/tetra/wheel_odom_scale.txt`(`odom_scale_file`)에 남긴다.
    값은 관리자 화면 "바퀴 오도메트리 보정"(tetra_util/tetra_odom_calibration)이 잰다. 보드 파라미터(바퀴 지름·간격으로 보이는 P4·P5)는 뜻을 적은 문서가 없어 건드리지 않는다.
    `wheel_radius`·`wheel_distance` 파라미터는 속도 명령 변환에만 쓰이고 오도메트리에는 영향이 없다.
