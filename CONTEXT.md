# tetra_driver - AI Agent Context

## 개요
로봇의 저수준(Low-level) 하드웨어와 ROS2 사이를 연결하는 핵심 드라이버 패키지 그룹입니다. 
시리얼 통신(RS232)을 통해 모터 컨트롤러, 전원 관리 보드, 서보 모터를 직접 제어합니다.

## 패키지 맵

| 서브 패키지 | 주요 소스 | 역할 | 주요 통신 (Topic/Service) |
|---|---|---|---|
| `tetra_wheel_driver` | `tetra_wheel_driver.cpp`, `drive_module.c` | 좌/우 바퀴 모터 제어, Odometry 계산, TF 브로드캐스팅 | Sub: `/cmd_vel`<br>Pub: `/odom`, `/tf` |
| `tetra_power_driver` | `tetra_power_driver.cpp`, `power_module.c` | 배터리, 충전 전류, 전압, Sonar 초음파, LED 제어, 비상정지(EMG), 범퍼 | Pub: `/tetra_battery`, `/tetra_power`, `/gpio_status`<br>Srv: `Power_outport_cmd`, `led_control` |
| `tetra_serial` | `rs232.c`, `rs232_common.c` | 두 드라이버가 같이 쓰는 RS232 포트 설정·패킷 읽기/쓰기 (정적 라이브러리) | - |

## 아키텍처 다이어그램

```
  [ /cmd_vel ] ──> [ tetra_wheel_driver ] ──> RS232 ──> [ 모터 컨트롤러 (휠) ]
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
